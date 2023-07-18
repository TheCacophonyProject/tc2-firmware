#![no_std]
#![no_main]

mod lepton;
mod utils;
pub mod bsp;

use bsp::{
    entry,
    hal::{
        clocks::{Clock, ClockSource, ClocksManager, StoppableClock},
        multicore::{Multicore, Stack},
        pac,
        rosc::RingOscillator,
        sio::Sio,
        xosc::setup_xosc_blocking,
        Spi,
    },
    pac::Peripherals,
    XOSC_CRYSTAL_FREQ,
};


use byteorder::{BigEndian, ByteOrder, LittleEndian};
use crc::{Crc, CRC_16_XMODEM};
use lepton::Lepton;
use cortex_m::{delay::Delay};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin};
use panic_probe as _;

use fugit::RateExtU32;
use fugit::{HertzU32};
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::digital::InputPin;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write};
use crate::bsp::hal::dma::{DMAExt, single_buffer};
use rp2040_hal::gpio::{FloatingInput, FunctionI2C, Interrupt, PinId};
use rp2040_hal::gpio::bank0::{Gpio1, Gpio4};
use rp2040_hal::I2C;
use rp2040_hal::pio::{PIOBuilder, PIOExt, ShiftDirection};

static mut CORE1_STACK: Stack<4096> = Stack::new();

// CORE1 consts
pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}

pub unsafe fn u8_slice_to_u16(p: &[u8]) -> &[u16] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u16, p.len() / 2)
}

pub unsafe fn u8_slice_to_u32(p: &[u8]) -> &[u32] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u32, p.len() / 4)
}

pub unsafe fn any_as_u32_slice<T: Sized>(p: &T) -> &[u32] {
    core::slice::from_raw_parts((p as *const T) as *const u32, core::mem::size_of::<T>() / 4) //core::mem::size_of::<T>()
}

pub const CORE1_TASK_COMPLETE: u32 = 0xEE;
pub const CORE1_TASK_READY: u32 = 0xDB;
pub const CORE1_TASK_START_WITH_FRAME_SEGMENT: u32 = 0xAC;
pub const CORE1_TASK_START_WITH_FULL_FRAME: u32 = 0xAE;
pub const CORE1_TASK_START_WITHOUT_FRAME_SEGMENT: u32 = 0xAD;

// Each segment has 4 even slices to transfer from the total segment length of  9760 + 4*4bytes for the segment number.
const FRAME_LENGTH: usize = 160 * 122 * 2;
pub const A_SEGMENT_LENGTH: usize = 9768; //9764;
pub const FULL_CHUNK_LENGTH: usize = A_SEGMENT_LENGTH / 4;
pub const CHUNK_LENGTH: usize = (A_SEGMENT_LENGTH - 4) / 4;
pub static mut FRAME_BUFFER: [u8; FRAME_LENGTH + 4] = [0u8; FRAME_LENGTH + 4];
pub const SEGMENT_LENGTH: usize = (160 * 122) / 4;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub static FIRMWARE_VERSION: u32 = 2;

pub type FramePacketData = [u8; 160];
pub struct FrameSeg([FramePacketData; 61]);

impl FrameSeg {
    pub const fn new() -> FrameSeg {
        FrameSeg([[0u8; 160]; 61])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        unsafe { any_as_u8_slice(&self.0) }
    }

    pub fn packet(&mut self, packet_id: usize) -> &mut FramePacketData {
        &mut self.0[packet_id]
    }
}

pub static FRAME_SEGMENT_BUFFER: DoubleBuffer = DoubleBuffer {
    front: Mutex::new(RefCell::new([
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new()
    ])),
    back: Mutex::new(RefCell::new([
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new(),
        FrameSeg::new()
    ])),
    swapper: Mutex::new(RefCell::new(true))
};

pub struct DoubleBuffer {
    front: Mutex<RefCell<[FrameSeg; 4]>>,
    back: Mutex<RefCell<[FrameSeg; 4]>>,
    swapper: Mutex<RefCell<bool>>
}

impl DoubleBuffer {
    pub fn swap(&self) {
        critical_section::with(|cs| {
            let mut val = self.swapper.borrow_ref_mut(cs);
            *val = !*val;
        });
    }

    pub fn get_front(&self) -> &Mutex<RefCell<[FrameSeg; 4]>> {
        let mut val = false;
        critical_section::with(|cs| {
            val = *self.swapper.borrow(cs).borrow();
        });
        if val {
            &self.front
        } else {
            &self.back
        }
    }

    pub fn get_back(&self) -> &Mutex<RefCell<[FrameSeg; 4]>> {
        let mut val = false;
        critical_section::with(|cs| {
            val = *self.swapper.borrow(cs).borrow();
        });
        if val {
            &self.back
        } else {
            &self.front
        }
    }
}

fn go_dormant_until_next_vsync<T:PinId>(rosc: RingOscillator<bsp::hal::rosc::Enabled>, lepton: &mut Lepton<T>, rosc_freq_hz: u32) -> RingOscillator<bsp::hal::rosc::Enabled> {
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    let dormant_rosc = unsafe { rosc.dormant() };
    let disabled_rosc = RingOscillator::new(dormant_rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq_hz);
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    initialized_rosc
}

fn go_dormant_until_woken<T:PinId, T2:PinId>(rosc: RingOscillator<bsp::hal::rosc::Enabled>, wake_pin: &mut bsp::hal::gpio::Pin<T2, FloatingInput>, lepton: &mut Lepton<T>, rosc_freq_hz: u32) -> RingOscillator<bsp::hal::rosc::Enabled> {
    lepton
        .vsync
        .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, false);
    wake_pin.set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true);
    let dormant_rosc = unsafe { rosc.dormant() };
    // Woken by pin
    let disabled_rosc = RingOscillator::new(dormant_rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq_hz);
    wake_pin.set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, false);
    initialized_rosc
}

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);

    let rosc_clock_freq: HertzU32 = 150_000_000.Hz(); //52.76mA, 25Mhz spi clock

    // NOTE: Actual clock speed achieved will depend on multiplier with system clock.  In practice
    //  the lepton seems happy enough at 28Mhz SPI.  Even at 37Mhz
    let lepton_spi_clock_freq: HertzU32 = 40_000_000.Hz();

    let mut peripherals = Peripherals::take().unwrap();
    //Enable the xosc, so that we can measure the rosc clock speed
    let xosc = match setup_xosc_blocking(peripherals.XOSC, XOSC_CRYSTAL_FREQ.Hz()) {
        Ok(xosc) => xosc,
        Err(_) => crate::panic!("xosc"),
    };

    let mut clocks = ClocksManager::new(peripherals.CLOCKS);
    clocks
        .reference_clock
        .configure_clock(&xosc, XOSC_CRYSTAL_FREQ.Hz())
        .unwrap();

    let measured_rosc_frequency = utils::find_target_rosc_frequency(&peripherals.ROSC, rosc_clock_freq.to_Hz());
    let rosc = RingOscillator::new(peripherals.ROSC);
    let mut rosc = rosc.initialize_with_freq(measured_rosc_frequency);
    let measured_hz: HertzU32 = measured_rosc_frequency.Hz();
    info!("Got {}, desired {}", measured_hz.to_MHz(), rosc_clock_freq.to_MHz());

    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    info!("System clock speed {}", clocks.system_clock.freq().to_MHz());
    let _xosc_disabled = xosc.disable();
    clocks.usb_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();

    // NOTE: PLLs are disabled by default.
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();


    let core = pac::CorePeripherals::take().unwrap();
    let mut sio = Sio::new(peripherals.SIO);

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let pins = bsp::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );
    {
        let mut pi_ping = pins.gpio5.into_push_pull_output();
        let _spi_clk = pins.gpio14.into_mode::<rp2040_hal::gpio::FunctionPio0>();
        let _spi_miso = pins.gpio15.into_mode::<rp2040_hal::gpio::FunctionPio0>();
        let _spi_cs = pins.gpio13.into_mode::<rp2040_hal::gpio::FunctionPio0>();
        let spi_cs_pin_id = 13;
        let miso_id = 15;

        let (mut pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
        let dma = peripherals.DMA.split(&mut peripherals.RESETS);
        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            let peripherals = unsafe { Peripherals::steal() };
            let mut sio = Sio::new(peripherals.SIO);
            sio.fifo.write_blocking(CORE1_TASK_READY);
            let radiometry_enabled = sio.fifo.read_blocking();

            // Setup a PIO-based SPI slave interface to send bytes to the raspberry pi
            let program_with_defines = pio_proc::pio_file!("./src/soft_spi_slave.pio");
            let installed = pio0.install(&program_with_defines.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
                .out_pins(miso_id, 1)
                .jmp_pin(spi_cs_pin_id)
                .out_shift_direction(ShiftDirection::Left)
                .pull_threshold(32)
                .autopush(true)
                .autopull(true)
                .build(sm0);
            let mut dma_ch0 = dma.ch0;
            sm.set_pindirs([(miso_id, bsp::hal::pio::PinDir::Output)]);
            sm.start();

            // Make sure we start our "pi-ping" pin low here.
            pi_ping.set_low().unwrap();

            pi_ping.set_high().unwrap();
            pi_ping.set_low().unwrap();
            unsafe {
                LittleEndian::write_u32(&mut FRAME_BUFFER[0..4], 0x1 << 28 | 8u32);
                LittleEndian::write_u32(&mut FRAME_BUFFER[4..8], radiometry_enabled);
                LittleEndian::write_u32(&mut FRAME_BUFFER[8..12], FIRMWARE_VERSION);
            }; // Read 8 bytes
            let tx_transfer = single_buffer::Config::new(dma_ch0, &unsafe { any_as_u32_slice(&FRAME_BUFFER) }[0..3], tx).start();
            // Wait on the DMA transfer to the Pi.  In the future we can be doing other work here while we wait.
            let (ch0, _, tx_ret) = tx_transfer.wait();
            tx = tx_ret;
            dma_ch0 = ch0;

            sio.fifo.write_blocking(CORE1_TASK_READY);
            pi_ping.set_low().unwrap();
            loop {
                let input = sio.fifo.read_blocking();
                crate::debug_assert_eq!(input, CORE1_TASK_START_WITH_FULL_FRAME, "Got unknown fifo input to core1 task loop {}", input);

                critical_section::with(|cs| {
                    for (seg_num, frame_seg) in FRAME_SEGMENT_BUFFER.get_back().borrow_ref(cs).iter().enumerate() {
                        let slice = frame_seg.as_u8_slice();
                        unsafe {
                            // Write the slice length to read
                            LittleEndian::write_u32(&mut FRAME_BUFFER[0..4], 0x2 << 28 | FRAME_LENGTH as u32);
                            let start = seg_num * slice.len();
                            let end = start + slice.len();
                            let frame_buffer = &mut FRAME_BUFFER[4..];
                            frame_buffer[start..end].copy_from_slice(slice);
                        }
                    }
                });

                pi_ping.set_high().unwrap();
                pi_ping.set_low().unwrap();
                let tx_transfer = single_buffer::Config::new(dma_ch0, unsafe { any_as_u32_slice(&FRAME_BUFFER) }, tx).start();
                // Wait on the DMA transfer to the Pi.  In the future we can be doing other work here while we wait.
                let (ch0, _, tx_ret) = tx_transfer.wait();
                tx = tx_ret;
                dma_ch0 = ch0;
                sio.fifo.write(CORE1_TASK_COMPLETE);
            }
        });
    }

    let (spi, actual_rate) = Spi::new(peripherals.SPI0).init(
        &mut peripherals.RESETS,
        clocks.peripheral_clock.freq(),
        lepton_spi_clock_freq,
        &embedded_hal::spi::MODE_3,
    );
    let mut lepton = Lepton::new(
        bsp::hal::I2C::i2c0(
            peripherals.I2C0,
            pins.gpio24.into_mode(),
            pins.gpio25.into_mode(),
            200.kHz(),
            &mut peripherals.RESETS,
            clocks.peripheral_clock.freq(),
        ),
        spi,
        pins.gpio21.into_mode(),
        pins.gpio22.into_mode(),
        pins.gpio20.into_mode(),
        pins.gpio23.into_mode(),
        pins.gpio19.into_mode(),
        // pins.gpio18.into_push_pull_output(), // Production P2 board
        pins.gpio4.into_push_pull_output(), // Dev test board
        pins.gpio27.into_push_pull_output(),
        pins.gpio28.into_push_pull_output(),
        pins.gpio29.into_push_pull_output(),
        pins.gpio26.into_push_pull_output(),
    );

    // TODO: When going dormant, can we make sure we don't have any gpio pins in a pullup/down mode.

    lepton
        .vsync
        .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, false);
    info!("Lepton startup sequence");
    lepton.power_down_sequence(&mut delay);
    lepton.power_on_sequence(&mut delay);
    // Set wake from dormant on vsync
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    lepton
        .vsync
        .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true);

    // TODO: On production board, this should be gpio4
    let mut wake_interrupt_pin = pins.gpio1.into_floating_input();

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    let result = sio.fifo.read_blocking();
    crate::debug_assert_eq!(result, CORE1_TASK_READY);
    sio.fifo.write_blocking(if radiometric_mode { 1 } else { 0 });

    let result = sio.fifo.read_blocking();
    crate::debug_assert_eq!(result, CORE1_TASK_READY);

    // NOTE: Do we want to do CRC checks?  What is the energy implication of this?
    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
    let do_crc_check = true;
    let mut crc_buffer = [0u8; 164];

    let mut prev_segment_was_4 = false;
    let mut scanline_buffer = [0u16; 82];

    let mut has_done_initial_ffc = false;
    let mut got_sync = false;
    let mut valid_frame_current_segment_num = 0u16;
    let mut started_segment = false;
    let mut attempt = 1;
    let mut transferring_prev_frame = false;
    let mut prev_frame_needs_transfer = false;

    // Each frame has 4 equally sized segments.
    let last_segment_num_for_frame = 4;
    // Each frame segment is comprised of 61 packets, zero based.
    let last_packet_id_for_segment = 60;
    // There are 2 "discard" frames between each real frame, each with 4 segments
    let total_segments_including_dummy_frames = 12;
    // Only packet 20 has a valid segment number, for some reason.
    let packet_id_with_valid_segment_num = 20;
    // Track the frame segment num including "discard" frames.
    let mut current_segment_num = 0;
    // Do FFC every 20 mins?
    let mut needs_ffc = false;
    let mut ffc_requested = false;

    let sda_pin = pins.gpio6.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_mode::<FunctionI2C>();
    let mut i2c = I2C::i2c1(
        peripherals.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );
    // Write three bytes to the I²C device with 7-bit address 0x2C
    // for i in 0u8..127u8 {
    //     if i2c.write(i, &[1, 2, 3]).is_ok() {
    //         info!("Found i2c device at {:#x}", i);
    //     }
    // }
    let mut attiny_regs = [0u8;24];
    if i2c.read(0x25, &mut attiny_regs).is_ok() {
        if attiny_regs[1] == 2 {
            info!("Should power off");
        }
        info!("camera state {:?}", attiny_regs);
    } else {
        info!("Failed to read i2c state from attiny");
    }
    // let mut cmd = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    // i2c.write_read(0x51, &[
    //     PCF8563_SEC_REG,
    //     PCF8563_MIN_REG,
    //     PCF8563_HR_REG,
    //     PCF8563_DAY_REG,
    //     PCF8563_MONTH_REG,
    //     PCF8563_YEAR_REG
    // ], &mut cmd).unwrap();
    // for (i, (unit, mask)) in [
    //     ("Second", 0x7f),
    //     ("Minute", 0x7f),
    //     ("Hour", 0x3f),
    //     ("Day", 0x07),
    //     ("Month", 0x1F),
    //     ("Year", 0xff)
    // ].iter().enumerate() {
    //     info!("#{}: {}: {}, {}", i, unit, bcd2dec(cmd[i] & mask), bcd2dec(cmd[i]));
    // }


    'frame_loop: loop {
        if got_sync {
            current_segment_num += 1;
            if current_segment_num > total_segments_including_dummy_frames {
                current_segment_num = 1;
            }
        }
        if got_sync && current_segment_num > last_segment_num_for_frame {
            // Go to sleep, skip dummy segment
            rosc = go_dormant_until_next_vsync(rosc, &mut lepton, measured_rosc_frequency);
            continue 'frame_loop;
        }
        if !transferring_prev_frame && prev_frame_needs_transfer {
            // Initiate the transfer of the previous frame
            FRAME_SEGMENT_BUFFER.swap();
            sio.fifo.write(CORE1_TASK_START_WITH_FULL_FRAME);
            transferring_prev_frame = true;
            prev_frame_needs_transfer = false;
        }
        {
            // Read the next frame
            let mut prev_packet_id = -1;
            'scanline: loop {
                // This is one scanline
                let packet = lepton.transfer(&mut scanline_buffer).unwrap();
                let packet_header = packet[0];
                let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
                if is_discard_packet {
                    continue 'scanline;
                }
                let packet_id = (packet_header & 0x0fff) as isize;
                let is_valid_packet_num = packet_id >= 0 && packet_id <= last_packet_id_for_segment;

                if packet_id == 0 {
                    prev_packet_id = -1;
                    started_segment = true;
                    // If we don't know, always start at segment 1 so that things will be
                    // written out.
                    if !got_sync || valid_frame_current_segment_num == 0 || prev_segment_was_4 {
                        valid_frame_current_segment_num = 1;
                    }

                    if got_sync && valid_frame_current_segment_num == 1 {
                        // Check if we need an FFC
                        let telemetry = &scanline_buffer[2..];
                        let mut buf = [0u8;4];
                        LittleEndian::write_u16_into(&telemetry[1..=2], &mut buf);
                        let time_on_ms = LittleEndian::read_u32(&buf);

                        LittleEndian::write_u16_into(&telemetry[30..=31], &mut buf);
                        let last_ffc_ms = LittleEndian::read_u32(&buf);

                        LittleEndian::write_u16_into(&telemetry[3..=4], &mut buf);
                        let status_bits = LittleEndian::read_u32(&buf);
                        let ffc_state = (status_bits >> 4) & 0x0000_000f;
                        let ffc_in_progress = ffc_state == 0b10;
                        let ffc_imminent = ffc_state == 0b01;
                        if time_on_ms - last_ffc_ms > FFC_INTERVAL_MS && !ffc_imminent && !ffc_in_progress {
                            needs_ffc = true;
                        }
                    }

                } else if packet_id == packet_id_with_valid_segment_num {
                    // Packet 20 is the only one that contains a meaningful segment number
                    let segment_num = packet_header >> 12;

                    // See if we're starting a frame, or ending it.
                    if valid_frame_current_segment_num > 1
                        && valid_frame_current_segment_num < 5
                        && segment_num != valid_frame_current_segment_num
                    {
                        // Segment order mismatch.
                        warn!("Segment order mismatch error: stored {}, this {}", current_segment_num, segment_num);
                        started_segment = false;
                        prev_segment_was_4 = false;

                        lepton.wait_for_ready(false);
                        lepton.reset_spi(& mut delay, true);
                        if !has_done_initial_ffc {
                            let _success = lepton.do_ffc();
                            has_done_initial_ffc = true;
                        }
                        break 'scanline;
                    }
                    valid_frame_current_segment_num = segment_num;
                    if valid_frame_current_segment_num == 0 {
                        started_segment = false;
                        break 'scanline;
                    }
                }

                let is_valid_segment_num = valid_frame_current_segment_num > 0 && valid_frame_current_segment_num <= last_segment_num_for_frame;
                let packets_are_in_order = packet_id == prev_packet_id + 1;
                if is_valid_segment_num && is_valid_packet_num && started_segment && packets_are_in_order {
                    if do_crc_check  {
                        let crc = scanline_buffer[1].to_le();
                        BigEndian::write_u16_into(&scanline_buffer, &mut crc_buffer);
                        crc_buffer[0] = crc_buffer[0] & 0x0f;
                        crc_buffer[2] = 0;
                        crc_buffer[3] = 0;
                        if crc_check.checksum(&crc_buffer) != crc && packet_id != 0 && valid_frame_current_segment_num != 1 {
                            warn!("Checksum fail on packet {}, segment {}", packet_id, current_segment_num);
                        }
                    }

                    // Copy the line out to the appropriate place in the current segment buffer.
                    critical_section::with(|cs| {
                        let segment_index = ((valid_frame_current_segment_num as u8).max(1).min(4) - 1) as usize;
                        let frame_seg = &mut FRAME_SEGMENT_BUFFER.get_front().borrow_ref_mut(cs)[segment_index];
                        // NOTE: We may be writing the incorrect seg number here initially, but it will always be
                        //  set correctly when we reach packet 20, assuming we do manage to write out a full segment.
                        // NOTE: We want LittleEndian for our purposes, but for emulating `leptond` it's big endian
                        BigEndian::write_u16_into(
                            &scanline_buffer[2..],
                            frame_seg.packet(packet_id as usize),
                        );
                    });

                    let is_last_segment = valid_frame_current_segment_num == 4;
                    prev_segment_was_4 = is_last_segment;
                    if packet_id == last_packet_id_for_segment {
                        if is_last_segment {
                            if needs_ffc && !ffc_requested {
                                ffc_requested = true;
                                needs_ffc = false;
                                info!("Requesting FFC");
                                let _ = lepton.do_ffc();
                            }
                            else if !has_done_initial_ffc {
                                // TODO: Does this make sense here?
                                let _ = lepton.do_ffc();
                                has_done_initial_ffc = true;
                            }
                            if !got_sync {
                                got_sync = true;
                                current_segment_num = valid_frame_current_segment_num;
                                warn!("Got sync");
                            }

                            attempt = 0;
                            prev_frame_needs_transfer = true;
                        } else {
                            // Increment in good faith if we're on the last packet of a valid segment
                            valid_frame_current_segment_num += 1;
                        }
                        started_segment = false;
                        break 'scanline;
                    }
                }

                // We only mark a segment as started if the packet num was 0 or 20.
                if started_segment && !packets_are_in_order {
                    if got_sync {
                        got_sync = false;
                        current_segment_num = 0;
                        warn!("Lost sync");
                    }
                    // Packet order mismatch
                    attempt += 1;
                    valid_frame_current_segment_num = 0;
                    started_segment = false;
                    prev_segment_was_4 = false;
                    if attempt > 250 && attempt % 5 == 0 {
                        warn!("Packet order mismatch current: {}, prev: {}, seg {} #{}", packet_id, prev_packet_id, current_segment_num, attempt);
                        lepton.wait_for_ready(false);
                        lepton.reset_spi(&mut delay, false);
                        if !has_done_initial_ffc {
                            let _ = lepton.do_ffc();
                            has_done_initial_ffc = true;
                        }
                        break 'scanline;
                    } else {
                        continue 'scanline;
                    }
                }
                prev_packet_id = packet_id;

                // Check if the previous frame is has finished transferring.
                if transferring_prev_frame {
                    if let Some(core_1_completed) = sio.fifo.read() {
                        if core_1_completed == CORE1_TASK_COMPLETE {
                            transferring_prev_frame = false;
                            prev_frame_needs_transfer = false;
                        }
                    }
                }
            }
        }

        /*
        if i2c.read(0x25, &mut attiny_regs).is_ok() {
            // If the pi is powered down, we can power down too.
            if attiny_regs[0] == 3 {
                info!("Powering off");
                // TODO: Wait for an interrupt on the wake pin, so we can be woken by a button press?
                // TODO: Are there any other pins we need to set low?
                lepton.power_down_sequence(&mut delay);
                rosc = go_dormant_until_woken(rosc, &mut wake_interrupt_pin, &mut lepton, measured_rosc_frequency);
                lepton.power_on_sequence(&mut delay);
                lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
                lepton
                    .vsync
                    .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true);
                got_sync = false;
                continue 'frame_loop;
            }
        }
         */

        // NOTE: If we're not transferring the previous frame, and the current segment is the second
        //  to last for a real frame, we can go dormant until the next vsync interrupt.
        if !transferring_prev_frame && current_segment_num == 3 {
            rosc = go_dormant_until_next_vsync(rosc, &mut lepton, measured_rosc_frequency);
        }
    }
}
