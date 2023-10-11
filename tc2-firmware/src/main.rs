#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]

mod lepton;
mod utils;

mod bsp;
mod clock_utils;
mod cptv_encoder;
mod double_frame_buffer;
mod ext_spi_transfers;
mod motion_detector;
mod onboard_flash;

use crate::cptv_encoder::streaming_cptv::CptvStream;
use crate::cptv_encoder::FRAME_WIDTH;
use crate::double_frame_buffer::FRAME_SEGMENT_BUFFER;
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::read_telemetry;
use crate::onboard_flash::OnboardFlash;
use crate::utils::{any_as_u8_slice, u8_slice_to_u16};
use bsp::{
    entry,
    hal::{
        clocks::{Clock, ClockSource, StoppableClock},
        multicore::{Multicore, Stack},
        pac,
        rosc::RingOscillator,
        sio::Sio,
        Spi,
    },
    pac::Peripherals,
};
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use cortex_m::delay::Delay;
use cortex_m::singleton;
use crc::{Crc, CRC_16_XMODEM};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_spi_Transfer,
    _embedded_hal_blocking_spi_Write, _embedded_hal_spi_FullDuplex,
};
use fugit::HertzU32;
use fugit::RateExtU32;
use lepton::Lepton;
use panic_probe as _;
use rp2040_hal::dma::{DMAExt, SingleChannel};
use rp2040_hal::gpio::{FunctionI2C, FunctionSio, Interrupt, Pin, PinId, PullNone, SioInput};
use rp2040_hal::I2C;

// This is 128KB, or half of our available memory
static mut CORE1_STACK: Stack<32768> = Stack::new();

// CORE1 consts
pub const START_TIMER: u32 = 0xfb;
pub const END_TIMER: u32 = 0xfc;
pub const CORE1_TASK_COMPLETE: u32 = 0xEE;
pub const CORE1_TASK_READY: u32 = 0xDB;
pub const CORE1_TASK_START_WITH_FRAME_SEGMENT: u32 = 0xAC;
pub const CORE1_TASK_START_WITH_FULL_FRAME: u32 = 0xAE;

pub const CORE1_RECORDING_STARTED: u32 = 0xBC;
pub const CORE1_RECORDING_ENDED: u32 = 0xBE;
pub const CORE1_TASK_START_WITHOUT_FRAME_SEGMENT: u32 = 0xAD;

pub const CORE1_TASK_START_FILE_TRANSFER: u32 = 0xAF;

// Each segment has 4 even slices to transfer from the total segment length of  9760 + 4*4bytes for the segment number.
const FRAME_LENGTH: usize = FRAME_WIDTH * 122 * 2;
pub const A_SEGMENT_LENGTH: usize = 9768; //9764;
pub const FULL_CHUNK_LENGTH: usize = A_SEGMENT_LENGTH / 4;
pub const CHUNK_LENGTH: usize = (A_SEGMENT_LENGTH - 4) / 4;
pub static mut FRAME_BUFFER: [u8; FRAME_LENGTH + 4] = [0u8; FRAME_LENGTH + 4];
pub const SEGMENT_LENGTH: usize = (FRAME_WIDTH * 122) / 4;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins

// transfer payloads to raspberry pi:

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub static FIRMWARE_VERSION: u32 = 3;

pub type FramePacketData = [u8; FRAME_WIDTH];
pub struct FrameSeg([FramePacketData; 61]);

impl FrameSeg {
    pub const fn new() -> FrameSeg {
        FrameSeg([[0u8; FRAME_WIDTH]; 61])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        unsafe { any_as_u8_slice(&self.0) }
    }
    pub fn packet(&mut self, packet_id: usize) -> &mut FramePacketData {
        &mut self.0[packet_id]
    }
}

// TODO: Just double buffer this on scanlines, or even FIFO between cores?

fn go_dormant_until_next_vsync<T: PinId>(
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    lepton: &mut Lepton<T>,
    rosc_freq: HertzU32,
) -> RingOscillator<bsp::hal::rosc::Enabled> {
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    let dormant_rosc = unsafe { rosc.dormant() };
    let disabled_rosc = RingOscillator::new(dormant_rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq);
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    initialized_rosc
}

fn go_dormant_until_woken<T: PinId, T2: PinId>(
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    wake_pin: &mut Pin<T2, FunctionSio<SioInput>, PullNone>,
    lepton: &mut Lepton<T>,
    rosc_freq: HertzU32,
) -> RingOscillator<bsp::hal::rosc::Enabled> {
    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    wake_pin.set_dormant_wake_enabled(Interrupt::EdgeHigh, true);
    let dormant_rosc = unsafe { rosc.dormant() };
    // Woken by pin
    let disabled_rosc = RingOscillator::new(dormant_rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq);
    wake_pin.set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    initialized_rosc
}

fn wake_raspberry_pi() {
    // TODO: Send a wake signal to the attiny, and poll its registers
    //  until we see that the raspberry pi is awake (and ideally that tc2-agent is running).
    //  Actually, tc2-agent will restart this firmware, so maybe we don't need to poll, we just
    //  block indefinitely?
    info!("Sent wake signal to raspberry pi");
}

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);

    let rosc_clock_freq: HertzU32 = 150_000_000.Hz();
    let lepton_spi_clock_freq: HertzU32 = 40_000_000.Hz();

    let mut peripherals = Peripherals::take().unwrap();
    let (clocks, mut rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        rosc_clock_freq,
    );
    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    crate::unreachable!("Break");
    let core = pac::CorePeripherals::take().unwrap();
    let mut sio = Sio::new(peripherals.SIO);

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );
    let should_record_new = false;
    let num_seconds = 80;
    let num_frames_to_record = num_seconds * 9;

    // TODO: I'd rather have these static arrays in ram.  Can we just unsafe pinky promise that they live for static?
    let crc_buf = singleton!(: [u8; 32] = [0x42; 32]).unwrap();
    let payload_buf = singleton!(: [u8; 2066] = [0x42; 2066]).unwrap();
    let flash_page_buf = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();
    let flash_page_buf_2 = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();
    {
        let pi_ping = pins.gpio5.into_push_pull_output();
        let cs = pins.gpio9.into_push_pull_output();
        let pi_miso = pins.gpio15.into_floating_disabled();
        let pi_mosi = pins.gpio12.into_floating_disabled();
        let pi_cs = pins.gpio13.into_floating_disabled();
        let pi_clk = pins.gpio14.into_floating_disabled();
        let fs_miso = pins.gpio8.into_pull_down_disabled().into_pull_type();
        let fs_mosi = pins.gpio11.into_pull_down_disabled().into_pull_type();
        let fs_clk = pins.gpio10.into_pull_down_disabled().into_pull_type();
        let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

        let mut pi_spi = ExtSpiTransfers::new(
            pi_mosi,
            pi_cs,
            pi_clk,
            pi_miso,
            pi_ping,
            dma_channels.ch0,
            payload_buf,
            crc_buf,
        );

        let mut spi_peripheral = Some(peripherals.SPI1);
        let clock_freq = clocks.peripheral_clock.freq();
        let mut flash_storage = OnboardFlash::new(
            cs,
            fs_mosi,
            fs_clk,
            fs_miso,
            flash_page_buf,
            flash_page_buf_2,
            dma_channels.ch1,
            dma_channels.ch2,
        );
        {
            // flash_storage.take_spi(
            //     spi_peripheral.take().unwrap(),
            //     &mut peripherals.RESETS,
            //     clock_freq,
            // );
            // flash_storage.init();
            // info!(
            //     "Finished scan, needs offload? {}",
            //     flash_storage.has_files_to_offload()
            // );
        }

        // let (mut pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);

        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // NOTE: This creates a buffer with a static lifetime, which is safe to access only this
            //  thread.
            let raspberry_pi_is_awake = true;
            let mut peripherals = unsafe { Peripherals::steal() };
            let core = unsafe { pac::CorePeripherals::steal() };
            let mut delay = Delay::new(core.SYST, sys_freq);
            let mut sio = Sio::new(peripherals.SIO);
            sio.fifo.write_blocking(CORE1_TASK_READY);
            let radiometry_enabled = sio.fifo.read_blocking();
            info!("Got radiometry enabled? {}", radiometry_enabled);

            if raspberry_pi_is_awake {
                let mut payload = [0u8; 8];
                let free_spi = flash_storage.free_spi();
                pi_spi.enable(free_spi, &mut peripherals.RESETS);

                LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
                LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);

                pi_spi.send_message(
                    ExtTransferMessage::CameraConnectInfo,
                    &payload,
                    &mut peripherals.DMA,
                );

                let spi_free = pi_spi.disable();
                flash_storage.take_spi(spi_free, &mut peripherals.RESETS, clock_freq);
            }
            // TODO: Switch modes here.
            //  If there are files to upload, do that.
            //  If the pi requests raw frames, do that (as well as being able to write to flash)
            //  Possibly we can get away with doing this via the PIO transfers?  Let's try!
            //  And then we switch to spi at the end to do the read-back?

            if false && flash_storage.has_files_to_offload() {
                warn!("There are files to offload!");
                if !raspberry_pi_is_awake {
                    wake_raspberry_pi();
                }
                if raspberry_pi_is_awake {
                    // do some offloading.
                    let mut file_count = 0;
                    flash_storage.begin_offload();
                    let mut file_start = true;
                    let mut part_count = 0;

                    // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
                    while let Some((part, is_last, spi)) = flash_storage.get_file_part() {
                        //fs_pins.disable();
                        pi_spi.enable(spi, &mut peripherals.RESETS);
                        let transfer_type = if file_start && !is_last {
                            ExtTransferMessage::BeginFileTransfer
                        } else if !file_start && !is_last {
                            ExtTransferMessage::ResumeFileTransfer
                        } else if is_last {
                            ExtTransferMessage::EndFileTransfer
                        } else if file_start && is_last {
                            ExtTransferMessage::BeginAndEndFileTransfer
                        } else {
                            crate::unreachable!("Invalid file transfer state");
                        };
                        let tt = match transfer_type {
                            ExtTransferMessage::BeginFileTransfer => "BEGIN FILE TRANSFER",
                            ExtTransferMessage::ResumeFileTransfer => "CONTINUE FILE TRANSFER",
                            ExtTransferMessage::EndFileTransfer => "END FILE TRANSFER",
                            ExtTransferMessage::BeginAndEndFileTransfer => {
                                "BEGIN & END FILE TRANSFER"
                            }
                            _ => crate::unreachable!("Invalid"),
                        };
                        //info!("tt {}, part len {}", tt, part.len());
                        pi_spi.send_message(transfer_type, &part, &mut peripherals.DMA);

                        part_count += 1;
                        if is_last {
                            file_count += 1;
                            info!("Offloaded {} file(s)", file_count);
                            file_start = true;
                        } else {
                            file_start = false;
                        }

                        // Give spi peripheral back to flash storage.
                        let spi = pi_spi.disable();
                        flash_storage.take_spi(spi, &mut peripherals.RESETS, clock_freq);
                    }
                    // TODO: Some validation from the raspberry pi that the transfer completed
                    //  without errors, in the form of a hash, and if we have errors, we'd re-transmit.

                    // Once we've successfully offloaded all files, we can erase the flash and we're
                    // ready to start recording new CPTV files there.
                    if should_record_new {
                        info!("Erasing after successful offload");
                        flash_storage.erase_all_good_used_blocks();
                    }
                }
            }

            // Let Core0 know that it can start the frame loop
            info!("Tell core 0 to start frame loop");
            sio.fifo.write_blocking(CORE1_TASK_READY);

            //pi_ping.set_low().unwrap();
            let mut motion_has_triggered = false;
            let mut this_frame_has_motion = should_record_new;
            let mut cptv_stream: Option<CptvStream> = None;

            let mut motion_left_frame = false;
            let mut five_seconds_have_passed_since_motion_left_frame = false;
            let mut thirty_seconds_have_passed_since_motion_disappeared_in_frame = false;
            let mut motion_disappeared_in_frame = false;
            let mut paused_cptv_recording_with_static_target_in_frame = false;
            let mut frames_written = 0;
            let mut frames_seen = 0;
            loop {
                let input = sio.fifo.read_blocking();
                crate::debug_assert_eq!(
                    input,
                    CORE1_TASK_START_WITH_FULL_FRAME,
                    "Got unknown fifo input to core1 task loop {}",
                    input
                );

                critical_section::with(|cs| {
                    for (seg_num, frame_seg) in FRAME_SEGMENT_BUFFER
                        .get_back()
                        .borrow_ref(cs)
                        .iter()
                        .enumerate()
                    {
                        let slice = frame_seg.as_u8_slice();
                        unsafe {
                            // Write the slice length to read
                            LittleEndian::write_u32(
                                &mut FRAME_BUFFER[0..4],
                                0x2 << 28 | FRAME_LENGTH as u32,
                            );
                            let start = seg_num * slice.len();
                            let end = start + slice.len();
                            let frame_buffer = &mut FRAME_BUFFER[4..];
                            frame_buffer[start..end].copy_from_slice(slice);
                        }
                    }
                });

                // Read the telemetry:
                let frame_buffer = unsafe { &mut FRAME_BUFFER[4..] };
                let frame_telemetry = read_telemetry(&frame_buffer);
                frames_seen += 1;
                // TODO: Add the frame to the current CPTV encode

                // TODO: Reserve next free page in storage for the cptv header, which will be written out
                // after the rest of the file.
                let should_start_new_recording = !motion_has_triggered
                    && this_frame_has_motion
                    && cptv_stream.is_none()
                    && frames_seen > 100; // wait until lepton stabilises before recording
                let should_end_current_recording = ((motion_left_frame
                    && five_seconds_have_passed_since_motion_left_frame)
                    || (motion_disappeared_in_frame
                        && thirty_seconds_have_passed_since_motion_disappeared_in_frame))
                    && cptv_stream.is_some();

                let raw_frame = unsafe { u8_slice_to_u16(&FRAME_BUFFER[644..]) }; // Telemetry + 4 byte header skipped

                //crate::assert_ne!(should_start_new_recording, should_end_current_recording);
                if should_start_new_recording && frames_written < num_frames_to_record {
                    error!("Starting new recording");
                    motion_has_triggered = true;
                    // Begin cptv file
                    // TODO: Record the current time when recording starts

                    // TODO: Pass in various cptv header info bits.
                    let mut cptv_streamer = CptvStream::new(0, &mut flash_storage);
                    cptv_streamer.init(&mut flash_storage, false);
                    cptv_stream = Some(cptv_streamer);

                    info!("Created CPTV stream");
                    sio.fifo.write(CORE1_RECORDING_STARTED);
                    // Write out the cptv header, with placeholders, then write out the previous
                    // frame and the current frame.  Going dormant on the main thread gets turned off?  Maybe, we'll see
                }
                if let Some(cptv_stream) = &mut cptv_stream {
                    // TODO: double buffer prev/current raw frames?
                    // At this point, do we really even want two cores running in tandem?
                    //wake_interrupt_pin.set_high().unwrap();
                    cptv_stream.push_frame(raw_frame, &frame_telemetry, &mut flash_storage);
                    //wake_interrupt_pin.set_low().unwrap();
                    frames_written += 1;
                }
                if should_end_current_recording || frames_written == num_frames_to_record {
                    if let Some(cptv_stream) = &mut cptv_stream {
                        error!("Ending current recording");
                        cptv_stream.finalise(&mut flash_storage);
                    }
                    cptv_stream = None;
                    sio.fifo.write(CORE1_RECORDING_ENDED);
                }
                // TODO: Encode and transfer to flash.  If rpi is awake, also transfer to rpi?

                // Check if we need to trigger:  Mostly at the moment we want to see what frame data
                // structures can be shared with encoding.

                // Wait on the DMA transfer to the Pi.  In the future we can be doing other work here while we wait.
                // if let Some(tx_transfer) = tx_transfer {
                //     let (ch0, _, tx_ret) = tx_transfer.wait();
                //     tx = tx_ret;
                //     dma_ch0 = ch0;
                // }
                sio.fifo.write(CORE1_TASK_COMPLETE);
            }

            // TODO: Need a way to exit this loop, and then offload files, and then re-enter the loop.
        });
    }

    let spi = Spi::new(
        peripherals.SPI0,
        (
            pins.gpio23.into_function(), // tx
            pins.gpio20.into_function(), // rx
            pins.gpio22.into_function(), // sck
        ),
    )
    .init(
        &mut peripherals.RESETS,
        clocks.peripheral_clock.freq(),
        lepton_spi_clock_freq,
        &embedded_hal::spi::MODE_3,
    );
    let mut lepton = Lepton::new(
        bsp::hal::I2C::i2c0(
            peripherals.I2C0,
            pins.gpio24.into_function(),
            pins.gpio25.into_function(),
            100.kHz(),
            &mut peripherals.RESETS,
            clocks.peripheral_clock.freq(),
        ),
        spi,
        pins.gpio21.into_function(), // cs
        pins.gpio19.into_function(),
        pins.gpio18.into_push_pull_output(), // Production P2 board
        pins.gpio28.into_push_pull_output(),
        pins.gpio29.into_push_pull_output(),
        pins.gpio27.into_push_pull_output(),
        pins.gpio26.into_floating_input(),
    );

    // TODO: When going dormant, can we make sure we don't have any gpio pins in a pullup/down mode.

    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    info!("Lepton startup sequence");
    lepton.power_down_sequence(&mut delay);
    lepton.power_on_sequence(&mut delay);
    // Set wake from dormant on vsync
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, true);

    //let mut wake_interrupt_pin = pins.gpio4.into_floating_input();

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    info!("Radiometry enabled? {}", radiometric_mode);
    let result = sio.fifo.read_blocking();
    crate::debug_assert_eq!(result, CORE1_TASK_READY);
    sio.fifo
        .write_blocking(if radiometric_mode { 1 } else { 0 });

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
    let mut is_recording = false;
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

    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();
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
    let mut attiny_regs = [0u8; 24];
    if i2c.read(0x25, &mut attiny_regs).is_ok() {
        if attiny_regs[1] == 2 {
            info!("Should power off");
        }
        info!("Attiny camera state {:?}", attiny_regs);
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
    let mut printed_telemetry_version = false;
    let mut frame_counter = 0;
    let mut unverified_frame_counter = 0;
    let mut prev_frame_counter = 0;
    let i2c_poll_counter = 0;
    'frame_loop: loop {
        if got_sync {
            current_segment_num += 1;
            if current_segment_num > total_segments_including_dummy_frames {
                current_segment_num = 1;
            }
        }
        if got_sync && current_segment_num > last_segment_num_for_frame && !is_recording {
            // Go to sleep, skip dummy segment
            rosc = go_dormant_until_next_vsync(rosc, &mut lepton, clocks.system_clock.freq());
            continue 'frame_loop;
        }
        if !transferring_prev_frame && prev_frame_needs_transfer {
            // Initiate the transfer of the previous frame
            FRAME_SEGMENT_BUFFER.swap();
            //info!("Hand off frame to core 1");
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
                let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
                if is_discard_packet {
                    continue 'scanline;
                }
                let packet_id = (packet_header & 0x0fff) as isize;
                let is_valid_packet_num = packet_id >= 0 && packet_id <= last_packet_id_for_segment;

                if packet_id == 0 {
                    prev_packet_id = -1;
                    started_segment = true;
                    //wake_interrupt_pin.set_high().unwrap();
                    // If we don't know, always start at segment 1 so that things will be
                    // written out.
                    if !got_sync || valid_frame_current_segment_num == 0 || prev_segment_was_4 {
                        valid_frame_current_segment_num = 1;
                    }
                    let mut buf = [0u8; 4];
                    let telemetry = &scanline_buffer[2..];
                    LittleEndian::write_u16_into(&telemetry[20..=21], &mut buf);
                    unverified_frame_counter = LittleEndian::read_u32(&buf);

                    if got_sync && valid_frame_current_segment_num == 1 {
                        // Check if we need an FFC
                        let telemetry = &scanline_buffer[2..];

                        // Let's check the telemetry version...
                        if !printed_telemetry_version {
                            printed_telemetry_version = true;
                            let telemetry_version =
                                LittleEndian::write_u16_into(&telemetry[0..1], &mut buf[0..2]);
                            info!("Lepton telemetry revision {}.{}", buf[1], buf[0]);
                        }

                        LittleEndian::write_u16_into(&telemetry[1..=2], &mut buf);
                        let time_on_ms = LittleEndian::read_u32(&buf);

                        LittleEndian::write_u16_into(&telemetry[30..=31], &mut buf);
                        let last_ffc_ms = LittleEndian::read_u32(&buf);
                        if unverified_frame_counter == prev_frame_counter + 2 {
                            if !is_recording
                                && time_on_ms != 0
                                && (last_ffc_ms != 0 || !has_done_initial_ffc)
                            {
                                // If time on ms is zero, that indicates a corrupt/invalid frame.
                                LittleEndian::write_u16_into(&telemetry[3..=4], &mut buf);
                                let status_bits = LittleEndian::read_u32(&buf);
                                let ffc_state = (status_bits >> 4) & 0x0000_000f;
                                let ffc_in_progress = ffc_state == 0b10;
                                let ffc_imminent = ffc_state == 0b01;

                                if time_on_ms < last_ffc_ms {
                                    warn!(
                                        "Time on less than last FFC: time_on_ms: {}, last_ffc_ms: {}",
                                        time_on_ms, last_ffc_ms
                                    );
                                } else if time_on_ms - last_ffc_ms > FFC_INTERVAL_MS
                                    && !ffc_imminent
                                    && !ffc_in_progress
                                {
                                    needs_ffc = true;
                                    ffc_requested = false;
                                }
                            }
                        }
                    }
                } else if packet_id == packet_id_with_valid_segment_num {
                    // Packet 20 is the only one that contains a meaningful segment number
                    let segment_num = packet_header >> 12;
                    if prev_packet_id == packet_id_with_valid_segment_num - 1 && segment_num == 1 {
                        prev_frame_counter = frame_counter;
                        frame_counter = unverified_frame_counter;
                    }

                    // See if we're starting a frame, or ending it.
                    if valid_frame_current_segment_num > 1
                        && valid_frame_current_segment_num < 5
                        && segment_num != valid_frame_current_segment_num
                    {
                        // Segment order mismatch.
                        warn!(
                            "Segment order mismatch error: stored {}, this {}",
                            current_segment_num, segment_num
                        );
                        started_segment = false;
                        prev_segment_was_4 = false;

                        lepton.wait_for_ready(false);
                        lepton.reset_spi(
                            &mut delay,
                            &mut peripherals.RESETS,
                            clocks.peripheral_clock.freq(),
                            lepton_spi_clock_freq,
                            true,
                        );
                        if !is_recording && !has_done_initial_ffc {
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

                let is_valid_segment_num = valid_frame_current_segment_num > 0
                    && valid_frame_current_segment_num <= last_segment_num_for_frame;
                let packets_are_in_order = packet_id == prev_packet_id + 1;
                if is_valid_segment_num
                    && is_valid_packet_num
                    && started_segment
                    && packets_are_in_order
                {
                    if do_crc_check {
                        let crc = scanline_buffer[1].to_le();
                        BigEndian::write_u16_into(&scanline_buffer, &mut crc_buffer);
                        crc_buffer[0] = crc_buffer[0] & 0x0f;
                        crc_buffer[2] = 0;
                        crc_buffer[3] = 0;
                        if crc_check.checksum(&crc_buffer) != crc
                            && packet_id != 0
                            && valid_frame_current_segment_num != 1
                        {
                            warn!(
                                "Checksum fail on packet {}, segment {}",
                                packet_id, current_segment_num
                            );
                        }
                    }

                    // Copy the line out to the appropriate place in the current segment buffer.
                    critical_section::with(|cs| {
                        let segment_index =
                            ((valid_frame_current_segment_num as u8).max(1).min(4) - 1) as usize;
                        let frame_seg =
                            &mut FRAME_SEGMENT_BUFFER.get_front().borrow_ref_mut(cs)[segment_index];
                        // NOTE: We may be writing the incorrect seg number here initially, but it will always be
                        //  set correctly when we reach packet 20, assuming we do manage to write out a full segment.
                        LittleEndian::write_u16_into(
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
                                info!("Requesting needed FFC");
                                let success = lepton.do_ffc();
                                match success {
                                    Ok(success) => {
                                        needs_ffc = false;
                                        info!("Success requesting needed FFC");
                                    }
                                    Err(e) => {
                                        info!("Failed to request FFC {:?}", e);
                                    }
                                }
                            } else if !has_done_initial_ffc {
                                info!("Requesting initial FFC");
                                let success = lepton.do_ffc();
                                match success {
                                    Ok(success) => {
                                        has_done_initial_ffc = true;
                                        info!("Success requesting initial FFC");
                                    }
                                    Err(e) => {
                                        info!("Failed to request FFC {:?}", e);
                                    }
                                }
                            }
                            if !got_sync && frame_counter == prev_frame_counter + 1 {
                                // TODO: Only set got sync if frame_count is = frame_count + 1 from previous frame.

                                got_sync = true;
                                current_segment_num = valid_frame_current_segment_num;
                                warn!("Got sync");
                            }

                            attempt = 0;
                            prev_frame_needs_transfer = true;
                            //wake_interrupt_pin.set_low().unwrap();
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
                        warn!(
                            "Packet order mismatch current: {}, prev: {}, seg {} #{}",
                            packet_id, prev_packet_id, current_segment_num, attempt
                        );
                        lepton.wait_for_ready(false);
                        lepton.reset_spi(
                            &mut delay,
                            &mut peripherals.RESETS,
                            clocks.peripheral_clock.freq(),
                            lepton_spi_clock_freq,
                            false,
                        );
                        if !has_done_initial_ffc {
                            info!("Requesting FFC");
                            let success = lepton.do_ffc();
                            match success {
                                Ok(success) => {
                                    info!("Success requesting FFC");
                                }
                                Err(e) => {
                                    info!("Failed to request FFC {:?}", e);
                                }
                            }
                            has_done_initial_ffc = true;
                        }
                        break 'scanline;
                    } else {
                        continue 'scanline;
                    }
                }
                prev_packet_id = packet_id;

                // Check if the previous frame is has finished transferring.
                // if transferring_prev_frame {
                //     if let Some(message) = sio.fifo.read() {
                //         if message == START_TIMER {
                //             let mut count = 0u32;
                //             while sio.fifo.read().is_none() {
                //                 count += 1;
                //             }
                //             sio.fifo.write(count);
                //         } else if message == CORE1_TASK_COMPLETE {
                //             transferring_prev_frame = false;
                //             prev_frame_needs_transfer = false;
                //         }
                //     }
                // }
                if transferring_prev_frame {
                    // TODO: Shouldn't we block and wait for core1 to complete?

                    if let Some(message) = sio.fifo.read() {
                        if message == CORE1_RECORDING_STARTED {
                            is_recording = true;
                        } else if message == CORE1_RECORDING_ENDED {
                            is_recording = false;
                        } else if message == CORE1_TASK_COMPLETE {
                            transferring_prev_frame = false;
                            prev_frame_needs_transfer = false;
                        }
                    }

                    // let message = sio.fifo.read_blocking();
                    // if message == CORE1_TASK_COMPLETE {
                    //     transferring_prev_frame = false;
                    //     prev_frame_needs_transfer = false;
                    // }
                }
            }
        }

        // TODO Check when can't get frames from lepton also.
        // Run `i2cset -y 1 0x25 0x02 0x03` on the pi to make it look like the RPi is powered off.
        // Run `i2cset -y 1 0x25 0x02 0x01` on the pi to make it look like the RPi is powered on.
        // Note that those you will want to have the RPi powered separately.
        // if i2c_poll_counter >= 20 {
        //     i2c_poll_counter = 0;
        //     let address = 0x25;
        //     let write_buffer: [u8; 1] = [0x02];
        //     if let Err(e) = i2c.write(address, &write_buffer) {
        //         error!("I2C write error: {:?}", e);
        //     } else {
        //         let mut read_buffer: [u8; 1] = [0; 1];
        //         if let Err(e) = i2c.read(address, &mut read_buffer) {
        //             error!("I2C read error: {:?}", e);
        //         }
        //
        //         if read_buffer[0] == 0x03 {
        //             info!("Powering off");
        //
        //             info!("Lets skip that for now, so we can get some logs...");
        //             /*
        //             // TODO: Wait for an interrupt on the wake pin, so we can be woken by a button press?
        //             // TODO: Are there any other pins we need to set low?
        //             lepton.power_down_sequence(&mut delay);
        //             rosc = go_dormant_until_woken(rosc, &mut wake_interrupt_pin, &mut lepton, measured_rosc_frequency);
        //             lepton.power_on_sequence(&mut delay);
        //             lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
        //             lepton
        //                 .vsync
        //                 .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true);
        //             got_sync = false;
        //             continue 'frame_loop;
        //             */
        //         }
        //     }
        // }
        // i2c_poll_counter += 1;

        // NOTE: If we're not transferring the previous frame, and the current segment is the second
        //  to last for a real frame, we can go dormant until the next vsync interrupt.
        if !is_recording && !transferring_prev_frame && current_segment_num == 3 {
            //wake_interrupt_pin.set_low().unwrap();
            rosc = go_dormant_until_next_vsync(rosc, &mut lepton, clocks.system_clock.freq());
            //wake_interrupt_pin.set_high().unwrap();
        } else if current_segment_num == 3 {
            //warn!("Overrunning frame time");
        }
    }
}
