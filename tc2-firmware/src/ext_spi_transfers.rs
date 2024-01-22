use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::{interrupt, DMA, PIO0, RESETS, SPI1};
use crate::onboard_flash::extend_lifetime;
use crate::utils::u8_slice_to_u32;
use byteorder::{ByteOrder, LittleEndian};
use core::cell::RefCell;
use core::ops::Not;
use critical_section::Mutex;
use defmt::{info, warn, Format};
use embedded_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
};
use fugit::MicrosDurationU32;
use rp2040_hal::dma::single_buffer::Transfer;
use rp2040_hal::dma::{single_buffer, Channel, CH0};
use rp2040_hal::gpio::bank0::{Gpio12, Gpio13, Gpio14, Gpio15, Gpio5};
use rp2040_hal::gpio::Interrupt::LevelLow;
use rp2040_hal::gpio::{
    FunctionNull, FunctionPio0, FunctionSio, FunctionSpi, Pin, PullDown, PullNone, PullUp, SioInput,
};
use rp2040_hal::pio::{
    PIOBuilder, Running, Rx, ShiftDirection, StateMachine, Tx, UninitStateMachine, PIO, SM0,
};
use rp2040_hal::timer::{Alarm, Alarm0};
use rp2040_hal::{Spi, Timer};

#[repr(u8)]
#[derive(Copy, Clone, Format, PartialEq)]
pub enum ExtTransferMessage {
    CameraConnectInfo = 0x1,
    CameraRawFrameTransfer = 0x2,
    BeginFileTransfer = 0x3,
    ResumeFileTransfer = 0x4,
    EndFileTransfer = 0x5,
    BeginAndEndFileTransfer = 0x6,
    GetMotionDetectionMask = 0x7,
    SendLoggerEvent = 0x8,
}

// We can store our ping pin here when we enter the ping-back interrupt
static GLOBAL_PING_PIN: Mutex<RefCell<Option<Pin<Gpio5, FunctionSio<SioInput>, PullUp>>>> =
    Mutex::new(RefCell::new(None));
static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let mut this_ref = GLOBAL_PING_PIN.borrow_ref_mut(cs);
        let ping_pin = this_ref.as_mut().unwrap();
        if ping_pin.interrupt_status(LevelLow) {
            ping_pin.clear_interrupt(LevelLow);
        }
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        let mut this_ref = GLOBAL_ALARM.borrow_ref_mut(cs);
        let alarm = this_ref.as_mut().unwrap();
        alarm.clear_interrupt();
    });
}

pub struct ExtSpiTransfers {
    pub spi: Option<
        Spi<
            rp2040_hal::spi::Enabled,
            SPI1,
            (
                Pin<Gpio15, FunctionSpi, PullNone>, // miso
                Pin<Gpio12, FunctionSpi, PullNone>, // mosi
                Pin<Gpio14, FunctionSpi, PullNone>, // clk
            ),
            8,
        >,
    >,
    cs_enabled: Option<Pin<Gpio13, FunctionSpi, PullNone>>, // cs
    miso_disabled: Option<Pin<Gpio15, FunctionNull, PullNone>>,
    mosi_disabled: Option<Pin<Gpio12, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio14, FunctionNull, PullNone>>,
    cs_disabled: Option<Pin<Gpio13, FunctionNull, PullNone>>,

    miso_pio: Option<Pin<Gpio15, FunctionPio0, PullNone>>,
    clk_pio: Option<Pin<Gpio14, FunctionPio0, PullNone>>,
    cs_pio: Option<Pin<Gpio13, FunctionPio0, PullNone>>,

    ping: Option<Pin<Gpio5, FunctionSio<SioInput>, PullDown>>,
    dma_channel_0: Option<Channel<CH0>>,
    payload_buffer: Option<&'static mut [u8; 2066]>,
    return_payload_buffer: Option<&'static mut [u8; 32 + 104]>,
    return_payload_offset: Option<usize>,
    pio: PIO<PIO0>,
    state_machine_0_uninit: Option<UninitStateMachine<(PIO0, SM0)>>,
    state_machine_0_running: Option<(StateMachine<(PIO0, SM0), Running>, Rx<(PIO0, SM0)>)>,
    pio_tx: Option<Tx<(PIO0, SM0)>>,
}
const DMA_CHANNEL_NUM: usize = 0;
impl ExtSpiTransfers {
    pub fn new(
        mosi: Pin<Gpio12, FunctionNull, PullNone>,
        cs: Pin<Gpio13, FunctionNull, PullNone>,
        clk: Pin<Gpio14, FunctionNull, PullNone>,
        miso: Pin<Gpio15, FunctionNull, PullNone>,
        ping: Pin<Gpio5, FunctionSio<SioInput>, PullDown>,
        dma_channel_0: Channel<CH0>,
        payload_buffer: &'static mut [u8; 2066],
        return_payload_buffer: &'static mut [u8; 32 + 104],
        pio: PIO<PIO0>,
        state_machine_0_uninit: UninitStateMachine<(PIO0, SM0)>,
    ) -> ExtSpiTransfers {
        ExtSpiTransfers {
            spi: None,
            cs_enabled: None,
            mosi_disabled: Some(mosi),
            cs_disabled: Some(cs),
            clk_disabled: Some(clk),
            miso_disabled: Some(miso),

            miso_pio: None,
            clk_pio: None,
            cs_pio: None,

            ping: Some(ping),
            dma_channel_0: Some(dma_channel_0),
            payload_buffer: Some(payload_buffer),
            return_payload_buffer: Some(return_payload_buffer),
            return_payload_offset: None,
            pio,
            state_machine_0_uninit: Some(state_machine_0_uninit),
            state_machine_0_running: None,
            pio_tx: None,
        }
    }

    pub fn return_payload(&self) -> Option<&'static [u8]> {
        if let Some(start) = self.return_payload_offset {
            Some(unsafe { extend_lifetime(&self.return_payload_buffer.as_ref().unwrap()[start..]) })
        } else {
            None
        }
    }

    pub fn enable(&mut self, spi: SPI1, resets: &mut RESETS) {
        self.disable_pio_spi();
        if self.spi.is_none() {
            self.cs_enabled = Some(
                self.cs_disabled
                    .take()
                    .unwrap()
                    .into_function::<FunctionSpi>()
                    .into_pull_type(),
            );
            let spi = Spi::<_, _, _, 8>::new(
                spi,
                (
                    self.miso_disabled
                        .take()
                        .unwrap()
                        .into_function::<FunctionSpi>()
                        .into_pull_type(),
                    self.mosi_disabled
                        .take()
                        .unwrap()
                        .into_function::<FunctionSpi>()
                        .into_pull_type(),
                    self.clk_disabled
                        .take()
                        .unwrap()
                        .into_function::<FunctionSpi>()
                        .into_pull_type(),
                ),
            )
            .init_slave(resets, &embedded_hal::spi::MODE_3);
            self.spi = Some(spi);
        }
    }
    pub fn ping(&mut self, timer: &mut Timer, pi_is_awake: bool) -> bool {
        let start = timer.get_counter();
        let ping_pin = self.ping.take().unwrap().into_pull_up_input();
        ping_pin.set_interrupt_enabled(LevelLow, true);
        let mut alarm = timer.alarm_0().unwrap();

        // Give away our pins by moving them into the `GLOBAL_PINS` variable.
        // We won't need to access them in the main thread again
        critical_section::with(|cs| {
            GLOBAL_PING_PIN.borrow(cs).replace(Some(ping_pin));
        });
        let _ = alarm.schedule(MicrosDurationU32::micros(300)).unwrap();
        alarm.enable_interrupt();
        critical_section::with(|cs| {
            GLOBAL_ALARM.borrow(cs).replace(Some(alarm));
        });
        // Unmask the IO_BANK0/TIMER_0) IRQ so that the NVIC interrupt controller
        // will jump to the interrupt function when the interrupt occurs.
        // We do this last so that the interrupt can't go off while
        // it is in the middle of being configured
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        }
        // Block until resumed by an interrupt from either the pin or from the alarm
        cortex_m::asm::wfi();

        pac::NVIC::mask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::mask(pac::Interrupt::TIMER_IRQ_0);

        let ping_time = (timer.get_counter() - start).to_micros();
        let (ping_pin, mut alarm) = critical_section::with(|cs| {
            (
                GLOBAL_PING_PIN.borrow(cs).take().unwrap(),
                GLOBAL_ALARM.borrow(cs).take().unwrap(),
            )
        });
        let finished = alarm.finished();
        alarm.clear_interrupt();
        let _ = alarm.cancel();
        alarm.disable_interrupt();
        ping_pin.set_interrupt_enabled(LevelLow, false);
        self.ping = Some(ping_pin.into_pull_down_input());

        // FIXME - Can we print this when we think the Pi should be awake?
        if finished && pi_is_awake {
            //warn!("Alarm triggered, ping took {}", ping_time);
        }
        // else if pi_is_awake {
        //     warn!("Pi responded, ping took {}", ping_time);
        // }
        !finished
    }

    pub fn begin_message(
        &mut self,
        message_type: ExtTransferMessage,
        payload: &mut [u8],
        crc: u16,
        is_recording: bool,
        dma_peripheral: &mut DMA,
        timer: &mut Timer,
    ) -> Option<(
        Transfer<Channel<CH0>, &'static [u32], Tx<(PIO0, SM0)>>,
        u32,
        u32,
    )> {
        if self.pio_tx.is_some() {
            // The transfer header contains the transfer type (2x)
            // the number of bytes to read for the payload (2x)
            // the 16 bit crc of the payload (twice)

            // It is followed by the payload itself
            let length = payload.len() as u32;
            let is_recording = if is_recording { 1 } else { 0 };

            let mut transfer_header = [0u8; 1 + 1 + 4 + 4 + 2 + 2 + 2 + 2];
            transfer_header[0] = message_type as u8;
            transfer_header[1] = message_type as u8;
            LittleEndian::write_u32(&mut transfer_header[2..6], payload.len() as u32);
            LittleEndian::write_u32(&mut transfer_header[6..10], payload.len() as u32);
            LittleEndian::write_u16(&mut transfer_header[10..12], is_recording);
            LittleEndian::write_u16(&mut transfer_header[12..14], is_recording);
            LittleEndian::write_u16(&mut transfer_header[14..16], is_recording.not());
            LittleEndian::write_u16(&mut transfer_header[16..=17], is_recording.not());
            payload[..transfer_header.len()].copy_from_slice(&transfer_header);

            loop {
                if !dma_peripheral.ch[DMA_CHANNEL_NUM]
                    .ch_ctrl_trig
                    .read()
                    .busy()
                    .bit_is_set()
                {
                    break;
                }
            }
            if self.ping(timer, false) {
                let mut config = single_buffer::Config::new(
                    self.dma_channel_0.take().unwrap(),
                    // Does this need to be aligned?  Maybe not.
                    unsafe { u8_slice_to_u32(extend_lifetime(&payload[..])) },
                    self.pio_tx.take().unwrap(),
                );
                config.bswap(true); // DMA peripheral does our swizzling for us.
                let transfer = config.start();
                let start_read_address = dma_peripheral.ch[DMA_CHANNEL_NUM]
                    .ch_read_addr
                    .read()
                    .bits();

                Some((transfer, start_read_address + length, start_read_address))
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn end_message(
        &mut self,
        dma_peripheral: &mut DMA,
        transfer_end_address: u32,
        transfer_start_address: u32,
        transfer: Transfer<Channel<CH0>, &'static [u32], Tx<(PIO0, SM0)>>,
    ) -> bool {
        // NOTE: Only needed if we thought the pi was awake, but then it goes to sleep
        // TODO: We need to timeout here?  What happens when tc2-agent goes away, then comes back?
        maybe_abort_dma_transfer(
            dma_peripheral,
            DMA_CHANNEL_NUM,
            transfer_end_address,
            transfer_start_address,
            0,
        );
        // Wait for the DMA transfer to finish
        let (r_ch0, _r_buf, tx) = transfer.wait();
        let end_read_addr = dma_peripheral.ch[DMA_CHANNEL_NUM]
            .ch_read_addr
            .read()
            .bits();
        let did_abort = end_read_addr + 20 < transfer_end_address;
        self.dma_channel_0 = Some(r_ch0);
        self.pio_tx = Some(tx);
        did_abort
    }

    pub fn enable_pio_spi(&mut self) {
        if self.pio_tx.is_none()
            && self.miso_disabled.is_some()
            && self.cs_disabled.is_some()
            && self.clk_disabled.is_some()
        {
            self.miso_pio = Some(
                self.miso_disabled
                    .take()
                    .unwrap()
                    .into_function()
                    .into_pull_type(),
            );
            self.cs_pio = Some(
                self.cs_disabled
                    .take()
                    .unwrap()
                    .into_function()
                    .into_pull_type(),
            );
            self.clk_pio = Some(
                self.clk_disabled
                    .take()
                    .unwrap()
                    .into_function()
                    .into_pull_type(),
            );
            let spi_cs_pin_id = 13;
            let miso_id = 15;
            // Setup a PIO-based SPI slave interface to send bytes to the raspberry pi
            let program_with_defines = pio_proc::pio_file!("./src/soft_spi_slave.pio");
            let installed = self.pio.install(&program_with_defines.program).unwrap();
            let (mut sm, rx, tx) = PIOBuilder::from_program(installed)
                .out_pins(miso_id, 1)
                .jmp_pin(spi_cs_pin_id)
                .out_shift_direction(ShiftDirection::Left)
                .pull_threshold(32)
                .autopush(true)
                .autopull(true)
                .build(self.state_machine_0_uninit.take().unwrap());
            sm.set_pindirs([(miso_id, bsp::hal::pio::PinDir::Output)]);
            self.state_machine_0_running = Some((sm.start(), rx));
            self.pio_tx = Some(tx);
        } else {
            warn!("Couldn't enable PIO SPI to send raw frames");
        }
    }

    pub fn disable_pio_spi(&mut self) {
        if self.pio_tx.is_some() {
            let (sm, rx) = self.state_machine_0_running.take().unwrap();
            let tx = self.pio_tx.take().unwrap();
            let (sm, program) = sm.uninit(rx, tx);
            self.pio.uninstall(program);

            self.miso_disabled = Some(
                self.miso_pio
                    .take()
                    .unwrap()
                    .into_function()
                    .into_pull_type(),
            );
            self.cs_disabled = Some(self.cs_pio.take().unwrap().into_function().into_pull_type());
            self.clk_disabled = Some(
                self.clk_pio
                    .take()
                    .unwrap()
                    .into_function()
                    .into_pull_type(),
            );

            self.state_machine_0_uninit = Some(sm);
        }
    }

    pub fn send_message(
        &mut self,
        message_type: ExtTransferMessage,
        payload: &[u8],
        crc: u16,
        dma_peripheral: &mut DMA,
        timer: &mut Timer,
        resets: &mut RESETS,
    ) -> bool {
        // The transfer header contains the transfer type (2x)
        // the number of bytes to read for the payload (should this be twice?)
        // the 16 bit crc of the payload (twice)

        // Looks like maybe we can't trust the first 8 bytes or so of the transfer to be aligned correctly?
        self.payload_buffer.as_mut().unwrap().fill(0x42);
        // It is followed by the payload itself
        let payload_length = payload.len() as u32;
        let actual_length = self.payload_buffer.as_ref().unwrap().len() as u32;
        // info!(
        //     "Send message {:?} of length {}/{}",
        //     message_type, payload_length, actual_length
        // );
        let mut transfer_header = [0u8; 18];
        let header_len = transfer_header.len() as u32;
        transfer_header[0] = message_type as u8;
        transfer_header[1] = message_type as u8;
        LittleEndian::write_u32(&mut transfer_header[2..6], payload_length);
        LittleEndian::write_u32(&mut transfer_header[6..10], payload_length);
        LittleEndian::write_u16(&mut transfer_header[10..12], crc);
        LittleEndian::write_u16(&mut transfer_header[12..14], crc);
        LittleEndian::write_u16(&mut transfer_header[14..16], crc.not());
        LittleEndian::write_u16(&mut transfer_header[16..=17], crc.not());
        let buffer_len = self.payload_buffer.as_ref().unwrap().len();
        self.payload_buffer.as_mut().unwrap()[0..transfer_header.len()]
            .copy_from_slice(&transfer_header);
        self.payload_buffer.as_mut().unwrap()
            [transfer_header.len()..transfer_header.len() + payload.len()]
            .copy_from_slice(&payload);

        let len = transfer_header.len() + payload.len();
        let mut transmit_success = false;
        let mut finished_transfer = false;
        while !transmit_success {
            if self.ping(timer, true) {
                finished_transfer = true;
                let start = timer.get_counter();
                let transfer = single_buffer::Config::new(
                    self.dma_channel_0.take().unwrap(),
                    self.payload_buffer.take().unwrap(),
                    self.spi.take().unwrap(),
                );
                let transfer = transfer.start();
                let transfer_read_address = dma_peripheral.ch[0].ch_read_addr.read().bits();
                maybe_abort_dma_transfer(
                    dma_peripheral,
                    DMA_CHANNEL_NUM,
                    transfer_read_address + actual_length,
                    transfer_read_address,
                    1,
                );
                let (r_ch0, r_buf, tx) = transfer.wait();
                self.dma_channel_0 = Some(r_ch0);
                self.payload_buffer = Some(r_buf);
                self.spi = Some(tx);

                // Now read the crc + return payload from the pi
                {
                    self.return_payload_buffer.as_mut().unwrap().fill(0);
                    let transfer = single_buffer::Config::new(
                        self.dma_channel_0.take().unwrap(),
                        self.spi.take().unwrap(),
                        self.return_payload_buffer.take().unwrap(),
                    )
                    .start();

                    let transfer_read_address = dma_peripheral.ch[DMA_CHANNEL_NUM]
                        .ch_read_addr
                        .read()
                        .bits();
                    let (r_ch0, spi, r_buf) = transfer.wait();
                    self.dma_channel_0 = Some(r_ch0);
                    // Find offset crc in buffer:
                    if let Some(start) = r_buf.iter().position(|&x| x == 1) {
                        if start < r_buf.len() - 8 {
                            let prelude = &r_buf[start + 1..start + 4];
                            if prelude[0] == 2 && prelude[1] == 3 && prelude[2] == 4 {
                                let crc_from_remote =
                                    LittleEndian::read_u16(&r_buf[start + 4..start + 6]);
                                let crc_from_remote_dup =
                                    LittleEndian::read_u16(&r_buf[start + 6..start + 8]);
                                if crc_from_remote == crc_from_remote_dup && crc_from_remote == crc
                                {
                                    transmit_success = true;
                                    //info!("Transfer success");
                                    if message_type == ExtTransferMessage::CameraConnectInfo {
                                        // We also expect to get a bunch of device config handshake info:
                                        self.return_payload_offset = Some(start + 4);
                                    } else if message_type
                                        == ExtTransferMessage::GetMotionDetectionMask
                                    {
                                        self.return_payload_offset = Some(start + 8);
                                    }
                                } else {
                                    info!("Return crc mismatch");
                                }
                            }
                        }
                    } else {
                        info!("Failed to find return data start in {:?}", r_buf);
                    }
                    self.return_payload_buffer = Some(r_buf);
                    self.spi = Some(spi);
                }
            } else {
                // warn!("Pi failed to receive");
                finished_transfer = false;
                transmit_success = true;
            }
        }
        finished_transfer
    }

    pub fn disable(&mut self) -> Option<SPI1> {
        if self.spi.is_some() {
            let spi = self.spi.take().unwrap();
            let spi_disabled = spi.disable();
            let (spi_free, (miso, mosi, clk)) = spi_disabled.free();
            self.mosi_disabled = Some(mosi.into_function::<FunctionNull>().into_pull_type());
            self.cs_disabled = Some(
                self.cs_enabled
                    .take()
                    .unwrap()
                    .into_function::<FunctionNull>()
                    .into_pull_type(),
            );

            self.clk_disabled = Some(clk.into_function::<FunctionNull>().into_pull_type());
            self.miso_disabled = Some(miso.into_function::<FunctionNull>().into_pull_type());
            Some(spi_free)
        } else {
            None
        }
    }

    pub fn write(&mut self, bytes: &[u8]) {
        self.spi.as_mut().unwrap().write(bytes).unwrap()
    }

    pub fn transfer<'a>(&mut self, bytes: &'a mut [u8]) -> &'a [u8] {
        self.spi.as_mut().unwrap().transfer(bytes).unwrap()
    }

    pub fn is_busy(&self) -> bool {
        self.spi.as_ref().unwrap().is_busy()
    }
}

fn maybe_abort_dma_transfer(
    dma: &mut DMA,
    channel: usize,
    transfer_end_address: u32,
    transfer_start_address: u32,
    location: u8,
) -> bool {
    // If the raspberry pi host requests less bytes than we want to send, we'd stall forever
    // unless we keep tabs on whether or not the channel has stalled, and abort appropriately.
    let mut same_address = 0;
    let mut prev_read_address = 0;
    let mut needs_abort = false;
    let mut some_progress = false;
    // Check that the FIFOs are empty too.
    loop {
        if dma.ch[channel].ch_ctrl_trig.read().busy().bit_is_set() {
            let current_transfer_read_address = dma.ch[channel].ch_read_addr.read().bits();
            if some_progress && prev_read_address == current_transfer_read_address {
                same_address += 1;
            }
            if same_address == 1_000_000 {
                //info!("Set needs abort with same address for 1million cycles");
                // We went 10,000 iterations without the crc changing, surely that means the transfer has stalled?
                needs_abort = true;
                break;
            }
            if prev_read_address != current_transfer_read_address {
                some_progress = true;
                same_address = 0;
            }
            prev_read_address = current_transfer_read_address;
        } else {
            break;
        }
    }
    if needs_abort && dma.ch[channel].ch_ctrl_trig.read().busy().bit_is_set() {
        info!(
            "Aborting dma transfer at {}/{}, #{}",
            prev_read_address - transfer_start_address,
            transfer_end_address - transfer_start_address,
            location
        );
        //info!("Aborting dma transfer");
        // See RP2040-E13 in rp2040 datasheet for explanation of errata workaround.
        let inte0 = dma.inte0.read().bits();
        let inte1 = dma.inte1.read().bits();
        let mask = (1u32 << channel).reverse_bits();
        dma.inte0.write(|w| unsafe { w.bits(inte0 & mask) });
        dma.inte1.write(|w| unsafe { w.bits(inte1 & mask) });
        // Abort all dma transfers
        dma.chan_abort.write(|w| unsafe { w.bits(1 << channel) });

        while dma.ch[channel].ch_ctrl_trig.read().busy().bit_is_set() {}

        dma.inte0.write(|w| unsafe { w.bits(inte0) });
        dma.inte1.write(|w| unsafe { w.bits(inte1) });
        true
    } else {
        false
    }
}

fn abort_dma(dma: &mut DMA, channel: usize) {
    if dma.ch[channel].ch_ctrl_trig.read().busy().bit_is_set() {
        let inte0 = dma.inte0.read().bits();
        let inte1 = dma.inte1.read().bits();
        let mask = (1u32 << channel).reverse_bits();
        dma.inte0.write(|w| unsafe { w.bits(inte0 & mask) });
        dma.inte1.write(|w| unsafe { w.bits(inte1 & mask) });
        // Abort all dma transfers
        dma.chan_abort.write(|w| unsafe { w.bits(1 << channel) });

        while dma.ch[channel].ch_ctrl_trig.read().busy().bit_is_set() {}

        dma.inte0.write(|w| unsafe { w.bits(inte0) });
        dma.inte1.write(|w| unsafe { w.bits(inte1) });
    }
}
