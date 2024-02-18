use crate::bsp;
use crate::bsp::pac::RESETS;
use crate::bsp::pac::{DMA, PIO1, SPI1};
use crate::utils::u32_slice_to_u8;
use bsp::{hal::sio::Sio, pac::Peripherals};
use cortex_m::singleton;
use defmt::{info, warn};
use fugit::HertzU32;
use rp2040_hal::dma::{double_buffer, CH3, CH4};
use rp2040_hal::dma::{single_buffer, Channel, Channels};
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1};
use rp2040_hal::gpio::{FunctionNull, FunctionPio1, Pin, PullNone};
use rp2040_hal::pio::{PIOBuilder, Running, Rx, StateMachine, Tx, UninitStateMachine, PIO, SM1};

use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::blocking::delay::DelayMs;
use rp2040_hal::Timer;
const PDM_DECIMATION: usize = 64;
const SAMPLE_RATE: usize = 18000;
// actually more like 8125 equates to 800 sr
struct RecordingStatus {
    total_samples: usize,
    samples_taken: usize,
}

impl RecordingStatus {
    pub fn is_complete(&self) -> bool {
        self.total_samples <= self.samples_taken
    }
}
pub type PageBufferRef = &'static mut [u8; 4 + 2048 + 128];
pub type DoublePageBuffer = (Option<PageBufferRef>, Option<PageBufferRef>);

pub struct PdmMicrophone {
    data_disabled: Option<Pin<Gpio0, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio1, FunctionNull, PullNone>>,
    data: Option<Pin<Gpio0, FunctionPio1, PullNone>>,
    clk: Option<Pin<Gpio1, FunctionPio1, PullNone>>,
    state_machine_1_running: Option<(StateMachine<(PIO1, SM1), Running>, Tx<(PIO1, SM1)>)>,
    state_machine_1_uninit: Option<UninitStateMachine<(PIO1, SM1)>>,
    pio_rx: Option<Rx<(PIO1, SM1)>>,
    system_clock_hz: HertzU32,
    buffers: Option<DoublePageBuffer>,
    pio: PIO<PIO1>,
    current_recording: Option<RecordingStatus>,
    dma_channel_0: Option<Channel<CH3>>,
    dma_channel_1: Option<Channel<CH4>>,
    spi: ExtSpiTransfers,
}
use rp2040_hal::dma::CH1;
use rp2040_hal::dma::CH2;

impl PdmMicrophone {
    pub fn new(
        data: Pin<Gpio0, FunctionNull, PullNone>,
        clk: Pin<Gpio1, FunctionNull, PullNone>,
        system_clock_hz: HertzU32,
        pio: PIO<PIO1>,
        state_machine_1_uninit: UninitStateMachine<(PIO1, SM1)>,
        spi: ExtSpiTransfers,
    ) -> PdmMicrophone {
        PdmMicrophone {
            data_disabled: Some(data),
            clk_disabled: Some(clk),
            data: None,
            clk: None,
            state_machine_1_running: None,
            system_clock_hz,
            pio_rx: None,
            buffers: None,
            pio,
            state_machine_1_uninit: Some(state_machine_1_uninit),
            current_recording: None,
            dma_channel_0: None,
            dma_channel_1: None,
            spi: spi,
        }
    }

    pub fn enable(&mut self) {
        let data: Pin<Gpio0, FunctionPio1, PullNone> = self
            .data_disabled
            .take()
            .unwrap()
            .into_function()
            .into_pull_type();
        let clk: Pin<Gpio1, FunctionPio1, PullNone> = self
            .clk_disabled
            .take()
            .unwrap()
            .into_function()
            .into_pull_type();

        let data_pin_id = data.id().num;
        let clk_pin_id = clk.id().num;

        self.data = Some(data);
        self.clk = Some(clk);

        // Start PIO program and work out a suitable clk_div

        // Data gets streamed in via a DMA channel, filtered, and then decimated.
        // We may also need to apply a gain step.

        let program_with_defines = pio_proc::pio_file!("./src/pdm_microphone.pio");
        let installed = self.pio.install(&program_with_defines.program).unwrap();
        // needs to run 4 instructions per evrery clock cycle
        // convert back to origianl sr SR * DECIMATION
        let clock_divider =
            self.system_clock_hz.to_Hz() as f32 / (SAMPLE_RATE * PDM_DECIMATION * 4) as f32;
        info!(
            "In {} side {} divider {}",
            data_pin_id, clk_pin_id, clock_divider
        );
        let clock_divider_fractional =
            (255.0 * clock_divider - (clock_divider as u32) as f32) as u8;

        // data_pin is in
        // clk pin is out
        // let data_pin_id = 1;
        // let clk_pin_id = 0;
        let (mut sm, rx, tx) = PIOBuilder::from_program(installed)
            .in_pin_base(data_pin_id)
            .side_set_pin_base(clk_pin_id)
            .pull_threshold(32)
            .push_threshold(32)
            .clock_divisor_fixed_point(clock_divider as u16, clock_divider_fractional)
            .autopush(true)
            .autopull(true)
            .build(self.state_machine_1_uninit.take().unwrap());
        sm.set_pindirs([(clk_pin_id, bsp::hal::pio::PinDir::Output)]);
        self.pio_rx = Some(rx);
        self.state_machine_1_running = Some((sm.start(), tx));
        // Start receiving data via DMA double buffer, and start streaming/writing out to disk, so
        // will need to have access to the fs
    }

    pub fn disable(&mut self) {
        let (sm, tx) = self.state_machine_1_running.take().unwrap();
        let rx = self.pio_rx.take().unwrap();
        let (sm, _program) = sm.uninit(rx, tx);

        self.data_disabled = Some(self.data.take().unwrap().into_function().into_pull_type());
        self.clk_disabled = Some(
            self.clk_disabled
                .take()
                .unwrap()
                .into_function()
                .into_pull_type(),
        );
        self.state_machine_1_uninit = Some(sm);
    }

    pub fn record_for_n_seconds(
        &mut self,
        num_seconds: usize,
        ch1: Channel<CH1>,
        ch2: Channel<CH2>,
        mut timer: Timer,
        dma_peripheral: &mut DMA,
        resets: &mut RESETS,
        spi: SPI1,
    ) {
        let mut current_recording = RecordingStatus {
            total_samples: SAMPLE_RATE * PDM_DECIMATION * num_seconds,
            samples_taken: 0,
        };
        info!(
            "Recording for {} is {}",
            num_seconds, current_recording.total_samples
        );
        self.enable();
        self.spi.enable(spi, resets);

        // Swap our buffers?
        // Pull out more samples via dma double_buffering.
        if let Some(pio_rx) = self.pio_rx.take() {
            let start = timer.get_counter();

            // Chain some buffers together for continuous transfers
            let b_0 = singleton!(: [u32; 512] = [0;512]).unwrap();
            let b_1 = singleton!(: [u32; 512] =  [0;512]).unwrap();

            let rx_transfer = double_buffer::Config::new((ch1, ch2), pio_rx, b_0).start();
            let mut rx_transfer = rx_transfer.write_next(b_1);

            loop {
                // When a transfer is done we immediately enqueue the buffers again.
                let (rx_buf, next_rx_transfer) = rx_transfer.wait();

                let payload = unsafe { &u32_slice_to_u8(rx_buf.as_mut()) };
                let transfer = self.spi.send_message(
                    ExtTransferMessage::AudioRawTransfer,
                    payload,
                    10u16,
                    dma_peripheral,
                    &mut timer,
                    resets,
                );

                current_recording.samples_taken += rx_buf.len() * 32;
                info!(
                    "Got {} samples out of {} samples",
                    current_recording.samples_taken, current_recording.total_samples
                );

                // We can just assume that data is going to transfer to flash much faster, so we do it in bursts.

                // Transfer back to flash.
                // Do we need to double buffer things to flash?
                // That would mean we'd want a quadruple buffer?
                if current_recording.is_complete() {
                    timer.delay_ms(1);

                    let transfer = self.spi.send_message(
                        ExtTransferMessage::AudioRawTransferFinished,
                        &[0u8; 0],
                        0u16,
                        dma_peripheral,
                        &mut timer,
                        resets,
                    );
                    self.spi.disable();

                    return;
                };
                rx_transfer = next_rx_transfer.write_next(rx_buf);

                // }
            }
            // TODO: Uninstall pio_rx program etc.
        }
        // Put the pio_rx back?  I guess we can do work while the DMA transfer is happening, assuming we have enough time,
        // Otherwise we may need to transfer the buffer/pointer to another thread for fs work to happen.
    }

    // pub fn read_next_page(&mut self) -> (Option<PageBufferRef>, Option<DoublePageBuffer>) {
    //     let mut current_recording = self.current_recording.take();
    //     if let Some(mut current_recording) = current_recording {
    //         if current_recording.is_complete() {
    //             let buffers = self.buffers.take();
    //             (None, buffers)
    //         } else {
    //             // Pull out more samples via dma double_buffering.

    //             // FIXME: Flow control
    //             // if let Some(pio_rx) = self.pio_rx.take() {
    //             //     // Chain some buffers together for continuous transfers
    //             //     let rx_transfer =
    //             //         double_buffer::Config::new((dma.ch2, dma.ch3), pio_rx, rx_buf).start();
    //             //     let mut rx_transfer = rx_transfer.write_next(rx_buf2);
    //             //     loop {
    //             //         // When a transfer is done we immediately enqueue the buffers again.
    //             //         if rx_transfer.is_done() {
    //             //             let (rx_buf, next_rx_transfer) = rx_transfer.wait();
    //             //             rx_transfer = next_rx_transfer.write_next(rx_buf);
    //             //         }
    //             //     }
    //             // }
    //             // Put the pio_rx back?  I guess we can do work while the DMA transfer is happening, assuming we have enough time,
    //             // Otherwise we may need to transfer the buffer/pointer to another thread for fs work to happen.

    //             current_recording.samples_taken += 1000;
    //             self.current_recording = Some(current_recording);

    //             (self.buffers.as_ref().1, None)
    //         }
    //     } else {
    //         (None, None)
    //     }
    // }
}
