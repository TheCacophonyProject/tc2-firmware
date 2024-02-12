use crate::bsp;
use crate::bsp::pac::{DMA, PIO1};
use cortex_m::singleton;
use defmt::{info, warn};
use fugit::HertzU32;
use rp2040_hal::dma::{double_buffer, CH3, CH4};
use rp2040_hal::dma::{single_buffer, Channel, Channels};
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1};
use rp2040_hal::gpio::{FunctionNull, FunctionPio1, Pin, PullNone};
use rp2040_hal::pio::{PIOBuilder, Running, Rx, StateMachine, Tx, UninitStateMachine, PIO, SM1};


use crate::core1_task::{core_1_task, Core1Pins, Core1Task};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};

const PDM_DECIMATION: usize = 64;
const SAMPLE_RATE: usize = 8000;

struct RecordingStatus {
    total_samples: usize,
    samples_taken: usize,
}

impl RecordingStatus {
    pub fn is_complete(&self) -> bool {
        self.total_samples == self.samples_taken
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
}

impl PdmMicrophone {
    pub fn new(
        data: Pin<Gpio0, FunctionNull, PullNone>,
        clk: Pin<Gpio1, FunctionNull, PullNone>,
        system_clock_hz: HertzU32,
        pio: PIO<PIO1>,
        state_machine_1_uninit: UninitStateMachine<(PIO1, SM1)>,
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
        // buffers: (
        //     singleton!(: [u32; 4] = [0;4]).unwrap(),
        // //     singleton!(: [u32; 4] = [0; 4]).unwrap(),
        // //     singleton!(: [u32; 4] = [0; 4]).unwrap(),
        // // singleton!(: [u32; 4] = [0; 4]).unwrap(),
        // // singleton!(: [u32; 4] = [0; 4]).unwrap(),
        // ),
        dma: Channels,
    ) {
        let mut current_recording = RecordingStatus {
            total_samples: SAMPLE_RATE * PDM_DECIMATION * num_seconds,
            samples_taken: 0,
        };
        self.enable();
        // Swap our buffers?

        // Pull out more samples via dma double_buffering.
        if let Some(pio_rx) = self.pio_rx.take() {
            // Chain some buffers together for continuous transfers
            // let mut b_0 = singleton!(: [u32; 64] = [0;64]).unwrap();
            // let mut b_1 = singleton!(: [u32; 64] =  [0;64]).unwrap();
            // let mut b_2 = singleton!(: [u32; 64] =  [0;64]).unwrap();
            let mut b_0 = [0; 64];
            let mut b_1 = [0; 64];
            let mut b_2 = [0; 64];

            let rx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), pio_rx, b_0).start();
            let mut rx_transfer = rx_transfer.write_next(b_1);

            let mut cycle = 0;
            loop {
                // When a transfer is done we immediately enqueue the buffers again.
                if rx_transfer.is_done() {
                    let (rx_buf, next_rx_transfer) = rx_transfer.wait();
                    // Take rx_buf that just finished transferring here, and write it to flash.  Provide another buffer
                    // for the next initially b0
                    // let back = *rx_buf;
                    // let next_buf = match cycle % 3 {
                    //     0 => b_2,
                    //     1 => b_1,
                    //     2 => b_0,
                    //     _ => {
                    //         unreachable!("mod 3")
                    //     }
                    // };

                    // next_buf = match cycle % d {}
                    cycle += 1;
                    for i in 0..rx_buf.len() {
                        if rx_buf[i] != 4294967295 {
                            info!("Got data {}", rx_buf[i]);
                        }
                    }
                    current_recording.samples_taken += rx_buf.len();
                    rx_transfer = next_rx_transfer.write_next(rx_buf);

                    // rx_transfer = next_rx_transfer.write_next(rx_buf);
                    info!("Read some data {}", current_recording.samples_taken / cycle);
                    // break;
                    // let tx_transfer = single_buffer::Config::new(&dma.ch2, back, spi).start();
                    // tx_transfer.wait();
                    // We can just assume that data is going to transfer to flash much faster, so we do it in bursts.

                    // Transfer back to flash.
                    // Do we need to double buffer things to flash?
                    // That would mean we'd want a quadruple buffer?
                    if current_recording.is_complete() {
                        break;
                    }
                }
            }
            // TODO: Uninstall pio_rx program etc.
        }
        // Put the pio_rx back?  I guess we can do work while the DMA transfer is happening, assuming we have enough time,
        // Otherwise we may need to transfer the buffer/pointer to another thread for fs work to happen.
    }
    pub fun spi_transfer(){
        let pins = Core1Pins {
                    pi_ping: pins.gpio5.into_pull_down_input(),
        
                    pi_miso: pins.gpio15.into_floating_disabled(),
                    pi_mosi: pins.gpio12.into_floating_disabled(),
                    pi_cs: pins.gpio13.into_floating_disabled(),
                    pi_clk: pins.gpio14.into_floating_disabled(),
        
                    fs_cs: pins.gpio9.into_push_pull_output(),
                    fs_miso: pins.gpio8.into_pull_down_disabled().into_pull_type(),
                    fs_mosi: pins.gpio11.into_pull_down_disabled().into_pull_type(),
                    fs_clk: pins.gpio10.into_pull_down_disabled().into_pull_type(),
                };
        let mut pi_spi = ExtSpiTransfers::new(
            pins.pi_mosi,
            pins.pi_cs,
            pins.pi_clk,
            pins.pi_miso,
            pins.pi_ping,
            dma_channels.ch0,
            payload_buf,
            crc_buf,
            pio0,
            sm0,
        );
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
