use core::time;

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::bsp::pac::RESETS;
use crate::bsp::pac::{DMA, PIO1, SPI1};
use crate::core1_sub_tasks::maybe_offload_flash_storage_and_events;
use crate::core1_task::SyncedDateTime;
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::utils::{
    u16_slice_to_u8, u16_slice_to_u8_mut, u32_slice_to_u8, u64_to_u16, u8_slice_to_u16,
};
use byteorder::{ByteOrder, LittleEndian};
use cortex_m::delay::Delay;
use cortex_m::singleton;
use defmt::{info, warn};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use fugit::HertzU32;
use rp2040_hal::dma::{double_buffer, CH3, CH4};
use rp2040_hal::dma::{single_buffer, Channel, Channels};
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1};
use rp2040_hal::gpio::{FunctionNull, FunctionPio1, Pin, PullNone};
use rp2040_hal::pio::{PIOBuilder, Running, Rx, StateMachine, Tx, UninitStateMachine, PIO, SM1};
use rp2040_hal::Timer;
const PDM_DECIMATION: usize = 64;
const SAMPLE_RATE: usize = 48000;
// actually more like 8125 equates to 800 sr
struct RecordingStatus {
    total_samples: usize,
    samples_taken: usize,
}
use crate::onboard_flash::OnboardFlash;

use crc::{Crc, CRC_16_XMODEM};

impl RecordingStatus {
    pub fn is_complete(&self) -> bool {
        self.total_samples <= self.samples_taken
    }
}

use crate::pdmfilter::PDMFilter;
pub struct PdmMicrophone {
    data_disabled: Option<Pin<Gpio0, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio1, FunctionNull, PullNone>>,
    data: Option<Pin<Gpio0, FunctionPio1, PullNone>>,
    clk: Option<Pin<Gpio1, FunctionPio1, PullNone>>,
    state_machine_1_running: Option<(StateMachine<(PIO1, SM1), Running>, Tx<(PIO1, SM1)>)>,
    state_machine_1_uninit: Option<UninitStateMachine<(PIO1, SM1)>>,
    pio_rx: Option<Rx<(PIO1, SM1)>>,
    system_clock_hz: HertzU32,
    pio: PIO<PIO1>,
    current_recording: Option<RecordingStatus>,
}

const VOLUME: u8 = 10;
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
            pio,
            state_machine_1_uninit: Some(state_machine_1_uninit),
            current_recording: None,
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
        let installed_program: rp2040_hal::pio::InstalledProgram<PIO1> =
            self.pio.install(&program_with_defines.program).unwrap();

        // needs to run 4 instructions per evrery clock cycle
        // convert back to origianl sr SR * DECIMATION
        // let clock_divider = self.system_clock_hz.to_MHz() as f32 / (4.0 * target_speed);
        //
        let clock_divider =
            self.system_clock_hz.to_Hz() as f32 / (SAMPLE_RATE * PDM_DECIMATION * 2) as f32;

        info!(
            "In {} side {} divider {}",
            data_pin_id, clk_pin_id, clock_divider
        );
        let clock_divider_fractional =
            (255.0 * (clock_divider - (clock_divider as u32) as f32)) as u8;
        info!(
            "This becomes {} {}",
            clock_divider as u16,
            clock_divider_fractional / 255
        );
        info!(
            " Mic CLock speed {}",
            self.system_clock_hz.to_MHz() as f32 / clock_divider / 2.0
        );
        // data_pin is in
        // clk pin is out
        // let data_pin_id = 1;
        // let clk_pin_id = 0;
        let (mut sm, rx, tx) = PIOBuilder::from_program(installed_program)
            .in_pin_base(data_pin_id)
            .side_set_pin_base(clk_pin_id)
            .pull_threshold(32)
            .push_threshold(32)
            .clock_divisor_fixed_point(clock_divider as u16, clock_divider_fractional)
            .autopush(true)
            .autopull(true)
            .build(self.state_machine_1_uninit.take().unwrap());
        sm.set_pindirs([
            (data_pin_id, bsp::hal::pio::PinDir::Input),
            (clk_pin_id, bsp::hal::pio::PinDir::Output),
        ]);
        self.pio_rx = Some(rx);
        self.state_machine_1_running = Some((sm.start(), tx));
        // Start receiving data via DMA double buffer, and start streaming/writing out to disk, so
        // will need to have access to the fs
    }
    pub fn alter_speed(&mut self, target_speed: f32) -> u32 {
        let (mut sm, tx) = self.state_machine_1_running.take().unwrap();
        let clock_divider = self.system_clock_hz.to_MHz() as f32 / (4.0 * target_speed);
        let clock_divider_fractional =
            (255.0 * (clock_divider - (clock_divider as u32) as f32)) as u8;
        sm.clock_divisor_fixed_point(clock_divider as u16, clock_divider_fractional);
        info!(
            "Altered Mic CLock speed {} divider {}",
            self.system_clock_hz.to_MHz() as f32 / clock_divider / 4.0,
            clock_divider
        );
        return (self.system_clock_hz.to_Hz() as f32 / clock_divider / 4.0 / 64.0) as u32;
    }
    pub fn disable(&mut self) {
        let (sm, tx) = self.state_machine_1_running.take().unwrap();
        let rx = self.pio_rx.take().unwrap();
        let (sm, _program) = sm.uninit(rx, tx);
        self.pio.uninstall(_program);

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
        ch3: Channel<CH3>,
        ch4: Channel<CH4>,
        timer: &mut Timer,
        resets: &mut RESETS,
        spi: SPI1,
        flash_storage: &mut OnboardFlash,
        timestamp: u64,
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
        let mut filter = PDMFilter::new(SAMPLE_RATE as u32);
        filter.init();
        let mut current_recording = RecordingStatus {
            total_samples: SAMPLE_RATE as usize * PDM_DECIMATION * num_seconds,
            samples_taken: 0,
        };
        // f
        // flash_storage.take_spi(spi, resets, self.system_clock_hz);
        // flash_storage.init();
        let crc_check: Crc<u16> = Crc::<u16>::new(&CRC_16_XMODEM);
        // Swap our buffers?
        let use_async: bool = true;
        // Pull out more samples via dma double_buffering.
        let mut transfer = None;
        let mut address = None;
        if let Some(pio_rx) = self.pio_rx.take() {
            let start: fugit::Instant<u64, 1, 1000000> = timer.get_counter();
            // Chain some buffers together for continuous transfers
            let b_0 = singleton!(: [u32; 512] = [0;512]).unwrap();
            let b_1 = singleton!(: [u32; 512] =  [0;512]).unwrap();
            let config = double_buffer::Config::new((ch3, ch4), pio_rx, b_0);
            let rx_transfer = config.start();
            // double_buffer::Config::new((ch3, ch4), pio_rx, b_0).start();
            let mut rx_transfer = rx_transfer.write_next(b_1);
            let mut cycle = 0;
            let mut audio_buffer = AudioBuffer::new();
            audio_buffer.init(timestamp, SAMPLE_RATE as u16);
            let start_block_index = flash_storage.start_file(0);
            loop {
                if rx_transfer.is_done() {
                    //this causes problems
                    warn!("Couldn't keep up with data discarding recording");
                    flash_storage.erase_block_range(
                        start_block_index,
                        flash_storage.last_used_block_index.unwrap(),
                    );
                    break;
                }
                // When a transfer is done we immediately enqueue the buffers again.
                let (rx_buf, next_rx_transfer) = rx_transfer.wait();
                cycle += 1;
                if (cycle < 200) {
                    // get the values initialized so the start of the recording is nice
                    let payload = unsafe { &u32_slice_to_u8(rx_buf.as_mut()) };
                    filter.filter(payload, VOLUME, &mut [0u16; 0], false);
                } else {
                    let payload = unsafe { &u32_slice_to_u8(rx_buf.as_mut()) };
                    let out = audio_buffer.slice_for(payload.len());
                    let (payload, leftover) = payload.split_at(out.len() * 8);

                    filter.filter(&payload, VOLUME, out, true);

                    if audio_buffer.is_full() {
                        let data_size = (audio_buffer.index - 2) * 2;
                        let data = audio_buffer.as_u8_slice();

                        if use_async {
                            info!("Writing {} size {}", data.len(), data_size);
                            timer.delay_us(700);

                            (transfer, address) = flash_storage.append_file_bytes_async(
                                data, data_size, false, None, None, transfer, address,
                            );
                        } else {
                            flash_storage.append_file_bytes(data, data_size, false, None, None);
                        }
                        audio_buffer.reset();
                        if leftover.len() > 0 {
                            // only works with this why???? even if i use new variables
                            timer.delay_us(700);
                            let out = audio_buffer.slice_for(leftover.len());
                            filter.filter(leftover, VOLUME, out, true);
                        }
                        // break;
                    }

                    current_recording.samples_taken += rx_buf.len() * 32;
                    if current_recording.is_complete() {
                        info!(
                            "Recording done counts are {} milis {}  samples {} took {}",
                            (timer.get_counter().ticks() - start.ticks()) as f32 / 1000000.0,
                            (timer.get_counter() - start).to_millis(),
                            current_recording.total_samples,
                            current_recording.samples_taken
                        );
                        let data_size = (audio_buffer.index - 2) * 2;
                        let mut payload = audio_buffer.as_u8_slice();
                        if use_async {
                            (transfer, address) = flash_storage.append_file_bytes_async(
                                payload, data_size, true, None, None, transfer, address,
                            );
                        } else {
                            flash_storage.append_file_bytes(payload, data_size, true, None, None);
                        }
                        break;
                    };
                }
                rx_transfer = next_rx_transfer.write_next(rx_buf);
            }
            // TODO: Uninstall pio_rx program etc.
        }
    }
}

const USER_BUFFER_LENGTH: usize = 1024;

// expecting 2066 bytes
// so for u16 data
pub struct AudioBuffer {
    data: [u16; (512 * 2 + 34)],
    index: usize,
}
const PAGE_COMMAND_ADDRESS: usize = 2;
const AUDIO_SHEBANG: u16 = 1;
impl AudioBuffer {
    pub const fn new() -> AudioBuffer {
        AudioBuffer {
            data: [0xffffu16; (512 * 2 + 34)],
            index: PAGE_COMMAND_ADDRESS,
        }
    }
    pub fn init(&mut self, timestamp: u64, samplerate: u16) {
        let time_data = unsafe { u64_to_u16(&timestamp) };
        let mut header: [u16; 4 + 1 + 1] = [0u16; 4 + 1 + 1];
        let u8_time = unsafe { u16_slice_to_u8(time_data) };

        header[0] = AUDIO_SHEBANG;
        header[1..1 + time_data.len()].copy_from_slice(&time_data);

        header[time_data.len() + 1] = samplerate;
        self.data[PAGE_COMMAND_ADDRESS..header.len() + PAGE_COMMAND_ADDRESS]
            .copy_from_slice(&header);
        self.index += header.len();
    }
    pub fn slice_for(&mut self, raw_data_length: usize) -> &mut [u16] {
        let end = self.index + raw_data_length / 8;
        let slice;
        if end > USER_BUFFER_LENGTH {
            slice = &mut self.data[self.index..USER_BUFFER_LENGTH];
        } else {
            slice = &mut self.data[self.index..self.index + raw_data_length / 8];
        }
        self.index += slice.len();
        return slice;
    }

    pub fn is_full(&mut self) -> bool {
        return self.index == USER_BUFFER_LENGTH;
    }

    pub fn reset(&mut self) {
        self.index = PAGE_COMMAND_ADDRESS;
    }

    pub fn as_u8_slice(&mut self) -> &mut [u8] {
        return unsafe { u16_slice_to_u8_mut(&mut self.data[..]) };
    }
}
