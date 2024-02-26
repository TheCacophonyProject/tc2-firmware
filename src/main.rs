#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
mod attiny_rtc_i2c;
mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod core0_task;
mod core1_sub_tasks;
mod core1_task;
mod cptv_encoder;
mod device_config;
mod event_logger;
mod example;
mod ext_spi_transfers;
mod lepton;
mod motion_detector;
mod onboard_flash;
mod pdm_microphone;
mod pdmfilter;
mod rp2040_flash;
mod sun_times;
mod utils;
use crate::attiny_rtc_i2c::SharedI2C;
pub use crate::core0_task::frame_acquisition_loop;
use crate::core1_task::{core_1_task, Core1Pins, Core1Task};
use crate::cptv_encoder::FRAME_WIDTH;
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::example::test_dma;
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{init_lepton_module, LeptonPins};
use crate::onboard_flash::extend_lifetime_generic;
use crate::pdm_microphone::PdmMicrophone;

use bsp::{
    entry,
    hal::{clocks::Clock, pac, sio::Sio},
    pac::Peripherals,
};

use core::cell::RefCell;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::gpio::FunctionI2C;
use rp2040_hal::I2C;
// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 9;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 12;
// static mut CORE1_STACK: Stack<45000> = Stack::new(); // 174,000 bytes
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 120_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs
pub type FramePacketData = [u8; FRAME_WIDTH];
pub type FrameSegments = [[FramePacketData; 61]; 4];
const TRANSFER_HEADER_LENGTH: usize = 18;
pub struct FrameBuffer([u8; TRANSFER_HEADER_LENGTH + (160 * 61 * 4) + 2]);

impl FrameBuffer {
    pub const fn new() -> FrameBuffer {
        // NOTE: Put an 18 byte padding at the start, and a 2 byte padding at the end, to make it 32bit aligned
        FrameBuffer([0u8; TRANSFER_HEADER_LENGTH + (160 * 61 * 4) + 2])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        &self.0[TRANSFER_HEADER_LENGTH..TRANSFER_HEADER_LENGTH + 39040]
    }

    pub fn as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }

    pub fn frame_data_as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0[TRANSFER_HEADER_LENGTH..TRANSFER_HEADER_LENGTH + 39040]
    }

    pub fn packet(&mut self, segment: usize, packet_id: usize) -> &mut [u8] {
        let segment_offset = FRAME_WIDTH * 61 * segment;
        let packet_offset = FRAME_WIDTH * packet_id;
        &mut self.0[TRANSFER_HEADER_LENGTH..]
            [segment_offset + packet_offset..segment_offset + packet_offset + FRAME_WIDTH]
    }
}
use rp2040_hal::dma::DMAExt;
// use rp2040_hal::pac;
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};
use rp2040_hal::pio::PIOExt;

#[entry]
fn main() -> ! {
    // let mut peripherals = pac::Peripherals::take().unwrap();
    // test_dma();
    // loop {}
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.

    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        ROSC_TARGET_CLOCK_FREQ_HZ.Hz(),
    );
    // let clocks = clock_utils::normal_clock();
    let clocks: &'static ClocksManager = unsafe { extend_lifetime_generic(&clocks) };

    let system_clock_freq = clocks.system_clock.freq().to_Hz();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.

    // let mut watchdog = bsp::hal::Watchdog::new(peripherals.WATCHDOG);
    // watchdog.pause_on_debug(true);
    // watchdog.start(8388607.micros());
    // watchdog.enable_tick_generation((sysd cac    tem_clock_freq / 1_000_000) as u8);

    info!("Enabled watchdog timer");
    let timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS, clocks);

    let core = pac::CorePeripherals::take().unwrap();
    let mut sio = Sio::new(peripherals.SIO);

    // let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    // let cores = mc.cores();
    // let core1 = &mut cores[1];

    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // Attiny + RTC comms
    let sda_pin = pins.gpio6.into_function::<FunctionI2C>();
    let scl_pin = pins.gpio7.into_function::<FunctionI2C>();
    let i2c1 = I2C::i2c1(
        peripherals.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );
    let mut delay = Delay::new(core.SYST, system_clock_freq);
    info!("Initing shared i2c");
    let mut shared_i2c = SharedI2C::new(i2c1, &mut delay);
    info!("Got shared i2c");
    // let alarm_woke_us = shared_i2c.alarm_triggered(&mut delay);
    // info!("Woken by RTC alarm? {}", alarm_woke_us);
    // if alarm_woke_us {
    //     shared_i2c.clear_alarm();
    // }
    // shared_i2c.disable_alarm(&mut delay);
    let i2c1 = shared_i2c.free();
    // If we're waking up to make an audio recording, do that.
    let existing_config = DeviceConfig::load_existing_config_from_flash();

    let i2c1 = if let Some(existing_config) = existing_config {
        info!(
            "Got device config {:?}, {}",
            existing_config,
            existing_config.device_name()
        );
        // Try to work out our wakeup reason.  If we're within a recording window, assume that we want
        // to record thermal video, otherwise make an audio recording and go to sleep.

        // We need a double buffering system of 3 flash pages.
        // One is the page that is currently being written to by data,
        // Another is ready to take over when that first buffer is full,

        let mut shared_i2c = SharedI2C::new(i2c1, &mut delay);
        match shared_i2c.get_datetime(&mut delay) {
            Ok(now) => {
                info!("Start mic rec");
                // let date_time_utc = get_naive_datetime(now);
                // let is_inside_recording_window =
                //     existing_config.time_is_in_recording_window(&date_time_utc);
                // if !is_inside_recording_window {
                // info!("Woke outside recording window, lets make an audio recording and go back to sleep?");
                // let mut pac = pac::Peripherals::take().unwrap();
                // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

                // let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

                let core1pins = Core1Pins {
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
                let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
                let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

                let mut crc_buf = [0x42u8; 32 + 104];
                let mut payload_buf = [0x42u8; 2066];
                let crc_buf = unsafe { extend_lifetime_generic_mut(&mut crc_buf) };
                let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };

                let mut pi_spi = ExtSpiTransfers::new(
                    core1pins.pi_mosi,
                    core1pins.pi_cs,
                    core1pins.pi_clk,
                    core1pins.pi_miso,
                    core1pins.pi_ping,
                    dma_channels.ch0,
                    payload_buf,
                    crc_buf,
                    pio0,
                    sm0,
                );

                let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
                let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
                let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
                let flash_page_buf_2 =
                    unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };
                let should_record_to_flash = true;

                let mut flash_storage = OnboardFlash::new(
                    core1pins.fs_cs,
                    core1pins.fs_mosi,
                    core1pins.fs_clk,
                    core1pins.fs_miso,
                    flash_page_buf,
                    flash_page_buf_2,
                    dma_channels.ch1,
                    dma_channels.ch2,
                    should_record_to_flash,
                );

                let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
                let mut microphone = PdmMicrophone::new(
                    pins.gpio0.into_function().into_pull_type(),
                    pins.gpio1.into_function().into_pull_type(),
                    system_clock_freq.Hz(),
                    pio1,
                    sm1,
                    pi_spi,
                );
                let mut front = [0u8; 2180];
                let mut back = [0u8; 2180];
                let mut front_ptr = &mut &mut front;
                let mut back_ptr = &mut &mut back;
                let mut peripherals = unsafe { Peripherals::steal() };
                // microphone.enable();
                let dt = shared_i2c.get_datetime(&mut delay).unwrap();
                let date_time_utc = get_naive_datetime(dt);

                info!("Started rec {}", date_time_utc.timestamp_millis());
                microphone.record_for_n_seconds(
                    5,
                    dma_channels.ch3,
                    dma_channels.ch4,
                    timer,
                    &mut peripherals.DMA,
                    &mut peripherals.RESETS,
                    peripherals.SPI1,
                    flash_storage,
                );
                let dt = shared_i2c.get_datetime(&mut delay).unwrap();
                let date_time_utc = get_naive_datetime(dt);

                info!("Ended rec {}", date_time_utc.timestamp_millis());
                // while let Some(data) = microphone.record_for_n_seconds(60) {
                // info!("Got mic data {:?}", data)
                // TODO: Process and stream the data out to flash before we get the next block.
                // }
                // }
            }
            Err(_) => error!("Unable to get DateTime from RTC"),
        }
        shared_i2c.free()
    } else {
        i2c1
    };
    loop {
        cortex_m::asm::wfe();
    }
    // thermal stuff
    // let lepton_pins = LeptonPins {
    //     tx: pins.gpio23.into_function(),
    //     rx: pins.gpio20.into_function(),
    //     clk: pins.gpio22.into_function(),
    //     cs: pins.gpio21.into_function(),

    //     vsync: pins.gpio19.into_function(),

    //     sda: pins.gpio24.into_function(),
    //     scl: pins.gpio25.into_function(),

    //     power_down: pins.gpio28.into_push_pull_output(),
    //     power_enable: pins.gpio18.into_push_pull_output(),
    //     reset: pins.gpio29.into_push_pull_output(),
    //     clk_disable: pins.gpio27.into_push_pull_output(),
    //     master_clk: pins.gpio26.into_floating_input(),
    // };
    // let mut lepton = init_lepton_module(
    //     peripherals.SPI0,
    //     peripherals.I2C0,
    //     system_clock_freq,
    //     &mut peripherals.RESETS,
    //     &mut delay,
    //     lepton_pins,
    // );

    // let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    // let lepton_serial = lepton.get_camera_serial().map_or(None, |x| Some(x));
    // let lepton_firmware_version = lepton.get_firmware_version().map_or(None, |x| Some(x));
    // if let Some(((m_major, m_minor, m_build), (d_major, d_minor, d_build))) =
    //     lepton_firmware_version
    // {
    //     info!(
    //         "Camera firmware versions: main: {}.{}.{}, dsp: {}.{}.{}",
    //         m_major, m_minor, m_build, d_major, d_minor, d_build
    //     );
    // }
    // info!("Camera serial #{}", lepton_serial);
    // info!("Radiometry enabled? {}", radiometric_mode);

    // let mut fb0 = FrameBuffer::new();
    // let mut fb1 = FrameBuffer::new();
    // let frame_buffer = Mutex::new(RefCell::new(Some(unsafe {
    //     extend_lifetime_generic_mut(&mut fb0)
    // })));
    // let frame_buffer_2 = Mutex::new(RefCell::new(Some(unsafe {
    //     extend_lifetime_generic_mut(&mut fb1)
    // })));
    // // Shenanigans to convince the second thread that all these values exist for the lifetime of the
    // // program.
    // let frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>> =
    //     unsafe { extend_lifetime_generic(&frame_buffer) };
    // let frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>> =
    //     unsafe { extend_lifetime_generic(&frame_buffer_2) };
    // watchdog.feed();
    // watchdog.disable();
    // let peripheral_clock_freq = clocks.peripheral_clock.freq();
    // {
    //     let pins = Core1Pins {
    //         pi_ping: pins.gpio5.into_pull_down_input(),

    //         pi_miso: pins.gpio15.into_floating_disabled(),
    //         pi_mosi: pins.gpio12.into_floating_disabled(),
    //         pi_cs: pins.gpio13.into_floating_disabled(),
    //         pi_clk: pins.gpio14.into_floating_disabled(),

    //         fs_cs: pins.gpio9.into_push_pull_output(),
    //         fs_miso: pins.gpio8.into_pull_down_disabled().into_pull_type(),
    //         fs_mosi: pins.gpio11.into_pull_down_disabled().into_pull_type(),
    //         fs_clk: pins.gpio10.into_pull_down_disabled().into_pull_type(),
    //     };
    //     let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
    //         core_1_task(
    //             frame_buffer_local,
    //             frame_buffer_local_2,
    //             system_clock_freq,
    //             pins,
    //             i2c1,
    //             lepton_serial,
    //             lepton_firmware_version,
    //             alarm_woke_us,
    //             timer,
    //         )
    //     });
    // }

    //     let result = sio.fifo.read_blocking();
    //     crate::assert_eq!(result, Core1Task::Ready.into());
    //     sio.fifo
    //         .write_blocking(if radiometric_mode { 2 } else { 1 });

    //     let result = sio.fifo.read_blocking();
    //     crate::assert_eq!(result, Core1Task::Ready.into());

    //     frame_acquisition_loop(
    //         rosc,
    //         &mut lepton,
    //         &mut sio.fifo,
    //         peripheral_clock_freq,
    //         &mut delay,
    //         &mut peripherals.RESETS,
    //         frame_buffer_local,
    //         frame_buffer_local_2,
    //         watchdog,
    //     );
}
