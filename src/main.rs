// #![allow(warnings)]
#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
mod attiny_rtc_i2c;
mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod core0_audio;
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
use crate::core0_audio::audio_task;
pub use crate::core0_task::frame_acquisition_loop;
use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake,
    maybe_offload_flash_storage_and_events,
};
use crate::core1_task::{core_1_task, wake_raspberry_pi, Core1Pins, Core1Task, SyncedDateTime};
use crate::cptv_encoder::FRAME_WIDTH;
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{init_lepton_module, LeptonPins};
use crate::onboard_flash::extend_lifetime_generic;
use crate::pdm_microphone::PdmMicrophone;
use bsp::{
    entry,
    hal::{clocks::Clock, pac, sio::Sio},
    pac::Peripherals,
};
use chrono::{NaiveDate, NaiveDateTime, Timelike};
use cortex_m::asm::wfe;

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
use rp2040_hal::rosc::RingOscillator;

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
    let clocks: &'static ClocksManager = unsafe { extend_lifetime_generic(&clocks) };

    let system_clock_freq = clocks.system_clock.freq().to_Hz();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.

    let mut watchdog = bsp::hal::Watchdog::new(peripherals.WATCHDOG);
    watchdog.enable_tick_generation((system_clock_freq / 1_000_000) as u8);

    watchdog.pause_on_debug(true);
    watchdog.start(8388607.micros());

    info!("Enabled watchdog timer");
    let mut timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);

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
    let alarm_woke_us = shared_i2c.alarm_triggered(&mut delay);
    info!("Woken by RTC alarm? {}", alarm_woke_us);
    if alarm_woke_us {
        shared_i2c.clear_alarm();
    }
    // shared_i2c.disable_alarm(&mut delay);

    // If we're waking up to make an audio recording, do that.
    let mut existing_config = DeviceConfig::load_existing_config_from_flash();

    if let Some(existing_config) = &existing_config {
        info!("Existing config {:#?}", existing_config.config());
    }

    if existing_config.is_none() {
        // We need to wake up the rpi and get a config
        if !(wake_raspberry_pi(&mut shared_i2c, &mut delay)) {
            crate::panic!("Could not wake rpi to get config");
        }
    }

    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    // init flash
    let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
    let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };

    let mut flash_payload_buf = [0x42u8; 2115];
    let flash_payload_buf = unsafe { extend_lifetime_generic_mut(&mut flash_payload_buf) };
    let mut flash_storage = OnboardFlash::new(
        pins.gpio9.into_push_pull_output(),
        pins.gpio11.into_pull_down_disabled().into_pull_type(),
        pins.gpio10.into_pull_down_disabled().into_pull_type(),
        pins.gpio8.into_pull_down_disabled().into_pull_type(),
        flash_page_buf,
        flash_page_buf_2,
        dma_channels.ch1,
        dma_channels.ch2,
        true,
        Some(flash_payload_buf),
    );

    let mut payload_buf = [0x42u8; 2066];
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let mut crc_buf = [0x42u8; 32 + 104];
    let crc_buf = unsafe { extend_lifetime_generic_mut(&mut crc_buf) };
    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
    let mut pi_spi = ExtSpiTransfers::new(
        pins.gpio12.into_floating_disabled(),
        pins.gpio13.into_floating_disabled(),
        pins.gpio14.into_floating_disabled(),
        pins.gpio15.into_floating_disabled(),
        pins.gpio5.into_pull_down_input(),
        dma_channels.ch0,
        payload_buf,
        crc_buf,
        pio0,
        sm0,
    );
    let mut peripherals = unsafe { Peripherals::steal() };

    flash_storage.take_spi(
        peripherals.SPI1,
        &mut peripherals.RESETS,
        system_clock_freq.Hz(),
    );
    flash_storage.init();
    let (device_config, device_config_was_updated) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            &mut flash_storage,
            &mut pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            system_clock_freq.Hz(),
            1u32, //need to get radiometry and leton serial
            0,
            &mut timer,
            existing_config,
        );
    existing_config = device_config;
    // flash_storage.free_spi();

    let mut existing_config = existing_config.unwrap();
    info!(
        "Existing is audio {}",
        existing_config.config().is_audio_device
    );

    let i2c1: I2C<
        pac::I2C1,
        (
            rp2040_hal::gpio::Pin<
                rp2040_hal::gpio::bank0::Gpio6,
                rp2040_hal::gpio::FunctionI2c,
                rp2040_hal::gpio::PullDown,
            >,
            rp2040_hal::gpio::Pin<
                rp2040_hal::gpio::bank0::Gpio7,
                rp2040_hal::gpio::FunctionI2c,
                rp2040_hal::gpio::PullDown,
            >,
        ),
    > = shared_i2c.free();
    if existing_config.config().is_audio_device {
        audio_task(
            i2c1,
            &mut flash_storage,
            &mut pi_spi,
            system_clock_freq,
            &mut existing_config,
            &mut timer,
            pins.gpio0,
            pins.gpio1,
        );
    }
    loop {
        wfe();
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
