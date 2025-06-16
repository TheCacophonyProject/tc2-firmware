#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
mod attiny_rtc_i2c;
mod audio_task;
mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod cptv_encoder;
mod device_config;
mod event_logger;
mod ext_spi_transfers;
mod frame_processing;
mod lepton;
mod lepton_task;
mod motion_detector;
mod onboard_flash;
mod pdm_filter;
mod pdm_microphone;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod utils;

use crate::attiny_rtc_i2c::SharedI2C;
use crate::audio_task::audio_task;
use crate::event_logger::EventLogger;
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::frame_processing::{
    thermal_motion_task, Core0Task, FrameBuffer, StaticFrameBuffer, SyncedDateTime,
};
use crate::lepton::{init_lepton_module, LeptonPins};
pub use crate::lepton_task::frame_acquisition_loop;
use crate::onboard_flash::extend_lifetime_generic;
use crate::onboard_flash::OnboardFlash;
use crate::onboard_flash::{
    extend_lifetime_generic_mut, extend_lifetime_generic_mut_with_const_size,
};
use crate::startup_functions::{get_device_config, get_synced_time, should_record_audio};
use bsp::hal::watchdog::Watchdog;
use bsp::hal::Timer;
use bsp::{
    entry,
    hal::{
        clocks::Clock,
        multicore::{Multicore, Stack},
        pac,
        sio::Sio,
    },
    pac::Peripherals,
};
use byteorder::{ByteOrder, LittleEndian};
use core::cell::RefCell;
use cortex_m::asm::nop;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::*;
use defmt::{assert_eq, panic};
use defmt_rtt as _;
use device_config::DeviceConfig;
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::{FunctionI2C, FunctionSio, PullDown, PullUp, SioInput};
use rp2040_hal::pio::PIOExt;
use rp2040_hal::rosc::RingOscillator;
use rp2040_hal::I2C;

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 19;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.
                                                    // TODO Check against minor version also.

// got funny results at 150 for audio seems to work better at 125
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs

#[entry]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.
    let mut peripherals: Peripherals = Peripherals::take().unwrap();

    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        ROSC_TARGET_CLOCK_FREQ_HZ.Hz(),
    );
    let system_clock_freq = clocks.system_clock.freq().to_Hz();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );
    let sio = Sio::new(peripherals.SIO);
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // init flash
    let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
    let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };

    let pi_ping = pins.gpio5.into_pull_down_input();
    let pi_miso = pins.gpio15.into_floating_disabled();
    let pi_mosi = pins.gpio12.into_floating_disabled();
    let pi_cs = pins.gpio13.into_floating_disabled();
    let pi_clk = pins.gpio14.into_floating_disabled();

    let fs_cs = pins.gpio9.into_push_pull_output();
    let fs_miso = pins.gpio8.into_pull_down_disabled().into_pull_type();
    let fs_mosi = pins.gpio11.into_pull_down_disabled().into_pull_type();
    let fs_clk = pins.gpio10.into_pull_down_disabled().into_pull_type();

    let mut flash_storage = OnboardFlash::new(
        fs_cs,
        fs_mosi,
        fs_clk,
        fs_miso,
        flash_page_buf,
        flash_page_buf_2,
        dma_channels.ch1,
        dma_channels.ch2,
        true,
        None,
    );

    let mut payload_buf = [0x42u8; 2066];
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let mut crc_buf = [0x42u8; 32 + 104];
    let crc_buf = unsafe { extend_lifetime_generic_mut(&mut crc_buf) };
    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);

    let mut pi_spi = ExtSpiTransfers::new(
        pi_mosi,
        pi_cs,
        pi_clk,
        pi_miso,
        pi_ping,
        dma_channels.ch0,
        payload_buf,
        crc_buf,
        pio0,
        sm0,
    );

    flash_storage.take_spi(
        peripherals.SPI1,
        &mut peripherals.RESETS,
        system_clock_freq.Hz(),
    );
    flash_storage.init();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, system_clock_freq);

    // Attiny + RTC comms
    let i2c1 = I2C::i2c1(
        peripherals.I2C1,
        pins.gpio6.into_function::<FunctionI2C>(),
        pins.gpio7.into_function::<FunctionI2C>(),
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );

    info!("Initing shared i2c");
    // Early on in development we got strange errors when the raspberry pi was accessing the
    // attiny-provided i2c interface at the same time as we wanted to.  The hacky?/ingenious?
    // solution was to allocate a gpio pin that would determine who has the 'lock' on the i2c bus.
    // This is handled by this `SharedI2C` abstraction which mediates comms with the attiny.
    let mut shared_i2c = SharedI2C::new(
        i2c1,
        pins.gpio3
            .into_function::<FunctionSio<SioInput>>()
            .into_pull_type::<PullDown>(),
        &mut delay,
    );
    info!("Got shared i2c");

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    watchdog.enable_tick_generation((system_clock_freq / 1_000_000) as u8);

    watchdog.pause_on_debug(true);
    watchdog.start(8388607.micros());
    info!("Enabled watchdog timer");

    let mut timer: Timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);
    let mut event_logger = EventLogger::new(&mut flash_storage);
    let mut synced_date_time = get_synced_time(
        &mut shared_i2c,
        &mut delay,
        &mut event_logger,
        &mut flash_storage,
        &mut timer,
    );
    let config = get_device_config(
        &mut flash_storage,
        &mut shared_i2c,
        &mut delay,
        &mut pi_spi,
        system_clock_freq.Hz(),
        &mut timer,
    );

    let alarm_woke_us = shared_i2c.alarm_triggered(&mut delay);
    info!("Woken by RTC alarm? {}", alarm_woke_us);
    if alarm_woke_us {
        shared_i2c.clear_alarm(&mut delay);
    }
    if should_record_audio(&config, &mut shared_i2c, &mut delay, &synced_date_time) {
        audio_task(
            shared_i2c,
            system_clock_freq,
            timer,
            pins.gpio0,
            pins.gpio1,
            watchdog,
            alarm_woke_us,
            config,
            flash_storage,
            pi_spi,
            event_logger,
            synced_date_time,
        );
    } else {
        let disabled_alarm = shared_i2c.disable_alarm(&mut delay);
        if disabled_alarm.is_err() {
            error!("{}", disabled_alarm.unwrap());
        }
        let lepton_pins = LeptonPins {
            tx: pins.gpio23.into_function(),
            rx: pins.gpio20.into_function(),
            clk: pins.gpio22.into_function(),
            cs: pins.gpio21.into_function(),
            vsync: pins.gpio19.into_function(),
            sda: pins
                .gpio24
                .into_function::<FunctionI2C>()
                .into_pull_type::<PullUp>(),
            scl: pins
                .gpio25
                .into_function::<FunctionI2C>()
                .into_pull_type::<PullUp>(),
            power_down: pins.gpio28.into_push_pull_output(),
            power_enable: pins.gpio18.into_push_pull_output(),
            reset: pins.gpio29.into_push_pull_output(),
            clk_disable: pins.gpio27.into_push_pull_output(),
            master_clk: pins.gpio26.into_floating_input(),
        };
        thermal_code(
            shared_i2c,
            pi_spi,
            flash_storage,
            lepton_pins,
            watchdog,
            system_clock_freq,
            delay,
            timer,
            &clocks,
            rosc,
            alarm_woke_us,
            config,
            &mut event_logger,
            &mut synced_date_time,
        );
    }
}

pub fn thermal_code(
    shared_i2c: SharedI2C,
    pi_spi: ExtSpiTransfers,
    onboard_flash: OnboardFlash,
    lepton_pins: LeptonPins,
    mut watchdog: Watchdog,
    system_clock_freq: u32,
    delay: Delay,
    timer: Timer,
    clocks: &ClocksManager,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    alarm_woke_us: bool,
    config: DeviceConfig,
    event_logger: &mut EventLogger,
    synced_date_time: &mut SyncedDateTime,
) -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core1 = &mut cores[1];

    // NOTE: We're allocating the stack memory for core1 on our core0 stack rather than using
    //  a `static` var so that the memory isn't used when we're in the audio mode code-path.
    let mut core1_stack: Stack<470> = Stack::new();
    let mem = unsafe { extend_lifetime_generic_mut_with_const_size(&mut core1_stack.mem) };

    let mut fb0 = FrameBuffer::new();
    let mut fb1 = FrameBuffer::new();

    let frame_buffer_a = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb0)
    })));
    let frame_buffer_b = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb1)
    })));
    let static_frame_buffer_a = unsafe { extend_lifetime_generic(&frame_buffer_a) };
    let static_frame_buffer_b = unsafe { extend_lifetime_generic(&frame_buffer_b) };
    let peripheral_clock_freq: fugit::Rate<u32, 1, 1> = clocks.peripheral_clock.freq();

    watchdog.feed();
    watchdog.disable();

    let _ = core1.spawn(mem, move || {
        lepton_core1_task(
            lepton_pins,
            watchdog,
            system_clock_freq,
            timer,
            peripheral_clock_freq,
            rosc,
            static_frame_buffer_a,
            static_frame_buffer_b,
        );
    });

    thermal_motion_task(
        shared_i2c,
        pi_spi,
        onboard_flash,
        static_frame_buffer_a,
        static_frame_buffer_b,
        system_clock_freq,
        alarm_woke_us,
        timer,
        &config,
        event_logger,
        synced_date_time,
    );
}

pub fn lepton_core1_task(
    lepton_pins: LeptonPins,
    mut watchdog: Watchdog,
    system_clock_freq: u32,
    timer: Timer,
    peripheral_clock_freq: fugit::Rate<u32, 1, 1>,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
) -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut delay = Delay::new(core.SYST, system_clock_freq);

    let mut lepton = init_lepton_module(
        peripherals.SPI0,
        peripherals.I2C0,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut delay,
        lepton_pins,
    );

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    let lepton_serial = lepton.get_camera_serial().map_or(0, |x| x);
    let lepton_firmware_version = lepton
        .get_firmware_version()
        .map_or(((0, 0, 0), (0, 0, 0)), |x| x);
    let ((m_major, m_minor, m_build), (d_major, d_minor, d_build)) = lepton_firmware_version;
    info!(
        "Camera firmware versions: main: {}.{}.{}, dsp: {}.{}.{}",
        m_major, m_minor, m_build, d_major, d_minor, d_build
    );
    info!("Camera serial #{}", lepton_serial);
    info!("Radiometry enabled? {}", radiometric_mode);

    let result = sio.fifo.read_blocking();
    assert_eq!(result, Core0Task::ReadyToReceiveLeptonConfig.into());
    let main_lepton_firmware = LittleEndian::read_u32(&[m_major, m_minor, m_build, 0]);
    let dsp_lepton_firmware = LittleEndian::read_u32(&[d_major, d_minor, d_build, 0]);
    sio.fifo
        .write_blocking(Core0Task::SendIntercoreArray.into());
    sio.fifo.write_blocking(4);
    sio.fifo
        .write_blocking(if radiometric_mode { 2 } else { 1 });
    sio.fifo.write_blocking(lepton_serial);
    sio.fifo.write_blocking(main_lepton_firmware);
    sio.fifo.write_blocking(dsp_lepton_firmware);

    let result = sio.fifo.read_blocking();
    if result == Core0Task::RequestReset.into() {
        watchdog.start(100.micros());
        loop {
            nop();
        }
    }
    // FIXME: Can we receive `Core0Task::HighPowerMode` here instead?
    assert_eq!(result, Core0Task::Ready.into());
    frame_acquisition_loop(
        rosc,
        &mut lepton,
        &mut sio.fifo,
        peripheral_clock_freq,
        &mut delay,
        &mut peripherals.RESETS,
        static_frame_buffer_a,
        static_frame_buffer_b,
        watchdog,
    );
}
