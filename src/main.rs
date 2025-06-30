#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
#![warn(clippy::all, clippy::pedantic)]
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
mod lepton_telemetry;
mod motion_detector;
mod onboard_flash;
mod pdm_filter;
mod pdm_microphone;
mod rpi_power;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod synced_date_time;
mod utils;

use crate::attiny_rtc_i2c::SharedI2C;
use crate::audio_task::audio_task;
use crate::event_logger::{Event, EventLogger};
use crate::ext_spi_transfers::{
    ExtSpiTransfers, RPI_PAYLOAD_LENGTH, RPI_RETURN_PAYLOAD_LENGTH, RPI_TRANSFER_HEADER_LENGTH,
};
use crate::frame_processing::{Core0Task, FrameBuffer, StaticFrameBuffer, thermal_motion_task};
use crate::lepton::{LeptonFirmwareInfo, LeptonPins, init_lepton_module};
pub use crate::lepton_task::frame_acquisition_loop;
use crate::onboard_flash::OnboardFlash;
use crate::onboard_flash::extend_lifetime_generic_mut;
use crate::onboard_flash::{FLASH_SPI_TOTAL_PAYLOAD_SIZE, extend_lifetime_generic};
use crate::startup_functions::{
    get_device_config, get_next_audio_alarm, get_synced_time,
    maybe_offload_files_and_events_on_startup, should_record_audio,
};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use bsp::hal::Timer;
use bsp::hal::watchdog::Watchdog;
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
use chrono::{DateTime, Utc};
use core::cell::RefCell;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::{assert, assert_eq, error, info, panic};
use defmt_rtt as _;
use device_config::DeviceConfig;
use fugit::{ExtU32, HertzU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::I2C;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::rosc::RingOscillator;

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 20;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.
// TODO Check against minor version also.

// got funny results at 150 for audio seems to work better at 125
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 20; // 20 mins between FFCs

#[entry]
#[allow(clippy::too_many_lines)]
fn main() -> ! {
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.
    let mut peripherals: Peripherals = Peripherals::take().unwrap();
    // Spit out the DMA channels, PIOs, and then steal peripherals again
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
    let mut peripherals = unsafe { Peripherals::steal() };

    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        ROSC_TARGET_CLOCK_FREQ_HZ.Hz(),
    );
    let system_clock_freq = clocks.system_clock.freq();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    assert!(system_clock_freq.to_Hz() / 1_000_000 <= u32::from(u8::MAX));
    #[allow(clippy::cast_possible_truncation)]
    watchdog.enable_tick_generation((system_clock_freq.to_Hz() / 1_000_000) as u8);
    let timer: Timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);
    let sio = Sio::new(peripherals.SIO);
    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // init flash
    // Create double buffers which are used for DMA transfers to the onboard flash module.
    // These need to live for the life of the program, so creating them here makes sense?
    // FIXME: I guess they could also be created inside OnboardFlash, since they'll live for as long as it
    //  does?  That would definitely feel much cleaner.
    let mut flash_page_buf = [0xffu8; FLASH_SPI_TOTAL_PAYLOAD_SIZE];
    let mut flash_page_buf_2 = [0xffu8; FLASH_SPI_TOTAL_PAYLOAD_SIZE];
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };
    let mut fs = {
        let fs_cs = pins.gpio9.into_push_pull_output();
        let fs_miso = pins.gpio8.into_pull_down_disabled().into_pull_type();
        let fs_mosi = pins.gpio11.into_pull_down_disabled().into_pull_type();
        let fs_clk = pins.gpio10.into_pull_down_disabled().into_pull_type();
        OnboardFlash::new(
            fs_cs,
            fs_mosi,
            fs_clk,
            fs_miso,
            flash_page_buf,
            flash_page_buf_2,
            dma_channels.ch1,
            dma_channels.ch2,
            None,
            system_clock_freq,
        )
    };

    // Setup soft/PIO SPI interface to raspberry pi
    let mut payload_buf = [0x42u8; RPI_TRANSFER_HEADER_LENGTH + RPI_PAYLOAD_LENGTH];
    let mut return_payload_buf = [0x42u8; RPI_RETURN_PAYLOAD_LENGTH];
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let return_payload_buf = unsafe { extend_lifetime_generic_mut(&mut return_payload_buf) };
    let mut pi_spi = {
        let pi_ping = pins.gpio5.into_pull_down_input();
        let pi_miso = pins.gpio15.into_floating_disabled();
        let pi_mosi = pins.gpio12.into_floating_disabled();
        let pi_cs = pins.gpio13.into_floating_disabled();
        let pi_clk = pins.gpio14.into_floating_disabled();
        ExtSpiTransfers::new(
            pi_mosi,
            pi_cs,
            pi_clk,
            pi_miso,
            pi_ping,
            dma_channels.ch0,
            payload_buf,
            return_payload_buf,
            pio0,
            sm0,
            timer,
        )
    };

    fs.take_spi(peripherals.SPI1, &mut peripherals.RESETS);
    fs.init();

    let mut delay = Delay::new(
        pac::CorePeripherals::take().unwrap().SYST,
        system_clock_freq.to_Hz(),
    );

    // Attiny + RTC comms
    info!("Initing shared i2c");
    // Early on in development we got strange errors when the raspberry pi was accessing the
    // attiny-provided i2c interface at the same time as we wanted to.  The hacky?/ingenious?
    // solution was to allocate a gpio pin that would determine who has the 'lock' on the i2c bus.
    // This is handled by this `SharedI2C` abstraction which mediates comms with the attiny.
    let mut i2c = SharedI2C::new(
        I2C::i2c1(
            peripherals.I2C1,
            pins.gpio6.reconfigure(),
            pins.gpio7.reconfigure(),
            400.kHz(),
            &mut peripherals.RESETS,
            &clocks.system_clock,
        ),
        pins.gpio3.reconfigure(),
        &mut delay,
    );

    watchdog.pause_on_debug(true);
    watchdog.start(8_388_607.micros());

    info!("Enabled watchdog timer");
    let mut events = EventLogger::new(&mut fs);
    let time = get_synced_time(&mut i2c, &mut delay, &mut events, &mut fs, timer);

    if let Ok(is_recording) = i2c.get_is_recording(&mut delay) {
        // Unset the is_recording flag on attiny on startup if still enabled.
        // If it was still set, that suggests a previous recording was interrupted.
        // TODO: We can check if that is true by checking the is_last flag on the last page
        //  written.
        if is_recording {
            events.log(Event::RecordingNotFinished, &time, &mut fs);
            if let Err(e) = i2c.stopped_recording(&mut delay) {
                error!("Error unsetting recording flag on attiny: {}", e);
            }
        }
    }

    let (device_config, prefer_not_to_offload_files_now) = get_device_config(
        &mut fs,
        &mut i2c,
        &mut delay,
        &mut pi_spi,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
    );

    let woken_by_alarm = i2c.alarm_triggered(&mut delay);
    if woken_by_alarm {
        info!("Woken by RTC alarm? {}", woken_by_alarm);
        events.log(Event::Rp2040WokenByAlarm, &time, &mut fs);
        i2c.clear_alarm(&mut delay);
    }

    let next_audio_alarm = get_next_audio_alarm(
        &device_config,
        &mut fs,
        &time,
        &mut delay,
        &mut i2c,
        &mut events,
    );
    let record_audio_now = should_record_audio(&device_config, &mut i2c, &mut delay, &time);

    // TODO: Maybe check sun_times for valid lat/lng which can give us a recording window,
    //  log if we can't get one

    // There are a number of reasons we may or may not want to offload files and events on startup.

    // If the rPi is already awake, we want to offload
    // - if the flash is full
    // - if we're not in a recording window, or it has been more than 24hrs since the previous offload.
    // - if an audio recording is not scheduled imminently?
    // - AND the user is not interacting with the device via sidekick.

    // Maybe offload files
    // TODO: We might defer this if the user is using sidekick etc?
    // TODO: Make offloads interruptable?
    let recording_mode = if record_audio_now { "audio" } else { "thermal" };

    // TODO: If the latest file is a test recording, at least offload the latest immediately.
    let did_offload = maybe_offload_files_and_events_on_startup(
        recording_mode,
        prefer_not_to_offload_files_now,
        &mut fs,
        &device_config,
        &time,
        &mut events,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        &mut i2c,
        &mut pi_spi,
        &mut delay,
        &mut watchdog,
    );

    // TODO: We might defer this if the user is using sidekick etc?
    if record_audio_now {
        let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
        audio_task(
            i2c,
            system_clock_freq,
            pins.gpio0,
            pins.gpio1,
            watchdog,
            woken_by_alarm,
            &device_config,
            fs,
            events,
            &time,
            pio1,
            sm1,
            delay,
        );
    } else {
        let disabled_alarm = i2c.disable_alarm(&mut delay);
        if disabled_alarm.is_err() {
            error!("{}", disabled_alarm.unwrap());
        }

        let lepton_pins = LeptonPins {
            tx: pins.gpio23.into_function(),
            rx: pins.gpio20.into_function(),
            clk: pins.gpio22.into_function(),
            cs: pins.gpio21.into_function(),
            vsync: pins.gpio19.into_function(),
            sda: pins.gpio24.reconfigure(),
            scl: pins.gpio25.reconfigure(),
            power_down: pins.gpio28.into_push_pull_output(),
            power_enable: pins.gpio18.into_push_pull_output(),
            reset: pins.gpio29.into_push_pull_output(),
            clk_disable: pins.gpio27.into_push_pull_output(),
            master_clk: pins.gpio26.into_floating_input(),
        };
        thermal_code(
            i2c,
            pi_spi,
            fs,
            lepton_pins,
            watchdog,
            system_clock_freq,
            delay,
            &clocks,
            rosc,
            &device_config,
            events,
            time,
            next_audio_alarm,
        );
    }
}

/// # Panics
///
/// TODO
pub fn thermal_code(
    i2c: SharedI2C,
    pi_spi: ExtSpiTransfers,
    onboard_flash: OnboardFlash,
    lepton_pins: LeptonPins,
    watchdog: Watchdog,
    system_clock_freq: HertzU32,
    delay: Delay,
    clocks: &ClocksManager,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    device_config: &DeviceConfig,
    events: EventLogger,
    time: SyncedDateTime,
    next_audio_alarm: Option<DateTime<Utc>>,
) -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core_1 = &mut cores[1];
    // NOTE: We're allocating the stack memory for core1 on our core0 stack rather than using
    //  a `static` var so that the memory isn't used when we're in the audio mode code-path.
    let core1_stack: Stack<470> = Stack::new();
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
    let peripheral_clock_freq = clocks.peripheral_clock.freq();

    watchdog.feed();
    watchdog.disable();

    let _ = core_1.spawn(core1_stack.take().unwrap(), move || {
        lepton_core1_task(
            lepton_pins,
            watchdog,
            system_clock_freq,
            peripheral_clock_freq,
            &rosc,
            static_frame_buffer_a,
            static_frame_buffer_b,
        );
    });

    thermal_motion_task(
        delay,
        sio,
        peripherals.DMA,
        i2c,
        pi_spi,
        onboard_flash,
        static_frame_buffer_a,
        static_frame_buffer_b,
        device_config,
        events,
        time,
        next_audio_alarm,
    );
}

pub fn lepton_core1_task(
    lepton_pins: LeptonPins,
    mut watchdog: Watchdog,
    system_clock_freq: HertzU32,
    peripheral_clock_freq: HertzU32,
    rosc: &RingOscillator<bsp::hal::rosc::Enabled>,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
) -> ! {
    // This task runs on the second core, so we need to steal the peripherals.
    // Core peripherals are per core, so we can just take our copy (but the current cortex-m API
    // makes us steal it anyway)

    let mut peripherals = unsafe { Peripherals::steal() };
    let sio = Sio::new(peripherals.SIO);
    let mut fifo = sio.fifo;
    let mut delay = Delay::new(
        unsafe { pac::CorePeripherals::steal().SYST },
        system_clock_freq.to_Hz(),
    );

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
        .map_or(LeptonFirmwareInfo::default(), |x| x);
    let LeptonFirmwareInfo {
        gpp_major,
        gpp_minor,
        gpp_build,
        dsp_major,
        dsp_minor,
        dsp_build,
    } = lepton_firmware_version;
    info!(
        "Camera firmware versions: main: {}.{}.{}, dsp: {}.{}.{}",
        gpp_major, gpp_minor, gpp_build, dsp_major, dsp_minor, dsp_build
    );
    info!("Camera serial #{}", lepton_serial);
    info!("Radiometry enabled? {}", radiometric_mode);

    let result = fifo.read_blocking();
    assert_eq!(result, Core0Task::ReadyToReceiveLeptonConfig.into());
    let main_lepton_firmware = LittleEndian::read_u32(&[gpp_major, gpp_minor, gpp_build, 0]);
    let dsp_lepton_firmware = LittleEndian::read_u32(&[dsp_major, dsp_minor, dsp_build, 0]);
    fifo.write_blocking(Core0Task::SendIntercoreArray.into());
    fifo.write_blocking(4);
    fifo.write_blocking(if radiometric_mode { 2 } else { 1 });
    fifo.write_blocking(lepton_serial);
    fifo.write_blocking(main_lepton_firmware);
    fifo.write_blocking(dsp_lepton_firmware);

    let result = fifo.read_blocking();
    if result == Core0Task::RequestReset.into() {
        restart(&mut watchdog);
    }
    // FIXME: Can we receive `Core0Task::HighPowerMode` here instead?
    assert_eq!(result, Core0Task::Ready.into());
    frame_acquisition_loop(
        rosc,
        &mut lepton,
        &mut fifo,
        peripheral_clock_freq,
        &mut delay,
        &mut peripherals.RESETS,
        static_frame_buffer_a,
        static_frame_buffer_b,
        watchdog,
    );
}
