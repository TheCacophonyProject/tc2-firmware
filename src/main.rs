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
mod motion_detector;
mod onboard_flash;
mod pdm_filter;
mod pdm_microphone;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod synced_date_time;
mod utils;

use crate::attiny_rtc_i2c::SharedI2C;
use crate::audio_task::{
    AlarmMode, MAX_GAP_MIN, audio_task, check_alarm_still_valid_with_thermal_window,
    schedule_audio_rec,
};
use crate::device_config::AudioMode;
use crate::event_logger::{
    EventLogger, LoggerEvent, LoggerEventKind, WakeReason, clear_audio_alarm, get_audio_alarm,
};
use crate::ext_spi_transfers::{
    ExtSpiTransfers, RPI_PAYLOAD_LENGTH, RPI_RETURN_PAYLOAD_LENGTH, RPI_TRANSFER_HEADER_LENGTH,
};
use crate::frame_processing::{
    Core0Task, FrameBuffer, StaticFrameBuffer, thermal_motion_task, wake_raspberry_pi,
};
use crate::lepton::{LeptonFirmwareInfo, LeptonPins, init_lepton_module};
pub use crate::lepton_task::frame_acquisition_loop;
use crate::onboard_flash::OnboardFlash;
use crate::onboard_flash::extend_lifetime_generic_mut;
use crate::onboard_flash::{FLASH_SPI_TOTAL_PAYLOAD_SIZE, extend_lifetime_generic};
use crate::startup_functions::{get_device_config, get_synced_time, should_record_audio};
use crate::sub_tasks::{maybe_offload_events, offload_flash_storage_and_events};
use crate::synced_date_time::SyncedDateTime;
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
use chrono::{DateTime, Datelike, Duration, Timelike, Utc};
use core::cell::RefCell;
use cortex_m::asm::nop;
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

fn get_next_audio_alarm(
    device_config: &DeviceConfig,
    flash_storage: &mut OnboardFlash,
    synced_date_time: &SyncedDateTime,
    delay: &mut Delay,
    shared_i2c: &mut SharedI2C,
    event_logger: &mut EventLogger,
) -> Option<DateTime<Utc>> {
    // FIXME: Shouldn't this have already happened on startup?
    let mut next_audio_alarm = None;
    match device_config.config().audio_mode {
        AudioMode::AudioOrThermal | AudioMode::AudioAndThermal => {
            let (audio_mode, audio_alarm) = get_audio_alarm(flash_storage);
            let mut schedule_alarm = true;
            // if audio alarm is set check it's within 60 minutes and before or on thermal window start
            if let Some(audio_alarm) = audio_alarm {
                if let Ok(audio_mode) = audio_mode {
                    // FIXME: Document what the AlarmModes mean
                    if audio_mode == AlarmMode::Audio {
                        let synced = synced_date_time.get_date_time();
                        let until_alarm = (audio_alarm - synced).num_minutes();
                        if until_alarm <= i64::from(MAX_GAP_MIN) {
                            info!(
                                "Audio alarm already scheduled for {}-{}-{} {}:{}",
                                audio_alarm.year(),
                                audio_alarm.month(),
                                audio_alarm.day(),
                                audio_alarm.hour(),
                                audio_alarm.minute()
                            );
                            if check_alarm_still_valid_with_thermal_window(
                                &audio_alarm,
                                &synced,
                                device_config,
                            ) {
                                next_audio_alarm = Some(audio_alarm);
                                schedule_alarm = false;
                            } else {
                                schedule_alarm = true;
                                // if window time changed and alarm is after rec window start
                                info!("Rescheduling as alarm is after window start");
                            }
                        } else {
                            info!("Alarm is missed");
                        }
                    }
                }
            }
            if schedule_alarm {
                if let Ok(next_alarm) = schedule_audio_rec(
                    delay,
                    synced_date_time,
                    shared_i2c,
                    flash_storage,
                    event_logger,
                    device_config,
                ) {
                    next_audio_alarm = Some(next_alarm);
                    info!("Setting a pending audio alarm");
                } else {
                    error!("Couldn't schedule alarm");
                }
            }
        }
        _ => {
            info!("Clearing audio alarm");
            clear_audio_alarm(flash_storage);
        }
    }
    next_audio_alarm
}

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

    info!("System clock speed {}MHz", clocks.system_clock.freq().to_MHz());

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
    let mut flash_storage = {
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

    flash_storage.take_spi(peripherals.SPI1, &mut peripherals.RESETS);
    flash_storage.init();

    let mut delay =
        Delay::new(pac::CorePeripherals::take().unwrap().SYST, system_clock_freq.to_Hz());

    // Attiny + RTC comms
    info!("Initing shared i2c");
    // Early on in development we got strange errors when the raspberry pi was accessing the
    // attiny-provided i2c interface at the same time as we wanted to.  The hacky?/ingenious?
    // solution was to allocate a gpio pin that would determine who has the 'lock' on the i2c bus.
    // This is handled by this `SharedI2C` abstraction which mediates comms with the attiny.
    let mut shared_i2c = SharedI2C::new(
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
    let mut event_logger = EventLogger::new(&mut flash_storage);
    let synced_date_time =
        get_synced_time(&mut shared_i2c, &mut delay, &mut event_logger, &mut flash_storage, timer);
    let device_config = get_device_config(
        &mut flash_storage,
        &mut shared_i2c,
        &mut delay,
        &mut pi_spi,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
    );

    let woken_by_alarm = shared_i2c.alarm_triggered(&mut delay);
    if woken_by_alarm {
        info!("Woken by RTC alarm? {}", woken_by_alarm);
        event_logger.log_event(
            LoggerEvent::new(LoggerEventKind::Rp2040WokenByAlarm, &synced_date_time),
            &mut flash_storage,
        );
        shared_i2c.clear_alarm(&mut delay);
    }

    // Do we want to offload files?

    let next_audio_alarm = get_next_audio_alarm(
        &device_config,
        &mut flash_storage,
        &synced_date_time,
        &mut delay,
        &mut shared_i2c,
        &mut event_logger,
    );
    // Check if next_audio_alarm < now?

    let record_audio_now =
        should_record_audio(&device_config, &mut shared_i2c, &mut delay, &synced_date_time);
    // TODO: Maybe check sun_times for valid lat/lng which can give us a recording window,
    //  log if we can't get one

    // NOTE: We'll only wake the pi if we have files to offload, and it is *outside* the recording
    //  window, or the previous offload happened more than 24 hours ago, or the flash is nearly full.
    //  Otherwise, if the rp2040 happens to restart, we'll pretty much
    //  always start the pi up, which we don't want.
    let has_files_to_offload = flash_storage.has_files_to_offload();
    let should_offload = (has_files_to_offload
        && !device_config.time_is_in_recording_window(&synced_date_time.get_date_time(), None))
        || flash_storage.is_too_full_to_start_new_recordings()
        || (has_files_to_offload && flash_storage.file_start_block_index.is_none());
    // means old file system offload once

    if should_offload {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::ToldRpiToWake(WakeReason::ThermalOffload),
                &synced_date_time,
            ),
            &mut flash_storage,
        );
    }
    let should_offload = if !should_offload && has_files_to_offload {
        let duration_since_prev_offload = event_logger
            .latest_event_of_kind(LoggerEventKind::OffloadedRecording, &mut flash_storage)
            .map_or(Duration::minutes(0), |prev_event| {
                prev_event.timestamp().map_or(Duration::minutes(0), |date_time_utc| {
                    synced_date_time.get_date_time() - date_time_utc
                })
            });
        if duration_since_prev_offload > Duration::hours(24) {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToWake(WakeReason::ThermalOffloadAfter24Hours),
                    &synced_date_time,
                ),
                &mut flash_storage,
            );
            true
        } else {
            false
        }
    } else {
        should_offload
    };

    // FIXME: This should all happen in main startup
    let did_offload_files = if should_offload {
        offload_flash_storage_and_events(
            &mut flash_storage,
            &mut pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            &mut shared_i2c,
            &mut delay,
            &mut event_logger,
            &synced_date_time,
            &mut watchdog,
            false,
        )
    } else {
        false
    };

    // TODO: We might defer this if the user is using sidekick etc?
    if record_audio_now {
        let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
        audio_task(
            shared_i2c,
            system_clock_freq,
            pins.gpio0,
            pins.gpio1,
            watchdog,
            woken_by_alarm,
            &device_config,
            flash_storage,
            pi_spi,
            event_logger,
            &synced_date_time,
            pio1,
            sm1,
        );
    } else {
        let disabled_alarm = shared_i2c.disable_alarm(&mut delay);
        if disabled_alarm.is_err() {
            error!("{}", disabled_alarm.unwrap());
        }

        if !device_config.use_low_power_mode() {
            // TODO: Do we want to do this in both branches, assuming the pi is awake?
            if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::ToldRpiToWake(WakeReason::ThermalHighPower),
                        &synced_date_time,
                    ),
                    &mut flash_storage,
                );
            }
            maybe_offload_events(
                &mut pi_spi,
                &mut peripherals.RESETS,
                &mut peripherals.DMA,
                &mut delay,
                &mut event_logger,
                &mut flash_storage,
                &synced_date_time,
                &mut watchdog,
            );
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
            shared_i2c,
            pi_spi,
            flash_storage,
            lepton_pins,
            watchdog,
            system_clock_freq,
            delay,
            &clocks,
            rosc,
            &device_config,
            event_logger,
            synced_date_time,
            next_audio_alarm,
        );
    }
}

/// # Panics
///
/// TODO
pub fn thermal_code(
    shared_i2c: SharedI2C,
    pi_spi: ExtSpiTransfers,
    onboard_flash: OnboardFlash,
    lepton_pins: LeptonPins,
    watchdog: Watchdog,
    system_clock_freq: HertzU32,
    delay: Delay,
    clocks: &ClocksManager,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    device_config: &DeviceConfig,
    event_logger: EventLogger,
    synced_date_time: SyncedDateTime,
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

    let frame_buffer_a =
        Mutex::new(RefCell::new(Some(unsafe { extend_lifetime_generic_mut(&mut fb0) })));
    let frame_buffer_b =
        Mutex::new(RefCell::new(Some(unsafe { extend_lifetime_generic_mut(&mut fb1) })));
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
        shared_i2c,
        pi_spi,
        onboard_flash,
        static_frame_buffer_a,
        static_frame_buffer_b,
        device_config,
        event_logger,
        synced_date_time,
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
    let mut delay =
        Delay::new(unsafe { pac::CorePeripherals::steal().SYST }, system_clock_freq.to_Hz());

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
    let lepton_firmware_version =
        lepton.get_firmware_version().map_or(LeptonFirmwareInfo::default(), |x| x);
    let LeptonFirmwareInfo { gpp_major, gpp_minor, gpp_build, dsp_major, dsp_minor, dsp_build } =
        lepton_firmware_version;
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
        &mut fifo,
        peripheral_clock_freq,
        &mut delay,
        &mut peripherals.RESETS,
        static_frame_buffer_a,
        static_frame_buffer_b,
        watchdog,
    );
}
