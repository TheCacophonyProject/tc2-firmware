use crate::attiny_rtc_i2c::{tc2_agent_state, I2CConfig, RecordingType, SharedI2C};
use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::device_config::{AudioMode, DeviceConfig};
use crate::event_logger::{
    clear_audio_alarm, get_audio_alarm, write_audio_alarm, EventLogger, LoggerEvent,
    LoggerEventKind, WakeReason,
};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::frame_processing::{
    advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi, SyncedDateTime,
};
use crate::onboard_flash::OnboardFlash;
use crate::pdm_microphone::PdmMicrophone;
use crate::sub_tasks::offload_flash_storage_and_events;
use cortex_m::delay::Delay;
use defmt::{error, info, warn};
use rp2040_hal::Timer;

use chrono::{Datelike, NaiveDateTime, NaiveTime, Timelike};
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};

use cortex_m::asm::nop;
use picorand::{PicoRandGenerate, WyRand, RNG};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::{FunctionSio, PullDown, SioInput};
use rp2040_hal::pio::PIOExt;

#[repr(u8)]
#[derive(PartialEq, Eq)]

pub enum AlarmMode {
    AUDIO = 0,
    THERMAL = 1,
}

impl TryFrom<u8> for AlarmMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AlarmMode::*;

        match value {
            0 => Ok(AUDIO),
            1 => Ok(THERMAL),
            _ => Err(()),
        }
    }
}

const DEV_MODE: bool = false;
pub const MAX_GAP_MIN: u8 = 60;
pub fn audio_task(
    i2c_config: I2CConfig,
    clock_freq: u32,
    timer: &mut Timer,
    gpio0: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio0,
        rp2040_hal::gpio::FunctionNull,
        rp2040_hal::gpio::PullDown,
    >,
    gpio1: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio1,
        rp2040_hal::gpio::FunctionNull,
        rp2040_hal::gpio::PullDown,
    >,
    watchdog: &mut bsp::hal::Watchdog,
    alarm_triggered: bool,
    unlocked_pin: rp2040_hal::gpio::Pin<
        rp2040_hal::gpio::bank0::Gpio3,
        FunctionSio<SioInput>,
        PullDown,
    >,

    device_config: DeviceConfig,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    event_logger: &mut EventLogger,
    synced_date_time: &mut SyncedDateTime,
) -> ! {
    watchdog.feed();

    let core = unsafe { pac::CorePeripherals::steal() };
    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    watchdog.feed();

    let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut shared_i2c = SharedI2C::new(i2c_config, unlocked_pin, &mut delay);

    if alarm_triggered {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::Rp2040WokenByAlarm,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
    }

    let mut duration = 60;
    let mut recording_type = None;
    let mut user_recording_requested = false;
    let mut thermal_requested_audio = false;
    if alarm_triggered {
        recording_type = Some(RecordingType::ScheduledRecording);
    } else {
        if let Ok(audio_rec) = shared_i2c.tc2_agent_requested_audio_recording(&mut delay) {
            recording_type = audio_rec;

            if let Some(rec_type) = recording_type.as_mut() {
                match rec_type {
                    RecordingType::LongRecording => {
                        duration = 60 * 5;
                        user_recording_requested = true;
                    }
                    RecordingType::TestRecording => {
                        duration = 10;
                        user_recording_requested = true;
                    }
                    RecordingType::ThermalRequestedScheduledRecording => {
                        duration = 60;
                        thermal_requested_audio = true;
                    }
                    _ => {}
                }
            }
        }
    }

    let mut reschedule = false;
    let mut alarm_date_time: Option<NaiveDateTime> = None;

    if !user_recording_requested {
        //this isn't reliable so use alarm stored in flash
        // let mut alarm_hours = shared_i2c.get_alarm_hours();
        // let mut alarm_minutes = shared_i2c.get_alarm_minutes();
        let mut scheduled: bool = false;

        // GP 30th Jan TODO if alarm is set we always need to check alarm against current time if the alarm wasnt triggered
        // otherwise can have edge cases where tc2 agent status code thinks we should rec but the alarm is later
        let (_, flash_alarm) = get_audio_alarm(flash_storage);
        if let Some(alarm) = flash_alarm {
            scheduled = true;
            info!(
                "Audio alarm scheduled for {}-{}-{} {}:{}",
                alarm.year(),
                alarm.month(),
                alarm.day(),
                alarm.hour(),
                alarm.minute(),
            );
            alarm_date_time = Some(alarm);
        } else {
            error!("Not scheduled");
        }

        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::AudioMode,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
        // Unset the is_recording flag on attiny on startup
        if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
            if is_recording {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::RecordingNotFinished,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
            }
        }
        let _ = shared_i2c
            .set_recording_flag(&mut delay, false)
            .map_err(|e| error!("Error setting recording flag on attiny: {}", e));

        reschedule = !scheduled;

        watchdog.feed();

        let mut should_wake = false;
        match device_config.config().audio_mode {
            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                let in_window = device_config
                    .time_is_in_recording_window(&synced_date_time.get_adjusted_dt(timer), &None);
                if !in_window {
                    let is_cptv = flash_storage.has_cptv_files(true);
                    //this means end of thermal window so should offload recordings
                    if is_cptv {
                        info!("Has cptv files {}", is_cptv);
                        should_wake = true;
                        event_logger.log_event(
                            LoggerEvent::new(
                                LoggerEventKind::ToldRpiToWake(WakeReason::AudioThermalEnded),
                                synced_date_time.get_timestamp_micros(&timer),
                            ),
                            flash_storage,
                        );
                    }
                }
            }
            _ => {}
        }
        if !should_wake {
            should_wake = should_offload_audio_recordings(
                flash_storage,
                event_logger,
                &mut delay,
                &mut shared_i2c,
                synced_date_time.date_time_utc,
            );
            if should_wake {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::ToldRpiToWake(WakeReason::AudioShouldOffload),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
            }
        };

        match offload(
            &mut shared_i2c,
            clock_freq,
            flash_storage,
            pi_spi,
            timer,
            event_logger,
            should_wake,
            &mut delay,
            &synced_date_time,
            watchdog,
        ) {
            Ok(_) => (),
            Err(()) => {
                warn!("Restarting as could not offload");
                restart(watchdog);
            }
        }

        info!(
            "Alarm triggered {} scheduled {} thermal requested {}",
            alarm_triggered, scheduled, recording_type
        );

        if recording_type.is_none() && scheduled {
            // check we haven't missed the alarm somehow
            if let Some(alarm) = alarm_date_time {
                let synced = synced_date_time.get_adjusted_dt(timer);
                let until_alarm = (alarm - synced_date_time.get_adjusted_dt(timer)).num_minutes();
                if until_alarm <= 0 || until_alarm > MAX_GAP_MIN as i64 {
                    info!(
                        "Missed alarm was scheduled for the {} at {}:{} but its {} minutes away",
                        alarm.day(),
                        alarm.hour(),
                        alarm.minute(),
                        until_alarm
                    );

                    // should take recording now
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::Rp2040MissedAudioAlarm(
                                alarm.and_utc().timestamp_micros() as u64,
                            ),
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
                        flash_storage,
                    );
                    recording_type = Some(RecordingType::ScheduledRecording);
                }
            }
        }
    }
    if recording_type.is_some() {
        watchdog.feed();
        //should of already offloaded but extra safety check
        if !flash_storage.is_too_full_for_audio() {
            let _ = shared_i2c
                .set_recording_flag(&mut delay, true)
                .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
            let timestamp = synced_date_time.get_timestamp_micros(&timer);
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

            let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
            let mut microphone = PdmMicrophone::new(
                gpio0.into_function().into_pull_type(),
                gpio1.into_function().into_pull_type(),
                clock_freq.Hz(),
                pio1,
                sm1,
            );

            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::StartedAudioRecording,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );

            let recorded = microphone.record_for_n_seconds(
                duration,
                dma_channels.ch3,
                dma_channels.ch4,
                timer,
                &mut peripherals.RESETS,
                peripherals.SPI1,
                flash_storage,
                timestamp,
                watchdog,
                &synced_date_time,
            );
            let _ = shared_i2c
                .set_recording_flag(&mut delay, false)
                .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));

            if !recorded {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::AudioRecordingFailed,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
                info!("Recording failed restarting and will try again");
                restart(watchdog);
            } else {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::EndedRecording,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
                match recording_type.as_mut().unwrap() {
                    RecordingType::LongRecording | RecordingType::TestRecording => {
                        info!("taken test recoridng clearing status");
                        watchdog.feed();
                        let _ = shared_i2c.tc2_agent_clear_and_set_flag(
                            &mut delay,
                            match recording_type.unwrap() {
                                RecordingType::LongRecording => {
                                    tc2_agent_state::LONG_AUDIO_RECORDING
                                }
                                RecordingType::TestRecording => {
                                    tc2_agent_state::TEST_AUDIO_RECORDING
                                }
                                _ => 0,
                            },
                            if device_config.config().audio_mode != AudioMode::AudioOnly {
                                Some(tc2_agent_state::THERMAL_MODE)
                            } else {
                                None
                            },
                        );

                        let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

                        offload_flash_storage_and_events(
                            flash_storage,
                            pi_spi,
                            &mut peripherals.RESETS,
                            &mut peripherals.DMA,
                            clock_freq,
                            &mut shared_i2c,
                            &mut delay,
                            timer,
                            event_logger,
                            &synced_date_time,
                            Some(watchdog),
                            true,
                        );

                        restart(watchdog);
                    }
                    _ => {
                        shared_i2c.clear_alarm(&mut delay);
                        reschedule = true;
                        clear_audio_alarm(flash_storage);
                        if thermal_requested_audio {
                            //if audio requested from thermal, the alarm will be re scheduled there
                            let _ = shared_i2c.tc2_agent_clear_and_set_flag(
                                &mut delay,
                                tc2_agent_state::TAKE_AUDIO,
                                Some(tc2_agent_state::THERMAL_MODE),
                            );
                            info!("Audio taken in thermal window clearing flag");
                            restart(watchdog);
                        }
                    }
                }
            }
        }
    }
    let mut should_sleep = true;
    let mut alarm_time: Option<NaiveDateTime>;
    if reschedule {
        watchdog.feed();
        info!("Scheduling new recording");
        if let Ok(scheduled_time) = schedule_audio_rec(
            &mut delay,
            &synced_date_time,
            &mut shared_i2c,
            flash_storage,
            timer,
            event_logger,
            &device_config,
        ) {
            alarm_date_time = Some(scheduled_time);
        } else {
            error!("Couldn't schedule alarm will restart");
            clear_audio_alarm(flash_storage);
            restart(watchdog);
        }
    }
    if let Some(alarm) = alarm_date_time {
        info!(
            "Recording scheduled for the {} at {}:{} ",
            alarm.day(),
            alarm.time().hour(),
            alarm.time().minute()
        );
    }

    watchdog.start(8388607.micros());

    let mut logged_power_down = false;
    let boot_thermal = match device_config.config().audio_mode {
        AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => true,
        _ => false,
    };
    let in_window = device_config.config().audio_mode == AudioMode::AudioAndThermal
        && device_config
            .time_is_in_recording_window(&synced_date_time.get_adjusted_dt(timer), &None);

    loop {
        advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
        if !logged_power_down {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToSleep,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
            logged_power_down = true;
        }

        watchdog.feed();
        if should_sleep {
            if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(&mut delay, true) {
                if pi_is_powered_down {
                    if let Some(alarm_time) = alarm_date_time {
                        let until_alarm =
                            (alarm_time - synced_date_time.get_adjusted_dt(timer)).num_minutes();

                        if until_alarm < 1 {
                            // otherwise the alarm could trigger  between here and sleeping
                            should_sleep = false;
                            info!("Alarm is scheduled in {} so not sleeping", until_alarm);
                            if until_alarm <= 0 {
                                restart(watchdog);
                            }
                            continue;
                        }
                    }

                    if !in_window {
                        info!("Ask Attiny to power down rp2040");
                        event_logger.log_event(
                            LoggerEvent::new(
                                LoggerEventKind::Rp2040Sleep,
                                synced_date_time.get_timestamp_micros(&timer),
                            ),
                            flash_storage,
                        );

                        if let Ok(_) = shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay) {
                            info!("Sleeping");
                        } else {
                            error!("Failed sending sleep request to attiny");
                        }
                    }
                }

                //may aswell offload again if we are awake and have just taken a recording
                if !pi_is_powered_down && flash_storage.has_files_to_offload() {
                    match offload(
                        &mut shared_i2c,
                        clock_freq,
                        flash_storage,
                        pi_spi,
                        timer,
                        event_logger,
                        false,
                        &mut delay,
                        &synced_date_time,
                        watchdog,
                    ) {
                        Ok(_) => (),
                        Err(()) => {
                            warn!("Restarting as failed to offload");
                            restart(watchdog);
                        }
                    }
                }
            }
        }
        if boot_thermal && should_sleep {
            //if we have done everything and PI is still on go into thermal for preview
            if let Ok(()) = shared_i2c.tc2_agent_request_thermal_mode(&mut delay) {
                info!("Going into thermal mode");
                restart(watchdog);
            }
        }

        let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
        if alarm_triggered {
            info!("Alarm triggered after taking a recording reseeting rp2040");
            restart(watchdog);
        }

        delay.delay_ms(5 * 1000);
        watchdog.feed();
    }
}

fn restart(watchdog: &mut bsp::hal::Watchdog) {
    watchdog.start(100.micros());
    loop {
        nop();
    }
}

pub fn offload(
    i2c: &mut SharedI2C,
    clock_freq: u32,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
    mut wake_if_asleep: bool,
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    watchdog: &mut bsp::hal::Watchdog,
) -> Result<(), ()> {
    if !wake_if_asleep {
        if let Ok(pi_waking) = i2c.pi_is_waking_or_awake(delay) {
            wake_if_asleep = pi_waking;
        }
    }
    if let Ok(mut awake) = i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
        if !awake && wake_if_asleep {
            watchdog.disable();
            wake_raspberry_pi(i2c, delay);
            awake = true;
            watchdog.start(8388607.micros());
        }
        if awake {
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
            if !offload_flash_storage_and_events(
                flash_storage,
                pi_spi,
                &mut peripherals.RESETS,
                &mut peripherals.DMA,
                clock_freq,
                i2c,
                delay,
                timer,
                event_logger,
                &synced_date_time,
                Some(watchdog),
                false,
            ) && flash_storage.has_files_to_offload()
            {
                return Err(());
            }
        }
    }
    Ok(())
}

pub fn schedule_audio_rec(
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    i2c: &mut SharedI2C,
    flash_storage: &mut OnboardFlash,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
    device_config: &DeviceConfig,
) -> Result<NaiveDateTime, ()> {
    if let Err(err) = i2c.disable_alarm(delay) {
        error!("Failed to disable alarm");
        return Err(());
    }
    let mut wakeup: NaiveDateTime;
    let seed: u64;
    let current_time = synced_date_time.get_adjusted_dt(timer);

    if device_config.config().audio_seed > 0 {
        wakeup = NaiveDateTime::new(
            current_time.date(),
            NaiveTime::from_hms_opt(0, 0, 0).unwrap(),
        );

        seed = u64::wrapping_add(
            wakeup.and_utc().timestamp_millis() as u64,
            device_config.config().audio_seed as u64,
        );
    } else {
        seed = synced_date_time.date_time_utc.and_utc().timestamp() as u64;
        wakeup = current_time;
    };
    let mut rng = RNG::<WyRand, u16>::new(seed);
    let r_max: u16 = 65535u16;
    let short_chance: u16 = r_max / 4;
    let short_pause: u64 = 2 * 60;
    let short_window: u64 = 5 * 60;
    let long_pause: u64 = 40 * 60;
    let long_window: u64 = 20 * 60;

    // if a seed is set always start alarm from 0am to keep consistent accross devices.So  will need to generate numbers until alarm is valid
    while wakeup <= current_time {
        let r = rng.generate();
        let mut wake_in;

        if r <= short_chance {
            wake_in = (short_pause + (r as u64 * short_window) / short_chance as u64) as u64;
        } else {
            wake_in = (long_pause + (r as u64 * long_window) / r_max as u64) as u64;
        }

        if DEV_MODE {
            wake_in = 120;
        }
        wakeup = wakeup + chrono::Duration::seconds(wake_in as i64);
    }
    info!(
        "Set alarm, current time is {}:{} Next alarm is {}:{}",
        synced_date_time.date_time_utc.time().hour(),
        synced_date_time.date_time_utc.time().minute(),
        wakeup.hour(),
        wakeup.minute()
    );

    let mut alarm_mode = AlarmMode::AUDIO;

    match device_config.config().audio_mode {
        AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
            let (start, end) = device_config.next_or_current_recording_window(&current_time);
            info!(
                "Checking next alarm {}:{} for rec window start{}:{}",
                wakeup.hour(),
                wakeup.minute(),
                start.hour(),
                start.minute(),
            );
            if wakeup >= start {
                if start < current_time {
                    if let AudioMode::AudioAndThermal = device_config.config().audio_mode {
                        //audio recording inside recording window
                        info!("Scheudling audio inside thermal window");
                    } else {
                        info!("Already in rec window so restart now");
                        return Err(());
                    }
                } else {
                    info!("Setting wake up to be start of next rec window");
                    wakeup = start;
                    alarm_mode = AlarmMode::THERMAL;
                }
            }
        }
        _ => (),
    }

    if let Err(err) = i2c.enable_alarm(delay) {
        error!("Failed to enable alarm");
        return Err(());
    }
    if let Ok(_) = i2c.set_wakeup_alarm(&wakeup, delay) {
        if let Ok(alarm_enabled) = i2c.alarm_interrupt_enabled(delay) {
            if alarm_enabled {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::SetAlarm(wakeup.and_utc().timestamp_micros() as u64),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );

                write_audio_alarm(flash_storage, wakeup, alarm_mode);
                return Ok(wakeup);
            }
        }
    } else {
        error!("Failed setting wake alarm, can't go to sleep");
    }
    return Err(());
}

fn should_offload_audio_recordings(
    flash_storage: &mut OnboardFlash,
    event_logger: &mut EventLogger,
    delay: &mut Delay,
    i2c: &mut SharedI2C,
    now: NaiveDateTime,
) -> bool {
    let has_files = flash_storage.has_files_to_offload() || event_logger.is_nearly_full();
    if !has_files {
        return false;
    }
    // flash getting full
    if flash_storage.is_too_full_for_audio() {
        info!("Offloading as flash is nearly full");
        return true;
    }

    // probably never happens if functioning correctly
    if event_logger.is_nearly_full() {
        info!("Offloading as logger is nearly full");
        return true;
    }
    if flash_storage.file_start_block_index.is_none() {
        //one off
        info!("Offloading as previous file system version");
        return true;
    }

    return false;
}

pub fn check_alarm_still_valid_with_thermal_window(
    alarm: &NaiveDateTime,
    now: &NaiveDateTime,
    device_config: &DeviceConfig,
) -> bool {
    // config has changed so check if not in audio only that alarm is still going to trigger on rec window start
    if device_config.config().audio_mode != AudioMode::AudioOnly {
        let (start, end) = device_config.next_or_current_recording_window(now);
        //alarm before start or we are in rec window
        return alarm <= &start || now >= &start;
    }
    true
}
