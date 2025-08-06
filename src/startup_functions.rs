use crate::attiny_rtc_i2c::{
    AlarmMode, CameraState, MainI2C, RecordingMode, RecordingRequestType, ScheduledAlarmTime,
    Tc2AgentState,
};
use crate::audio_task::AUDIO_DEV_MODE;
use crate::bsp::pac::RESETS;
use crate::device_config::{AudioMode, DeviceConfig};
use crate::event_logger::{Event, EventLogger, LoggerEvent, NewConfigInfo, WakeReason};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::frame_processing::THERMAL_DEV_MODE;
use crate::onboard_flash::{FileType, OnboardFlash};
use crate::rpi_power::wake_raspberry_pi;
use crate::sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    offload_all_recordings_and_events, offload_latest_recording,
};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use chrono::{DateTime, Datelike, Duration, NaiveTime, Timelike, Utc};
use cortex_m::prelude::*;
use defmt::{error, info, warn};
use picorand::{PicoRandGenerate, RNG, WyRand};
use rp2040_hal::pac::DMA;
use rp2040_hal::{Timer, Watchdog};

pub fn get_device_config(
    fs: &mut OnboardFlash,
    i2c: &mut MainI2C,
    timer: Timer,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    watchdog: &mut Watchdog,
    events: &mut EventLogger,
    time: &SyncedDateTime,
) -> Result<(DeviceConfig, bool, bool, bool), ()> {
    let event_count = events.count();
    let device_config: Option<DeviceConfig> = DeviceConfig::load_existing_config_from_flash(fs);
    if let Some(device_config) = &device_config {
        info!("Existing config {:#?}", device_config.config());
    } else {
        info!("Waking pi to get config");
        // We need to wake up the rpi and get a config
        wake_raspberry_pi(i2c, timer, Some(watchdog), None);
    }
    let (device_config, device_config_was_updated, prioritise_frame_preview, force_offload_now) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            fs,
            pi_spi,
            resets,
            dma,
            device_config,
            event_count,
        );
    if let Some(device_config) = device_config {
        if device_config_was_updated {
            events.log(
                Event::Rp2040GotNewConfig(NewConfigInfo::from_config(&device_config)),
                time,
                fs,
            );
        }

        Ok((
            device_config,
            prioritise_frame_preview,
            device_config_was_updated,
            force_offload_now,
        ))
    } else {
        error!("Couldn't get config");
        Err(())
    }
}

pub fn get_synced_time(
    i2c: &mut MainI2C,
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    watchdog: &mut Watchdog,
    mut timer: Timer,
) -> Result<SyncedDateTime, &'static str> {
    info!("Getting synced time");
    // NOTE: I2C communication from the rp2040 to the RTC unit is not perfect – sometimes
    //  a bit gets flipped when reading the time, and we get a wrong date.
    //  When we get the initial time on startup, we sample it a bunch of times to try and reduce
    //  the likelihood of this error.
    let mut prev_time: Option<SyncedDateTime> = None;
    let mut test_iterations = 0;
    loop {
        let time = i2c.get_datetime(timer, false);
        match time {
            Err(e) => {
                if e == "Time integrity compromised" {
                    // Maybe wake up pi, and ask for RTC to be set?
                    info!("Should wake pi because {:?}", e);
                    let _wait_to_wake = wake_raspberry_pi(
                        i2c,
                        timer,
                        Some(watchdog),
                        Some((
                            fs,
                            events,
                            LoggerEvent::new_with_unknown_time(Event::ToldRpiToWake(
                                WakeReason::RtcTimeCompromised,
                            )),
                        )),
                    );

                    loop {
                        watchdog.feed();
                        // Now block here until we can get a good date with the lv flag unset.
                        let time = get_synced_time(i2c, events, fs, watchdog, timer);
                        if time.is_ok() {
                            return time;
                        }
                    }
                } else {
                    // can't get time so use 0 and add a time when tc2-agent uploads
                    events.log_event(LoggerEvent::new_with_unknown_time(Event::RtcCommError), fs);
                    error!("{}", e);
                    restart(watchdog);
                }
            }
            Ok(time) => {
                if let Some(prev_time) = &prev_time {
                    let prev_date_time = prev_time.date_time_utc;
                    let this_date_time = time.date_time_utc;

                    if this_date_time < prev_date_time
                        || prev_date_time.year() != this_date_time.year()
                        || prev_date_time.month() != this_date_time.month()
                        || prev_date_time.day() != this_date_time.day()
                        || prev_date_time.hour() != this_date_time.hour()
                        || prev_date_time.minute() != this_date_time.minute()
                    {
                        test_iterations = 0;
                    } else {
                        test_iterations += 1;
                    }
                }
                prev_time = Some(time);
            }
        }
        timer.delay_ms(1);
        if test_iterations >= 10 {
            return time;
        }
    }
}

// Make sure the alarm exists, otherwise reschedule.
#[allow(clippy::ref_option)]
pub fn validate_scheduled_alarm(
    config: &DeviceConfig,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
    i2c: &mut MainI2C,
    events: &mut EventLogger,
    scheduled_alarm: &Option<ScheduledAlarmTime>,
    watchdog: &mut Watchdog,
) {
    if scheduled_alarm.is_none() {
        warn!("Scheduled alarm is None, rescheduling");
        // We should always have a next alarm: schedule new alarm and restart
        match schedule_next_recording(time, i2c, fs, events, config) {
            Ok(next_alarm) => {
                info!("Setting a pending recording alarm: {:?}", next_alarm);
            }
            Err(e) => {
                error!("Couldn't schedule alarm: {}", e);
            }
        }
        restart(watchdog);
    }

    let scheduled_alarm = scheduled_alarm.as_ref().unwrap();
    if scheduled_alarm.has_triggered() {
        info!("Scheduled alarm triggered, scheduling next alarm");
        match schedule_next_recording(time, i2c, fs, events, config) {
            Ok(next_alarm) => {
                info!("Setting a pending recording alarm: {:?}", next_alarm);
            }
            Err(e) => {
                error!("Couldn't schedule alarm: {}", e);
                restart(watchdog);
            }
        }
    }

    if scheduled_alarm.is_audio_alarm() && scheduled_alarm.has_triggered() {
        let alarm_time = scheduled_alarm.date_time();
        let until_alarm = alarm_time - time.date_time();
        if until_alarm.num_minutes() <= -1 {
            warn!(
                "Missed alarm was scheduled for {} minutes ago",
                -until_alarm.num_minutes()
            );
            events.log(
                Event::Rp2040MissedAudioAlarm(alarm_time.timestamp_micros()),
                time,
                fs,
            );
        }
    }
}

#[allow(clippy::too_many_lines)]
pub fn maybe_offload_files_and_events_on_startup(
    recording_mode: RecordingMode,
    prioritise_frame_preview: bool,
    force_offload_now: bool,
    fs: &mut OnboardFlash,
    config: &DeviceConfig,
    time: &SyncedDateTime,
    events: &mut EventLogger,
    resets: &mut RESETS,
    dma: &mut DMA,
    i2c: &mut MainI2C,
    pi_spi: &mut ExtSpiTransfers,
    watchdog: &mut Watchdog,
) {
    /*
    There are a number of reasons we may or may not want to offload files and events on startup.
    If the rPi is already awake, we want to offload
    - if the flash is full/too full to record with the current recording mode
    - if we're not in a recording window, or it has been more than 24hrs since the previous offload.
    - if an audio recording is not scheduled imminently?
    - AND the user is not interacting with the device via sidekick.

    Maybe offload files and events:
    Offloads can be interrupted by a user via tc2-agent, and can also be deferred as much as
    possible (also via tc2-agent) if there is a user interacting with side-kick, in which
    case we try to prioritise getting frames sent.
     */

    let has_files_to_offload = fs.has_recordings_to_offload();
    if has_files_to_offload {
        let blocks_used = fs.first_used_block_index.unwrap()..fs.last_used_block_index.unwrap();
        let num_blocks_used = blocks_used.len() + 1;
        #[allow(clippy::cast_precision_loss)]
        let space_used = (num_blocks_used * 2048 * 64) as f32 / 1024.0 / 1024.0;
        info!(
            "{} files found. Flash blocks used {:?} space used {}MB",
            fs.num_files_in_initial_scan, blocks_used, space_used
        );
    }
    let has_events_to_offload = events.has_events_to_offload();
    if has_events_to_offload {
        info!("There are {} event(s) to offload", events.count());
    }
    let last_recording_was_user_requested = fs.last_recording_was_user_requested(watchdog);
    if last_recording_was_user_requested {
        info!("Previous recording was user requested.",);
    }
    let rpi_is_awake = i2c
        .get_camera_state()
        .is_ok_and(CameraState::pi_is_waking_or_awake)
        && i2c
            .get_tc2_agent_state()
            .is_ok_and(|state| state.is_ready());

    if rpi_is_awake {
        info!("rPi is awake and ready");
    }

    let is_inside_thermal_recording_window =
        config.time_is_in_configured_recording_window(&time.date_time());

    if is_inside_thermal_recording_window {
        info!("Inside recording window.",);
    } else {
        info!("Outside recording window",);
    }

    let is_outside_thermal_recording_window = !is_inside_thermal_recording_window;
    let is_too_full_to_record_in_current_mode = match recording_mode {
        RecordingMode::Audio(mode) => fs.is_too_full_to_start_new_audio_recordings(&mode),
        RecordingMode::Thermal(_) => fs.is_too_full_to_start_new_cptv_recordings(),
        RecordingMode::None => false,
    };

    if is_too_full_to_record_in_current_mode {
        warn!("Too full to record in current mode");
    }

    let mut offload_wake_reason = WakeReason::Unknown;
    let mut should_offload = false;

    if last_recording_was_user_requested {
        if !rpi_is_awake {
            warn!("Last recording was user requested, but rPi is somehow not awake");
        }
        info!("Last recording was user requested, offload immediately");
    }

    let end_of_thermal_window = fs.file_types_found.has_type(FileType::CptvShutdown)
        && !last_recording_was_user_requested
        && is_outside_thermal_recording_window;

    if end_of_thermal_window {
        should_offload = true;
        if recording_mode.record_thermal() {
            offload_wake_reason = WakeReason::ThermalOffload;
        } else {
            offload_wake_reason = WakeReason::AudioThermalEnded;
        }
    }
    if is_too_full_to_record_in_current_mode {
        should_offload = true;
        if recording_mode.record_thermal() {
            offload_wake_reason = WakeReason::ThermalTooFull;
        } else {
            offload_wake_reason = WakeReason::AudioTooFull;
        }
    }
    if events.is_nearly_full() {
        should_offload = true;
        offload_wake_reason = WakeReason::EventsTooFull;
    }
    if last_recording_was_user_requested {
        should_offload = true;
        offload_wake_reason = WakeReason::OffloadTestRecording;
    }
    if rpi_is_awake
        && recording_mode.record_audio()
        && (AUDIO_DEV_MODE || config.use_high_power_mode())
    {
        // Opportunistically offload recordings anyway,
        // since the rpi is on (maybe in high power mode)
        offload_wake_reason = WakeReason::OpportunisticOffload;
        should_offload = true;
    }
    if !should_offload && has_events_to_offload && force_offload_now {
        should_offload = true;
        offload_wake_reason = WakeReason::OffloadOnUserDemand;
    }
    let should_offload = if !should_offload && has_files_to_offload {
        offload_wake_reason = WakeReason::ThermalOffloadAfter24Hours;
        // If in high-power thermal mode, always offload when there are files.
        config.use_high_power_mode()
            || duration_since_prev_offload_greater_than_23hrs(events, fs, time)
    } else {
        should_offload
    };

    // Regardless of whether we need forced offload, try to honour `prioritise_frame_preview`.
    // If the rpi is awake, offload unless it would prefer us not to.
    if should_offload {
        // NOTE: We'll only wake the pi if we have files to offload, and it is *outside* the recording
        //  window, or the previous offload happened more than 24 hours ago, or the flash is nearly full.
        //  Otherwise, if the rp2040 happens to restart, we'll pretty much
        //  always start the pi up, which we don't want.
        if recording_mode.record_thermal() && config.use_high_power_mode() {
            offload_wake_reason = WakeReason::ThermalHighPower;
        }
        info!("Should wake pi because {:?}", offload_wake_reason);
        let _wait_to_wake = wake_raspberry_pi(
            i2c,
            time.get_timer(),
            Some(watchdog),
            Some((
                fs,
                events,
                LoggerEvent::new(Event::ToldRpiToWake(offload_wake_reason), time),
            )),
        );
        if last_recording_was_user_requested {
            if !offload_latest_recording(fs, pi_spi, resets, dma, i2c, events, time, watchdog) {
                // Offload failed, restart and try again.
                error!("File offload failed, restarting");
                restart(watchdog);
            }
        } else if prioritise_frame_preview {
            // Just offload the bare minimum of files to free up space, and all events if needed.
            if events.is_nearly_full() {
                maybe_offload_events(pi_spi, resets, dma, events, fs, time, watchdog);
            }
            let mut is_too_full = is_too_full_to_record_in_current_mode;
            while is_too_full {
                if offload_latest_recording(fs, pi_spi, resets, dma, i2c, events, time, watchdog) {
                    is_too_full = match recording_mode {
                        RecordingMode::Audio(mode) => {
                            fs.is_too_full_to_start_new_audio_recordings(&mode)
                        }
                        RecordingMode::Thermal(_) => fs.is_too_full_to_start_new_cptv_recordings(),
                        RecordingMode::None => false,
                    };
                } else {
                    // Offload failed, restart and try again.
                    error!("File offload failed, restarting");
                    restart(watchdog);
                }
            }
        } else {
            // Offload everything
            if !offload_all_recordings_and_events(
                fs, pi_spi, resets, dma, i2c, events, time, watchdog,
            ) {
                // Failed to offload, restart and try again
                error!("File offload failed, restarting");
                restart(watchdog);
            }
        }
    }
}

fn duration_since_prev_offload_greater_than_23hrs(
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
) -> bool {
    let duration_since_prev_offload = events
        .latest_event_of_kind(Event::OffloadedRecording(FileType::Unknown), fs)
        .map_or(Duration::minutes(0), |prev_event| {
            prev_event
                .timestamp()
                .map_or(Duration::minutes(0), |date_time_utc| {
                    time.date_time() - date_time_utc
                })
        });
    duration_since_prev_offload > Duration::hours(23)
}

#[allow(clippy::too_many_lines)]
pub fn schedule_next_recording(
    time: &SyncedDateTime,
    i2c: &mut MainI2C,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    config: &DeviceConfig,
) -> Result<ScheduledAlarmTime, &'static str> {
    let audio_mode = config.audio_mode();
    let current_time = time.date_time();
    let mut wakeup: DateTime<Utc>;
    let mut alarm_mode = AlarmMode::Audio;
    if audio_mode == AudioMode::Disabled
        || (audio_mode == AudioMode::AudioOrThermal && config.is_continuous_recorder())
    {
        let current_window = config.next_or_current_recording_window(&current_time)?;
        alarm_mode = AlarmMode::Thermal;
        wakeup = if config.time_is_in_supplied_recording_window(&current_time, current_window) {
            // In the window
            config
                .next_recording_window_start(&(current_time + Duration::hours(24)))
                .expect("Invalid next recording window")
        } else {
            info!("Setting wake up to be start of next thermal recording window");
            assert!(
                current_time < current_window.0,
                "Time should be before next window"
            );
            assert!(
                current_window.0 < current_time + Duration::minutes((24 * 60) + 1),
                "Next window should be no more than 24hrs from now"
            );
            current_window.0
        };
        if THERMAL_DEV_MODE {
            wakeup = current_time + Duration::minutes(3);
        }
    } else {
        let seed: u64;
        let current_time = time.date_time();

        if config.audio_seed() > 0 {
            wakeup = if let Some(time) = current_time
                .with_time(NaiveTime::from_hms_opt(0, 0, 0).unwrap())
                .latest()
            {
                time
            } else {
                return Err("Failed to clamp current time to midnight");
            };
            let ts_millis = u64::try_from(wakeup.timestamp_millis())
                .map_err(|_| "Failed to convert wakeup timestamp milliseconds to u64")?;
            seed = u64::wrapping_add(ts_millis, u64::from(config.config().audio_seed));
        } else {
            let ts_seconds = u64::try_from(current_time.timestamp())
                .map_err(|_| "Failed to convert current_time timestamp to u64")?;

            seed = ts_seconds;
            wakeup = current_time;
        }
        let mut rng = RNG::<WyRand, u16>::new(seed);
        let r_max: u16 = u16::MAX;
        // Tuned to average 32 recordings per day with a ratio of 1:3 short to long pauses.
        let short_chance: u16 = r_max / 4;
        let short_pause: i64 = 3 * 59;
        let short_window: i64 = 5 * 59;
        let long_pause: i64 = 43 * 59;
        let long_window: i64 = 25 * 60;

        // If a seed is set, always start alarm from 0am to keep consistent across devices.
        // So will need to generate numbers until the alarm is valid
        while wakeup <= current_time {
            let r = rng.generate();
            let wake_in = if AUDIO_DEV_MODE {
                120
            } else if r <= short_chance {
                short_pause + (i64::from(r) * short_window) / i64::from(short_chance)
            } else {
                long_pause + (i64::from(r) * long_window) / i64::from(r_max)
            };
            wakeup += Duration::seconds(wake_in);
        }
        if config.records_audio_and_thermal() {
            if let Ok((start, end)) = config.next_or_current_recording_window(&current_time) {
                info!(
                    "Checking next alarm {}:{} for rec window start {}:{}",
                    wakeup.hour(),
                    wakeup.minute(),
                    start.hour(),
                    start.minute(),
                );
                if wakeup >= start {
                    if start < current_time {
                        info!("Audio mode {}", audio_mode);
                        if audio_mode == AudioMode::AudioAndThermal {
                            // audio recording inside recording window
                            info!("Scheduling audio inside thermal window");
                        } else if audio_mode == AudioMode::AudioOrThermal {
                            // AudioOrThermal mode. Append audio wakeup to end of recording window
                            // when scheduling inside the current thermal window.
                            wakeup = end + (wakeup - current_time);
                            alarm_mode = AlarmMode::Audio;
                        }
                    } else {
                        info!("Setting wake up to be start of next thermal recording window");
                        if THERMAL_DEV_MODE {
                            wakeup = current_time + Duration::minutes(3);
                        } else {
                            wakeup = start;
                        }
                        alarm_mode = AlarmMode::Thermal;
                    }
                }
            }
        }
    }
    if wakeup < current_time + Duration::seconds(70) {
        wakeup = current_time + Duration::seconds(70);
    }
    i2c.set_wakeup_alarm(&wakeup, alarm_mode, time)
        .inspect_err(|e| {
            error!("Failed to set wakeup alarm: {}", e);
        })?;
    if alarm_mode == AlarmMode::Audio {
        events.log(Event::SetAudioAlarm(wakeup.timestamp_micros()), time, fs);
    } else {
        events.log(Event::SetThermalAlarm(wakeup.timestamp_micros()), time, fs);
    }

    Ok(ScheduledAlarmTime {
        time: wakeup,
        mode: alarm_mode,
        already_triggered: false,
    })
}

pub fn work_out_recording_mode(
    scheduled_alarm: &ScheduledAlarmTime,
    prioritise_frame_preview: bool,
    i2c: &mut MainI2C,
    config: &DeviceConfig,
) -> RecordingMode {
    let pi_is_awake = i2c
        .get_camera_state()
        .is_ok_and(CameraState::pi_is_powered_on);
    let tc2_agent_state = if pi_is_awake {
        i2c.get_tc2_agent_state().unwrap_or_default()
    } else {
        Tc2AgentState::default()
    };
    let tc2_agent_ready = tc2_agent_state.is_ready();
    let pi_is_asleep = !(pi_is_awake && tc2_agent_ready);
    let is_audio_only_device =
        config.is_audio_only_device() || i2c.check_if_is_audio_device().unwrap_or(false);
    let records_thermal = !is_audio_only_device;
    /*
    We want to record audio if any of:
    a) we were woken by an audio alarm AND config still has us as an audio device
    AND the audio alarm time is still correct with regard to the thermal window AND
    we don't want to prioritise frame preview
    b) We were woken by a request to make an manual audio recording.

    Otherwise, we go to audio mode but don't record?  Or we go back to sleep if we're not in the
    thermal window, and the pi isn't on?
     */
    if pi_is_asleep {
        if scheduled_alarm.has_triggered() {
            // We woke with an alarm trigger for a scheduled recording.
            if config.is_audio_device() && scheduled_alarm.is_audio_alarm() {
                return RecordingMode::Audio(RecordingRequestType::scheduled_recording());
            } else if !is_audio_only_device && scheduled_alarm.is_thermal_alarm() {
                return RecordingMode::Thermal(RecordingRequestType::scheduled_recording());
            }
        }
        // We don't know why were woken up, but we don't need to make a recording.
        // We could have been woken by user action, and the pi is currently booting,
        // but in that case we'll get restarted when tc2-agent starts.
        RecordingMode::None
    } else {
        // rPi is awake, and we could have been woken up for various reasons.
        if scheduled_alarm.has_triggered() {
            // We woke with an alarm trigger for a scheduled recording.
            if config.is_audio_device() && scheduled_alarm.is_audio_alarm() {
                return RecordingMode::Audio(RecordingRequestType::scheduled_recording());
            } else if !is_audio_only_device && scheduled_alarm.is_thermal_alarm() {
                return RecordingMode::Thermal(RecordingRequestType::scheduled_recording());
            }
        }
        // No scheduled recording or scheduled recording hasn't triggered yet,
        // but that's okay, we'll go into the thermal branch and start serving frames to the rPi,
        // as long as we're not an audio-only device.

        if prioritise_frame_preview {
            warn!("Prioritising frame preview mode");
            RecordingMode::None
        } else if config.is_audio_device() && tc2_agent_state.test_audio_recording_requested() {
            let recording_request_type = if tc2_agent_state.short_test_audio_recording_requested() {
                RecordingRequestType::test_recording(60)
            } else {
                RecordingRequestType::test_recording(60 * 5)
            };
            RecordingMode::Audio(recording_request_type)
        } else if records_thermal && tc2_agent_state.test_thermal_recording_requested() {
            let recording_request_type = if tc2_agent_state.short_test_thermal_recording_requested()
            {
                RecordingRequestType::test_recording(10)
            } else {
                RecordingRequestType::test_recording(60)
            };
            RecordingMode::Thermal(recording_request_type)
        } else {
            RecordingMode::None
        }
    }
}
