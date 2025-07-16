use crate::attiny_rtc_i2c::{
    AlarmMode, MainI2C, RecordingMode, RecordingRequestType, ScheduledAlarmTime, Tc2AgentState,
};
use crate::audio_task::MAX_INTERVAL_BETWEEN_AUDIO_RECORDINGS;
use crate::bsp::pac::RESETS;
use crate::device_config::{AudioMode, DeviceConfig};
use crate::event_logger::{Event, EventLogger, LoggerEvent, WakeReason};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::OnboardFlash;
use crate::rpi_power::wake_raspberry_pi;
use crate::sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    offload_all_recordings_and_events, offload_latest_recording,
};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use chrono::{DateTime, Duration, NaiveTime, Timelike, Utc};
use cortex_m::prelude::*;
use defmt::{error, info, warn};
use fugit::HertzU32;
use picorand::{PicoRandGenerate, RNG, WyRand};
use rp2040_hal::pac::DMA;
use rp2040_hal::{Timer, Watchdog};

pub fn get_device_config(
    fs: &mut OnboardFlash,
    i2c: &mut MainI2C,
    timer: Timer,
    pi_spi: &mut ExtSpiTransfers,
    system_clock_freq_hz: HertzU32,
    resets: &mut RESETS,
    dma: &mut DMA,
    watchdog: &mut Watchdog,
    event_count: u16,
) -> Result<(DeviceConfig, bool), ()> {
    let device_config: Option<DeviceConfig> = DeviceConfig::load_existing_config_from_flash(fs);
    if let Some(device_config) = &device_config {
        info!("Existing config {:#?}", device_config.config());
    } else {
        info!("Waking pi to get config");
        // We need to wake up the rpi and get a config
        wake_raspberry_pi(i2c, timer, Some(watchdog), None);
    }

    // FIXME: This should become "config startup handshake", and we have another
    //  camera config handshake later on when the lepton has actually initialised or whatever.
    let (device_config, device_config_was_updated, prioritise_frame_preview) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            fs,
            pi_spi,
            resets,
            dma,
            system_clock_freq_hz,
            device_config,
            event_count,
        );
    if let Some(device_config) = device_config {
        Ok((device_config, prioritise_frame_preview))
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
    timer: Timer,
) -> Result<SyncedDateTime, &'static str> {
    let time = i2c.get_datetime(timer);
    if let Err(e) = time {
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
    time
}

#[allow(clippy::ref_option)]
pub fn get_or_schedule_next_alarm(
    config: &DeviceConfig,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
    i2c: &mut MainI2C,
    events: &mut EventLogger,
    scheduled_alarm: &Option<ScheduledAlarmTime>,
    watchdog: &mut Watchdog,
) -> bool {
    // if audio alarm is set check it's within the next 60 minutes
    // and before or on thermal window start
    let mut missed_audio_alarm = false;
    if let Some(scheduled_alarm) = scheduled_alarm
        && scheduled_alarm.is_audio_alarm()
        && config.records_audio_and_thermal()
    {
        let synced_time = time.date_time();
        let alarm_time = scheduled_alarm.time();
        let until_alarm = (alarm_time - time.date_time()).num_minutes();

        // TODO: Handle different audio modes differently.
        if alarm_time - synced_time <= MAX_INTERVAL_BETWEEN_AUDIO_RECORDINGS {
            if !alarm_still_valid_for_thermal_window(&alarm_time, &synced_time, config) {
                // if window time changed and alarm is after rec window start
                info!("Rescheduling as alarm is after window start");
                // Reschedule a new alarm
                if let Ok(next_alarm) = schedule_next_recording(time, i2c, fs, events, config) {
                    info!("Setting a pending recording alarm");
                    restart(watchdog);
                } else {
                    error!("Couldn't schedule alarm");
                }
            }
        } else {
            warn!(
                "Missed alarm was scheduled for {} minutes ago",
                -until_alarm
            );
            events.log(
                Event::Rp2040MissedAudioAlarm(alarm_time.timestamp_micros()),
                time,
                fs,
            );
            // should take recording now
            missed_audio_alarm = true;
        }
    } else if config.records_audio_and_thermal() {
        if schedule_next_recording(time, i2c, fs, events, config).is_ok() {
            info!("Setting a pending recording alarm");
            restart(watchdog);
        } else {
            error!("Couldn't schedule alarm");
        }
    }
    missed_audio_alarm
}

#[allow(clippy::too_many_lines)]
pub fn maybe_offload_files_and_events_on_startup(
    recording_mode: RecordingMode,
    prioritise_frame_preview: bool,
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
        info!(
            "First used block {:?}, last used {:?}",
            fs.first_used_block_index, fs.last_used_block_index
        );
    }
    let has_events_to_offload = events.has_events_to_offload();
    let last_recording_was_user_requested = fs.last_recording_was_user_requested();
    let rpi_is_awake = i2c
        .get_camera_state()
        .is_ok_and(|state| state.pi_is_waking_or_awake())
        && i2c
            .get_tc2_agent_state()
            .is_ok_and(|state| state.is_ready());
    let is_inside_thermal_recording_window =
        config.time_is_in_configured_recording_window(&time.date_time());

    info!(
        "Inside recording window? {}",
        is_inside_thermal_recording_window
    );

    let is_outside_thermal_recording_window = !is_inside_thermal_recording_window;
    let records_audio = config.audio_mode() != AudioMode::Disabled;
    let is_too_full_to_record_in_current_mode = match recording_mode {
        RecordingMode::Audio(mode) => fs.is_too_full_to_start_new_audio_recordings(&mode),
        RecordingMode::Thermal(_) => fs.is_too_full_to_start_new_cptv_recordings(),
        RecordingMode::None => false,
    };

    if is_too_full_to_record_in_current_mode {
        warn!("Too full to record in current mode");
    }

    let is_old_file_system = fs.file_start_block_index.is_none();
    let mut offload_wake_reason = WakeReason::Unknown;
    let mut should_offload = false;

    if has_files_to_offload {
        info!("Onboard flash has files to offload");
    }
    if has_events_to_offload {
        info!("There are {} event(s) to offload", events.count());
    }
    if last_recording_was_user_requested {
        if !rpi_is_awake {
            warn!("Last recording was user requested, but rPi is somehow not awake");
        }
        info!("Last recording was user requested, offload immediately");
    }

    // Also
    // fs.last_recording_was_status()
    let end_of_thermal_window = fs.last_recorded_file_is_cptv()
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
    if is_old_file_system {
        should_offload = true;
        offload_wake_reason = WakeReason::Unknown;
    }
    if events.is_nearly_full() {
        should_offload = true;
        offload_wake_reason = WakeReason::EventsTooFull;
    }
    if last_recording_was_user_requested {
        should_offload = true;
        offload_wake_reason = WakeReason::OffloadTestRecording;
    }
    if rpi_is_awake && recording_mode.record_audio() {
        // Opportunistically offload recordings anyway,
        // since the rpi is on (maybe in high power mode)
        offload_wake_reason = WakeReason::OpportunisticOffload;
        should_offload = true;
    }
    let should_offload = if !should_offload && has_files_to_offload {
        offload_wake_reason = WakeReason::ThermalOffloadAfter24Hours;
        duration_since_prev_offload_greater_than_24hrs(events, fs, time)
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

fn duration_since_prev_offload_greater_than_24hrs(
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
) -> bool {
    let duration_since_prev_offload = events
        .latest_event_of_kind(Event::OffloadedRecording, fs)
        .map_or(Duration::minutes(0), |prev_event| {
            prev_event
                .timestamp()
                .map_or(Duration::minutes(0), |date_time_utc| {
                    time.date_time() - date_time_utc
                })
        });
    duration_since_prev_offload > Duration::hours(24)
}

fn power_down_or_restart(
    alarm_date_time: Option<DateTime<Utc>>,
    time: &SyncedDateTime,
    config: &DeviceConfig,
    mut watchdog: Watchdog,
    mut i2c: MainI2C,
    mut events: EventLogger,
    mut fs: OnboardFlash,
) -> ! {
    let mut should_sleep = true;
    let mut logged_power_down = Some(());
    let boot_into_thermal_mode = config.records_audio_and_thermal();
    // FIXME: This looks sus to me – what happens in AudioOrThermal mode?
    let in_thermal_recording_window = config.audio_mode() == AudioMode::AudioAndThermal
        && config.time_is_in_configured_recording_window(&time.date_time());

    // FIXME: Clean up this flow control
    loop {
        let pi_is_powered_off = i2c
            .get_camera_state()
            .is_ok_and(|state| state.pi_is_powered_off());
        // If we're in low-power mode, we don't need the rPi anymore.
        if !pi_is_powered_off
            && i2c.advise_raspberry_pi_it_may_shutdown().is_ok()
            && logged_power_down.take().is_some()
        {
            events.log(Event::ToldRpiToSleep, time, &mut fs);
        }

        watchdog.feed();
        if should_sleep {
            if let Ok(state) = i2c.get_camera_state() {
                if state.pi_is_powered_off() {
                    if let Some(alarm_time) = alarm_date_time {
                        let until_alarm = (alarm_time - time.date_time()).num_minutes();
                        if until_alarm < 1 {
                            // otherwise the alarm could trigger between here and sleeping
                            should_sleep = false;
                            info!("Alarm is scheduled in {} so not sleeping", until_alarm);
                            if until_alarm <= 0 {
                                info!("Alarm in past, so restart now to take recording");
                                restart(&mut watchdog);
                            }
                            continue;
                        }
                    }

                    if !in_thermal_recording_window {
                        info!("Ask Attiny to power down rp2040");
                        events.log(Event::Rp2040Sleep, time, &mut fs);
                        if i2c.tell_attiny_to_power_down_rp2040().is_ok() {
                            info!("Sleeping");
                        } else {
                            error!("Failed sending sleep request to attiny");
                        }
                    }
                }
            }
        }
        if boot_into_thermal_mode && should_sleep {
            // if we have done everything and PI is still on go into thermal for preview
            if let Ok(()) = i2c.tc2_agent_request_thermal_mode() {
                info!("Going into thermal mode");
                restart(&mut watchdog);
            }
        }

        if let Some(scheduled_alarm) = i2c.get_scheduled_alarm(time) {
            if scheduled_alarm.has_triggered() {
                info!("Alarm triggered after taking a recording resetting rp2040");
                restart(&mut watchdog);
            }
        }

        time.get_timer().delay_ms(5 * 1000);
        watchdog.feed();
    }
}

#[allow(clippy::too_many_lines)]
pub fn schedule_next_recording(
    time: &SyncedDateTime,
    i2c: &mut MainI2C,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    config: &DeviceConfig,
) -> Result<DateTime<Utc>, ()> {
    // FIXME: Meaning of returned result seems inconsistent
    let mut wakeup: DateTime<Utc>;
    let seed: u64;
    let current_time = time.date_time();

    if config.audio_seed() > 0 {
        wakeup = if let Some(time) = current_time
            .with_time(NaiveTime::from_hms_opt(0, 0, 0).expect("Invalid time"))
            .latest()
        {
            time
        } else {
            error!("Failed to clamp current time to midnight");
            return Err(());
        };
        let Ok(ts_millis) = u64::try_from(wakeup.timestamp_millis()) else {
            error!("Failed to convert wakeup timestamp milliseconds to u64");
            return Err(());
        };

        seed = u64::wrapping_add(ts_millis, u64::from(config.config().audio_seed));
    } else {
        let Ok(ts_seconds) = u64::try_from(time.date_time().timestamp()) else {
            error!("Failed to convert current_time timestamp to u64");
            return Err(());
        };

        seed = ts_seconds;
        wakeup = current_time;
    }
    let mut rng = RNG::<WyRand, u16>::new(seed);
    let r_max: u16 = u16::MAX;
    let short_chance: u16 = r_max / 4;
    let short_pause: i64 = 2 * 60;
    let short_window: i64 = 5 * 60;
    let long_pause: i64 = 40 * 60;
    let long_window: i64 = 20 * 60;

    // If a seed is set, always start alarm from 0am to keep consistent across devices.
    // So will need to generate numbers until the alarm is valid
    while wakeup <= current_time {
        let r = rng.generate();
        let wake_in = if crate::audio_task::DEV_MODE {
            120
        } else if r <= short_chance {
            short_pause + (i64::from(r) * short_window) / i64::from(short_chance)
        } else {
            long_pause + (i64::from(r) * long_window) / i64::from(r_max)
        };
        wakeup += Duration::seconds(wake_in);
    }

    let mut alarm_mode = AlarmMode::Audio;
    let audio_mode = config.audio_mode();
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
                    if let AudioMode::AudioAndThermal = audio_mode {
                        // audio recording inside recording window
                        info!("Scheduling audio inside thermal window");
                    } else {
                        // FIXME: Seems like we should still schedule an audio recording
                        //  if we're in audioAndThermal mode?
                        info!("Already in recording window so restart now");
                        return Err(());
                    }
                } else {
                    info!("Setting wake up to be start of next thermal recording window");
                    wakeup = start;
                    alarm_mode = AlarmMode::Thermal;
                }
            }
        }
    }
    i2c.set_wakeup_alarm(&wakeup, alarm_mode, time)
        .map_err(|e| {
            error!("Failed to set wakeup alarm: {}", e);
        })?;
    if alarm_mode == AlarmMode::Audio {
        events.log(Event::SetAudioAlarm(wakeup.timestamp_micros()), time, fs);
    } else {
        events.log(Event::SetThermalAlarm(wakeup.timestamp_micros()), time, fs);
    }

    Ok(wakeup)
}

pub fn alarm_still_valid_for_thermal_window(
    alarm: &DateTime<Utc>,
    now: &DateTime<Utc>,
    config: &DeviceConfig,
) -> bool {
    // config has changed so if not in audio-only mode check that
    // the alarm is still going to trigger on recording window start
    if !config.is_audio_only_device() {
        // FIXME: Handle both types of audio+thermal
        return if let Ok((start, end)) = config.next_or_current_recording_window(now) {
            // alarm before start or we are in recording window
            *alarm <= start || *now >= start
        } else {
            false
        };
    }
    true
}

#[allow(clippy::ref_option)]
pub fn work_out_recording_mode(
    scheduled_recording: &Option<ScheduledAlarmTime>,
    prioritise_frame_preview: bool,
    i2c: &mut MainI2C,
    config: &DeviceConfig,
    time: &SyncedDateTime,
) -> RecordingMode {
    // TODO: Check if pi is powering on, and if it is, wait for it.  Maybe not in this function tho?
    //  If config.high_power_mode() and the pi is not awake, always wake it?

    let pi_is_awake = i2c.get_camera_state().is_ok_and(|s| s.pi_is_powered_on());
    let tc2_agent_state = if pi_is_awake {
        i2c.get_tc2_agent_state().unwrap_or_default()
    } else {
        Tc2AgentState::default()
    };
    let tc2_agent_ready = tc2_agent_state.is_ready();
    let test_recording_requested = tc2_agent_state.requested_audio_mode();
    let pi_is_asleep = !(pi_is_awake && tc2_agent_ready);
    let is_audio_only_device =
        config.is_audio_only_device() || i2c.check_if_is_audio_device().unwrap_or(false);
    // FIXME: Probably should clear mode flags after a successful test recording?
    if i2c.tc2_agent_clear_mode_flags().is_err() {
        error!("Failed to clear recording mode flags");
    }
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
        if let Some(scheduled_alarm) = scheduled_recording {
            if scheduled_alarm.has_triggered() {
                // We woke with an alarm trigger for a scheduled recording.
                if config.is_audio_device() && scheduled_alarm.is_audio_alarm() {
                    return RecordingMode::Audio(RecordingRequestType::rp2040_scheduled_recording());
                } else if !is_audio_only_device && scheduled_alarm.is_thermal_alarm() {
                    return RecordingMode::Thermal(
                        RecordingRequestType::rp2040_scheduled_recording(),
                    );
                }
            }
            // We don't know why were woken up, but we don't need to make a recording.
        }
        RecordingMode::None
    } else {
        // rPi is awake, and we could have been woken up for various reasons.
        if let Some(scheduled_alarm) = scheduled_recording {
            // FIXME: In both branches, we just check to see if the alarm has triggered to see if we need to restart into a diff mode,
            //  rather than checking time offsets
            if scheduled_alarm.has_triggered() {
                // We woke with an alarm trigger for a scheduled recording.
                if config.is_audio_device() && scheduled_alarm.is_audio_alarm() {
                    return RecordingMode::Audio(RecordingRequestType::rp2040_scheduled_recording());
                } else if !is_audio_only_device && scheduled_alarm.is_thermal_alarm() {
                    return RecordingMode::Thermal(
                        RecordingRequestType::rp2040_scheduled_recording(),
                    );
                }
            }
        }
        // No scheduled recording or scheduled recording hasn't triggered yet,
        // but that's okay, we'll go into the thermal branch and start serving frames to the rPi,
        // as long as we're not an audio-only device.

        // FIXME: Do we need to deal with prioritise frames here, or would we expect the pi to just
        //  restart us in thermal mode if needed?

        if config.is_audio_device() && tc2_agent_state.test_audio_recording_requested() {
            let recording_request_type = if tc2_agent_state.short_test_audio_recording_requested() {
                RecordingRequestType::short_test_recording()
            } else {
                RecordingRequestType::long_test_recording()
            };
            RecordingMode::Audio(recording_request_type)
        } else if !is_audio_only_device && tc2_agent_state.test_thermal_recording_requested() {
            let recording_request_type = if tc2_agent_state.short_test_thermal_recording_requested()
            {
                RecordingRequestType::short_test_recording()
            } else {
                RecordingRequestType::long_test_recording()
            };
            return RecordingMode::Thermal(recording_request_type);
        } else if !is_audio_only_device && tc2_agent_state.requested_thermal_mode() {
            return RecordingMode::Thermal(RecordingRequestType::tc2_agent_scheduled_recording());
        } else if config.is_audio_device() && tc2_agent_state.requested_audio_mode() {
            return RecordingMode::Audio(RecordingRequestType::tc2_agent_scheduled_recording());
        } else if !is_audio_only_device {
            RecordingMode::Thermal(RecordingRequestType::rp2040_scheduled_recording())
        } else {
            return RecordingMode::None;
        }
    }
}
