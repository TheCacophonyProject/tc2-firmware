use crate::attiny_rtc_i2c::{AudioRecordingType, RecordingMode, SharedI2C, tc2_agent_state};
use crate::audio_task::{
    AlarmMode, MAX_GAP_MIN, check_alarm_still_valid_with_thermal_window, schedule_next_recording,
};
use crate::bsp::pac::RESETS;
use crate::device_config::{AudioMode, DeviceConfig, get_datetime_utc};
use crate::event_logger::{
    Event, EventLogger, LoggerEvent, WakeReason, clear_audio_alarm, get_audio_alarm,
};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::OnboardFlash;
use crate::rpi_power::wake_raspberry_pi;
use crate::sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    offload_all_recordings_and_events, offload_latest_recording,
};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use chrono::{DateTime, Datelike, Duration, Timelike, Utc};
use cortex_m::delay::Delay;
use defmt::{error, info, warn};
use fugit::HertzU32;
use rp2040_hal::pac::DMA;
use rp2040_hal::{Timer, Watchdog};

pub fn current_recording_mode(
    config: &DeviceConfig,
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    time: &SyncedDateTime,
) -> RecordingMode {
    let mut is_audio_device = config.is_audio_device();
    let mut is_audio_only_device = config.audio_mode() == AudioMode::AudioOnly;
    if let Ok(audio_only) = i2c.check_if_is_audio_device(delay) {
        if audio_only {
            info!("Found EEPROM audio-only device");
        }
        is_audio_only_device = audio_only;
        is_audio_device = is_audio_device || audio_only;
    }
    let state = i2c.tc2_agent_state(delay).unwrap_or(0);
    let audio_mode_requested_thermal = state & (tc2_agent_state::THERMAL_MODE) != 0;
    let long_audio_recording = state & (tc2_agent_state::LONG_AUDIO_RECORDING) != 0;
    let test_audio_recording = state & (tc2_agent_state::TEST_AUDIO_RECORDING) != 0;
    let regular_scheduled_audio_recording = state & (tc2_agent_state::TAKE_AUDIO) != 0;
    // All of these flags *should* be mutually exclusive:
    if audio_mode_requested_thermal
        && (long_audio_recording || test_audio_recording || regular_scheduled_audio_recording)
    {
        error!(
            "Mixed recording flags: audio_mode_requested_thermal: {}, long_audio_recording: {}, test_audio_recording: {}, regular_scheduled_audio_recording: {}",
            audio_mode_requested_thermal,
            long_audio_recording,
            test_audio_recording,
            regular_scheduled_audio_recording
        );
    } else if long_audio_recording
        && (audio_mode_requested_thermal
            || test_audio_recording
            || regular_scheduled_audio_recording)
    {
        error!(
            "Mixed recording flags: audio_mode_requested_thermal: {}, long_audio_recording: {}, test_audio_recording: {}, regular_scheduled_audio_recording: {}",
            audio_mode_requested_thermal,
            long_audio_recording,
            test_audio_recording,
            regular_scheduled_audio_recording
        );
    } else if test_audio_recording
        && (long_audio_recording
            || audio_mode_requested_thermal
            || regular_scheduled_audio_recording)
    {
        error!(
            "Mixed recording flags: audio_mode_requested_thermal: {}, long_audio_recording: {}, test_audio_recording: {}, regular_scheduled_audio_recording: {}",
            audio_mode_requested_thermal,
            long_audio_recording,
            test_audio_recording,
            regular_scheduled_audio_recording
        );
    } else if regular_scheduled_audio_recording
        && (long_audio_recording || test_audio_recording || audio_mode_requested_thermal)
    {
        error!(
            "Mixed recording flags: audio_mode_requested_thermal: {}, long_audio_recording: {}, test_audio_recording: {}, regular_scheduled_audio_recording: {}",
            audio_mode_requested_thermal,
            long_audio_recording,
            test_audio_recording,
            regular_scheduled_audio_recording
        );
    }

    if i2c.tc2_agent_clear_mode_flags(delay).is_err() {
        error!("Failed to clear recording mode flags");
    }
    if is_audio_only_device {
        if long_audio_recording {
            RecordingMode::Audio(AudioRecordingType::long_recording())
        } else if test_audio_recording {
            RecordingMode::Audio(AudioRecordingType::test_recording())
        } else {
            RecordingMode::Audio(AudioRecordingType::scheduled_recording())
        }
    } else if is_audio_device {
        if audio_mode_requested_thermal {
            info!("Audio request thermal mode");
            // audio mode wants to go in thermal mode
            RecordingMode::Thermal
        } else {
            // NOTE: Audio only gets scheduled in the thermal window if we're configured
            //  as `AudioMode::AudioAndThermal`, we promise.

            if long_audio_recording {
                info!("Will record audio because long test requested");
                RecordingMode::Audio(AudioRecordingType::long_recording())
            } else if test_audio_recording {
                info!("Will record audio because test requested");
                RecordingMode::Audio(AudioRecordingType::test_recording())
            } else if config.time_is_in_configured_recording_window(&time.date_time()) {
                info!("Will record audio because thermal requested");
                RecordingMode::Audio(AudioRecordingType::thermal_scheduled_recording())
            } else {
                RecordingMode::Audio(AudioRecordingType::scheduled_recording())
            }
        }
    } else {
        RecordingMode::Thermal
    }
}

pub fn get_device_config(
    fs: &mut OnboardFlash,
    i2c: &mut SharedI2C,
    delay: &mut Delay,
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
        wake_raspberry_pi(i2c, delay, Some(watchdog));
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
    match device_config {
        Some(device_config) => Ok((device_config, prioritise_frame_preview)),
        None => {
            error!("Couldn't get config");
            Err(())
        }
    }
}

pub fn get_synced_time(
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    timer: Timer,
) -> Result<SyncedDateTime, ()> {
    match i2c.get_datetime(delay) {
        Ok(now) => {
            info!(
                "RTC Date time {}:{}:{}",
                now.hours, now.minutes, now.seconds
            );
            Ok(SyncedDateTime::new(get_datetime_utc(now), timer))
        }
        Err(e) => {
            // can't get time so use 0 and add a time when tc2-agent uploads
            events.log_event(LoggerEvent::new_with_unknown_time(Event::RtcCommError), fs);
            error!("Failed getting date from RTC");
            Err(())
        }
    }
}

pub fn get_next_audio_alarm(
    config: &DeviceConfig,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
    delay: &mut Delay,
    i2c: &mut SharedI2C,
    events: &mut EventLogger,
) -> Option<DateTime<Utc>> {
    let mut next_audio_alarm = None;
    if config.records_audio_and_thermal() {
        let (alarm_mode, audio_alarm) = get_audio_alarm(fs);
        let mut schedule_alarm = true;
        // if audio alarm is set check it's within 60 minutes and before or on thermal window start
        if let Some(audio_alarm) = audio_alarm {
            match alarm_mode {
                Ok(alarm_mode) => {
                    let synced = time.date_time();
                    let until_alarm = (audio_alarm - synced).num_minutes();
                    info!(
                        "Alarm in mode {} already scheduled for {}-{}-{} {}:{} ({}mins from now)",
                        alarm_mode,
                        audio_alarm.year(),
                        audio_alarm.month(),
                        audio_alarm.day(),
                        audio_alarm.hour(),
                        audio_alarm.minute(),
                        until_alarm
                    );
                    if alarm_mode == AlarmMode::Audio {
                        if until_alarm <= i64::from(MAX_GAP_MIN) {
                            if check_alarm_still_valid_with_thermal_window(
                                &audio_alarm,
                                &synced,
                                config,
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
                Err(reason) => {
                    error!("{}", reason);
                }
            }
        }
        if schedule_alarm {
            // Reschedule a new alarm
            if let Ok(next_alarm) = schedule_next_recording(delay, time, i2c, fs, events, config) {
                next_audio_alarm = Some(next_alarm);
                info!("Setting a pending audio alarm");
            } else {
                error!("Couldn't schedule alarm");
            }
        }
    } else {
        info!("Clearing audio alarm");
        clear_audio_alarm(fs);
    }

    next_audio_alarm
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
    i2c: &mut SharedI2C,
    pi_spi: &mut ExtSpiTransfers,
    delay: &mut Delay,
    watchdog: &mut Watchdog,
) {
    let has_files_to_offload = fs.has_recordings_to_offload();
    if has_files_to_offload {
        info!(
            "First used block {:?}, last used {:?}",
            fs.first_used_block_index, fs.last_used_block_index
        );
    }
    let has_events_to_offload = events.has_events_to_offload();
    let last_recording_was_user_requested = fs.last_recording_was_user_requested();
    let rpi_is_awake = i2c.pi_is_waking_or_awake(delay).unwrap_or(false)
        && i2c.tc2_agent_is_ready(delay, true).unwrap_or(false);
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
        RecordingMode::Thermal => fs.is_too_full_to_start_new_cptv_recordings(),
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
        if wake_raspberry_pi(i2c, delay, Some(watchdog)) {
            events.log(Event::ToldRpiToWake(offload_wake_reason), time, fs);
        }
        if last_recording_was_user_requested {
            if !offload_latest_recording(
                fs, pi_spi, resets, dma, i2c, delay, events, time, watchdog,
            ) {
                // Offload failed, restart and try again.
                error!("File offload failed, restarting");
                restart(watchdog);
            }
        } else if prioritise_frame_preview {
            // Just offload the bare minimum of files to free up space, and all events if needed.
            if events.is_nearly_full() {
                maybe_offload_events(pi_spi, resets, dma, delay, events, fs, time, watchdog);
            }
            let mut is_too_full = is_too_full_to_record_in_current_mode;
            while is_too_full {
                if offload_latest_recording(
                    fs, pi_spi, resets, dma, i2c, delay, events, time, watchdog,
                ) {
                    is_too_full = match recording_mode {
                        RecordingMode::Audio(mode) => {
                            fs.is_too_full_to_start_new_audio_recordings(&mode)
                        }
                        RecordingMode::Thermal => fs.is_too_full_to_start_new_cptv_recordings(),
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
                fs, pi_spi, resets, dma, i2c, delay, events, time, watchdog,
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
