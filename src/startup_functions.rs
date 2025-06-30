use crate::attiny_rtc_i2c::{AudioRecordingType, SharedI2C, tc2_agent_state};
use crate::audio_task::{
    AlarmMode, MAX_GAP_MIN, check_alarm_still_valid_with_thermal_window, schedule_audio_rec,
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
    offload_all_recordings_and_events,
};
use crate::synced_date_time::SyncedDateTime;
use chrono::{DateTime, Datelike, Duration, Timelike, Utc};
use cortex_m::delay::Delay;
use defmt::{error, info, panic, warn};
use fugit::HertzU32;
use rp2040_hal::pac::DMA;
use rp2040_hal::{Timer, Watchdog};

pub fn should_record_audio(
    config: &DeviceConfig,
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    time: &SyncedDateTime,
) -> bool {
    let mut wants_to_record_audio_now: bool = config.is_audio_device();
    if let Ok(audio_only) = i2c.check_if_is_audio_device(delay) {
        info!("EEPROM audio device: {}", audio_only);
        wants_to_record_audio_now = wants_to_record_audio_now || audio_only;
    }
    if wants_to_record_audio_now {
        // FIXME: If we have actual audio only devices as set by eeprom, it doesn't seem like the
        //  device config should be able to override that.
        if config.records_audio_and_thermal() {
            if let Ok(state) = i2c.tc2_agent_state(delay) {
                if (state & (tc2_agent_state::THERMAL_MODE)) != 0 {
                    let _ = i2c.tc2_agent_clear_thermal_mode(delay);
                    info!("Audio request thermal mode");
                    // audio mode wants to go in thermal mode
                    wants_to_record_audio_now = false;
                } else {
                    let in_window =
                        config.time_is_in_configured_recording_window(&time.get_date_time());
                    if in_window {
                        wants_to_record_audio_now = (state
                            & (tc2_agent_state::LONG_AUDIO_RECORDING
                                | tc2_agent_state::TAKE_AUDIO
                                | tc2_agent_state::TEST_AUDIO_RECORDING))
                            != 0;
                        if wants_to_record_audio_now {
                            info!("Will record audio because thermal requested or test rec");
                        }
                    }
                }
            }
        }
    }
    wants_to_record_audio_now
}

pub fn get_device_config(
    fs: &mut OnboardFlash,
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    pi_spi: &mut ExtSpiTransfers,
    system_clock_freq_hz: HertzU32,
    resets: &mut RESETS,
    dma: &mut DMA,
) -> (DeviceConfig, bool) {
    let device_config: Option<DeviceConfig> = DeviceConfig::load_existing_config_from_flash(fs);
    if let Some(device_config) = &device_config {
        info!("Existing config {:#?}", device_config.config());
    } else {
        info!("Waking pi to get config");
        // We need to wake up the rpi and get a config
        wake_raspberry_pi(i2c, delay);
    }

    // FIXME: This should become "config startup handshake", and we have another
    //  camera config handshake later on when the lepton has actually initialised or whatever.
    let (device_config, device_config_was_updated, prefer_not_to_offload_files) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            fs,
            pi_spi,
            resets,
            dma,
            system_clock_freq_hz,
            device_config,
        );
    match device_config {
        Some(device_config) => (device_config, prefer_not_to_offload_files),
        None => {
            panic!("Couldn't get config");
        }
    }
}

pub fn get_synced_time(
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    timer: Timer,
) -> SyncedDateTime {
    loop {
        // NOTE: Keep retrying until we get a datetime from RTC.
        match i2c.get_datetime(delay) {
            Ok(now) => {
                info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
                return SyncedDateTime::new(get_datetime_utc(now), timer);
            }
            Err(e) => {
                // cant get time so use 0 and add a time when tc2-agent uploads
                events.log_event(LoggerEvent::new_with_unknown_time(Event::RtcCommError), fs);

                warn!("Failed getting date from RTC, retrying");
                delay.delay_ms(10);
            }
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
    // FIXME: Shouldn't this have already happened on startup?
    let mut next_audio_alarm = None;
    match config.audio_mode() {
        AudioMode::AudioOrThermal | AudioMode::AudioAndThermal => {
            let (alarm_mode, audio_alarm) = get_audio_alarm(fs);
            let mut schedule_alarm = true;
            // if audio alarm is set check it's within 60 minutes and before or on thermal window start
            if let Some(audio_alarm) = audio_alarm {
                match alarm_mode {
                    Ok(alarm_mode) => {
                        // FIXME: Document what the AlarmModes mean
                        if alarm_mode == AlarmMode::Audio {
                            let synced = time.get_date_time();
                            let until_alarm = (audio_alarm - synced).num_minutes();
                            // FIXME: This doesn't seem to handle alarms in the past?
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
                if let Ok(next_alarm) = schedule_audio_rec(delay, time, i2c, fs, events, config) {
                    next_audio_alarm = Some(next_alarm);
                    info!("Setting a pending audio alarm");
                } else {
                    error!("Couldn't schedule alarm");
                }
            }
        }
        _ => {
            info!("Clearing audio alarm");
            clear_audio_alarm(fs);
        }
    }
    next_audio_alarm
}

#[allow(clippy::too_many_lines)]
pub fn maybe_offload_files_and_events_on_startup(
    recording_mode: &str,
    prefer_not_to_offload_files_now: bool,
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
) -> bool {
    let has_files_to_offload = fs.has_recordings_to_offload();
    if has_files_to_offload {
        info!("Finished scan, has files to offload");
    }

    if events.has_events_to_offload() {
        info!("There are {} event(s) to offload", events.count());
    }

    // NOTE: We'll only wake the pi if we have files to offload, and it is *outside* the recording
    //  window, or the previous offload happened more than 24 hours ago, or the flash is nearly full.
    //  Otherwise, if the rp2040 happens to restart, we'll pretty much
    //  always start the pi up, which we don't want.

    let should_offload = (has_files_to_offload
        && !config.time_is_in_configured_recording_window(&time.get_date_time()))
        || fs.is_too_full_to_start_new_cptv_recordings()
        || (has_files_to_offload && fs.file_start_block_index.is_none());
    // means old file system offload once

    if should_offload {
        events.log(Event::ToldRpiToWake(WakeReason::ThermalOffload), time, fs);
    }
    let should_offload = if !should_offload && has_files_to_offload {
        duration_since_prev_offload_greater_than_24hrs(events, fs, time)
    } else {
        should_offload
    };

    ////////
    // Happened in thermal branch
    if !config.use_low_power_mode() {
        // TODO: Do we want to do this in both branches, assuming the pi is awake?
        if wake_raspberry_pi(i2c, delay) {
            events.log(Event::ToldRpiToWake(WakeReason::ThermalHighPower), time, fs);
        }
        maybe_offload_events(pi_spi, resets, dma, delay, events, fs, time, watchdog);
    }

    /////////

    // FIXME: This should all happen in main startup
    let did_offload_files = if should_offload {
        offload_all_recordings_and_events(
            fs, pi_spi, resets, dma, i2c, delay, events, time, watchdog,
        )
    } else {
        false
    };

    //////
    // From audio startup
    // FIXME: Consolidate this logic in main

    // Rather than deciding if we should wake, just decide if we should offload, and then of course
    // we'll wake if needed.

    let mut should_wake = false;
    if config.records_audio_and_thermal()
        && config.time_is_in_configured_recording_window(&time.get_date_time())
    {
        let has_cptv_files = fs.last_recorded_file_is_cptv();
        // this means end of thermal window so should offload recordings
        if has_cptv_files {
            info!("Has cptv files {}", has_cptv_files);
            should_wake = true;
            events.log(
                Event::ToldRpiToWake(WakeReason::AudioThermalEnded),
                time,
                fs,
            );
        }
    }
    if !should_wake {
        should_wake = should_offload_audio_recordings(fs, events);
        if should_wake {
            events.log(
                Event::ToldRpiToWake(WakeReason::AudioShouldOffload),
                time,
                fs,
            );
        }
    }
    //
    // if offload(i2c, fs, pi_spi, events, should_wake, delay, time, watchdog).is_err() {
    //     warn!("Restarting as could not offload");
    //     restart(watchdog);
    // }

    //////

    did_offload_files
}

fn should_offload_audio_recordings(fs: &mut OnboardFlash, events: &mut EventLogger) -> bool {
    let has_files = fs.has_recordings_to_offload() || events.is_nearly_full();
    if !has_files {
        return false;
    }
    // flash getting full
    if fs.is_too_full_to_start_new_audio_recordings(&AudioRecordingType::long_recording()) {
        info!("Offloading as flash is nearly full");
        return true;
    }

    // probably never happens if functioning correctly
    if events.is_nearly_full() {
        info!("Offloading as logger is nearly full");
        return true;
    }
    if fs.file_start_block_index.is_none() {
        //one off
        info!("Offloading as previous file system version");
        return true;
    }

    false
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
                    time.get_date_time() - date_time_utc
                })
        });
    if duration_since_prev_offload > Duration::hours(24) {
        events.log(
            Event::ToldRpiToWake(WakeReason::ThermalOffloadAfter24Hours),
            time,
            fs,
        );
        true
    } else {
        false
    }
}
