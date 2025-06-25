use crate::attiny_rtc_i2c::{AudioRecordingType, SharedI2C, tc2_agent_state};
use crate::bsp;
use crate::bsp::pac::{PIO1, Peripherals};
use crate::device_config::{AudioMode, DeviceConfig};
use crate::event_logger::{
    Event, EventLogger, clear_audio_alarm, get_audio_alarm, write_audio_alarm,
};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::OnboardFlash;
use crate::pdm_microphone::PdmMicrophone;
use crate::sub_tasks::{offload_all_recordings_and_events, offload_latest_recording};
use cortex_m::delay::Delay;
use defmt::{error, info, warn};
use rp2040_hal::gpio;

use chrono::{DateTime, Datelike, NaiveTime, Timelike, Utc};
use fugit::{ExtU32, HertzU32};

use crate::rpi_power::{advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi};
use crate::synced_date_time::SyncedDateTime;
use crate::utils::restart;
use bsp::hal::Watchdog;
use gpio::FunctionNull;
use gpio::bank0::{Gpio0, Gpio1};
use picorand::{PicoRandGenerate, RNG, WyRand};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::pio::PIO;
use rp2040_hal::pio::SM1;
use rp2040_hal::pio::UninitStateMachine;

#[repr(u8)]
#[derive(PartialEq, Eq)]
pub enum AlarmMode {
    Audio = 0,
    Thermal = 1,
}

impl TryFrom<u8> for AlarmMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AlarmMode::Audio),
            1 => Ok(AlarmMode::Thermal),
            _ => Err(()),
        }
    }
}

const DEV_MODE: bool = false;
pub const MAX_GAP_MIN: u8 = 60;

fn maybe_reschedule_audio_recording(
    recording_type: &mut Option<AudioRecordingType>,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    time: &SyncedDateTime,
    alarm_date_time: &mut Option<DateTime<Utc>>,
    reschedule: &mut bool,
    watchdog: &mut Watchdog,
    alarm_triggered: &bool,
) {
    // this isn't reliable so use alarm stored in flash
    // let mut alarm_hours = i2c.get_alarm_hours();
    // let mut alarm_minutes = i2c.get_alarm_minutes();
    let mut scheduled: bool = false;

    // GP 30th Jan TODO if alarm is set we always need to check alarm against current time if the alarm wasnt triggered
    // otherwise can have edge cases where tc2 agent status code thinks we should rec but the alarm is later
    let (_, flash_alarm) = get_audio_alarm(fs);
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
        *alarm_date_time = Some(alarm);
    } else {
        error!("Not scheduled");
    }
    events.log(Event::AudioMode, time, fs);
    *reschedule = !scheduled;

    watchdog.feed();

    info!(
        "Alarm triggered {} scheduled {} thermal requested {}",
        alarm_triggered, scheduled, recording_type
    );

    if recording_type.is_none() && scheduled {
        // check we haven't missed the alarm somehow
        if let Some(alarm) = alarm_date_time {
            let until_alarm = (*alarm - time.get_date_time()).num_minutes();
            if until_alarm <= 0 || until_alarm > i64::from(MAX_GAP_MIN) {
                info!(
                    "Missed alarm was scheduled for the {} at {}:{} but its {} minutes away",
                    alarm.day(),
                    alarm.hour(),
                    alarm.minute(),
                    until_alarm
                );

                // should take recording now
                events.log(
                    Event::Rp2040MissedAudioAlarm(alarm.timestamp_micros()),
                    time,
                    fs,
                );
                *recording_type = Some(AudioRecordingType::scheduled_recording());
            }
        }
    }
}

#[allow(clippy::too_many_lines)]
pub fn audio_task(
    mut i2c: SharedI2C,
    system_clock_freq: HertzU32,
    gpio0: gpio::Pin<Gpio0, FunctionNull, PullDown>,
    gpio1: gpio::Pin<Gpio1, FunctionNull, PullDown>,
    mut watchdog: Watchdog,
    alarm_triggered: bool,
    config: &DeviceConfig,
    mut fs: OnboardFlash,
    mut pi_spi: ExtSpiTransfers,
    mut events: EventLogger,
    time: &SyncedDateTime,
    pio1: PIO<PIO1>,
    sm1: UninitStateMachine<(PIO1, SM1)>,
    mut delay: Delay,
) -> ! {
    watchdog.feed();
    let mut recording_type = None;
    let thermal_requested_audio = false;
    if alarm_triggered {
        recording_type = Some(AudioRecordingType::scheduled_recording());
    } else if let Ok(audio_rec) = i2c.tc2_agent_requested_audio_recording(&mut delay) {
        recording_type = audio_rec;
    }

    let mut reschedule = false;
    let mut alarm_date_time: Option<DateTime<Utc>> = None;

    if recording_type.is_none() || !recording_type.as_ref().unwrap().is_user_requested() {
        // Something strange has happened.  We're in the audio branch without a scheduled
        // recording, or a user requested recording.
        // FIXME Can this really happen?
        // At any rate, we'll reschedule a new recording.
        maybe_reschedule_audio_recording(
            &mut recording_type,
            &mut fs,
            &mut events,
            time,
            &mut alarm_date_time,
            &mut reschedule,
            &mut watchdog,
            &alarm_triggered,
        );
    }
    if let Some(recording_type) = recording_type {
        watchdog.feed();
        // should have already offloaded but extra safety check
        // FIXME Actually, lets not
        if !fs.is_too_full_for_to_start_new_audio_recordings(&recording_type) {
            let _ = i2c
                .set_recording_flag(&mut delay, true)
                .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
            let timestamp = time.get_timestamp_micros();
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

            let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
            let mut microphone = PdmMicrophone::new(
                gpio0.into_function().into_pull_type(),
                gpio1.into_function().into_pull_type(),
                system_clock_freq,
                pio1,
                sm1,
            );

            events.log(Event::StartedAudioRecording, time, &mut fs);

            let recording_failed = !microphone.record_for_n_seconds(
                recording_type.duration_seconds(),
                dma_channels.ch3,
                dma_channels.ch4,
                &mut peripherals.RESETS,
                &peripherals.SPI1,
                &mut fs,
                timestamp,
                &mut watchdog,
                time,
            );
            let _ = i2c
                .set_recording_flag(&mut delay, false)
                .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));
            if recording_failed {
                events.log(Event::AudioRecordingFailed, time, &mut fs);
                info!("Recording failed restarting and will try again");
                restart(&mut watchdog);
            } else {
                events.log(Event::EndedRecording, time, &mut fs);
                match recording_type {
                    AudioRecordingType::Long(_) | AudioRecordingType::Test(_) => {
                        info!("taken test recording clearing status");
                        watchdog.feed();
                        // FIXME: Make the intent of this much clearer
                        let _ = i2c.tc2_agent_clear_and_set_flag(
                            &mut delay,
                            match recording_type {
                                AudioRecordingType::Long(_) => {
                                    tc2_agent_state::LONG_AUDIO_RECORDING
                                }
                                AudioRecordingType::Test(_) => {
                                    tc2_agent_state::TEST_AUDIO_RECORDING
                                }
                                _ => 0,
                            },
                            if config.config().audio_mode == AudioMode::AudioOnly {
                                None
                            } else {
                                Some(tc2_agent_state::THERMAL_MODE)
                            },
                        );

                        let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

                        offload_latest_recording(
                            &mut fs,
                            &mut pi_spi,
                            &mut peripherals.RESETS,
                            &mut peripherals.DMA,
                            &mut i2c,
                            &mut delay,
                            &mut events,
                            time,
                            &mut watchdog,
                        );

                        restart(&mut watchdog);
                    }
                    _ => {
                        i2c.clear_alarm(&mut delay);
                        reschedule = true;
                        clear_audio_alarm(&mut fs);
                        if thermal_requested_audio {
                            //if audio requested from thermal, the alarm will be re scheduled there
                            let _ = i2c.tc2_agent_clear_and_set_flag(
                                &mut delay,
                                tc2_agent_state::TAKE_AUDIO,
                                Some(tc2_agent_state::THERMAL_MODE),
                            );
                            info!("Audio taken in thermal window clearing flag");
                            restart(&mut watchdog);
                        }
                    }
                }
            }
        }
    }

    let mut should_sleep = true;
    if reschedule {
        watchdog.feed();
        info!("Scheduling new recording");
        if let Ok(scheduled_time) =
            schedule_audio_rec(&mut delay, time, &mut i2c, &mut fs, &mut events, config)
        {
            alarm_date_time = Some(scheduled_time);
        } else {
            error!("Couldn't schedule alarm will restart");
            clear_audio_alarm(&mut fs);
            restart(&mut watchdog);
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

    watchdog.start(8_388_607.micros());

    let mut logged_power_down = false;
    let boot_thermal = matches!(
        config.config().audio_mode,
        AudioMode::AudioAndThermal | AudioMode::AudioOrThermal
    );
    let in_window = config.config().audio_mode == AudioMode::AudioAndThermal
        && config.time_is_in_recording_window(&time.get_date_time(), None);

    loop {
        advise_raspberry_pi_it_may_shutdown(&mut i2c, &mut delay);
        if !logged_power_down {
            events.log(Event::ToldRpiToSleep, time, &mut fs);
            logged_power_down = true;
        }

        watchdog.feed();
        if should_sleep {
            if let Ok(pi_is_powered_down) = i2c.pi_is_powered_down(&mut delay, true) {
                if pi_is_powered_down {
                    if let Some(alarm_time) = alarm_date_time {
                        let until_alarm = (alarm_time - time.get_date_time()).num_minutes();

                        if until_alarm < 1 {
                            // otherwise the alarm could trigger between here and sleeping
                            should_sleep = false;
                            info!("Alarm is scheduled in {} so not sleeping", until_alarm);
                            if until_alarm <= 0 {
                                restart(&mut watchdog);
                            }
                            continue;
                        }
                    }

                    if !in_window {
                        info!("Ask Attiny to power down rp2040");
                        events.log(Event::Rp2040Sleep, time, &mut fs);

                        if i2c.tell_attiny_to_power_down_rp2040(&mut delay).is_ok() {
                            info!("Sleeping");
                        } else {
                            error!("Failed sending sleep request to attiny");
                        }
                    }
                }

                // may as well offload again if we are awake and have just taken a recording
                if !pi_is_powered_down
                    && fs.has_files_to_offload()
                    && offload(
                        &mut i2c,
                        &mut fs,
                        &mut pi_spi,
                        &mut events,
                        false,
                        &mut delay,
                        time,
                        &mut watchdog,
                    )
                    .is_err()
                {
                    warn!("Restarting as failed to offload");
                    restart(&mut watchdog);
                }
            }
        }
        if boot_thermal && should_sleep {
            //if we have done everything and PI is still on go into thermal for preview
            if let Ok(()) = i2c.tc2_agent_request_thermal_mode(&mut delay) {
                info!("Going into thermal mode");
                restart(&mut watchdog);
            }
        }

        let alarm_triggered: bool = i2c.alarm_triggered(&mut delay);
        if alarm_triggered {
            info!("Alarm triggered after taking a recording resetting rp2040");
            restart(&mut watchdog);
        }

        delay.delay_ms(5 * 1000);
        watchdog.feed();
    }
}

pub fn offload(
    i2c: &mut SharedI2C,
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    events: &mut EventLogger,
    mut wake_if_asleep: bool,
    delay: &mut Delay,
    time: &SyncedDateTime,
    watchdog: &mut Watchdog,
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
            watchdog.start(8_388_607.micros());
        }
        if awake {
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
            if !offload_all_recordings_and_events(
                fs,
                pi_spi,
                &mut peripherals.RESETS,
                &mut peripherals.DMA,
                i2c,
                delay,
                events,
                time,
                watchdog,
            ) && fs.has_files_to_offload()
            {
                return Err(());
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_lines)]
pub fn schedule_audio_rec(
    delay: &mut Delay,
    time: &SyncedDateTime,
    i2c: &mut SharedI2C,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    config: &DeviceConfig,
) -> Result<DateTime<Utc>, ()> {
    if let Err(err) = i2c.disable_alarm(delay) {
        error!("Failed to disable alarm");
        return Err(());
    }
    let mut wakeup: DateTime<Utc>;
    let seed: u64;
    let current_time = time.get_date_time();

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
        let Ok(ts_seconds) = u64::try_from(time.get_date_time().timestamp()) else {
            error!("Failed to convert current_time timestamp to u64");
            return Err(());
        };

        seed = ts_seconds;
        wakeup = current_time;
    }
    let mut rng = RNG::<WyRand, u16>::new(seed);
    let r_max: u16 = 65535u16;
    let short_chance: u16 = r_max / 4;
    let short_pause: i64 = 2 * 60;
    let short_window: i64 = 5 * 60;
    let long_pause: i64 = 40 * 60;
    let long_window: i64 = 20 * 60;

    // If a seed is set, always start alarm from 0am to keep consistent across devices.
    // So will need to generate numbers until the alarm is valid
    while wakeup <= current_time {
        let r = rng.generate();
        let wake_in = if DEV_MODE {
            120
        } else if r <= short_chance {
            short_pause + (i64::from(r) * short_window) / i64::from(short_chance)
        } else {
            long_pause + (i64::from(r) * long_window) / i64::from(r_max)
        };
        wakeup += chrono::Duration::seconds(wake_in);
    }
    info!(
        "Set alarm, current time is {}:{} Next alarm is {}:{}",
        time.get_date_time().time().hour(),
        time.get_date_time().time().minute(),
        wakeup.hour(),
        wakeup.minute()
    );

    let mut alarm_mode = AlarmMode::Audio;
    let audio_mode = config.audio_mode();
    match audio_mode {
        AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
            let (start, end) = config.next_or_current_recording_window(&current_time);
            info!(
                "Checking next alarm {}:{} for rec window start{}:{}",
                wakeup.hour(),
                wakeup.minute(),
                start.hour(),
                start.minute(),
            );
            if wakeup >= start {
                if start < current_time {
                    if let AudioMode::AudioAndThermal = audio_mode {
                        //audio recording inside recording window
                        info!("Scheduling audio inside thermal window");
                    } else {
                        info!("Already in rec window so restart now");
                        return Err(());
                    }
                } else {
                    info!("Setting wake up to be start of next rec window");
                    wakeup = start;
                    alarm_mode = AlarmMode::Thermal;
                }
            }
        }
        _ => (),
    }

    if let Err(err) = i2c.enable_alarm(delay) {
        error!("Failed to enable alarm");
        return Err(());
    }
    if i2c.set_wakeup_alarm(&wakeup, delay).is_ok() {
        if let Ok(alarm_enabled) = i2c.alarm_interrupt_enabled(delay) {
            if alarm_enabled {
                events.log(Event::SetAlarm(wakeup.timestamp_micros()), time, fs);

                write_audio_alarm(fs, wakeup, alarm_mode);
                return Ok(wakeup);
            }
        }
    } else {
        error!("Failed setting wake alarm, can't go to sleep");
    }
    Err(())
}

pub fn check_alarm_still_valid_with_thermal_window(
    alarm: &DateTime<Utc>,
    now: &DateTime<Utc>,
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
