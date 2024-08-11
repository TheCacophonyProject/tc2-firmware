use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C, Tc2AgentState};
use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, offload_file,
    offload_flash_storage_and_events,
};
use crate::core1_task::Core1Pins;
use crate::core1_task::{advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi, SyncedDateTime};
use crate::device_config::{get_naive_datetime, AudioMode, DeviceConfig};
use crate::event_logger::{
    clear_audio_alarm, get_audio_alarm, write_audio_alarm, EventLogger, LoggerEvent,
    LoggerEventKind,
};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::OnboardFlash;
use crate::pdm_microphone::PdmMicrophone;
use cortex_m::delay::Delay;
use defmt::{error, info, warn};
use rp2040_hal::{Sio, Timer};

use chrono::{Datelike, NaiveDateTime, Timelike};
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

use crate::onboard_flash::extend_lifetime_generic_mut;

// use crate::rp2040_flash::{
//     clear_flash_alarm, read_alarm_from_rp2040_flash, write_alarm_schedule_to_rp2040_flash,
// };

const DEV_MODE: bool = false;
pub const MAX_GAP_MIN: u8 = 60;
pub fn audio_task(
    i2c_config: I2CConfig,
    clock_freq: u32,
    timer: &mut Timer,
    pins: Core1Pins,
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
) -> ! {
    let mut device_config = DeviceConfig::load_existing_config_from_flash().unwrap();

    watchdog.feed();

    let core = unsafe { pac::CorePeripherals::steal() };
    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

    let sio = Sio::new(peripherals.SIO);
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    // init flash
    let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
    let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };

    let mut flash_storage = OnboardFlash::new(
        pins.fs_cs,
        pins.fs_mosi,
        pins.fs_clk,
        pins.fs_miso,
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
        pins.pi_mosi,
        pins.pi_cs,
        pins.pi_clk,
        pins.pi_miso,
        pins.pi_ping,
        dma_channels.ch0,
        payload_buf,
        crc_buf,
        pio0,
        sm0,
    );
    let mut peripherals = unsafe { Peripherals::steal() };

    watchdog.feed();
    flash_storage.take_spi(peripherals.SPI1, &mut peripherals.RESETS, clock_freq.Hz());
    flash_storage.init();

    let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut shared_i2c = SharedI2C::new(i2c_config, unlocked_pin, &mut delay);

    let mut synced_date_time = SyncedDateTime::default();
    let mut event_logger: EventLogger = EventLogger::new(&mut flash_storage);

    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            synced_date_time.set(get_naive_datetime(now), &timer);
        }
        Err(e) => {
            // should take recording now
            event_logger.log_event(
                LoggerEvent::new(LoggerEventKind::RtcCommError, 2206224000000000),
                &mut flash_storage,
            );
            panic!("Unable to get DateTime from RTC {}", e)
        }
    }
    let mut take_test_rec = false;
    if let Ok(test_rec) = shared_i2c.tc2_agent_requested_test_audio_rec(&mut delay) {
        take_test_rec = test_rec;
    }
    let mut do_recording = alarm_triggered;
    let mut thermal_requested_audio = false;
    let mut reschedule = false;
    let mut alarm_date_time: Option<NaiveDateTime> = None;

    //do this so tc2-agent knows whats going on
    let (new_config, device_config_was_updated) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            &mut flash_storage,
            &mut pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            clock_freq.Hz(),
            2u32, //need to get radiometry and leton serial
            1,
            true,
            timer,
            Some(device_config),
        );
    device_config = new_config.unwrap();

    if device_config_was_updated {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::SavedNewConfig,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            &mut flash_storage,
        );
    }

    if !take_test_rec {
        if device_config_was_updated {
            let reboot;
            if device_config.config().is_audio_device() && !thermal_requested_audio {
                if let AudioMode::AudioOnly = device_config.config().audio_mode {
                    reboot = false;
                } else {
                    let in_window = device_config
                        .time_is_in_recording_window(&synced_date_time.date_time_utc, &None);
                    reboot = in_window;
                }
            } else {
                reboot = !device_config.config().is_audio_device()
            }

            if reboot {
                let _ = shared_i2c.disable_alarm(&mut delay);
                clear_audio_alarm(&mut flash_storage);
                info!("Restarting as should be in thermal mode");
                restart(watchdog);
            }
        }

        //this isn't reliable so use alarm stored in flash
        // let mut alarm_hours = shared_i2c.get_alarm_hours();
        // let mut alarm_minutes = shared_i2c.get_alarm_minutes();
        let flash_alarm = get_audio_alarm(&mut flash_storage);
        let mut scheduled = false;
        if let Some(flash_alarm) = flash_alarm {
            let alarm_day = flash_alarm[0];
            let alarm_hours = flash_alarm[1];
            let alarm_minutes = flash_alarm[2];
            let alarm_mode = flash_alarm[3];
            scheduled = alarm_day != u8::MAX && alarm_hours != u8::MAX && alarm_minutes != u8::MAX;

            info!(
                "Alarm day {} hours {} minutes {} in mode {}",
                alarm_day, alarm_hours, alarm_minutes, alarm_mode
            );
            if scheduled {
                match get_alarm_dt(
                    synced_date_time.get_adjusted_dt(timer),
                    alarm_day,
                    alarm_hours,
                    alarm_minutes,
                ) {
                    Ok(alarm) => {
                        if device_config_was_updated {
                            if check_alarm_and_maybe_clear(
                                &alarm,
                                &synced_date_time.get_adjusted_dt(timer),
                                &device_config,
                            ) {
                                //if window time changed and alarm is after rec window start
                                clear_audio_alarm(&mut flash_storage);
                                scheduled = false;
                                info!("Rescehduling as alarm is after window start");
                            } else {
                                alarm_date_time = Some(alarm);
                            }
                        }
                    }
                    Err(_) => {
                        error!(
                            "Could not get alarm dt for {} {} {}",
                            alarm_day, alarm_hours, alarm_minutes
                        );
                    }
                }
            }
        }

        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::AudioMode,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            &mut flash_storage,
        );
        // Unset the is_recording flag on attiny on startup
        if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
            if is_recording {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::RecordingNotFinished,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
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
                                LoggerEventKind::ToldRpiToWake(5),
                                synced_date_time.get_timestamp_micros(&timer),
                            ),
                            &mut flash_storage,
                        );
                    }
                }
            }
            _ => {}
        }
        if !should_wake {
            let wake_up = should_offload_audio_recordings(
                &mut flash_storage,
                &mut event_logger,
                &mut delay,
                &mut shared_i2c,
                synced_date_time.date_time_utc,
            );
            if wake_up {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::ToldRpiToWake(6),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
            }
            should_wake = wake_up;
        };

        if let Ok(take_rec) = shared_i2c.tc2_agent_requested_audio_rec(&mut delay) {
            thermal_requested_audio = take_rec;
            if thermal_requested_audio {
                do_recording = true;
            }
        } else {
            thermal_requested_audio = false;
        }
        match offload(
            &mut shared_i2c,
            clock_freq,
            &mut flash_storage,
            &mut pi_spi,
            timer,
            &mut event_logger,
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
            "ALarm triggered {} scheduled {}",
            alarm_triggered, scheduled
        );
        if !alarm_triggered && scheduled {
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
                        LoggerEventKind::Rp2040MissedAudioAlarm(alarm.timestamp_micros() as u64),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
                    do_recording = true;
                }
            }
        }
        if alarm_triggered {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::Rp2040WokenByAlarm,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
        }
    }
    let mut file_position = (0, 0);
    if do_recording || take_test_rec {
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
            let mut duration = 60;
            if !do_recording && take_test_rec {
                duration = 10;
            }
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::StartedAudioRecording(
                        flash_storage.current_block_index as u64,
                    ),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );

            let (recorded, pos, end_pos) = microphone.record_for_n_seconds(
                duration,
                dma_channels.ch3,
                dma_channels.ch4,
                timer,
                &mut peripherals.RESETS,
                peripherals.SPI1,
                &mut flash_storage,
                timestamp,
                watchdog,
                &synced_date_time,
            );
            file_position = pos;
            let _ = shared_i2c
                .set_recording_flag(&mut delay, false)
                .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));

            if !recorded {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::AudioRecordingFailed,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
                info!("Recording failed restarting and will try again");
                restart(watchdog);
            } else {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::EndedRecording(
                            ((end_pos.0 as u64) << 32) | end_pos.1 as u64,
                        ),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
                if take_test_rec {
                    info!("taken test recoridng clearing status");
                    watchdog.feed();
                    let _ = shared_i2c.tc2_agent_clear_and_set_flag(
                        &mut delay,
                        Tc2AgentState::TAKE_AUDIO,
                        Tc2AgentState::THERMAL_MODE,
                    );

                    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

                    offload_file(
                        &mut flash_storage,
                        &mut pi_spi,
                        &mut peripherals.RESETS,
                        &mut peripherals.DMA,
                        clock_freq,
                        &mut shared_i2c,
                        &mut delay,
                        timer,
                        &mut event_logger,
                        &synced_date_time,
                        Some(watchdog),
                        file_position,
                    );

                    restart(watchdog);
                } else {
                    shared_i2c.clear_alarm(&mut delay);
                    reschedule = true;
                    clear_audio_alarm(&mut flash_storage);

                    if thermal_requested_audio {
                        //if audio requested from thermal, the alarm will be re scheduled there
                        let _ = shared_i2c.tc2_agent_clear_and_set_flag(
                            &mut delay,
                            Tc2AgentState::TEST_AUDIO_RECORDING,
                            Tc2AgentState::THERMAL_MODE,
                        );
                        info!("Audio taken in thermal window clearing flag");
                        restart(watchdog);
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
            &mut flash_storage,
            timer,
            &mut event_logger,
            &device_config,
        ) {
            alarm_date_time = Some(scheduled_time);
        } else {
            error!("Couldn't schedule alarm will restart");
            clear_audio_alarm(&mut flash_storage);
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
    loop {
        advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
        if !logged_power_down {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToSleep,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
            logged_power_down = true;
        }

        match device_config.config().audio_mode {
            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                if let Ok(()) = shared_i2c.tc2_agent_request_thermal_mode(&mut delay) {
                    info!("Going into thermal mode");
                    restart(watchdog);
                }
            }
            _ => (),
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
                    info!("Ask Attiny to power down rp2040");
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::Rp2040Sleep,
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
                        &mut flash_storage,
                    );

                    if let Ok(_) = shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay) {
                        info!("Sleeping");
                    } else {
                        error!("Failed sending sleep request to attiny");
                    }
                }

                //may aswell offload again if we are awake and have just taken a recording
                if !pi_is_powered_down && flash_storage.has_files_to_offload() {
                    match offload(
                        &mut shared_i2c,
                        clock_freq,
                        &mut flash_storage,
                        &mut pi_spi,
                        timer,
                        &mut event_logger,
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
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToWake(7),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
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
            ) && flash_storage.has_files_to_offload()
            {
                return Err(());
            }
        }
    }
    Ok(())
}
pub fn get_alarm_dt(
    datetime: NaiveDateTime,
    alarm_day: u8,
    alarm_hours: u8,
    alarm_minutes: u8,
) -> Result<NaiveDateTime, ()> {
    let naive_date = chrono::NaiveDate::from_ymd_opt(
        datetime.year() as i32,
        datetime.month() as u32,
        alarm_day as u32,
    );
    if naive_date.is_none() {
        return Err(());
    }
    let naive_date = naive_date.unwrap();

    let naive_time = chrono::NaiveTime::from_hms_opt(alarm_hours as u32, alarm_minutes as u32, 0);
    if naive_time.is_none() {
        return Err(());
    }
    Ok(NaiveDateTime::new(naive_date, naive_time.unwrap()))
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
    let mut rng = RNG::<WyRand, u16>::new(synced_date_time.date_time_utc.timestamp() as u64);
    let r = rng.generate();
    let r_max: u16 = 65535u16;
    let short_chance: u16 = r_max / 4;
    let short_pause: u64 = 2 * 60;
    let short_window: u64 = 5 * 60;
    let long_pause: u64 = 40 * 60;
    let long_window: u64 = 20 * 60;
    let mut wake_in;
    if r <= short_chance {
        wake_in = (short_pause + (r as u64 * short_window) / short_chance as u64) as u64;
    } else {
        wake_in = (long_pause + (r as u64 * long_window) / r_max as u64) as u64;
    }
    if DEV_MODE {
        wake_in = 120;
    }
    let current_time = synced_date_time.get_adjusted_dt(timer);
    let mut wakeup = current_time + chrono::Duration::seconds(wake_in as i64);
    let mut alarm_mode = 0u8;

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
            if wakeup > start {
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
                    alarm_mode = 1u8;
                }
            }
        }
        _ => (),
    }
    info!(
        "Current time is {}:{}",
        synced_date_time.date_time_utc.time().hour(),
        synced_date_time.date_time_utc.time().minute()
    );

    if let Err(err) = i2c.enable_alarm(delay) {
        error!("Failed to enable alarm");
        return Err(());
    }
    if let Ok(_) = i2c.set_wakeup_alarm(&wakeup, delay) {
        if let Ok(alarm_enabled) = i2c.alarm_interrupt_enabled(delay) {
            if alarm_enabled {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::SetAlarm(wakeup.timestamp_micros() as u64),
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );

                write_audio_alarm(
                    flash_storage,
                    wakeup.day() as u8,
                    wakeup.hour() as u8,
                    wakeup.minute() as u8,
                    alarm_mode,
                );
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
    let offload_hour = 10;
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

    return false;
}

pub fn check_alarm_and_maybe_clear(
    alarm: &NaiveDateTime,
    now: &NaiveDateTime,
    device_config: &DeviceConfig,
) -> bool {
    if device_config.config().audio_mode != AudioMode::AudioOnly {
        let (start, end) = device_config.next_or_current_recording_window(now);
        return alarm > &start && &start > now;
    }
    false
}
