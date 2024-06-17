use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    offload_flash_storage_and_events,
};
use crate::core1_task::Core1Pins;
use crate::core1_task::{advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi, SyncedDateTime};
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
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

use crate::rp2040_flash::{read_alarm_from_rp2040_flash, write_alarm_schedule_to_rp2040_flash};

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
    watchdog.feed();
    let mut device_config: DeviceConfig = DeviceConfig::load_existing_config_from_flash().unwrap();

    //this isn't reliable so use alarm stored in flash
    // let mut alarm_hours = shared_i2c.get_alarm_hours();
    // let mut alarm_minutes = shared_i2c.get_alarm_minutes();
    let flash_alarm = read_alarm_from_rp2040_flash();
    let alarm_day = flash_alarm[0];
    let alarm_hours = flash_alarm[1];
    let alarm_minutes = flash_alarm[2];
    info!(
        "Alarm day {} hours {} minutes {} ",
        alarm_day, alarm_hours, alarm_minutes
    );

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

    // Unset the is_recording flag on attiny on startup
    let _ = shared_i2c
        .set_recording_flag(&mut delay, false)
        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));

    let mut synced_date_time = SyncedDateTime::default();
    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
            synced_date_time.set(get_naive_datetime(now), &timer);
        }
        Err(_) => error!("Unable to get DateTime from RTC"),
    }

    let mut scheduled: bool =
        alarm_day != u8::MAX && alarm_hours != u8::MAX && alarm_minutes != u8::MAX;

    let mut event_logger: EventLogger = EventLogger::new(&mut flash_storage);
    watchdog.feed();
    let should_wake = should_offload_audio_recordings(
        &mut flash_storage,
        &mut event_logger,
        &mut delay,
        &mut shared_i2c,
        synced_date_time.date_time_utc,
    );
    match offload(
        &mut shared_i2c,
        clock_freq,
        &mut flash_storage,
        &mut pi_spi,
        timer,
        &mut event_logger,
        should_wake,
        device_config,
        &mut delay,
        &synced_date_time,
        watchdog,
    ) {
        Ok((new_config, was_updated)) => {
            device_config = new_config.unwrap();
            if was_updated {
                if !device_config.config().is_audio_device {
                    shared_i2c.disable_alarm(&mut delay);
                    info!("Not audio device so restarting");
                    watchdog.start(100.micros());
                    loop {
                        // Wait to be reset and become thermal device
                        nop();
                    }
                }
            }
        }
        Err(()) => {
            warn!("Restarting as could not offload");
            watchdog.start(100.micros());
            loop {
                // Wait to be reset and become thermal device
                nop();
            }
        }
    }

    let mut do_recording = alarm_triggered;
    let mut reschedule = !scheduled;
    info!(
        "ALarm triggered {} scheduled {}",
        alarm_triggered, scheduled
    );
    if !alarm_triggered && scheduled {
        // check we haven't missed the alarm somehow
        match get_alarm_dt(
            synced_date_time.get_adjusted_dt(timer),
            alarm_day,
            alarm_hours,
            alarm_minutes,
        ) {
            Ok(alarm_dt) => {
                let synced = synced_date_time.get_adjusted_dt(timer);
                let until_alarm =
                    (alarm_dt - synced_date_time.get_adjusted_dt(timer)).num_minutes();
                if until_alarm <= 0 || until_alarm > 60 {
                    info!(
                        "Missed alarm was scheduled for the {} at {}:{} but its {} minutes away",
                        alarm_day, alarm_hours, alarm_minutes, until_alarm
                    );

                    // should take recording now
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::Rp2040MissedAudioAlarm(
                                alarm_dt.timestamp_micros() as u64
                            ),
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
                        &mut flash_storage,
                    );
                    do_recording = true;
                }
            }
            Err(_) => {
                error!(
                    "Could not get alarm dt for {} {} {}",
                    alarm_day, alarm_hours, alarm_minutes
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
    let mut take_test_rec = false;
    if let Ok(test_rec) = shared_i2c.tc2_agent_requested_audio_rec(&mut delay) {
        take_test_rec = test_rec;
    }
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
                    LoggerEventKind::StartedRecording,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
            let recorded = microphone.record_for_n_seconds(
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
                loop {
                    nop();
                }
            } else {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::EndedRecording,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
            }
        }
        if do_recording && !take_test_rec {
            shared_i2c.clear_alarm(&mut delay);
            reschedule = true;
            write_alarm_schedule_to_rp2040_flash(u8::MAX, u8::MAX, u8::MAX);
        } else {
            info!("taken test recoridng clearing status");
            let _ = shared_i2c.tc2_agent_clear_test_audio_rec(&mut delay);
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
        ) {
            write_alarm_schedule_to_rp2040_flash(
                scheduled_time.day() as u8,
                scheduled_time.hour() as u8,
                scheduled_time.minute() as u8,
            );
            alarm_time = Some(scheduled_time);
            scheduled = true;
        } else {
            error!("Couldn't schedule alarm will restart");
            write_alarm_schedule_to_rp2040_flash(u8::MAX, u8::MAX, u8::MAX);
            watchdog.start(100.micros());
            loop {
                nop();
            }
        }
    } else {
        match get_alarm_dt(
            synced_date_time.get_adjusted_dt(timer),
            alarm_day,
            alarm_hours,
            alarm_minutes,
        ) {
            Ok(alarm_dt) => {
                alarm_time = Some(alarm_dt);
            }
            Err(_) => {
                alarm_time = None;
                should_sleep = false;
                error!(
                    "Could not get alarm dt for {} {}",
                    alarm_hours, alarm_minutes
                )
            }
        }
    }
    if alarm_time.is_some() {
        info!(
            "Recording scheduled for the {} at {}:{} ",
            alarm_time.as_mut().unwrap().day(),
            alarm_time.as_mut().unwrap().time().hour(),
            alarm_time.as_mut().unwrap().time().minute()
        );
    }
    if do_recording && take_test_rec {
        //means we didn't take the test rec as it was a normal rec time
        watchdog.start(100.micros());
        loop {
            nop();
        }
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
        watchdog.feed();
        if should_sleep {
            if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(&mut delay, true) {
                if pi_is_powered_down {
                    if alarm_time.is_some() {
                        let until_alarm = (alarm_time.unwrap()
                            - synced_date_time.get_adjusted_dt(timer))
                        .num_minutes();

                        if until_alarm < 1 {
                            // otherwise the alarm could trigger  between here and sleeping
                            should_sleep = false;
                            info!("Alarm is scheduled in {} so not sleeping", until_alarm);
                            if until_alarm <= 0 {
                                watchdog.start(100.micros());
                                loop {
                                    // Wait to be reset and become thermal device
                                    nop();
                                }
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
                        should_wake,
                        device_config,
                        &mut delay,
                        &synced_date_time,
                        watchdog,
                    ) {
                        Ok((new_config, was_updated)) => {
                            device_config = new_config.unwrap();
                            if was_updated {
                                if !device_config.config().is_audio_device {
                                    info!("Not audio device so restarting");
                                    watchdog.start(100.micros());
                                    loop {
                                        // Wait to be reset and become thermal device
                                        nop();
                                    }
                                }
                            }
                        }
                        Err(()) => {
                            warn!("Restarting as failed to offload");
                            watchdog.start(100.micros());
                            loop {
                                // Wait to be reset and become thermal device
                                nop();
                            }
                        }
                    }
                }
            }
        }

        let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
        if alarm_triggered {
            info!("Alarm triggered after taking a recording reseeting rp2040");
            watchdog.start(100.micros());
            loop {
                // wait for watchdog to reset rp2040
                nop();
            }
        }

        delay.delay_ms(5 * 1000);
        watchdog.feed();
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
    device_config: DeviceConfig,
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    watchdog: &mut bsp::hal::Watchdog,
) -> Result<(Option<DeviceConfig>, bool), ()> {
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
            if offload_flash_storage_and_events(
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
            ) {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::OffloadedRecording,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
            } else if flash_storage.has_files_to_offload() {
                return Err(());
            }
            watchdog.feed();
            let (update_config, device_config_was_updated) =
                get_existing_device_config_or_config_from_pi_on_initial_handshake(
                    flash_storage,
                    pi_spi,
                    &mut peripherals.RESETS,
                    &mut peripherals.DMA,
                    clock_freq.Hz(),
                    2u32, //need to get radiometry and leton serial
                    1,
                    timer,
                    Some(device_config),
                );
            return Ok((update_config, device_config_was_updated));
        }
    }
    Ok((Some(device_config), false))
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

fn schedule_audio_rec(
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    i2c: &mut SharedI2C,
    flash_storage: &mut OnboardFlash,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
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
    let wake_in;
    if r <= short_chance {
        wake_in = (short_pause + (r as u64 * short_window) / short_chance as u64) as u64;
    } else {
        wake_in = (long_pause + (r as u64 * long_window) / r_max as u64) as u64;
    }
    info!(
        "Scheduling to wake up in {} seconds random was {}",
        wake_in, r
    );
    info!(
        "Current time is {}:{}",
        synced_date_time.date_time_utc.time().hour(),
        synced_date_time.date_time_utc.time().minute()
    );

    let wakeup = synced_date_time.date_time_utc + chrono::Duration::seconds(wake_in as i64);
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
