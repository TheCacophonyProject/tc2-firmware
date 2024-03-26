use core::ops::DerefMut;

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::{bsp, event_logger};
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use rp2040_hal::{Sio, Timer};

use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    maybe_offload_flash_storage_and_events,
};
use crate::core1_task::{advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi, SyncedDateTime};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::OnboardFlash;
use crate::pdm_microphone::PdmMicrophone;

use chrono::{Duration, NaiveDateTime, NaiveTime, Timelike};
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};
use pcf8563::DateTime;

use cortex_m::asm::wfe;
use picorand::{PicoRandGenerate, WyRand, RNG};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;

pub fn audio_task(
    i2c_config: I2CConfig,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    clock_freq: u32,
    mut device_config: DeviceConfig,
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
) -> ! {
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
    let sio = Sio::new(peripherals.SIO);

    let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);

    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut shared_i2c = SharedI2C::new(i2c_config, &mut delay);
    info!(
        "Got device config {:?}, {}",
        device_config,
        device_config.device_name()
    );
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
    let mut event_logger: EventLogger = EventLogger::new(flash_storage);
    if should_offload_audio_recordings(
        flash_storage,
        &mut event_logger,
        &mut delay,
        &mut shared_i2c,
        synced_date_time.date_time_utc,
        device_config.config().last_offload,
    ) {
        watchdog.feed();
        if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
            // watchdog.disable(); //should watch dog go into offload
            if maybe_offload_flash_storage_and_events(
                flash_storage,
                pi_spi,
                &mut peripherals.RESETS,
                &mut peripherals.DMA,
                clock_freq,
                &mut shared_i2c,
                &mut delay,
                timer,
                &mut event_logger,
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
            }
            let (update_config, device_config_was_updated) =
                get_existing_device_config_or_config_from_pi_on_initial_handshake(
                    flash_storage,
                    pi_spi,
                    &mut peripherals.RESETS,
                    &mut peripherals.DMA,
                    clock_freq.Hz(),
                    2u32, //need to get radiometry and leton serial
                    1,
                    Some(synced_date_time.date_time_utc.timestamp_millis()),
                    timer,
                    Some(device_config),
                );
            device_config = update_config.unwrap();
            info!(
                "Updating last off load to {}",
                device_config.config().last_offload
            );
            // watchdog.pause_on_debug(true);
            // watchdog.start(8388607.micros());
        }
    }
    let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
    let mut do_recording = alarm_triggered;
    let scheduled = shared_i2c.is_alarm_set();
    let mut reschedule = !scheduled;
    let mut alarm_hours = shared_i2c.get_alarm_hours();
    let mut alarm_minutes = shared_i2c.get_alarm_minutes();
    if !alarm_triggered && scheduled {
        // check we haven't missed the alarm somehow
        let until_alarm = minutes_to_alarm(alarm_hours, alarm_minutes, &mut shared_i2c, &mut delay);
        if until_alarm < 0 || until_alarm > 60 {
            info!(
                "Missed alarm was scheduled for {}:{} but its {} minutes away",
                alarm_hours, alarm_minutes, until_alarm
            );

            // should take recording now
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::Rp2040MissedAudioAlarm,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
            do_recording = true;
        }
    }
    if alarm_triggered {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::Rp2040WokenByAlarm,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
    }

    if do_recording {
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
            microphone.record_for_n_seconds(
                5,
                dma_channels.ch3,
                dma_channels.ch4,
                timer,
                &mut peripherals.RESETS,
                peripherals.SPI1,
                flash_storage,
                timestamp,
                watchdog,
            );
            let _ = shared_i2c
                .set_recording_flag(&mut delay, false)
                .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));
        }
        shared_i2c.clear_alarm();
        reschedule = true;
        if let Some(err) = shared_i2c.set_minutes(0, &mut delay).err() {
            warn!("Could not clear alarm {}", err);
        }
    }

    if reschedule {
        info!("Scheduling new recording");
        schedule_audio_rec(
            &mut delay,
            &synced_date_time,
            &mut shared_i2c,
            flash_storage,
            timer,
            &mut event_logger,
        );
        alarm_hours = shared_i2c.get_alarm_hours();
        alarm_minutes = shared_i2c.get_alarm_minutes();
    }
    let until_alarm = minutes_to_alarm(alarm_hours, alarm_minutes, &mut shared_i2c, &mut delay);

    info!(
        "Recording scheduled for {}:{} in {} minutes",
        alarm_hours, alarm_minutes, until_alarm
    );
    let mut should_sleep = true;
    let mut logged_power_down = false;
    loop {
        info!(
            "Recording scheduled for {}:{}",
            shared_i2c.get_alarm_hours(),
            shared_i2c.get_alarm_minutes(),
        );
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
                    let until_alarm =
                        minutes_to_alarm(alarm_hours, alarm_minutes, &mut shared_i2c, &mut delay);
                    if until_alarm < 1 {
                        // otherwise the alarm could trigger  between here and sleeping
                        should_sleep = false;
                        info!("Alarm is scheduled in {} so not sleeping", until_alarm);
                        continue;
                    }
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
        }
        let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
        if alarm_triggered {
            info!("Alarm triggered after taking a recording reseeting rp2040");
            loop {
                // wait for watchdog to reset rp2040
                wfe();
            }
        }

        delay.delay_ms(5 * 1000);
        watchdog.feed();
    }
}

pub fn get_alarm_dt(
    datetime: DateTime,
    alarm_hours: u8,
    alarm_minutes: u8,
) -> Result<NaiveDateTime, ()> {
    let mut naive_date = chrono::NaiveDate::from_ymd_opt(
        2000 + datetime.year as i32,
        datetime.month as u32,
        datetime.day as u32,
    );
    if naive_date.is_none() {
        return Err(());
    }
    let mut naive_date = naive_date.unwrap();
    if alarm_hours < datetime.hours {
        naive_date = naive_date + Duration::days(1);
    }
    let naive_time = chrono::NaiveTime::from_hms_opt(alarm_hours as u32, alarm_minutes as u32, 0);
    if naive_time.is_none() {
        return Err(());
    }
    Ok(NaiveDateTime::new(naive_date, naive_time.unwrap()))
}

fn minutes_to_alarm(
    alarm_hours: u8,
    alarm_minutes: u8,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
) -> i64 {
    let mut synced_date_time = SyncedDateTime::default();
    match shared_i2c.get_datetime(delay) {
        Ok(now) => {
            let day = now.day;
            match get_alarm_dt(now, alarm_hours, alarm_minutes) {
                Ok(alarm_dt) => {
                    let now = get_naive_datetime(now);
                    let until_alarm = alarm_dt - now;
                    return until_alarm.num_minutes();
                }
                Err(_) => error!(
                    "Could not get alarm dt for {} {}",
                    alarm_hours, alarm_minutes
                ),
            }
        }
        Err(_) => error!("Unable to get DateTime from RTC"),
    }
    return 0;
}
fn schedule_audio_rec(
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    i2c: &mut SharedI2C,
    flash_storage: &mut OnboardFlash,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
) {
    i2c.enable_alarm(delay);

    let mut rng = RNG::<WyRand, u16>::new(synced_date_time.date_time_utc.timestamp() as u64);
    let mut r = rng.generate();
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
    info!(
        "Scheduling to wake up in {} seconds random was {}",
        wake_in, r
    );
    info!(
        "Current time is {}:{}",
        synced_date_time.date_time_utc.time().hour(),
        synced_date_time.date_time_utc.time().minute()
    );

    // GP TESTING
    let wake_in: i32 = 60 * 5;
    let wakeup = synced_date_time.date_time_utc + chrono::Duration::seconds(wake_in as i64);
    info!(
        "Wake up is set to be {}:{}",
        wakeup.time().hour(),
        wakeup.time().minute()
    );
    if let Ok(_) = i2c.set_wakeup_alarm(&wakeup, delay) {
        let alarm_enabled = i2c.alarm_interrupt_enabled();
        info!("Wake up alarm interrupt enabled {}", alarm_enabled);

        info!(
            "Recording scheduled for {}:{}",
            i2c.get_alarm_hours(),
            i2c.get_alarm_minutes(),
        );
        if alarm_enabled {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::SetAlarm(wakeup.timestamp_micros() as u64),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
        }
    } else {
        error!("Failed setting wake alarm, can't go to sleep");
    }
}

fn should_offload_audio_recordings(
    flash_storage: &mut OnboardFlash,
    event_logger: &mut EventLogger,
    delay: &mut Delay,
    i2c: &mut SharedI2C,
    now: NaiveDateTime,
    last_offload: i64,
) -> bool {
    let offload_hour = 10;
    let has_files = flash_storage.has_files_to_offload() || event_logger.is_nearly_full();
    if !has_files {
        return false;
    }
    // not woken from alarm
    let alarm_enabled: bool = i2c.alarm_interrupt_enabled();
    if !alarm_enabled {
        info!("Offloading because not woken by alarm");
        return true;
    }
    // offloaded in last hour
    let has_offloaded = (now.timestamp_millis() - last_offload) < 60 * 60 * 1000;
    if (now.hour() >= offload_hour) && !has_offloaded {
        info!(
            "Waking up because need to offload after {} and its {}last offload {} now {}",
            offload_hour,
            now.hour(),
            last_offload,
            now.timestamp_millis(),
        );
        return true;
    }

    // if pi is on
    if let Ok(pi_is_awake) = i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
        if pi_is_awake {
            info!("Offloading because pi is awake");
            return true;
        }
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
