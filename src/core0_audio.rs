use core::ops::DerefMut;

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp;
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use rp2040_hal::{Sio, Timer};

use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_events,
    maybe_offload_flash_storage_and_events,
};
use crate::core1_task::{wake_raspberry_pi, SyncedDateTime};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};
use crate::pdm_microphone::PdmMicrophone;
use crate::rp2040_flash::write_device_config_to_rp2040_flash;
use chrono::{NaiveDateTime, Timelike};
use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};
use fugit::{ExtU32, RateExtU32};

use picorand::{PicoRandGenerate, WyRand, RNG};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;

pub fn audio_task(
    i2c_config: I2CConfig,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    clock_freq: u32,
    device_config: DeviceConfig,
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
) {
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut shared_i2c = SharedI2C::new(i2c_config, &mut delay);
    info!(
        "Got device config {:?}, {}",
        device_config,
        device_config.device_name()
    );
    shared_i2c.print_alarm_status(&mut delay);

    let mut synced_date_time = SyncedDateTime::default();
    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
            synced_date_time.set(get_naive_datetime(now), &timer);
        }
        Err(_) => error!("Unable to get DateTime from RTC"),
    }
    // flash_storage.scan();
    let mut event_logger: EventLogger = EventLogger::new(flash_storage);
    event_logger.log_event(
        LoggerEvent::new(
            LoggerEventKind::Rp2040Woken,
            synced_date_time.get_timestamp_micros(&timer),
        ),
        flash_storage,
    );

    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
    let has_files = flash_storage.has_files_to_offload();

    if (has_files
        && should_offload_audio_recordings(
            flash_storage,
            &mut delay,
            &mut shared_i2c,
            synced_date_time.date_time_utc,
            device_config.config().last_offload,
        ))
    {
        watchdog.feed();
        if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
            watchdog.disable(); //should watch dog go into offload
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
            ) {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::OffloadedRecording,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );

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
            }
            watchdog.pause_on_debug(true);
            watchdog.start(8388607.micros());
        }
    }
    let alarm_triggered: bool = shared_i2c.alarm_triggered(&mut delay);
    let mut scheduled = shared_i2c.is_alarm_set();

    if alarm_triggered {
        take_recording(
            clock_freq,
            &synced_date_time,
            flash_storage,
            timer,
            gpio0,
            gpio1,
            watchdog,
        );
        shared_i2c.clear_alarm();
        scheduled = false;
        if let Some(err) = shared_i2c.set_minutes(0, &mut delay).err() {
            warn!("Could not clear alarm {}", err);
        }
    }

    info!("Alarm scheduled? {}", scheduled);
    info!("Alarm hours is {}", shared_i2c.get_alarm_hours());
    if alarm_triggered || !scheduled {
        info!("Scheduling new recording");
        schedule_audio_rec(
            &mut delay,
            &synced_date_time,
            &mut shared_i2c,
            flash_storage,
            timer,
            &mut event_logger,
        );
    } else {
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

fn take_recording(
    system_clock_freq: u32,
    synced_date_time: &SyncedDateTime,
    flash_storage: &mut OnboardFlash,
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
) {
    let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
    let sio = Sio::new(peripherals.SIO);

    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
    let mut microphone = PdmMicrophone::new(
        gpio0.into_function().into_pull_type(),
        gpio1.into_function().into_pull_type(),
        system_clock_freq.Hz(),
        pio1,
        sm1,
    );
    let timestamp = synced_date_time.get_timestamp_micros(&timer);
    microphone.record_for_n_seconds(
        60,
        dma_channels.ch3,
        dma_channels.ch4,
        timer,
        &mut peripherals.RESETS,
        peripherals.SPI1,
        flash_storage,
        timestamp,
        watchdog,
    );
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
    let mut wake_in = 0u64;
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
    info!(
        "Wake up is set to be {}:{}",
        wakeup.time().hour(),
        wakeup.time().minute()
    );
    if let Ok(_) = i2c.set_wakeup_alarm(&wakeup, delay) {
        let alarm_enabled = i2c.alarm_interrupt_enabled();
        info!("Wake up alarm interrupt enabled {}", alarm_enabled);

        if alarm_enabled {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::SetAlarm(wakeup.timestamp_micros() as u64),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );

            info!("Ask Attiny to power down rp2040");
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::Rp2040Sleep,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );

            if let Ok(_) = i2c.tell_attiny_to_power_down_rp2040(delay) {
                info!("Sleeping");
            } else {
                error!("Failed sending sleep request to attiny");
            }
        }
    } else {
        error!("Failed setting wake alarm, can't go to sleep");
    }
}

fn should_offload_audio_recordings(
    flash_storage: &mut OnboardFlash,
    delay: &mut Delay,
    i2c: &mut SharedI2C,
    now: NaiveDateTime,
    last_offload: i64,
) -> bool {
    let offload_hour = 10;

    // not woken from alarm
    let alarm_enabled: bool = i2c.alarm_interrupt_enabled();
    if !alarm_enabled {
        info!("Offloading because not woken by alarm");
        return true;
    }
    // offloaded in last hour
    NaiveDateTime::from_timestamp_opt(last_offload, 0);
    let has_offloaded = (now.timestamp_micros() - last_offload) < 60 * 60 * 1000;
    if (now.hour() >= offload_hour) && !has_offloaded {
        info!(
            "Waking up because need to offload after {} and its {}",
            offload_hour,
            now.hour()
        );
        return true;
    }

    // if pi is on
    if let Ok(pi_is_awake) = i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
        info!("Offloading because pi is awake");
        return true;
    }

    // flash getting full
    let full = flash_storage.is_nearly_full();
    if full {
        info!("Offloading as flash is nearly full");
        return true;
    }
    return false;
}
