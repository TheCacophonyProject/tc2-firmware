use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use cortex_m::delay::Delay;
use defmt::{error, info, warn, Format};
use rp2040_hal::{Sio, Timer};

use crate::core1_sub_tasks::maybe_offload_flash_storage_and_events;
use crate::core1_task::{wake_raspberry_pi, SyncedDateTime};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};
use crate::pdm_microphone::PdmMicrophone;
use fugit::RateExtU32;
use picorand::{PicoRandGenerate, WyRand, RNG};

use crate::rp2040_flash::write_device_config_to_rp2040_flash;
use chrono::{NaiveDateTime, Timelike};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;

pub fn audio_task(
    i2c_config: I2CConfig,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    clock_freq: u32,
    existing_config: &mut DeviceConfig,
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
) {
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut shared_i2c = SharedI2C::new(i2c_config, &mut delay);
    info!(
        "Got device config {:?}, {}",
        existing_config,
        existing_config.device_name()
    );

    let mut synced_date_time: SyncedDateTime = SyncedDateTime::default();
    match shared_i2c.get_datetime(&mut delay) {
        Ok(now) => {
            synced_date_time.set(get_naive_datetime(now), &timer);
        }
        Err(_) => error!("Unable to get DateTime from RTC"),
    }
    // flash_storage.scan();
    let has_files = flash_storage.has_files_to_offload();
    info!("Has files?? {}", has_files);
    let mut event_logger: EventLogger = EventLogger::new(flash_storage);
    return;
    if (has_files
        && should_offload_audio_recordings(
            flash_storage,
            &mut delay,
            &mut shared_i2c,
            synced_date_time.date_time_utc,
            existing_config.config().last_offload,
        ))
    {
        info!("WAKING PI");
        if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
            let mut peripherals: Peripherals = unsafe { Peripherals::steal() };
            info!("MAYB OFF");
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
                existing_config.set_last_offload(synced_date_time.date_time_utc.timestamp_millis());
                // flash_storage.write_
            }
        }
    }
    return;

    take_recording(
        clock_freq,
        &synced_date_time,
        flash_storage,
        timer,
        gpio0,
        gpio1,
    );
    flash_storage.free_spi();
    // let mut peripherals: Peripherals = unsafe { Peripherals::steal() };

    // if maybe_offload_flash_storage_and_events(
    //     flash_storage,
    //     pi_spi,
    //     &mut peripherals.RESETS,
    //     &mut peripherals.DMA,
    //     clock_freq,
    //     &mut shared_i2c,
    //     &mut delay,
    //     timer,
    //     &mut event_logger,
    //     &synced_date_time,
    // ) {
    //     event_logger.log_event(
    //         LoggerEvent::new(
    //             LoggerEventKind::OffloadedRecording,
    //             synced_date_time.get_timestamp_micros(&timer),
    //         ),
    //         flash_storage,
    //     );
    //     existing_config.set_last_offload(synced_date_time.date_time_utc.timestamp_millis());
    //     // flash_storage.write_
    // }
    // schedule_audio_rec(
    //     &mut delay,
    //     &synced_date_time,
    //     &mut shared_i2c,
    //     flash_storage,
    //     timer,
    // );
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
        1,
        dma_channels.ch3,
        dma_channels.ch4,
        timer,
        &mut peripherals.RESETS,
        peripherals.SPI1,
        flash_storage,
        timestamp,
    );
    // flash_storage.scan();
    let has_files = flash_storage.has_files_to_offload();
    info!("Has files?? {}", has_files);
}

fn schedule_audio_rec(
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
    i2c: &mut SharedI2C,
    flash_storage: &mut OnboardFlash,
    timer: &mut Timer,
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
        let mut event_logger: EventLogger = EventLogger::new(flash_storage);

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
    let alarm_enabled = i2c.alarm_interrupt_enabled();
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