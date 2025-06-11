use crate::attiny_rtc_i2c::{tc2_agent_state, SharedI2C};
use crate::bsp::pac::Peripherals;
use crate::device_config::{get_naive_datetime, AudioMode, DeviceConfig};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::ext_spi_transfers::ExtSpiTransfers;
use crate::frame_processing::{wake_raspberry_pi, SyncedDateTime};
use crate::onboard_flash::OnboardFlash;
use crate::sub_tasks::get_existing_device_config_or_config_from_pi_on_initial_handshake;
use cortex_m::delay::Delay;
use defmt::{info, panic, warn};
use fugit::HertzU32;
use rp2040_hal::Timer;

pub fn should_record_audio(
    config: &DeviceConfig,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    synced_date_time: &SyncedDateTime,
) -> bool {
    let mut is_audio: bool = config.config().is_audio_device();

    if let Ok(audio_only) = shared_i2c.is_audio_device(delay) {
        info!("EEPROM audio device: {}", audio_only);
        is_audio = is_audio || audio_only;
    }
    if is_audio {
        match config.config().audio_mode {
            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                if let Ok(state) = shared_i2c.tc2_agent_state(delay) {
                    if (state & (tc2_agent_state::THERMAL_MODE)) > 0 {
                        let _ = shared_i2c.tc2_agent_clear_thermal_mode(delay);
                        info!("Audio request thermal mode");
                        // audio mode wants to go in thermal mode
                        is_audio = false;
                    } else {
                        let in_window = config
                            .time_is_in_recording_window(&synced_date_time.date_time_utc, &None);
                        if in_window {
                            is_audio = (state
                                & (tc2_agent_state::LONG_AUDIO_RECORDING
                                    | tc2_agent_state::TAKE_AUDIO
                                    | tc2_agent_state::TEST_AUDIO_RECORDING))
                                > 0;
                            if is_audio {
                                info!("Is audio because thermal requested or test rec");
                            }
                        }
                    }
                }
            }
            _ => (),
        }
    }
    is_audio
}

pub fn get_device_config(
    flash_storage: &mut OnboardFlash,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    pi_spi: &mut ExtSpiTransfers,
    system_clock_freq_hz: HertzU32,
    timer: &mut Timer,
) -> DeviceConfig {
    let mut peripherals = unsafe { Peripherals::steal() };
    let config: Option<DeviceConfig> = DeviceConfig::load_existing_config_from_flash(flash_storage);
    match &config {
        Some(device_config) => {
            info!("Existing config {:#?}", device_config.config());
        }
        None => {
            info!("Waking pi to get config");
            // We need to wake up the rpi and get a config
            wake_raspberry_pi(shared_i2c, delay);
        }
    }

    // FIXME: This should become "config startup handshake", and we have another
    //  camera config handshake later on when the lepton has actually initialised or whatever.
    let (device_config, device_config_was_updated) =
        get_existing_device_config_or_config_from_pi_on_initial_handshake(
            flash_storage,
            pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            system_clock_freq_hz,
            2u32,  // radiometry enabled
            0,     // camera serial
            false, // audio mode
            timer,
            config,
        );
    match device_config {
        Some(device_config) => device_config,
        None => {
            panic!("Couldn't get config");
        }
    }
}

pub fn get_synced_time(
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    event_logger: &mut EventLogger,
    flash_storage: &mut OnboardFlash,
    timer: &mut Timer,
) -> SyncedDateTime {
    let mut synced_date_time = SyncedDateTime::default();
    loop {
        // NOTE: Keep retrying until we get a datetime from RTC.
        match shared_i2c.get_datetime(delay) {
            Ok(now) => {
                info!("Date time {}:{}:{}", now.hours, now.minutes, now.seconds);
                synced_date_time.set(get_naive_datetime(now), &timer);
                break;
            }
            Err(e) => {
                // cant get time so use 0 and add a time when tc2-agent uploads
                event_logger.log_event(
                    LoggerEvent::new(LoggerEventKind::RtcCommError, 0),
                    flash_storage,
                );

                warn!("Failed getting date from RTC, retrying");
                delay.delay_ms(10);
            }
        }
    }
    synced_date_time
}
