#![no_std]
#![no_main]
#![warn(clippy::all, clippy::pedantic)]

mod attiny_rtc_i2c;
mod audio_task;
mod bsp;
mod byte_slice_cursor;
mod clock_utils;
mod cptv_encoder;
mod device_config;
mod event_logger;
mod ext_spi_transfers;
mod formatted_time;
mod frame_processing;
mod lepton;
mod lepton_task;
mod lepton_telemetry;
mod motion_detector;
mod onboard_flash;
mod pdm_filter;
mod pdm_microphone;
mod rpi_power;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod synced_date_time;
mod utils;

use crate::attiny_rtc_i2c::{CameraState, MainI2C, RecordingMode};
use crate::audio_task::record_audio;
use crate::event_logger::{Event, EventLogger};
use crate::ext_spi_transfers::{
    ExtSpiTransfers, RPI_PAYLOAD_LENGTH, RPI_RETURN_PAYLOAD_LENGTH, RPI_TRANSFER_HEADER_LENGTH,
};
use crate::frame_processing::{THERMAL_DEV_MODE, record_thermal};
use crate::lepton::LeptonPins;
pub use crate::lepton_task::frame_acquisition_loop;
use crate::onboard_flash::{FLASH_SPI_TOTAL_PAYLOAD_SIZE, OnboardFlash};
use crate::rpi_power::wake_raspberry_pi;
use crate::startup_functions::{
    get_device_config, get_synced_time, maybe_offload_files_and_events_on_startup,
    validate_scheduled_alarm, work_out_recording_mode,
};
use crate::utils::{extend_lifetime_generic_mut, restart};
use bsp::hal::Timer;
use bsp::hal::watchdog::Watchdog;
use bsp::{
    entry,
    hal::{clocks::Clock, sio::Sio},
    pac::Peripherals,
};
use chrono::{Duration, Timelike, Utc};
use cortex_m::asm::nop;
use defmt::{assert, assert_eq, error, info, warn};
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::I2C;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 36;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.
// TODO Check against minor version also.
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 10; // 10 mins between FFCs

// TODO: Something with this info
// "In register 0x0F it will return the number of minutes since it first powered on or there was a button pressed on the camera. This is on dev now."

#[entry]
#[allow(clippy::too_many_lines)]
fn main() -> ! {
    info!("");
    info!("-----------------------");
    info!("Startup tc2-firmware {}", FIRMWARE_VERSION);
    // TODO: Check wake_en and sleep_en registers to make sure we're not enabling any clocks we don't need.
    let mut peripherals: Peripherals = Peripherals::take().unwrap();
    // Spit out the DMA channels, PIOs, and then steal peripherals again
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);
    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
    let mut peripherals = unsafe { Peripherals::steal() };
    let (clocks, rosc) = clock_utils::setup_rosc_as_system_clock(
        peripherals.CLOCKS,
        peripherals.XOSC,
        peripherals.ROSC,
        ROSC_TARGET_CLOCK_FREQ_HZ.Hz(),
    );
    let system_clock_freq = clocks.system_clock.freq();

    info!(
        "System clock speed {}MHz",
        clocks.system_clock.freq().to_MHz()
    );

    // Watchdog ticks are required to run the timer peripheral, since they're shared between both.
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    assert!(system_clock_freq.to_Hz() / 1_000_000 <= u32::from(u8::MAX));
    #[allow(clippy::cast_possible_truncation)]
    watchdog.enable_tick_generation((system_clock_freq.to_Hz() / 1_000_000) as u8);
    let mut timer: Timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);
    let sio = Sio::new(peripherals.SIO);
    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    let i2c_result = MainI2C::new(
        I2C::i2c1(
            peripherals.I2C1,
            pins.gpio6.reconfigure(),
            pins.gpio7.reconfigure(),
            400.kHz(),
            &mut peripherals.RESETS,
            &clocks.system_clock,
        ),
        pins.gpio3.reconfigure(),
        timer,
    );
    if let Err(e) = i2c_result {
        // FIXME: Basically we'll be in a restart loop if the attiny version is not
        //  what we expect.  Is this unrecoverable?
        error!("{}", e);
        restart(&mut watchdog);
    }
    let mut i2c = i2c_result.unwrap();

    if let Err(e) = i2c.attiny_keep_alive() {
        error!("Failed to send keep alive: {}", e);
    }

    // If the pi is powering up, wait for it to restart us when tc2-agent is ready.
    if i2c
        .get_camera_state()
        .is_ok_and(CameraState::pi_is_powering_on)
    {
        loop {
            if i2c
                .get_camera_state()
                .is_ok_and(CameraState::pi_is_powering_on)
            {
                timer.delay_ms(1000);
            } else {
                break;
            }
        }
    }

    // init flash
    // Create double buffers which are used for DMA transfers to the onboard flash module.
    // These need to live for the life of the program, so creating them here makes sense?
    let mut flash_page_buf = [0xffu8; FLASH_SPI_TOTAL_PAYLOAD_SIZE];
    let mut flash_page_buf_2 = [0xffu8; FLASH_SPI_TOTAL_PAYLOAD_SIZE];
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };
    let mut fs = {
        let fs_cs = pins.gpio9.into_push_pull_output();
        let fs_miso = pins.gpio8.into_pull_down_disabled().into_pull_type();
        let fs_mosi = pins.gpio11.into_pull_down_disabled().into_pull_type();
        let fs_clk = pins.gpio10.into_pull_down_disabled().into_pull_type();
        OnboardFlash::new(
            fs_cs,
            fs_mosi,
            fs_clk,
            fs_miso,
            flash_page_buf,
            flash_page_buf_2,
            dma_channels.ch1,
            dma_channels.ch2,
            system_clock_freq,
        )
    };

    // Setup soft/PIO SPI interface to raspberry pi
    let mut payload_buf = [0x42u8; RPI_TRANSFER_HEADER_LENGTH + RPI_PAYLOAD_LENGTH];
    let mut return_payload_buf = [0x42u8; RPI_RETURN_PAYLOAD_LENGTH];
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let return_payload_buf = unsafe { extend_lifetime_generic_mut(&mut return_payload_buf) };
    let mut pi_spi = {
        let pi_ping = pins.gpio5.into_pull_down_input();
        let pi_miso = pins.gpio15.into_floating_disabled();
        let pi_mosi = pins.gpio12.into_floating_disabled();
        let pi_cs = pins.gpio13.into_floating_disabled();
        let pi_clk = pins.gpio14.into_floating_disabled();
        ExtSpiTransfers::new(
            pi_mosi,
            pi_cs,
            pi_clk,
            pi_miso,
            pi_ping,
            dma_channels.ch0,
            payload_buf,
            return_payload_buf,
            pio0,
            sm0,
            timer,
        )
    };
    fs.init(peripherals.SPI1, &mut peripherals.RESETS);

    watchdog.pause_on_debug(true);
    watchdog.start(8_388_607.micros());

    let mut events = EventLogger::new(&mut fs);
    let time = get_synced_time(&mut i2c, &mut events, &mut fs, &mut watchdog, timer).unwrap();
    info!("Startup time {}", time);

    // let er = fs.erase_good_blocks();
    // if let Err(e) = er {
    //     error!("{}", e);
    // }

    let dc_result = get_device_config(
        &mut fs,
        &mut i2c,
        timer,
        &mut pi_spi,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        &mut watchdog,
        &mut events,
        &time,
    );
    if let Err(e) = dc_result {
        error!("{}", e);
        restart(&mut watchdog);
    }
    let (config, prioritise_frame_preview, config_was_updated, force_offload_now) =
        dc_result.unwrap();
    if config.use_high_power_mode()
        && (config.time_is_in_configured_recording_window(&time.date_time())
            || config.is_continuous_recorder())
    {
        if i2c
            .get_camera_state()
            .is_ok_and(CameraState::pi_is_waking_or_awake)
        {
            // Set the pi-needs-to-be-awake bit, just in case it's not set on startup.
            let _ = i2c.tell_pi_to_wakeup();
        } else {
            info!("Waking pi for high power mode");
            let _ = wake_raspberry_pi(&mut i2c, timer, Some(&mut watchdog), None);
        }
    }

    if prioritise_frame_preview {
        warn!("Booted in frame priority mode");
    }
    if force_offload_now {
        warn!("Booted in offload priority mode");
    }

    let scheduled_alarm = if config_was_updated {
        // If the config was updated, create a new alarm and restart
        None
    } else {
        i2c.get_scheduled_alarm(&time)
    };
    if let Some(scheduled_alarm) = &scheduled_alarm {
        if scheduled_alarm.has_triggered() {
            let reset_without_sleep = i2c.get_tc2_agent_state().is_ok_and(|state| {
                (state.requested_thermal_mode() || state.requested_audio_mode())
                    && !state.test_recording_requested()
            });
            if reset_without_sleep {
                info!("Reset without sleep");
                if let Err(e) = i2c.tc2_agent_clear_mode_flags() {
                    error!("{}", e);
                }
            } else {
                // Make sure the reason we woke was the trigger - if now != alarm time,
                // we missed the alarm, and were woken up for some other reason.
                if scheduled_alarm.date_time().hour() != time.date_time().hour()
                    || scheduled_alarm.date_time().minute() != time.date_time().minute()
                {
                    if scheduled_alarm.is_audio_alarm() {
                        // Missed alarm
                        warn!("Missed alarm");
                    }
                } else {
                    info!("Woken by RTC alarm");
                    events.log(Event::Rp2040WokenByAlarm, &time, &mut fs);
                }
            }
        }
        let alarm_out_of_bounds =
            scheduled_alarm.date_time() > time.date_time() + Duration::hours(25);
        if alarm_out_of_bounds {
            error!("Alarm out of bounds: {}", scheduled_alarm);
        }
        if scheduled_alarm.has_triggered() || alarm_out_of_bounds {
            // Clear the alarm flag:
            if let Err(e) = i2c.clear_and_disable_alarm(&time) {
                error!("{}", e);
                timer.delay_ms(100);
                restart(&mut watchdog);
            }
        }
        if alarm_out_of_bounds {
            timer.delay_ms(100);
            restart(&mut watchdog);
        }
    }

    validate_scheduled_alarm(
        &config,
        &mut fs,
        &time,
        &mut i2c,
        &mut events,
        &scheduled_alarm,
        &mut watchdog,
    );

    // We ALWAYS have a valid scheduled next alarm at this point.
    let scheduled_alarm = scheduled_alarm.unwrap();
    info!("Got valid scheduled alarm {}", scheduled_alarm);
    if scheduled_alarm.is_audio_alarm() {
        let num_mins = (scheduled_alarm.date_time() - time.date_time()).num_minutes();
        if num_mins > 0 {
            error!(
                "Next audio alarm scheduled in {} mins ({})",
                num_mins, scheduled_alarm
            );
        } else {
            error!(
                "Next audio alarm scheduled in {} seconds ({})",
                (scheduled_alarm.date_time() - time.date_time()).num_seconds(),
                scheduled_alarm
            );
        }
    }

    if let Ok(is_recording) = i2c.get_is_recording() {
        // Unset the is_recording flag on attiny on startup if still enabled.
        // If it was still set, that suggests a previous recording was interrupted.
        if is_recording {
            if config.use_low_power_mode() && fs.last_recording_is_complete(&mut watchdog) {
                error!("Still had recording bit set, but last recording seems to be complete");
            }
            events.log(Event::RecordingNotFinished, &time, &mut fs);
            if let Err(e) = i2c.stopped_recording() {
                error!("Error unsetting recording flag on attiny: {}", e);
            }
        }
    }

    let recording_mode = work_out_recording_mode(
        &scheduled_alarm,
        prioritise_frame_preview,
        &mut i2c,
        &config,
    );
    warn!("Recording mode: {:?}", recording_mode);
    maybe_offload_files_and_events_on_startup(
        recording_mode,
        prioritise_frame_preview,
        force_offload_now,
        &mut fs,
        &config,
        &time,
        &mut events,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        &mut i2c,
        &mut pi_spi,
        &mut watchdog,
    );
    if let Err(e) = i2c.attiny_keep_alive() {
        error!("Failed to send keep alive: {}", e);
    }
    if let Some(scheduled_alarm) = i2c.get_scheduled_alarm(&time)
        && scheduled_alarm.has_triggered()
    {
        warn!("Alarm triggered during offload, restarting");
        // If we triggered an alarm during offload, restart
        restart(&mut watchdog);
    }

    let current_recording_window = config.next_or_current_recording_window(&time.date_time());
    if let Err(e) = current_recording_window {
        error!("{}", e);
    }
    let current_recording_window = current_recording_window.unwrap();
    let record_audio_now = if let RecordingMode::Audio(recording_request_type) = recording_mode {
        if prioritise_frame_preview && !recording_request_type.is_user_requested() {
            // We're not trying to make a test audio recording, and the user is interacting
            // with sidekick.
            false
        } else {
            true
        }
    } else {
        false
    };

    if record_audio_now && let RecordingMode::Audio(recording_request_type) = recording_mode {
        let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
        record_audio(
            i2c,
            &config,
            system_clock_freq,
            pins.gpio0,
            pins.gpio1,
            watchdog,
            recording_request_type,
            fs,
            events,
            pi_spi,
            time,
            pio1,
            sm1,
        );
    } else {
        let current_recording_window = if THERMAL_DEV_MODE {
            (time.date_time(), time.date_time() + Duration::minutes(60))
        } else {
            current_recording_window
        };

        if let Ok(camera_connection_state) = i2c.get_camera_connection_state() {
            info!(
                "Current camera connection state: {:?}",
                camera_connection_state
            );
        }

        // If we're not doing an audio recording, check if we need to shutdown now.
        let inside_thermal_window = config
            .time_is_in_supplied_recording_window(&time.date_time(), current_recording_window);
        let should_shutdown = recording_mode == RecordingMode::None
            && i2c
                .get_camera_state()
                .is_ok_and(CameraState::pi_is_powered_off)
            && !inside_thermal_window
            && time.date_time() + Duration::minutes(2) < scheduled_alarm.date_time();
        if should_shutdown {
            info!("Tell attiny to shut us down");
            events.log(Event::Rp2040Sleep, &time, &mut fs);
            timer.delay_ms(1000);
            if let Err(e) = i2c.tell_attiny_to_power_down_rp2040() {
                error!("Failed to tell attiny to power down: {}", e);
            } else {
                loop {
                    nop();
                }
            }
        } else if recording_mode == RecordingMode::None && !inside_thermal_window {
            info!("Entering thermal mode because pi is on");
        } else if recording_mode == RecordingMode::None && inside_thermal_window {
            info!("Entering thermal mode because inside recording window");
        } else if let RecordingMode::Thermal(_) = recording_mode {
            info!("Entering thermal mode because it was requested/scheduled");
        }

        // If we don't have a recording_mode, default into thermal mode to serve frames to the rPi.
        let lepton_pins = LeptonPins {
            tx: pins.gpio23.into_function(),
            rx: pins.gpio20.into_function(),
            clk: pins.gpio22.into_function(),
            cs: pins.gpio21.into_function(),
            vsync: pins.gpio19.into_function(),
            sda: pins.gpio24.reconfigure(),
            scl: pins.gpio25.reconfigure(),
            power_down: pins.gpio28.into_push_pull_output(),
            power_enable: pins.gpio18.into_push_pull_output(),
            reset: pins.gpio29.into_push_pull_output(),
            clk_disable: pins.gpio27.into_push_pull_output(),
            master_clk: pins.gpio26.into_floating_input(),
        };
        record_thermal(
            i2c,
            pi_spi,
            fs,
            lepton_pins,
            watchdog,
            system_clock_freq,
            &clocks,
            rosc,
            &config,
            events,
            time,
            current_recording_window,
            recording_mode,
        );
    }
}
