#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_variables)]
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

use crate::attiny_rtc_i2c::{MainI2C, RecordingMode};
use crate::audio_task::record_audio;
use crate::event_logger::{Event, EventLogger};
use crate::ext_spi_transfers::{
    ExtSpiTransfers, RPI_PAYLOAD_LENGTH, RPI_RETURN_PAYLOAD_LENGTH, RPI_TRANSFER_HEADER_LENGTH,
};
use crate::frame_processing::record_thermal;
use crate::lepton::LeptonPins;
pub use crate::lepton_task::frame_acquisition_loop;
use crate::onboard_flash::FLASH_SPI_TOTAL_PAYLOAD_SIZE;
use crate::onboard_flash::OnboardFlash;
use crate::onboard_flash::extend_lifetime_generic_mut;
use crate::startup_functions::{
    get_device_config, get_or_schedule_next_alarm, get_synced_time,
    maybe_offload_files_and_events_on_startup, work_out_recording_mode,
};
use crate::utils::restart;
use bsp::hal::Timer;
use bsp::hal::watchdog::Watchdog;
use bsp::{
    entry,
    hal::{clocks::Clock, sio::Sio},
    pac::Peripherals,
};
use chrono::{Datelike, Timelike, Utc};
use defmt::{assert, assert_eq, error, info, warn};
use defmt_rtt as _;
use fugit::{ExtU32, RateExtU32};
use panic_probe as _;
use rp2040_hal::I2C;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::pio::PIOExt;

// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
pub const FIRMWARE_VERSION: u32 = 20;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.
// TODO Check against minor version also.

// got funny results at 150 for audio seems to work better at 125
const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
const FFC_INTERVAL_MS: u32 = 60 * 1000 * 10; // 10 mins between FFCs

#[entry]
#[allow(clippy::too_many_lines)]
fn main() -> ! {
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
    let timer: Timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);
    let sio = Sio::new(peripherals.SIO);
    let pins = rp2040_hal::gpio::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

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
            None,
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

    fs.take_spi(peripherals.SPI1, &mut peripherals.RESETS);
    fs.init();

    watchdog.pause_on_debug(true);
    watchdog.start(8_388_607.micros());
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
        error!("{}", e);
        restart(&mut watchdog);
    }
    let mut i2c = i2c_result.unwrap();

    let mut events = EventLogger::new(&mut fs);
    let time = get_synced_time(&mut i2c, &mut events, &mut fs, &mut watchdog, timer).unwrap();
    let scheduled_alarm = i2c.get_scheduled_alarm(&time);
    if let Some(scheduled_alarm) = &scheduled_alarm {
        if scheduled_alarm.has_triggered() {
            // Make sure the reason we woke was the trigger - if now != alarm time,
            // we missed the alarm, and were woken up for some other reason.
            if scheduled_alarm.time().hour() != time.date_time().hour()
                || scheduled_alarm.time().minute() != time.date_time().minute()
            {
                // Missed alarm
                warn!("Missed alarm");
            } else {
                info!("Woken by RTC alarm");
                events.log(Event::Rp2040WokenByAlarm, &time, &mut fs);
            }
            // Clear the alarm flag:
            if let Err(e) = i2c.clear_and_disable_alarm(&time) {
                error!("{}", e);
                restart(&mut watchdog);
            }
        }
    } else {
        // FIXME: If there's nothing scheduled, schedule a new alarm and restart.
    }
    if let Some(alarm) = &scheduled_alarm
        && alarm.is_audio_alarm()
    {
        warn!(
            "Next audio alarm scheduled in {}mins (at {} {}:{})",
            (alarm.time() - time.date_time()).num_minutes(),
            alarm.time().day(),
            alarm.time().hour(),
            alarm.time().minute(),
        );
    }

    info!("Scheduled alarm: {}", scheduled_alarm);
    let dc_result = get_device_config(
        &mut fs,
        &mut i2c,
        timer,
        &mut pi_spi,
        system_clock_freq,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        &mut watchdog,
        events.count(),
    );
    if let Err(e) = dc_result {
        error!("{}", e);
        restart(&mut watchdog);
    }
    let (config, prioritise_frame_preview) = dc_result.unwrap();

    // If the current schedule is good, use it, otherwise reschedule and restart
    let missed_audio_alarm = get_or_schedule_next_alarm(
        &config,
        &mut fs,
        &time,
        &mut i2c,
        &mut events,
        &scheduled_alarm,
        &mut watchdog,
    );

    if let Ok(is_recording) = i2c.get_is_recording() {
        // Unset the is_recording flag on attiny on startup if still enabled.
        // If it was still set, that suggests a previous recording was interrupted.
        if is_recording {
            if config.use_low_power_mode() && fs.last_recording_is_complete() {
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
        &time,
    );
    warn!("Recording mode: {:?}", recording_mode);
    // TODO: Maybe check sun_times for valid lat/lng which can give us a recording window,
    //  log if we can't get one
    maybe_offload_files_and_events_on_startup(
        recording_mode,
        prioritise_frame_preview,
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

    if recording_mode == RecordingMode::None {
        // FIXME: Maybe the power down should happen in startup if there's not a recording to be made.
        // crate::audio_task::power_down_or_restart(
        //     alarm_date_time,
        //     time,
        //     config,
        //     watchdog,
        //     i2c,
        //     events,
        //     fs,
        // )
    }

    let record_audio_now = if let RecordingMode::Audio(recording_request_type) = recording_mode {
        if prioritise_frame_preview && !recording_request_type.is_user_requested() {
            // We're not trying to make a test audio recording, and the user is interacting
            // with sidekick.
            false
        } else {
            // missed_audio recordings will hit this case.
            true
        }
    } else {
        false
    };

    if record_audio_now && let RecordingMode::Audio(recording_request_type) = recording_mode {
        let (pio1, _, sm1, _, _) = peripherals.PIO1.split(&mut peripherals.RESETS);
        record_audio(
            i2c,
            system_clock_freq,
            pins.gpio0,
            pins.gpio1,
            watchdog,
            recording_request_type,
            &config,
            fs,
            events,
            pi_spi,
            &time,
            pio1,
            sm1,
        );
    } else {
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
            scheduled_alarm,
        );
    }
}
