#![warn(clippy::all, clippy::pedantic)]
#![cfg(test)]
#![cfg(feature = "std")]
mod attiny_rtc_i2c;
mod audio_task;
mod byte_slice_cursor;
mod clock_utils;
mod constants;
mod cptv_encoder;
mod device_config;
mod entry;
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
mod re_exports;
mod rpi_power;
mod startup_functions;
mod sub_tasks;
mod sun_times;
mod synced_date_time;
mod tests;
mod utils;

fn main() {
    // NOTE: Never called in tests, see ./tests/testing/* or ./entry.rs::real_main()
}
