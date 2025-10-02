#![warn(clippy::all, clippy::pedantic)]
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

pub extern crate std;

use crate::entry::real_main;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::log::error;
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::test_state::test_global_state::{CURRENT_TIME, DEVICE_CONFIG};
use chrono::{DateTime, Utc};

pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
) {
    *CURRENT_TIME.lock().unwrap() = from;
    *DEVICE_CONFIG.lock().unwrap() = Some(config);
    error!(
        "Simulating camera from {} until {}",
        FormattedNZTime(from),
        FormattedNZTime(until)
    );
    let end_time = until;
    loop {
        let now = { *CURRENT_TIME.lock().unwrap() };
        if now > end_time {
            break;
        }
        real_main();
    }
}

#[test]
pub fn example_test() {
    println!("Hello, world!");
    assert_eq!(2 + 2, 4);
}

fn main() {}
