use crate::entry::real_main;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::log::error;
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::test_state::test_global_state::{
    CPTV_FILES, CURRENT_THERMAL_WINDOW, CURRENT_TIME, DEVICE_CONFIG, ROSC_DRIVE_ITERATOR,
    THERMAL_TRIGGER_OFFSETS_MINS,
};
use chrono::{DateTime, Utc};
use std::path::Path;

pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
    thermal_trigger_recordings_x_mins_into_thermal_window: Option<Vec<u32>>,
    cptv_files_to_playback: Option<Vec<String>>,
) {
    *CURRENT_TIME.lock().unwrap() = from;

    *THERMAL_TRIGGER_OFFSETS_MINS.lock().unwrap() =
        thermal_trigger_recordings_x_mins_into_thermal_window;
    *CPTV_FILES.lock().unwrap() = cptv_files_to_playback;

    *DEVICE_CONFIG.lock().unwrap() = Some(config.clone());
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

        // Reset ROSC_ITERATOR:
        *ROSC_DRIVE_ITERATOR.lock().unwrap() = 0;
        *CURRENT_THERMAL_WINDOW.lock().unwrap() =
            Some(config.next_recording_window(&now.naive_utc()));
        real_main();
    }
}
