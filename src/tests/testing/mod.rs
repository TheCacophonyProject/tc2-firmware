use crate::re_exports::log::warn;
use crate::tests::helpers::simulate_camera_with_config;
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::test_state::test_global_state::{
    CURRENT_TIME, EVENTS_OFFLOADED, FILES_OFFLOADED,
};
use chrono::Duration;

#[test]
fn low_power_mode_dusk_till_dawn() {
    // Create a flash device and figure out where to write config and events to it.
    // Assume bad blocks are not known.

    // Trigger thermal videos at 10, 150, 270 minutes into a thermal window.
    let thermal_trigger_offsets = Some(vec![5, 150, 270]);
    // Cptv files to loop through for testing.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);

    env_logger::init();
    warn!("Running tests");
    let start_time = { *CURRENT_TIME.lock().unwrap() };
    let end_time = start_time + Duration::hours(48);
    let test_latitude: f64 = -43.5;
    let test_longitude: f64 = 172.64;
    let config: DeviceConfig = toml::from_str(&format!(
        r#"
[device]
id = 123
name = "CI-fake-test-device"

[location]
accuracy = 0.0
altitude = 0.0
latitude = {}
longitude = {}

[thermal-recorder]
use-low-power-mode = true

[audio-recording]
audio-mode = 'AudioOrThermal'
random-seed = 0
"#,
        test_latitude, test_longitude
    ))
    .unwrap();
    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );
    // NOTE(wip): Store files offloaded and the times they were made? events offloaded.
    // Start and end recording and the appropriate number of audio recordings.
    // There should be some stuff that gets written to in the simulation that we can now read back,
    // and make some tests assertions on.

    println!("FILES");
    println!("{:#?}", FILES_OFFLOADED.lock().unwrap());
    println!("EVENTS");
    println!("{:#?}", EVENTS_OFFLOADED.lock().unwrap());
}
