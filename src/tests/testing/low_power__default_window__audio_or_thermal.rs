use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config};
use crate::tests::test_state::test_global_state::{
    CURRENT_TIME, EVENTS_OFFLOADED, FILES_OFFLOADED,
};
use chrono::Duration;

#[test]
fn low_power_mode_dusk_til_dawn_audio_or_thermal() {
    env_logger::init();
    // Trigger thermal videos at 10, 150, 270 minutes into a thermal window.
    let thermal_trigger_offsets = Some(vec![5, 150, 270]);
    // Cptv files to loop through for testing.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    let start_time = { *CURRENT_TIME.lock().unwrap() };
    let end_time = start_time + Duration::hours(48);
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_or_thermal()
        .default_recording_window()
        .build();

    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );

    println!("FILES");
    println!("{:#?}", FILES_OFFLOADED.lock().unwrap());
    println!("EVENTS");
    println!("{:#?}", EVENTS_OFFLOADED.lock().unwrap());
}
