use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config};
use crate::tests::test_state::test_global_state::{
    CURRENT_TIME, EVENTS_OFFLOADED, FILES_OFFLOADED,
};
use chrono::Duration;

#[test]
fn high_power_mode_always_on_audio_and_thermal() {
    env_logger::init();
    let start_time = { *CURRENT_TIME.lock().unwrap() };
    let end_time = start_time + Duration::hours(48);
    let config = ConfigBuilder::new()
        .high_power_mode()
        .audio_and_thermal()
        .always_on()
        .build();

    simulate_camera_with_config(config, start_time, end_time, None, None);

    // TODO: Make test state thread local so tests can be run in parallel.

    println!("FILES");
    println!("{:#?}", FILES_OFFLOADED.lock().unwrap());
    println!("EVENTS");
    println!("{:#?}", EVENTS_OFFLOADED.lock().unwrap());
}
