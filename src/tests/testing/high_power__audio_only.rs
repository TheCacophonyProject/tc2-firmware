use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config};
use crate::tests::test_state::test_global_state::{
    CURRENT_TIME, EVENTS_OFFLOADED, FILES_OFFLOADED,
};
use chrono::Duration;

#[test]
fn high_power_mode_audio_only() {
    // FIXME: Is there really such a thing as high power audio only?
    //  Would that imply recordings are always offloaded as they are created?

    env_logger::init();
    let start_time = { *CURRENT_TIME.lock().unwrap() };
    let end_time = start_time + Duration::hours(48);
    let config = ConfigBuilder::new().high_power_mode().audio_only().build();

    simulate_camera_with_config(config, start_time, end_time, None, None);

    // TODO: Make test state thread local so tests can be run in parallel.

    println!("FILES");
    println!("{:#?}", FILES_OFFLOADED.lock().unwrap());
    println!("EVENTS");
    println!("{:#?}", EVENTS_OFFLOADED.lock().unwrap());
}
