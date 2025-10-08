use crate::re_exports::log::info;
use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config, test_start_and_end_time};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;

#[test]
fn high_power_mode_always_on_audio_and_thermal() {
    // Start simulation 10mins before the beginning of the 24hr window, and end simulation
    // 30mins after to allow time for offload to happen.
    let config = ConfigBuilder::new()
        .high_power_mode()
        .audio_and_thermal()
        .always_on()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);

    // Expectations:
    // - Roughly 32 audio recordings should be made per 24 hr period.
    // - All audio recordings and events should be offloaded at the end of the 24hr window.
    // - The firmware makes no thermal recordings; this is handled by the rPi in high power mode.
    // - The rPi should never be powered off.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    simulate_camera_with_config(config, start_time, end_time, None, cptv_files);
    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        info!("FILES");
        info!("{:#?}", state.files_offloaded);
        info!("EVENTS");
        info!("{:#?}", state.events_offloaded);
    });
}
