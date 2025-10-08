use crate::re_exports::log::info;
use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config, test_start_and_end_time};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;
#[test]
fn low_power_mode_fixed_window_audio_disabled() {
    // Trigger thermal videos at 10, 150, 270 minutes into a thermal window.
    let thermal_trigger_offsets = Some(vec![5, 150, 270]);
    // Cptv files to loop through for testing.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .fixed_recording_window()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);
    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );
    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        info!("FILES");
        info!("{:#?}", state.files_offloaded);
        info!("EVENTS");
        info!("{:#?}", state.events_offloaded);
    });
}
