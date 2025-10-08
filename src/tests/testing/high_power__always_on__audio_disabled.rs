use crate::re_exports::log::info;
use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config, test_start_and_end_time};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;
#[test]
fn high_power_mode_always_on_audio_disabled() {
    let config = ConfigBuilder::new()
        .high_power_mode()
        .audio_disabled()
        .always_on()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);
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
