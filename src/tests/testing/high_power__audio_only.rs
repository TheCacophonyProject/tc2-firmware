use crate::re_exports::log::info;
use crate::tests::helpers::{ConfigBuilder, simulate_camera_with_config, test_start_and_end_time};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;

#[test]
fn high_power_mode_audio_only() {
    // FIXME: Is there really such a thing as high power audio only?
    //  Would that imply recordings are always offloaded as they are created?
    let config = ConfigBuilder::new().high_power_mode().audio_only().build();
    let (start_time, end_time) = test_start_and_end_time(&config);
    simulate_camera_with_config(config, start_time, end_time, None, None);
    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        info!("FILES");
        info!("{:#?}", state.files_offloaded);
        info!("EVENTS");
        info!("{:#?}", state.events_offloaded);
    });
}
