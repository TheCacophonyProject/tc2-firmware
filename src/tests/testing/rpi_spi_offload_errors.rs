use crate::tests::helpers::{
    ConfigBuilder, last_shutdown_recording_is_within_4_mins_of,
    last_startup_recording_is_within_2_mins_of, next_or_current_thermal_window,
    num_audio_recordings_offloaded, num_audio_recordings_stored_in_flash,
    num_thermal_recordings_offloaded, simulate_camera_with_config,
    startup_and_shutdown_recordings_made, test_start_and_end_time,
};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;
#[test]
fn file_offload_errors() {
    // Cptv files to loop through for testing.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .default_recording_window()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);
    let thermal_window = next_or_current_thermal_window(&config);
    TEST_SIM_STATE.with(|s| {
        // Force offload responses from the rpi to fail on a given restart iteration,
        // and make sure they resume on the next.
        s.borrow_mut().offloads_fail_on_restart_iteration = Some(5);
    });
    // TODO: May need to make sure this works with each profile?
    simulate_camera_with_config(config, start_time, end_time, None, cptv_files);
    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        assert!(startup_and_shutdown_recordings_made(&state.files_offloaded));
        assert!(last_startup_recording_is_within_2_mins_of(
            &state.files_offloaded,
            thermal_window.0
        ));
        assert!(last_shutdown_recording_is_within_4_mins_of(
            &state.files_offloaded,
            thermal_window.1
        ));
        assert_eq!(2, num_thermal_recordings_offloaded(&state.files_offloaded));
        assert_eq!(0, num_audio_recordings_offloaded(&state.files_offloaded));
        assert_eq!(
            0,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
    });
}
