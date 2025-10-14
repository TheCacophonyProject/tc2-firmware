use crate::re_exports::log::info;
use crate::tests::helpers::{
    ConfigBuilder, last_shutdown_recording_is_within_4_mins_of,
    last_startup_recording_is_within_2_mins_of, next_or_current_thermal_window,
    num_audio_recordings_offloaded, num_audio_recordings_stored_in_flash,
    num_thermal_recordings_offloaded, num_thermal_recordings_stored_in_flash,
    simulate_camera_with_config, startup_and_shutdown_recordings_made, test_start_and_end_time,
};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use test_log::test;
#[test]
fn low_power_mode_always_on_audio_disabled() {
    // Trigger thermal videos at 10, 150, 270 minutes into a thermal window.
    let thermal_trigger_offsets = Some(vec![5, 150, 270]);
    // Cptv files to loop through for testing.
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .always_on()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);
    let thermal_window = next_or_current_thermal_window(&config);
    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );
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
        assert_eq!(7, num_thermal_recordings_offloaded(&state.files_offloaded));
        assert_eq!(0, num_audio_recordings_offloaded(&state.files_offloaded));
        assert_eq!(
            0,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        // NOTE: Because we're padding our 24hr window with 10mins on either side, we
        //  end up simulating two full days, so there are some CPTV status recordings left in the
        //  flash at the end.
        assert_eq!(
            2,
            num_thermal_recordings_stored_in_flash(&state.flash_backing_storage)
        );
    });
}
