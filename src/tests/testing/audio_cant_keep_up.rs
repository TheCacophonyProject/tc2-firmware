use crate::re_exports::log::assert_eq;
use crate::tests::helpers::{
    ConfigBuilder, num_audio_recordings_stored_in_flash, offloaded_event_exists,
    simulate_camera_with_config, stored_event_exists, test_start_and_end_time,
};
use crate::tests::mocks::fake_rpi_event_logger::LoggerEventKind;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
use test_log::test;

#[test]
fn low_power_mode_audio_only() {
    let config = ConfigBuilder::new().low_power_mode().audio_only().build();
    let (start_time, end_time) = test_start_and_end_time(&config);

    // Make the window really short, we don't care about having a lot of recordings.
    let end_time = end_time - Duration::hours(22);

    TEST_SIM_STATE.with(|s| {
        s.borrow_mut().audio_recording_fails_on_restart_iteration = Some(2);
    });

    simulate_camera_with_config(config, start_time, end_time, None, None);

    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        assert!(stored_event_exists(
            &state.flash_backing_storage,
            LoggerEventKind::AudioRecordingFailed
        ));
        // Make sure we got the retried audio recording.
        assert_eq!(
            4,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
    });
}
