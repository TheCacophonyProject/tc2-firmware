use crate::tests::helpers::{
    ConfigBuilder, simulate_camera_with_config, test_start_and_end_time_with_min_duration,
};
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
use std::string::String;
use std::vec;
use test_log::test;

fn simulate_low_power_camera() {
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .custom_recording_window("10:00", "11:00")
        .build();
    let (start_time, end_time) =
        test_start_and_end_time_with_min_duration(&config, Duration::hours(3));
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    // Motion recordings 5 and 30 minutes into the recording window, as well as the status
    // recordings.
    let thermal_trigger_offsets = Some(vec![5, 30]);
    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );
}

fn assert_pixel_ranges_match_headers() {
    TEST_SIM_STATE.with(|s| {
        let state = s.borrow();
        // The startup and shutdown status recordings, and at least one motion recording.
        assert!(
            state.cptv_files_pixel_checked > 2,
            "Only {} CPTV files were offloaded to check",
            state.cptv_files_pixel_checked
        );
        assert!(
            state.cptv_pixel_range_mismatches.is_empty(),
            "Decoded pixels don't match the header min/max in {} of {} files: {:?}",
            state.cptv_pixel_range_mismatches.len(),
            state.cptv_files_pixel_checked,
            state.cptv_pixel_range_mismatches
        );
    });
}

/// The min and max values in a recording's header should match its decoded pixels.
#[test]
fn recording_pixel_range_matches_header() {
    simulate_low_power_camera();
    assert_pixel_ranges_match_headers();
}

/// Invalid frames aren't written to a recording. Each frame is stored as deltas from the one
/// before, and the skipped frame used to become the frame the next one was encoded against, so
/// every frame after it decoded wrong.
#[test]
fn invalid_frame_during_a_recording_does_not_corrupt_it() {
    TEST_SIM_STATE.with(|s| s.borrow_mut().invalid_frame_every = Some(50));
    simulate_low_power_camera();
    assert_pixel_ranges_match_headers();
}
