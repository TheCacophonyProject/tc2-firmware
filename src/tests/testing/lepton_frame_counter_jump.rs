use crate::tests::helpers::{
    ConfigBuilder, last_shutdown_recording_is_within_4_mins_of, next_or_current_thermal_window,
    simulate_camera_with_config, test_start_and_end_time,
};
use crate::tests::mocks::fake_rpi_event_logger::FileType;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
use test_log::test;

/// If the lepton frame counter jumps ahead by 100 or more frames (about 11 seconds of frames
/// we didn't get), recording should carry on once frames are coming in again.
///
/// Today the frame after the jump is flagged as having invalid telemetry, and since the previous
/// telemetry is only updated from valid frames, every frame after it is compared against the same
/// stale frame number and is invalid too, until the rp2040 restarts. Nothing gets recorded, not
/// even the shutdown status recording, so there is no offload at the end of the window. The
/// periodic bookkeeping still runs, so the attiny keep-alive is still sent and the attiny never
/// resets the rp2040.
///
/// This is what A108 (device 2866) did from 18 Nov 2026 14:57 until its 12:00 alarm the next day
/// (low power, 24/7 window from 12:00 to 12:00, audio disabled).
#[test]
fn recording_continues_after_lepton_frame_counter_jump() {
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .always_on()
        .build();
    let (start_time, end_time) = test_start_and_end_time(&config);
    let thermal_window = next_or_current_thermal_window(&config);

    // Frames jump 200 frames (about 23s) ahead 60 minutes into the window, after the first
    // trigger and before the other two.
    let jump_at_mins = 60;
    let jump_time = thermal_window.0 + Duration::minutes(jump_at_mins);
    TEST_SIM_STATE.with(|s| s.borrow_mut().frame_num_jump = Some((jump_time, 200)));
    let thermal_trigger_offsets = Some(vec![5, 150, 270]);
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    simulate_camera_with_config(
        config,
        start_time,
        end_time,
        thermal_trigger_offsets,
        cptv_files,
    );

    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        assert!(
            state.frame_num_jump.is_none(),
            "The simulation never reached the frame counter jump"
        );
        let motion_recordings_after_jump = state
            .files_offloaded
            .iter()
            .filter(|file| {
                file.file_type == FileType::CptvScheduled
                    && file.recording_time > jump_time
                    && file.recording_time < thermal_window.1
            })
            .count();
        assert_eq!(
            2, motion_recordings_after_jump,
            "Expected the triggers at 150 and 270 minutes to be recorded after the frame \
             counter jump at {jump_at_mins} minutes. Offloaded: {:#?}",
            state.files_offloaded
        );
        assert!(
            state
                .files_offloaded
                .iter()
                .any(|file| file.file_type == FileType::CptvShutdown),
            "No shutdown status recording was made, so the end of window offload is missed"
        );
        assert!(last_shutdown_recording_is_within_4_mins_of(
            &state.files_offloaded,
            thermal_window.1
        ));
    });
}
