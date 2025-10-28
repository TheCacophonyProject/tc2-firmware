use crate::re_exports::log::assert_eq;
use crate::tests::helpers::{
    ConfigBuilder, num_audio_recordings_offloaded, num_audio_recordings_stored_in_flash,
    num_thermal_recordings_offloaded, num_thermal_recordings_stored_in_flash,
    offloaded_event_count, offloaded_event_does_not_exist, simulate_camera_with_config,
    stored_event_does_not_exist, stored_event_exists, test_start_and_end_time,
};
use crate::tests::mocks::fake_rpi_event_logger::LoggerEventKind;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
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
    // Add some simulation time to allow 24hr window to end, or oldest audio
    // recording in the flash storage to be older than 24 hrs.
    let end_time = end_time + Duration::hours(2);
    simulate_camera_with_config(config, start_time, end_time, None, cptv_files);
    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        assert_eq!(32, num_audio_recordings_offloaded(&state.files_offloaded));
        assert_eq!(0, num_thermal_recordings_offloaded(&state.files_offloaded));
        assert_eq!(
            1,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert_eq!(
            0,
            num_thermal_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert_eq!(
            1,
            offloaded_event_count(&state.events_offloaded, LoggerEventKind::OffloadedLogs)
        );
        assert_eq!(
            35,
            offloaded_event_count(
                &state.events_offloaded,
                LoggerEventKind::StartedSendingFramesToRpi
            )
        );
        assert!(offloaded_event_does_not_exist(
            &state.events_offloaded,
            LoggerEventKind::Rp2040Sleep
        ));
        assert!(stored_event_does_not_exist(
            &state.flash_backing_storage,
            LoggerEventKind::Rp2040Sleep
        ));
        assert!(offloaded_event_does_not_exist(
            &state.events_offloaded,
            LoggerEventKind::ToldRpiToSleep
        ));
        assert!(stored_event_does_not_exist(
            &state.flash_backing_storage,
            LoggerEventKind::ToldRpiToSleep
        ));
    });
}
