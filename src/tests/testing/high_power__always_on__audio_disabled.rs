use crate::tests::helpers::{
    ConfigBuilder, num_audio_recordings_offloaded, num_audio_recordings_stored_in_flash,
    num_thermal_recordings_offloaded, num_thermal_recordings_stored_in_flash,
    offloaded_event_count, offloaded_event_does_not_exist, simulate_camera_with_config,
    test_start_and_end_time,
};
use crate::tests::mocks::fake_rpi_event_logger::LoggerEventKind;
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
        assert_eq!(0, num_audio_recordings_offloaded(&state.files_offloaded));
        assert_eq!(0, num_thermal_recordings_offloaded(&state.files_offloaded));
        assert_eq!(
            0,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert_eq!(
            0,
            num_thermal_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert_ne!(0, state.events_offloaded.len());
        assert_eq!(
            3,
            offloaded_event_count(
                &state.events_offloaded,
                LoggerEventKind::StartedSendingFramesToRpi,
            )
        );
        assert!(offloaded_event_does_not_exist(
            &state.events_offloaded,
            LoggerEventKind::Rp2040Sleep
        ));
    });
}
