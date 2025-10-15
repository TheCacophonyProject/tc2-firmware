use crate::re_exports::log::assert_eq;
use crate::tests::helpers::{
    ConfigBuilder, num_audio_recordings_offloaded, num_audio_recordings_stored_in_flash,
    num_thermal_recordings_offloaded, num_thermal_recordings_stored_in_flash,
    simulate_camera_with_config, test_start_and_end_time, wake_reason_present,
};
use crate::tests::mocks::fake_rpi_event_logger::WakeReason;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
use test_log::test;

#[test]
fn low_power_mode_audio_only() {
    let config = ConfigBuilder::new().low_power_mode().audio_only().build();
    let (start_time, end_time) = test_start_and_end_time(&config);

    // Add some simulation time to give extra time for the flash to fill up to trigger offloading.
    let end_time = end_time + Duration::hours(24);
    simulate_camera_with_config(config, start_time, end_time, None, None);

    TEST_SIM_STATE.with(|state| {
        let state = state.borrow();
        assert_eq!(40, num_audio_recordings_offloaded(&state.files_offloaded));
        assert_eq!(0, num_thermal_recordings_offloaded(&state.files_offloaded));
        assert_eq!(
            27,
            num_audio_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert_eq!(
            0,
            num_thermal_recordings_stored_in_flash(&state.flash_backing_storage)
        );
        assert!(wake_reason_present(
            &state.events_offloaded,
            WakeReason::AudioTooFull
        ))
    });
}
