use crate::entry_point;
use crate::formatted_time::FormattedNZTime;
#[cfg(feature = "std")]
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
#[cfg(feature = "std")]
use crate::tests::test_global_state::{CURRENT_TIME, DEVICE_CONFIG};
use chrono::{DateTime, Utc};
#[cfg(feature = "std")]
use log::error;

#[cfg(feature = "std")]
#[cfg(not(target_arch = "thumbv6m"))]
mod stubs;
#[cfg(feature = "std")]
pub mod test_global_state;

#[cfg(feature = "std")]
pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
) {
    *CURRENT_TIME.lock().unwrap() = from;
    *DEVICE_CONFIG.lock().unwrap() = Some(config);
    error!(
        "Simulating camera from {} until {}",
        FormattedNZTime(from),
        FormattedNZTime(until)
    );
    let end_time = until;
    loop {
        let now = { *CURRENT_TIME.lock().unwrap() };
        if now > end_time {
            break;
        }
        entry_point();
    }
}
