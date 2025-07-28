use crate::attiny_rtc_i2c::{CameraState, MainI2C};
use crate::event_logger::{EventLogger, LoggerEvent};
use crate::onboard_flash::OnboardFlash;
use defmt::{info, warn};
use embedded_hal::delay::DelayNs;
use rp2040_hal::{Timer, Watchdog};

#[allow(clippy::needless_pass_by_value)]
fn poll_until_tc2_agent_is_ready(
    i2c: &mut MainI2C,
    watchdog: Option<&mut Watchdog>,
    mut delay: Timer,
) {
    loop {
        info!("Polling tc2-agent");
        if let Some(ref watchdog) = watchdog {
            watchdog.feed();
        }
        if let Ok(pi_is_awake) = i2c.check_if_pi_is_awake_and_tc2_agent_is_ready() {
            if pi_is_awake {
                break;
            }
            // Try to wake it again, just in case it was shutdown behind our backs.
            let _ = i2c.tell_pi_to_wakeup();
        }
        delay.delay_ms(1000);
    }
}

/// Returns `true` if it did wake the rPI
pub fn wake_raspberry_pi(
    i2c: &mut MainI2C,
    delay: Timer,
    watchdog: Option<&mut Watchdog>,
    event_to_log: Option<(&mut OnboardFlash, &mut EventLogger, LoggerEvent)>,
) -> bool {
    if i2c
        .get_camera_state()
        .is_ok_and(CameraState::pi_is_powered_off)
    {
        if i2c.tell_pi_to_wakeup().is_ok() {
            // TODO: Log here if this was an unexpected wakeup
            warn!("Sent rPi wake signal to attiny");
            if let Some((fs, events, event_to_log)) = event_to_log {
                events.log_event(event_to_log, fs);
            }
            poll_until_tc2_agent_is_ready(i2c, watchdog, delay);
            true
        } else {
            warn!("Failed to send rPi wake signal to attiny");
            false
        }
    } else {
        poll_until_tc2_agent_is_ready(i2c, watchdog, delay);
        false
    }
}
