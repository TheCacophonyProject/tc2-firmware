use crate::attiny_rtc_i2c::MainI2C;
use crate::event_logger::{EventLogger, LoggerEvent};
use crate::onboard_flash::OnboardFlash;
use defmt::{info, warn};
use embedded_hal::delay::DelayNs;
use rp2040_hal::{Timer, Watchdog};

/// Returns `true` if it did wake the rPI
#[allow(clippy::needless_pass_by_value)]
pub fn wake_raspberry_pi(
    i2c: &mut MainI2C,
    mut delay: Timer,
    watchdog: Option<&mut Watchdog>,
    event_to_log: Option<(&mut OnboardFlash, &mut EventLogger, LoggerEvent)>,
) -> bool {
    if i2c.get_camera_state().is_ok_and(|s| s.pi_is_powered_off()) {
        if i2c.tell_pi_to_wakeup().is_ok() {
            // TODO: Log here if this was an unexpected wakeup
            warn!("Sent wake signal to raspberry pi");
            if let Some((fs, events, event_to_log)) = event_to_log {
                events.log_event(event_to_log, fs);
            }
            // Poll to see when tc2-agent is ready.
            loop {
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
            true
        } else {
            warn!("Failed to send wake signal to raspberry pi");
            false
        }
    } else {
        loop {
            info!("Polling tc2-agent");
            if let Some(ref watchdog) = watchdog {
                watchdog.feed();
            }
            if let Ok(pi_is_awake) = i2c.check_if_pi_is_awake_and_tc2_agent_is_ready() {
                if pi_is_awake {
                    break;
                }
                // Try to wake it again, just in case it was shutdown behind our back.
                let _ = i2c.tell_pi_to_wakeup();
            }
            delay.delay_ms(1000);
        }
        false
    }
}
