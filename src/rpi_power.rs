use crate::attiny_rtc_i2c::SharedI2C;
use cortex_m::delay::Delay;
use defmt::{error, info, warn};
use rp2040_hal::Watchdog;

/// Returns `true` if it did wake the rPI
#[allow(clippy::needless_pass_by_value)]
pub fn wake_raspberry_pi(
    i2c: &mut SharedI2C,
    delay: &mut Delay,
    watchdog: Option<&mut Watchdog>,
) -> bool {
    if let Ok(true) = i2c.pi_is_powered_down(delay, false) {
        if i2c.tell_pi_to_wakeup(delay).is_ok() {
            // TODO: Log here if this was an unexpected wakeup
            warn!("Sent wake signal to raspberry pi");
            // Poll to see when tc2-agent is ready.
            loop {
                if let Some(ref watchdog) = watchdog {
                    watchdog.feed();
                }
                if let Ok(pi_is_awake) = i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
                    if pi_is_awake {
                        break;
                    }
                    // Try to wake it again, just in case it was shutdown behind our backs.
                    let _ = i2c.tell_pi_to_wakeup(delay);
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
            if let Ok(pi_is_awake) = i2c.pi_is_awake_and_tc2_agent_is_ready(delay, false) {
                if pi_is_awake {
                    break;
                }
                // Try to wake it again, just in case it was shutdown behind our back.
                let _ = i2c.tell_pi_to_wakeup(delay);
            }
            delay.delay_ms(1000);
        }
        false
    }
}

pub fn advise_raspberry_pi_it_may_shutdown<'a>(
    i2c: &'a mut SharedI2C,
    delay: &mut Delay,
) -> Result<(), &'a str> {
    let result = i2c.tell_pi_to_shutdown(delay);
    if result.is_err() {
        error!("Error sending power-down advice to raspberry pi");
    } else {
        info!("Sent power-down advice to raspberry pi");
    }
    result
}
