use crate::attiny_rtc_i2c::MainI2C;
use crate::event_logger::{Event, EventLogger};
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::OnboardFlash;
use chrono::{DateTime, Duration, Utc};
use defmt::{Format, Formatter, error};
use rp2040_hal::Timer;
use rp2040_hal::timer::Instant;

// NOTE: Important: If we start using dormant states again, the timer will be incorrect
#[derive(Copy, Clone)]
pub struct SyncedDateTime {
    pub date_time_utc: DateTime<Utc>,
    pub timer_offset: Instant,
    timer: Timer,
}

impl Format for SyncedDateTime {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "{}", FormattedNZTime(self.date_time_utc));
    }
}

impl SyncedDateTime {
    pub fn get_timestamp_micros(&self) -> i64 {
        self.date_time().timestamp_micros()
    }

    pub fn get_timer(&self) -> Timer {
        self.timer
    }

    #[allow(clippy::cast_possible_wrap)]
    pub fn date_time(&self) -> DateTime<Utc> {
        self.date_time_utc
            + Duration::microseconds(
                (self.timer.get_counter() - self.timer_offset).to_micros() as i64
            )
    }

    pub fn set(&mut self, date_time: DateTime<Utc>) {
        self.date_time_utc = date_time;
        self.timer_offset = self.timer.get_counter();
    }

    pub fn resync_with_rtc(
        &mut self,
        i2c: &mut MainI2C,
        events: &mut EventLogger,
        fs: &mut OnboardFlash,
        print: bool,
    ) {
        match i2c.get_datetime(self.timer, print) {
            Ok(new_time) => {
                if new_time.date_time_utc > self.date_time_utc
                    && new_time.date_time_utc < self.date_time_utc + Duration::minutes(15)
                {
                    *self = new_time;
                }
            }
            Err(e) => {
                events.log_if_not_dupe(Event::RtcCommError, self, fs);
                error!("Unable to get DateTime from RTC: {}", e);
            }
        }
    }

    pub fn new(date_time: DateTime<Utc>, timer: Timer) -> SyncedDateTime {
        SyncedDateTime {
            date_time_utc: date_time,
            timer_offset: timer.get_counter(),
            timer,
        }
    }
}
