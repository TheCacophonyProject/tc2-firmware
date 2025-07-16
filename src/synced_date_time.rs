use crate::attiny_rtc_i2c::MainI2C;
use crate::event_logger::{Event, EventLogger};
use crate::onboard_flash::OnboardFlash;
use chrono::{DateTime, Duration, Utc};
use defmt::error;
use rp2040_hal::Timer;
use rp2040_hal::timer::Instant;

// NOTE: Important: If we start using dormant states again, the timer will be incorrect
pub struct SyncedDateTime {
    date_time_utc: DateTime<Utc>,
    pub timer_offset: Instant,
    timer: Timer,
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
    ) {
        match i2c.get_datetime(self.timer) {
            Ok(new_time) => *self = new_time,
            Err(e) => {
                events.log(Event::RtcCommError, self, fs);
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
