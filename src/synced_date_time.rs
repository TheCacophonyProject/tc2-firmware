use chrono::{DateTime, Duration, Utc};
use rp2040_hal::timer::Instant;
use rp2040_hal::Timer;

// NOTE: Important: If we start using dormant states again, the timer will be incorrect
pub struct SyncedDateTime {
    date_time_utc: DateTime<Utc>,
    pub timer_offset: Instant,
    timer: Timer,
}

impl SyncedDateTime {
    pub fn get_timestamp_micros(&self) -> u64 {
        self.get_date_time().timestamp_micros() as u64
    }

    pub fn get_timer(&self) -> Timer {
        self.timer
    }

    // FIXME: Work out how often the timer counter overflows, and make sure we're not hitting that.
    pub fn get_date_time(&self) -> DateTime<Utc> {
        self.date_time_utc
            + Duration::microseconds(
                (self.timer.get_counter() - self.timer_offset).to_micros() as i64
            )
    }

    pub fn set(&mut self, date_time: DateTime<Utc>) {
        self.date_time_utc = date_time;
        self.timer_offset = self.timer.get_counter();
    }

    pub fn new(date_time: DateTime<Utc>, timer: &Timer) -> SyncedDateTime {
        SyncedDateTime {
            date_time_utc: date_time,
            timer_offset: timer.get_counter(),
            timer: timer.clone(),
        }
    }
}
