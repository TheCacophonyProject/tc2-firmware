use chrono::{DateTime, Duration, Utc};
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
        self.get_date_time().timestamp_micros()
    }

    pub fn get_timer(&self) -> Timer {
        self.timer
    }

    #[allow(clippy::cast_possible_wrap)]
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

    pub fn new(date_time: DateTime<Utc>, timer: Timer) -> SyncedDateTime {
        SyncedDateTime { date_time_utc: date_time, timer_offset: timer.get_counter(), timer }
    }
}
