use chrono::{DateTime, Datelike, Duration, Timelike, Utc};
use defmt::Formatter;

pub struct FormattedNZTime(pub DateTime<Utc>);
impl defmt::Format for FormattedNZTime {
    fn format(&self, fmt: Formatter) {
        // Very crude daylight savings calc, will be off by an hour sometimes at the
        // beginning of April and end of September – but that's okay, it's just for debug display.
        let month = self.0.month();
        let day = self.0.day();
        let nzdt = if (4..=10).contains(&month) {
            (month == 4 && day < 7) || (month == 10 && day > 23)
        } else {
            true
        };
        let approx_nz_time = if nzdt {
            self.0 + Duration::hours(13)
        } else {
            self.0 + Duration::hours(12)
        };

        defmt::write!(
            fmt,
            "DateTime: {}-{}-{} {}:{}:{} {}",
            approx_nz_time.year(),
            approx_nz_time.month(),
            approx_nz_time.day(),
            approx_nz_time.hour(),
            approx_nz_time.minute(),
            approx_nz_time.second(),
            if nzdt { "NZDT" } else { "NZST" },
        );
    }
}
