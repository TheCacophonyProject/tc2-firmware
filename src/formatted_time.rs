use chrono::{DateTime, Datelike, Duration, Timelike, Utc};
pub struct FormattedNZTime(pub DateTime<Utc>);

impl FormattedNZTime {
    fn approx_nz_time(&self) -> (DateTime<Utc>, bool) {
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
        (approx_nz_time, nzdt)
    }
}

#[cfg(feature = "no-std")]
impl defmt::Format for FormattedNZTime {
    fn format(&self, fmt: defmt::Formatter) {
        let (approx_nz_time, nzdt) = self.approx_nz_time();
        defmt::write!(
            fmt,
            "DateTime: {}-{}-{} {:02}:{:02}:{} {}",
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

#[cfg(feature = "std")]
impl core::fmt::Display for FormattedNZTime {
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::fmt::Result {
        let (approx_nz_time, nzdt) = self.approx_nz_time();
        write!(
            fmt,
            "DateTime: {}-{}-{} {:02}:{:02}:{} {}",
            approx_nz_time.year(),
            approx_nz_time.month(),
            approx_nz_time.day(),
            approx_nz_time.hour(),
            approx_nz_time.minute(),
            approx_nz_time.second(),
            if nzdt { "NZDT" } else { "NZST" },
        )
    }
}
