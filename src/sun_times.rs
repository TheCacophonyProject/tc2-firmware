use chrono::{DateTime, TimeZone, Timelike, Utc};
#[allow(unused_imports)]
use num_traits::real::Real; // Allows sin/cosine for suntimes
const UNIX_EPOCH: JulianDate = JulianDate(2_440_587.5);
const SECONDS_PER_DAY: u64 = 24 * 60 * 60;
const JAN_2000: JulianDate = JulianDate(2_451_545.0);
const LEAP_SECONDS: JulianDate = JulianDate(0.0008);
const OBLIQUITY_OF_THE_ECLIPTIC: f64 = 23.44;

#[derive(Debug, Clone, Copy)]
struct JulianDate(f64);

impl JulianDate {
    fn ceil_days(self) -> f64 {
        self.0.ceil()
    }

    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_precision_loss)]
    fn to_datetime(self) -> Option<DateTime<Utc>> {
        Utc.timestamp_opt(((self - UNIX_EPOCH).0 * SECONDS_PER_DAY as f64).round() as i64, 0)
            .single()
    }
}

impl From<DateTime<Utc>> for JulianDate {
    #[allow(clippy::cast_precision_loss)]
    fn from(date: DateTime<Utc>) -> Self {
        Self((date.timestamp() as f64 / SECONDS_PER_DAY as f64) + UNIX_EPOCH.0)
    }
}

impl core::ops::Sub<JulianDate> for JulianDate {
    type Output = Self;

    fn sub(self, rhs: JulianDate) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl core::ops::Add<JulianDate> for JulianDate {
    type Output = Self;

    fn add(self, rhs: JulianDate) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

fn rem_euclid(lhs: f64, rhs: f64) -> f64 {
    let r = lhs % rhs;
    if r < 0.0 { r + rhs.abs() } else { r }
}

/// Calculates the approximate sunset and sunrise times at a given latitude, longitude, and altitude
///
/// Note that elevation is used to correct for atmospheric refraction, so negative elevations are treated as being at
/// sea level due to having minimal difference in refraction to being at sea level
///
/// # Arguments
///
/// * `date` - The date on which to calculate the sunset and sunrise, in UTC
/// * `latitude` - The latitude at which to calculate the times. Expressed as degrees
/// * `longitude` - The longitude at which to calculate the times. Expressed as degrees
/// * `elevation` - The elevation at which to calculate the times. Expressed as meters above sea level. Negative values will be ignored
///
/// # Return value
///
/// Returns
///  - `None` if the date is not representable in chrono (~5M years from now), or sunsets/rises cannot be calculated due to long arctic/antarctic day/night (outside ~±67° of latitude)
///  - `Some((sunrise,sunset))` otherwise
///
/// # Examples
///
/// ```
/// //Calculate the sunset and sunrise times today at Sheffield university's new computer science building
/// let times = sun_times(Utc::today(),53.38,-1.48,100.0);
/// println!("Sunrise: {}, Sunset: {}",times.0,times.1);
/// ```
pub fn sun_times(
    date: DateTime<Utc>,
    latitude: f64,
    longitude: f64,
    elevation: f64,
) -> Option<(DateTime<Utc>, DateTime<Utc>)> {
    //see https://en.wikipedia.org/wiki/Sunrise_equation

    const ARGUMENT_OF_PERIHELION: f64 = 102.9372;
    let julian_date = JulianDate::from(date.with_hour(0)?.with_minute(0)?.with_second(0)?);

    //elevations below sea level will have minimal atmospheric refraction + the
    //calculation is broken below sea level, so treat negative elevations as being at sea level
    let elevation = elevation.max(0.0);
    let elevation_correction = -2.076 * (elevation.sqrt()) / 60.0;

    let days_since_2000 = (julian_date - JAN_2000 + LEAP_SECONDS).ceil_days();

    let mean_solar_time = days_since_2000 - (longitude / 360.0);
    let solar_mean_anomaly = rem_euclid(357.5291 + 0.985_600_280 * mean_solar_time, 360.0);
    let center = 1.9148 * solar_mean_anomaly.to_radians().sin()
        + 0.0200 * (2.0 * solar_mean_anomaly).to_radians().sin()
        + 0.0003 * (3.0 * solar_mean_anomaly).to_radians().sin();
    let ecliptic_longitude =
        rem_euclid(solar_mean_anomaly + center + 180.0 + ARGUMENT_OF_PERIHELION, 360.0);

    let declination = (ecliptic_longitude.to_radians().sin()
        * OBLIQUITY_OF_THE_ECLIPTIC.to_radians().sin())
    .asin();
    let event_hour_angle = (((-0.83 + elevation_correction).to_radians().sin()
        - (latitude.to_radians().sin() * declination.sin()))
        / (latitude.to_radians().cos() * declination.cos()))
    .acos()
    .to_degrees();

    if event_hour_angle.is_nan() {
        return None;
    }

    let solar_transit =
        JAN_2000.0 + mean_solar_time + 0.0053 * solar_mean_anomaly.to_radians().sin()
            - 0.0069 * (2.0 * ecliptic_longitude).to_radians().sin();
    let solar_transit_julian = JulianDate(solar_transit);

    let julian_rise = JulianDate(solar_transit_julian.0 - event_hour_angle / 360.0);
    let julian_set = JulianDate(solar_transit_julian.0 + event_hour_angle / 360.0);
    let rise = julian_rise.to_datetime();
    let set = julian_set.to_datetime();
    if let (Some(rise), Some(set)) = (rise, set) { Some((rise, set)) } else { None }
}
