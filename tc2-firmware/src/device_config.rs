use crate::byte_slice_cursor::Cursor;
use crate::rp2040_flash::read_rp2040_flash;
use crate::sun_times::sun_times;
use chrono::{NaiveDateTime, NaiveTime, Timelike};
use core::ops::Add;
use defmt::{info, Format};
use embedded_io::Read;
use pcf8563::DateTime;

#[derive(Format, PartialEq)]
pub struct DeviceConfig {
    pub device_id: u32,
    device_name: [u8; 64], // length, ...payload
    pub location: (f32, f32),
    pub location_altitude: Option<f32>,
    pub location_timestamp: Option<u64>,
    pub location_accuracy: Option<f32>,
    start_recording_time: (bool, i32),
    end_recording_time: (bool, i32),
    pub is_continuous_recorder: bool,
    pub use_low_power_mode: bool,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        let test_name = "default".as_bytes();
        let mut device_name = [0u8; 64];
        device_name[0..test_name.len()].copy_from_slice(test_name);
        DeviceConfig {
            device_id: 0,
            device_name,
            location: (0.0, 0.0),
            location_altitude: None,
            location_timestamp: None,
            location_accuracy: None,
            start_recording_time: (false, 0),
            end_recording_time: (false, 0),
            is_continuous_recorder: false,
            use_low_power_mode: false,
        }
    }
}

impl DeviceConfig {
    pub fn load_existing_config_from_flash() -> Option<DeviceConfig> {
        let slice = read_rp2040_flash();
        let (device_config, _) = DeviceConfig::from_bytes(slice);
        device_config
    }

    pub fn from_bytes(bytes: &[u8]) -> (Option<DeviceConfig>, usize) {
        let mut cursor = Cursor::new(bytes);
        let device_id = cursor.read_u32();
        if device_id == u32::MAX {
            // Device config is uninitialised in flash
            return (None, 0);
        }
        let latitude = cursor.read_f32();
        let longitude = cursor.read_f32();
        let has_location_timestamp = cursor.read_bool();
        let location_timestamp = cursor.read_u64();
        let location_timestamp = has_location_timestamp.then_some(location_timestamp);
        let has_location_altitude = cursor.read_bool();
        let location_altitude = cursor.read_f32();
        let location_altitude = has_location_altitude.then_some(location_altitude);
        let has_location_accuracy = cursor.read_bool();
        let location_accuracy = cursor.read_f32();
        let location_accuracy = has_location_accuracy.then_some(location_accuracy);
        let start_recording_time = (cursor.read_bool(), cursor.read_i32());
        let end_recording_time = (cursor.read_bool(), cursor.read_i32());
        let is_continuous_recorder = cursor.read_bool();
        let use_low_power_mode = cursor.read_bool();
        let device_name_length = cursor.read_u8() as usize;
        let mut device_name = [0u8; 64];
        device_name[0] = device_name_length as u8;
        let len = device_name.len();
        let device_name = {
            cursor
                .read(&mut device_name[1..(device_name_length + 1).min(len)])
                .unwrap();
            device_name
        };

        (
            Some(DeviceConfig {
                device_id,
                device_name,
                location: (latitude, longitude),
                location_altitude,
                location_timestamp,
                location_accuracy,
                start_recording_time,
                end_recording_time,
                is_continuous_recorder,
                use_low_power_mode,
            }),
            cursor.position(),
        )
    }

    pub fn device_name(&self) -> &str {
        let len = self.device_name[0] as usize;
        let slice_len = self.device_name.len();
        core::str::from_utf8(&self.device_name[1..(1 + len).min(slice_len)])
            .unwrap_or("Invalid device name")
    }

    pub fn device_name_bytes(&self) -> &[u8] {
        let len = self.device_name[0] as usize;
        &self.device_name[1..1 + len]
    }

    pub fn next_recording_window_start(&self, now_utc: &NaiveDateTime) -> NaiveDateTime {
        self.next_recording_window(now_utc).0
    }

    pub fn next_recording_window(&self, now_utc: &NaiveDateTime) -> (NaiveDateTime, NaiveDateTime) {
        let (is_absolute_start, mut start_offset) = self.start_recording_time;
        let (is_absolute_end, mut end_offset) = self.end_recording_time;
        if is_absolute_end && end_offset < 0 {
            end_offset = 86_400 + end_offset;
        }
        if is_absolute_start && start_offset < 0 {
            start_offset = 86_400 + start_offset;
        }
        let (sunrise, sunset) = if !is_absolute_start || !is_absolute_end {
            let (lat, lng) = self.location;
            let altitude = self.location_altitude;
            let (mut sunrise, sunset) = sun_times(
                now_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            if sunrise < sunset {
                sunrise = sunrise.add(chrono::Duration::days(1));
            }
            (Some(sunrise), Some(sunset))
        } else {
            (None, None)
        };

        let mut start_time = if !is_absolute_start {
            sunset
                .unwrap()
                .naive_utc()
                .checked_add_signed(chrono::Duration::seconds(start_offset as i64))
                .unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(start_offset as u32, 0).unwrap(),
            )
        };
        let mut end_time = if !is_absolute_end {
            sunrise
                .unwrap()
                .naive_utc()
                .checked_add_signed(chrono::Duration::seconds(end_offset as i64))
                .unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(end_offset as u32, 0).unwrap(),
            )
        };

        // FIXME: Sometimes wrong for absolute windows?
        if end_time < *now_utc {
            end_time = end_time.add(chrono::Duration::days(1));
            start_time = start_time.add(chrono::Duration::days(1));
        } else if end_time < start_time {
            end_time = end_time.add(chrono::Duration::days(1));
        }
        (start_time, end_time)
    }

    pub fn time_is_in_daylight(&self, date_time_utc: &NaiveDateTime) -> bool {
        let (lat, lng) = self.location;
        let altitude = self.location_altitude;
        let (sunrise, sunset) = sun_times(
            date_time_utc.date(),
            lat as f64,
            lng as f64,
            altitude.unwrap_or(0.0) as f64,
        )
        .unwrap();
        date_time_utc.hour() > sunrise.hour() && date_time_utc.hour() < sunset.hour()
    }
    pub fn time_is_in_recording_window(&self, date_time_utc: &NaiveDateTime) -> bool {
        if self.is_continuous_recorder {
            return true;
        }
        let (start_time, end_time) = self.next_recording_window(date_time_utc);
        let starts_in = start_time - *date_time_utc;
        let starts_in_hours = starts_in.num_hours();
        let starts_in_mins = starts_in.num_minutes() - (starts_in_hours * 60);
        let ends_in = end_time - *date_time_utc;
        let ends_in_hours = ends_in.num_hours();
        let ends_in_mins = ends_in.num_minutes() - (ends_in_hours * 60);
        let window = end_time - start_time;
        let window_hours = window.num_hours();
        let window_mins = window.num_minutes() - (window_hours * 60);
        if start_time > *date_time_utc && end_time > *date_time_utc {
            info!(
                "Recording will start in {}h{}m and end in {}h{}m, window duration {}h{}m",
                starts_in_hours,
                starts_in_mins,
                ends_in_hours,
                ends_in_mins,
                window_hours,
                window_mins
            );
        } else if end_time > *date_time_utc {
            info!(
                "Recording will end in {}h{}m, window duration {}h{}m",
                ends_in_hours, ends_in_mins, window_hours, window_mins
            );
        }
        *date_time_utc > start_time && *date_time_utc < end_time
    }
}

pub fn get_naive_datetime(datetime: DateTime) -> NaiveDateTime {
    let naive_date = chrono::NaiveDate::from_ymd_opt(
        2000 + datetime.year as i32,
        datetime.month as u32,
        datetime.day as u32,
    );
    if naive_date.is_none() {
        panic!(
            "Couldn't get date for {}, {}, {}",
            2000 + datetime.year as i32,
            datetime.month as u32,
            datetime.day as u32
        );
    }
    let naive_time = chrono::NaiveTime::from_hms_opt(
        datetime.hours as u32,
        datetime.minutes as u32,
        datetime.seconds as u32,
    );
    if naive_time.is_none() {
        panic!(
            "Couldn't get time for {}, {}, {}",
            datetime.hours as u32, datetime.minutes as u32, datetime.seconds as u32,
        );
    }
    let naive_datetime = chrono::NaiveDateTime::new(naive_date.unwrap(), naive_time.unwrap());
    naive_datetime
}
