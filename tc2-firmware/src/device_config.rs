use crate::sun_times::sun_times;
use byteorder::{ByteOrder, LittleEndian};
use chrono::{Days, NaiveDateTime, NaiveTime};
use core::ops::{Add, Sub};
use defmt::{info, Format};
use fugit::Duration;

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
}

impl DeviceConfig {
    pub fn from_bytes(bytes: &[u8]) -> (DeviceConfig, usize) {
        let device_id = LittleEndian::read_u32(&bytes[0..4]);
        let latitude = LittleEndian::read_f32(&bytes[4..8]);
        let longitude = LittleEndian::read_f32(&bytes[8..12]);
        let location_timestamp = if bytes[12] == 1 {
            Some(LittleEndian::read_u64(&bytes[13..21]))
        } else {
            None
        };
        let location_altitude = if bytes[21] == 1 {
            Some(LittleEndian::read_f32(&bytes[22..26]))
        } else {
            None
        };
        let location_accuracy = if bytes[26] == 1 {
            Some(LittleEndian::read_f32(&bytes[27..31]))
        } else {
            None
        };
        let start_recording_time = if bytes[31] == 1 {
            (true, LittleEndian::read_i32(&bytes[32..36]))
        } else {
            (false, LittleEndian::read_i32(&bytes[32..36]))
        };
        let end_recording_time = if bytes[36] == 1 {
            (true, LittleEndian::read_i32(&bytes[37..41]))
        } else {
            (false, LittleEndian::read_i32(&bytes[37..41]))
        };
        let is_continuous_recorder = bytes[41] == 1;
        let device_name_length = bytes[42] as usize;
        let mut device_name = [0u8; 64];
        let device_name = if device_name_length == 255 {
            // Flash hasn't been written yet;
            device_name
        } else {
            device_name[0..device_name_length + 1]
                .copy_from_slice(&bytes[42..(42 + device_name_length + 1).min(bytes.len())]);
            device_name
        };

        (
            DeviceConfig {
                device_id,
                device_name,
                location: (latitude, longitude),
                location_altitude,
                location_timestamp,
                location_accuracy,
                start_recording_time,
                end_recording_time,
                is_continuous_recorder,
            },
            (41 + device_name_length + 1).min(bytes.len()),
        )
    }

    pub fn device_name(&self) -> &str {
        let len = self.device_name[0] as usize;
        core::str::from_utf8(&self.device_name[1..1 + len]).unwrap()
    }

    pub fn device_name_bytes(&self) -> &[u8] {
        let len = self.device_name[0] as usize;
        &self.device_name[1..1 + len]
    }

    pub fn time_is_in_recording_window(&self, date_time_utc: &NaiveDateTime) -> bool {
        if self.is_continuous_recorder {
            return true;
        }
        let (is_absolute_start, mut start_offset) = self.start_recording_time;
        let (is_absolute_end, mut end_offset) = self.end_recording_time;
        if end_offset < 0 {
            end_offset = 86_400 + end_offset;
        }
        if start_offset < 0 {
            start_offset = 86_400 + start_offset;
        }
        let (sunrise, sunset) = if !is_absolute_start || !is_absolute_end {
            let (lat, lng) = self.location;
            let altitude = self.location_altitude;
            let (mut sunrise, sunset) = sun_times(
                date_time_utc.date(),
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
                date_time_utc.date(),
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
                date_time_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(end_offset as u32, 0).unwrap(),
            )
        };

        if end_time < start_time {
            end_time = end_time.add(chrono::Duration::days(1));
        }
        info!(
            "Start time {}, end time {}",
            start_time.timestamp(),
            end_time.timestamp()
        );
        if start_time > *date_time_utc {
            info!(
                "Recording will start in {}h{}m",
                (start_time - *date_time_utc).num_hours(),
                (start_time - *date_time_utc).num_minutes()
            );
        }
        if end_time > *date_time_utc {
            info!(
                "Recording will end in {}h{}m",
                (end_time - *date_time_utc).num_hours(),
                (end_time - *date_time_utc).num_minutes()
            );
        }
        info!(
            "Recording window duration {}h{}m",
            (end_time - start_time).num_hours(),
            (end_time - start_time).num_minutes()
        );
        *date_time_utc > start_time && *date_time_utc < end_time
    }
}
