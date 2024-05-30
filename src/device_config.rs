use crate::motion_detector::DetectionMask;
use crate::rp2040_flash::read_device_config_from_rp2040_flash;
use crate::sun_times::sun_times;
use crate::{byte_slice_cursor::Cursor, motion_detector};
use chrono::{Duration, NaiveDateTime, NaiveTime, Timelike};
use defmt::{info, Format, Formatter};
use embedded_io::Read;
use pcf8563::DateTime;

pub enum AudioMode {
    AudioOnly = 0,
    AudioOrThermal = 1,
    AudioAndThermal = 2,
}
impl PartialEq for AudioMode {
    fn eq(&self, other: &Self) -> bool {
        matches!(self, other)
    }
}

impl Format for AudioMode {
    fn format(&self, fmt: Formatter) {}
}

impl TryFrom<u8> for AudioMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AudioMode::*;

        match value {
            0 => Ok(AudioOnly),
            1 => Ok(AudioOrThermal),
            2 => Ok(AudioAndThermal),
            _ => Err(()),
        }
    }
}
#[derive(Format, PartialEq)]
pub struct DeviceConfigInner {
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
    pub is_audio_device: bool,
    pub audio_mode: AudioMode,
}

pub struct DeviceConfig {
    config_inner: DeviceConfigInner,
    pub motion_detection_mask: DetectionMask,
    pub cursor_position: usize,
}

impl PartialEq for DeviceConfig {
    fn eq(&self, other: &Self) -> bool {
        self.config_inner == other.config_inner
            && self.motion_detection_mask == other.motion_detection_mask
    }
}

impl Format for DeviceConfig {
    fn format(&self, fmt: Formatter) {}
}

impl Default for DeviceConfig {
    fn default() -> Self {
        let test_name = "default".as_bytes();
        let mut device_name = [0u8; 64];
        device_name[0..test_name.len()].copy_from_slice(test_name);
        DeviceConfig {
            config_inner: DeviceConfigInner {
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
                is_audio_device: false,
                audio_mode: AudioMode::try_from(0).ok().unwrap(),
            },
            motion_detection_mask: DetectionMask::new(None),
            cursor_position: 0,
        }
    }
}

impl DeviceConfig {
    pub fn load_existing_config_from_flash(load_mask: bool) -> Option<DeviceConfig> {
        let slice = read_device_config_from_rp2040_flash();
        let device_config = DeviceConfig::from_bytes(slice, load_mask);
        device_config
    }

    pub fn from_bytes(bytes: &[u8], load_mask: bool) -> Option<DeviceConfig> {
        let mut cursor = Cursor::new(bytes);
        let device_id = cursor.read_u32();
        if device_id == u32::MAX {
            // Device config is uninitialised in flash
            return None;
        }
        let is_audio_device = cursor.read_bool();
        let audio_mode = AudioMode::try_from(0).ok().unwrap();
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
            let read_bytes = cursor
                .read(&mut device_name[1..(device_name_length + 1).min(len)])
                .unwrap();
            device_name
        };
        // let mask_length = cursor.read_i32();
        let mut cursor_pos = cursor.position();
        let mut motion_detection_mask;
        if load_mask {
            motion_detection_mask = DetectionMask::new(Some([0u8; 2400]));
            let len = cursor
                .read(&mut motion_detection_mask.inner.unwrap())
                .unwrap();

            if len != motion_detection_mask.inner.unwrap().len() {
                // This payload came without the mask attached (i.e. from the rPi)
                motion_detection_mask.set_empty();
            } else {
                cursor_pos = cursor.position();
            }
        } else {
            motion_detection_mask = DetectionMask::new(None);
        }
        Some(DeviceConfig {
            config_inner: DeviceConfigInner {
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
                is_audio_device,
                audio_mode,
            },
            motion_detection_mask,
            cursor_position: cursor_pos,
        })
    }

    pub fn config(&self) -> &DeviceConfigInner {
        &self.config_inner
    }
    pub fn device_name(&self) -> &str {
        let len = self.config_inner.device_name[0] as usize;
        let slice_len = self.config_inner.device_name.len();
        core::str::from_utf8(&self.config_inner.device_name[1..(1 + len).min(slice_len)])
            .unwrap_or("Invalid device name")
    }

    pub fn device_name_bytes(&self) -> &[u8] {
        let len = self.config_inner.device_name[0] as usize;
        &self.config_inner.device_name[1..1 + len]
    }

    pub fn next_recording_window_start(&self, now_utc: &NaiveDateTime) -> NaiveDateTime {
        self.next_or_current_recording_window(now_utc).0
    }

    pub fn use_low_power_mode(&self) -> bool {
        self.config_inner.use_low_power_mode
    }

    pub fn next_or_current_recording_window(
        &self,
        now_utc: &NaiveDateTime,
    ) -> (NaiveDateTime, NaiveDateTime) {
        let (is_absolute_start, mut start_offset) = self.config_inner.start_recording_time;
        let (is_absolute_end, mut end_offset) = self.config_inner.end_recording_time;

        if is_absolute_end && end_offset < 0 {
            end_offset = 86_400 + end_offset;
        }
        if is_absolute_start && start_offset < 0 {
            start_offset = 86_400 + start_offset;
        }
        let (window_start, window_end) = if !is_absolute_start || !is_absolute_end {
            let (lat, lng) = self.config_inner.location;
            let altitude = self.config_inner.location_altitude;
            let yesterday_utc = *now_utc - Duration::days(1);
            let (_, yesterday_sunset) = sun_times(
                yesterday_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            let yesterday_sunset =
                yesterday_sunset.naive_utc() + Duration::seconds(start_offset as i64);
            let (today_sunrise, today_sunset) = sun_times(
                now_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            let today_sunrise = today_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
            let today_sunset = today_sunset.naive_utc() + Duration::seconds(start_offset as i64);
            let tomorrow_utc = *now_utc + Duration::days(1);
            let (tomorrow_sunrise, tomorrow_sunset) = sun_times(
                tomorrow_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            let tomorrow_sunrise =
                tomorrow_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
            let tomorrow_sunset =
                tomorrow_sunset.naive_utc() + Duration::seconds(start_offset as i64);

            if *now_utc > today_sunset && *now_utc > tomorrow_sunrise {
                let two_days_from_now_utc = *now_utc + Duration::days(2);
                let (two_days_sunrise, _) = sun_times(
                    two_days_from_now_utc.date(),
                    lat as f64,
                    lng as f64,
                    altitude.unwrap_or(0.0) as f64,
                )
                .unwrap();
                let two_days_sunrise =
                    two_days_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
                (Some(tomorrow_sunset), Some(two_days_sunrise))
            } else if (*now_utc > today_sunset && *now_utc < tomorrow_sunrise)
                || (*now_utc < today_sunset && *now_utc > today_sunrise)
            {
                (Some(today_sunset), Some(tomorrow_sunrise))
            } else if *now_utc < tomorrow_sunset
                && *now_utc < today_sunrise
                && *now_utc > yesterday_sunset
            {
                (Some(yesterday_sunset), Some(today_sunrise))
            } else {
                panic!("Unable to calculate relative time window");
            }
        } else {
            (None, None)
        };

        let mut start_time = if !is_absolute_start {
            window_start.unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(start_offset as u32, 0).unwrap(),
            )
        };
        let mut end_time = if !is_absolute_end {
            window_end.unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(end_offset as u32, 0).unwrap(),
            )
        };

        if is_absolute_start || is_absolute_end {
            let start_minus_one_day = start_time - Duration::days(1);
            let mut start_plus_one_day = start_time + Duration::days(1);
            let mut end_minus_one_day = end_time - Duration::days(1);
            let end_plus_one_day = end_time + Duration::days(1);

            if start_minus_one_day > end_minus_one_day {
                end_minus_one_day = end_minus_one_day + Duration::days(1);
            }
            if start_plus_one_day > end_plus_one_day {
                start_plus_one_day = start_time;
            }
            if end_minus_one_day > *now_utc {
                if is_absolute_start {
                    start_time = start_minus_one_day;
                }
                if is_absolute_end {
                    end_time = end_minus_one_day;
                }
            }
            if end_time < start_time && is_absolute_end {
                end_time = end_plus_one_day;
            }
            if *now_utc > end_time {
                if is_absolute_start {
                    start_time = start_plus_one_day;
                }
                if is_absolute_end {
                    end_time = end_plus_one_day;
                }
            }
        }
        (start_time, end_time)
    }

    pub fn time_is_in_daylight(&self, date_time_utc: &NaiveDateTime) -> bool {
        let (lat, lng) = self.config_inner.location;
        let altitude = self.config_inner.location_altitude;
        let (sunrise, sunset) = sun_times(
            date_time_utc.date(),
            lat as f64,
            lng as f64,
            altitude.unwrap_or(0.0) as f64,
        )
        .unwrap();
        date_time_utc.hour() > sunrise.hour() && date_time_utc.hour() < sunset.hour()
    }
    pub fn time_is_in_recording_window(
        &self,
        date_time_utc: &NaiveDateTime,
        window: &Option<(NaiveDateTime, NaiveDateTime)>,
    ) -> bool {
        if self.is_continuous_recorder() {
            info!("Continuous recording mode enabled");
            return true;
        }
        let (start_time, end_time) =
            window.unwrap_or(self.next_or_current_recording_window(date_time_utc));
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
        *date_time_utc >= start_time && *date_time_utc <= end_time
    }

    pub fn is_continuous_recorder(&self) -> bool {
        self.config_inner.is_continuous_recorder
            || (self.config_inner.start_recording_time.0
                && self.config_inner.end_recording_time.0
                && self.config_inner.start_recording_time == self.config_inner.end_recording_time)
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
    let naive_datetime = NaiveDateTime::new(naive_date.unwrap(), naive_time.unwrap());
    naive_datetime
}
