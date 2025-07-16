use crate::byte_slice_cursor::Cursor;
use crate::motion_detector::DetectionMask;
use crate::onboard_flash::OnboardFlash;
use crate::sun_times::sun_times;
use chrono::{Duration, NaiveDate, NaiveDateTime, NaiveTime, Timelike, Utc};
use defmt::{Format, Formatter, error};
use embedded_io::Read;

#[derive(PartialEq)]
pub struct SmallString([u8; 64]);

impl Format for SmallString {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "{}", self.as_str());
    }
}

impl SmallString {
    pub fn new(data: [u8; 64]) -> SmallString {
        SmallString(data)
    }

    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::range_plus_one)]
    pub fn new_from_bytes(data: &[u8]) -> SmallString {
        let mut inner = [0u8; 64];
        assert!(data.len() < inner.len(), "Invalid small string length");
        inner[0] = data.len().min(inner.len()) as u8;
        inner[1..1 + data.len()].copy_from_slice(data);
        SmallString(inner)
    }

    pub fn as_str(&self) -> &str {
        let len = self.0[0] as usize;
        let slice_len = self.0.len();
        core::str::from_utf8(&self.0[1..(1 + len).min(slice_len)]).unwrap_or("Invalid str")
    }

    #[allow(clippy::range_plus_one)]
    pub fn as_bytes(&self) -> &[u8] {
        let len = self.0[0] as usize;
        &self.0[1..1 + len]
    }
}

#[derive(Format, PartialEq, Copy, Clone)]
pub enum AudioMode {
    Disabled = 0,
    AudioOnly = 1,
    AudioOrThermal = 2,
    AudioAndThermal = 3,
}

impl TryFrom<u8> for AudioMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AudioMode::Disabled),
            1 => Ok(AudioMode::AudioOnly),
            2 => Ok(AudioMode::AudioOrThermal),
            3 => Ok(AudioMode::AudioAndThermal),
            _ => Err(()),
        }
    }
}
#[derive(Format, PartialEq)]
pub struct DeviceConfigInner {
    pub device_id: u32,
    device_name: SmallString,
    pub location: (f32, f32),
    pub location_altitude: Option<f32>,
    pub location_timestamp: Option<u64>,
    pub location_accuracy: Option<f32>,
    start_recording_time: (bool, i32),
    end_recording_time: (bool, i32),
    pub is_continuous_recorder: bool,
    pub use_low_power_mode: bool,
    pub audio_mode: AudioMode,
    pub audio_seed: u32,
}

impl DeviceConfigInner {
    pub fn is_continuous_recorder(&self) -> bool {
        self.is_continuous_recorder
            || (self.start_recording_time.0
                && self.end_recording_time.0
                && self.start_recording_time == self.end_recording_time)
    }

    pub fn is_audio_device(&self) -> bool {
        self.audio_mode != AudioMode::Disabled
    }

    pub fn is_audio_only_device(&self) -> bool {
        self.audio_mode == AudioMode::AudioOnly
    }

    pub fn time_is_in_recording_window(
        &self,
        date_time_utc: &chrono::DateTime<Utc>,
        window: Option<(chrono::DateTime<Utc>, chrono::DateTime<Utc>)>,
    ) -> bool {
        if self.is_continuous_recorder() {
            // info!("Continuous recording mode enabled");
            return true;
        }
        let start_time;
        let end_time;
        if window.is_some() {
            start_time = window.unwrap().0;
            end_time = window.unwrap().1;
        } else if let Ok((s, e)) = self.next_or_current_recording_window(date_time_utc) {
            start_time = s;
            end_time = e;
        } else {
            return false;
        }
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
            // info!(
            //     "Recording will start in {}h{}m and end in {}h{}m, window duration {}h{}m",
            //     starts_in_hours,
            //     starts_in_mins,
            //     ends_in_hours,
            //     ends_in_mins,
            //     window_hours,
            //     window_mins
            // );
        } else if end_time > *date_time_utc {
            // info!(
            //     "Recording will end in {}h{}m, window duration {}h{}m",
            //     ends_in_hours, ends_in_mins, window_hours, window_mins
            // );
        }
        *date_time_utc >= start_time && *date_time_utc <= end_time
    }

    #[allow(clippy::too_many_lines)]
    pub fn next_or_current_recording_window(
        &self,
        now_utc: &chrono::DateTime<Utc>,
    ) -> Result<(chrono::DateTime<Utc>, chrono::DateTime<Utc>), ()> {
        let (is_absolute_start, mut start_offset) = self.start_recording_time;
        let (is_absolute_end, mut end_offset) = self.end_recording_time;

        // Make sure start and end offsets are positive integers
        if is_absolute_end && end_offset < 0 {
            end_offset += 86_400;
        }
        if is_absolute_start && start_offset < 0 {
            start_offset += 86_400;
        }
        let (window_start, window_end) = if !is_absolute_start || !is_absolute_end {
            let (lat, lng) = self.location;
            let altitude = self.location_altitude;
            let yesterday_utc = *now_utc - Duration::days(1);
            let (_, yesterday_sunset) = sun_times(
                yesterday_utc,
                f64::from(lat),
                f64::from(lng),
                f64::from(altitude.unwrap_or(0.0)),
            )
            .unwrap();
            let yesterday_sunset = yesterday_sunset + Duration::seconds(i64::from(start_offset));
            let (today_sunrise, today_sunset) = sun_times(
                *now_utc,
                f64::from(lat),
                f64::from(lng),
                f64::from(altitude.unwrap_or(0.0)),
            )
            .unwrap();
            let today_sunrise = today_sunrise + Duration::seconds(i64::from(end_offset));
            let today_sunset = today_sunset + Duration::seconds(i64::from(start_offset));
            let tomorrow_utc = *now_utc + Duration::days(1);
            let (tomorrow_sunrise, tomorrow_sunset) = sun_times(
                tomorrow_utc,
                f64::from(lat),
                f64::from(lng),
                f64::from(altitude.unwrap_or(0.0)),
            )
            .unwrap();
            let tomorrow_sunrise = tomorrow_sunrise + Duration::seconds(i64::from(end_offset));
            let tomorrow_sunset = tomorrow_sunset + Duration::seconds(i64::from(start_offset));

            if *now_utc > today_sunset && *now_utc > tomorrow_sunrise {
                let two_days_from_now_utc = *now_utc + Duration::days(2);
                let (two_days_sunrise, _) = sun_times(
                    two_days_from_now_utc,
                    f64::from(lat),
                    f64::from(lng),
                    f64::from(altitude.unwrap_or(0.0)),
                )
                .unwrap();
                let two_days_sunrise = two_days_sunrise + Duration::seconds(i64::from(end_offset));
                (Some(tomorrow_sunset), Some(two_days_sunrise))
            } else if (*now_utc > today_sunset && *now_utc < tomorrow_sunrise)
                || (*now_utc < today_sunset && *now_utc > today_sunrise)
            {
                (Some(today_sunset), Some(tomorrow_sunrise))
            } else if *now_utc < tomorrow_sunset && *now_utc < today_sunrise {
                (Some(yesterday_sunset), Some(today_sunrise))
            } else {
                error!("Unable to calculate relative time window");
                return Err(());
            }
        } else {
            (None, None)
        };
        let mut start_time = if is_absolute_start {
            now_utc
                .with_time(
                    NaiveTime::from_num_seconds_from_midnight_opt(
                        u32::try_from(start_offset).expect("start offset should be positive"),
                        0,
                    )
                    .unwrap(),
                )
                .unwrap()
        } else {
            window_start.unwrap()
        };
        let mut end_time = if is_absolute_end {
            now_utc
                .with_time(
                    NaiveTime::from_num_seconds_from_midnight_opt(
                        u32::try_from(end_offset).expect("end offset should be positive"),
                        0,
                    )
                    .unwrap(),
                )
                .unwrap()
        } else {
            window_end.unwrap()
        };

        if is_absolute_start || is_absolute_end {
            let start_minus_one_day = start_time - Duration::days(1);
            let mut start_plus_one_day = start_time + Duration::days(1);
            let mut end_minus_one_day = end_time - Duration::days(1);
            let end_plus_one_day = end_time + Duration::days(1);

            if start_minus_one_day > end_minus_one_day {
                end_minus_one_day += Duration::days(1);
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
        Ok((start_time, end_time))
    }
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
        DeviceConfig {
            config_inner: DeviceConfigInner {
                device_id: 0,
                device_name: SmallString::new_from_bytes("default".as_bytes()),
                location: (0.0, 0.0),
                location_altitude: None,
                location_timestamp: None,
                location_accuracy: None,
                start_recording_time: (false, 0),
                end_recording_time: (false, 0),
                is_continuous_recorder: false,
                use_low_power_mode: false,
                audio_mode: AudioMode::Disabled,
                audio_seed: 0,
            },
            motion_detection_mask: DetectionMask::new(None),
            cursor_position: 0,
        }
    }
}

impl DeviceConfig {
    pub fn load_existing_config_from_flash(fs: &mut OnboardFlash) -> Option<DeviceConfig> {
        let slice = fs.read_device_config();
        if let Ok(slice) = slice {
            DeviceConfig::from_bytes(&slice)
        } else {
            None
        }
    }

    pub fn inner_from_bytes(bytes: &[u8]) -> Option<(DeviceConfigInner, usize)> {
        let mut cursor = Cursor::new(bytes);
        let device_id = cursor.read_u32();
        if device_id == u32::MAX {
            // Device config is uninitialised in flash
            return None;
        }
        let audio_mode = AudioMode::try_from(cursor.read_u8())
            .ok()
            .unwrap_or(AudioMode::Disabled);
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
        let device_name_length = cursor.read_u8();
        let mut device_name = [0u8; 64];
        device_name[0] = device_name_length;
        let len = device_name.len();
        let device_name = {
            let read_bytes = cursor
                .read(&mut device_name[1..(1 + usize::from(device_name_length)).min(len)])
                .unwrap();
            device_name
        };
        let device_name = SmallString::new(device_name);
        let audio_seed = cursor.read_u32();

        Some((
            DeviceConfigInner {
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
                audio_mode,
                audio_seed,
            },
            cursor.position(),
        ))
    }
    pub fn from_bytes(bytes: &[u8]) -> Option<DeviceConfig> {
        let (inner, mut cursor_pos) = Self::inner_from_bytes(bytes)?;

        // let mask_length = cursor.read_i32();
        let mut cursor = Cursor::new(bytes);
        cursor.set_position(cursor_pos);
        let mut motion_detection_mask;
        motion_detection_mask = DetectionMask::new(Some([0u8; 2400]));
        let len = cursor.read(&mut motion_detection_mask.inner).unwrap();

        if len == motion_detection_mask.inner.len() {
            cursor_pos = cursor.position();
        } else {
            // This payload came without the mask attached (i.e. from the rPi)
            motion_detection_mask.set_empty();
        }

        Some(DeviceConfig {
            config_inner: inner,
            motion_detection_mask,
            cursor_position: cursor_pos,
        })
    }

    pub fn config(&self) -> &DeviceConfigInner {
        &self.config_inner
    }

    pub fn is_audio_device(&self) -> bool {
        self.config_inner.is_audio_device()
    }

    pub fn is_audio_only_device(&self) -> bool {
        self.config_inner.is_audio_device()
    }

    pub fn audio_seed(&self) -> u32 {
        self.config_inner.audio_seed
    }

    pub fn audio_mode(&self) -> AudioMode {
        self.config_inner.audio_mode
    }

    pub fn records_audio_and_thermal(&self) -> bool {
        matches!(
            self.audio_mode(),
            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal
        )
    }

    pub fn records_audio_or_thermal(&self) -> bool {
        matches!(self.audio_mode(), AudioMode::AudioOrThermal)
    }

    pub fn device_name(&self) -> &str {
        self.config_inner.device_name.as_str()
    }

    pub fn device_name_bytes(&self) -> &[u8] {
        self.config_inner.device_name.as_bytes()
    }

    pub fn next_recording_window_start(
        &self,
        now_utc: &chrono::DateTime<Utc>,
    ) -> Result<chrono::DateTime<Utc>, ()> {
        if let Ok((start, ..)) = self.next_or_current_recording_window(now_utc) {
            Ok(start)
        } else {
            Err(())
        }
    }

    pub fn use_low_power_mode(&self) -> bool {
        self.config_inner.use_low_power_mode
    }

    pub fn use_high_power_mode(&self) -> bool {
        !self.config_inner.use_low_power_mode
    }

    pub fn next_or_current_recording_window(
        &self,
        now_utc: &chrono::DateTime<Utc>,
    ) -> Result<(chrono::DateTime<Utc>, chrono::DateTime<Utc>), ()> {
        self.config_inner.next_or_current_recording_window(now_utc)
    }

    pub fn time_is_in_daylight(&self, date_time_utc: &chrono::DateTime<Utc>) -> bool {
        let (lat, lng) = self.config_inner.location;
        let altitude = self.config_inner.location_altitude;
        let (sunrise, sunset) = sun_times(
            *date_time_utc,
            f64::from(lat),
            f64::from(lng),
            f64::from(altitude.unwrap_or(0.0)),
        )
        .unwrap();
        date_time_utc.hour() > sunrise.hour() && date_time_utc.hour() < sunset.hour()
    }

    fn time_is_in_recording_window(
        &self,
        date_time_utc: &chrono::DateTime<Utc>,
        window: Option<(chrono::DateTime<Utc>, chrono::DateTime<Utc>)>,
    ) -> bool {
        self.config_inner
            .time_is_in_recording_window(date_time_utc, window)
    }

    pub fn time_is_in_configured_recording_window(
        &self,
        date_time_utc: &chrono::DateTime<Utc>,
    ) -> bool {
        self.time_is_in_recording_window(date_time_utc, None)
    }

    pub fn time_is_in_supplied_recording_window(
        &self,
        date_time_utc: &chrono::DateTime<Utc>,
        window: (chrono::DateTime<Utc>, chrono::DateTime<Utc>),
    ) -> bool {
        self.time_is_in_recording_window(date_time_utc, Some(window))
    }
}

pub fn get_datetime_utc(
    year: u8,
    month: u8,
    day: u8,
    hours: u8,
    minutes: u8,
    seconds: u8,
) -> chrono::DateTime<Utc> {
    let naive_date =
        NaiveDate::from_ymd_opt(2000 + i32::from(year), u32::from(month), u32::from(day));

    assert!(
        naive_date.is_some(),
        "Couldn't get date for {}, {}, {}",
        2000 + i32::from(year),
        u32::from(month),
        u32::from(day)
    );
    let naive_time =
        NaiveTime::from_hms_opt(u32::from(hours), u32::from(minutes), u32::from(seconds));

    assert!(
        naive_time.is_some(),
        "Couldn't get time for {}, {}, {}",
        u32::from(hours),
        u32::from(minutes),
        u32::from(seconds),
    );

    let naive_datetime = NaiveDateTime::new(naive_date.unwrap(), naive_time.unwrap());
    naive_datetime.and_utc()
}
