use crate::device_config::{AudioMode, DeviceConfig as FirmwareDeviceConfig};
use crate::entry::real_main;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::log::{error, info};
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::{DateTime, Duration, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc};
use chrono_tz::Tz::Pacific__Auckland;

pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
    thermal_trigger_recordings_x_mins_into_thermal_window: Option<Vec<u32>>,
    cptv_files_to_playback: Option<Vec<String>>,
) {
    TEST_SIM_STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.current_time = from;
        state.thermal_trigger_offsets_mins = thermal_trigger_recordings_x_mins_into_thermal_window;
        state.cptv_files = cptv_files_to_playback;
        state.device_config = Some(config.clone());
        let firmware_config = rpi_device_config_to_firmware_device_config(&config);

        if firmware_config.audio_mode() == AudioMode::AudioOnly {
            // FIXME: Is it true that we don't really have a current thermal window in audio only mode?
            state.current_thermal_window = None;
        } else {
            state.current_thermal_window = Some(
                firmware_config
                    .next_or_current_recording_window(&from)
                    .unwrap(),
            );
        }
        state.current_test_window = (from, until);
        state.firmware_device_config = Some(firmware_config);
    });
    error!(
        "Simulating camera from {} until {}",
        FormattedNZTime(from),
        FormattedNZTime(until)
    );
    let end_time = until;
    loop {
        let mut now = Utc::now();
        TEST_SIM_STATE.with(|state| {
            let mut state = state.borrow_mut();
            now = state.current_time;
            // Reset ROSC_ITERATOR:
            state.rosc_drive_iterator = 0;
            //state.current_thermal_window
            // *CURRENT_THERMAL_WINDOW.lock().unwrap() =
            //     Some(config.next_recording_window(&now.naive_utc()));
        });
        if now > end_time {
            break;
        }
        real_main();
    }
}

pub struct ConfigBuilder {
    low_power_mode: bool,
    audio_mode: AudioMode,
    location: Option<(f64, f64)>,
    recording_window: Option<(String, String)>,
}

impl ConfigBuilder {
    pub(crate) fn new() -> Self {
        Self {
            low_power_mode: false,
            audio_mode: AudioMode::Disabled,
            location: None,
            recording_window: None,
        }
    }

    pub(crate) fn low_power_mode(mut self) -> Self {
        self.low_power_mode = true;
        self
    }

    pub(crate) fn high_power_mode(mut self) -> Self {
        self.low_power_mode = false;
        self
    }

    pub(crate) fn audio_disabled(mut self) -> Self {
        self.audio_mode = AudioMode::Disabled;
        self
    }

    pub(crate) fn audio_and_thermal(mut self) -> Self {
        self.audio_mode = AudioMode::AudioAndThermal;
        self
    }

    pub(crate) fn audio_or_thermal(mut self) -> Self {
        self.audio_mode = AudioMode::AudioOrThermal;
        self
    }

    pub(crate) fn audio_only(mut self) -> Self {
        self.audio_mode = AudioMode::AudioOnly;
        self
    }

    pub(crate) fn with_location(mut self, latitude: f64, longitude: f64) -> Self {
        self.location = Some((latitude, longitude));
        self
    }

    pub(crate) fn fixed_recording_window(mut self) -> Self {
        self.recording_window = Some((String::from("10:00"), String::from("22:00")));
        self
    }

    pub(crate) fn always_on(mut self) -> Self {
        self.recording_window = Some((String::from("12:00"), String::from("12:00")));
        self
    }

    pub(crate) fn default_recording_window(mut self) -> Self {
        self.recording_window = Some((String::from("-30m"), String::from("+30m")));
        self
    }

    pub(crate) fn build(self) -> DeviceConfig {
        let location = self.location.unwrap_or((-43.5, 172.64));
        let latitude = location.0;
        let longitude = location.1;
        let use_low_power_mode = self.low_power_mode;
        let audio_mode = format!("{:?}", self.audio_mode);
        let recording_window = self
            .recording_window
            .unwrap_or((String::from("-30m"), String::from("+30m")));
        let start_recording = recording_window.0;
        let end_recording = recording_window.1;

        let windows = if self.audio_mode != AudioMode::AudioOnly {
            format!(
                r#"
[windows]
start-recording = '{start_recording}'
end-recording = '{end_recording}'
        "#
            )
        } else {
            String::from("")
        };
        let config_toml = format!(
            r#"
[device]
id = 123
name = "CI-fake-test-device"

[location]
accuracy = 0.0
altitude = 0.0
latitude = {latitude}
longitude = {longitude}

[thermal-recorder]
use-low-power-mode = {use_low_power_mode}

{windows}

[audio-recording]
audio-mode = '{audio_mode}'
random-seed = 0
"#,
        );
        let config: DeviceConfig = toml::from_str(&config_toml).unwrap();
        config
    }
}

pub fn rpi_device_config_to_firmware_device_config(
    device_config: &DeviceConfig,
) -> FirmwareDeviceConfig {
    let mut device_config_slice = [0u8; 1024];
    device_config.write_to_slice(&mut device_config_slice[6..], false, false);
    FirmwareDeviceConfig::from_bytes(&device_config_slice).unwrap()
}

pub fn test_start_day() -> DateTime<Utc> {
    Utc.from_utc_datetime(&NaiveDateTime::new(
        NaiveDate::from_ymd_opt(2025, 9, 23).unwrap(),
        NaiveTime::from_hms_opt(2, 0, 0).unwrap(),
    ))
}

pub fn test_start_and_end_time(config: &DeviceConfig) -> (DateTime<Utc>, DateTime<Utc>) {
    let date = test_start_day();
    let firmware_config = rpi_device_config_to_firmware_device_config(&config);

    // FIXME: Do fixed and 24/7 windows shift with daylight savings time?
    //  We're using naive UTC after all...

    // TODO: If the duration of the window is less than 24 hours, we should extend the window to be
    //  24 hrs with the thermal window centered in the middle of the 24hr window.

    let next_or_current_window = firmware_config
        .next_or_current_recording_window(&date)
        .unwrap();
    (
        next_or_current_window.0 - Duration::minutes(10),
        next_or_current_window.1 + Duration::minutes(10),
    )
}
