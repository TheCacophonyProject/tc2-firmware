use crate::entry::real_main;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::log::error;
use crate::tests::stubs::fake_rpi_device_config::{AudioMode, DeviceConfig};
use crate::tests::test_state::test_global_state::{
    CPTV_FILES, CURRENT_THERMAL_WINDOW, CURRENT_TIME, DEVICE_CONFIG, ROSC_DRIVE_ITERATOR,
    THERMAL_TRIGGER_OFFSETS_MINS,
};
use chrono::{DateTime, Utc};

pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
    thermal_trigger_recordings_x_mins_into_thermal_window: Option<Vec<u32>>,
    cptv_files_to_playback: Option<Vec<String>>,
) {
    *CURRENT_TIME.lock().unwrap() = from;

    *THERMAL_TRIGGER_OFFSETS_MINS.lock().unwrap() =
        thermal_trigger_recordings_x_mins_into_thermal_window;
    *CPTV_FILES.lock().unwrap() = cptv_files_to_playback;

    *DEVICE_CONFIG.lock().unwrap() = Some(config.clone());
    error!(
        "Simulating camera from {} until {}",
        FormattedNZTime(from),
        FormattedNZTime(until)
    );
    let end_time = until;
    loop {
        let now = { *CURRENT_TIME.lock().unwrap() };
        if now > end_time {
            break;
        }

        // Reset ROSC_ITERATOR:
        *ROSC_DRIVE_ITERATOR.lock().unwrap() = 0;
        *CURRENT_THERMAL_WINDOW.lock().unwrap() =
            Some(config.next_recording_window(&now.naive_utc()));
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
