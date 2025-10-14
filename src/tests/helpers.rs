use crate::device_config::{AudioMode, DeviceConfig as FirmwareDeviceConfig};
use crate::entry::real_main;
use crate::event_logger::{EventIndex, LoggerEvent, MAX_EVENTS_IN_LOGGER};
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::{
    BlockIndex, FLASH_SPI_HEADER, NUM_EVENT_BYTES, OnboardFlash, PageIndex,
};
use crate::re_exports::log::{error, info, warn};
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::stubs::fake_rpi_event_logger::{FileType, LoggerEventKind, WakeReason};
use crate::tests::stubs::fake_shared_spi::StorageBlock;
use crate::tests::test_state::test_global_state::{EventOffload, FileOffload, TEST_SIM_STATE};
use chrono::{DateTime, Duration, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc};
use chrono_tz::Tz::Pacific__Auckland;
use std::cmp::PartialEq;

pub fn simulate_camera_with_config(
    config: DeviceConfig,
    from: DateTime<Utc>,
    until: DateTime<Utc>,
    thermal_trigger_recordings_x_mins_into_thermal_window: Option<Vec<u32>>,
    cptv_files_to_playback: Option<Vec<String>>,
) {
    let simulation_hours = (until - from).num_hours();
    let max_restarts = ((simulation_hours as f32 / 24.0) * 100.0) as u32;
    error!("Simulation hours: {simulation_hours}, max restarts {max_restarts}");
    let firmware_config = rpi_device_config_to_firmware_device_config(&config);
    TEST_SIM_STATE.with(|state| {
        let mut state = state.borrow_mut();
        state.current_time = from;
        state.thermal_trigger_offsets_mins = thermal_trigger_recordings_x_mins_into_thermal_window;
        state.cptv_files = cptv_files_to_playback;
        state.device_config = Some(config.clone());

        if firmware_config.audio_mode() == AudioMode::AudioOnly {
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
        let now = TEST_SIM_STATE.with(|state| {
            let mut state = state.borrow_mut();
            // Reset ROSC_ITERATOR:
            state.rosc_drive_iterator = 0;

            if let Some(fw) = &state.firmware_device_config {
                // In continuous mode, always update the window
                if fw.is_continuous_recorder() && !fw.is_audio_only_device() {
                    let current_time = state.current_time;
                    state.current_thermal_window =
                        Some(fw.next_or_current_recording_window(&current_time).unwrap());
                }
            }

            state.current_time
        });
        if now >= end_time {
            break;
        }
        TEST_SIM_STATE.with(|state| {
            let state = state.borrow();
            if state.restart_num > max_restarts
                && !state.audio_recording_fails_on_restart_iteration.is_some()
            {
                panic!("Too many restarts, something is probably wrong");
            }
            warn!(
                "=== Entering test loop (iteration #{}) at {} ===",
                state.restart_num,
                FormattedNZTime(now)
            );
        });

        // Run the actual firmware code
        real_main();

        let now = TEST_SIM_STATE.with(|state| state.borrow().current_time);
        TEST_SIM_STATE.with(|state| {
            let mut state = state.borrow_mut();
            warn!(
                "=== Exiting test loop (iteration #{}) at {} ===",
                state.restart_num,
                FormattedNZTime(now)
            );
            state.restart_num += 1;
        });
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
stop-recording = '{end_recording}'
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
        info!("{}", config_toml);
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

pub fn next_or_current_thermal_window(config: &DeviceConfig) -> (DateTime<Utc>, DateTime<Utc>) {
    let date = test_start_day();
    info!("Test start date time {}", FormattedNZTime(date));
    let firmware_config = rpi_device_config_to_firmware_device_config(&config);

    // FIXME: Do fixed and 24/7 windows shift with daylight savings time?
    //  We're using naive UTC after all...
    firmware_config
        .next_or_current_recording_window(&date)
        .unwrap()
}

pub fn test_start_and_end_time(config: &DeviceConfig) -> (DateTime<Utc>, DateTime<Utc>) {
    let (start, end) = next_or_current_thermal_window(config);
    info!(
        "Next or current window {} - {}",
        FormattedNZTime(start),
        FormattedNZTime(end)
    );
    let duration = end - start;

    // If less than 24 hours, recalculate to a 24‑hour window centered on the midpoint
    let (extended_start, extended_end) = if duration < Duration::hours(24) {
        let mid = start + duration / 2;
        (mid - Duration::hours(12), mid + Duration::hours(12))
    } else {
        (start, end)
    };

    // Return the adjusted window with a small safety margin
    (
        extended_start - Duration::minutes(10),
        extended_end + Duration::minutes(10),
    )
}

pub fn num_audio_recordings_offloaded(files_offloaded: &Vec<FileOffload>) -> usize {
    files_offloaded
        .iter()
        .filter(|file| file.file_type == FileType::AudioScheduled)
        .count()
}

pub fn get_file_start_blocks(flash_storage: &Vec<StorageBlock>) -> Vec<u16> {
    let mut start_block_indexes = Vec::new();
    'outer: for (block_index, block) in flash_storage.iter().enumerate().take(2041) {
        'inner: for (page_index, page) in block.inner.iter().enumerate() {
            // Get the page, check the metadata
            if let Some(start_block_index) = page.file_start_block_index() {
                if !start_block_indexes.contains(&start_block_index) {
                    start_block_indexes.push(start_block_index);
                }
            } else {
                assert!(
                    !page.page_is_used(),
                    "Got used page with no start block index {}:{}",
                    block_index,
                    page_index
                );
                break 'inner;
            }
        }
    }
    start_block_indexes
}

pub fn num_audio_recordings_stored_in_flash(flash_storage: &Vec<StorageBlock>) -> usize {
    let mut num_audio_recordings = 0;
    let start_block_indexes = get_file_start_blocks(flash_storage);
    for start_block_index in start_block_indexes {
        let page = &flash_storage[start_block_index as usize].inner[0];
        if page.is_audio_recording() {
            num_audio_recordings += 1;
        }
    }
    num_audio_recordings
}

pub fn num_thermal_recordings_stored_in_flash(flash_storage: &Vec<StorageBlock>) -> usize {
    let mut num_thermal_recordings = 0;
    let start_block_indexes = get_file_start_blocks(flash_storage);
    for start_block_index in start_block_indexes {
        let page = &flash_storage[start_block_index as usize].inner[0];
        if page.is_cptv_recording() {
            num_thermal_recordings += 1;
        }
    }
    num_thermal_recordings
}

pub fn wake_reason_present(offloaded_events: &Vec<EventOffload>, wake_reason: WakeReason) -> bool {
    offloaded_events
        .iter()
        .any(|event| match event.event.event {
            LoggerEventKind::ToldRpiToWake(w) => w == wake_reason,
            _ => false,
        })
}

pub fn stored_events(flash_storage: &Vec<StorageBlock>) -> Vec<LoggerEvent> {
    let mut events = Vec::new();
    for i in 0..MAX_EVENTS_IN_LOGGER {
        match get_event_at_index(flash_storage, i) {
            Some(event) => {
                if let Ok(event) = LoggerEvent::try_from(&event) {
                    events.push(event);
                } else {
                    error!("Failed to convert event to logger event");
                }
            }
            None => {
                break;
            }
        }
    }
    events
}

fn get_bad_blocks(flash_storage: &Vec<StorageBlock>) -> Vec<u16> {
    let mut bad_blocks = Vec::new();
    for (block_index, block) in flash_storage.iter().enumerate() {
        if block.inner[0].is_part_of_bad_block() {
            bad_blocks.push(block_index as u16);
        }
    }
    bad_blocks
}

fn address_for_event_index(
    event_index: EventIndex,
    flash_storage: &Vec<StorageBlock>,
) -> (BlockIndex, PageIndex) {
    let bad_blocks = get_bad_blocks(flash_storage);
    let mut block =
        crate::event_logger::FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index / 256);
    while bad_blocks.contains(&block) {
        block += 1;
    }
    let page = ((event_index % 256) / 4) as PageIndex;
    (block, page)
}

fn get_event_at_index(
    flash_storage: &Vec<StorageBlock>,
    event_index: EventIndex,
) -> Option<[u8; crate::event_logger::EVENT_LENGTH]> {
    // 4 partial writes per page, 64 pages per block.
    let (block, page) = address_for_event_index(event_index, flash_storage);
    // Events need to start at each of the User Main data 0..=3 blocks, otherwise ECC breaks.
    let page_offset = ((event_index % 4) * 512) as usize;
    if block >= crate::event_logger::FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX {
        None
    } else {
        let mut event = [0u8; NUM_EVENT_BYTES];
        event.copy_from_slice(
            &flash_storage[block as usize].inner[page as usize].inner
                [page_offset..page_offset + NUM_EVENT_BYTES],
        );
        if event[0] == 0xff { None } else { Some(event) }
    }
}

pub fn num_thermal_recordings_offloaded(files_offloaded: &Vec<FileOffload>) -> usize {
    files_offloaded
        .iter()
        .filter(|file| {
            file.file_type == FileType::CptvScheduled
                || file.file_type == FileType::CptvStartup
                || file.file_type == FileType::CptvShutdown
        })
        .count()
}

pub fn startup_and_shutdown_recordings_made(files_offloaded: &Vec<FileOffload>) -> bool {
    files_offloaded
        .iter()
        .any(|file| file.file_type == FileType::CptvStartup)
        && files_offloaded
            .iter()
            .any(|file| file.file_type == FileType::CptvShutdown)
}

pub fn last_startup_recording_is_within_2_mins_of(
    files_offloaded: &Vec<FileOffload>,
    time: DateTime<Utc>,
) -> bool {
    let recording_time = files_offloaded
        .iter()
        .filter(|file| file.file_type == FileType::CptvStartup)
        .last()
        .unwrap()
        .recording_time;
    recording_time >= time && recording_time < time + Duration::minutes(2)
}

pub fn last_shutdown_recording_is_within_4_mins_of(
    files_offloaded: &Vec<FileOffload>,
    time: DateTime<Utc>,
) -> bool {
    let shutdown_recordings = files_offloaded
        .iter()
        .filter(|file| file.file_type == FileType::CptvShutdown);
    for shutdown_recording in shutdown_recordings {
        info!(
            "Shutdown recording at {}, window end at {}",
            FormattedNZTime(shutdown_recording.recording_time),
            FormattedNZTime(time)
        );
    }

    let recording_time = files_offloaded
        .iter()
        .filter(|file| file.file_type == FileType::CptvShutdown)
        .last()
        .unwrap()
        .recording_time;
    recording_time >= time - Duration::minutes(2) && recording_time <= time + Duration::minutes(2)
}

impl PartialEq for LoggerEventKind {
    fn eq(&self, other: &Self) -> bool {
        let a: u16 = self.clone().into();
        let b: u16 = other.clone().into();
        a == b
    }
}

pub fn offloaded_event_exists(
    events_offloaded: &Vec<EventOffload>,
    event: LoggerEventKind,
) -> bool {
    events_offloaded.iter().any(|e| e.event.event == event)
}
pub fn offloaded_event_count(
    events_offloaded: &Vec<EventOffload>,
    event: LoggerEventKind,
) -> usize {
    events_offloaded
        .iter()
        .filter(|e| e.event.event == event)
        .count()
}

pub fn offloaded_event_count_before_simulation_end_time(
    events_offloaded: &Vec<EventOffload>,
    event: LoggerEventKind,
    end_time: DateTime<Utc>,
) -> usize {
    events_offloaded
        .iter()
        .filter(|e| e.event.event == event && e.event.date_time() <= end_time)
        .count()
}

pub fn stored_event_count(flash_storage: &Vec<StorageBlock>, event: LoggerEventKind) -> usize {
    stored_events(flash_storage)
        .iter()
        .filter(|e| {
            let a: u16 = e.kind().into();
            let b: u16 = event.clone().into();
            a == b
        })
        .count()
}

pub fn offloaded_event_does_not_exist(
    events_offloaded: &Vec<EventOffload>,
    event: LoggerEventKind,
) -> bool {
    !offloaded_event_exists(events_offloaded, event)
}

pub fn stored_event_exists(flash_storage: &Vec<StorageBlock>, event: LoggerEventKind) -> bool {
    stored_events(flash_storage).iter().any(|e| {
        let a: u16 = e.kind().into();
        let b: u16 = event.clone().into();
        a == b
    })
}
