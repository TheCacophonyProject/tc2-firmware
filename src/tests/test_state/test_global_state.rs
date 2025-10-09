extern crate std;

use crate::attiny_rtc_i2c::{CameraState, Tc2AgentState};
use crate::device_config::DeviceConfig as FirmwareDeviceConfig;
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::{BlockIndex, PageIndex};
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::stubs::fake_rpi_event_logger::{FileType, LoggerEvent};
use crate::tests::stubs::fake_rpi_recording_state::RecordingState;
use crate::tests::stubs::fake_shared_spi::{StorageBlock, StoragePage};
use chrono::{DateTime, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc};
use codec::decode::CptvFrame;
use std::cell::RefCell;
use std::sync::{Arc, LazyLock, Mutex};
use std::time::Instant;
use std::vec;
use std::vec::Vec;

pub struct RtcAlarm {
    pub minutes: u8,
    pub(crate) hours: u8,
    pub(crate) day: u8,
    pub(crate) weekday_alarm_mode: u8,
    pub enabled: u8,
}

impl RtcAlarm {
    pub fn is_initialised(&self) -> bool {
        !(self.minutes == 0
            && self.hours == 0
            && self.day == 0
            && self.weekday_alarm_mode == 0
            && self.enabled == 0)
    }
}

pub struct SimState {
    pub(crate) current_time: DateTime<Utc>,
    pub(crate) last_frame: Option<CptvFrame>,
    pub(crate) frame_num: u32,
    pub(crate) expected_attiny_firmware_version: u8,
    pub(crate) camera_state: CameraState,
    pub(crate) attiny_power_ctrl_state: u8,
    pub(crate) attiny_keep_alive: DateTime<Utc>,
    pub(crate) tc2_agent_state: Tc2AgentState,
    pub(crate) rtc_alarm_state: RtcAlarm,
    pub(crate) ecc_error_addresses: Vec<(BlockIndex, PageIndex)>,
    pub(crate) flash_backing_storage: Vec<StorageBlock>,
    pub(crate) pending_forced_offload_request: bool,
    pub(crate) prefer_not_to_offload_files_now: bool,
    pub(crate) file_download: Option<Vec<u8>>,
    pub(crate) file_part_count: usize,
    pub(crate) file_download_start: Instant,
    pub(crate) fake_pi_recording_state: RecordingState,
    pub(crate) rosc_drive_iterator: usize,
    pub(crate) files_offloaded: Vec<FileOffload>,
    pub(crate) events_offloaded: Vec<EventOffload>,
    pub(crate) device_config: Option<DeviceConfig>,
    pub(crate) firmware_device_config: Option<FirmwareDeviceConfig>,
    pub(crate) thermal_trigger_offsets_mins: Option<Vec<u32>>,
    pub(crate) cptv_files: Option<Vec<String>>,
    pub(crate) current_cptv_file: Option<String>,
    pub(crate) cptv_decoder: Option<codec::decode::CptvDecoder<std::fs::File>>,
    pub(crate) current_thermal_window: Option<(DateTime<Utc>, DateTime<Utc>)>,
    pub(crate) current_test_window: (DateTime<Utc>, DateTime<Utc>),
}

pub struct FileOffload {
    pub size: usize,
    pub file_type: FileType,
    pub recording_time: DateTime<Utc>,
    pub offloaded_at: DateTime<Utc>,
}

impl core::fmt::Debug for FileOffload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(
            f,
            "{}",
            format!(
                r#"FileOffload {{
    size: {},
    file_type: {:?},
    recording_time: {},
    offloaded_at: {}
}}"#,
                self.size,
                self.file_type,
                FormattedNZTime(self.recording_time),
                FormattedNZTime(self.offloaded_at)
            )
        )
    }
}

pub struct EventOffload {
    pub(crate) event: LoggerEvent,
    pub(crate) offloaded_at: DateTime<Utc>,
}

impl core::fmt::Debug for EventOffload {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{}",
            format!(
                r#"EventOffload {{
    event: LoggerEvent {{
        event: {:?} {},
        time: {}
    }},
    offloaded_at: {},
}}"#,
                self.event.event,
                self.event.inner_time(),
                FormattedNZTime(
                    DateTime::from_timestamp_millis(self.event.timestamp / 1000)
                        .unwrap_or(chrono::Local::now().with_timezone(&Utc))
                ),
                FormattedNZTime(self.offloaded_at)
            )
        )
    }
}

thread_local! {
    pub static TEST_SIM_STATE: RefCell<SimState> = RefCell::new(SimState {
        current_time: Default::default(),
        last_frame: None,
        frame_num: 0,
        expected_attiny_firmware_version: 1,
        camera_state: CameraState::PoweringOn,
        attiny_power_ctrl_state: 0,
        attiny_keep_alive: Utc::now(),
        tc2_agent_state: Default::default(),
        rtc_alarm_state: RtcAlarm {
            minutes: 0,
            hours: 0,
            day: 0,
            weekday_alarm_mode: 0,
            enabled: 0
        },
        ecc_error_addresses: vec![],
        flash_backing_storage: vec![
            StorageBlock {
                inner: [StoragePage {
                    inner: [0xff; 2048 + 128]
                }; 64]
            };
            2048
        ],
        pending_forced_offload_request: false,
        prefer_not_to_offload_files_now: false,
        file_download: None,
        file_part_count: 0,
        file_download_start: Instant::now(),
        fake_pi_recording_state: RecordingState::new(),
        rosc_drive_iterator: 0,
        files_offloaded: vec![],
        events_offloaded: vec![],
        device_config: None,
        firmware_device_config: None,
        thermal_trigger_offsets_mins: None,
        current_test_window: (Utc::now(), Utc::now()),
        current_thermal_window: Some((Utc::now(), Utc::now())),
        cptv_files: None,
        current_cptv_file: None,
        cptv_decoder: None,
    });

}
//
// pub static CURRENT_TIME: LazyLock<Arc<Mutex<chrono::DateTime<Utc>>>> = LazyLock::new(|| {
//     Arc::new(Mutex::new(Utc.from_utc_datetime(&NaiveDateTime::new(
//         NaiveDate::from_ymd_opt(2025, 9, 23).unwrap(),
//         NaiveTime::from_hms_opt(2, 9, 0).unwrap(),
//     ))))
// });
// pub static LAST_FRAME: LazyLock<Arc<Mutex<Option<CptvFrame>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
//
// pub static FRAME_NUM: LazyLock<Arc<Mutex<u32>>> = LazyLock::new(|| Arc::new(Mutex::new(0)));
//
// pub static ATTINY_FIRMWARE_VERSION: LazyLock<Arc<Mutex<u8>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(1)));
//
// pub static CAMERA_STATE: LazyLock<Arc<Mutex<CameraState>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(CameraState::PoweredOff)));
//
// pub static ATTINY_POWER_CTRL_STATE: LazyLock<Arc<Mutex<u8>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(0)));
//
// pub static ATTINY_KEEP_ALIVE: LazyLock<Arc<Mutex<Instant>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Instant::now())));
//
// pub static TC2_AGENT_STATE: LazyLock<Arc<Mutex<Tc2AgentState>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Tc2AgentState::from(0))));
//
// pub static RTC_ALARM_STATE: LazyLock<Arc<Mutex<RtcAlarm>>> = LazyLock::new(|| {
//     Arc::new(Mutex::new(RtcAlarm {
//         minutes: 0,
//         hours: 0,
//         day: 0,
//         weekday_alarm_mode: 0,
//         enabled: 0,
//     }))
// });
//
// pub static ECC_ERROR_ADDRESSES: LazyLock<Arc<Mutex<Vec<(BlockIndex, PageIndex)>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
// pub static FLASH_BACKING_STORAGE: LazyLock<Arc<Mutex<Vec<StorageBlock>>>> = LazyLock::new(|| {
//     Arc::new(Mutex::new(vec![
//         StorageBlock {
//             inner: [StoragePage {
//                 inner: [0xff; 2048 + 128]
//             }; 64]
//         };
//         2048
//     ]))
// });
// pub static PENDING_FORCED_OFFLOAD_REQUEST: LazyLock<Arc<Mutex<bool>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(false)));
// pub static PREFER_NOT_TO_OFFLOAD_FILES_NOW: LazyLock<Arc<Mutex<bool>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(false)));
// pub static FILE_DOWNLOAD: LazyLock<Arc<Mutex<Option<Vec<u8>>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
// pub static FILE_PART_COUNT: LazyLock<Arc<Mutex<usize>>> = LazyLock::new(|| Arc::new(Mutex::new(0)));
// pub static FILE_DOWNLOAD_START: LazyLock<Arc<Mutex<Instant>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Instant::now())));
// pub static FAKE_PI_RECORDING_STATE: LazyLock<Arc<Mutex<RecordingState>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(RecordingState::new())));
//
// #[derive(Debug)]
// pub struct FileOffload {
//     pub size: usize,
//     pub file_type: FileType,
//     pub offloaded_at: DateTime<Utc>,
// }
//
// pub struct EventOffload {
//     pub(crate) event: LoggerEvent,
//     pub(crate) offloaded_at: DateTime<Utc>,
// }
//
// impl core::fmt::Debug for EventOffload {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//         write!(f, "event offload at {}", FormattedNZTime(self.offloaded_at))
//     }
// }
//
// pub static ROSC_DRIVE_ITERATOR: LazyLock<Arc<Mutex<usize>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(0)));
//
// pub static FILES_OFFLOADED: LazyLock<Arc<Mutex<Vec<FileOffload>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
// pub static EVENTS_OFFLOADED: LazyLock<Arc<Mutex<Vec<EventOffload>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
//
// pub static DEVICE_CONFIG: LazyLock<Arc<Mutex<Option<DeviceConfig>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
//
// pub static THERMAL_TRIGGER_OFFSETS_MINS: LazyLock<Arc<Mutex<Option<Vec<u32>>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
//
// pub static CPTV_FILES: LazyLock<Arc<Mutex<Option<Vec<String>>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
// pub static CURRENT_CPTV_FILE: LazyLock<Arc<Mutex<Option<String>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
// pub static CPTV_DECODER: LazyLock<Arc<Mutex<Option<codec::decode::CptvDecoder<std::fs::File>>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
//
// pub static CURRENT_THERMAL_WINDOW: LazyLock<Arc<Mutex<Option<(NaiveDateTime, NaiveDateTime)>>>> =
//     LazyLock::new(|| Arc::new(Mutex::new(None)));
