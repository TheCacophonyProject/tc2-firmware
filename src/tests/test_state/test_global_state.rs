extern crate std;
use crate::attiny_rtc_i2c::{CameraState, Tc2AgentState};
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::{BlockIndex, PageIndex};
use crate::tests::stubs::fake_rpi_device_config::DeviceConfig;
use crate::tests::stubs::fake_rpi_event_logger::{FileType, LoggerEvent};
use crate::tests::stubs::fake_rpi_recording_state::RecordingState;
use crate::tests::stubs::fake_shared_spi::{StorageBlock, StoragePage};
use chrono::{DateTime, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc};
use codec::decode::CptvFrame;
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

pub static CURRENT_TIME: LazyLock<Arc<Mutex<chrono::DateTime<Utc>>>> = LazyLock::new(|| {
    Arc::new(Mutex::new(Utc.from_utc_datetime(&NaiveDateTime::new(
        NaiveDate::from_ymd_opt(2025, 9, 23).unwrap(),
        NaiveTime::from_hms_opt(2, 9, 0).unwrap(),
    ))))
});
pub static LAST_FRAME: LazyLock<Arc<Mutex<Option<CptvFrame>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));

pub static FRAME_NUM: LazyLock<Arc<Mutex<u32>>> = LazyLock::new(|| Arc::new(Mutex::new(0)));

pub static ATTINY_FIRMWARE_VERSION: LazyLock<Arc<Mutex<u8>>> =
    LazyLock::new(|| Arc::new(Mutex::new(1)));

pub static CAMERA_STATE: LazyLock<Arc<Mutex<CameraState>>> =
    LazyLock::new(|| Arc::new(Mutex::new(CameraState::PoweredOff)));

pub static ATTINY_POWER_CTRL_STATE: LazyLock<Arc<Mutex<u8>>> =
    LazyLock::new(|| Arc::new(Mutex::new(0)));

pub static ATTINY_KEEP_ALIVE: LazyLock<Arc<Mutex<Instant>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Instant::now())));

pub static TC2_AGENT_STATE: LazyLock<Arc<Mutex<Tc2AgentState>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Tc2AgentState::from(0))));

pub static RTC_ALARM_STATE: LazyLock<Arc<Mutex<RtcAlarm>>> = LazyLock::new(|| {
    Arc::new(Mutex::new(RtcAlarm {
        minutes: 0,
        hours: 0,
        day: 0,
        weekday_alarm_mode: 0,
        enabled: 0,
    }))
});

pub static ECC_ERROR_ADDRESSES: LazyLock<Arc<Mutex<Vec<(BlockIndex, PageIndex)>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
pub static FLASH_BACKING_STORAGE: LazyLock<Arc<Mutex<Vec<StorageBlock>>>> = LazyLock::new(|| {
    Arc::new(Mutex::new(vec![
        StorageBlock {
            inner: [StoragePage {
                inner: [0xff; 2048 + 128]
            }; 64]
        };
        2048
    ]))
});
pub static PENDING_FORCED_OFFLOAD_REQUEST: LazyLock<Arc<Mutex<bool>>> =
    LazyLock::new(|| Arc::new(Mutex::new(false)));
pub static PREFER_NOT_TO_OFFLOAD_FILES_NOW: LazyLock<Arc<Mutex<bool>>> =
    LazyLock::new(|| Arc::new(Mutex::new(false)));
pub static FILE_DOWNLOAD: LazyLock<Arc<Mutex<Option<Vec<u8>>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));
pub static FILE_PART_COUNT: LazyLock<Arc<Mutex<usize>>> = LazyLock::new(|| Arc::new(Mutex::new(0)));
pub static FILE_DOWNLOAD_START: LazyLock<Arc<Mutex<Instant>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Instant::now())));
pub static FAKE_PI_RECORDING_STATE: LazyLock<Arc<Mutex<RecordingState>>> =
    LazyLock::new(|| Arc::new(Mutex::new(RecordingState::new())));

#[derive(Debug)]
pub struct FileOffload {
    pub size: usize,
    pub file_type: FileType,
    pub offloaded_at: DateTime<Utc>,
}

pub struct EventOffload {
    pub(crate) event: LoggerEvent,
    pub(crate) offloaded_at: DateTime<Utc>,
}

impl core::fmt::Debug for EventOffload {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "event offload at {}", FormattedNZTime(self.offloaded_at))
    }
}

pub static ROSC_DRIVE_ITERATOR: LazyLock<Arc<Mutex<usize>>> =
    LazyLock::new(|| Arc::new(Mutex::new(0)));

pub static FILES_OFFLOADED: LazyLock<Arc<Mutex<Vec<FileOffload>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
pub static EVENTS_OFFLOADED: LazyLock<Arc<Mutex<Vec<EventOffload>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));

pub static DEVICE_CONFIG: LazyLock<Arc<Mutex<Option<DeviceConfig>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));

pub static THERMAL_TRIGGER_OFFSETS_MINS: LazyLock<Arc<Mutex<Option<Vec<u32>>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));

pub static CPTV_FILES: LazyLock<Arc<Mutex<Option<Vec<String>>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));
pub static CURRENT_CPTV_FILE: LazyLock<Arc<Mutex<Option<String>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));
pub static CPTV_DECODER: LazyLock<Arc<Mutex<Option<codec::decode::CptvDecoder<std::fs::File>>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));

pub static CURRENT_THERMAL_WINDOW: LazyLock<Arc<Mutex<Option<(NaiveDateTime, NaiveDateTime)>>>> =
    LazyLock::new(|| Arc::new(Mutex::new(None)));
