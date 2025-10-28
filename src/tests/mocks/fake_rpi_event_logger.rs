extern crate std;

use crate::device_config::AudioMode;
use crate::formatted_time::FormattedNZTime;
use crate::re_exports::log::{debug, info};
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Utc};
use std::format;
use std::string::String;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeReason {
    Unknown = 0,
    ThermalOffload = 1,
    ThermalOffloadAfter24Hours = 2,
    ThermalHighPower = 3,
    AudioThermalEnded = 4,
    AudioShouldOffload = 5,
    AudioTooFull = 6,
    ThermalTooFull = 7,
    EventsTooFull = 8,
    OffloadTestRecording = 9,
    OffloadOnUserDemand = 10,
    RtcTimeCompromised = 11,
    OpportunisticOffload = 12,
}
impl core::fmt::Display for WakeReason {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl TryFrom<u8> for WakeReason {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use WakeReason::*;

        match value {
            0 => Ok(Unknown),
            1 => Ok(ThermalOffload),
            2 => Ok(ThermalOffloadAfter24Hours),
            3 => Ok(ThermalHighPower),
            4 => Ok(AudioThermalEnded),
            5 => Ok(AudioShouldOffload),
            6 => Ok(AudioTooFull),
            7 => Ok(ThermalTooFull),
            8 => Ok(EventsTooFull),
            9 => Ok(OffloadTestRecording),
            10 => Ok(OffloadOnUserDemand),
            _ => Err(()),
        }
    }
}

impl From<WakeReason> for u64 {
    fn from(value: WakeReason) -> Self {
        value as u64
    }
}

impl From<WakeReason> for u8 {
    fn from(value: WakeReason) -> Self {
        value as u8
    }
}

#[repr(u8)]
#[derive(PartialEq, Clone, Copy, Debug)]
pub enum FileType {
    Unknown = 0,
    CptvScheduled = 1 << 0,
    CptvUserRequested = 1 << 1,
    CptvStartup = 1 << 2,
    CptvShutdown = 1 << 3,
    AudioScheduled = 1 << 4,
    AudioUserRequested = 1 << 5,
    AudioStartup = 1 << 6,
    AudioShutdown = 1 << 7,
}

impl From<u8> for FileType {
    fn from(value: u8) -> Self {
        match value {
            0b0000_0001 => FileType::CptvScheduled,
            0b0000_0010 => FileType::CptvUserRequested,
            0b0000_0100 => FileType::CptvStartup,
            0b0000_1000 => FileType::CptvShutdown,
            0b0001_0000 => FileType::AudioScheduled,
            0b0010_0000 => FileType::AudioUserRequested,
            0b0100_0000 => FileType::AudioStartup,
            0b1000_0000 => FileType::AudioShutdown,
            _ => FileType::Unknown,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct DiscardedRecordingInfo {
    pub recording_type: FileType,
    pub num_frames: u16,
    pub seconds_since_last_ffc: u16,
}

impl DiscardedRecordingInfo {
    #[allow(clippy::trivially_copy_pass_by_ref)]
    #[allow(dead_code)]
    fn as_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0] = self.recording_type as u8;
        LittleEndian::write_u16(&mut bytes[1..=2], self.num_frames);
        LittleEndian::write_u16(&mut bytes[3..=4], self.seconds_since_last_ffc);
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        DiscardedRecordingInfo {
            recording_type: FileType::from(bytes[0]),
            num_frames: LittleEndian::read_u16(&bytes[1..=2]),
            seconds_since_last_ffc: LittleEndian::read_u16(&bytes[3..=4]),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct NewConfigInfo {
    audio_mode: AudioMode,
    continuous_recorder: bool,
    high_power_mode: bool,
    firmware_version: u32,
}

impl NewConfigInfo {
    #[allow(clippy::trivially_copy_pass_by_ref)]
    #[allow(dead_code)]
    fn as_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0] = self.audio_mode as u8;
        bytes[1] = u8::from(self.continuous_recorder);
        bytes[2] = u8::from(self.high_power_mode);
        LittleEndian::write_u32(&mut bytes[3..=6], self.firmware_version);
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        NewConfigInfo {
            audio_mode: AudioMode::try_from(bytes[0]).unwrap_or(AudioMode::Disabled),
            continuous_recorder: bytes[1] == 1,
            high_power_mode: bytes[2] == 1,
            firmware_version: LittleEndian::read_u32(&bytes[3..=6]),
        }
    }
}

#[derive(Debug, Clone)]
pub enum LoggerEventKind {
    Rp2040Sleep,
    OffloadedRecording(FileType),
    SavedNewConfig,
    StartedSendingFramesToRpi,
    StartedRecording,
    EndedRecording,
    ToldRpiToSleep,
    GotRpiPoweredDown,
    GotRpiPoweredOn,
    ToldRpiToWake(WakeReason),
    LostSync,
    SetAudioAlarm(i64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
    WouldDiscardAsFalsePositive(DiscardedRecordingInfo),
    StartedGettingFrames,
    FlashStorageNearlyFull,
    Rp2040WokenByAlarm,
    RtcCommError,
    AttinyCommError,
    Rp2040MissedAudioAlarm(i64),
    AudioRecordingFailed,
    ErasePartialOrCorruptRecording(DiscardedRecordingInfo),
    StartedAudioRecording,
    ThermalMode,
    AudioMode,
    RecordingNotFinished,
    FileOffloadFailed,
    LogOffloadFailed,
    OffloadedLogs,
    CorruptFile,
    LostFrames(u64),
    FileOffloadInterruptedByUser,
    RtcVoltageLowError,
    SetThermalAlarm(i64),
    Rp2040GotNewConfig(NewConfigInfo),
    UnrecoverableDataCorruption((u16, u16)),
}

impl From<LoggerEventKind> for u16 {
    fn from(val: LoggerEventKind) -> Self {
        use LoggerEventKind::*;
        match val {
            Rp2040Sleep => 1,
            OffloadedRecording(_) => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording => 5,
            EndedRecording => 6,
            ToldRpiToSleep => 7,
            GotRpiPoweredDown => 8,
            GotRpiPoweredOn => 9,
            ToldRpiToWake(_) => 10,
            LostSync => 11,
            SetAudioAlarm(_) => 12,
            GotPowerOnTimeout => 13,
            WouldDiscardAsFalsePositive(_) => 14,
            StartedGettingFrames => 15,
            FlashStorageNearlyFull => 16,
            Rp2040WokenByAlarm => 17,
            RtcCommError => 18,
            AttinyCommError => 19,
            Rp2040MissedAudioAlarm(_) => 20,
            AudioRecordingFailed => 21,
            ErasePartialOrCorruptRecording(_) => 22,
            StartedAudioRecording => 23,
            ThermalMode => 24,
            AudioMode => 25,
            RecordingNotFinished => 26,
            FileOffloadFailed => 27,
            OffloadedLogs => 28,
            LogOffloadFailed => 29,
            CorruptFile => 30,
            LostFrames(_) => 31,
            FileOffloadInterruptedByUser => 32,
            RtcVoltageLowError => 33,
            SetThermalAlarm(_) => 34,
            Rp2040GotNewConfig(_) => 35,
            UnrecoverableDataCorruption(_) => 36,
        }
    }
}

impl TryFrom<u16> for LoggerEventKind {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        use LoggerEventKind::*;
        match value {
            1 => Ok(Rp2040Sleep),
            2 => Ok(OffloadedRecording(FileType::Unknown)),
            3 => Ok(SavedNewConfig),
            4 => Ok(StartedSendingFramesToRpi),
            5 => Ok(StartedRecording),
            6 => Ok(EndedRecording),
            7 => Ok(ToldRpiToSleep),
            8 => Ok(GotRpiPoweredDown),
            9 => Ok(GotRpiPoweredOn),
            10 => Ok(ToldRpiToWake(WakeReason::Unknown)),
            11 => Ok(LostSync),
            12 => Ok(SetAudioAlarm(0)),
            13 => Ok(GotPowerOnTimeout),
            14 => Ok(WouldDiscardAsFalsePositive(
                DiscardedRecordingInfo::from_bytes(&[0u8; 8]),
            )),
            15 => Ok(StartedGettingFrames),
            16 => Ok(FlashStorageNearlyFull),
            17 => Ok(Rp2040WokenByAlarm),
            18 => Ok(RtcCommError),
            19 => Ok(AttinyCommError),
            20 => Ok(Rp2040MissedAudioAlarm(0)),
            21 => Ok(AudioRecordingFailed),
            22 => Ok(ErasePartialOrCorruptRecording(
                DiscardedRecordingInfo::from_bytes(&[0u8; 8]),
            )),
            23 => Ok(StartedAudioRecording),
            24 => Ok(ThermalMode),
            25 => Ok(AudioMode),
            26 => Ok(RecordingNotFinished),
            27 => Ok(FileOffloadFailed),
            28 => Ok(OffloadedLogs),
            29 => Ok(LogOffloadFailed),
            30 => Ok(CorruptFile),
            31 => Ok(LostFrames(0)),
            32 => Ok(FileOffloadInterruptedByUser),
            33 => Ok(RtcVoltageLowError),
            34 => Ok(SetThermalAlarm(0)),
            35 => Ok(Rp2040GotNewConfig(NewConfigInfo::from_bytes(&[0u8; 8]))),
            36 => Ok(UnrecoverableDataCorruption((u16::MAX, u16::MAX))),
            _ => Err(()),
        }
    }
}

pub struct LoggerEvent {
    pub timestamp: i64,
    pub event: LoggerEventKind,
}

impl LoggerEvent {
    pub fn date_time(&self) -> DateTime<Utc> {
        DateTime::from_timestamp_millis(self.timestamp / 1000)
            .unwrap_or(chrono::Local::now().with_timezone(&Utc))
    }

    pub fn date_time_inner(&self) -> Option<DateTime<Utc>> {
        match self.event {
            LoggerEventKind::Rp2040MissedAudioAlarm(ts)
            | LoggerEventKind::SetAudioAlarm(ts)
            | LoggerEventKind::SetThermalAlarm(ts) => Some(
                DateTime::from_timestamp_millis(ts / 1000)
                    .unwrap_or(chrono::Local::now().with_timezone(&Utc)),
            ),
            _ => None,
        }
    }

    pub fn inner_time(&self) -> String {
        match self.date_time_inner() {
            Some(date_time) => format!("({})", FormattedNZTime(date_time)),
            None => String::from(""),
        }
    }
}

impl LoggerEvent {
    pub fn new(event: LoggerEventKind, timestamp: i64) -> LoggerEvent {
        LoggerEvent { event, timestamp }
    }

    pub fn log(&self, json_payload: Option<String>) {
        // If the type is SavedNewConfig, maybe make the payload the config?
        if let LoggerEventKind::SetAudioAlarm(alarm) = self.event {
            // Microseconds to nanoseconds
            let json = format!(
                "SetAudioAlarm{{{}}}",
                format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::SetThermalAlarm(alarm) = self.event {
            // Microseconds to nanoseconds
            let json = format!(
                "SetThermalAlarm{{{}}}",
                format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::Rp2040MissedAudioAlarm(alarm) = self.event {
            // Microseconds to nanoseconds
            let json = format!(
                "Rp2040MissedAudioAlarm{{{}}}",
                format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::ToldRpiToWake(reason) = self.event {
            let json = format!(
                "ToldRpiToWake{{{}}}",
                format!(r#"{{ "wakeup-reason": {reason} }}"#)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::LostFrames(lost_frames) = self.event {
            let json = format!(
                "LostFrames{{{}}}",
                format!(r#"{{ "lost-frames": "{lost_frames}" }}"#)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::ErasePartialOrCorruptRecording(discard_info) = self.event {
            let recording_type = discard_info.recording_type;
            let json = format!(
                "ErasePartialOrCorruptRecording{{{}}}",
                format!(r#"{{ "recording-type": "{recording_type:?}" }}"#)
            );
            info!("Log Event {}", json);
        } else if let LoggerEventKind::WouldDiscardAsFalsePositive(discard_info) = self.event {
            let recording_type = discard_info.recording_type;
            let num_frames = discard_info.num_frames;
            let seconds_since_last_ffc = discard_info.seconds_since_last_ffc;
            let json = format!(
                "WouldDiscardAsFalsePositive{{{}}}",
                format!(
                    r#"{{ "recording-type": "{recording_type:?}", "num-frames": "{num_frames}", "seconds-since-last-ffc": "{seconds_since_last_ffc}" }}"#
                )
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::Rp2040GotNewConfig(new_config_info) = self.event {
            let audio_mode = new_config_info.audio_mode;
            let continuous_recorder = new_config_info.continuous_recorder;
            let high_power_mode = new_config_info.high_power_mode;
            let rp2040_firmware_version = new_config_info.firmware_version;
            let json = format!(
                "Rp2040GotNewConfig{{{}}}",
                format!(
                    r#"{{ "audio-mode": "{audio_mode:?}", "continuous-recorder": "{continuous_recorder}", "high-power-mode": "{high_power_mode}", "rp2040-firmware-version": "{rp2040_firmware_version}" }}"#
                )
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::UnrecoverableDataCorruption((block, page)) = self.event {
            let json = format!(
                "UnrecoverableDataCorruption{{{}}}",
                format!(r#"{{ "block": "{block}", "page": "{page}" }}"#)
            );
            debug!("Log Event {}", json);
        } else if let LoggerEventKind::OffloadedRecording(file_type) = self.event {
            let json = format!(
                "OffloadedRecording{{{}}}",
                format!(r#"{{ "file-type": "{file_type:?}" }}"#)
            );
            debug!("Log Event {}", json);
        } else {
            let json = format!(
                "{:?}{}",
                self.event,
                json_payload.unwrap_or(String::from("{}"))
            );
            debug!("Log Event {}", json);
        }
        // Microseconds to nanoseconds
        debug!("Event timestamp: {}", self.timestamp * 1000);
    }
}
