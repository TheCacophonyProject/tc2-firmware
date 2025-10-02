use crate::constants::FIRMWARE_VERSION;
use crate::device_config::{AudioMode, DeviceConfig};
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::{BlockIndex, FileType, OnboardFlash, PageIndex, TOTAL_FLASH_BLOCKS};
use crate::re_exports::log::{error, info, warn};
use crate::synced_date_time::SyncedDateTime;
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Duration, Utc};
use core::cmp::PartialEq;
use core::ops::Range;

#[repr(u8)]
#[derive(Copy, Clone)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
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

impl From<WakeReason> for u8 {
    fn from(value: WakeReason) -> Self {
        value as u8
    }
}

impl From<WakeReason> for u64 {
    fn from(value: WakeReason) -> Self {
        value as u64
    }
}

impl TryFrom<u64> for WakeReason {
    type Error = ();

    fn try_from(value: u64) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(WakeReason::Unknown),
            1 => Ok(WakeReason::ThermalOffload),
            2 => Ok(WakeReason::ThermalOffloadAfter24Hours),
            3 => Ok(WakeReason::ThermalHighPower),
            4 => Ok(WakeReason::AudioThermalEnded),
            5 => Ok(WakeReason::AudioShouldOffload),
            6 => Ok(WakeReason::AudioTooFull),
            7 => Ok(WakeReason::ThermalTooFull),
            8 => Ok(WakeReason::EventsTooFull),
            9 => Ok(WakeReason::OffloadTestRecording),
            10 => Ok(WakeReason::OffloadOnUserDemand),
            11 => Ok(WakeReason::RtcTimeCompromised),
            12 => Ok(WakeReason::OpportunisticOffload),
            _ => Err(()),
        }
    }
}

#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
#[derive(Copy, Clone)]
pub struct DiscardedRecordingInfo {
    pub recording_type: FileType,
    pub num_frames: u16,
    pub seconds_since_last_ffc: u16,
}

impl DiscardedRecordingInfo {
    #[allow(clippy::trivially_copy_pass_by_ref)]
    fn as_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0] = self.recording_type as u8;
        LittleEndian::write_u16(&mut bytes[1..=2], self.num_frames);
        LittleEndian::write_u16(&mut bytes[3..=4], self.seconds_since_last_ffc);
        bytes
    }

    fn from_bytes(bytes: &[u8]) -> Self {
        DiscardedRecordingInfo {
            recording_type: FileType::from(bytes[0]),
            num_frames: LittleEndian::read_u16(&bytes[1..=2]),
            seconds_since_last_ffc: LittleEndian::read_u16(&bytes[3..=4]),
        }
    }
}

#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
#[derive(Copy, Clone)]
pub struct NewConfigInfo {
    audio_mode: AudioMode,
    continuous_recorder: bool,
    high_power_mode: bool,
    firmware_version: u32,
}

impl NewConfigInfo {
    pub fn from_config(config: &DeviceConfig) -> Self {
        Self {
            audio_mode: config.audio_mode(),
            continuous_recorder: config.is_continuous_recorder(),
            high_power_mode: config.use_high_power_mode(),
            firmware_version: FIRMWARE_VERSION,
        }
    }

    #[allow(clippy::trivially_copy_pass_by_ref)]
    fn as_bytes(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        bytes[0] = self.audio_mode as u8;
        bytes[1] = u8::from(self.continuous_recorder);
        bytes[2] = u8::from(self.high_power_mode);
        LittleEndian::write_u32(&mut bytes[3..=6], self.firmware_version);
        bytes
    }

    fn from_bytes(bytes: &[u8]) -> Self {
        NewConfigInfo {
            audio_mode: AudioMode::try_from(bytes[0]).unwrap_or(AudioMode::Disabled),
            continuous_recorder: bytes[1] == 1,
            high_power_mode: bytes[2] == 1,
            firmware_version: LittleEndian::read_u32(&bytes[3..=6]),
        }
    }
}

#[cfg_attr(feature = "no-std", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(Debug))]
#[derive(Copy, Clone)]
pub enum Event {
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
    SetThermalAlarm(i64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
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
    CorruptFile, // Unused?
    LostFrames(u64),
    FileOffloadInterruptedByUser,
    RtcVoltageLowError,
    Rp2040GotNewConfig(NewConfigInfo),
    UnrecoverableDataCorruption((u16, u16)),
}

#[cfg(feature = "std")]
impl core::fmt::Display for Event {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        write!(f, "{:?}", self)
    }
}

impl From<Event> for u16 {
    #[allow(clippy::enum_glob_use)]
    fn from(value: Event) -> Self {
        use Event::*;
        match value {
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

impl TryFrom<&[u8; EVENT_LENGTH]> for LoggerEvent {
    type Error = ();

    #[allow(clippy::enum_glob_use)]
    fn try_from(event: &[u8; EVENT_LENGTH]) -> Result<Self, Self::Error> {
        use Event::*;
        let kind = LittleEndian::read_u16(&event[0..2]);
        let timestamp = LittleEndian::read_i64(&event[2..10]);
        let payload = &event[10..18];
        let ext_data = LittleEndian::read_u64(payload);
        let kind = match kind {
            1 => Ok(Rp2040Sleep),
            2 => Ok(OffloadedRecording(FileType::from(payload[0]))),
            3 => Ok(SavedNewConfig),
            4 => Ok(StartedSendingFramesToRpi),
            5 => Ok(StartedRecording),
            6 => Ok(EndedRecording),
            7 => Ok(ToldRpiToSleep),
            8 => Ok(GotRpiPoweredDown),
            9 => Ok(GotRpiPoweredOn),
            10 => Ok(ToldRpiToWake(
                ext_data.try_into().unwrap_or(WakeReason::Unknown),
            )),
            11 => Ok(LostSync),
            12 =>
            {
                #[allow(clippy::cast_possible_wrap)]
                Ok(SetAudioAlarm(ext_data as i64))
            }
            13 => Ok(GotPowerOnTimeout),
            14 => Ok(WouldDiscardAsFalsePositive(
                DiscardedRecordingInfo::from_bytes(payload),
            )),
            15 => Ok(StartedGettingFrames),
            16 => Ok(FlashStorageNearlyFull),
            17 => Ok(Rp2040WokenByAlarm),
            18 => Ok(RtcCommError),
            19 => Ok(AttinyCommError),
            20 =>
            {
                #[allow(clippy::cast_possible_wrap)]
                Ok(Rp2040MissedAudioAlarm(ext_data as i64))
            }
            21 => Ok(AudioRecordingFailed),
            22 => Ok(ErasePartialOrCorruptRecording(
                DiscardedRecordingInfo::from_bytes(payload),
            )),
            23 => Ok(StartedAudioRecording),
            24 => Ok(ThermalMode),
            25 => Ok(AudioMode),
            26 => Ok(RecordingNotFinished),
            27 => Ok(FileOffloadFailed),
            28 => Ok(OffloadedLogs),
            29 => Ok(LogOffloadFailed),
            30 => Ok(CorruptFile),
            31 => Ok(LostFrames(ext_data)),
            32 => Ok(FileOffloadInterruptedByUser),
            33 => Ok(RtcVoltageLowError),
            34 =>
            {
                #[allow(clippy::cast_possible_wrap)]
                Ok(SetThermalAlarm(ext_data as i64))
            }
            35 => Ok(Rp2040GotNewConfig(NewConfigInfo::from_bytes(payload))),
            36 => Ok(UnrecoverableDataCorruption((
                LittleEndian::read_u16(&payload[0..=1]),
                LittleEndian::read_u16(&payload[2..=3]),
            ))),
            _ => Err(()),
        }?;
        Ok(LoggerEvent { timestamp, kind })
    }
}

type EventIndex = u16;

#[derive(Copy, Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
pub struct LoggerEvent {
    timestamp: i64,
    kind: Event,
}

#[cfg(feature = "std")]
impl core::fmt::Display for LoggerEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl LoggerEvent {
    pub fn new(event: Event, time: &SyncedDateTime) -> LoggerEvent {
        LoggerEvent {
            kind: event,
            timestamp: time.get_timestamp_micros(),
        }
    }

    pub fn new_with_unknown_time(event: Event) -> LoggerEvent {
        LoggerEvent {
            kind: event,
            timestamp: 0,
        }
    }

    pub fn timestamp(&self) -> Option<DateTime<Utc>> {
        DateTime::from_timestamp_micros(self.timestamp)
    }

    pub fn kind(&self) -> Event {
        self.kind
    }

    pub fn as_bytes(&self) -> [u8; EVENT_LENGTH] {
        let mut event_data = [0u8; EVENT_LENGTH];
        LittleEndian::write_u16(&mut event_data[0..2], self.kind.into());
        LittleEndian::write_i64(&mut event_data[2..10], self.timestamp);
        let ext_data = &mut event_data[10..18];
        match self.kind {
            Event::SetThermalAlarm(alarm_time)
            | Event::SetAudioAlarm(alarm_time)
            | Event::Rp2040MissedAudioAlarm(alarm_time) => {
                LittleEndian::write_i64(ext_data, alarm_time);
            }
            Event::LostFrames(data) => {
                LittleEndian::write_u64(ext_data, data);
            }
            Event::ToldRpiToWake(data) => {
                LittleEndian::write_u64(ext_data, u64::from(data));
            }
            Event::WouldDiscardAsFalsePositive(discard_info)
            | Event::ErasePartialOrCorruptRecording(discard_info) => {
                ext_data.copy_from_slice(&discard_info.as_bytes());
            }
            Event::Rp2040GotNewConfig(data) => {
                ext_data.copy_from_slice(&data.as_bytes());
            }
            Event::UnrecoverableDataCorruption(location) => {
                LittleEndian::write_u16(&mut ext_data[0..2], location.0);
                LittleEndian::write_u16(&mut ext_data[2..4], location.1);
            }
            Event::OffloadedRecording(file_type) => {
                ext_data[0] = file_type as u8;
            }
            _ => {}
        }
        event_data
    }
}
pub const MAX_EVENTS_IN_LOGGER: u16 = 1280;
const EVENT_CODE_LENGTH: usize = 2;
const EVENT_TIMESTAMP_LENGTH: usize = 8;
const EVENT_PAYLOAD_LENGTH: usize = 8;
const EVENT_LENGTH: usize = EVENT_CODE_LENGTH + EVENT_TIMESTAMP_LENGTH + EVENT_PAYLOAD_LENGTH;

// 2043, 2044, 2045, 2046, 2047: event log
// 2041, 2042: device config

const FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX: u16 = TOTAL_FLASH_BLOCKS - 5;
const FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX: u16 = TOTAL_FLASH_BLOCKS;
pub struct EventLogger {
    next_event_index: Option<EventIndex>,
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        u16::from(*self) == u16::from(*other)
    }
}

impl EventLogger {
    pub fn new(fs: &mut OnboardFlash) -> EventLogger {
        // Scan all the pages
        EventLogger::init(0, None, fs)
    }

    fn init(
        mut index: EventIndex,
        mut next_free_event_index: Option<EventIndex>,
        fs: &mut OnboardFlash,
    ) -> EventLogger {
        while Self::get_event_at_index(index, fs).is_some() {
            index += 1;
        }
        if index < MAX_EVENTS_IN_LOGGER {
            next_free_event_index = Some(index);
        }

        if let Some(index) = next_free_event_index {
            info!(
                "Init EventLogger: next free event logger slot at {}, slots available {}",
                index,
                MAX_EVENTS_IN_LOGGER - index
            );
        } else {
            info!("Init EventLogger: no free slots available, logger full");
        }

        EventLogger {
            next_event_index: next_free_event_index,
        }
    }

    #[allow(dead_code)]
    pub fn list(&self, fs: &mut OnboardFlash) {
        for (i, event_index) in self.event_range().enumerate() {
            let event = Self::get_event_at_index(event_index, fs);
            if let Some(event) = event {
                Self::print_event(i, &event);
            }
        }
    }

    pub fn print_event(index: usize, event: &[u8; EVENT_LENGTH]) {
        if let Ok(event) = LoggerEvent::try_from(event) {
            let utc_time = DateTime::<Utc>::from_timestamp_micros(event.timestamp).unwrap();
            info!(
                "Event #{}, kind {}, time {}",
                index,
                event.kind,
                FormattedNZTime(utc_time),
            );
            let payload_time = match event.kind {
                Event::SetAudioAlarm(alarm_time) | Event::SetThermalAlarm(alarm_time) => {
                    let utc_time = DateTime::<Utc>::from_timestamp_micros(alarm_time).unwrap();
                    Some(utc_time)
                }
                _ => None,
            };
            if let Some(utc_time) = payload_time {
                info!("-- Alarm time {}", FormattedNZTime(utc_time),);
            }
        } else {
            warn!("Unknown event kind found on flash log {:?}", event);
        }
    }

    pub fn get_event_at_index_old(
        event_index: EventIndex,
        fs: &mut OnboardFlash,
    ) -> Option<[u8; EVENT_LENGTH]> {
        // 4 partial writes per page, 64 pages per block.
        let (block, page) = Self::address_for_event_index(event_index, fs);
        // Originally events started 64 bytes apart in the first page, but that broke ECC
        let page_offset = (event_index % 4) * 64;
        if block >= FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX {
            None
        } else if fs.read_page(block, page).is_ok() {
            let event = fs.read_event_from_cache_at_column_offset_spi(block, page_offset);
            if event[0] == 0xff { None } else { Some(event) }
        } else {
            // FIXME: Log potential bad block
            None
        }
    }

    pub fn get_event_at_index(
        event_index: EventIndex,
        fs: &mut OnboardFlash,
    ) -> Option<[u8; EVENT_LENGTH]> {
        // 4 partial writes per page, 64 pages per block.
        let (block, page) = Self::address_for_event_index(event_index, fs);
        // Events need to start at each of the User Main data 0..=3 blocks, otherwise ECC breaks.
        let page_offset = (event_index % 4) * 512;
        if block >= FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX {
            None
        } else if fs.read_page(block, page).is_ok() {
            let event = fs.read_event_from_cache_at_column_offset_spi(block, page_offset);
            if event[0] == 0xff {
                // TODO: Temporary migration code, take out in version 21
                Self::get_event_at_index_old(event_index, fs)
            } else {
                Some(event)
            }
        } else {
            // FIXME: Log potential bad block
            None
        }
    }

    pub fn has_events_to_offload(&self, fs: &mut OnboardFlash) -> bool {
        self.count() > 0
            && !(self.count() == 1
                && self
                    .latest_event(fs)
                    .is_some_and(|event| event.kind == Event::OffloadedLogs))
    }

    pub fn clear(&mut self, fs: &mut OnboardFlash) {
        if self.has_events_to_offload(fs) {
            let start_block_index = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX;
            let end_block_index = FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX;
            for block_index in start_block_index..end_block_index {
                if fs.erase_block(block_index).is_err() {
                    // FIXME: Return err and handle?
                    error!("Block erase failed for block {}", block_index);
                }
            }
            let logger = Self::init(0, None, fs);
            self.next_event_index = logger.next_event_index;
        } else {
            warn!("Cannot clear: event log already empty");
        }
    }

    fn last_event_index(&self) -> Option<EventIndex> {
        if let Some(index) = self.next_event_index {
            if index == 0 { None } else { Some(index - 1) }
        } else {
            Some(MAX_EVENTS_IN_LOGGER - 1)
        }
    }

    pub fn event_range(&self) -> Range<EventIndex> {
        0..self.next_event_index.unwrap_or(0)
    }

    pub fn count(&self) -> u16 {
        if let Some(last_event_index) = self.last_event_index() {
            last_event_index + 1
        } else {
            0
        }
    }

    pub fn latest_event(&self, fs: &mut OnboardFlash) -> Option<LoggerEvent> {
        if let Some(last_event_index) = self.last_event_index() {
            if let Some(event_bytes) = Self::get_event_at_index(last_event_index, fs) {
                LoggerEvent::try_from(&event_bytes).ok()
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn latest_event_of_kind(
        &self,
        event_type: Event,
        fs: &mut OnboardFlash,
    ) -> Option<LoggerEvent> {
        let event_indices = self.event_range();
        let mut latest_event: Option<LoggerEvent> = None;
        for event_index in event_indices {
            let event_bytes = Self::get_event_at_index(event_index, fs);
            if let Some(event_bytes) = event_bytes {
                let event = LoggerEvent::try_from(&event_bytes);
                if let Ok(event) = event
                    && event.kind == event_type
                {
                    latest_event = Some(event);
                }
            }
        }
        latest_event
    }

    pub fn latest_audio_recording_failed(&mut self, fs: &mut OnboardFlash) -> bool {
        self.latest_event_of_kind(Event::AudioRecordingFailed, fs)
            .is_some_and(|audio_failure_event| {
                let last_recording_success = self.latest_event_of_kind(Event::EndedRecording, fs);
                match last_recording_success {
                    None => true,
                    Some(recording_success_event) => {
                        if let Some(success_time) = recording_success_event.timestamp()
                            && let Some(failure_time) = audio_failure_event.timestamp()
                        {
                            success_time < failure_time
                        } else {
                            // Weird edge case where we won't re-attempt the audio recording.
                            false
                        }
                    }
                }
            })
    }

    pub fn is_nearly_full(&self) -> bool {
        self.count() >= (MAX_EVENTS_IN_LOGGER - 3)
    }

    pub fn log(&mut self, kind: Event, time: &SyncedDateTime, fs: &mut OnboardFlash) {
        self.log_event(LoggerEvent::new(kind, time), fs);
    }

    pub fn log_if_not_dupe(&mut self, kind: Event, time: &SyncedDateTime, fs: &mut OnboardFlash) {
        self.log_event_if_not_dupe(LoggerEvent::new(kind, time), fs);
    }

    pub fn address_for_event_index(
        event_index: EventIndex,
        fs: &OnboardFlash,
    ) -> (BlockIndex, PageIndex) {
        let mut block = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index / 256);
        while fs.bad_blocks.contains(&block) {
            block += 1;
        }
        let page = ((event_index % 256) / 4) as PageIndex;
        (block, page)
    }

    pub fn log_event(&mut self, event: LoggerEvent, fs: &mut OnboardFlash) {
        let count = self.count();
        if count < MAX_EVENTS_IN_LOGGER {
            if let Some(next_event_index) = &mut self.next_event_index {
                // Write to the end of the flash storage.
                // We can do up to 4 partial page writes per page, so in a block of 64 pages we get 256 entries.
                // We reserve 4 blocks at the end of the flash memory for events, so 1024 events, which should be plenty.
                let event_data = event.as_bytes();
                let event_index = *next_event_index;
                let (block, page) = Self::address_for_event_index(event_index, fs);
                let page_offset = (event_index % 4) * 512;
                fs.write_event(&event_data, block, page, page_offset);
                *next_event_index += 1;
            }
        } else {
            warn!("Event log full");
        }
    }

    pub fn log_event_if_not_dupe(&mut self, event: LoggerEvent, fs: &mut OnboardFlash) {
        let prev_event_is_dupe = self.latest_event(fs).is_some_and(|prev_event| {
            if prev_event.kind() == event.kind() {
                if prev_event.timestamp == event.timestamp {
                    true
                } else if let Some(prev_time) = prev_event.timestamp()
                    && let Some(time) = event.timestamp()
                    && time - prev_time < Duration::seconds(60)
                {
                    true
                } else {
                    false
                }
            } else {
                false
            }
        });
        if !prev_event_is_dupe {
            self.log_event(event, fs);
        }
    }
}
