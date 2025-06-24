use crate::onboard_flash::{BlockIndex, OnboardFlash, PageIndex, TOTAL_FLASH_BLOCKS};
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Datelike, Timelike, Utc};
use core::ops::Range;
use defmt::{Format, error, info, warn};

use crate::audio_task::AlarmMode;
use crate::synced_date_time::SyncedDateTime;

#[repr(u8)]
#[derive(Format, Copy, Clone)]
pub enum WakeReason {
    Unknown = 0,
    ThermalOffload = 1,
    ThermalOffloadAfter24Hours = 2,
    ThermalHighPower = 3,
    AudioThermalEnded = 4,
    AudioShouldOffload = 5,
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

#[derive(Format, Copy, Clone)]
pub enum LoggerEventKind {
    Rp2040Sleep,
    OffloadedRecording,
    SavedNewConfig,
    StartedSendingFramesToRpi,
    StartedRecording,
    EndedRecording,
    ToldRpiToSleep,
    GotRpiPoweredDown,
    GotRpiPoweredOn,
    ToldRpiToWake(WakeReason),
    LostSync,
    SetAlarm(i64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
    WouldDiscardAsFalsePositive,
    StartedGettingFrames,
    FlashStorageNearlyFull,
    Rp2040WokenByAlarm,
    RtcCommError,
    AttinyCommError,
    Rp2040MissedAudioAlarm(i64),
    AudioRecordingFailed,
    ErasePartialOrCorruptRecording,
    StartedAudioRecording,
    ThermalMode,
    AudioMode,
    RecordingNotFinished,
    FileOffloadFailed,
    LogOffloadFailed,
    OffloadedLogs,
    CorruptFile,
    LostFrames(u64),
}

impl From<LoggerEventKind> for u16 {
    #[allow(clippy::enum_glob_use)]
    fn from(value: LoggerEventKind) -> Self {
        use LoggerEventKind::*;
        match value {
            Rp2040Sleep => 1,
            OffloadedRecording => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording => 5,
            EndedRecording => 6,
            ToldRpiToSleep => 7,
            GotRpiPoweredDown => 8,
            GotRpiPoweredOn => 9,
            ToldRpiToWake(_) => 10,
            LostSync => 11,
            SetAlarm(_) => 12,
            GotPowerOnTimeout => 13,
            WouldDiscardAsFalsePositive => 14,
            StartedGettingFrames => 15,
            FlashStorageNearlyFull => 16,
            Rp2040WokenByAlarm => 17,
            RtcCommError => 18,
            AttinyCommError => 19,
            Rp2040MissedAudioAlarm(_) => 20,
            AudioRecordingFailed => 21,
            ErasePartialOrCorruptRecording => 22,
            StartedAudioRecording => 23,
            ThermalMode => 24,
            AudioMode => 25,
            RecordingNotFinished => 26,
            FileOffloadFailed => 27,
            OffloadedLogs => 28,
            LogOffloadFailed => 29,
            CorruptFile => 30,
            LostFrames(_) => 31,
        }
    }
}

impl TryFrom<u16> for LoggerEventKind {
    type Error = ();

    #[allow(clippy::enum_glob_use)]
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        use LoggerEventKind::*;
        match value {
            1 => Ok(Rp2040Sleep),
            2 => Ok(OffloadedRecording),
            3 => Ok(SavedNewConfig),
            4 => Ok(StartedSendingFramesToRpi),
            5 => Ok(StartedRecording),
            6 => Ok(EndedRecording),
            7 => Ok(ToldRpiToSleep),
            8 => Ok(GotRpiPoweredDown),
            9 => Ok(GotRpiPoweredOn),
            10 => Ok(ToldRpiToWake(WakeReason::Unknown)),
            11 => Ok(LostSync),
            12 => Ok(SetAlarm(0)),
            13 => Ok(GotPowerOnTimeout),
            14 => Ok(WouldDiscardAsFalsePositive),
            15 => Ok(StartedGettingFrames),
            16 => Ok(FlashStorageNearlyFull),
            17 => Ok(Rp2040WokenByAlarm),
            18 => Ok(RtcCommError),
            19 => Ok(AttinyCommError),
            20 => Ok(Rp2040MissedAudioAlarm(0)),
            21 => Ok(AudioRecordingFailed),
            22 => Ok(ErasePartialOrCorruptRecording),
            23 => Ok(StartedAudioRecording),
            24 => Ok(ThermalMode),
            25 => Ok(AudioMode),
            26 => Ok(RecordingNotFinished),
            27 => Ok(FileOffloadFailed),
            28 => Ok(OffloadedLogs),
            29 => Ok(LogOffloadFailed),
            30 => Ok(CorruptFile),
            31 => Ok(LostFrames(0)),
            _ => Err(()),
        }
    }
}

type EventIndex = u16;

// TODO: Maybe have a map of various event payload sizes?
#[derive(Copy, Clone)]
pub struct LoggerEvent {
    timestamp: i64,
    kind: LoggerEventKind,
}

impl LoggerEvent {
    pub fn new(event: LoggerEventKind, synced_date_time: &SyncedDateTime) -> LoggerEvent {
        LoggerEvent {
            kind: event,
            timestamp: synced_date_time.get_timestamp_micros(),
        }
    }

    pub fn new_with_unknown_time(event: LoggerEventKind) -> LoggerEvent {
        LoggerEvent {
            kind: event,
            timestamp: 0,
        }
    }

    pub fn timestamp(&self) -> Option<DateTime<Utc>> {
        DateTime::from_timestamp_micros(self.timestamp)
    }
}
pub const MAX_EVENTS_IN_LOGGER: u16 = 1024;
//- 4 * 64; //leave last page for config stuff

const EVENT_CODE_LENGTH: u8 = 2;
const EVENT_TIMESTAMP_LENGTH: u8 = 8;
const EVENT_PAYLOAD_LENGTH: u8 = 8;
const EVENT_LENGTH: u8 = EVENT_CODE_LENGTH + EVENT_TIMESTAMP_LENGTH;

const FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX: u16 = TOTAL_FLASH_BLOCKS - 4;
const FLASH_STORAGE_EVENT_LOG_END_BLOCK_INDEX: u16 = TOTAL_FLASH_BLOCKS;
pub struct EventLogger {
    next_event_index: Option<EventIndex>,
}
impl EventLogger {
    pub fn new(flash_storage: &mut OnboardFlash) -> EventLogger {
        // Scan all the pages
        let logger = EventLogger::init(0, None, flash_storage);
        for (i, event_index) in logger.event_range().enumerate() {
            let event = Self::get_event_at_index(event_index, flash_storage);
            if let Some(event) = event {
                let kind = LittleEndian::read_u16(&event[0..2]);
                if let Ok(kind) = LoggerEventKind::try_from(kind) {
                    let time = LittleEndian::read_i64(&event[2..10]);
                    let utc_time = DateTime::<Utc>::from_timestamp_micros(time).unwrap();
                    let year = utc_time.year();
                    let month = utc_time.month();
                    let day = utc_time.day();
                    let hour = utc_time.hour();
                    let minute = utc_time.minute();
                    let second = utc_time.second();
                    info!(
                        "Event #{}, kind {}, time {}/{}/{} {}:{}:{}",
                        i, kind, year, month, day, hour, minute, second
                    );
                } else {
                    warn!("Unknown event kind found on flash log {}", kind);
                }
            }
        }

        logger
    }

    fn init(
        mut index: EventIndex,
        mut next_free_event_index: Option<EventIndex>,
        flash_storage: &mut OnboardFlash,
    ) -> EventLogger {
        while Self::get_event_at_index(index, flash_storage).is_some() {
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

    pub fn get_event_at_index(
        event_index: EventIndex,
        flash_storage: &mut OnboardFlash,
    ) -> Option<[u8; 18]> {
        // 4 partial writes per page, 64 pages per block.
        let block = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index / 256);
        let page = ((event_index % 256) / 4) as PageIndex;
        let page_offset = (event_index % 4) * 64; // Allocate 64 bytes for each event

        if block >= FLASH_STORAGE_EVENT_LOG_END_BLOCK_INDEX {
            None
        } else if flash_storage.read_page(block, page).is_ok() {
            let event =
                flash_storage.read_event_from_cache_at_column_offset_spi(block, page_offset);
            if event[0] == 0xff { None } else { Some(event) }
        } else {
            None
        }
    }

    pub fn has_events_to_offload(&self) -> bool {
        self.count() != 0
    }

    pub fn clear(&mut self, flash_storage: &mut OnboardFlash) {
        if self.has_events_to_offload() {
            let start_block_index = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX;
            let end_block_index = FLASH_STORAGE_EVENT_LOG_END_BLOCK_INDEX;
            for block_index in start_block_index..end_block_index {
                if flash_storage.bad_blocks.contains(&block_index) {
                    info!("Skipping erase of bad block {}", block_index);
                } else if flash_storage.erase_block(block_index).is_err() {
                    // FIXME: Return err and handle?
                    error!("Block erase failed for block {}", block_index);
                }
            }
            let logger = Self::init(0, None, flash_storage);
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

    pub fn latest_event_of_kind(
        &self,
        event_type: LoggerEventKind,
        flash_storage: &mut OnboardFlash,
    ) -> Option<LoggerEvent> {
        let event_indices = self.event_range();
        let total_events = event_indices.end;
        let mut latest_event: Option<LoggerEvent> = None;
        for event_index in event_indices {
            let event_bytes = Self::get_event_at_index(event_index, flash_storage);
            if let Some(event_bytes) = event_bytes {
                let kind = LittleEndian::read_u16(&event_bytes[0..2]);
                if let Ok(LoggerEventKind::OffloadedRecording) = LoggerEventKind::try_from(kind) {
                    let timestamp = LittleEndian::read_i64(&event_bytes[2..10]);
                    latest_event = Some(LoggerEvent {
                        timestamp,
                        kind: LoggerEventKind::OffloadedRecording,
                    });
                }
            }
        }
        latest_event
    }

    pub fn is_nearly_full(&self) -> bool {
        self.count() >= (MAX_EVENTS_IN_LOGGER - 3)
    }

    pub fn log_event(&mut self, event: LoggerEvent, flash_storage: &mut OnboardFlash) {
        if self.count() < MAX_EVENTS_IN_LOGGER {
            if let Some(next_event_index) = &mut self.next_event_index {
                let mut event_data = [0u8; 18];
                LittleEndian::write_u16(&mut event_data[0..2], event.kind.into());
                LittleEndian::write_i64(&mut event_data[2..10], event.timestamp);
                let ext_data = &mut event_data[10..18];
                match event.kind {
                    LoggerEventKind::SetAlarm(alarm_time)
                    | LoggerEventKind::Rp2040MissedAudioAlarm(alarm_time) => {
                        LittleEndian::write_i64(ext_data, alarm_time);
                    }
                    LoggerEventKind::LostFrames(data) => {
                        LittleEndian::write_u64(ext_data, data);
                    }
                    LoggerEventKind::ToldRpiToWake(data) => {
                        LittleEndian::write_u64(ext_data, u64::from(data));
                    }
                    _ => {}
                }
                // Write to the end of the flash storage.
                // We can do up to 4 partial page writes per page, so in a block of 64 pages we get 256 entries.
                // We reserve 4 blocks at the end of the flash memory for events, so 1280 events, which should be plenty.
                let event_index = *next_event_index;
                let block = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index / 256);
                let page = ((event_index % 256) / 4) as u8;
                let page_offset = (event_index % 4) * 64;
                flash_storage.write_event(&event_data, block, page, page_offset);
                *next_event_index += 1;
            }
        } else {
            warn!("Event log full");
        }
    }
}

//write some audio config into flash storage instead of rp240 flash
//rp2040 flash was causing some slow downs in the code after every write
const AUDIO_BLOCK: BlockIndex = 2047;
const AUDIO_PAGE: PageIndex = 0;
pub fn clear_audio_alarm(flash_storage: &mut OnboardFlash) {
    let _ = flash_storage.erase_block(AUDIO_BLOCK);
}

// write audio alarm into last block of event storage
// byte 0 is alarm mode 0 for audio 1 for thermal
// bytes 1-9 is millisecond timestamp of the alarm
pub fn write_audio_alarm(
    flash_storage: &mut OnboardFlash,
    alarm_dt: DateTime<Utc>,
    mode: AlarmMode,
) {
    let _ = flash_storage.erase_block(AUDIO_BLOCK);
    let page_offset = 0u16;
    let mut event_data = [0u8; 18];
    event_data[0] = mode as u8;
    LittleEndian::write_i64(&mut event_data[1..9], alarm_dt.timestamp_millis());
    flash_storage.write_event(&event_data, AUDIO_BLOCK, AUDIO_PAGE, page_offset);
}

pub fn get_audio_alarm(
    flash_storage: &mut OnboardFlash,
) -> (Result<AlarmMode, ()>, Option<DateTime<Utc>>) {
    let page_offset = 0u16;
    if flash_storage.read_page(AUDIO_BLOCK, AUDIO_PAGE).is_ok() {
        let event =
            flash_storage.read_event_from_cache_at_column_offset_spi(AUDIO_BLOCK, page_offset);
        if event[0..9].iter().all(|x: &u8| *x == u8::MAX) {
            return (Err(()), None);
        }
        let dt = DateTime::from_timestamp_millis(LittleEndian::read_i64(&event[1..9]));
        (AlarmMode::try_from(event[0]), dt)
    } else {
        (Err(()), None)
    }
}
