// TODO: Essentially we want to log an error code and a timestamp for each error/event to flash storage.
//  There would be a maximum number of errors we can store before we run out of memory.  Timestamp can
//  be in 32bit seconds past a given epoch (let's say Jan 1 2023).  Is a 1 second granularity enough?
use crate::onboard_flash::OnboardFlash;
use crate::rp2040_flash::{BLOCK_SIZE, FLASH_END, FLASH_EVENT_LOG_SIZE, FLASH_XIP_BASE, PAGE_SIZE};
use byteorder::{ByteOrder, LittleEndian};
use core::ops::Range;
use core::slice;
use defmt::{error, info, warn};

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
    ToldRpiToWake,
    LostSync,
    SetAlarm(u64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
}

impl Into<u16> for LoggerEventKind {
    fn into(self) -> u16 {
        use LoggerEventKind::*;
        match self {
            Rp2040Sleep => 1,
            OffloadedRecording => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording => 5,
            EndedRecording => 6,
            ToldRpiToSleep => 7,
            GotRpiPoweredDown => 8,
            GotRpiPoweredOn => 9,
            ToldRpiToWake => 10,
            LostSync => 11,
            SetAlarm(_) => 12,
            GotPowerOnTimeout => 13,
        }
    }
}

// TODO: Maybe have a map of various event payload sizes?
pub struct LoggerEvent {
    timestamp: u64,
    event: LoggerEventKind,
}

impl LoggerEvent {
    pub fn new(event: LoggerEventKind, timestamp: u64) -> LoggerEvent {
        LoggerEvent { event, timestamp }
    }
}
pub const MAX_EVENTS_IN_LOGGER: usize = 1024;

const EVENT_CODE_LENGTH: usize = 2;
const EVENT_TIMESTAMP_LENGTH: usize = 8;
const EVENT_PAYLOAD_LENGTH: usize = 8;
const EVENT_LENGTH: usize = EVENT_CODE_LENGTH + EVENT_TIMESTAMP_LENGTH;

const FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX: isize = 2048 - 4;
pub struct EventLogger {
    next_event_index: Option<usize>,
}
impl EventLogger {
    pub fn new(flash_storage: &mut OnboardFlash) -> EventLogger {
        // Scan all the pages
        let logger = EventLogger::init(0, None, flash_storage);
        for (i, event_index) in logger.event_range().enumerate() {
            let event = logger.event_at_index(event_index, flash_storage);
            if let Some(event) = event {
                let kind = LittleEndian::read_u16(&event[0..2]);
                let time = LittleEndian::read_u64(&event[2..2 + 8]);
                info!("Event #{}, kind {}, timestamp {}", event, kind, time);
            }
        }

        logger
    }

    fn init(
        mut index: usize,
        mut next_free_page_index: Option<usize>,
        flash_storage: &mut OnboardFlash,
    ) -> EventLogger {
        while Self::get_event_at_index(index, flash_storage).is_some() {
            index += 1;
        }
        if index < MAX_EVENTS_IN_LOGGER {
            next_free_page_index = Some(index);
        }
        info!(
            "Init EventLogger: next free event logger slot at {}",
            next_free_page_index
        );

        EventLogger {
            next_event_index: next_free_page_index,
        }
    }

    fn get_event_at_index(
        event_index: usize,
        flash_storage: &mut OnboardFlash,
    ) -> Option<[u8; 10]> {
        // 4 blocks per page, 64 pages per block.
        let block = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index as isize / 256);
        let page = ((event_index % 256) / 4) as isize;
        let page_offset = (event_index % 4) * 10;
        if block >= 2048 {
            None
        } else if flash_storage.read_page(block, page).is_ok() {
            let event = flash_storage
                .read_event_from_cache_at_column_offset_spi(block, page_offset as isize);
            info!("Got event {:?}", event);
            if event[0] != 0xff {
                Some(event)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn has_events_to_offload(&self) -> bool {
        self.next_event_index.is_some_and(|index| index != 0)
    }

    pub fn clear(&mut self, flash_storage: &mut OnboardFlash) {
        if self.has_events_to_offload() {
            let start_block_index = FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX;
            let end_block_index = 2048;
            for block_index in start_block_index..end_block_index {
                while flash_storage.bad_blocks.contains(&(block_index as i16)) {
                    info!("Skipping erase of bad block {}", block_index);
                    continue;
                }
                if !flash_storage.erase_block(block_index).is_ok() {
                    error!("Block erase failed for block {}", block_index);
                }
            }
            let logger = Self::init(0, None, flash_storage);
            self.next_event_index = logger.next_event_index;
        } else {
            warn!("Cannot clear: event log already empty");
        }
    }

    fn last_event_index(&self) -> Option<usize> {
        if let Some(index) = self.next_event_index {
            if index == 0 {
                None
            } else {
                Some(index - 1)
            }
        } else {
            Some(MAX_EVENTS_IN_LOGGER - 1)
        }
    }

    pub fn event_range(&self) -> Range<usize> {
        0..self.next_event_index.unwrap_or(0)
    }

    pub fn count(&self) -> usize {
        if let Some(last_event_index) = self.last_event_index() {
            last_event_index + 1
        } else {
            0
        }
    }

    pub fn event_at_index(
        &self,
        index: usize,
        flash_storage: &mut OnboardFlash,
    ) -> Option<[u8; 10]> {
        Self::get_event_at_index(index, flash_storage)
    }

    pub fn log_event(&mut self, event: LoggerEvent, flash_storage: &mut OnboardFlash) {
        if self.count() < MAX_EVENTS_IN_LOGGER {
            if let Some(next_event_index) = &mut self.next_event_index {
                let mut event_data = [0u8; 10];
                LittleEndian::write_u16(&mut event_data[0..2], event.event.into());
                LittleEndian::write_u64(&mut event_data[2..2 + 8], event.timestamp);
                // Write to the end of the flash storage.
                // We can do up to 4 partial page writes per page, so in a block of 64 pages we get 256 entries.
                // We reserve 4 blocks at the end of the flash memory for events, so 1280 events, which should be plenty.
                let event_index = *next_event_index;
                let block =
                    FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX + (event_index as isize / 256);
                let page = ((event_index % 256) / 4) as isize;
                let page_offset = (event_index % 4) * 10;
                flash_storage.write_event(&event_data, block, page, page_offset as u16);
                *next_event_index += 1;
            }
        } else {
            warn!("Event log full");
        }
    }
}
