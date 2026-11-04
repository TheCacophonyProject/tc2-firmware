use crate::event_logger::{
    Event, FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX,
    FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX, LoggerEvent,
};
use crate::onboard_flash::{BlockIndex, FLASH_USER_PAGE_SIZE};
use crate::tests::helpers::{
    ConfigBuilder, offloaded_event_count, simulate_camera_with_config,
    test_start_and_end_time_with_min_duration,
};
use crate::tests::mocks::fake_rpi_event_logger::LoggerEventKind;
use crate::tests::mocks::fake_shared_spi::StoragePage;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use byteorder::{ByteOrder, LittleEndian};
use chrono::Duration;
use crc::{CRC_16_XMODEM, Crc};
use std::string::String;
use std::vec;
use std::vec::Vec;
use test_log::test;

const BAD_BLOCK_MARKER_OFFSET: usize = 0x800;
const EVENTS_PER_BLOCK: usize = 256;

/// Marks `bad_blocks` as factory bad, and fills the good event log blocks with `num_events`
/// events, as they would be laid out by the firmware.
fn set_up_event_log(bad_blocks: &[BlockIndex], num_events: usize) {
    let good_blocks: Vec<BlockIndex> = (FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX
        ..FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX)
        .filter(|block| !bad_blocks.contains(block))
        .collect();
    let event_bytes = LoggerEvent::new_with_unknown_time(Event::GotPowerOnTimeout).as_bytes();
    TEST_SIM_STATE.with(|s| {
        let storage = &mut s.borrow_mut().flash_backing_storage;
        for block in bad_blocks {
            storage[usize::from(*block)].inner[0].inner[BAD_BLOCK_MARKER_OFFSET] = 0;
        }
        for event_index in 0..num_events {
            let block = good_blocks[event_index / EVENTS_PER_BLOCK];
            let page = (event_index % EVENTS_PER_BLOCK) / 4;
            let offset = (event_index % 4) * 512;
            storage[usize::from(block)].inner[page].inner[offset..offset + event_bytes.len()]
                .copy_from_slice(&event_bytes);
        }
    });
}

fn simulate_low_power_camera() {
    let config = ConfigBuilder::new()
        .low_power_mode()
        .audio_disabled()
        .custom_recording_window("10:00", "11:00")
        .build();
    let (start_time, end_time) =
        test_start_and_end_time_with_min_duration(&config, Duration::hours(3));
    let cptv_files = Some(vec![String::from("./test-fixtures/cat-trigger.cptv")]);
    simulate_camera_with_config(config, start_time, end_time, None, cptv_files);
}

fn num_preloaded_events_offloaded() -> usize {
    TEST_SIM_STATE.with(|s| {
        offloaded_event_count(&s.borrow().events_offloaded, LoggerEventKind::GotPowerOnTimeout)
    })
}

/// With only one good event block and that block full, the next event used to skip over the bad
/// blocks and be written to block 2048, past the end of the flash.
#[test]
fn events_are_not_written_past_the_end_of_flash() {
    set_up_event_log(&[2044, 2045, 2046, 2047], EVENTS_PER_BLOCK);
    simulate_low_power_camera();
    assert_eq!(EVENTS_PER_BLOCK, num_preloaded_events_offloaded());
}

/// With a bad block in the middle of the event log, two ranges of event indices used to map to
/// the same block. Booting with the first range full meant reading those events twice.
#[test]
fn events_do_not_overlap_after_a_bad_event_block() {
    set_up_event_log(&[2045], EVENTS_PER_BLOCK * 3);
    simulate_low_power_camera();
    assert_eq!(EVENTS_PER_BLOCK * 3, num_preloaded_events_offloaded());
}

/// Same overlap as above, but booting with one slot left in the first range, so that new events
/// cross into the second range without a restart. These used to overwrite the stored events.
#[test]
fn events_are_not_overwritten_after_a_bad_event_block() {
    let num_events = EVENTS_PER_BLOCK * 3 - 1;
    set_up_event_log(&[2045], num_events);
    simulate_low_power_camera();
    // Events are offloaded in index order, so each stored event should still be in its own slot
    TEST_SIM_STATE.with(|s| {
        let state = s.borrow();
        for (index, offload) in state.events_offloaded[..num_events].iter().enumerate() {
            assert_eq!(
                LoggerEventKind::GotPowerOnTimeout,
                offload.event.event,
                "Event at index {index} was overwritten"
            );
        }
    });
    assert_eq!(num_events, num_preloaded_events_offloaded());
}

const USER_METADATA_OFFSET: usize = 0x820;
const CPTV_SCHEDULED_RECORDING_TYPE: u8 = 0b100;
const PAGES_PER_BLOCK: usize = 64;
// All of block 0, and just page 0 of block 1.
const NUM_PAGES_ENDING_ON_PAGE_0: usize = PAGES_PER_BLOCK + 1;

fn write_recording_page(page: &mut StoragePage, data: &[u8], is_last: bool) {
    page.inner[..data.len()].copy_from_slice(data);
    let crc = Crc::<u16>::new(&CRC_16_XMODEM).checksum(data);
    let metadata = &mut page.inner[USER_METADATA_OFFSET..];
    metadata[0] = 0; // Page is used
    metadata[1] = if is_last { 0 } else { 0xff };
    LittleEndian::write_u16(&mut metadata[2..=3], data.len() as u16);
    metadata[4] = CPTV_SCHEDULED_RECORDING_TYPE;
    metadata[5] = CPTV_SCHEDULED_RECORDING_TYPE;
    LittleEndian::write_u16(&mut metadata[6..=7], data.len() as u16);
    LittleEndian::write_u16(&mut metadata[8..=9], crc);
    LittleEndian::write_u16(&mut metadata[10..=11], crc);
    LittleEndian::write_u16(&mut metadata[12..=13], 0); // File starts at block 0
}

/// Stores a recording that starts at block 0 and whose last page is page 0 of block 1, so that
/// page 1 of block 1 is blank. The offload only needs to be able to read the CPTV header, so a
/// truncated recording is fine.
fn store_recording_ending_on_page_0() {
    let recording = std::fs::read("./test-fixtures/cat-trigger.cptv").unwrap();
    let parts = recording[..NUM_PAGES_ENDING_ON_PAGE_0 * FLASH_USER_PAGE_SIZE]
        .chunks(FLASH_USER_PAGE_SIZE);
    TEST_SIM_STATE.with(|s| {
        let storage = &mut s.borrow_mut().flash_backing_storage;
        for (index, part) in parts.enumerate() {
            let is_last = index == NUM_PAGES_ENDING_ON_PAGE_0 - 1;
            let page = &mut storage[index / PAGES_PER_BLOCK].inner[index % PAGES_PER_BLOCK];
            write_recording_page(page, part, is_last);
        }
    });
}

/// A recording ending on page 0 of a block used to leave that block looking free to the flash
/// scan, as it decides that from page 1. The block was then never erased, and programming
/// page 0 again when a later recording reached it corrupted that page.
#[test]
fn recording_ending_on_page_0_of_a_block_is_erased() {
    store_recording_ending_on_page_0();
    simulate_low_power_camera();
    TEST_SIM_STATE.with(|s| {
        let state = s.borrow();
        let expected_size = NUM_PAGES_ENDING_ON_PAGE_0 * FLASH_USER_PAGE_SIZE;
        assert_eq!(
            1,
            state.files_offloaded.iter().filter(|file| file.size == expected_size).count(),
            "The recording ending on page 0 should be offloaded in full"
        );
        assert!(
            state.files_offloaded.len() > 1,
            "New recordings should have been made after it"
        );
    });
}
