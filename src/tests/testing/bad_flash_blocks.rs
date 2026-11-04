use crate::event_logger::{
    Event, FLASH_STORAGE_EVENT_LOG_ONE_PAST_END_BLOCK_INDEX,
    FLASH_STORAGE_EVENT_LOG_START_BLOCK_INDEX, LoggerEvent,
};
use crate::onboard_flash::BlockIndex;
use crate::tests::helpers::{
    ConfigBuilder, offloaded_event_count, simulate_camera_with_config,
    test_start_and_end_time_with_min_duration,
};
use crate::tests::mocks::fake_rpi_event_logger::LoggerEventKind;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use chrono::Duration;
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
    assert_eq!(num_events, num_preloaded_events_offloaded());
}
