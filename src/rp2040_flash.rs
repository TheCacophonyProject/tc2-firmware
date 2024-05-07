use core::slice;
use rp2040_hal::rom_data;

use crate::utils::u16_slice_to_u8;
use defmt::{info, warn};

/// Taken from https://github.com/rp-rs/rp-hal/issues/257
/// This allows writing config data to the rp2040's 2MB companion flash chip.
///
/// Pico Bootrom source is here: https://github.com/raspberrypi/pico-bootrom/blob/master/bootrom/program_flash_generic.c
pub const BLOCK_SIZE: u32 = 65536;
pub const SECTOR_SIZE: usize = 4096;
pub const PAGE_SIZE: u32 = 256;
// These _ERASE commands are highly dependent on the flash chip you're using
pub const SECTOR_ERASE: u8 = 0x20; // Tested and works with W25Q16JV flash chip
pub const BLOCK32_ERASE: u8 = 0x52;
pub const BLOCK64_ERASE: u8 = 0xD8;
/* IMPORTANT NOTE ABOUT RP2040 FLASH SPACE ADDRESSES:
When you pass an `addr` to a `rp2040-hal::rom_data` function it wants
addresses that start at `0x0000_0000`. However, when you want to read
that data back using something like `slice::from_raw_parts()` you
need the address space to start at `0x1000_0000` (aka `FLASH_XIP_BASE`).
*/
pub const FLASH_XIP_BASE: u32 = 0x1000_0000;
pub const FLASH_END: u32 = 0x0020_0000;
pub const FLASH_USER_SIZE: u32 = 4096; // Amount dedicated to user device config
pub const FLASH_EVENT_LOG_SIZE: u32 = 256 * 4096; // Amount dedicated to user events

#[inline(never)]
#[link_section = ".data.ram_func"]

// 2460 bytes

pub fn write_device_config_to_rp2040_flash(data: &[u8]) {
    let addr = FLASH_END - FLASH_USER_SIZE;

    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_erase(addr, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
            rom_data::flash_range_program(addr, data.as_ptr(), data.len());
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
}
pub fn read_device_config_from_rp2040_flash() -> &'static [u8] {
    let addr = (FLASH_XIP_BASE + FLASH_END - FLASH_USER_SIZE) as *const u8;
    unsafe { slice::from_raw_parts(addr, FLASH_USER_SIZE as usize) }
}
pub fn read_alarm_from_rp2040_flash(offset: u32) -> &'static [u8] {
    let mut addr = FLASH_XIP_BASE + FLASH_END - FLASH_USER_SIZE + offset;
    if addr % 256 != 0 {
        addr = 256 * (1 + (addr / 256) as u32);
    }
    info!("Alarm read from {} offset {}", addr, offset);

    unsafe { slice::from_raw_parts(addr as *const u8, 2 as usize) }
}

#[inline(never)]
#[link_section = ".data.ram_func"]

pub fn write_alarm_schedule_to_rp2040_flash(alarm_hours: u8, alarm_minutes: u8, offset: u32) {
    let data = &[alarm_hours, alarm_minutes];
    let mut addr = FLASH_END - FLASH_USER_SIZE + offset;
    if addr % 256 != 0 {
        addr = 256 * (1 + (addr / 256) as u32);
    }
    info!(
        "Alarm time written to {} {} : {} offset {}",
        addr, alarm_hours, alarm_minutes, offset
    );
    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_program(addr, data.as_ptr(), 2);
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
}
