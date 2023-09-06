// Onboard flash owns the SPI peripheral it needs to function.
// When it initialises, it looks for a version number written at the beginning of the flash address
// space.  If the version doesn't match the known version it re-inits the flash.
// If there's no version number, it initialises a basic file table.

// The file table just contains a series of offsets into the flash memory, and we simply
// allocate linearly, relying on the fact we should be able to get 100,000 writes just fine.

// If we need some kind of bad-block detection we can worry about implementing that later.

// We keep a pointer to the next free byte offset updated, though we may only be able to write things
// in blocks of a certain size.

use crate::bsp::pac::SPI1;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use cortex_m::asm::nop;
use cortex_m::delay::Delay;
use defmt::{error, info, println, warn};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
    _embedded_hal_spi_FullDuplex,
};
use rp2040_hal::gpio::bank0::Gpio9;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::{Output, PushPull};
use rp2040_hal::spi::Enabled;
use rp2040_hal::Spi;

const WRITE_ENABLE: u8 = 0x06;

const BLOCK_ERASE: u8 = 0xd8;
const PROGRAM_LOAD: u8 = 0x02;
const PROGRAM_EXECUTE: u8 = 0x10;
const RESET: u8 = 0xff;
const GET_FEATURES: u8 = 0x0f;
const SET_FEATURES: u8 = 0x1f;
const DEVICE_ID: u8 = 0x9f;
const PAGE_READ: u8 = 0x13;
const CACHE_READ: u8 = 0x03;

const FEATURE_STATUS: u8 = 0xc0;
const FEATURE_CONFIG: u8 = 0xb0;
const FEATURE_BLOCK_LOCK: u8 = 0xa0;
const FEATURE_DIE_SELECT: u8 = 0xd0;

struct FileAllocation {
    offset: u32,
    length: u32,
    fat_index: u16,
    // Do we want extra redundancy here?
}

struct FlashSpi {
    spi: Spi<Enabled, SPI1, 8>,
    cs: Pin<Gpio9, Output<PushPull>>, // We manually control the CS pin
                                      // _sck: Pin<SCK, FunctionSpi>,
                                      // _miso: Pin<MISO, FunctionSpi>,
                                      // _mosi: Pin<MOSI, FunctionSpi>,
}

struct FileAllocationTable {
    next_free_byte_offset: usize,
    next_fat_index: usize,
    //fat: [FileAllocation; 3000],
}

const FAT_VERSION: u32 = 1;
const FILE_ADDRESS_START_OFFSET: usize = 1 << 16;

struct OnboardFlashStatus {
    inner: u8,
}

struct EccStatus {
    okay: bool,
    should_relocate: bool,
}

impl OnboardFlashStatus {
    fn cache_read_busy(&self) -> bool {
        self.inner & 0b1000_0000 != 0
    }

    fn ecc_status(&self) -> EccStatus {
        // TODO: return relocation info?
        let ecc_bits = (self.inner >> 4) & 0b0000_0111;
        match ecc_bits {
            0 => EccStatus {
                okay: true,
                should_relocate: false,
            },
            1 => {
                warn!("ECC error - 1-3 bits corrected");
                EccStatus {
                    okay: true,
                    should_relocate: false,
                }
            }
            2 => {
                error!("ECC error - uncorrectable error, data corrupted!");
                EccStatus {
                    okay: false,
                    should_relocate: true,
                }
            }
            3 => {
                warn!("ECC error - 4-6 bits corrected, should re-locate data");
                EccStatus {
                    okay: true,
                    should_relocate: true,
                }
            }
            5 => {
                warn!("ECC error - 7-8 bits corrected, MUST re-locate data");
                EccStatus {
                    okay: true,
                    should_relocate: true,
                }
            }
            _ => unreachable!("Unknown ECC status"),
        }
    }

    fn program_failed(&self) -> bool {
        self.inner & 0b0000_1000 != 0
    }

    fn erase_failed(&self) -> bool {
        self.inner & 0b0000_0100 != 0
    }

    fn write_enabled(&self) -> bool {
        self.inner & 0b0000_0010 != 0
    }

    fn operation_in_progress(&self) -> bool {
        self.inner & 0b0000_0001 != 0
    }
}

struct Page {
    inner: [u8; 4 + 2048 + 128],
}

impl Page {
    fn new() -> Page {
        // NOTE: We include 4 bytes at the beginning of the buffer for the command + address
        let mut blank_page = [0xff; 4 + 2048 + 128];
        blank_page[0] = CACHE_READ;
        blank_page[1] = 0;
        blank_page[2] = 0;
        blank_page[3] = 0; // We always read from the beginning of the page cache
        Page { inner: blank_page }
    }

    fn cache_data(&self) -> &[u8] {
        &self.inner[4..]
    }
    fn user_data(&self) -> &[u8] {
        &self.cache_data()[0..2048]
    }

    fn user_metadata_1(&self) -> &[u8] {
        &self.cache_data()[0x820..=0x83f]
    }

    fn user_metadata_2(&self) -> &[u8] {
        &self.cache_data()[0x804..=0x81f]
    }

    fn bad_block_data(&self) -> &[u8] {
        &self.cache_data()[0x800..=0x803]
    }

    fn is_part_of_bad_block(&self) -> bool {
        self.bad_block_data().iter().find(|&x| *x == 0).is_some()
    }

    // User metadata 1 contains 211 bytes
    // [0] = set to zero if page is used.
    // [1] = set to zero if this is the *last* page for a file.
    // [2, 3] = length of page used as little-endian u16
    fn page_is_used(&self) -> bool {
        self.user_metadata_1()[0] == 0
    }

    fn is_last_page_for_file(&self) -> bool {
        self.user_metadata_1()[1] == 0
    }

    fn page_bytes_used(&self) -> usize {
        LittleEndian::read_u16(&self.user_metadata_1()[2..=3]) as usize
    }

    fn cache_data_mut(&mut self) -> &mut [u8] {
        &mut self.inner[4..]
    }

    fn page_mut(&mut self) -> &mut [u8] {
        &mut self.inner
    }
}

pub struct OnboardFlash {
    spi: FlashSpi, //, SCK, MISO, MOSI
    fat: FileAllocationTable,
    current_file_block_index: Option<i16>,
    current_page_index: usize,
    current_block_index: usize,
    next_free_good_block_index: Option<u16>,
    next_used_block_index: Option<u16>,
    bad_blocks: [i16; 40],
    current_page: Page,
}

/// Each block is made up 64 pages of 2176 bytes. 139,264 bytes per block.
/// Each page has a 2048 byte data storage section and a 128byte spare area for ECC codes.
/// We should check for ECC errors after read/writes ourselves
/// There are 2048 blocks on this device for a total of 256MB

impl OnboardFlash {
    pub fn init(spi: Spi<Enabled, SPI1, 8>, cs: Pin<Gpio9, Output<PushPull>>) -> OnboardFlash {
        // Init the spi peripheral and either init the FAT, or
        info!("Initing onboard flash");

        // NOTE: We don't try to use all the flash memory efficiently.  Files always start at the
        //  beginning of a block, and if when they end part way through a block, we don't use that
        //  block for anything else.  Each page of each block has user metadata saying how many bytes
        //  of the page were used.  If it's less than the page length, we know that this was the last
        //  page of the file.  We could also write the *last* page of the block with a value
        //  saying how many pages of the block were used.

        let mut flash_storage = OnboardFlash {
            spi: FlashSpi { spi, cs },
            fat: FileAllocationTable {
                next_free_byte_offset: FILE_ADDRESS_START_OFFSET,
                next_fat_index: 0,
            },
            next_free_good_block_index: None,
            current_file_block_index: None,
            next_used_block_index: None,
            bad_blocks: [i16::MAX; 40],
            current_page_index: 0,
            current_block_index: 0,
            current_page: Page::new(),
        };

        flash_storage.wait_for_ready();
        //flash_storage.read_page_from_cache();

        // info!(
        //     "Current page is okay? {}",
        //     !flash_storage.current_page.is_part_of_bad_block()
        // );

        // There are guaranteed to be no more than 40 bad blocks for the lifetime of the device,
        // according to the micron datasheet.
        let mut bad_blocks = [i16::MAX; 40];

        // Find first good free block:
        // flash_storage.read_page_into_cache(0, 0);
        for block_index in 0..2048 {
            // TODO: Interleave with random cache read

            // TODO: We can see if this is faster if we just read the column index of the end of the page?
            // For simplicity at the moment, just read the full pages
            flash_storage.read_page_into_cache(block_index, 0).unwrap();
            flash_storage.read_page_from_cache();
            if flash_storage.current_page.is_part_of_bad_block() {
                if let Some(slot) = bad_blocks.iter_mut().find(|x| **x == i16::MAX) {
                    // Add the bad block to our runtime table.
                    *slot = block_index as i16;
                }
            } else {
                if !flash_storage.current_page.page_is_used() {
                    if flash_storage.next_free_good_block_index.is_none() {
                        // This will be the starting block of the next file to be written.
                        flash_storage.next_free_good_block_index = Some(block_index as u16);
                    }
                } else {
                    if flash_storage.next_used_block_index.is_none() {
                        // This is the starting block of the first file stored.
                        flash_storage.next_used_block_index = Some(block_index as u16);
                    }
                    // Starting new file?

                    // We don't really need to keep records of existing files on startup, we just
                    // need to transfer them one by one.  We won't remove any files until all
                    // the files have been transferred.
                }
            }
        }
        flash_storage.bad_blocks = bad_blocks;

        info!(
            "First free good block index {:?}",
            flash_storage.next_free_good_block_index
        );
        info!(
            "First file start block index {:?}",
            flash_storage.next_used_block_index
        );

        // If there are *any* files on the flash when we're initialising, we need to wake up the pi
        // or wait for the pi to be available and offload them.  We then erase blocks up til the
        // first free good block.  Maybe discovering the first free good block can be part of the
        // process of scanning and uploading files.

        // block 0 and block 1 will be double-buffered FAT if both are good, otherwise
        // it's just the first two good blocks.

        // Writing a file:
        // Lookup first free page, and start writing out bytes.
        // 128KB per block
        // 2 x 128KB?
        // Writing copies of file tables sequentially, just need a way of

        // Maybe when the device powers on, we just scan to find the first free (good) block,
        // rather than keeping a table that needs updating?  We then just start writing out to that
        // block.  Reserve the last page of the block for info about if the block is the last block
        // for the file, and record the length of the file.  When reading out files, we'd just scan over
        // all the blocks linearly, until we find a good block that's empty.

        // When writing, in the event that a block has gone bad, we need to move the contents from
        // the block to the next good block, then mark the block as bad, erasing it first.

        // In a file, we write the contents of the file out, then the header at the end as a separate
        // gzip member.  We reassemble into the correct order when loading back to the pi.

        // READ the FAT at the beginning by loading all 64 pages of block 1

        // for block in 0..2048 {
        //     flash_storage.read_page_into_cache(block, 0);
        //     flash_storage.read_page_from_cache();
        //     if flash_storage.current_page.is_part_of_bad_block() {
        //         warn!(
        //             "Found bad block {}, {:?}, {:?}, {:?}",
        //             block,
        //             flash_storage.current_page.inner[0..20],
        //             flash_storage.current_page.cache_data()[0..20],
        //             flash_storage.current_page.bad_block_data(),
        //         );
        //     }
        // }
        // Actually, shouldn't need to call reset by default on power up according to pg.50 of
        // the datasheet
        // flash_storage.reset();
        // After reset, the first page of the first block is automatically loaded into the cache
        // register.

        // First check if the first page is a good page by reading the extra metadata first byte to see if it's 0x00
        // If not, load pages sequentially until we find the first good byte.

        // Now we need to "format" the flash, scanning for any bad blocks and mapping around them somehow.
        // What should this data structure look like?

        // Read out the FAT info:
        //core::mem::size_of::<FileAllocationTable>()

        flash_storage
    }

    pub fn has_files_to_offload(&self) -> bool {
        // When we did our initial scan, did we encounter any used blocks?
        self.next_used_block_index.is_some()
    }

    fn advance_file_cursor(&mut self) {
        // If current_page < 64, increment page
        if self.current_page_index < 63 {
            self.current_page_index += 1;
        } else {
            let mut next_block_index = (self.current_block_index + 1) as i16;
            while self.bad_blocks.contains(&next_block_index) {
                next_block_index += 1;
            }
            self.current_block_index = next_block_index as usize;
            self.current_page_index = 0;
        }
        // info!(
        //     "File cursor ({}, {})",
        //     self.current_block_index, self.current_page_index
        // );
    }

    fn advance_to_next_file(&mut self) -> usize {
        let next_block_index = self.current_file_block_index.get_or_insert(-1);
        info!("Start block index {}", next_block_index);
        info!("Bad blocks: {:?}", self.bad_blocks);
        while self.bad_blocks.contains(&*next_block_index) {
            info!("Increment next_block inside bad block check");
            *next_block_index += 1;
        }
        *next_block_index += 1;

        info!(
            "At file block {}, page {}",
            next_block_index, self.current_page_index
        );
        *next_block_index as usize
    }

    pub fn erase_all_good_used_blocks(&mut self) {
        for block_index in 0..2048 {
            while self.bad_blocks.contains(&block_index) {
                continue;
            }
            self.read_page_into_cache(self.current_block_index, 0)
                .unwrap();
            self.read_page_from_cache();
            if self.current_page.page_is_used() {
                info!("Erasing used block {}", block_index);
                self.erase_block(block_index as usize);
            } else {
                // If we encounter an unused first page of a block, that means we've gone past the
                // end of used storage, and blocks should already be in an erased state.
                break;
            }
        }
    }

    fn write_enable(&mut self) {
        self.spi_write(&[WRITE_ENABLE]);
    }
    fn erase_block(&mut self, block_index: usize) {
        self.write_enable();
        let address = OnboardFlash::get_address(block_index, 0);
        self.spi_write(&[BLOCK_ERASE, address[0], address[1], address[2]]);
        self.wait_for_ready();
    }

    pub fn get_file_part(&mut self) -> Option<(&[u8], bool)> {
        // if let Some(next_used_block) = self.next_used_block_index {
        // TODO: Could interleave using cache_random_read
        if self
            .read_page_into_cache(self.current_block_index, self.current_page_index)
            .is_ok()
        {
            self.read_page_from_cache();
            let length = self.current_page.page_bytes_used();
            let is_last_page_for_file = self.current_page.is_last_page_for_file();

            self.advance_file_cursor();

            Some((
                &self.current_page.user_data()[0..length],
                is_last_page_for_file,
            ))
        } else {
            None
        }
    }

    fn spi_transfer(&mut self, bytes: &mut [u8]) {
        self.spi.cs.set_low().unwrap();
        self.spi.spi.transfer(bytes).unwrap();
        self.spi.cs.set_high().unwrap();
    }

    // Not sure if we want this, probably we'll just read full pages+blocks
    fn read_bytes(&mut self, start_offset: usize, length: usize, target: &mut [u8]) {
        // There are 131,072 pages total, where does our start and end address fall within them?
        // We need to generate a series of block, page addresses and read each page in sequence into
        // the cache, then read that out via SPI.  Is it possible to interleave these things?

        // We need to create 17bit block/page addresses as 3 bytes/24bits

        // We are guaranteed to have 251MB of valid blocks in our flash memory.
        // We might need to scan for all bad blocks on initialisation the first time,
        // and create a bad-block table in the first good block.  We need to scan for the
        // first good block each time we switch on the device I guess?

        // Looks like the read page cache random command is used for this.
        let global_page_index = start_offset / 64; // page 3,595,673
        let block_index = global_page_index / 2048; //
                                                    // 230_123_123 - block 1755,
                                                    // 230123123 % 2048 = 1561

        // Calculate where in the device the start and end offset are, how many pages we need to read
        // into cache etc.  We'll want to do chained DMA transfers where we can, and in the case
        // of transferring files to the rpi, we want it fully handed by DMA where possible.

        // Is the address within the current page?
        // If so, we don't have to read the page into the cache again.

        // Calculate the address to read from

        // Read page to cache
        self.spi_write(&[PAGE_READ, 0x00, 0x00, 0x00]);

        self.spi.cs.set_low().unwrap();
        self.spi.spi.transfer(target).unwrap();
        self.spi.cs.set_high().unwrap();
    }

    fn get_status(&mut self) -> OnboardFlashStatus {
        let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
        self.spi_transfer(&mut features);
        OnboardFlashStatus { inner: features[2] }
    }

    fn wait_for_ready(&mut self) {
        while self.get_status().operation_in_progress() {
            nop()
        }
    }

    fn read_page(&mut self, block: usize, page: usize) {
        // TODO?
    }

    fn get_address(block: usize, page: usize) -> [u8; 3] {
        // 11 bits for block, 6 bits for page
        let address: u32 = (block as u32) << 6 | page as u32;
        let mut buf = [0u8; 4];
        BigEndian::write_u32(&mut buf, address);
        [buf[1], buf[2], buf[3]]
    }
    fn read_page_into_cache(&mut self, block: usize, page: usize) -> Result<(), &str> {
        assert!(block < 2048, "Invalid block");
        assert!(page < 64, "Invalid page");

        let address = OnboardFlash::get_address(block, page);
        self.spi_write(&[PAGE_READ, address[0], address[1], address[2]]);
        self.wait_for_ready();

        // To keep things simple, we'll never do relocations if we get ECC errors during reading
        // – only during writing, when we can just advance to the next good block, marking the
        // current block as bad.

        // TODO: Now we need to check the ECC status bits, and return okay if it is ok
        let EccStatus {
            okay,
            should_relocate,
        } = self.get_status().ecc_status();
        if okay {
            Ok(())
        } else {
            // Unrecoverable failure.  Maybe just mark this block as bad, and mark all the blocks
            // that the file spans as temporarily corrupt, so this file doesn't get read and
            // send to the raspberry pi
            Err(&"unrecoverable data corruption error")
        }
    }
    fn read_page_from_cache(&mut self) {
        let bytes = self.current_page.page_mut();
        bytes[0] = CACHE_READ;
        bytes[1] = 0;
        bytes[2] = 0;
        bytes[3] = 0;
        self.spi.cs.set_low().unwrap();
        self.spi.spi.transfer(bytes).unwrap();
        self.spi.cs.set_high().unwrap();
    }
    fn spi_write(&mut self, bytes: &[u8]) {
        self.spi.cs.set_low().unwrap();
        self.spi.spi.write(bytes).unwrap();
        self.spi.cs.set_high().unwrap();
    }

    fn print_feature(&mut self, name: &str, feature: u8) {
        let mut features: [u8; 3] = [GET_FEATURES, feature, 0x00];
        self.spi_transfer(&mut features);
        info!("Feature {}: {:08b}", name, features[2]);
    }

    // pub fn reset(&mut self, delay: &mut Delay) {
    //     info!("Resetting onboard flash");
    //     self.spi_write(&[RESET]);
    //     // After issuing reset, must wait at least 1.25ms
    //     delay.delay_us(1500);
    //     self.spi_wait_for_ready();
    //     info!("Onboard flash ready");
    // }

    fn spi_write_enabled(&mut self) -> bool {
        let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
        self.spi_transfer(&mut features);
        features[2] & 0b0000_0010 != 0
    }

    fn get_device_id(&mut self) {
        let mut id: [u8; 4] = [DEVICE_ID, 0x00, 0x00, 0x00];
        self.spi_transfer(&mut id);
        info!("Device id {:x}", id);
    }

    fn check_occupancy() {
        // TODO: Check if we're getting dangerously full, and if so, wake up the pi to offload files.
    }

    pub fn start_file(&mut self) -> usize {
        // We always start writing a new file at page 1, reserving page 0 for when we come back
        // and write the header once we've finished writing the file.
        self.current_page_index = 1;
        self.advance_to_next_file()
    }

    pub fn append_file_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: Option<usize>,
        page_index: Option<usize>,
    ) {
        // self.write_enable();
        //
        // // Bytes will always be a full page + metadata + command info at the start
        // assert_eq!(bytes.len(), 2176 + 4);
        // if !is_last {
        //     assert_eq!(user_bytes_length, 2048);
        // }
        // bytes[0] = PROGRAM_LOAD;
        // let address = OnboardFlash::get_address(
        //     block_index.unwrap_or(self.current_block_index),
        //     page_index.unwrap_or(self.current_page_index),
        // );
        // if block_index.is_none() && page_index.is_none() {
        //     self.advance_file_cursor();
        // }
        // bytes[1] = address[0];
        // bytes[2] = address[1];
        // bytes[3] = address[2];
        //
        // // Now write into the user meta section
        // bytes[4..][0x820..=0x83f][0] = 0; // Page is used
        // bytes[4..][0x820..=0x83f][1] = if is_last { 0 } else { 0xff }; // Page is last page of file?
        // LittleEndian::write_u16(
        //     &mut bytes[4..][0x820..=0x83f][2..=3],
        //     user_bytes_length as u16,
        // );
        //
        // self.spi_write(&bytes);
        // self.spi_write(&[PROGRAM_EXECUTE]);
        // self.wait_for_ready();
        //
        // // TODO: Check ECC status, mark and relocate block if needed.
        // let status = self.get_status().ecc_status();
        // info!("ECC status {}", status.okay);
    }

    pub fn write_data_to_current_file(&mut self, data: &[u8]) {
        // Write some data at the current next_free_byte_offset, and increment
        // TODO: Can we use DMA transfers, and just block on the transfer when all other work is done?
        // the next_free_byte_offset.
    }

    // pub fn write_data_to_current_page(&mut self, data: &[u8]) {
    //     // Write some data at the current next_free_byte_offset, and increment
    //     // TODO: Can we use DMA transfers, and just block on the transfer when all other work is done?
    //     // the next_free_byte_offset.
    // }

    // fn write_data_in_place_to_current_file(&mut self, offset_in_file: usize, data: &[u8]) {
    //     // Write some data at the current file_start + offset_in_file.
    //     // If it overflows next_free_byte_offset, increment that.
    // }
}

/*
pub fn setup() {
    // TODO: Make this work via PIO SPI?
    // The only time we need two SPIs is when we're transferring stuff from flash to rPi.

    // Reset
    spi_write(spi, cs, &[RESET]);
    wait_for_ready(spi, cs);

    // Write enable
    spi_write(spi, cs, &[WRITE_ENABLE]);
    wait_for_ready(spi, cs);
    print_feature(spi, cs, "status", FEATURE_STATUS);
    print_feature(spi, cs, "block lock", FEATURE_BLOCK_LOCK);
    spi_write(spi, cs, &[SET_FEATURES, FEATURE_BLOCK_LOCK, 0b0000_0000]);
    print_feature(spi, cs, "block lock", FEATURE_BLOCK_LOCK);
    //if write_enabled(spi, cs) {
    // Let's write something.
    //println!("Write enabled");
    // spi_write(spi, cs, &[WRITE_ENABLE]);
    // spi_write(spi, cs, &[PROGRAM_LOAD, 0x00, 0x00, b'H', b'e', b'l', b'l', b'o']);
    // spi_write(spi, cs, &[PROGRAM_EXECUTE, 0x00, 0x00, 0x00]);
    //}
    wait_for_ready(spi, cs);
    print_feature(spi, cs, "status", FEATURE_STATUS);
    // Try to read out data
    spi_write(spi, cs, &[PAGE_READ, 0x00, 0x00, 0x00]);
    wait_for_ready(spi, cs);
    print_feature(spi, cs, "block lock", FEATURE_BLOCK_LOCK);
    let mut data: [u8; 9] = [CACHE_READ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    spi_transfer(spi, cs, &mut data);
    print_feature(spi, cs, "status", FEATURE_STATUS);
    info!("data read {:x}", data);

    //println!("Status {:?}", features);
    // Write enable
    //spi.write(&[0x06]).unwrap();
    let mut id: [u8; 4] = [DEVICE_ID, 0x00, 0x00, 0x00];
    spi_transfer(spi, cs, &mut id);
    info!("id {:x}", id);
}


fn write_enabled(spi: &mut Spi<Enabled, SPI1, 8>, cs: &mut Pin<Gpio9, PushPullOutput>) -> bool {
    let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
    spi_transfer(spi, cs, &mut features);
    features[2] & 0b0000_0010 != 0
}

fn spi_write(spi: &mut Spi<Enabled, SPI1, 8>, cs: &mut Pin<Gpio9, PushPullOutput>, bytes: &[u8]) {
    cs.set_low().unwrap();
    spi.write(bytes).unwrap();
    cs.set_high().unwrap();
}
fn spi_transfer(
    spi: &mut Spi<Enabled, SPI1, 8>,
    cs: &mut Pin<Gpio9, PushPullOutput>,
    bytes: &mut [u8],
) {
    cs.set_low().unwrap();
    spi.transfer(bytes).unwrap();
    cs.set_high().unwrap();
}

fn wait_for_ready(spi: &mut Spi<Enabled, SPI1, 8>, cs: &mut Pin<Gpio9, PushPullOutput>) {
    loop {
        let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
        spi_transfer(spi, cs, &mut features);
        if features[2] & 0b0000_0001 == 0 {
            break;
        }
    }
}

fn print_feature(
    spi: &mut Spi<Enabled, SPI1, 8>,
    cs: &mut Pin<Gpio9, PushPullOutput>,
    name: &str,
    feature: u8,
) {
    let mut features: [u8; 3] = [GET_FEATURES, feature, 0x00];
    spi_transfer(spi, cs, &mut features);
    info!("Feature {}: {:08b}", name, features[2]);
}
*/
