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
use byteorder::{ByteOrder, LittleEndian};
use defmt::{error, info, println, warn};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
};
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio8, Gpio9};
use rp2040_hal::gpio::{
    FunctionNull, FunctionSio, FunctionSpi, Pin, PullDown, PullNone, SioOutput,
};
use rp2040_hal::pac::RESETS;
use rp2040_hal::spi::Enabled;
use rp2040_hal::Spi;

const WRITE_ENABLE: u8 = 0x06;

const BLOCK_ERASE: u8 = 0xd8;
pub const PROGRAM_LOAD: u8 = 0x02;
pub const PROGRAM_EXECUTE: u8 = 0x10;
const RESET: u8 = 0xff;
const GET_FEATURES: u8 = 0x0f;
const SET_FEATURES: u8 = 0x1f;
const DEVICE_ID: u8 = 0x9f;
pub const PAGE_READ: u8 = 0x13;
const PAGE_READ_RANDOM: u8 = 0x30;
const PAGE_READ_LAST: u8 = 0x3f;
const CACHE_READ: u8 = 0x0b;

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

// struct FlashSpi {
//     pub spi: Spi<Enabled, SPI1, 8>,
//     pub cs: Pin<Gpio9, FunctionSio<SioOutput>, PullDown>, // We manually control the CS pin
//                                                 // _sck: Pin<SCK, FunctionSpi>,
//                                                 // _miso: Pin<MISO, FunctionSpi>,
//                                                 // _mosi: Pin<MOSI, FunctionSpi>,
// }

struct FileAllocationTable {
    next_free_byte_offset: usize,
    next_fat_index: usize,
    //fat: [FileAllocation; 3000],
}

const FAT_VERSION: u32 = 1;
const FILE_ADDRESS_START_OFFSET: usize = 1 << 16;

pub struct OnboardFlashStatus {
    inner: u8,
}

pub struct EccStatus {
    pub okay: bool,
    should_relocate: bool,
}

impl OnboardFlashStatus {
    pub fn cache_read_busy(&self) -> bool {
        self.inner & 0b1000_0000 != 0
    }

    pub fn ecc_status(&self) -> EccStatus {
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
                //error!("ECC error - uncorrectable error, data corrupted!");
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

    pub fn operation_in_progress(&self) -> bool {
        self.inner & 0b0000_0001 != 0
    }
}

pub struct Page {
    inner: [u8; 4 + 2048 + 128],
}

impl Page {
    fn new() -> Page {
        // NOTE: We include 3 bytes at the beginning of the buffer for the command + column address
        let mut blank_page = [0xff; 4 + 2048 + 128];
        blank_page[0] = CACHE_READ;
        blank_page[1] = 0;
        blank_page[2] = 0;
        blank_page[3] = 0;
        Page { inner: blank_page }
    }

    fn cache_data(&self) -> &[u8] {
        &self.inner[4..]
    }
    pub fn user_data(&self) -> &[u8] {
        &self.cache_data()[0..2048]
    }

    pub fn user_metadata_1(&self) -> &[u8] {
        &self.cache_data()[0x820..=0x83f]
    }

    pub fn block_index(&self) -> u8 {
        self.user_metadata_1()[4]
    }
    pub fn page_index(&self) -> u8 {
        self.user_metadata_1()[5]
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
    pub fn page_is_used(&self) -> bool {
        self.user_metadata_1()[0] == 0
    }

    fn is_last_page_for_file(&self) -> bool {
        self.user_metadata_1()[1] == 0
    }

    pub fn page_bytes_used(&self) -> usize {
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
    spi: Option<
        Spi<
            Enabled,
            SPI1,
            (
                Pin<Gpio11, FunctionSpi, PullDown>,
                Pin<Gpio8, FunctionSpi, PullDown>,
                Pin<Gpio10, FunctionSpi, PullDown>,
            ),
            8,
        >,
    >, //, SCK, MISO, MOSI
    cs: Pin<Gpio9, FunctionSio<SioOutput>, PullDown>,
    mosi_disabled: Option<Pin<Gpio11, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio10, FunctionNull, PullNone>>,
    miso_disabled: Option<Pin<Gpio8, FunctionNull, PullNone>>,
    pub current_page_index: isize,
    pub current_block_index: isize,
    pub last_used_block_index: Option<isize>,
    pub first_used_block_index: Option<isize>,
    bad_blocks: [i16; 40],
    pub current_page: Page,
}

/// Each block is made up 64 pages of 2176 bytes. 139,264 bytes per block.
/// Each page has a 2048 byte data storage section and a 128byte spare area for ECC codes.
/// We should check for ECC errors after read/writes ourselves
/// There are 2048 blocks on this device for a total of 256MB

impl OnboardFlash {
    pub fn new(
        cs: Pin<Gpio9, FunctionSio<SioOutput>, PullDown>,
        mosi: Pin<Gpio11, FunctionNull, PullNone>,
        clk: Pin<Gpio10, FunctionNull, PullNone>,
        miso: Pin<Gpio8, FunctionNull, PullNone>,
    ) -> OnboardFlash {
        OnboardFlash {
            cs,
            mosi_disabled: Some(mosi),
            clk_disabled: Some(clk),
            miso_disabled: Some(miso),
            spi: None,
            first_used_block_index: None,
            last_used_block_index: None,
            bad_blocks: [i16::MAX; 40],
            current_page_index: 0,
            current_block_index: 0,
            current_page: Page::new(),
        }
    }
    pub fn init(&mut self) {
        // Init the spi peripheral and either init the FAT, or
        info!("Initing onboard flash");

        // NOTE: We don't try to use all the flash memory efficiently.  Files always start at the
        //  beginning of a block, and if when they end part way through a block, we don't use that
        //  block for anything else.  Each page of each block has user metadata saying how many bytes
        //  of the page were used.  If it's less than the page length, we know that this was the last
        //  page of the file.  We could also write the *last* page of the block with a value
        //  saying how many pages of the block were used.

        self.reset();
        self.scan();
        self.unlock_blocks();
        // if erase {
        // info!("Erasing");
        //
        // for block in 0..2048 {
        //     if !self.erase_block(block).is_ok() {
        //         error!("Block erase failed for block {}", block);
        //     }
        // }
        // self.scan();
        // // }
        // crate::unreachable!("Foo");
    }

    pub fn reset(&mut self) {
        self.spi_write(&[RESET]);
        self.wait_for_ready();
    }
    pub fn unlock_blocks(&mut self) {
        self.spi_write(&[SET_FEATURES, FEATURE_BLOCK_LOCK, 0x00]);
        self.wait_for_ready();
    }
    pub fn scan(&mut self) {
        let mut bad_blocks = [i16::MAX; 40];
        self.first_used_block_index = None;
        self.last_used_block_index = None;
        self.current_page_index = 0;
        self.current_block_index = 0;
        // Find first good free block:
        {
            self.read_page(0, 1).unwrap();
            self.read_page_from_cache(0);
            self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                warn!("Page 1 has data");
            }
        }

        for block_index in 0..2048isize {
            // TODO: Interleave with random cache read

            // TODO: We can see if this is faster if we just read the column index of the end of the page?
            // For simplicity at the moment, just read the full pages
            self.read_page(block_index, 0).unwrap();
            self.read_page_from_cache(block_index);
            self.wait_for_all_ready();
            if self.current_page.is_part_of_bad_block() {
                warn!("Found bad block {}", block_index);
                if let Some(slot) = bad_blocks.iter_mut().find(|x| **x == i16::MAX) {
                    // Add the bad block to our runtime table.
                    *slot = block_index as i16;
                }
            } else {
                if !self.current_page.page_is_used() {
                    // if self.first_used_block_index.is_none() {
                    //     println!("Meta {:?}", self.current_page.user_metadata_1()[0..10]);
                    // }
                    // This will be the starting block of the next file to be written.
                    if self.last_used_block_index.is_none() && self.first_used_block_index.is_some()
                    {
                        self.last_used_block_index = Some(block_index - 1);
                        self.current_block_index = block_index;
                        self.current_page_index = 0;
                        println!("Setting next starting block index {}", block_index);
                    }
                } else {
                    // println!(
                    //     "Scan found used page on block {}: {}, {:?}",
                    //     block_index,
                    //     self.current_page.page_is_used(),
                    //     self.current_page.user_metadata_1()[0..10]
                    // );
                    let address = OnboardFlash::get_address(block_index, 0);
                    // println!("Block address {:?}", address);
                    if self.first_used_block_index.is_none() {
                        // This is the starting block of the first file stored.
                        println!("Storing first used block {}", block_index);
                        self.first_used_block_index = Some(block_index);
                    }
                    // Starting new file?

                    // We don't really need to keep records of existing files on startup, we just
                    // need to transfer them one by one.  We won't remove any files until all
                    // the files have been transferred.
                }
            }
        }
        self.bad_blocks = bad_blocks;
    }

    pub fn take_spi(&mut self, peripheral: SPI1, resets: &mut RESETS, freq: HertzU32) {
        let miso = self.miso_disabled.take().unwrap();
        let mosi = self.mosi_disabled.take().unwrap();
        let clk = self.clk_disabled.take().unwrap();
        let spi = Spi::new(
            peripheral,
            (
                mosi.into_function().into_pull_type(),
                miso.into_function().into_pull_type(),
                clk.into_function().into_pull_type(),
            ),
        )
        .init(
            resets,
            freq, // 125_000_000
            40_000_000.Hz(),
            &embedded_hal::spi::MODE_3,
        );
        self.spi = Some(spi);
    }

    pub fn has_files_to_offload(&self) -> bool {
        // When we did our initial scan, did we encounter any used blocks?
        info!(
            "First used block {:?}, last used {:?}",
            self.first_used_block_index, self.last_used_block_index
        );
        self.first_used_block_index.is_some()
    }

    fn advance_file_cursor(&mut self, is_last: bool) {
        // If current_page < 64, increment page
        if self.current_page_index < 63 && !is_last {
            self.current_page_index += 1;
        } else {
            let mut next_block_index = self.current_block_index + 1;
            while self.bad_blocks.contains(&(next_block_index as i16)) {
                info!("Skipping bad block {}", next_block_index);
                next_block_index += 1;
            }
            self.current_block_index = next_block_index;
            self.current_page_index = 0;
        }
        // println!(
        //     "Advanced cursor {}:{}",
        //     self.current_block_index, self.current_page_index
        // );
    }

    // fn advance_to_next_file(&mut self) -> usize {
    //     println!("Advance from {:?}", self.current_block_index);
    //     let next_block_index = self.current_block_index.get_or_insert(-1);
    //     println!("Start block index {}", next_block_index);
    //     println!("Bad blocks: {:?}", self.bad_blocks);
    //     while self.bad_blocks.contains(&(*next_block_index as i16)) {
    //         println!("Increment next_block inside bad block check");
    //         *next_block_index += 1;
    //     }
    //     *next_block_index += 1;
    //
    //     println!(
    //         "At file block {}, page {}",
    //         next_block_index, self.current_page_index
    //     );
    //     *next_block_index as usize
    // }

    pub fn erase_all_good_used_blocks(&mut self) {
        for block_index in 0..2048isize {
            while self.bad_blocks.contains(&(block_index as i16)) {
                info!("Skipping erase of bad block {}", block_index);
                continue;
            }
            self.read_page(block_index, 0).unwrap();
            self.read_page_from_cache(block_index);
            if self.current_page.page_is_used() {
                //println!("Erasing used block {}", block_index);
                self.erase_block(block_index).unwrap();
            } else {
                // If we encounter an unused first page of a block, that means we've gone past the
                // end of used storage, and blocks should already be in an erased state.
                break;
            }
        }
        self.first_used_block_index = None;
        self.last_used_block_index = None;
        self.current_page_index = 0;
        self.current_block_index = 0;
    }
    pub fn write_enable(&mut self) {
        self.spi_write(&[WRITE_ENABLE]);
    }
    pub fn erase_block(&mut self, block_index: isize) -> Result<(), &str> {
        self.write_enable();
        let address = OnboardFlash::get_address(block_index, 0);
        self.spi_write(&[BLOCK_ERASE, address[0], address[1], address[2]]);
        let status = self.wait_for_ready();
        if status.erase_failed() {
            Err(&"Block erase failed")
        } else {
            Ok(())
        }
    }

    pub fn free_spi(&mut self) -> SPI1 {
        let spi_enabled = self.spi.take().unwrap();
        let spi_disabled = spi_enabled.disable();
        let (spi, (mosi, miso, clk)) = spi_disabled.free();
        self.mosi_disabled = Some(mosi.into_pull_down_disabled().into_pull_type());
        self.clk_disabled = Some(clk.into_pull_down_disabled().into_pull_type());
        self.miso_disabled = Some(miso.into_pull_down_disabled().into_pull_type());

        spi
    }
    pub fn get_file_part(&mut self) -> Option<(&[u8], bool, SPI1)> {
        // TODO: Could interleave using cache_random_read
        if self
            .read_page(self.current_block_index, self.current_page_index)
            .is_ok()
        {
            self.read_page_from_cache(self.current_block_index);
            //self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                let length = self.current_page.page_bytes_used();
                let is_last_page_for_file = self.current_page.is_last_page_for_file();
                // info!(
                //     "Get file part {:?}, {:?}, at {}:{}, length {}, was last {}",
                //     &self.current_page.user_data()[0..10],
                //     &self.current_page.user_metadata_1()[0..10],
                //     self.current_block_index,
                //     self.current_page_index,
                //     length,
                //     is_last_page_for_file
                // );
                self.advance_file_cursor(is_last_page_for_file);
                let spi = self.free_spi();
                Some((
                    &self.current_page.user_data()[0..length],
                    is_last_page_for_file,
                    spi,
                ))
            } else {
                // warn!(
                //     "Unoccupied {}:{}",
                //     self.current_block_index, self.current_page_index
                // );
                // info!(
                //     "Get file part {:?}, {:?}, at {}:{}",
                //     &self.current_page.user_data()[0..10],
                //     &self.current_page.user_metadata_1()[0..10],
                //     self.current_block_index,
                //     self.current_page_index,
                // );

                None
            }
        } else {
            None
        }
    }

    fn spi_transfer(&mut self, bytes: &mut [u8]) {
        self.cs.set_low().unwrap();
        self.spi.as_mut().unwrap().transfer(bytes).unwrap();
        self.cs.set_high().unwrap();
    }

    pub fn get_status(&mut self) -> OnboardFlashStatus {
        let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
        self.spi_transfer(&mut features);
        OnboardFlashStatus { inner: features[2] }
    }

    pub fn wait_for_ready(&mut self) -> OnboardFlashStatus {
        let mut status = self.get_status();
        while status.operation_in_progress() {
            status = self.get_status();
        }
        status
    }

    pub fn wait_for_all_ready(&mut self) -> OnboardFlashStatus {
        let mut status = self.get_status();
        while status.operation_in_progress() || status.cache_read_busy() {
            status = self.get_status();
        }
        status
    }

    pub fn wait_for_cache_ready(&mut self) -> OnboardFlashStatus {
        let mut status = self.get_status();
        while status.cache_read_busy() {
            status = self.get_status();
        }
        status
    }

    pub fn get_address(block: isize, page: isize) -> [u8; 3] {
        // 11 bits for block, 6 bits for page
        let address: u32 = (block as u32) << 6 | page as u32;
        [(address >> 16) as u8, (address >> 8) as u8, (address) as u8]
    }
    pub fn read_page(&mut self, block: isize, page: isize) -> Result<(), &str> {
        assert!(block < 2048, "Invalid block");
        assert!(page < 64, "Invalid page");
        let address = OnboardFlash::get_address(block, page);
        self.spi_write(&[PAGE_READ, address[0], address[1], address[2]]);
        let status = self.wait_for_all_ready();

        // TODO: Check ECC status, mark and relocate block if needed.

        // To keep things simple, we'll never do relocations if we get ECC errors during reading
        // – only during writing, when we can just advance to the next good block, marking the
        // current block as bad.

        // TODO: Now we need to check the ECC status bits, and return okay if it is ok
        //info!("Status after read into cache {:#010b}", status.inner);
        let EccStatus {
            okay,
            should_relocate,
        } = status.ecc_status();
        if okay {
            Ok(())
        } else {
            // Unrecoverable failure.  Maybe just mark this block as bad, and mark all the blocks
            // that the file spans as temporarily corrupt, so this file doesn't get read and
            // send to the raspberry pi
            //Err(&"unrecoverable data corruption error")
            warn!("unrecoverable data corruption error at {}:{}", block, page);
            Ok(())
        }
    }

    pub fn read_page_cache_random(&mut self, block: isize, page: isize) -> Result<(), &str> {
        assert!(block < 2048, "Invalid block");
        assert!(page < 64, "Invalid page");
        let address = OnboardFlash::get_address(block, page);
        self.spi_write(&[PAGE_READ_RANDOM, address[0], address[1], address[2]]);
        self.wait_for_ready();
        Ok(())
    }

    pub fn read_page_last(&mut self) -> Result<(), &str> {
        self.spi_write(&[PAGE_READ_LAST]);
        self.wait_for_ready();
        Ok(())
    }

    pub fn begin_offload(&mut self) {
        if let Some(block_index) = self.first_used_block_index {
            self.current_block_index = block_index;
            self.current_page_index = 0;
        }
    }

    pub fn read_page_from_cache(&mut self, block: isize) {
        let plane = ((block % 2) << 4) as u8;
        let bytes = self.current_page.page_mut();
        bytes[0] = CACHE_READ;
        bytes[1] = plane;
        bytes[2] = 0;
        bytes[3] = 0;
        //info!("Reading block {}, plane {}", block, plane);
        self.cs.set_low().unwrap();
        self.spi.as_mut().unwrap().transfer(bytes).unwrap();
        self.cs.set_high().unwrap();
    }
    pub fn spi_write(&mut self, bytes: &[u8]) {
        self.cs.set_low().unwrap();
        self.spi.as_mut().unwrap().write(bytes).unwrap();
        self.cs.set_high().unwrap();
    }

    fn print_feature(&mut self, name: &str, feature: u8) {
        let mut features: [u8; 3] = [GET_FEATURES, feature, 0x00];
        self.spi_transfer(&mut features);
        info!("Feature {}: {:08b}", name, features[2]);
    }

    fn write_enabled(&mut self) -> bool {
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

    pub fn start_file(&mut self) -> isize {
        // We always start writing a new file at page 1, reserving page 0 for when we come back
        // and write the header once we've finished writing the file.
        self.current_page_index = 1;
        warn!(
            "Starting file at file block {}, page {}",
            self.current_block_index, self.current_page_index
        );
        self.current_block_index
    }

    pub fn append_file_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: Option<isize>,
        page_index: Option<isize>,
    ) {
        self.write_enable();
        assert_eq!(self.write_enabled(), true);
        // Bytes will always be a full page + metadata + command info at the start
        assert_eq!(bytes.len(), 2112 + 4);
        // Skip the first byte in the buffer

        let b = block_index.unwrap_or(self.current_block_index);
        let p = page_index.unwrap_or(self.current_page_index);
        let address = OnboardFlash::get_address(b, p);
        let plane = ((b % 2) << 4) as u8;
        bytes[1] = PROGRAM_LOAD;
        bytes[2] = plane;
        bytes[3] = 0;
        {
            //Now write into the user meta section
            bytes[4..][0x820..=0x83f][0] = 0; // Page is used
            bytes[4..][0x820..=0x83f][1] = if is_last { 0 } else { 0xff }; // Page is last page of file?
            {
                let space = &mut bytes[4..][0x820..=0x83f][2..=3];
                //info!("Writing {} bytes", user_bytes_length);
                LittleEndian::write_u16(space, user_bytes_length as u16);
            }
            // TODO Write user detected bad blocks into user-metadata section?

            bytes[4..][0x820..=0x83f][4] = b as u8;
            bytes[4..][0x820..=0x83f][5] = p as u8;
            {
                let space2 = &mut bytes[4..][0x820..=0x83f][6..=7];
                LittleEndian::write_u16(space2, user_bytes_length as u16);
            }
            //info!("Wrote user meta {:?}", bytes[4..][0x820..=0x83f][0..10]);
        }
        // info!(
        //     "Writing {} to block:page {}:{}, is last {}",
        //     user_bytes_length, b, p, is_last
        // );
        if is_last {
            warn!("Ending file at {}:{}", b, p);
        }

        //info!("Write address {:?}", address);
        self.spi_write(&bytes[1..]);
        self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
        let status = self.wait_for_ready();

        // TODO: Check ECC status, mark and relocate block if needed.
        //info!("Status after program {:#010b}", status.inner);
        if !status.program_failed() {
            if self.first_used_block_index.is_none() {
                self.first_used_block_index = Some(b);
            }
            if self.last_used_block_index.is_none() {
                self.last_used_block_index = Some(b);
            } else if let Some(last_used_block_index) = self.last_used_block_index {
                if last_used_block_index < b {
                    self.last_used_block_index = Some(b);
                }
            }
            if block_index.is_none() && page_index.is_none() {
                self.advance_file_cursor(is_last);
            }
        } else {
            error!("Programming failed");
        }
        if !status.erase_failed() {
            // Relocate earlier pages on this block to the next free block
            // Re-write this page
            // Erase and mark the earlier block as bad.
        }
    }
}
