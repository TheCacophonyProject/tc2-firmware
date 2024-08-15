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
use crate::rp2040_flash::PAGE_SIZE;
use byteorder::{ByteOrder, LittleEndian};
use core::char::MAX;
use core::mem;
use cortex_m::singleton;
use crc::{Crc, CRC_16_XMODEM};
use defmt::{error, info, println, warn, Format};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
};

use fugit::{HertzU32, RateExtU32};
use rp2040_hal::dma::single_buffer::Transfer;
use rp2040_hal::dma::{bidirectional, Channel, CH1, CH2, CH5};
use rp2040_hal::dma::{single_buffer, CH0};
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

const NUM_RECORDING_BLOCKS: isize = 2048 - 5; // Leave 1 block between recordings and event logs

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

#[derive(Format)]
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
    inner: Option<&'static mut [u8; 4 + 2048 + 128]>,
}

impl Page {
    fn new(blank_page: &'static mut [u8; 4 + 2048 + 128]) -> Page {
        // NOTE: We include 3 bytes at the beginning of the buffer for the command + column address
        blank_page[0] = CACHE_READ;
        blank_page[1] = 0;
        blank_page[2] = 0;
        blank_page[3] = 0;
        Page {
            inner: Some(blank_page),
        }
    }

    fn cache_data(&self) -> &[u8] {
        &self.inner.as_ref().unwrap()[4..]
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

    pub fn file_start(&self) -> Option<u16> {
        let start = LittleEndian::read_u16(&self.user_metadata_1()[12..=13]);
        if start == u16::MAX {
            None
        } else {
            Some(start)
        }
    }

    pub fn previous_file_start(&self) -> Option<(u16)> {
        let block = LittleEndian::read_u16(&self.user_metadata_1()[14..=15]);
        if block == u16::MAX {
            None
        } else {
            Some(block)
        }
    }

    fn is_last_page_for_file(&self) -> bool {
        self.user_metadata_1()[1] == 0
    }

    pub fn page_bytes_used(&self) -> usize {
        LittleEndian::read_u16(&self.user_metadata_1()[2..=3]) as usize
    }

    pub fn page_crc(&self) -> u16 {
        let crc_1 = LittleEndian::read_u16(&self.user_metadata_1()[8..=9]);
        let crc_2 = LittleEndian::read_u16(&self.user_metadata_1()[10..=11]);
        if crc_1 != crc_2 {
            warn!("crc mismatch {} vs {}", crc_1, crc_2);
        }
        crc_1
    }

    fn cache_data_mut(&mut self) -> &mut [u8] {
        &mut self.inner.as_mut().unwrap()[4..]
    }

    fn page_mut(&mut self) -> &mut [u8] {
        &mut self.inner.as_mut().unwrap()[..]
    }

    fn take(&mut self) -> &'static mut [u8; 4 + 2048 + 128] {
        self.inner.take().unwrap()
    }
}

pub unsafe fn extend_lifetime<'b>(r: &'b [u8]) -> &'static [u8] {
    mem::transmute::<&'b [u8], &'static [u8]>(r)
}

pub unsafe fn extend_lifetime_generic<'b, T>(r: &'b T) -> &'static T {
    mem::transmute::<&'b T, &'static T>(r)
}

pub unsafe fn extend_lifetime_generic_mut<'b, T>(r: &'b mut T) -> &'static mut T {
    mem::transmute::<&'b mut T, &'static mut T>(r)
}

pub unsafe fn extend_lifetime_generic_mut_2<'b, T, const SIZE: usize>(
    r: &'b mut [T; SIZE],
) -> &'static mut [T; SIZE] {
    mem::transmute::<&'b mut [T; SIZE], &'static mut [T; SIZE]>(r)
}

// pub unsafe fn extend_lifetime_a<'b>(
//     r: &'b Mutex<RefCell<[FrameSeg; 4]>>,
// ) -> &'static Mutex<RefCell<[FrameSeg; 4]>> {
//     mem::transmute::<&'b Mutex<RefCell<[FrameSeg; 4]>>, &'static Mutex<RefCell<[FrameSeg; 4]>>>(r)
// }

pub unsafe fn extend_lifetime_mut<'b>(r: &'b mut [u8]) -> &'static mut [u8] {
    mem::transmute::<&'b mut [u8], &'static mut [u8]>(r)
}

// pub unsafe fn extend_lifetime_mut<'b, T>(r: &'b mut T) -> &'static mut T {
//     mem::transmute::<&'b mut T, &'static mut T>(r)
// }

pub struct OnboardFlash {
    pub spi: Option<
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
    pub previous_file_start: Option<u16>,
    pub first_used_block_index: Option<isize>,
    pub bad_blocks: [i16; 40],
    pub current_page: Page,
    pub prev_page: Page,
    dma_channel_1: Option<Channel<CH1>>,
    dma_channel_2: Option<Channel<CH2>>,
    record_to_flash: bool,
    pub payload_buffer: Option<&'static mut [u8; 2115]>,
    pub file_start: Option<u16>,
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
        flash_page_buf: &'static mut [u8; 4 + 2048 + 128],
        flash_page_buf_2: &'static mut [u8; 4 + 2048 + 128],

        dma_channel_1: Channel<CH1>,
        dma_channel_2: Channel<CH2>,
        should_record: bool,
        payload_buffer: Option<&'static mut [u8; 2115]>,
    ) -> OnboardFlash {
        OnboardFlash {
            cs,
            mosi_disabled: Some(mosi),
            clk_disabled: Some(clk),
            miso_disabled: Some(miso),
            spi: None,
            first_used_block_index: None,
            last_used_block_index: None,
            // previous_file_index: None,
            bad_blocks: [i16::MAX; 40],
            current_page_index: 0,
            current_block_index: 0,
            current_page: Page::new(flash_page_buf),
            prev_page: Page::new(flash_page_buf_2),
            dma_channel_1: Some(dma_channel_1),
            dma_channel_2: Some(dma_channel_2),
            record_to_flash: should_record,
            payload_buffer: payload_buffer,
            file_start: None,
            previous_file_start: None,
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
            self.read_page_metadata(0);
            self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                warn!("Page 1 has data");
            }
        }
        let mut good_block = false;
        for block_index in 0..2048 {
            // TODO: Interleave with random cache read
            // TODO: We can see if this is faster if we just read the column index of the end of the page?
            // For simplicity at the moment, just read the full pages
            self.read_page(block_index, 1).unwrap();
            self.read_page_metadata(block_index);
            self.wait_for_all_ready();
            if self.current_page.is_part_of_bad_block() {
                if let Some(slot) = bad_blocks.iter_mut().find(|x| **x == i16::MAX) {
                    // Add the bad block to our runtime table.
                    *slot = block_index as i16;
                    if !good_block && block_index < NUM_RECORDING_BLOCKS - 1 {
                        self.current_block_index = block_index + 1;
                    }
                }
            } else if block_index < NUM_RECORDING_BLOCKS {
                good_block = true;
                if !self.current_page.page_is_used() {
                    // This will be the starting block of the next file to be written.
                    if self.last_used_block_index.is_none() && self.first_used_block_index.is_some()
                    {
                        self.last_used_block_index = Some(block_index - 1);
                        self.file_start = self.prev_page.file_start();

                        self.current_block_index = block_index;
                        self.current_page_index = 0;
                        println!(
                            "Setting next starting block index {} last used is {}",
                            block_index, self.last_used_block_index
                        );
                    }
                } else {
                    let address = OnboardFlash::get_address(block_index, 0);
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
        .init(resets, freq, 40_000_000.Hz(), &embedded_hal::spi::MODE_3);
        self.spi = Some(spi);
    }

    pub fn has_files_to_offload(&self) -> bool {
        // When we did our initial scan, did we encounter any used blocks?
        let has_files = self.first_used_block_index.is_some();
        if has_files {
            info!(
                "First used block {:?}, last used {:?}",
                self.first_used_block_index, self.last_used_block_index
            );
        }
        has_files
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
    }

    pub fn is_too_full_to_start_new_recordings(&self) -> bool {
        // Whether or not we should start any new recordings, or should offload.
        self.current_block_index > (NUM_RECORDING_BLOCKS - 256)
    }
    pub fn is_too_full_for_audio(&self) -> bool {
        // Lets us know when we should end the current recording.
        // Need about 43 blocks for a 60 seconds recording at 48khz
        self.current_block_index > (NUM_RECORDING_BLOCKS - 44)
    }

    pub fn is_nearly_full(&self) -> bool {
        // Lets us know when we should end the current recording.
        // We only need to allow a single frames worth – 2–3 blocks should be more than enough!
        self.current_block_index > (NUM_RECORDING_BLOCKS - 3)
    }

    pub fn erase_all_blocks(&mut self) {
        'outer: for block_index in 0..NUM_RECORDING_BLOCKS {
            if self.bad_blocks.contains(&(block_index as i16)) {
                info!("Skipping erase of bad block {}", block_index);
            } else if !self.erase_block(block_index).is_ok() {
                error!("Block erase failed for block {}", block_index);
            }
        }
        self.scan();
    }

    pub fn erase_all_good_used_blocks(&mut self) {
        'outer: for block_index in 0..NUM_RECORDING_BLOCKS {
            if self.bad_blocks.contains(&(block_index as i16)) {
                info!("Skipping erase of bad block {}", block_index);
                continue 'outer;
            }
            self.read_page(block_index, 0).unwrap();

            // TODO: Could just read the user-metadata, not the full page, might be faster.
            self.read_page_metadata(block_index);
            if self.current_page.page_is_used() {
                self.erase_block(block_index).unwrap();
            } else {
                // If we encounter an unused first page of a block, that means we've gone past the
                // end of used storage, and blocks should already be in an erased state.
                break;
            }
        }
        self.first_used_block_index = None;
        self.last_used_block_index = None;
        self.file_start = None;
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

    pub fn erase_last_file(&mut self) -> Result<(), &str> {
        //havent started a file
        //haven't used a block
        //havent written to file yet

        let (start, end) = (self.file_start, self.last_used_block_index);

        if start.is_none()
            || self.last_used_block_index.is_none()
            || self.last_used_block_index.unwrap() < start.unwrap() as isize
        {
            // self.last_used_block_index = self.previous_file_index;
            info!(
                "Nothing to erase start {} last used block {}",
                start, self.last_used_block_index
            );
            return Err("File hasn't been written too");
        }
        let start_block_index = start.unwrap() as isize;
        info!("Erasing last file {}:0 to {}", start_block_index, end);

        for block_index in start_block_index..=end.unwrap() {
            if self.bad_blocks.contains(&(block_index as i16)) {
                info!("Skipping erase of bad block {}", block_index);
            } else if !self.erase_block(block_index).is_ok() {
                error!("Block erase failed for block {}", block_index);
                return Err("Block erase failed");
            }
        }
        self.file_start = self.previous_file_start;
        self.previous_file_start = None;
        info!("Set file start {}", self.file_start);
        if start_block_index == 0 {
            self.first_used_block_index = None;
            self.last_used_block_index = None;
            self.current_page_index = 0;
            self.current_block_index = 0;
        } else {
            self.last_used_block_index = Some(start_block_index - 1);
            self.current_block_index = start_block_index;
            self.current_page_index = 0;
        }
        Ok(())
    }

    pub fn free_spi(&mut self) -> Option<SPI1> {
        if self.spi.is_some() {
            let spi_enabled = self.spi.take().unwrap();
            let spi_disabled = spi_enabled.disable();
            let (spi, (mosi, miso, clk)) = spi_disabled.free();
            self.mosi_disabled = Some(mosi.into_pull_down_disabled().into_pull_type());
            self.clk_disabled = Some(clk.into_pull_down_disabled().into_pull_type());
            self.miso_disabled = Some(miso.into_pull_down_disabled().into_pull_type());

            Some(spi)
        } else {
            None
        }
    }
    pub fn get_file_part(&mut self) -> Option<((&[u8], u16, isize, isize), bool, SPI1)> {
        // TODO: Could interleave using cache_random_read
        if self
            .read_page(self.current_block_index, self.current_page_index)
            .is_ok()
        {
            self.read_page_from_cache(self.current_block_index);
            if self.current_page.page_is_used() {
                let length = self.current_page.page_bytes_used();
                let crc = self.current_page.page_crc();
                let is_last_page_for_file = self.current_page.is_last_page_for_file();
                if is_last_page_for_file {
                    info!(
                        "Got last file {}:{} file start is {} previous file start is {}",
                        self.current_block_index,
                        self.current_page_index,
                        self.current_page.file_start(),
                        self.current_page.previous_file_start()
                    );
                }
                let block = self.current_block_index;
                let page = self.current_page_index;
                self.advance_file_cursor(is_last_page_for_file);
                let spi = self.free_spi().unwrap();
                Some((
                    (&self.current_page.user_data()[0..length], crc, block, page),
                    is_last_page_for_file,
                    spi,
                ))
            } else {
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
            // warn!(
            //     "unrecoverable data corruption error at {}:{} - should relocate? {}",
            //     block, page, should_relocate
            // );
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

    pub fn begin_offload_reverse(&mut self) -> (bool, bool) {
        if let Some(last_block_index) = self.last_used_block_index {
            info!("Offload reverse last used {}", self.last_used_block_index);
            self.read_page(last_block_index, 0).unwrap();
            self.read_page_metadata(last_block_index);
            self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                info!("Page used {}:{}", last_block_index, 0);
                if let Some(start_block) = self.current_page.file_start() {
                    self.current_block_index = start_block as isize;
                    self.current_page_index = 0;
                    self.file_start = Some(self.current_block_index as u16);
                    self.previous_file_start = self.current_page.previous_file_start();
                    info!(
                        "Set file start to {}:{} and previous {}",
                        self.current_block_index, 0, self.previous_file_start
                    );
                    return (true, true);
                } else {
                    //do something old file system only need to offload them once
                    return (true, false);
                }
            } else {
                //this shouldn't happen either
                warn!("Last used block is empty {}:{}", last_block_index, 0);
                return (false, true);
            }
        } else {
            (false, true)
        }
    }

    fn last_dirty_page(&mut self, block_index: isize) -> Result<isize, ()> {
        for page in (0..64).rev() {
            info!("Looking for dirty page at {}:{}", block_index, page);
            self.read_page(block_index, page).unwrap();
            self.read_page_metadata(block_index);
            self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                return Ok(page);
            }
        }
        Err(())
    }

    pub fn read_from_cache_at_column_offset(
        &mut self,
        block: isize,
        offset: isize,
        length: Option<usize>,
    ) {
        let plane = (((block % 2) << 4) | offset >> 8) as u8;
        let current_page = self.current_page.take();
        current_page[0] = CACHE_READ;
        current_page[1] = plane;
        current_page[2] = (offset & 0xff) as u8;
        current_page[3] = 0; // Dummy byte
        let length = length.unwrap_or((2048 + 128) - offset as usize);
        // TODO: Use a pair of global buffers for all DMA transfers

        let prev_page = self.prev_page.take();
        {
            self.cs.set_low().unwrap();

            // If the offset is 2048, we want the actual offset in our buffer to be 2052, then the
            // start is at 2048
            let offset = offset as usize;
            let src_range = 0..length + 4;
            let dst_range = offset..offset + length + 4;
            crate::assert_eq!(src_range.len(), dst_range.len());
            let transfer = bidirectional::Config::new(
                (
                    self.dma_channel_1.take().unwrap(),
                    self.dma_channel_2.take().unwrap(),
                ),
                unsafe { extend_lifetime(&current_page[src_range]) },
                self.spi.take().unwrap(),
                // TO ensure the data is placed in the correct place on the page, offset by -4
                unsafe { extend_lifetime_mut(&mut prev_page[dst_range]) },
            )
            .start();

            let ((r_ch1, r_ch2), r_tx_buf, spi, r_rx_buf) = transfer.wait();
            // Do we also need some CRC checking?
            self.cs.set_high().unwrap();
            self.dma_channel_1 = Some(r_ch1);
            self.dma_channel_2 = Some(r_ch2);
            self.spi = Some(spi);
        }

        // Copy to the appropriate column offset:

        // Swap the buffers here.
        self.current_page.inner = Some(prev_page);
        self.prev_page.inner = Some(current_page);
    }

    pub fn read_event_from_cache_at_column_offset_spi(
        &mut self,
        block: isize,
        offset: isize,
    ) -> [u8; 18] {
        let plane = ((block % 2) << 4) as u8;
        let mut bytes = [0u8; 22];
        bytes[0] = CACHE_READ;
        bytes[1] = plane;
        bytes[2] = offset as u8;
        bytes[3] = 0;
        self.cs.set_low().unwrap();
        self.spi.as_mut().unwrap().transfer(&mut bytes).unwrap();
        self.cs.set_high().unwrap();
        let mut event = [0u8; 18];
        event.copy_from_slice(&bytes[4..]);
        event
    }

    pub fn read_page_metadata(&mut self, block: isize) {
        self.read_from_cache_at_column_offset(block, 2048, None);
    }

    pub fn read_page_from_cache(&mut self, block: isize) {
        self.read_from_cache_at_column_offset(block, 0, None);
    }
    pub fn spi_write(&mut self, bytes: &[u8]) {
        if let Some(spi) = self.spi.as_mut() {
            self.cs.set_low().unwrap();
            spi.write(bytes).unwrap();
            self.cs.set_high().unwrap();
        } else {
            error!("Onboard flash doesn't own SPI, can't do write");
        }
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
    pub fn start_file(&mut self, start_page: isize) -> isize {
        // CPTV always start writing a new file at page 1, reserving page 0 for when we come back
        // and write the header once we've finished writing the file.
        self.previous_file_start = self.file_start;
        self.current_page_index = start_page;
        warn!(
            "Starting file at file block {}, page {} previous is {}",
            self.current_block_index, self.current_page_index, self.previous_file_start
        );
        self.file_start = Some(self.current_block_index as u16);
        self.current_block_index
    }

    pub fn finish_transfer(
        &mut self,
        block_index: Option<isize>,
        page_index: Option<isize>,
        transfer: bidirectional::Transfer<
            Channel<CH1>,
            Channel<CH2>,
            &'static mut [u8; 2115],
            Spi<
                Enabled,
                SPI1,
                (
                    Pin<Gpio11, FunctionSpi, PullDown>,
                    Pin<Gpio8, FunctionSpi, PullDown>,
                    Pin<Gpio10, FunctionSpi, PullDown>,
                ),
            >,
            &'static mut [u8; 2115],
        >,
        address: [u8; 3],
        is_last: bool,
    ) -> OnboardFlashStatus {
        let mut b = block_index.unwrap_or(self.current_block_index);
        let mut p = page_index.unwrap_or(self.current_page_index);
        let ((r_ch1, r_ch2), tx_buf, spi, rx_buf) = transfer.wait();
        self.cs.set_high().unwrap();
        self.payload_buffer = Some(tx_buf);
        self.dma_channel_1 = Some(r_ch1);
        self.dma_channel_2 = Some(r_ch2);
        self.spi = Some(spi);
        self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
        let status = self.wait_for_ready();
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
        return status;
    }

    pub fn append_file_bytes_async(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: Option<isize>,
        page_index: Option<isize>,
        transfer: Option<
            bidirectional::Transfer<
                Channel<CH1>,
                Channel<CH2>,
                &'static mut [u8; 2115],
                Spi<
                    Enabled,
                    SPI1,
                    (
                        Pin<Gpio11, FunctionSpi, PullDown>,
                        Pin<Gpio8, FunctionSpi, PullDown>,
                        Pin<Gpio10, FunctionSpi, PullDown>,
                    ),
                >,
                &'static mut [u8; 2115],
            >,
        >,
        address: Option<[u8; 3]>,
    ) -> Result<
        (
            Option<
                bidirectional::Transfer<
                    Channel<CH1>,
                    Channel<CH2>,
                    &'static mut [u8; 2115],
                    Spi<
                        Enabled,
                        SPI1,
                        (
                            Pin<Gpio11, FunctionSpi, PullDown>,
                            Pin<Gpio8, FunctionSpi, PullDown>,
                            Pin<Gpio10, FunctionSpi, PullDown>,
                        ),
                    >,
                    &'static mut [u8; 2115],
                >,
            >,
            Option<[u8; 3]>,
        ),
        &str,
    > {
        if let Some(transfer) = transfer {
            let status =
                self.finish_transfer(block_index, page_index, transfer, address.unwrap(), false);

            if !status.erase_failed() {}
        }
        let mut b = block_index.unwrap_or(self.current_block_index);
        let mut p = page_index.unwrap_or(self.current_page_index);

        if b > NUM_RECORDING_BLOCKS {
            return Err(&"Flash full");
        }
        self.write_enable();
        assert_eq!(self.write_enabled(), true);
        // Bytes will always be a full page + metadata + command info at the start
        assert_eq!(bytes.len(), 2112 + 4); // 2116
                                           // Skip the first byte in the buffer

        let address = Some(OnboardFlash::get_address(b, p));
        let plane = ((b % 2) << 4) as u8;
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&bytes[4..4 + user_bytes_length]);
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
                let space = &mut bytes[4..][0x820..=0x83f][6..=7];
                LittleEndian::write_u16(space, user_bytes_length as u16);
            }
            {
                let space = &mut bytes[4..][0x820..=0x83f][8..=9];
                LittleEndian::write_u16(space, crc);
            }
            {
                let space = &mut bytes[4..][0x820..=0x83f][10..=11];
                LittleEndian::write_u16(space, crc);
            }
            //info!("Wrote user meta {:?}", bytes[4..][0x820..=0x83f][0..10]);
        }
        // info!(
        //     "Writing {} to block:page {}:{}, is last {}",
        //     user_bytes_length, b, p, is_last
        // );
        // if is_last {
        // warn!("Ending file at {}:{}", b, p);
        // }
        let mut transfer = None;
        if self.record_to_flash {
            if self.payload_buffer.is_none() {
                return Err("Payload buffer is None have you called init_async_buf??");
            }
            self.payload_buffer.as_mut().unwrap()[..bytes.len() - 1].copy_from_slice(&bytes[1..]);
            self.cs.set_low().unwrap();
            let buf = self.payload_buffer.as_mut().unwrap();

            let mut rx_buf = [0x42u8; 2115];
            let rx_buf = unsafe { extend_lifetime_generic_mut(&mut rx_buf) };

            transfer = Some(
                bidirectional::Config::new(
                    (
                        self.dma_channel_1.take().unwrap(),
                        self.dma_channel_2.take().unwrap(),
                    ),
                    self.payload_buffer.take().unwrap(),
                    self.spi.take().unwrap(),
                    rx_buf,
                )
                .start(),
            );
            if (is_last) {
                let status = self.finish_transfer(
                    block_index,
                    page_index,
                    transfer.take().unwrap(),
                    address.unwrap(),
                    true,
                );
                return Ok((None, None));
            }
        }

        return Ok((transfer, address));
    }

    pub fn append_file_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: Option<isize>,
        page_index: Option<isize>,
    ) -> Result<(), &str> {
        self.write_enable();
        assert_eq!(self.write_enabled(), true);
        // Bytes will always be a full page + metadata + command info at the start
        assert_eq!(bytes.len(), 2112 + 4); // 2116

        // Skip the first byte in the buffer
        let b = block_index.unwrap_or(self.current_block_index);
        let p = page_index.unwrap_or(self.current_page_index);
        if b > NUM_RECORDING_BLOCKS {
            return Err(&"Flash full");
        }
        let address = OnboardFlash::get_address(b, p);
        let plane = ((b % 2) << 4) as u8;
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&bytes[4..4 + user_bytes_length]);
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
                let space = &mut bytes[4..][0x820..=0x83f][6..=7];
                LittleEndian::write_u16(space, user_bytes_length as u16);
            }
            {
                let space = &mut bytes[4..][0x820..=0x83f][8..=9];
                LittleEndian::write_u16(space, crc);
            }
            {
                let space = &mut bytes[4..][0x820..=0x83f][10..=11];
                LittleEndian::write_u16(space, crc);
            }
            //write file start block
            let space = &mut bytes[4..][0x820..=0x83f][12..=13];
            LittleEndian::write_u16(space, self.file_start.unwrap());
            if self.file_start.unwrap() > 0 {
                // if let Some(previous_start_block) = self.previous_file_start {
                //write previous file start could just write on first page if it matters
                let space = &mut bytes[4..][0x820..=0x83f][14..=15];
                LittleEndian::write_u16(space, self.previous_file_start.unwrap() as u16);
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

        if self.record_to_flash {
            self.spi_write(&bytes[1..]);
            self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
        }

        // FIXME - can program failed bit get set, and then discarded, before wait for ready completes?
        let status = self.wait_for_ready();

        // TODO: Check ECC status, mark and relocate block if needed.
        //info!("Status after program {:#010b}", status.inner);
        if !status.program_failed() {
            if self.first_used_block_index.is_none() {
                self.first_used_block_index = Some(b);
            }
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
        return Ok(());
    }

    pub fn write_event(
        &mut self,
        event_bytes: &[u8; 18],
        block_index: isize,
        page_index: isize,
        offset: u16,
    ) {
        if self.spi.is_some() {
            self.write_enable();
            assert_eq!(self.write_enabled(), true);
            let mut bytes = [0u8; 21];
            bytes[3..21].copy_from_slice(event_bytes);
            let address = OnboardFlash::get_address(block_index, page_index);
            let plane = ((block_index % 2) << 4) as u8;

            // Skip the first byte in the buffer
            bytes[0] = PROGRAM_LOAD;
            bytes[1] = plane;
            bytes[2] = offset as u8;
            self.spi_write(&bytes);
            self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
            // FIXME - can program failed bit get set, and then discarded, before wait for ready completes?
            let status = self.wait_for_ready();
            if !status.program_failed() {
            } else {
                error!("Programming failed");
            }
            if !status.erase_failed() {
                // Relocate earlier pages on this block to the next free block
                // Re-write this page
                // Erase and mark the earlier block as bad.
            }
        } else {
            error!("Failed writing event: SPI not available");
        }
    }
}
