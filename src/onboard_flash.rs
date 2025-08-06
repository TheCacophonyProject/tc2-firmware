// Onboard flash owns the SPI peripheral it needs to function.
// When it initialises, it looks for a version number written at the beginning of the flash address
// space.  If the version doesn't match the known version it re-inits the flash.
// If there's no version number, it initialises a basic file table.

// The file table just contains a series of offsets into the flash memory, and we simply
// allocate linearly, relying on the fact we should be able to get 100,000 writes just fine.

// If we need some kind of bad-block detection we can worry about implementing that later.

// We keep a pointer to the next free byte offset updated, though we may only be able to write things
// in blocks of a certain size.

use crate::Utc;
use crate::bsp::pac::SPI1;
use crate::byte_slice_cursor::CursorMut;
use crate::utils::{extend_lifetime, extend_lifetime_mut};

use byteorder::{ByteOrder, LittleEndian};
use chrono::DateTime;
use crc::{CRC_16_XMODEM, Crc};
use defmt::{Format, error, info, warn};

use crate::attiny_rtc_i2c::RecordingRequestType;
use crate::event_logger::{Event, EventLogger};
use crate::synced_date_time::SyncedDateTime;
use cortex_m::prelude::*;
use embedded_hal::digital::OutputPin;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::dma::{CH1, CH2, Channel, bidirectional};
use rp2040_hal::gpio::bank0::{Gpio8, Gpio9, Gpio10, Gpio11};
use rp2040_hal::gpio::{
    FunctionNull, FunctionSio, FunctionSpi, Pin, PullDown, PullNone, SioOutput,
};
use rp2040_hal::pac::RESETS;
use rp2040_hal::spi::Enabled;
use rp2040_hal::{Spi, Watchdog};

const WRITE_ENABLE: u8 = 0x06;

const BLOCK_ERASE: u8 = 0xd8;
pub const PROGRAM_LOAD: u8 = 0x02;
pub const PROGRAM_EXECUTE: u8 = 0x10;
const RESET: u8 = 0xff;
const GET_FEATURES: u8 = 0x0f;
const SET_FEATURES: u8 = 0x1f;
const _DEVICE_ID: u8 = 0x9f;
pub const PAGE_READ: u8 = 0x13;
const _PAGE_READ_RANDOM: u8 = 0x30;
const _PAGE_READ_LAST: u8 = 0x3f;
const CACHE_READ: u8 = 0x0b;

const FEATURE_STATUS: u8 = 0xc0;
const _FEATURE_CONFIG: u8 = 0xb0;
const FEATURE_BLOCK_LOCK: u8 = 0xa0;
const _FEATURE_DIE_SELECT: u8 = 0xd0;
const NUM_EVENT_BYTES: usize = 18;

pub const TOTAL_FLASH_BLOCKS: u16 = 2048;
const NUM_CONFIG_BLOCKS: u8 = 2;
const NUM_EVENT_BLOCKS: u8 = 5;
pub const NUM_RECORDING_BLOCKS: u16 =
    TOTAL_FLASH_BLOCKS - (NUM_CONFIG_BLOCKS as u16 + NUM_EVENT_BLOCKS as u16);
const NUM_PAGES_PER_BLOCK: u8 = 64;

pub struct OnboardFlashStatus {
    inner: u8,
}

#[derive(Format)]
pub struct EccStatus {
    pub okay: bool,
    should_relocate: bool,
}

pub type BlockIndex = u16;
pub type PageIndex = u8;

#[derive(Copy, Clone, Format)]
pub struct RecordingFileTypeDetails {
    pub user_requested: bool,
    pub startup_status: bool,
    pub shutdown_status: bool,
}

#[derive(Copy, Clone)]
pub enum RecordingFileType {
    Cptv(RecordingFileTypeDetails),
    Audio(RecordingFileTypeDetails),
    Other,
}

impl OnboardFlashStatus {
    pub fn cache_read_busy(&self) -> bool {
        self.inner & 0b1000_0000 != 0
    }

    pub fn ecc_status(&self) -> EccStatus {
        let ecc_bits = (self.inner >> 4) & 0b0000_0111;
        match ecc_bits {
            0b000 => EccStatus {
                okay: true,
                should_relocate: false,
            },
            0b001 => {
                warn!("ECC error - 1-3 bits corrected");
                EccStatus {
                    okay: true,
                    should_relocate: false,
                }
            }
            0b010 => EccStatus {
                okay: false,
                should_relocate: true,
            },
            0b011 => {
                warn!("ECC error - 4-6 bits corrected, should re-locate data");
                EccStatus {
                    okay: true,
                    should_relocate: true,
                }
            }
            0b101 => {
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

    pub fn operation_in_progress(&self) -> bool {
        self.inner & 0b0000_0001 != 0
    }
}

pub const FLASH_SPI_HEADER: usize = 4;
pub const FLASH_SPI_HEADER_SMALL: usize = 3;
pub const FLASH_USER_PAGE_SIZE: usize = 2048;
pub const FLASH_USER_METADATA_SIZE: usize = 64;
pub const FLASH_METADATA_SIZE: usize = 128;

// NOTE: To save some bytes, when reading back the flash page from cache via SPI we don't read the
//  full page, just the user parts we care about
pub const FLASH_SPI_TOTAL_PAYLOAD_SIZE: usize =
    FLASH_SPI_HEADER + FLASH_USER_PAGE_SIZE + FLASH_METADATA_SIZE;
pub const FLASH_SPI_USER_PAYLOAD_SIZE: usize =
    FLASH_SPI_HEADER + FLASH_USER_PAGE_SIZE + FLASH_USER_METADATA_SIZE;

pub type FlashSpiFullPayload = &'static mut [u8; FLASH_SPI_TOTAL_PAYLOAD_SIZE];

pub struct Page {
    inner: Option<FlashSpiFullPayload>,
}

impl Page {
    fn new(blank_page: FlashSpiFullPayload) -> Page {
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
        &self.inner.as_ref().unwrap()[FLASH_SPI_HEADER..]
    }

    pub fn user_data(&self) -> &[u8] {
        &self.cache_data()[0..FLASH_USER_PAGE_SIZE]
    }

    #[allow(dead_code)]
    pub fn metadata(&self) -> &[u8] {
        &self.cache_data()[FLASH_USER_PAGE_SIZE..]
    }

    pub fn user_metadata_1(&self) -> &[u8] {
        &self.cache_data()[0x820..=0x83f]
    }

    // NOTE: User metadata 2 doesn't have ECC correction, so we may never want to use it.
    #[allow(dead_code)]
    fn user_metadata_2(&self) -> &[u8] {
        &self.cache_data()[0x804..=0x81f]
    }

    fn bad_block_data(&self) -> &[u8] {
        &self.cache_data()[0x800..=0x803]
    }

    fn is_part_of_bad_block(&self) -> bool {
        self.bad_block_data().contains(&0)
    }

    // User metadata 1 contains 32 bytes total
    // [0] = set to zero if page is used.
    // [1] = set to zero if this is the *last* page for a file.
    // [2, 3] = length of page used as little-endian u16
    pub fn page_is_used(&self) -> bool {
        self.user_metadata_1()[0] == 0
    }

    pub fn is_user_requested_test_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b011 == 0b010
    }

    pub fn is_status_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b001 == 0b001
    }

    pub fn is_startup_status_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b011 == 0b001
    }

    pub fn is_shutdown_status_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b011 == 0b011
    }

    pub fn is_cptv_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b100 == 0b100
    }

    pub fn is_audio_recording(&self) -> bool {
        self.user_metadata_1()[4] & 0b100 == 0b000
    }

    pub fn file_type(&self) -> FileType {
        if self.is_cptv_recording() {
            if self.is_startup_status_recording() {
                FileType::CptvStartup
            } else if self.is_shutdown_status_recording() {
                FileType::CptvShutdown
            } else if self.is_user_requested_test_recording() {
                FileType::CptvUserRequested
            } else {
                FileType::CptvScheduled
            }
        } else if self.is_user_requested_test_recording() {
            FileType::AudioUserRequested
        } else {
            FileType::AudioScheduled
        }
    }

    // NOTE: File written time is only available in the metadata section of the *first page* in a file.
    pub fn file_written_time(&self) -> Option<DateTime<Utc>> {
        let start = LittleEndian::read_u16(&self.user_metadata_1()[16..=17]);
        if start == u16::MAX {
            None
        } else {
            let timetamp = LittleEndian::read_i64(&self.user_metadata_1()[16..24]);
            DateTime::from_timestamp_micros(timetamp)
        }
    }

    pub fn file_start_block_index(&self) -> Option<u16> {
        let start = LittleEndian::read_u16(&self.user_metadata_1()[12..=13]);
        if start == u16::MAX { None } else { Some(start) }
    }

    pub fn previous_file_start_block_index(&self) -> Option<u16> {
        let block = LittleEndian::read_u16(&self.user_metadata_1()[14..=15]);
        if block == u16::MAX { None } else { Some(block) }
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

    fn take(&mut self) -> FlashSpiFullPayload {
        self.inner.take().unwrap()
    }
}

// ((&[u8], u16, BlockIndex, isize), bool, SPI1)
pub struct FilePartReturn<'a> {
    pub part: &'a [u8],
    pub crc16: u16,
    pub block: BlockIndex,
    pub page: PageIndex,
    pub is_last_page_for_file: bool,
    pub spi: SPI1,
    pub metadata: FileType,
    pub timestamp: Option<DateTime<Utc>>,
}

type SpiEnabledPeripheral = Spi<
    Enabled,
    SPI1,
    (
        Pin<Gpio11, FunctionSpi, PullDown>, //, SCK
        Pin<Gpio8, FunctionSpi, PullDown>,  // MISO
        Pin<Gpio10, FunctionSpi, PullDown>, // MOSI
    ),
    8,
>;

#[repr(u8)]
#[derive(Format, PartialEq, Clone, Copy)]
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

#[derive(Default)]
pub struct FileTypes(u8);

impl FileTypes {
    pub(crate) fn add_type(&mut self, file_type: FileType) {
        self.0 |= file_type as u8;
    }

    pub(crate) fn has_type(&self, file_type: FileType) -> bool {
        (self.0 & file_type as u8) != 0
    }
}

pub struct OnboardFlash {
    pub spi: Option<SpiEnabledPeripheral>,
    cs: Pin<Gpio9, FunctionSio<SioOutput>, PullDown>,
    mosi_disabled: Option<Pin<Gpio11, FunctionNull, PullNone>>,
    clk_disabled: Option<Pin<Gpio10, FunctionNull, PullNone>>,
    miso_disabled: Option<Pin<Gpio8, FunctionNull, PullNone>>,
    pub current_page_index: PageIndex,
    pub current_block_index: BlockIndex,
    pub last_used_block_index: Option<BlockIndex>,
    pub previous_file_start_block_index: Option<BlockIndex>,
    pub first_used_block_index: Option<BlockIndex>,
    pub bad_blocks: [BlockIndex; 40],
    pub num_files_in_initial_scan: u16,
    pub current_page: Page,
    pub prev_page: Page,
    dma_channel_1: Option<Channel<CH1>>,
    dma_channel_2: Option<Channel<CH2>>,
    pub file_start_block_index: Option<BlockIndex>,
    //could use same block for both but would have to write config every audio change and vice versa
    pub config_block: Option<BlockIndex>,
    system_clock_hz: HertzU32,
    pub file_types_found: FileTypes,
    pub last_startup_recording_time: Option<DateTime<Utc>>,
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
        flash_page_buf: FlashSpiFullPayload,
        flash_page_buf_2: FlashSpiFullPayload,
        dma_channel_1: Channel<CH1>,
        dma_channel_2: Channel<CH2>,
        system_clock_hz: HertzU32,
    ) -> OnboardFlash {
        OnboardFlash {
            cs,
            mosi_disabled: Some(mosi),
            clk_disabled: Some(clk),
            miso_disabled: Some(miso),
            spi: None,
            first_used_block_index: None,
            last_used_block_index: None,
            bad_blocks: [u16::MAX; 40],
            current_page_index: 0,
            current_block_index: 0,
            current_page: Page::new(flash_page_buf),
            prev_page: Page::new(flash_page_buf_2),
            dma_channel_1: Some(dma_channel_1),
            dma_channel_2: Some(dma_channel_2),
            num_files_in_initial_scan: 0,
            file_start_block_index: None,
            previous_file_start_block_index: None,
            config_block: None,
            system_clock_hz,
            file_types_found: FileTypes::default(),
            last_startup_recording_time: None,
        }
    }
    pub fn init(&mut self, peripheral: SPI1, resets: &mut RESETS) {
        self.take_spi(peripheral, resets);
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
        self.set_config_block();
    }

    pub fn set_config_block(&mut self) {
        // Find last non bad block to use for config.  Note that we need 2 contigous good blocks
        // to store the device config.

        // FIXME: Should probably follow a similar strategy for events
        //  Also mark page metadata for first page in the block with type.
        let mut block_index = NUM_RECORDING_BLOCKS;
        while self.bad_blocks.contains(&block_index) || self.bad_blocks.contains(&(block_index + 1))
        {
            block_index = block_index.saturating_sub(1);
            info!(
                "Bad contiguous block set {}/{} for device config",
                block_index,
                block_index + 1
            );
        }
        assert_ne!(block_index, 0, "No good blocks found for config");
        self.config_block = Some(block_index);
    }

    pub fn write_device_config(&mut self, device_bytes: &mut [u8]) {
        assert!(
            self.config_block.is_some(),
            "Config block has not been initialized, call fs.init()"
        );
        let config_block = self.config_block.unwrap();
        let _ = self.erase_block(config_block);
        let mut start = 0;
        let mut page = 0;
        let mut is_last = false;

        while !is_last {
            let mut end = start + FLASH_USER_PAGE_SIZE;
            if end > device_bytes.len() {
                end = device_bytes.len();
            }
            let mut payload = [0xffu8; FLASH_SPI_USER_PAYLOAD_SIZE];
            is_last = end == device_bytes.len();

            let device_chunk = &mut device_bytes[start..end];
            payload[FLASH_SPI_HEADER..FLASH_SPI_HEADER + device_chunk.len()]
                .copy_from_slice(device_chunk);
            start += FLASH_USER_PAGE_SIZE;
            let _ = self.write_config_bytes(
                &mut payload,
                device_chunk.len(),
                is_last,
                config_block,
                page,
            );
            page += 1;
            //shouldn't ever happen unless config becomes unreasonably large
            assert!(
                page < 64,
                "Trying to write more pages of config than we have allocated"
            );
        }
    }

    pub fn write_config_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: BlockIndex,
        page_index: PageIndex,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            is_last,
            Some(block_index),
            Some(page_index),
            true,
            RecordingFileType::Other,
            None,
        )
    }

    pub fn read_device_config(&mut self) -> Result<[u8; 2505], &str> {
        assert!(
            self.config_block.is_some(),
            "Config block has not been initialized, call fs.init()"
        );
        // guess this is the size of the config at the moment
        let mut config_bytes = [0u8; 2400 + 105];

        let block_index = self.config_block.unwrap();
        let mut page_index = 0;
        let mut is_last = false;
        let mut cursor = CursorMut::new(&mut config_bytes);

        while !is_last {
            if self.read_page(block_index, page_index).is_ok() {
                self.read_page_from_cache(block_index);
                self.wait_for_all_ready();
                if !self.current_page.page_is_used() {
                    return Err("Config block empty");
                }
                let length = self.current_page.page_bytes_used();
                let data = &self.current_page.user_data()[..length];
                let _ = cursor.write_bytes(data);
                is_last = self.current_page.is_last_page_for_file();
                page_index += 1;
            } else {
                return Err("Failed to read page");
            }
        }
        Ok(config_bytes)
    }

    pub fn reset(&mut self) {
        self.spi_write(&[RESET]);
        self.wait_for_ready();
    }
    pub fn unlock_blocks(&mut self) {
        self.spi_write(&[SET_FEATURES, FEATURE_BLOCK_LOCK, 0x00]);
        self.wait_for_ready();
    }

    #[allow(dead_code)]
    pub fn last_recorded_file_is_cptv(&mut self) -> bool {
        self.has_cptv_files_internal(true)
    }

    #[allow(dead_code)]
    pub fn last_recorded_file_is_audio(&mut self) -> bool {
        !self.has_cptv_files_internal(true)
    }

    #[allow(dead_code)]
    pub fn has_cptv_files(&mut self) -> bool {
        self.has_cptv_files_internal(false)
    }

    pub fn has_cptv_files_internal(&mut self, only_last: bool) -> bool {
        if !self.has_recordings_to_offload() {
            return false;
        }
        let mut file_start = self.file_start_block_index;
        while let Some(fs) = file_start {
            // read 1 as if incomplete 0 won't be writen to
            let _ = self.read_page(fs, 0);
            self.read_page_from_cache(fs);
            self.wait_for_all_ready();
            if !self.current_page.page_is_used() {
                // possibly unfinished cptv file try previous
                let _ = self.read_page(fs, 1);
                self.read_page_metadata(fs);
                self.wait_for_all_ready();
                file_start = self.current_page.previous_file_start_block_index();
                continue;
            }
            let is_cptv =
                self.current_page.user_data()[0] != 1 || self.current_page.is_cptv_recording();
            if only_last || is_cptv {
                return is_cptv;
            }

            file_start = self.current_page.previous_file_start_block_index();
        }
        false
    }

    #[allow(clippy::too_many_lines)]
    pub fn scan(&mut self) {
        let mut bad_blocks = [u16::MAX; 40];
        self.file_types_found = FileTypes::default();
        self.first_used_block_index = None;
        self.last_used_block_index = None;
        self.current_page_index = 0;
        self.current_block_index = 0;
        let mut last_good_block = None;
        // Find first good free block:
        let mut good_block = false;
        let mut last_file_block_index = u16::MAX;
        let mut num_files = 0;

        for block_index in 0..TOTAL_FLASH_BLOCKS {
            // TODO: Interleave with random cache read
            // in the case of incomplete cptv files page 0 will be empty
            if self.read_page(block_index, 1).is_ok() {
                self.read_page_metadata(block_index);
                self.wait_for_all_ready();
                if self.current_page.is_part_of_bad_block() {
                    if let Some(slot) = bad_blocks.iter_mut().find(|x| **x == u16::MAX) {
                        // Add the bad block to our runtime table.
                        *slot = block_index;
                        if !good_block && block_index < NUM_RECORDING_BLOCKS - 1 {
                            self.current_block_index = block_index + 1;
                        }
                    }
                } else if block_index < NUM_RECORDING_BLOCKS {
                    good_block = true;
                    last_good_block = Some(block_index);

                    if self.current_page.page_is_used()
                        && let Some(file_start_block_index) =
                            self.current_page.file_start_block_index()
                        && last_file_block_index != file_start_block_index
                    {
                        last_file_block_index = block_index;
                        num_files += 1;

                        if self.current_page.is_cptv_recording() {
                            if self.current_page.is_startup_status_recording() {
                                // For the file time, we need to go back and read the first page.
                                if self.read_page(block_index, 0).is_ok() {
                                    self.read_page_metadata(block_index);
                                    self.wait_for_all_ready();

                                    self.file_types_found.add_type(FileType::CptvStartup);
                                    if self.current_page.file_written_time().is_some() {
                                        self.last_startup_recording_time =
                                            self.current_page.file_written_time();
                                    }
                                }
                                if self.read_page(block_index, 1).is_ok() {
                                    self.read_page_metadata(block_index);
                                    self.wait_for_all_ready();
                                }
                            } else if self.current_page.is_shutdown_status_recording() {
                                self.file_types_found.add_type(FileType::CptvShutdown);
                            } else if self.current_page.is_user_requested_test_recording() {
                                self.file_types_found.add_type(FileType::CptvUserRequested);
                            } else {
                                self.file_types_found.add_type(FileType::CptvScheduled);
                            }
                        } else if self.current_page.is_audio_recording() {
                            if self.current_page.is_startup_status_recording() {
                                self.file_types_found.add_type(FileType::AudioStartup);
                            } else if self.current_page.is_shutdown_status_recording() {
                                self.file_types_found.add_type(FileType::AudioShutdown);
                            } else if self.current_page.is_user_requested_test_recording() {
                                self.file_types_found.add_type(FileType::AudioUserRequested);
                            } else {
                                self.file_types_found.add_type(FileType::AudioScheduled);
                            }
                        }
                    }

                    if !self.current_page.page_is_used() {
                        // This will be the starting block of the next file to be written.
                        if self.last_used_block_index.is_none()
                            && self.first_used_block_index.is_some()
                        {
                            self.last_used_block_index = Some(block_index.saturating_sub(1));
                            self.file_start_block_index = self.prev_page.file_start_block_index();
                            self.current_block_index = block_index;
                            self.current_page_index = 0;
                            info!("Setting next starting block index {}", block_index);
                        }
                    } else if self.first_used_block_index.is_none() {
                        // This is the starting block of the first file stored.
                        info!("Storing first used block {}", block_index);
                        self.first_used_block_index = Some(block_index);
                    }
                }
            } else {
                error!("Failed to read page 1 at block {}", block_index);
                self.read_page_metadata(block_index);
                self.wait_for_all_ready();
                info!(
                    "Bad metadata, used {} {:?}",
                    self.current_page.page_is_used(),
                    self.current_page.user_metadata_1()
                );
            }
        }
        self.num_files_in_initial_scan = num_files;

        // if we have written write to the end of the flash need to handle this
        if self.last_used_block_index.is_none() && self.first_used_block_index.is_some() {
            if let Some(last_good_block) = last_good_block {
                self.last_used_block_index = Some(last_good_block);
                self.file_start_block_index = self.prev_page.file_start_block_index();

                self.current_block_index = last_good_block + 1;
                self.current_page_index = 0;
                info!(
                    "Setting next starting block as last good block index {}",
                    self.current_block_index
                );
            }
        }

        // FIXME: Maybe we'll log the number of bad blocks seen every so often
        //  (if it's a monday for instance), just so we can get a sense of what is normal in the field.

        self.bad_blocks = bad_blocks;
    }

    pub fn take_spi(&mut self, peripheral: SPI1, resets: &mut RESETS) {
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
            self.system_clock_hz,
            40_000_000.Hz(),
            embedded_hal::spi::MODE_3,
        );
        self.spi = Some(spi);
    }

    pub fn has_recordings_to_offload(&self) -> bool {
        // When we did our initial scan, did we encounter any used blocks?
        self.first_used_block_index.is_some()
    }

    pub fn last_recording_was_user_requested(&mut self, watchdog: &mut Watchdog) -> bool {
        if !self.has_recordings_to_offload() {
            false
        } else if self.last_recording_is_complete(watchdog) {
            self.current_page.is_user_requested_test_recording()
        } else {
            false
        }
    }

    #[allow(dead_code)]
    pub fn last_recording_was_status(&mut self, watchdog: &mut Watchdog) -> bool {
        if !self.has_recordings_to_offload() {
            false
        } else if self.last_recording_is_complete(watchdog) {
            self.current_page.is_status_recording()
        } else {
            false
        }
    }

    pub fn last_recording_is_complete(&mut self, watchdog: &mut Watchdog) -> bool {
        if self.has_recordings_to_offload() {
            let mut found_last_page = false;
            if let Some(start_block_index) = self.file_start_block_index {
                let mut block_index = start_block_index;
                let mut page_index = 1;
                info!(
                    "Checking if last recording is complete from block {}",
                    block_index
                );
                if self.read_page(block_index, 0).is_ok() {
                    self.read_page_from_cache(block_index);
                    self.wait_for_all_ready();
                    if !self.current_page.page_is_used() {
                        // An incomplete CPTV file.
                        return false;
                    }
                }

                loop {
                    // read 1 as if incomplete 0 won't be writen to
                    if self.read_page(block_index, page_index).is_ok() {
                        self.read_page_from_cache(block_index);
                        self.wait_for_all_ready();

                        if self.current_page.is_last_page_for_file() {
                            found_last_page = true;
                            break;
                        }
                        if !self.current_page.page_is_used() {
                            break;
                        }
                        if page_index == NUM_PAGES_PER_BLOCK - 1 {
                            page_index = 0;
                            block_index += 1;
                        } else {
                            page_index += 1;
                        }
                        if block_index == NUM_RECORDING_BLOCKS - 1 {
                            break;
                        }
                    } else {
                        error!("Failed to read block {}, page {}", block_index, page_index);
                        break;
                    }
                    watchdog.feed();
                }
            }
            found_last_page
        } else {
            true
        }
    }

    fn advance_file_cursor(&mut self, is_last: bool) {
        // If current_page < 64, increment page
        if self.current_page_index < 63 && !is_last {
            self.current_page_index += 1;
        } else {
            let mut next_block_index = self.current_block_index + 1;
            while self.bad_blocks.contains(&next_block_index) {
                info!("Skipping bad block {}", next_block_index);
                next_block_index += 1;
            }
            self.current_block_index = next_block_index;
            self.current_page_index = 0;
        }
    }

    pub fn is_too_full_to_start_new_cptv_recordings(&self) -> bool {
        // Whether or not we should start any new recordings, or should offload.
        self.current_block_index > (NUM_RECORDING_BLOCKS - 256)
    }

    pub fn can_begin_new_cptv_recordings(&self) -> bool {
        !self.is_too_full_to_start_new_cptv_recordings()
    }

    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_precision_loss)]
    #[allow(clippy::cast_sign_loss)]
    pub fn is_too_full_to_start_new_audio_recordings(
        &self,
        recording_type: &RecordingRequestType,
    ) -> bool {
        // Lets us know when we should end the current recording.
        // Need about 43 blocks for a 60 seconds recording at 48khz
        let blocks_per_second = 44.0 / 60.0;

        let num_needed_blocks =
            (recording_type.duration_seconds() as f32 * blocks_per_second) as u16 + 1;
        self.current_block_index > (NUM_RECORDING_BLOCKS - num_needed_blocks)
    }

    pub fn is_full(&self) -> bool {
        // check for page 62 incase it has to write one more page worth of data when finalising
        self.current_block_index >= NUM_RECORDING_BLOCKS
            || (self.current_block_index == (NUM_RECORDING_BLOCKS - 1)
                && self.current_page_index >= 62)
    }

    pub fn is_nearly_full_for_thermal_recordings(&self) -> bool {
        // Lets us know when we should end the current recording.
        // We only need to allow a single frames worth – 2–3 blocks should be more than enough!
        self.current_block_index > (NUM_RECORDING_BLOCKS - 3)
    }

    pub fn write_enable(&mut self) {
        self.spi_write(&[WRITE_ENABLE]);
    }

    pub fn erase_block(&mut self, block_index: BlockIndex) -> Result<(), &str> {
        self.write_enable();
        let address = OnboardFlash::get_address(block_index, 0);
        self.spi_write(&[BLOCK_ERASE, address[0], address[1], address[2]]);
        let status = self.wait_for_ready();
        if status.erase_failed() {
            Err("Block erase failed")
        } else {
            Ok(())
        }
    }

    pub fn erase_last_file(&mut self) -> Result<(), &str> {
        // havent started a file
        // haven't used a block
        // havent written to file yet

        // FIXME: Audit these unwraps
        if self.file_start_block_index.is_none()
            || self.last_used_block_index.is_none()
            || self.last_used_block_index.unwrap() < self.file_start_block_index.unwrap()
        {
            info!(
                "Nothing to erase start {} last used block {}",
                self.file_start_block_index, self.last_used_block_index
            );
            return Err("File hasn't been written to");
        }
        let start_block_index = self.file_start_block_index.unwrap();
        info!(
            "Erasing last file {}:0 to {}",
            start_block_index, self.last_used_block_index
        );

        for block_index in start_block_index..=self.last_used_block_index.unwrap() {
            if self.bad_blocks.contains(&block_index) {
                info!("Skipping erase of bad block {}", block_index);
            } else if self.erase_block(block_index).is_err() {
                error!("Block erase failed for block {}", block_index);
                return Err("Block erase failed");
            }
        }
        self.file_start_block_index = self.previous_file_start_block_index;
        self.previous_file_start_block_index = None;
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
    pub fn get_file_part(
        &mut self,
        events: &mut EventLogger,
        time: &SyncedDateTime,
    ) -> Option<FilePartReturn> {
        if let Ok(()) = self.read_page(self.current_block_index, self.current_page_index) {
            self.read_page_from_cache(self.current_block_index);
            if self.current_page.page_is_used() {
                let length = self.current_page.page_bytes_used();
                let crc16 = self.current_page.page_crc();
                let is_last_page_for_file = self.current_page.is_last_page_for_file();

                let timestamp = self.current_page.file_written_time();
                let metadata = self.current_page.file_type();

                let block = self.current_block_index;
                let page = self.current_page_index;
                self.advance_file_cursor(is_last_page_for_file);
                let spi = self.free_spi().unwrap();
                Some(FilePartReturn {
                    part: &self.current_page.user_data()[0..length],
                    crc16,
                    block,
                    page,
                    is_last_page_for_file,
                    spi,
                    metadata,
                    timestamp,
                })
            } else {
                None
            }
        } else {
            events.log(
                Event::UnrecoverableDataCorruption((
                    self.current_block_index,
                    u16::from(self.current_page_index),
                )),
                time,
                self,
            );
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
        status = self.get_status();
        status
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn get_address(block: BlockIndex, page: PageIndex) -> [u8; 3] {
        // 11 bits for block, 6 bits for page
        let address: u32 = u32::from(block) << 6 | u32::from(page);
        [(address >> 16) as u8, (address >> 8) as u8, address as u8]
    }

    #[allow(clippy::unnecessary_wraps)]
    pub fn read_page(&mut self, block: BlockIndex, page: PageIndex) -> Result<(), &str> {
        // FIXME: Handle logging of ECC errors and results of read page properly.
        assert!(block < 2048, "Invalid block");
        assert!(page < 64, "Invalid page");
        let address = OnboardFlash::get_address(block, page);
        self.spi_write(&[PAGE_READ, address[0], address[1], address[2]]);
        let status = self.wait_for_all_ready();

        let EccStatus {
            okay,
            should_relocate,
        } = status.ecc_status();
        if okay {
            if should_relocate {
                // FIXME: mark and relocate block if needed.
                // If we get a block that's going bad, we need to mark it as such after reading
                // the page.  Because we're reading the page, that means we're offloading files,
                // so we'll be erasing this block soon.  Maybe we just never use it again?
                // We don't really know how much a few ECC errors that were detected and corrected
                // escalates into a fully unusable block.  It might be better just to log an event,
                // so we can detect the frequency of this happening.

                // We don't care about refreshing/relocating the data, since it's only
                // getting read off the flash once, and then erased.
            }
            Ok(())
        } else {
            warn!(
                "unrecoverable data corruption error at {}:{} - should relocate? {}",
                block, page, should_relocate
            );
            // Unrecoverable failure.  Maybe just mark this block as bad, and mark all the blocks
            // that the file spans as temporarily corrupt, so this file doesn't get read and
            // sent to the raspberry pi.  This returns no useful data.
            Err("unrecoverable data corruption error")
        }
    }

    pub fn begin_offload_reverse(&mut self) -> bool {
        if let Some(last_block_index) = self.last_used_block_index {
            if let Some(file_start) = self.file_start_block_index {
                // read 1 as if incomplete 0 won't be writen to
                self.read_page(file_start, 1).unwrap();
                self.read_page_metadata(file_start);
                self.wait_for_all_ready();
                self.current_block_index = file_start;
                self.current_page_index = 0;
                self.previous_file_start_block_index =
                    self.current_page.previous_file_start_block_index();
                info!(
                    "Set file start to {}:{} and previous {}",
                    self.current_block_index, 0, self.previous_file_start_block_index
                );
                return true;
            }
            // old file system should only happen once
            self.current_block_index = self.find_start(last_block_index);
            self.current_page_index = 0;
            self.file_start_block_index = Some(self.current_block_index);
            info!(
                "Searched for file start found {}:{}",
                self.current_block_index, self.last_used_block_index
            );
            return true;
        }
        false
    }

    pub fn find_start(&mut self, file_block_index: BlockIndex) -> BlockIndex {
        if file_block_index == 0 {
            return 0;
        }
        // find previous file end
        for block_index in (1..file_block_index).rev() {
            if self.last_dirty_page(block_index).is_some()
                && self.current_page.is_last_page_for_file()
            {
                return block_index + 1;
            }
        }
        0
    }

    fn last_dirty_page(&mut self, block_index: BlockIndex) -> Option<PageIndex> {
        for page in (0..NUM_PAGES_PER_BLOCK).rev() {
            info!("Looking for dirty page at {}:{}", block_index, page);
            self.read_page(block_index, page).unwrap();
            self.read_page_metadata(block_index);
            self.wait_for_all_ready();
            if self.current_page.page_is_used() {
                return Some(page);
            }
        }
        None
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn read_from_cache_at_column_offset(
        &mut self,
        block: BlockIndex,
        offset: u16,
        length: Option<usize>,
    ) {
        let plane_and_offset_msb = (((block % 2) << 4) | offset >> 8) as u8;
        let current_page = self.current_page.take();
        {
            let spi_header = &mut current_page[..FLASH_SPI_HEADER];
            spi_header[0] = CACHE_READ;
            spi_header[1] = plane_and_offset_msb;
            spi_header[2] = (offset & 0xff) as u8;
            spi_header[3] = 0; // Dummy byte
        }
        let length =
            length.unwrap_or((FLASH_USER_PAGE_SIZE + FLASH_METADATA_SIZE) - offset as usize);
        let prev_page = self.prev_page.take();
        {
            self.cs.set_low().unwrap();
            // If the offset is 2048, we want the actual offset in our buffer to be 2052, then the
            // start is at 2048
            let offset = offset as usize;
            let src_range = 0..length + FLASH_SPI_HEADER;
            let dst_range = offset..offset + length + FLASH_SPI_HEADER;
            crate::assert_eq!(src_range.len(), dst_range.len());

            // FIXME: Why is this a bidirectional transfer again?
            //  That seems to be the main reason we need these Page abstractions.
            let transfer = bidirectional::Config::new(
                (
                    self.dma_channel_1.take().unwrap(),
                    self.dma_channel_2.take().unwrap(),
                ),
                unsafe { extend_lifetime(&current_page[src_range]) },
                self.spi.take().unwrap(),
                // To ensure the data is placed in the correct place on the page, offset by -4
                unsafe { extend_lifetime_mut(&mut prev_page[dst_range]) },
            )
            .start();
            let ((dma_ch1, dma_ch2), _from_buf, spi, _to_buf) = transfer.wait();
            // Do we also need some CRC checking?
            self.cs.set_high().unwrap();
            self.dma_channel_1 = Some(dma_ch1);
            self.dma_channel_2 = Some(dma_ch2);
            self.spi = Some(spi);
        }

        // Copy to the appropriate column offset:

        // Swap the buffers here.
        self.current_page.inner = Some(prev_page);
        self.prev_page.inner = Some(current_page);
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn read_event_from_cache_at_column_offset_spi(
        &mut self,
        block: BlockIndex,
        offset: u16,
    ) -> [u8; NUM_EVENT_BYTES] {
        let plane = ((block % 2) << 4) as u8;

        let plane_and_offset_msb = plane | ((offset >> 8) as u8);
        let mut bytes = [0u8; FLASH_SPI_HEADER + NUM_EVENT_BYTES];
        {
            let spi_header = &mut bytes[..FLASH_SPI_HEADER];
            spi_header[0] = CACHE_READ;
            spi_header[1] = plane_and_offset_msb;
            spi_header[2] = (offset & 0xff) as u8;
            spi_header[3] = 0; // dummy byte;
        }
        self.cs.set_low().unwrap();
        self.spi.as_mut().unwrap().transfer(&mut bytes).unwrap();
        self.cs.set_high().unwrap();
        let mut event = [0u8; NUM_EVENT_BYTES];
        event.copy_from_slice(&bytes[FLASH_SPI_HEADER..]);
        event
    }

    pub fn read_page_metadata(&mut self, block: BlockIndex) {
        #[allow(clippy::cast_possible_truncation)]
        self.read_from_cache_at_column_offset(block, FLASH_USER_PAGE_SIZE as u16, None);
    }

    pub fn read_page_from_cache(&mut self, block: BlockIndex) {
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

    fn write_enabled(&mut self) -> bool {
        let mut features: [u8; 3] = [GET_FEATURES, FEATURE_STATUS, 0x00];
        self.spi_transfer(&mut features);
        features[2] & 0b0000_0010 != 0
    }

    pub fn start_file(&mut self, start_page: PageIndex) -> BlockIndex {
        // CPTV always start writing a new file at page 1, reserving page 0 for when we come back
        // and write the header once we've finished writing the file.
        self.previous_file_start_block_index = self.file_start_block_index;
        self.current_page_index = start_page;

        warn!(
            "Starting file at {}:{}",
            self.current_block_index, self.current_page_index,
        );

        self.file_start_block_index = Some(self.current_block_index);
        self.current_block_index
    }

    pub fn append_recording_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        recording_file_type: RecordingFileType,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            false,
            None,
            None,
            false,
            recording_file_type,
            None,
        )
    }

    pub fn append_recording_bytes_at_location(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        block_index: Option<BlockIndex>,
        page_index: Option<PageIndex>,
        recording_file_type: RecordingFileType,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            false,
            block_index,
            page_index,
            false,
            recording_file_type,
            None,
        )
    }

    pub fn append_recording_bytes_at_location_with_time(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        block_index: Option<BlockIndex>,
        page_index: Option<PageIndex>,
        recording_file_type: RecordingFileType,
        time: Option<&SyncedDateTime>,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            false,
            block_index,
            page_index,
            false,
            recording_file_type,
            time,
        )
    }

    pub fn append_recording_bytes_with_time(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        recording_file_type: RecordingFileType,
        time: &SyncedDateTime,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            false,
            None,
            None,
            false,
            recording_file_type,
            Some(time),
        )
    }

    pub fn append_last_recording_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        recording_file_type: RecordingFileType,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            true,
            None,
            None,
            false,
            recording_file_type,
            None,
        )
    }

    pub fn append_last_recording_bytes_at_location(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        block_index: Option<BlockIndex>,
        page_index: Option<PageIndex>,
        recording_file_type: RecordingFileType,
    ) -> Result<(), &str> {
        self.append_file_bytes(
            bytes,
            user_bytes_length,
            true,
            block_index,
            page_index,
            false,
            recording_file_type,
            None,
        )
    }

    #[allow(clippy::too_many_lines)]
    fn append_file_bytes(
        &mut self,
        bytes: &mut [u8],
        user_bytes_length: usize,
        is_last: bool,
        block_index: Option<BlockIndex>,
        page_index: Option<PageIndex>,
        extended_write: bool,
        recording_file_type: RecordingFileType,
        time: Option<&SyncedDateTime>,
    ) -> Result<(), &str> {
        // NOTE: `extended_write` is set when we're using this function to write outside
        //  the regular user data, as when we set the device config

        self.write_enable();
        assert!(self.write_enabled());
        // Bytes will always be a full page + metadata + command info at the start
        assert_eq!(bytes.len(), FLASH_SPI_USER_PAYLOAD_SIZE); // 2116

        // Skip the first byte in the buffer
        let block = block_index.unwrap_or(self.current_block_index);
        let page = page_index.unwrap_or(self.current_page_index);
        if !extended_write && block > NUM_RECORDING_BLOCKS {
            return Err("Flash full");
        }
        let address = OnboardFlash::get_address(block, page);
        #[allow(clippy::cast_possible_truncation)]
        let plane = ((block % 2) << 4) as u8;
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc =
            crc_check.checksum(&bytes[FLASH_SPI_HEADER..FLASH_SPI_HEADER + user_bytes_length]);
        {
            // PROGRAM_LOAD only takes 3 bytes for its header, so we skip the first byte of our max
            // 4 byte header
            let spi_header = &mut bytes[..FLASH_SPI_HEADER];
            spi_header[1] = PROGRAM_LOAD;
            spi_header[2] = plane;
            spi_header[3] = 0;
        }
        {
            let user_metadata_bytes = &mut bytes[FLASH_SPI_HEADER..][0x820..=0x83f];
            //Now write into the user meta section
            user_metadata_bytes[0] = 0; // Page is used
            user_metadata_bytes[1] = if is_last { 0 } else { 0xff }; // Page is last page of file?
            {
                let space = &mut user_metadata_bytes[2..=3];
                #[allow(clippy::cast_possible_truncation)]
                LittleEndian::write_u16(space, user_bytes_length as u16);
            }
            // TODO Write user detected bad blocks into user-metadata section?
            let recording_type_bits = match recording_file_type {
                RecordingFileType::Cptv(RecordingFileTypeDetails {
                    user_requested,
                    shutdown_status,
                    startup_status,
                }) => {
                    if user_requested {
                        0b110
                    } else if startup_status {
                        0b101
                    } else if shutdown_status {
                        0b111
                    } else {
                        0b100
                    }
                }
                RecordingFileType::Audio(RecordingFileTypeDetails {
                    user_requested,
                    shutdown_status,
                    startup_status,
                }) =>
                {
                    #[allow(clippy::bool_to_int_with_if)]
                    if user_requested {
                        0b010
                    } else if startup_status {
                        0b001
                    } else if shutdown_status {
                        0b011
                    } else {
                        0b000
                    }
                }
                RecordingFileType::Other => {
                    // Do nothing for regular config or whatever files.
                    0xff
                }
            };
            // Write this twice.
            user_metadata_bytes[4] = recording_type_bits;
            user_metadata_bytes[5] = recording_type_bits;

            {
                let space = &mut user_metadata_bytes[6..=7];
                #[allow(clippy::cast_possible_truncation)]
                LittleEndian::write_u16(space, user_bytes_length as u16);
            }
            {
                let space = &mut user_metadata_bytes[8..=9];
                LittleEndian::write_u16(space, crc);
            }
            {
                let space = &mut user_metadata_bytes[10..=11];
                LittleEndian::write_u16(space, crc);
            }
            if !extended_write {
                //write file start block
                let space = &mut user_metadata_bytes[12..=13];
                LittleEndian::write_u16(space, self.file_start_block_index.unwrap());
                if let Some(previous_start) = self.previous_file_start_block_index {
                    //write previous file start could just write on first page if it matters
                    let space = &mut user_metadata_bytes[14..=15];
                    LittleEndian::write_u16(space, previous_start);
                }
            }
            if let Some(time) = time {
                // Write the timestamp when the file was finalised
                let space = &mut user_metadata_bytes[16..24];
                LittleEndian::write_i64(space, time.get_timestamp_micros());
            }
        }
        if is_last {
            if let RecordingFileType::Audio(RecordingFileTypeDetails {
                user_requested,
                startup_status,
                shutdown_status,
            }) = recording_file_type
            {
                if user_requested {
                    self.file_types_found.add_type(FileType::AudioUserRequested);
                } else if shutdown_status {
                    self.file_types_found.add_type(FileType::AudioShutdown);
                } else if startup_status {
                    self.file_types_found.add_type(FileType::AudioStartup);
                } else {
                    self.file_types_found.add_type(FileType::AudioScheduled);
                }
            }

            warn!("Ending file at {}:{}", block, page);
        }
        // PROGRAM_LOAD
        self.spi_write(&bytes[1..]);
        self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
        let status = self.wait_for_ready();

        // TODO: Check ECC status, mark and relocate block if needed.
        if status.program_failed() {
            // FIXME: Should we return an error here?
            error!("Programming failed");
        } else if !extended_write {
            if self.first_used_block_index.is_none() {
                self.first_used_block_index = Some(block);
            }
            if self.last_used_block_index.is_none() {
                self.last_used_block_index = Some(block);
            } else if let Some(last_used_block_index) = self.last_used_block_index {
                if last_used_block_index < block {
                    self.last_used_block_index = Some(block);
                }
            }
            if block_index.is_none() && page_index.is_none() {
                self.advance_file_cursor(is_last);
            }
        }
        if !status.erase_failed() {
            // Relocate earlier pages on this block to the next free block
            // Re-write this page
            // Erase and mark the earlier block as bad.
        }
        Ok(())
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn write_event(
        &mut self,
        event_bytes: &[u8; NUM_EVENT_BYTES],
        block_index: u16,
        page_index: u8,
        offset: u16,
    ) {
        if self.spi.is_some() {
            self.write_enable();
            assert!(self.write_enabled());
            let mut bytes = [0u8; FLASH_SPI_HEADER_SMALL + NUM_EVENT_BYTES];
            let address = OnboardFlash::get_address(block_index, page_index);
            let plane_and_offset_msb = (((block_index % 2) << 4) | offset >> 8) as u8;
            {
                let spi_header = &mut bytes[..FLASH_SPI_HEADER_SMALL];
                spi_header[0] = PROGRAM_LOAD;
                spi_header[1] = plane_and_offset_msb;
                spi_header[2] = (offset & 0xff) as u8;
            }
            bytes[FLASH_SPI_HEADER_SMALL..].copy_from_slice(event_bytes);
            self.spi_write(&bytes);
            self.spi_write(&[PROGRAM_EXECUTE, address[0], address[1], address[2]]);
            let status = self.wait_for_ready();
            if status.program_failed() {
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
