extern crate std;
use crate::byte_slice_cursor::Cursor;
use crate::ext_spi_transfers::{RPI_RETURN_PAYLOAD_LENGTH, RPI_TRANSFER_HEADER_LENGTH};
use crate::onboard_flash::{
    BLOCK_ERASE, BlockIndex, CACHE_READ, FEATURE_STATUS, FLASH_SPI_HEADER, FLASH_SPI_HEADER_SMALL,
    GET_FEATURES, OnboardFlash, PAGE_READ, PROGRAM_EXECUTE, PROGRAM_LOAD, PageIndex, RESET,
    SET_FEATURES, WRITE_ENABLE,
};
use crate::tests::stubs::fake_rpi_event_logger::{
    DiscardedRecordingInfo, FileType, LoggerEvent, LoggerEventKind, NewConfigInfo, WakeReason,
};
use crate::tests::stubs::fake_rpi_recording_state::RecordingMode;
use crate::tests::test_state::test_global_state::{
    CURRENT_TIME, DEVICE_CONFIG, ECC_ERROR_ADDRESSES, EVENTS_OFFLOADED, EventOffload,
    FAKE_PI_RECORDING_STATE, FILE_DOWNLOAD, FILE_DOWNLOAD_START, FILE_PART_COUNT, FILES_OFFLOADED,
    FLASH_BACKING_STORAGE, FileOffload, PENDING_FORCED_OFFLOAD_REQUEST,
    PREFER_NOT_TO_OFFLOAD_FILES_NOW,
};
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Utc};
use chrono_tz::Tz::Pacific__Auckland;
use codec::decode::CptvDecoder;
use crc::{CRC_16_XMODEM, Crc};
use log::{debug, error, info, warn};
use std::ops::Not;
use std::string::String;
use std::sync::{LazyLock, Mutex};
use std::time::Instant;
use std::vec::Vec;
use std::{format, fs, println};

pub struct SpiEnabledPeripheral {
    buffer: Vec<u8>,
    last_read_address: (BlockIndex, PageIndex),
    last_page_offset: usize,
}

#[derive(Clone, Copy)]
pub struct StoragePage {
    pub inner: [u8; 2048 + 128],
}
#[derive(Clone)]
pub struct StorageBlock {
    pub inner: [StoragePage; 64],
}

pub const CAMERA_CONNECT_INFO: u8 = 0x1;
pub const CAMERA_RAW_FRAME_TRANSFER: u8 = 0x2;
pub const CAMERA_BEGIN_FILE_TRANSFER: u8 = 0x3;
pub const CAMERA_RESUME_FILE_TRANSFER: u8 = 0x4;
pub const CAMERA_END_FILE_TRANSFER: u8 = 0x5;
pub const CAMERA_BEGIN_AND_END_FILE_TRANSFER: u8 = 0x6;
pub const CAMERA_GET_MOTION_DETECTION_MASK: u8 = 0x7;
pub const CAMERA_SEND_LOGGER_EVENT: u8 = 0x8;
pub const CAMERA_STARTUP_HANDSHAKE: u8 = 0x9;

static NEXT_RPI_RESPONSE: LazyLock<Mutex<[u8; RPI_RETURN_PAYLOAD_LENGTH]>> =
    LazyLock::new(|| Mutex::new([0; RPI_RETURN_PAYLOAD_LENGTH]));

pub const EXPECTED_RP2040_FIRMWARE_VERSION: u32 = 36;

impl SpiEnabledPeripheral {
    pub fn reset(&mut self) {
        self.buffer.drain(..);
        self.last_read_address = (0, 0);
    }

    pub fn new() -> SpiEnabledPeripheral {
        SpiEnabledPeripheral {
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }

    fn get_block_and_page_from_address(address_bytes: [u8; 3]) -> (BlockIndex, PageIndex) {
        // Reconstruct the 32-bit address from the 3 bytes
        let address: u32 = (u32::from(address_bytes[0]) << 16)
            | (u32::from(address_bytes[1]) << 8)
            | u32::from(address_bytes[2]);

        // Extract page from lower 6 bits
        let page = (address & 0x3F) as PageIndex; // 0x3F = 0b111111 (6 bits)

        // Extract block from upper bits (shift right by 6)
        let block = (address >> 6) as BlockIndex;

        (block, page)
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<(), ()> {
        match bytes[0] {
            PROGRAM_LOAD => {
                // Set the plane to write some bytes to, nop
                // FIXME: Load needs to also handle offsets into the page
                let offset_in_page: u16 = (bytes[1] as u16 & 0x0f) << 8 | bytes[2] as u16;
                self.last_page_offset = offset_in_page as usize;
                self.buffer.drain(..);
                self.buffer
                    .extend_from_slice(&bytes[FLASH_SPI_HEADER - 1..]);
            }
            PROGRAM_EXECUTE => {
                // 11 bits for block, 6 bits for page
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);

                // NOTE: was 2112 instead of buffer.len()
                let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                storage[block_index as usize].inner[page_index as usize].inner
                    [self.last_page_offset..self.last_page_offset + self.buffer.len()]
                    .copy_from_slice(&self.buffer);
            }
            PAGE_READ => {
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);
                self.last_read_address = (block_index, page_index);
                self.buffer.drain(..);
                let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                self.buffer.extend_from_slice(
                    &storage[block_index as usize].inner[page_index as usize].inner,
                );

                let ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                if ecc_error_addresses.contains(&self.last_read_address) {
                    //println!("Corrupting buffer");
                    self.buffer[1] = 0x42;
                }
            }
            RESET => {
                self.buffer.drain(..);
            }
            WRITE_ENABLE => {
                // Do nothing
            }
            SET_FEATURES => {
                // Do nothing
            }
            BLOCK_ERASE => {
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, _page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);
                let mut ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                ecc_error_addresses.retain(|(b, p)| *b != block_index);
                self.last_read_address = (0, 0);
                let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                for page in &mut storage[block_index as usize].inner {
                    page.inner.fill(0xff);
                }
            }
            _ => panic!("Unhandled command 0x{:02x?}", bytes[0]),
        }
        Ok(())
    }

    pub fn transfer(&mut self, dst: &mut [u8], src: Option<&[u8]>) -> Result<(), ()> {
        if let Some(src) = src
            && src[0] == CACHE_READ
        {
            let col_offset_0 = src[1] as u16;
            let col_offset_1 = src[2] as u16;

            let col_offset = ((col_offset_0 & 0x0f) << 8 | col_offset_1) as usize;
            assert_eq!(dst.len(), src.len());
            let len = col_offset..col_offset + src.len() - FLASH_SPI_HEADER;
            dst[FLASH_SPI_HEADER..].copy_from_slice(&self.buffer[len]);
        } else {
            match dst[0] {
                CACHE_READ => {
                    let col_offset_0 = dst[1] as u16;
                    let col_offset_1 = dst[2] as u16;
                    let col_offset = ((col_offset_0 & 0x0f) << 8 | col_offset_1) as usize;
                    let len = col_offset..col_offset + dst.len() - FLASH_SPI_HEADER;
                    dst[FLASH_SPI_HEADER..].copy_from_slice(&self.buffer[len]);
                }
                GET_FEATURES => {
                    let feature_type = dst[1];
                    match feature_type {
                        FEATURE_STATUS => {
                            let ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                            if ecc_error_addresses.contains(&self.last_read_address) {
                                println!("ECC Error @ {:?}", self.last_read_address);
                                dst[2] = 0b0010_0010;
                            } else {
                                dst[2] = 0b0000_0010;
                            }
                        }
                        _ => error!("Unhandled feature_type {:?}", feature_type),
                    }
                }
                _ => error!("unhandled command 0x{:02x?}", dst[0]),
            }
        }
        Ok(())
    }

    pub fn disable(self) -> SpiEnabledPeripheral {
        self
    }
}

pub fn write_to_rpi(bytes: &[u8]) -> Result<(), ()> {
    let mut config_crc = 0;
    let mut config_crc_from_rp2040 = 0;
    let mut radiometry_enabled = false;
    let mut lepton_serial_number = String::new();
    let mut got_startup_info = false;
    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
    let header_slice = &bytes[..RPI_TRANSFER_HEADER_LENGTH];
    let transfer_type = header_slice[0];
    let transfer_type_dup = header_slice[1];
    let mut num_bytes = LittleEndian::read_u32(&header_slice[2..6]) as usize;
    let num_bytes_dup = LittleEndian::read_u32(&header_slice[6..10]) as usize;
    let crc_from_remote = LittleEndian::read_u16(&header_slice[10..12]);
    let crc_from_remote_dup = LittleEndian::read_u16(&header_slice[12..14]);
    let crc_from_remote_inv = LittleEndian::read_u16(&header_slice[14..16]);
    let crc_from_remote_inv_dup = LittleEndian::read_u16(&header_slice[16..=17]);
    let transfer_type_check = transfer_type == transfer_type_dup;
    let device_config = DEVICE_CONFIG
        .lock()
        .unwrap()
        .as_ref()
        .expect("No tests device config set")
        .clone();

    {
        let mut return_payload_buf = NEXT_RPI_RESPONSE.lock().unwrap();
        // This sequence is used to synchronise the return payload start on the rp2040, since
        // it seems to have a fair bit of slop/offsetting.
        return_payload_buf[0..4].copy_from_slice(&[1, 2, 3, 4]);
    }

    let mut transfer_block = 0;
    let is_file_transfer_message =
        (CAMERA_BEGIN_FILE_TRANSFER..=CAMERA_BEGIN_AND_END_FILE_TRANSFER).contains(&transfer_type);
    let is_file_transfer_progress_message = transfer_type_check
        && (transfer_type == CAMERA_BEGIN_FILE_TRANSFER
            || transfer_type == CAMERA_END_FILE_TRANSFER);
    let header_crc_check = if is_file_transfer_progress_message {
        transfer_block = crc_from_remote_dup;

        crc_from_remote_inv_dup == crc_from_remote_inv
            && crc_from_remote_inv.not() == crc_from_remote
    } else {
        crc_from_remote == crc_from_remote_dup
            && crc_from_remote_inv_dup == crc_from_remote_inv
            && crc_from_remote_inv.not() == crc_from_remote
    };
    let num_bytes_check = num_bytes == num_bytes_dup;
    if !num_bytes_check || !header_crc_check || !transfer_type_check {
        warn!("rpi received invalid message");
        let mut return_payload_buf = NEXT_RPI_RESPONSE.lock().unwrap();
        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
        // spi.write(&return_payload_buf).unwrap();
    }
    if num_bytes == 0 {
        // warn!("zero-sized payload");
        let mut return_payload_buf = NEXT_RPI_RESPONSE.lock().unwrap();
        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
        // spi.write(&return_payload_buf).unwrap();
    }
    if !(CAMERA_CONNECT_INFO..=CAMERA_STARTUP_HANDSHAKE).contains(&transfer_type) {
        // Unknown transfer type
        warn!("rpi received unknown message type");
        let mut return_payload_buf = NEXT_RPI_RESPONSE.lock().unwrap();
        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
        // spi.write(&return_payload_buf).unwrap();
    }

    if transfer_type != CAMERA_RAW_FRAME_TRANSFER {
        let chunk = &bytes[RPI_TRANSFER_HEADER_LENGTH..RPI_TRANSFER_HEADER_LENGTH + num_bytes];
        // Write back the crc we calculated.
        let crc = crc_check.checksum(chunk);
        let mut return_payload_buf = NEXT_RPI_RESPONSE.lock().unwrap();
        LittleEndian::write_u16(&mut return_payload_buf[4..6], crc);
        LittleEndian::write_u16(&mut return_payload_buf[6..8], crc);

        if transfer_type == CAMERA_STARTUP_HANDSHAKE {
            debug!("rPi Got camera startup handshake {chunk:?}");
            // Write all the info we need about the device:
            let force_offload_files_now = PENDING_FORCED_OFFLOAD_REQUEST.lock().unwrap();
            let prefer_not_to_offload_files_now = PREFER_NOT_TO_OFFLOAD_FILES_NOW.lock().unwrap();

            let config_length = device_config.write_to_slice(
                &mut return_payload_buf[14..],
                *prefer_not_to_offload_files_now,
                *force_offload_files_now,
            );
            config_crc =
                crc_check.checksum(&return_payload_buf[14..14 + usize::from(config_length)]);

            LittleEndian::write_u16(&mut return_payload_buf[8..10], config_crc);
            LittleEndian::write_u16(&mut return_payload_buf[10..12], config_crc);
            return_payload_buf[12] = config_length;
            return_payload_buf[13] = config_length;
            debug!("Sending camera device config to rp2040");
        } else if transfer_type == CAMERA_GET_MOTION_DETECTION_MASK {
            let piece_number = &chunk[0];
            let piece = device_config.mask_piece(*piece_number as usize);

            let mut piece_with_length = [0u8; 101];
            piece_with_length[0] = piece.len() as u8;
            piece_with_length[1..1 + piece.len()].copy_from_slice(piece);
            let crc = crc_check.checksum(&piece_with_length[0..1 + piece.len()]);
            // info!("Sending camera mask config piece {piece_number}, crc {crc}");
            LittleEndian::write_u16(&mut return_payload_buf[8..10], crc);
            return_payload_buf[10..10 + piece.len() + 1].copy_from_slice(&piece_with_length);
        }
        // Always write the return buffer
        // spi.write(&return_payload_buf).unwrap();

        if crc == crc_from_remote {
            let mut recording_state = FAKE_PI_RECORDING_STATE.lock().unwrap();
            if is_file_transfer_progress_message {
                recording_state.update_offload_progress(transfer_block);
            } else if !is_file_transfer_message && recording_state.is_offloading() {
                recording_state.end_offload();
            }

            match transfer_type {
                CAMERA_STARTUP_HANDSHAKE => {
                    let firmware_version = LittleEndian::read_u32(&chunk[2..6]);

                    let num_files_to_offload = LittleEndian::read_u16(&chunk[6..8]);
                    let num_blocks_to_offload = LittleEndian::read_u16(&chunk[8..10]);
                    let num_events_to_offload = LittleEndian::read_u16(&chunk[0..2]);
                    config_crc_from_rp2040 = LittleEndian::read_u16(&chunk[10..12]);

                    recording_state.set_offload_totals(
                        num_files_to_offload,
                        num_blocks_to_offload,
                        num_events_to_offload,
                    );

                    if firmware_version != EXPECTED_RP2040_FIRMWARE_VERSION {
                        panic!(
                            "Unsupported firmware version, expected \
                                        {EXPECTED_RP2040_FIRMWARE_VERSION}, got \
                                        {firmware_version}. Will reprogram RP2040."
                        );
                    }
                    info!(
                        "rPi Got startup handshake: \
                                        firmware version: {firmware_version}"
                    );
                    let estimated_offload_mb = num_blocks_to_offload as f32 * 128.0 / 1024.0;
                    let estimated_offload_time_seconds = (estimated_offload_mb * 2.0) / 60.0;
                    if num_files_to_offload > 0 && num_events_to_offload > 0 {
                        info!(
                            "{num_files_to_offload} file(s) to offload \
                                            totalling {estimated_offload_mb}MB \
                                            estimated offload time {estimated_offload_time_seconds:.2} mins. \
                                            Events to offload {num_events_to_offload}"
                        );
                    } else if num_events_to_offload > 0 {
                        info!("rpi got events to offload {num_events_to_offload}");
                    }
                    // Terminate any existing file download.
                    let in_progress_file_transfer = FILE_DOWNLOAD.lock().unwrap().take();
                    if let Some(file) = in_progress_file_transfer {
                        warn!(
                            "Aborting in progress file transfer with {} bytes",
                            file.len()
                        );
                    }
                }
                CAMERA_CONNECT_INFO => {
                    if config_crc_from_rp2040 != config_crc {
                        panic!(
                            "RP2040 device config didn't match the one we sent it, forcing restart ({config_crc_from_rp2040} vs {config_crc})"
                        );
                    }

                    let recording_mode = if LittleEndian::read_u32(&chunk[12..16]) != 0 {
                        RecordingMode::Audio
                    } else {
                        RecordingMode::Thermal
                    };
                    recording_state.set_mode(recording_mode);
                    if recording_mode == RecordingMode::Thermal {
                        radiometry_enabled = LittleEndian::read_u32(&chunk[0..4]) == 2;
                        lepton_serial_number = format!("{}", LittleEndian::read_u32(&chunk[8..12]));
                        got_startup_info = true;
                        info!(
                            "Got thermal startup info: \
                                        radiometry enabled: {radiometry_enabled}, \
                                        lepton serial #{lepton_serial_number} \
                                        recording mode {recording_mode:?}"
                        );
                    } else {
                        info!("Got audio mode startup");
                    }
                    if device_config.use_low_power_mode()
                        && !radiometry_enabled
                        && !device_config.is_audio_device()
                    {
                        error!(
                            "Low power mode is currently only supported on \
                                        lepton sensors with radiometry or audio device, exiting."
                        );
                        panic!("Exit");
                    }
                    // Terminate any existing file download.
                    let in_progress_file_transfer = FILE_DOWNLOAD.lock().unwrap().take();
                    if let Some(file) = in_progress_file_transfer {
                        warn!(
                            "Aborting in progress file transfer with {} bytes",
                            file.len()
                        );
                    }
                }
                CAMERA_SEND_LOGGER_EVENT => {
                    let event_kind = LittleEndian::read_u16(&chunk[0..2]);
                    let mut event_timestamp = LittleEndian::read_i64(&chunk[2..10]);
                    let payload_bytes = &chunk[10..18];
                    let event_payload = LittleEndian::read_u64(payload_bytes);
                    if let Ok(mut event_kind) = LoggerEventKind::try_from(event_kind) {
                        recording_state.completed_event_offload();
                        if let Some(mut time) = DateTime::from_timestamp_micros(event_timestamp) {
                            if let LoggerEventKind::SetAudioAlarm(alarm_time) = &mut event_kind {
                                if DateTime::from_timestamp_micros(event_payload as i64).is_some() {
                                    *alarm_time = event_payload as i64;
                                } else {
                                    warn!("Wakeup alarm from event was invalid {event_payload}");
                                }
                            } else if let LoggerEventKind::SetThermalAlarm(alarm_time) =
                                &mut event_kind
                            {
                                if DateTime::from_timestamp_micros(event_payload as i64).is_some() {
                                    *alarm_time = event_payload as i64;
                                } else {
                                    warn!("Wakeup alarm from event was invalid {event_payload}");
                                }
                            } else if let LoggerEventKind::Rp2040MissedAudioAlarm(alarm_time) =
                                &mut event_kind
                            {
                                if DateTime::from_timestamp_micros(event_payload as i64).is_some() {
                                    *alarm_time = event_payload as i64;
                                } else {
                                    warn!("Missed alarm from event was invalid {event_payload}");
                                }
                            } else if let LoggerEventKind::ToldRpiToWake(reason) = &mut event_kind {
                                if let Ok(wake_reason) = WakeReason::try_from(event_payload as u8) {
                                    *reason = wake_reason;
                                } else {
                                    warn!("Told rpi to wake invalid reason {event_payload}");
                                }
                            } else if matches!(
                                event_kind,
                                LoggerEventKind::RtcCommError | LoggerEventKind::RtcVoltageLowError
                            ) {
                                if event_timestamp == 0 {
                                    time = chrono::Local::now().with_timezone(&Utc);
                                    event_timestamp = time.timestamp_micros();
                                }
                            } else if let LoggerEventKind::LostFrames(lost_frames) = &mut event_kind
                            {
                                *lost_frames = event_payload;
                            } else if let LoggerEventKind::ErasePartialOrCorruptRecording(
                                discard_info,
                            ) = &mut event_kind
                            {
                                *discard_info = DiscardedRecordingInfo::from_bytes(payload_bytes);
                            } else if let LoggerEventKind::WouldDiscardAsFalsePositive(
                                discard_info,
                            ) = &mut event_kind
                            {
                                *discard_info = DiscardedRecordingInfo::from_bytes(payload_bytes);
                            } else if let LoggerEventKind::Rp2040GotNewConfig(new_config_info) =
                                &mut event_kind
                            {
                                *new_config_info = NewConfigInfo::from_bytes(payload_bytes);
                            } else if let LoggerEventKind::UnrecoverableDataCorruption(location) =
                                &mut event_kind
                            {
                                location.0 = LittleEndian::read_u16(&payload_bytes[0..=1]);
                                location.1 = LittleEndian::read_u16(&payload_bytes[2..=3]);
                            } else if let LoggerEventKind::OffloadedRecording(file_type) =
                                &mut event_kind
                            {
                                *file_type = FileType::from(payload_bytes[0]);
                            }
                            let payload_json = if let LoggerEventKind::SavedNewConfig = event_kind {
                                // If we get saved new config, the rp2040 would have just been
                                // restarted after the config change, so we can log the current
                                // config in relation to that event.
                                let json_inner = format!(
                                    r#""continuous-recorder": {},
                                                        "use-low-power-mode": {},
                                                        "start-recording": "{:?}",
                                                        "stop-recording": "{:?}",
                                                        "location": "({}, {}, {})"
                                                        "#,
                                    device_config.is_continuous_recorder(),
                                    device_config.use_low_power_mode(),
                                    device_config.recording_window().0,
                                    device_config.recording_window().1,
                                    device_config.lat_lng().0,
                                    device_config.lat_lng().1,
                                    device_config.location_altitude().unwrap_or(0.0)
                                );
                                let json = format!("{{{json_inner}}}");
                                Some(json)
                            } else {
                                None
                            };
                            info!(
                                "Got logger event {:?} at {}",
                                event_kind,
                                time.with_timezone(&Pacific__Auckland)
                            );
                            let event = LoggerEvent::new(event_kind, event_timestamp);
                            event.log(payload_json);
                            EVENTS_OFFLOADED.lock().unwrap().push(EventOffload {
                                event,
                                offloaded_at: CURRENT_TIME.lock().unwrap().clone(),
                            })
                        } else {
                            warn!("Event had invalid timestamp {event_timestamp}");
                        }
                    } else {
                        warn!("Unknown logger event kind {event_kind}");
                    }
                }
                CAMERA_BEGIN_FILE_TRANSFER => {
                    if FILE_DOWNLOAD.lock().unwrap().is_some() {
                        warn!("Trying to begin file without ending current");
                        *FILE_PART_COUNT.lock().unwrap() = 0;
                    }
                    info!("Begin file transfer");
                    // Open new file transfer
                    *FILE_PART_COUNT.lock().unwrap() += 1;
                    // If we have to grow this Vec once it gets big it can be slow and
                    // interrupt the transfer, so pre-allocate to a high-water mark.
                    let mut file = Vec::with_capacity(50_000_000);
                    file.extend_from_slice(chunk);
                    *FILE_DOWNLOAD.lock().unwrap() = Some(file);
                }
                CAMERA_RESUME_FILE_TRANSFER => {
                    if let Some(file) = &mut **FILE_DOWNLOAD.lock().as_mut().unwrap() {
                        // Continue current file transfer
                        //println!("Continue file transfer");
                        {
                            let part_count = *FILE_PART_COUNT.lock().unwrap();
                            if part_count % 100 == 0 {
                                let start = *FILE_DOWNLOAD_START.lock().unwrap();
                                let megabytes_per_second = (file.len() + chunk.len()) as f32
                                    / Instant::now().duration_since(start).as_secs_f32()
                                    / (1024.0 * 1024.0);
                                debug!(
                                    "Transferring part #{part_count} {:?} for {} bytes, {megabytes_per_second}MB/s",
                                    Instant::now().duration_since(start),
                                    file.len() + chunk.len(),
                                );
                            }
                        }
                        *FILE_PART_COUNT.lock().unwrap() += 1;
                        file.extend_from_slice(chunk);
                    } else {
                        // FIXME: This might actually break forced offload.
                        warn!("Trying to continue file with no open file");
                        if !got_startup_info && recording_state.safe_to_restart_rp2040() {
                            let date = chrono::Local::now();
                            error!(
                                "1) Requesting reset of rp2040 to \
                                                force handshake, {}",
                                date.format("%Y-%m-%d--%H:%M:%S")
                            );
                            panic!("Pi restart rp2040");
                        }
                    }
                }
                CAMERA_END_FILE_TRANSFER => {
                    // End current file transfer
                    if FILE_DOWNLOAD.lock().unwrap().is_none() {
                        warn!("Trying to end file with no open file");
                    }
                    if let Some(mut file) = FILE_DOWNLOAD.lock().unwrap().take() {
                        // Continue current file transfer
                        let start = *FILE_DOWNLOAD_START.lock().unwrap();
                        let megabytes_per_second = (file.len() + chunk.len()) as f32
                            / Instant::now().duration_since(start).as_secs_f32()
                            / (1024.0 * 1024.0);
                        info!(
                            "End file transfer, took {:?} for {} bytes, {megabytes_per_second}MB/s",
                            Instant::now().duration_since(start),
                            file.len() + chunk.len(),
                        );
                        *FILE_PART_COUNT.lock().unwrap() = 0;
                        //needs_to_offload_test_recording = false;
                        file.extend_from_slice(chunk);
                        recording_state.completed_file_offload();
                        let shebang = LittleEndian::read_u16(&file[0..2]);
                        if shebang == 1 {
                            //save_audio_file_to_disk(file, device_config.clone());
                            info!("Saving audio file {}", file.len());
                            FILES_OFFLOADED.lock().unwrap().push(FileOffload {
                                size: file.len(),
                                file_type: FileType::AudioScheduled,
                                offloaded_at: CURRENT_TIME.lock().unwrap().clone(),
                            });
                        } else {
                            //save_cptv_file_to_disk(file, device_config.output_dir())
                            let mut files_offloaded = FILES_OFFLOADED.lock().unwrap();
                            let file_num = files_offloaded.len();

                            // Decode cptv file:
                            let mut decoder =
                                CptvDecoder::from(std::io::Cursor::new(&file)).unwrap();
                            let header = decoder.get_header().unwrap();
                            info!("CPTV file header: {:#?}", header);
                            let cptv_recording_type = if let Some(status) = header.motion_config {
                                match status.as_string().as_ref() {
                                    "status: shutdown" => FileType::CptvShutdown,
                                    "status: startup" => FileType::CptvStartup,
                                    "test: true" => FileType::CptvUserRequested,
                                    _ => FileType::CptvScheduled,
                                }
                            } else {
                                FileType::CptvScheduled
                            };
                            // fs::write(format!("CPTV_{}.cptv", file_num), &file).unwrap();
                            info!("Saving cptv file {}", file.len());
                            files_offloaded.push(FileOffload {
                                size: file.len(),
                                file_type: cptv_recording_type,
                                offloaded_at: CURRENT_TIME.lock().unwrap().clone(),
                            });
                        }
                    } else {
                        warn!("Trying to end file with no open file");
                    }
                }
                CAMERA_BEGIN_AND_END_FILE_TRANSFER => {
                    if FILE_DOWNLOAD.lock().unwrap().is_some() {
                        info!("Trying to begin (and end) file without ending current");
                    }
                    // Open and end new file transfer
                    *FILE_PART_COUNT.lock().unwrap() = 0;
                    //needs_to_offload_test_recording = false;
                    let mut file = Vec::new();
                    file.extend_from_slice(chunk);
                    let shebang = LittleEndian::read_u16(&file[0..2]);
                    if shebang == 1 {
                        //save_audio_file_to_disk(file, device_config.clone());
                        info!("Saving audio file {}", file.len());
                        FILES_OFFLOADED.lock().unwrap().push(FileOffload {
                            size: file.len(),
                            file_type: FileType::AudioScheduled,
                            offloaded_at: CURRENT_TIME.lock().unwrap().clone(),
                        });
                    } else {
                        //save_cptv_file_to_disk(file, device_config.output_dir())
                        info!("Saving cptv file {}", file.len());
                        FILES_OFFLOADED.lock().unwrap().push(FileOffload {
                            size: file.len(),
                            file_type: FileType::CptvScheduled,
                            offloaded_at: CURRENT_TIME.lock().unwrap().clone(),
                        });
                    }
                }
                CAMERA_GET_MOTION_DETECTION_MASK => {
                    // Already handled
                }
                _ => {
                    if num_bytes != 0 {
                        warn!("Unhandled transfer type, {transfer_type:#x}")
                    }
                }
            }
        } else {
            warn!("Crc check failed, remote was notified and will re-transmit");
        }
    }
    Ok(())
}
pub fn read_from_rpi(dst: &mut [u8]) -> Result<(), ()> {
    let response = NEXT_RPI_RESPONSE.lock().unwrap();
    dst.copy_from_slice(&response[..]);
    Ok(())
}
