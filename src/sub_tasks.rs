use crate::attiny_rtc_i2c::MainI2C;
use crate::constants::FIRMWARE_VERSION;
use crate::device_config::DeviceConfig;
use crate::event_logger::{DiscardedRecordingInfo, Event, EventLogger, LoggerEvent};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::formatted_time::FormattedNZTime;
use crate::onboard_flash::{FilePartReturn, FileType, OnboardFlash};
use crate::re_exports::bsp::hal::Watchdog;
use crate::re_exports::bsp::pac::{DMA, RESETS};
use crate::re_exports::log::{info, unreachable, warn};
use crate::rpi_power::wake_raspberry_pi;
use crate::synced_date_time::SyncedDateTime;
use byteorder::{ByteOrder, LittleEndian};
use crc::{CRC_16_XMODEM, Crc};

pub fn maybe_offload_events(
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    events: &mut EventLogger,
    fs: &mut OnboardFlash,
    time: &SyncedDateTime,
    watchdog: &mut Watchdog,
) -> bool {
    let mut success = true;
    if events.has_events_to_offload(fs) {
        let event_indices = events.event_range();
        let total_events = event_indices.end;
        let mut events_sent = 0;
        warn!("Attempting to transfer {} events", total_events);
        'transfer_all_events: for event_index in event_indices {
            watchdog.feed();
            let event_bytes = EventLogger::get_event_at_index(event_index, fs);
            if let Some(event_bytes) = event_bytes {
                EventLogger::print_event(event_index as usize, &event_bytes);
                if let Some(spi) = fs.free_spi() {
                    pi_spi.enable(spi, resets);
                    let transfer_type = ExtTransferMessage::SendLoggerEvent;
                    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                    let current_crc = crc_check.checksum(&event_bytes);
                    let mut attempts = 0;
                    'transfer_event: loop {
                        let did_transfer = pi_spi.send_message_over_spi(
                            transfer_type,
                            &event_bytes,
                            current_crc,
                            dma,
                            None,
                        );
                        if did_transfer {
                            events_sent += 1;
                            if attempts > 0 {
                                warn!("File part took multiple attempts: {}", attempts);
                            }
                            break 'transfer_event;
                        }
                        attempts += 1;
                        if attempts > 100 {
                            warn!("Failed sending logger event to raspberry pi");
                            success = false;
                            break 'transfer_event;
                        }
                    }
                    if let Some(spi) = pi_spi.disable() {
                        fs.take_spi(spi, resets);
                    }
                    if !success {
                        break 'transfer_all_events;
                    }
                }
            }
        }
        info!("Offloaded {} event(s)", events_sent);
        if success {
            events.clear(fs);
            events.log(Event::OffloadedLogs, time, fs);
        } else {
            events.log(Event::LogOffloadFailed, time, fs);
        }
    }
    success
}

pub fn offload_latest_recording(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    i2c: &mut MainI2C,
    events: &mut EventLogger,
    time: &SyncedDateTime,
    watchdog: &mut Watchdog,
) -> bool {
    offload_recordings_and_events(fs, pi_spi, resets, dma, i2c, events, time, watchdog, true)
}

pub fn offload_all_recordings_and_events(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    i2c: &mut MainI2C,

    events: &mut EventLogger,
    time: &SyncedDateTime,
    watchdog: &mut Watchdog,
) -> bool {
    offload_recordings_and_events(fs, pi_spi, resets, dma, i2c, events, time, watchdog, false)
}

#[allow(clippy::too_many_lines)]
fn offload_recordings_and_events(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    i2c: &mut MainI2C,
    events: &mut EventLogger,
    time: &SyncedDateTime,
    watchdog: &mut Watchdog,
    only_last_file: bool,
) -> bool {
    // Fixme, make the return a result rather than a boolean

    warn!("There are files to offload!");
    let _wait_for_wake = wake_raspberry_pi(
        i2c,
        time.get_timer(),
        Some(watchdog),
        Some((fs, events, LoggerEvent::new(Event::GotRpiPoweredOn, time))),
    );
    if !only_last_file {
        maybe_offload_events(pi_spi, resets, dma, events, fs, time, watchdog);
    }

    let mut has_file = fs.begin_offload_reverse();

    if has_file {
        info!("Has file to offload");
    }

    if has_file && i2c.begin_offload().is_err() {
        warn!("Failed setting offload-in-progress flag");
    }

    if !has_file {
        return true;
    }

    // do some offloading.
    let mut file_count = 0;
    let mut success_transferring_parts_to_rpi: bool = true;
    let mut interrupted_by_user = false;
    'offload_all_files: while has_file {
        if let Ok(false) = i2c.offload_flag_is_set() {
            warn!("Offload interrupted by user");

            // FIXME: Log this?
            // We were interrupted by the rPi,
            success_transferring_parts_to_rpi = false;
            interrupted_by_user = true;
            break;
        }

        let mut file_start = true;
        let mut part_count = 0;
        let mut file_ended = false;
        let last_used_block_index = fs.last_used_block_index.unwrap_or(0);

        let mut current_file_metadata = None;

        // For speed of offloading, we read from the flash cache into one of the page buffers
        // held by the flash_storage.  Then we swap buffers and return the just read page
        // so that it can be immediately transferred via DMA to the raspberry pi.
        // Only, we're sharing the same SPI peripheral to do this, so while the transfer is happening
        // we can't be transferring more to the back buffer from the flash.  So what's the point
        // of having a complex double buffering system?
        if !interrupted_by_user {
            'offload_file: while let Some(file_part) = fs.get_file_part(events, time) {
                let FilePartReturn {
                    part,
                    crc16,
                    block,
                    page,
                    is_last_page_for_file,
                    spi,
                    metadata,
                    timestamp,
                } = file_part;
                watchdog.feed();
                pi_spi.enable(spi, resets);
                let transfer_type = if file_start && !is_last_page_for_file {
                    ExtTransferMessage::BeginFileTransfer
                } else if !file_start && !is_last_page_for_file {
                    ExtTransferMessage::ResumeFileTransfer
                } else if is_last_page_for_file {
                    ExtTransferMessage::EndFileTransfer
                } else if file_start && is_last_page_for_file {
                    ExtTransferMessage::BeginAndEndFileTransfer
                } else {
                    unreachable!("Invalid file transfer state");
                };

                let mut progress = None;
                if transfer_type == ExtTransferMessage::BeginFileTransfer {
                    // Each file start will give the max-used block
                    progress = Some(last_used_block_index);
                } else if transfer_type == ExtTransferMessage::EndFileTransfer {
                    progress = Some(block);
                }

                if file_start {
                    current_file_metadata = Some(metadata);
                    let cptv_recording = metadata == FileType::CptvStartup
                        || metadata == FileType::CptvShutdown
                        || metadata == FileType::CptvScheduled
                        || metadata == FileType::CptvUserRequested;
                    let audio_recording = metadata == FileType::AudioStartup
                        || metadata == FileType::AudioShutdown
                        || metadata == FileType::AudioUserRequested
                        || metadata == FileType::AudioScheduled;
                    if let Some(timestamp) = timestamp {
                        info!(
                            "First file part at {}:{}, written at {}",
                            block,
                            page,
                            FormattedNZTime(timestamp)
                        );
                    } else {
                        info!("First file part at {}:{}", block, page,);
                    }
                    if cptv_recording {
                        if metadata == FileType::CptvStartup {
                            info!("Offloading CPTV startup status recording");
                        } else if metadata == FileType::CptvShutdown {
                            info!("Offloading CPTV shutdown status recording");
                        } else if metadata == FileType::CptvUserRequested {
                            info!("Offloading CPTV tests recording");
                        } else {
                            info!("Offloading scheduled CPTV recording");
                        }
                    } else if audio_recording {
                        if metadata == FileType::AudioStartup {
                            info!("Offloading audio startup status recording");
                        } else if metadata == FileType::AudioShutdown {
                            info!("Offloading audio shutdown status recording");
                        } else if metadata == FileType::AudioUserRequested {
                            info!("Offloading tests audio recording");
                        } else {
                            info!("Offloading scheduled audio recording");
                        }
                    }
                }
                if is_last_page_for_file {
                    info!("Got last file part at {}:{}", block, page);
                }
                let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                let current_crc = crc_check.checksum(part);
                if current_crc != crc16 {
                    warn!(
                        "Data corrupted at part #{} ({}:{}) in transfer to or from flash memory",
                        part_count, block, page
                    );
                }

                let mut attempts = 0;
                'transfer_file_part: loop {
                    watchdog.feed();
                    let did_transfer = pi_spi.send_message_over_spi(
                        transfer_type,
                        part,
                        current_crc,
                        dma,
                        progress,
                    );
                    if did_transfer {
                        if attempts > 0 {
                            warn!("File part took multiple attempts: {}", attempts);
                        }
                        break 'transfer_file_part;
                    }
                    attempts += 1;
                    if attempts > 200 {
                        success_transferring_parts_to_rpi = false;
                        break 'transfer_file_part;
                    }
                }

                // Give spi peripheral back to flash storage.
                if let Some(spi) = pi_spi.disable() {
                    fs.take_spi(spi, resets);
                    if is_last_page_for_file && success_transferring_parts_to_rpi {
                        if let Some(file_type) = current_file_metadata {
                            events.log(Event::OffloadedRecording(file_type), time, fs);
                        } else {
                            events.log(Event::OffloadedRecording(FileType::Unknown), time, fs);
                        }
                    }
                }

                if !success_transferring_parts_to_rpi {
                    break 'offload_all_files;
                }

                part_count += 1;
                if is_last_page_for_file {
                    // NOTE: This could also be the *first* page of the file for
                    //  tests/status recordings.
                    file_count += 1;
                    info!("Offloaded {} file(s)", file_count);
                    watchdog.feed();
                    let _ = fs.erase_latest_file();
                    file_ended = true;
                    break 'offload_file;
                }
                file_start = false;
            }
        }

        if !file_ended && !interrupted_by_user && success_transferring_parts_to_rpi {
            info!(
                "Incomplete file at block {:?} erasing",
                fs.file_start_block_index
            );
            watchdog.feed();
            if fs.erase_latest_file().is_ok() {
                events.log(
                    Event::ErasePartialOrCorruptRecording(DiscardedRecordingInfo {
                        recording_type: current_file_metadata.unwrap_or(FileType::Unknown),
                        num_frames: 0,
                        seconds_since_last_ffc: 0,
                    }),
                    time,
                    fs,
                );
            }
        }
        if only_last_file {
            break;
        }

        has_file = fs.begin_offload_reverse();
    }

    if !interrupted_by_user && i2c.end_offload().is_err() {
        warn!("Failed un-setting offload-in-progress flag");
    }

    watchdog.feed();
    if interrupted_by_user {
        events.log(Event::FileOffloadInterruptedByUser, time, fs);
    }
    if success_transferring_parts_to_rpi {
        info!(
            "Completed file offload, transferred {:?} files start {:?} previous is {:?}",
            file_count, fs.file_start_block_index, fs.previous_file_start_block_index
        );
        // Always rescan to update what kinds of files we have
        fs.scan();
        file_count != 0
    } else {
        if !interrupted_by_user {
            events.log_if_not_dupe(Event::FileOffloadFailed, time, fs);
        }
        fs.scan();
        warn!("File transfer to pi failed");
        false
    }
}

/// Returns `Ok((Option<DeviceConfig>, true, _, _))` when config was updated
#[allow(clippy::too_many_lines)]
pub fn get_existing_device_config_or_config_from_pi_on_initial_handshake(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    existing_config: Option<DeviceConfig>,
    event_count: u16,
) -> Result<(Option<DeviceConfig>, bool, bool, bool), &'static str> {
    let mut payload = [0u8; 16];
    let mut config_was_updated = false;
    let mut prioritise_frame_preview = false;
    let mut force_offload_now = false;

    let num_files_to_offload = fs.num_files_in_initial_scan;
    let num_blocks_to_offload = fs.last_used_block_index.unwrap_or(0);
    let num_events_to_offload = event_count;
    let device_config_crc = match &existing_config {
        Some(existing_config) => existing_config.config().crc,
        None => 0,
    };

    if let Some(free_spi) = fs.free_spi() {
        pi_spi.enable(free_spi, resets);
        LittleEndian::write_u16(&mut payload[0..2], num_events_to_offload);
        LittleEndian::write_u32(&mut payload[2..6], FIRMWARE_VERSION);
        LittleEndian::write_u16(&mut payload[6..8], num_files_to_offload);
        LittleEndian::write_u16(&mut payload[8..10], num_blocks_to_offload);
        LittleEndian::write_u16(&mut payload[10..12], device_config_crc);

        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        info!("Sending startup handshake");
        if pi_spi.send_message_over_spi(
            ExtTransferMessage::StartupHandshake,
            &payload,
            crc,
            dma,
            None,
        ) {
            let new_config = if let Some(device_config) = pi_spi.return_payload() {
                // Check the device config against a CRC
                let crc = LittleEndian::read_u16(&device_config[4..6]);
                let crc_2 = LittleEndian::read_u16(&device_config[6..8]);
                if crc != crc_2 {
                    return Err("Provided CRCs don't match.");
                }
                let length = device_config[8];
                let length_2 = device_config[9];
                if length != length_2 {
                    return Err("Lengths of device config don't match.");
                }
                let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                let crc_calc = crc_check.checksum(&device_config[10..10 + usize::from(length)]);
                if crc_calc != crc {
                    return Err("New DeviceConfig CRC failure.");
                }

                let mut new_config = DeviceConfig::from_bytes(&device_config[4..]);
                let mut length_used = 0;
                if let Some(new_config) = &new_config {
                    length_used = new_config.cursor_position;
                    prioritise_frame_preview = device_config[4 + length_used] == 5;
                    force_offload_now = device_config[4 + length_used + 1] == 1;
                }

                let mut new_config_bytes = [0u8; 2400 + 112];
                new_config_bytes[0..length_used]
                    .copy_from_slice(&device_config[4..4 + length_used]);
                if let Some(new_config) = &mut new_config {
                    // Now get the motion detection mask which should take 24 transfers (since the return payload size is small)
                    for piece in 0..24 {
                        let payload = [piece];
                        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                        let crc = crc_check.checksum(&payload);
                        loop {
                            if pi_spi.send_message_over_spi(
                                ExtTransferMessage::GetMotionDetectionMask,
                                &payload,
                                crc,
                                dma,
                                None,
                            ) && let Some(piece_bytes) = pi_spi.return_payload()
                            {
                                let crc_from_remote = LittleEndian::read_u16(&piece_bytes[0..2]);
                                let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                                let length = piece_bytes[2];
                                let crc = crc_check.checksum(&piece_bytes[2..=2 + length as usize]);
                                if crc == crc_from_remote {
                                    new_config
                                        .motion_detection_mask
                                        .append_piece(&piece_bytes[4..4 + length as usize]);
                                    break;
                                }
                                warn!(
                                    "crc failed for mask piece {}, re-requesting ({:?})",
                                    piece,
                                    &piece_bytes[0..10]
                                );
                            }
                        }
                    }

                    info!("Got config from rPi");
                    if existing_config.is_none()
                        || *new_config != *existing_config.as_ref().unwrap()
                    {
                        if existing_config.is_some() {
                            warn!("Config has changed: {:?}", new_config.config());
                        }
                        new_config_bytes[length_used..length_used + 2400]
                            .copy_from_slice(&new_config.motion_detection_mask.inner);
                        let slice_to_write = &mut new_config_bytes[0..length_used + 2400];
                        if let Some(spi_free) = pi_spi.disable() {
                            fs.take_spi(spi_free, resets);
                        }
                        fs.write_device_config(slice_to_write);
                        new_config.cursor_position += 2400;
                        config_was_updated = true;
                    }
                }
                (
                    new_config,
                    config_was_updated,
                    prioritise_frame_preview,
                    force_offload_now,
                )
            } else {
                warn!("Pi did not respond");
                (
                    existing_config,
                    config_was_updated,
                    prioritise_frame_preview,
                    force_offload_now,
                )
            };
            if let Some(spi_free) = pi_spi.disable() {
                fs.take_spi(spi_free, resets);
            }
            Ok(new_config)
        } else {
            if let Some(spi_free) = pi_spi.disable() {
                fs.take_spi(spi_free, resets);
            }
            Ok((
                existing_config,
                config_was_updated,
                prioritise_frame_preview,
                force_offload_now,
            ))
        }
    } else {
        warn!("Flash spi not enabled");
        if let Some(existing_config) = &existing_config {
            info!(
                "Got device config {:?}, {}",
                existing_config,
                existing_config.device_name()
            );
        }
        Ok((
            existing_config,
            config_was_updated,
            prioritise_frame_preview,
            force_offload_now,
        ))
    }
}
