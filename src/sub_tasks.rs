use crate::FIRMWARE_VERSION;
use crate::attiny_rtc_i2c::MainI2C;
use crate::bsp;
use crate::bsp::pac::{DMA, RESETS};
use crate::device_config::DeviceConfig;
use crate::event_logger::{Event, EventLogger, LoggerEvent};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::onboard_flash::{FilePartReturn, OnboardFlash};
use crate::rpi_power::wake_raspberry_pi;
use crate::synced_date_time::SyncedDateTime;
use bsp::hal::Watchdog;
use byteorder::{ByteOrder, LittleEndian};
use crc::{CRC_16_XMODEM, Crc};
use defmt::{info, unreachable, warn};
use fugit::HertzU32;

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
    if events.has_events_to_offload() {
        let event_indices = events.event_range();
        let total_events = event_indices.end;
        let mut transferred_events = 0;
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
                            resets,
                            None, // FIXME: Do we want progress here?
                        );
                        if did_transfer {
                            transferred_events += 1;
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
        info!("Offloaded {} event(s)", total_events);
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

    // do some offloading.
    let mut file_count = 0;
    let mut success: bool = true;
    // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
    //  Probably doesn't matter though.

    let mut interrupted_by_user = false;
    while has_file {
        if let Ok(offload_flag_set) = i2c.offload_flag_is_set() {
            if !offload_flag_set {
                warn!("Offload interrupted by user");
                // We were interrupted by the rPi,
                interrupted_by_user = true;
                break;
            }
        }

        let mut file_start = true;
        let mut part_count = 0;
        let mut file_ended = false;
        let last_used_block_index = fs.last_used_block_index.unwrap_or(0);

        // For speed of offloading, we read from flash cache into a one of the page buffers
        // held by the flash_storage.  Then we swap buffers and return the just read page,
        // so that it can be immediately transferred via DMA to the raspberry pi.
        // Only, we're sharing the same SPI peripheral to do this, so while the transfer is happening
        // we can't be transferring more to the back buffer from the flash.  So what's the point
        // of having a complex double buffering system?
        'outer: while let Some(file_part) = fs.get_file_part() {
            let FilePartReturn {
                part,
                crc16,
                block,
                page,
                is_last_page_for_file,
                spi,
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
                // Each file start will give the max used block
                progress = Some(last_used_block_index);
            } else if transfer_type == ExtTransferMessage::EndFileTransfer {
                progress = Some(block);
            }

            if file_start {
                info!("Offload start is {}:{}", block, page);
            }
            if is_last_page_for_file {
                info!("Got last file {}:{}", block, page);
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
            'transfer_part: loop {
                watchdog.feed();
                let did_transfer = pi_spi.send_message_over_spi(
                    transfer_type,
                    part,
                    current_crc,
                    dma,
                    resets,
                    progress,
                );
                if did_transfer {
                    if attempts > 0 {
                        warn!("File part took multiple attempts: {}", attempts);
                    }
                    break 'transfer_part;
                }
                attempts += 1;
                if attempts > 100 {
                    success = false;
                    break 'transfer_part;
                }
            }

            // Give spi peripheral back to flash storage.
            if let Some(spi) = pi_spi.disable() {
                fs.take_spi(spi, resets);
                if is_last_page_for_file && success {
                    events.log(Event::OffloadedRecording, time, fs);
                }
            }
            if !success {
                break 'outer;
            }

            part_count += 1;
            if is_last_page_for_file {
                file_count += 1;
                info!("Offloaded {} file(s)", file_count);
                watchdog.feed();
                let _ = fs.erase_last_file();
                file_ended = true;
                break;
            }
            file_start = false;
        }

        if !file_ended {
            info!(
                "Incomplete file at block {} erasing",
                fs.file_start_block_index
            );
            watchdog.feed();
            if let Err(e) = fs.erase_last_file() {
                events.log(Event::ErasePartialOrCorruptRecording, time, fs);
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
    if success {
        if interrupted_by_user {
            events.log(Event::FileOffloadInterruptedByUser, time, fs);
        }
        info!(
            "Completed file offload, transferred {} files start {} previous is {}",
            file_count, fs.file_start_block_index, fs.previous_file_start_block_index
        );

        // Always rescan to update what kinds of files we have
        fs.scan();
        file_count != 0
    } else {
        events.log(Event::FileOffloadFailed, time, fs);
        fs.scan();
        warn!("File transfer to pi failed");
        false
    }
}

/// Returns `(Option<DeviceConfig>, true)` when config was updated
#[allow(clippy::too_many_lines)]
pub fn get_existing_device_config_or_config_from_pi_on_initial_handshake(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: HertzU32,
    existing_config: Option<DeviceConfig>,
    event_count: u16,
) -> (Option<DeviceConfig>, bool, bool) {
    let mut payload = [0u8; 16];
    let mut config_was_updated = false;
    let mut prioritise_frame_preview = false;

    let num_files_to_offload = fs.num_files_in_initial_scan;
    let num_blocks_to_offload = fs.last_used_block_index.unwrap_or(0);
    let num_events_to_offload = event_count;

    if let Some(free_spi) = fs.free_spi() {
        pi_spi.enable(free_spi, resets);
        LittleEndian::write_u32(&mut payload[0..4], u32::from(num_events_to_offload));
        LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);
        LittleEndian::write_u32(&mut payload[8..12], u32::from(num_files_to_offload));
        LittleEndian::write_u32(&mut payload[12..16], u32::from(num_blocks_to_offload));

        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        info!("Sending startup handshake");
        if pi_spi.send_message_over_spi(
            ExtTransferMessage::StartupHandshake,
            &payload,
            crc,
            dma,
            resets,
            None,
        ) {
            let new_config = if let Some(device_config) = pi_spi.return_payload() {
                // Skip 4 bytes of CRC checking

                let mut new_config = DeviceConfig::from_bytes(&device_config[4..]);
                let mut length_used = 0;
                if new_config.is_some() {
                    length_used = new_config.as_mut().unwrap().cursor_position;
                    prioritise_frame_preview = device_config[4 + length_used] == 5;
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
                                resets,
                                None,
                            ) {
                                if let Some(piece_bytes) = pi_spi.return_payload() {
                                    let crc_from_remote =
                                        LittleEndian::read_u16(&piece_bytes[0..2]);
                                    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                                    let length = piece_bytes[2];
                                    let crc =
                                        crc_check.checksum(&piece_bytes[2..=2 + length as usize]);
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
                (new_config, config_was_updated, prioritise_frame_preview)
            } else {
                warn!("Pi did not respond");
                (
                    existing_config,
                    config_was_updated,
                    prioritise_frame_preview,
                )
            };
            if let Some(spi_free) = pi_spi.disable() {
                fs.take_spi(spi_free, resets);
            }
            new_config
        } else {
            if let Some(spi_free) = pi_spi.disable() {
                fs.take_spi(spi_free, resets);
            }
            (
                existing_config,
                config_was_updated,
                prioritise_frame_preview,
            )
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
        (
            existing_config,
            config_was_updated,
            prioritise_frame_preview,
        )
    }
}
