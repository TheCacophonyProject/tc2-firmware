use crate::attiny_rtc_i2c::SharedI2C;
use crate::bsp;
use crate::bsp::pac::{DMA, RESETS};
use crate::device_config::DeviceConfig;
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::frame_processing::wake_raspberry_pi;
use crate::onboard_flash::OnboardFlash;
use crate::synced_date_time::SyncedDateTime;
use crate::FIRMWARE_VERSION;
use bsp::hal::Watchdog;
use byteorder::{ByteOrder, LittleEndian};
use cortex_m::delay::Delay;
use crc::{Crc, CRC_16_XMODEM};
use defmt::{info, warn};
use fugit::{ExtU32, HertzU32};

pub fn maybe_offload_events(
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    delay: &mut Delay,
    event_logger: &mut EventLogger,
    flash_storage: &mut OnboardFlash,
    synced_date_time: &SyncedDateTime,
    watchdog: &mut Option<&mut Watchdog>,
) -> bool {
    let mut success = true;
    if event_logger.has_events_to_offload() {
        let event_indices = event_logger.event_range();
        let total_events = event_indices.end;
        warn!("Transferring {} events", total_events);
        'transfer_all_events: for event_index in event_indices {
            watchdog.as_mut().map(|w| w.feed());
            let event_bytes = event_logger.event_at_index(event_index, flash_storage);
            if let Some(event_bytes) = event_bytes {
                if let Some(spi) = flash_storage.free_spi() {
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
                        if !did_transfer {
                            attempts += 1;
                            if attempts > 100 {
                                warn!("Failed sending logger event to raspberry pi");
                                success = false;
                                break 'transfer_event;
                            }
                        } else {
                            if attempts > 0 {
                                warn!("File part took multiple attempts: {}", attempts);
                            }
                            break 'transfer_event;
                        }
                    }
                    if let Some(spi) = pi_spi.disable() {
                        flash_storage.take_spi(spi, resets);
                    }
                    if !success {
                        break 'transfer_all_events;
                    }
                }
            }
        }
        info!("Offloaded {} event(s)", total_events);
        if success {
            event_logger.clear(flash_storage);
            event_logger.log_event(
                LoggerEvent::new(LoggerEventKind::OffloadedLogs, synced_date_time),
                flash_storage,
            );
        } else {
            event_logger.log_event(
                LoggerEvent::new(LoggerEventKind::LogOffloadFailed, synced_date_time),
                flash_storage,
            );
        }
    }
    success
}

pub fn offload_flash_storage_and_events(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    event_logger: &mut EventLogger,
    synced_date_time: &SyncedDateTime,
    mut watchdog: Option<&mut Watchdog>,
    only_last_file: bool,
) -> bool {
    warn!("There are files to offload!");
    watchdog.as_mut().map(|w| w.disable());

    if wake_raspberry_pi(shared_i2c, delay) {
        event_logger.log_event(
            LoggerEvent::new(LoggerEventKind::GotRpiPoweredOn, synced_date_time),
            flash_storage,
        );
    }
    watchdog.as_mut().map(|w| w.start(8388607.micros()));

    if !only_last_file {
        maybe_offload_events(
            pi_spi,
            resets,
            dma,
            delay,
            event_logger,
            flash_storage,
            synced_date_time,
            &mut watchdog,
        );
    }
    let mut has_file = flash_storage.begin_offload_reverse();

    // do some offloading.
    let mut file_count = 0;
    let mut success: bool = true;
    // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
    //  Probably doesn't matter though.

    while has_file {
        let mut file_start = true;
        let mut part_count = 0;
        let mut file_ended = false;
        let last_used_block_index = flash_storage.last_used_block_index.unwrap_or(0) as u16;

        // For speed of offloading, we read from flash cache into a one of the page buffers
        // held by the flash_storage.  Then we swap buffers and return the just read page,
        // so that it can be immediately transferred via DMA to the raspberry pi.
        // Only, we're sharing the same SPI peripheral to do this, so while the transfer is happening
        // we can't be transferring more to the back buffer from the flash.  So what's the point
        // of having a complex double buffering system?
        'outer: while let Some(((part, crc, block_index, page_index), is_last, spi)) =
            flash_storage.get_file_part()
        {
            watchdog.as_mut().map(|w| w.feed());
            pi_spi.enable(spi, resets);
            let transfer_type = if file_start && !is_last {
                ExtTransferMessage::BeginFileTransfer
            } else if !file_start && !is_last {
                ExtTransferMessage::ResumeFileTransfer
            } else if is_last {
                ExtTransferMessage::EndFileTransfer
            } else if file_start && is_last {
                ExtTransferMessage::BeginAndEndFileTransfer
            } else {
                crate::unreachable!("Invalid file transfer state");
            };

            let mut progress = None;
            if transfer_type == ExtTransferMessage::BeginFileTransfer {
                // Each file start will give the max used block
                progress = Some(last_used_block_index)
            } else if transfer_type == ExtTransferMessage::EndFileTransfer {
                progress = Some(block_index as u16)
            }

            if file_start {
                info!("Offload start is {}:{}", block_index, page_index)
            }
            if is_last {
                info!("Got last file {}:{}", block_index, page_index);
            }
            let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
            let current_crc = crc_check.checksum(&part);
            if current_crc != crc {
                warn!(
                    "Data corrupted at part #{} ({}:{}) in transfer to or from flash memory",
                    part_count, block_index, page_index
                );
            }

            let mut attempts = 0;
            'transfer_part: loop {
                watchdog.as_mut().map(|w| w.feed());
                let did_transfer = pi_spi.send_message_over_spi(
                    transfer_type,
                    &part,
                    current_crc,
                    dma,
                    resets,
                    progress,
                );
                if !did_transfer {
                    attempts += 1;
                    if attempts > 100 {
                        success = false;
                        break 'transfer_part;
                    }
                } else {
                    if attempts > 0 {
                        warn!("File part took multiple attempts: {}", attempts);
                    }
                    break 'transfer_part;
                }
            }

            // Give spi peripheral back to flash storage.
            if let Some(spi) = pi_spi.disable() {
                flash_storage.take_spi(spi, resets);
                if is_last && success {
                    event_logger.log_event(
                        LoggerEvent::new(LoggerEventKind::OffloadedRecording, synced_date_time),
                        flash_storage,
                    );
                }
            }
            if !success {
                break 'outer;
            }

            part_count += 1;
            if is_last {
                file_count += 1;
                info!("Offloaded {} file(s)", file_count);
                watchdog.as_mut().map(|w| w.feed());
                let _ = flash_storage.erase_last_file();
                file_ended = true;
                break;
            }
            file_start = false;
        }

        if !file_ended {
            info!(
                "Incomplete file at block {} erasing",
                flash_storage.file_start_block_index
            );
            watchdog.as_mut().map(|w| w.feed());
            if let Err(e) = flash_storage.erase_last_file() {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::ErasePartialOrCorruptRecording,
                        synced_date_time,
                    ),
                    flash_storage,
                );
            }
        }
        if only_last_file {
            break;
        }
        has_file = flash_storage.begin_offload_reverse();
    }

    if success {
        info!(
            "Completed file offload, transferred {} files start {} previous is {}",
            file_count,
            flash_storage.file_start_block_index,
            flash_storage.previous_file_start_block_index
        );
        file_count != 0
    } else {
        event_logger.log_event(
            LoggerEvent::new(LoggerEventKind::FileOffloadFailed, synced_date_time),
            flash_storage,
        );
        flash_storage.scan();
        warn!("File transfer to pi failed");
        false
    }
}

/// Returns `(Option<DeviceConfig>, true)` when config was updated
pub fn get_existing_device_config_or_config_from_pi_on_initial_handshake(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: HertzU32,
    radiometry_enabled: u32,
    camera_serial_number: u32,
    audio_mode: bool,
    existing_config: Option<DeviceConfig>,
) -> (Option<DeviceConfig>, bool) {
    let mut payload = [0u8; 16];
    let mut config_was_updated = false;
    if let Some(free_spi) = flash_storage.free_spi() {
        pi_spi.enable(free_spi, resets);

        // FIXME: Rework initial handshake
        LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
        LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);
        LittleEndian::write_u32(&mut payload[8..12], camera_serial_number);
        LittleEndian::write_u32(&mut payload[12..16], if audio_mode { 1 } else { 0 });

        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        if pi_spi.send_message_over_spi(
            ExtTransferMessage::CameraConnectInfo,
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
                                    let crc = crc_check
                                        .checksum(&piece_bytes[2..2 + length as usize + 1]);
                                    if crc == crc_from_remote {
                                        new_config
                                            .motion_detection_mask
                                            .append_piece(&piece_bytes[4..4 + length as usize]);
                                        break;
                                    } else {
                                        warn!("crc failed for mask piece {}, re-requesting", piece);
                                    }
                                }
                            }
                        }
                    }

                    info!("Got config from rPi {:#?}", new_config.config());
                    if existing_config.is_none()
                        || *new_config != *existing_config.as_ref().unwrap()
                    {
                        if existing_config.is_some() {
                            warn!(
                                "Config has changed {}",
                                *existing_config.as_ref().unwrap() != *new_config
                            );
                        }
                        new_config_bytes[length_used..length_used + 2400]
                            .copy_from_slice(&new_config.motion_detection_mask.inner);
                        let mut slice_to_write = &mut new_config_bytes[0..length_used + 2400];
                        if let Some(spi_free) = pi_spi.disable() {
                            flash_storage.take_spi(spi_free, resets);
                        }
                        flash_storage.write_device_config(&mut slice_to_write);
                        new_config.cursor_position += 2400;
                        config_was_updated = true;
                    }
                }
                (new_config, config_was_updated)
            } else {
                warn!("Pi did not respond");
                (existing_config, config_was_updated)
            };
            if let Some(spi_free) = pi_spi.disable() {
                flash_storage.take_spi(spi_free, resets);
            }
            new_config
        } else {
            if let Some(spi_free) = pi_spi.disable() {
                flash_storage.take_spi(spi_free, resets);
            }
            (existing_config, config_was_updated)
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
        (existing_config, config_was_updated)
    }
}
