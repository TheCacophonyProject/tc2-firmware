use crate::attiny_rtc_i2c::SharedI2C;
use crate::bsp;
use crate::bsp::pac::{DMA, RESETS};
use crate::core1_task::{wake_raspberry_pi, SyncedDateTime};
use crate::device_config::DeviceConfig;
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::onboard_flash::OnboardFlash;
use crate::rp2040_flash::write_device_config_to_rp2040_flash;
use crate::FIRMWARE_VERSION;
use byteorder::{ByteOrder, LittleEndian};
use cortex_m::delay::Delay;
use crc::{Crc, CRC_16_XMODEM};
use defmt::{info, warn};
use fugit::{ExtU32, HertzU32, RateExtU32};
use rp2040_hal::Timer;

use embedded_hal::prelude::{
    _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogDisable,
    _embedded_hal_watchdog_WatchdogEnable,
};

pub fn maybe_offload_events(
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    delay: &mut Delay,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
    flash_storage: &mut OnboardFlash,
    clock_freq: u32,
    time: &SyncedDateTime,
    mut watchdog: Option<&mut bsp::hal::Watchdog>,
) -> bool {
    let mut success = true;
    if event_logger.has_events_to_offload() {
        let event_indices = event_logger.event_range();
        let total_events = event_indices.end;
        warn!("Transferring {} events", total_events);
        let mut counter = timer.get_counter();
        'transfer_all_events: for event_index in event_indices {
            if watchdog.is_some() {
                watchdog.as_mut().unwrap().feed();
            }
            let event_bytes = event_logger.event_at_index(event_index, flash_storage);
            if let Some(event_bytes) = event_bytes {
                if let Some(spi) = flash_storage.free_spi() {
                    pi_spi.enable(spi, resets);
                    let transfer_type = ExtTransferMessage::SendLoggerEvent;
                    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                    let current_crc = crc_check.checksum(&event_bytes);
                    let mut attempts = 0;
                    'transfer_event: loop {
                        let did_transfer = pi_spi.send_message(
                            transfer_type,
                            &event_bytes,
                            current_crc,
                            dma,
                            timer,
                            resets,
                        );
                        counter = timer.get_counter();
                        if !did_transfer {
                            attempts += 1;
                            if attempts > 100 {
                                warn!("Failed sending logger event to raspberry pi");
                                success = false;
                                break 'transfer_event;
                            }
                            //takes tc2-agent about this long to poll again will always fail otherwise
                            let time_since = (timer.get_counter() - counter).to_micros();
                            if time_since < TIME_BETWEEN_TRANSFER {
                                delay.delay_us((TIME_BETWEEN_TRANSFER - time_since) as u32);
                            }
                        } else {
                            break 'transfer_event;
                        }
                    }
                    if let Some(spi) = pi_spi.disable() {
                        flash_storage.take_spi(spi, resets, clock_freq.Hz());
                    }
                    if !success {
                        break 'transfer_all_events;
                    }
                }
            }
        }
        info!("Offloaded {} event(s)", total_events);
        if success {
            let start = timer.get_counter();
            event_logger.clear(flash_storage);
            info!(
                "Clear events took {}µs",
                (timer.get_counter() - start).to_micros()
            );
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::OffloadedLogs,
                    time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
        } else {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::LogOffloadFailed,
                    time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
        }
    }
    return success;
}

//feels like should be longer but seems to work
const TIME_BETWEEN_TRANSFER: u64 = 1000;

pub fn offload_flash_storage_and_events(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: u32,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
    time: &SyncedDateTime,
    mut watchdog: Option<&mut bsp::hal::Watchdog>,
) -> bool {
    warn!("There are files to offload!");
    if watchdog.is_some() {
        watchdog.as_mut().unwrap().disable();
    }

    if wake_raspberry_pi(shared_i2c, delay) {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::GotRpiPoweredOn,
                time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
    }
    if watchdog.is_some() {
        watchdog.as_mut().unwrap().start(8388607.micros());
    }

    maybe_offload_events(
        pi_spi,
        resets,
        dma,
        delay,
        timer,
        event_logger,
        flash_storage,
        clock_freq,
        time,
        if watchdog.is_some() {
            Some(watchdog.as_mut().unwrap())
        } else {
            None
        },
    );
    // do some offloading.
    let mut file_count = 0;
    flash_storage.begin_offload();
    let mut success: bool = true;

    loop {
        let mut file_start = true;
        let mut part_count = 0;
        let mut counter = timer.get_counter();
        let mut start_block = 0;
        let mut start_page = 0;
        let mut file_end = false;
        // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
        //  Probably doesn't matter though.
        while let Some(((part, crc, block_index, page_index), is_last, spi)) =
            flash_storage.get_file_part()
        {
            file_end = is_last;
            if watchdog.is_some() {
                watchdog.as_mut().unwrap().feed();
            }
            pi_spi.enable(spi, resets);
            let transfer_type = if file_start && !is_last {
                start_block = block_index;
                start_page = 0;
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
                if watchdog.is_some() {
                    watchdog.as_mut().unwrap().feed();
                }
                let did_transfer =
                    pi_spi.send_message(transfer_type, &part, current_crc, dma, timer, resets);
                counter = timer.get_counter();
                if !did_transfer {
                    attempts += 1;
                    if attempts % 10 == 0 {
                        info!("Failed {}", attempts);
                    }
                    if attempts > 100 {
                        success = false;
                        break 'transfer_part;
                    }
                    //takes tc2-agent about this long to poll again will fail a lot otherwise
                    let time_since = (timer.get_counter() - counter).to_micros();
                    if time_since < TIME_BETWEEN_TRANSFER {
                        delay.delay_us((TIME_BETWEEN_TRANSFER - time_since) as u32);
                    }
                } else {
                    break 'transfer_part;
                }
            }

            // Give spi peripheral back to flash storage.
            if let Some(spi) = pi_spi.disable() {
                flash_storage.take_spi(spi, resets, clock_freq.Hz());
            }
            if !success {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::FileOffloadFailed,
                        time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
                break;
            } else if is_last {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::OffloadedRecording(
                            (((start_block as u64) << 32) | start_page as u64) as u64,
                        ),
                        time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::OffloadedRecording(
                            (((block_index as u64) << 32) | page_index as u64) as u64,
                        ),
                        time.get_timestamp_micros(&timer),
                    ),
                    flash_storage,
                );
            }
            part_count += 1;
            if is_last {
                file_count += 1;
                info!("Offloaded {} file(s)", file_count);
                file_start = true;
            } else {
                file_start = false;
            }
        }
        if !file_end && part_count > 0 {
            //possible corrupt file
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::CorruptFile,
                    time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
            info!(
                "Corrupt file trying next block {}",
                flash_storage.current_block_index + 1
            );
            flash_storage.set_current_position(flash_storage.current_block_index + 1, 0);
        } else {
            break;
        }
    }
    if success {
        info!("Completed file offload, transferred {} files", file_count);
        if file_count > 0 {
            // event_logger.log_event(
            //     LoggerEvent::new(
            //         LoggerEventKind::OffloadedRecording(file_count),
            //         time.get_timestamp_micros(&timer),
            //     ),
            //     flash_storage,
            // );
        }
        // TODO: Some validation from the raspberry pi that the transfer completed
        //  without errors, in the form of a hash, and if we have errors, we'd re-transmit.

        // Once we've successfully offloaded all files, we can erase the flash and we're
        // ready to start recording new CPTV files there.

        info!("Erasing after successful offload");
        //flash_storage.erase_all_good_used_blocks();
        flash_storage.erase_all_blocks();
        file_count != 0
    } else {
        flash_storage.scan();
        warn!("File transfer to pi failed");
        false
    }
}

pub fn offload_file(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: u32,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    timer: &mut Timer,
    event_logger: &mut EventLogger,
    time: &SyncedDateTime,
    mut watchdog: Option<&mut bsp::hal::Watchdog>,
    start_index: (isize, isize),
) -> bool {
    if watchdog.is_some() {
        watchdog.as_mut().unwrap().disable();
    }

    if wake_raspberry_pi(shared_i2c, delay) {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::GotRpiPoweredOn,
                time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
    }
    if watchdog.is_some() {
        watchdog.as_mut().unwrap().start(8388607.micros());
    }

    // do some offloading.
    let mut file_count = 0;
    flash_storage.set_current_position(start_index.0, start_index.1);
    let mut file_start = true;
    let mut part_count = 0;
    let mut success: bool = true;
    let mut counter;
    let mut last_block: isize = 0;
    let mut file_end = false;
    while let Some(((part, crc, block_index, page_index), is_last, spi)) =
        flash_storage.get_file_part()
    {
        file_end = is_last;
        if watchdog.is_some() {
            watchdog.as_mut().unwrap().feed();
        }
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
            if watchdog.is_some() {
                watchdog.as_mut().unwrap().feed();
            }
            let did_transfer =
                pi_spi.send_message(transfer_type, &part, current_crc, dma, timer, resets);
            counter = timer.get_counter();
            if !did_transfer {
                attempts += 1;
                if attempts % 10 == 0 {
                    info!("Failed {}", attempts);
                }
                if attempts > 100 {
                    success = false;
                    break 'transfer_part;
                }
                //takes tc2-agent about this long to poll again will fail a lot otherwise
                let time_since = (timer.get_counter() - counter).to_micros();
                if time_since < TIME_BETWEEN_TRANSFER {
                    delay.delay_us((TIME_BETWEEN_TRANSFER - time_since) as u32);
                }
            } else {
                break 'transfer_part;
            }
        }

        // Give spi peripheral back to flash storage.
        if let Some(spi) = pi_spi.disable() {
            flash_storage.take_spi(spi, resets, clock_freq.Hz());
        }
        if !success {
            info!("NOT success so breaking");
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::FileOffloadFailed,
                    time.get_timestamp_micros(&timer),
                ),
                flash_storage,
            );
            break;
        }

        part_count += 1;
        if is_last {
            last_block = block_index;
            file_count += 1;
            break;
        }
        file_start = false;
    }
    if !file_end && part_count > 0 {
        //possible corrupt file
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::CorruptFile,
                time.get_timestamp_micros(&timer),
            ),
            flash_storage,
        );
    }

    if success {
        info!("Completed file offload, transferred {} files", file_count);
        if file_count > 0 {
            // event_logger.log_event(
            //     LoggerEvent::new(
            //         LoggerEventKind::OffloadedRecording(file_count),
            //         time.get_timestamp_micros(&timer),
            //     ),
            //     flash_storage,
            // );
            info!("Erasing after successful offload");
            let _ = flash_storage.erase_block_range(start_index.0, last_block);
        }
        file_count != 0
    } else {
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
    timer: &mut Timer,
    existing_config: Option<DeviceConfig>,
) -> (Option<DeviceConfig>, bool) {
    let mut payload = [0u8; 13];
    let mut config_was_updated = false;
    if let Some(free_spi) = flash_storage.free_spi() {
        pi_spi.enable(free_spi, resets);

        LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
        LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);
        LittleEndian::write_u32(&mut payload[8..12], camera_serial_number);
        payload[12] = if audio_mode { 1 } else { 0 };
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        if pi_spi.send_message(
            ExtTransferMessage::CameraConnectInfo,
            &payload,
            crc,
            dma,
            timer,
            resets,
        ) {
            let new_config = if let Some(device_config) = pi_spi.return_payload() {
                // Skip 4 bytes of CRC checking

                let mut new_config = DeviceConfig::from_bytes(&device_config[4..]);
                let mut length_used = 0;
                if new_config.is_some() {
                    length_used = new_config.as_mut().unwrap().cursor_position;
                }
                let mut new_config_bytes = [0u8; 2400 + 105];
                new_config_bytes[0..length_used]
                    .copy_from_slice(&device_config[4..4 + length_used]);
                if let Some(new_config) = &mut new_config {
                    // Now get the motion detection mask which should take 24 transfers (since the return payload size is small)
                    for piece in 0..24 {
                        let payload = [piece];
                        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                        let crc = crc_check.checksum(&payload);
                        loop {
                            if pi_spi.send_message(
                                ExtTransferMessage::GetMotionDetectionMask,
                                &payload,
                                crc,
                                dma,
                                timer,
                                resets,
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
                        let slice_to_write = &new_config_bytes[0..length_used + 2400];
                        write_device_config_to_rp2040_flash(slice_to_write);
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
                flash_storage.take_spi(spi_free, resets, clock_freq);
            }
            new_config
        } else {
            if let Some(spi_free) = pi_spi.disable() {
                flash_storage.take_spi(spi_free, resets, clock_freq);
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
