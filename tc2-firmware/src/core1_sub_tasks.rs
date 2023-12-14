use crate::attiny_rtc_i2c::SharedI2C;
use crate::bsp::pac::{DMA, RESETS};
use crate::core1_task::wake_raspberry_pi;
use crate::device_config::DeviceConfig;
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::onboard_flash::OnboardFlash;
use crate::rp2040_flash::write_rp2040_flash;
use crate::FIRMWARE_VERSION;
use byteorder::{ByteOrder, LittleEndian};
use cortex_m::delay::Delay;
use crc::{Crc, CRC_16_XMODEM};
use defmt::{info, warn};
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::Timer;

pub fn maybe_offload_flash_storage(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    raspberry_pi_is_awake: &mut bool,
    clock_freq: u32,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    timer: &mut Timer,
) {
    if flash_storage.has_files_to_offload() {
        warn!("There are files to offload!");
        if !*raspberry_pi_is_awake {
            wake_raspberry_pi(raspberry_pi_is_awake, shared_i2c, delay);
        }
        if *raspberry_pi_is_awake {
            // do some offloading.
            let mut file_count = 0;
            flash_storage.begin_offload();
            let mut file_start = true;
            let mut part_count = 0;

            // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
            while let Some(((part, crc, block_index, page_index), is_last, spi)) =
                flash_storage.get_file_part()
            {
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
                //let start = timer.get_counter();
                pi_spi.send_message(transfer_type, &part, current_crc, dma, timer, resets);
                //info!("Took {}µs", (timer.get_counter() - start).to_micros());

                part_count += 1;
                if is_last {
                    file_count += 1;
                    info!("Offloaded {} file(s)", file_count);
                    file_start = true;
                } else {
                    file_start = false;
                }

                // Give spi peripheral back to flash storage.
                if let Some(spi) = pi_spi.disable() {
                    flash_storage.take_spi(spi, resets, clock_freq.Hz());
                }
            }
            info!("Completed file offload, transferred {} files", file_count);
            // TODO: Some validation from the raspberry pi that the transfer completed
            //  without errors, in the form of a hash, and if we have errors, we'd re-transmit.

            // Once we've successfully offloaded all files, we can erase the flash and we're
            // ready to start recording new CPTV files there.

            info!("Erasing after successful offload");
            //flash_storage.erase_all_good_used_blocks();
            flash_storage.erase_all_blocks();
        }
    }
}

pub fn get_existing_device_config_or_config_from_pi_on_initial_handshake(
    raspberry_pi_is_awake: bool,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: HertzU32,
    radiometry_enabled: u32,
    camera_serial_number: u32,
    timer: &mut Timer,
) -> Option<DeviceConfig> {
    let existing_config = DeviceConfig::load_existing_config_from_flash();
    if raspberry_pi_is_awake {
        let mut payload = [0u8; 12];
        if let Some(free_spi) = flash_storage.free_spi() {
            pi_spi.enable(free_spi, resets);

            LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
            LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);
            LittleEndian::write_u32(&mut payload[8..12], camera_serial_number);

            let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
            let crc = crc_check.checksum(&payload);
            info!("Sending camera connect info {:?}", payload);
            pi_spi.send_message(
                ExtTransferMessage::CameraConnectInfo,
                &payload,
                crc,
                dma,
                timer,
                resets,
            );
            let new_config = if let Some(device_config) = pi_spi.return_payload() {
                // Skip 4 bytes of CRC checking

                let (new_config, length_used) = DeviceConfig::from_bytes(&device_config[4..]);
                if let Some(new_config) = &new_config {
                    info!("Got config from rPi {:?}", new_config);
                    if existing_config.is_none()
                        || *new_config != *existing_config.as_ref().unwrap()
                    {
                        if existing_config.is_some() {
                            warn!(
                                "Config has changed {}",
                                *existing_config.as_ref().unwrap() != *new_config
                            );
                        }
                        write_rp2040_flash(&device_config[4..4 + length_used]);
                    }
                }
                new_config
            } else {
                warn!("Pi did not respond");
                existing_config
            };
            if let Some(spi_free) = pi_spi.disable() {
                flash_storage.take_spi(spi_free, resets, clock_freq);
            }
            new_config
        } else {
            warn!("Flash spi not enabled");
            existing_config
        }
    } else {
        if let Some(existing_config) = &existing_config {
            info!(
                "Got device config {:?}, {}",
                existing_config,
                existing_config.device_name()
            );
        }
        existing_config
    }
}
