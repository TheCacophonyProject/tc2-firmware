#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp::pac;
use crate::bsp::pac::{Peripherals, DMA, RESETS};
use crate::cptv_encoder::huffman::{HuffmanEntry, HUFFMAN_TABLE};
use crate::cptv_encoder::streaming_cptv::{make_crc_table, CptvStream};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{read_telemetry, FFCStatus, Telemetry};
use crate::motion_detector::{track_motion, MotionTracking};
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};
use crate::rp2040_flash::write_rp2040_flash;
use crate::utils::u8_slice_to_u16;
use crate::{bsp, FrameBuffer, FIRMWARE_VERSION};
use byteorder::{ByteOrder, LittleEndian};
use chrono::NaiveDateTime;
use core::cell::RefCell;
use core::ops::Add;
use cortex_m::delay::Delay;
use crc::{Crc, CRC_16_XMODEM};
use critical_section::Mutex;
use defmt::export::timestamp;
use defmt::{error, info, warn, Format};
use fugit::{Duration, HertzU32, RateExtU32};
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio5, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, PullNone, PullUp, SioOutput};
use rp2040_hal::pio::PIOExt;
use rp2040_hal::{Sio, Timer};

#[repr(u32)]
#[derive(Format)]
pub enum Core1Task {
    Ready = 0xdb,
    FrameProcessingComplete = 0xee,
    ReceiveFrame = 0xae,
    GotFrame = 0xab,
    StartRecording = 0xbc,
    EndRecording = 0xbe,
    StartFileOffload = 0xfa,
    EndFileOffload = 0xfb,
    ReadyToSleep = 0xef,
}

impl Into<u32> for Core1Task {
    fn into(self) -> u32 {
        self as u32
    }
}

impl From<u32> for Core1Task {
    fn from(value: u32) -> Self {
        value.into()
    }
}

pub struct Core1Pins {
    pub(crate) pi_ping: Pin<Gpio5, FunctionSio<SioOutput>, PullDown>,

    pub(crate) pi_miso: Pin<Gpio15, FunctionNull, PullNone>,
    pub(crate) pi_mosi: Pin<Gpio12, FunctionNull, PullNone>,
    pub(crate) pi_clk: Pin<Gpio14, FunctionNull, PullNone>,
    pub(crate) pi_cs: Pin<Gpio13, FunctionNull, PullNone>,

    pub(crate) fs_cs: Pin<Gpio9, FunctionSio<SioOutput>, PullDown>,
    pub(crate) fs_mosi: Pin<Gpio11, FunctionNull, PullNone>,
    pub(crate) fs_clk: Pin<Gpio10, FunctionNull, PullNone>,
    pub(crate) fs_miso: Pin<Gpio8, FunctionNull, PullNone>,
}
const FRAME_LENGTH: usize = FRAME_WIDTH * 122 * 2;
const WAIT_N_FRAMES_FOR_STABLE: usize = 20;
pub fn wake_raspberry_pi(is_awake: &mut bool, shared_i2c: &mut SharedI2C, delay: &mut Delay) {
    // TODO: Send a wake signal to the attiny, and poll its registers
    //  until we see that the raspberry pi is awake (and ideally that tc2-agent is running).
    //  Actually, tc2-agent will restart this firmware, so maybe we don't need to poll, we just
    //  block indefinitely?
    match shared_i2c.pi_is_powered_down(delay) {
        Ok(true) => {
            if shared_i2c.tell_pi_to_wakeup(delay).is_ok() {
                warn!("Sent wake signal to raspberry pi");
                // Poll to see when tc2-agent is ready.
                loop {
                    delay.delay_ms(1000);
                    if let Ok(pi_is_awake) =
                        shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true)
                    {
                        if pi_is_awake {
                            *is_awake = true;
                            break;
                        }
                    }
                }
            }
        }
        _ => loop {
            delay.delay_ms(1000);
            if let Ok(pi_is_awake) = shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
                if pi_is_awake {
                    *is_awake = true;
                    break;
                }
            }
        },
    }
}

pub fn power_down_raspberry_pi(is_awake: &mut bool, shared_i2c: &mut SharedI2C, delay: &mut Delay) {
    if shared_i2c.tell_pi_to_shutdown(delay).is_ok() {
        info!("Sent power-down signal to raspberry pi");
        loop {
            delay.delay_ms(1000);
            info!("Checking if pi is powered down yet");
            if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(delay) {
                if pi_is_powered_down {
                    *is_awake = false;
                    break;
                }
            }
        }
    }
}

fn maybe_offload_flash_storage(
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    raspberry_pi_is_awake: &mut bool,
    clock_freq: u32,
    shared_i2c: &mut SharedI2C,
    delay: &mut Delay,
    timer: &Timer,
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

fn get_existing_device_config_or_config_from_pi_on_initial_handshake(
    raspberry_pi_is_awake: bool,
    flash_storage: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    resets: &mut RESETS,
    dma: &mut DMA,
    clock_freq: HertzU32,
    radiometry_enabled: u32,
    camera_serial_number: u32,
    timer: &Timer,
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
                &timer,
                resets,
            );
            let device_config = pi_spi.return_payload().unwrap();
            // Skip 4 bytes of CRC checking

            let (new_config, length_used) = DeviceConfig::from_bytes(&device_config[4..]);
            if let Some(new_config) = &new_config {
                info!("Got config from rPi {:?}", new_config);
                if existing_config.is_none() || *new_config != *existing_config.as_ref().unwrap() {
                    if existing_config.is_some() {
                        warn!(
                            "Config has changed {}",
                            *existing_config.as_ref().unwrap() != *new_config
                        );
                    }
                    write_rp2040_flash(&device_config[4..4 + length_used]);
                }
            }
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
pub fn core_1_task(
    frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    clock_freq: u32,
    pins: Core1Pins,
    i2c_config: I2CConfig,
    lepton_serial: Option<u32>,
    lepton_firmware_version: Option<((u8, u8, u8), (u8, u8, u8))>,
) {
    let mut crc_buf = [0x42u8; 32 + 104];
    let mut payload_buf = [0x42u8; 2066];
    let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
    let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
    let crc_buf = unsafe { extend_lifetime_generic_mut(&mut crc_buf) };
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };

    let mut peripherals = unsafe { Peripherals::steal() };
    let timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
    let should_record_to_flash = true;

    let mut pi_spi = ExtSpiTransfers::new(
        pins.pi_mosi,
        pins.pi_cs,
        pins.pi_clk,
        pins.pi_miso,
        pins.pi_ping,
        dma_channels.ch0,
        payload_buf,
        crc_buf,
        pio0,
        sm0,
    );

    let mut spi_peripheral = Some(peripherals.SPI1);
    let mut flash_storage = OnboardFlash::new(
        pins.fs_cs,
        pins.fs_mosi,
        pins.fs_clk,
        pins.fs_miso,
        flash_page_buf,
        flash_page_buf_2,
        dma_channels.ch1,
        dma_channels.ch2,
        should_record_to_flash,
    );
    {
        flash_storage.take_spi(
            spi_peripheral.take().unwrap(),
            &mut peripherals.RESETS,
            clock_freq.Hz(),
        );
        flash_storage.init();
        if flash_storage.has_files_to_offload() {
            info!("Finished scan, has files to offload");
        }
    }

    // This is the raw frame buffer which can be sent to the rPi as is: it has 18 bytes
    // reserved at the beginning for a header, and 2 bytes of padding to make it align to 32bits
    //let mut thread_local_frame_buffer: [u8; FRAME_LENGTH + 20] = [0u8; FRAME_LENGTH + 20];
    let mut thread_local_frame_buffer: Option<&mut FrameBuffer> = None;

    // Create the GZIP crc table once on startup, then reuse, since it's expensive to
    // re-create inline.
    let crc_table = make_crc_table();

    // Copy huffman table into ram for faster access.
    let mut huffman_table = [HuffmanEntry { code: 0, bits: 0 }; 257];
    huffman_table.copy_from_slice(&HUFFMAN_TABLE[..]);

    let mut peripherals = unsafe { Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut sio = Sio::new(peripherals.SIO);
    sio.fifo.write_blocking(Core1Task::Ready.into());
    let radiometry_enabled = sio.fifo.read_blocking();
    info!("Core 1 got radiometry enabled: {}", radiometry_enabled == 2);
    let lepton_version = if radiometry_enabled == 2 { 35 } else { 3 };
    let mut shared_i2c = SharedI2C::new(i2c_config, &mut delay);
    let mut raspberry_pi_is_awake = shared_i2c
        .pi_is_awake_and_tc2_agent_is_ready(&mut delay, true)
        .unwrap_or(false);
    info!("Pi is awake and ready? {}", raspberry_pi_is_awake);
    let existing_config = DeviceConfig::load_existing_config_from_flash();

    info!("Existing config {:?}", existing_config);
    // FIXME: Usually we wouldn't need to wake the rPi at the beginning of the night,
    // but currently we need the rPi awake to access the RTC
    if existing_config.is_none() {
        // We need to wake up the rpi and get a config
        wake_raspberry_pi(&mut raspberry_pi_is_awake, &mut shared_i2c, &mut delay);
    }

    let device_config = get_existing_device_config_or_config_from_pi_on_initial_handshake(
        raspberry_pi_is_awake,
        &mut flash_storage,
        &mut pi_spi,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        clock_freq.Hz(),
        radiometry_enabled,
        lepton_serial.unwrap_or(0),
        &timer,
    );

    if device_config.is_none() {
        panic!("Device has no configuration yet.");
    }
    let mut device_config = device_config.unwrap();
    if !device_config.use_low_power_mode && !raspberry_pi_is_awake {
        wake_raspberry_pi(&mut raspberry_pi_is_awake, &mut shared_i2c, &mut delay);
    }
    maybe_offload_flash_storage(
        &mut flash_storage,
        &mut pi_spi,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        &mut raspberry_pi_is_awake,
        clock_freq,
        &mut shared_i2c,
        &mut delay,
        &timer,
    );
    if raspberry_pi_is_awake {
        pi_spi.enable_pio_spi();
    }
    // NOTE: Let Core0 know that it can start the frame loop
    // TODO: This could potentially start earlier to get lepton up and synced,
    //  we just need core 0 to not block on core 1 until this message is sent.
    warn!("Core 1 is ready to receive frames");
    sio.fifo.write_blocking(Core1Task::Ready.into());

    let mut cptv_stream: Option<CptvStream> = None;
    let mut prev_frame: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1] =
        [0u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1];

    let mut prev_frame_2: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1] =
        [0u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1];

    let mut frames_written = 0;
    let mut frames_seen = 0usize;
    let mut slowest_frame = 0;
    let mut prev_frame_telemetry: Option<Telemetry> = None;
    let mut last_datetime = None;
    let mut motion_detection: Option<MotionTracking> = None;

    // TODO: If flash storage is full, don't record.
    loop {
        let input = sio.fifo.read_blocking();
        crate::assert_eq!(
            input,
            Core1Task::ReceiveFrame.into(),
            "Got unknown fifo input to core1 task loop {}",
            input
        );

        let start = timer.get_counter();
        let selected_frame_buffer = sio.fifo.read_blocking();
        critical_section::with(|cs| {
            // Now we just swap the buffers?
            let buffer = if selected_frame_buffer == 0 {
                frame_buffer_local
            } else {
                frame_buffer_local_2
            };
            thread_local_frame_buffer = buffer.borrow_ref_mut(cs).take();
        });
        if !raspberry_pi_is_awake && frames_seen % 9 == 0 {
            // If the pi is asleep, check once a second to see if it's now available.
            //info!("Periodically checking if raspberry pi is awake");
            raspberry_pi_is_awake = shared_i2c
                .pi_is_awake_and_tc2_agent_is_ready(&mut delay, true)
                .unwrap_or(false);
            if raspberry_pi_is_awake {
                // NOTE: If the pi woke up this frame, we need to re-transmit the camera connect info.
                device_config = get_existing_device_config_or_config_from_pi_on_initial_handshake(
                    raspberry_pi_is_awake,
                    &mut flash_storage,
                    &mut pi_spi,
                    &mut peripherals.RESETS,
                    &mut peripherals.DMA,
                    clock_freq.Hz(),
                    radiometry_enabled,
                    lepton_serial.unwrap_or(0),
                    &timer,
                )
                .unwrap();
                // Enable raw frame transfers to pi – if not already enabled.
                pi_spi.enable_pio_spi();
            }
        }
        if raspberry_pi_is_awake {
            pi_spi.enable_pio_spi();
        }
        // Transfer RAW frame to pi if it is available.
        let transfer = pi_spi.begin_message(
            ExtTransferMessage::CameraRawFrameTransfer,
            &mut thread_local_frame_buffer
                .as_mut()
                .unwrap()
                .as_u8_slice_mut(),
            0,
            cptv_stream.is_some(),
            &mut peripherals.DMA,
        );
        let frame_buffer = &mut thread_local_frame_buffer
            .as_mut()
            .unwrap()
            .frame_data_as_u8_slice_mut();
        // Read the telemetry:
        let frame_telemetry = read_telemetry(&frame_buffer);
        let frame_num = frame_telemetry.frame_num;
        //info!("Got frame {}", frame_telemetry.frame_num);
        let too_close_to_ffc_event = frame_telemetry.msec_since_last_ffc < 5000
            || frame_telemetry.ffc_status == FFCStatus::Imminent
            || frame_telemetry.ffc_status == FFCStatus::InProgress;
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        let mut should_end_current_recording = false;
        if let Some(prev_telemetry) = &prev_frame_telemetry {
            if frame_telemetry.frame_num != prev_telemetry.frame_num + 1 {
                let skipped_frames =
                    (frame_telemetry.frame_num as i32 - prev_telemetry.frame_num as i32) - 1;
                if skipped_frames > 0 && frame_telemetry.frame_num < 1_000_000 {
                    // TODO: Never going to see more than 1_000_000 frames between reboots?
                    //  What about continuous recording mode?
                    warn!(
                        "Lost {} frame(s), got {}, prev was {}",
                        skipped_frames, frame_telemetry.frame_num, prev_telemetry.frame_num
                    );
                }
            }
        }
        if too_close_to_ffc_event && motion_detection.is_some() {
            warn!("Resetting motion detection");
            frames_seen = 0;
            motion_detection = None;
        }
        let mut queried_rtc_this_frame = false;
        if !too_close_to_ffc_event && frames_seen > WAIT_N_FRAMES_FOR_STABLE {
            let frame_buffer = &mut thread_local_frame_buffer
                .as_mut()
                .unwrap()
                .frame_data_as_u8_slice_mut();
            let current_raw_frame =
                unsafe { &u8_slice_to_u16(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT] }; // Telemetry skipped

            let start = timer.get_counter();
            let this_frame_motion_detection = Some(track_motion(
                &current_raw_frame,
                &prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT],
                &motion_detection,
            ));
            let end = timer.get_counter();
            let max_length_in_frames = 60 * 10 * 9;
            let max_length_in_frames = 60 * 9;
            if let Some(this_frame_motion_detection) = &this_frame_motion_detection {
                should_start_new_recording = device_config.use_low_power_mode
                    && this_frame_motion_detection.got_new_trigger()
                    && cptv_stream.is_none(); // wait until lepton stabilises before recording

                // Time out after 10 mins?
                should_end_current_recording = (this_frame_motion_detection.triggering_ended()
                    || frames_written >= max_length_in_frames)
                    && cptv_stream.is_some();
            }
            motion_detection = this_frame_motion_detection;

            last_datetime = if should_start_new_recording {
                // Begin cptv file
                match shared_i2c.get_datetime(&mut delay) {
                    Ok(now) => {
                        // info!(
                        //     "NOW {}:{}:{} {}:{}:{}",
                        //     now.year, now.month, now.day, now.hours, now.minutes, now.seconds
                        // );
                        queried_rtc_this_frame = true;
                        let date_time_utc = get_naive_datetime(now);
                        let is_inside_recording_window =
                            device_config.time_is_in_recording_window(&date_time_utc);
                        // FIXME: This is just for debug purposes.
                        let is_inside_recording_window = true;
                        if is_inside_recording_window {
                            let _ = shared_i2c
                                .set_recording_flag(&mut delay, true)
                                .map_err(|e| {
                                    error!("Error setting recording flag on attiny: {}", e)
                                });
                            error!("Starting new recording, {:?}", &frame_telemetry);
                            // TODO: Pass in various cptv header info bits.
                            let mut cptv_streamer = CptvStream::new(
                                date_time_utc.timestamp() as u64 * 1000 * 1000, // Microseconds
                                lepton_version,
                                lepton_serial.clone(),
                                lepton_firmware_version.clone(),
                                &device_config,
                                &mut flash_storage,
                                &huffman_table,
                                &crc_table,
                            );
                            cptv_streamer.init_gzip_stream(&mut flash_storage, false);

                            prev_frame_2.copy_from_slice(&prev_frame);
                            // Prev frame needs to be zeroed out at the start.
                            prev_frame.fill(0);
                            // NOTE: Write the initial frame before the trigger.
                            cptv_streamer.push_frame(
                                &prev_frame_2,
                                &mut prev_frame, // This should be zeroed out before starting a new clip.
                                &prev_frame_telemetry.as_ref().unwrap(),
                                &mut flash_storage,
                            );
                            prev_frame.copy_from_slice(&prev_frame_2);
                            frames_written += 1;

                            cptv_stream = Some(cptv_streamer);
                        } else {
                            info!("Would start recording, but outside recording window");
                        }
                        Some(date_time_utc)
                    }
                    Err(err_str) => {
                        error!("Unable to get DateTime from RTC: {}", err_str);
                        last_datetime
                    }
                }
            } else {
                last_datetime
            };
            if !should_end_current_recording {
                if let Some(cptv_stream) = &mut cptv_stream {
                    if let Some(prev_telemetry) = prev_frame_telemetry {
                        if frame_telemetry.frame_num == prev_telemetry.frame_num {
                            warn!("Duplicate frame {}", frame_telemetry.frame_num);
                        }
                        if frame_telemetry.msec_on == prev_telemetry.msec_on {
                            warn!(
                                "Duplicate frame {} (same time {})",
                                frame_telemetry.frame_num, frame_telemetry.msec_on
                            );
                        }
                    }
                    cptv_stream.push_frame(
                        current_raw_frame,
                        &mut prev_frame,
                        &frame_telemetry,
                        &mut flash_storage,
                    );
                    frames_written += 1;
                }
            } else {
                // Finalise on a a different frame period to writing out the prev/last frame,
                // to give more breathing room.
                if let Some(cptv_stream) = &mut cptv_stream {
                    error!("Ending current recording");
                    cptv_stream.finalise(&mut flash_storage);
                    ended_recording = true;
                    let _ = shared_i2c
                        .set_recording_flag(&mut delay, false)
                        .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));
                }
                cptv_stream = None;
            }
            prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT].copy_from_slice(current_raw_frame);
        }
        prev_frame_telemetry = Some(frame_telemetry);
        frames_seen += 1;
        // Check if we need to trigger:  Mostly at the moment we want to see what frame data
        // structures can be shared with encoding.
        if let Some((transfer, transfer_end_address, transfer_start_address)) = transfer {
            let did_abort_transfer = pi_spi.end_message(
                &mut peripherals.DMA,
                transfer_end_address,
                transfer_start_address,
                transfer,
            );
            if did_abort_transfer {
                raspberry_pi_is_awake = shared_i2c
                    .pi_is_awake_and_tc2_agent_is_ready(&mut delay, false)
                    .unwrap_or(false);
                if !raspberry_pi_is_awake {
                    info!("Transfer aborted, pi must be asleep");
                    pi_spi.disable_pio_spi();
                }
            }
        }
        critical_section::with(|cs| {
            // Now we just swap the buffers?
            let buffer = if selected_frame_buffer == 0 {
                frame_buffer_local
            } else {
                frame_buffer_local_2
            };
            *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
        });

        let end = timer.get_counter();
        let frame_time_us = (end - start).to_micros();
        slowest_frame = slowest_frame.max(frame_time_us);

        // TODO: Actually might be more useful as a moving average.
        // if frames_seen % 100 == 0 {
        //     info!(
        //         "Frame processing {}µs, worst case {}µs",
        //         (end - start).to_micros(),
        //         slowest_frame
        //     );
        // }

        if should_start_new_recording && cptv_stream.is_some() {
            info!("Send start recording message to core0");
            sio.fifo.write(Core1Task::StartRecording.into());
        }
        if ended_recording && cptv_stream.is_none() {
            info!("Send end recording message to core0");
            sio.fifo.write(Core1Task::EndRecording.into());
        }
        if flash_storage.is_too_full() {
            info!("Offload flash storage");
            maybe_offload_flash_storage(
                &mut flash_storage,
                &mut pi_spi,
                &mut peripherals.RESETS,
                &mut peripherals.DMA,
                &mut raspberry_pi_is_awake,
                clock_freq,
                &mut shared_i2c,
                &mut delay,
                &timer,
            );
            // TODO: If we woke to pi, when do we put it back to sleep?
        }
        if device_config.use_low_power_mode {
            if frames_seen % (10 * 9) == 0 {
                info!("Got frame #{}", frame_num);
            }

            if frames_seen % (60 * 9) == 0 && cptv_stream.is_none() {
                // Check this every minute.
                let date_time_utc = if queried_rtc_this_frame && last_datetime.is_some() {
                    last_datetime
                } else {
                    match shared_i2c.get_datetime(&mut delay) {
                        Ok(now) => Some(get_naive_datetime(now)),
                        Err(err_str) => {
                            error!("Unable to get DateTime from RTC: {}", err_str);
                            last_datetime
                        }
                    }
                };
                if let Some(date_time_utc) = date_time_utc {
                    let is_inside_recording_window =
                        device_config.time_is_in_recording_window(&date_time_utc);
                    if !is_inside_recording_window {
                        // Tell core0 we're exiting the recording loop
                        maybe_offload_flash_storage(
                            &mut flash_storage,
                            &mut pi_spi,
                            &mut peripherals.RESETS,
                            &mut peripherals.DMA,
                            &mut raspberry_pi_is_awake,
                            clock_freq,
                            &mut shared_i2c,
                            &mut delay,
                            &timer,
                        );
                        pi_spi.disable_pio_spi();

                        let date_time_utc = match shared_i2c.get_datetime(&mut delay) {
                            Ok(now) => Some(get_naive_datetime(now)),
                            Err(err_str) => {
                                error!("Unable to get DateTime from RTC: {}", err_str);
                                last_datetime
                            }
                        };
                        //info!("Would power down rpi");
                        // let next_recording_window_start =
                        //     device_config.next_recording_window_start(&date_time_utc.unwrap());

                        let now = date_time_utc.unwrap();
                        let next_recording_window_start = now.add(chrono::Duration::minutes(2));
                        shared_i2c.clear_alarm();
                        if let Ok(_) =
                            shared_i2c.set_wakeup_alarm(&next_recording_window_start, &mut delay)
                        {
                            info!("Interrupt enabled {}", shared_i2c.alarm_interrupt_enabled());
                            info!("Put us to sleep");

                            power_down_raspberry_pi(
                                &mut raspberry_pi_is_awake,
                                &mut shared_i2c,
                                &mut delay,
                            );
                            info!("Pi is now powered down: {}", raspberry_pi_is_awake);

                            // let mut elapsed = 0;
                            // while !shared_i2c.alarm_triggered() {
                            //     delay.delay_ms(1000);
                            //     info!("Alarm has not triggered, {}", elapsed);
                            //     elapsed += 1;
                            //     if elapsed > 3 * 60 {
                            //         break;
                            //     }
                            // }
                            // info!("Alarm triggered after {}s", elapsed);

                            // Tell core 0 to power down the lepton module, and wait for reply.
                            sio.fifo.write(Core1Task::ReadyToSleep.into());
                            let _ = sio.fifo.read();
                            if let Ok(_) = shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay) {
                                info!("Sleeping");
                            } else {
                                error!("Failed sending sleep request to attiny");
                            }
                            // Now we can put ourselves to sleep.
                        } else {
                            error!("Failed setting wake alarm, can't go to sleep");
                        }
                        // TODO: If we woke to pi, when do we put it back to sleep?
                        // TODO: Now set the wakeup alarm to the next window start time, and ask the attiny
                        // to put us to sleep.
                    }
                }
            } else {
                // Spend the same time as we would otherwise use querying the RTC to keep frame-times
                //  about the same
                delay.delay_us(2150);
            }
        }
        sio.fifo.write(Core1Task::FrameProcessingComplete.into());
    }
}
