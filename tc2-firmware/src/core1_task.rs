#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::core1_sub_tasks::{
    get_existing_device_config_or_config_from_pi_on_initial_handshake, maybe_offload_flash_storage,
};
use crate::cptv_encoder::huffman::{HuffmanEntry, HUFFMAN_TABLE};
use crate::cptv_encoder::streaming_cptv::{make_crc_table, CptvStream};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{get_naive_datetime, DeviceConfig};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{read_telemetry, FFCStatus, Telemetry};
use crate::motion_detector::{track_motion, MotionTracking};
use crate::onboard_flash::{extend_lifetime_generic_mut, OnboardFlash};
use crate::utils::u8_slice_to_u16;
use crate::{bsp, FrameBuffer};
use core::cell::RefCell;
use cortex_m::asm::wfe;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::{error, info, warn, Format};
use fugit::RateExtU32;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio5, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, PullNone, SioInput, SioOutput};
use rp2040_hal::pio::PIOExt;
use rp2040_hal::Sio;

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
    pub(crate) pi_ping: Pin<Gpio5, FunctionSio<SioInput>, PullDown>,

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
const WAIT_N_FRAMES_FOR_STABLE: usize = 45;
pub fn wake_raspberry_pi(shared_i2c: &mut SharedI2C, delay: &mut Delay) {
    match shared_i2c.pi_is_powered_down(delay) {
        Ok(true) => {
            if shared_i2c.tell_pi_to_wakeup(delay).is_ok() {
                warn!("Sent wake signal to raspberry pi");
                // Poll to see when tc2-agent is ready.
                loop {
                    if let Ok(pi_is_awake) =
                        shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true)
                    {
                        if pi_is_awake {
                            break;
                        } else {
                            // Try to wake it again, just in case it was shutdown behind our backs.
                            let _ = shared_i2c.tell_pi_to_wakeup(delay);
                        }
                    }
                    delay.delay_ms(1000);
                }
            }
        }
        _ => loop {
            if let Ok(pi_is_awake) = shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true) {
                if pi_is_awake {
                    break;
                } else {
                    // Try to wake it again, just in case it was shutdown behind our backs.
                    let _ = shared_i2c.tell_pi_to_wakeup(delay);
                }
            }
            delay.delay_ms(1000);
        },
    }
}

pub fn advise_raspberry_pi_it_may_shutdown(shared_i2c: &mut SharedI2C, delay: &mut Delay) {
    if shared_i2c.tell_pi_to_shutdown(delay).is_err() {
        error!("Error sending power-down advice to raspberry pi");
    } else {
        info!("Sent power-down advice to raspberry pi");
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

pub fn core_1_task(
    frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    clock_freq: u32,
    pins: Core1Pins,
    i2c_config: I2CConfig,
    lepton_serial: Option<u32>,
    lepton_firmware_version: Option<((u8, u8, u8), (u8, u8, u8))>,
) {
    info!("Core 1 start");
    let mut crc_buf = [0x42u8; 32 + 104];
    let mut payload_buf = [0x42u8; 2066];
    let mut flash_page_buf = [0xffu8; 4 + 2048 + 128];
    let mut flash_page_buf_2 = [0xffu8; 4 + 2048 + 128];
    let crc_buf = unsafe { extend_lifetime_generic_mut(&mut crc_buf) };
    let payload_buf = unsafe { extend_lifetime_generic_mut(&mut payload_buf) };
    let flash_page_buf = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf) };
    let flash_page_buf_2 = unsafe { extend_lifetime_generic_mut(&mut flash_page_buf_2) };

    let mut peripherals = unsafe { Peripherals::steal() };
    let mut timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
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
    let existing_config = DeviceConfig::load_existing_config_from_flash();

    info!("Existing config {:?}", existing_config);
    if existing_config.is_none() {
        // We need to wake up the rpi and get a config
        wake_raspberry_pi(&mut shared_i2c, &mut delay);
    } else {
        // NOTE: Wake RPi for DEBUG purposes.
        wake_raspberry_pi(&mut shared_i2c, &mut delay);
    }

    let device_config = get_existing_device_config_or_config_from_pi_on_initial_handshake(
        &mut flash_storage,
        &mut pi_spi,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        clock_freq.Hz(),
        radiometry_enabled,
        lepton_serial.unwrap_or(0),
        &mut timer,
    );

    if device_config.is_none() {
        panic!("Device has no configuration yet.");
    }
    let mut device_config = device_config.unwrap();
    if !device_config.use_low_power_mode {
        wake_raspberry_pi(&mut shared_i2c, &mut delay);
    }
    maybe_offload_flash_storage(
        &mut flash_storage,
        &mut pi_spi,
        &mut peripherals.RESETS,
        &mut peripherals.DMA,
        clock_freq,
        &mut shared_i2c,
        &mut delay,
        &mut timer,
    );

    warn!("Unset recording flag on attiny");
    let _ = shared_i2c
        .set_recording_flag(&mut delay, false)
        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
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

    // NOTE: Keep retrying until we get a datetime from RTC – if we're in low power mode, otherwise maybe we don't care.
    let mut last_datetime = None;
    if device_config.use_low_power_mode {
        while last_datetime.is_none() {
            if let Ok(now) = shared_i2c.get_datetime(&mut delay) {
                last_datetime = Some(get_naive_datetime(now));
            } else {
                warn!("Failed getting date from RTC, retrying");
                delay.delay_ms(10);
            }
        }
    }

    let mut motion_detection: Option<MotionTracking> = None;
    let mut should_reboot = false;

    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    loop {
        let start_block = timer.get_counter();
        loop {
            let input = sio.fifo.read();
            if let Some(input) = input {
                crate::assert_eq!(
                    input,
                    Core1Task::ReceiveFrame.into(),
                    "Got unknown fifo input to core1 task loop {}",
                    input
                );
                break;
            } else {
                if (timer.get_counter() - start_block).to_millis() > 30_000 {
                    // Something went wrong and we didn't get any frames for 30 seconds
                    should_reboot = true;
                    break;
                }
                wfe();
            }
        }

        let start = timer.get_counter();

        // Get the currently selected buffer to transfer/write to disk.
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
            &mut timer,
        );
        let frame_buffer = &mut thread_local_frame_buffer
            .as_mut()
            .unwrap()
            .frame_data_as_u8_slice_mut();
        // Read the telemetry:
        let frame_telemetry = read_telemetry(&frame_buffer);
        let frame_num = frame_telemetry.frame_num;
        let too_close_to_ffc_event = frame_telemetry.msec_since_last_ffc < 5000
            || frame_telemetry.ffc_status == FFCStatus::Imminent
            || frame_telemetry.ffc_status == FFCStatus::InProgress;
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        let mut should_end_current_recording = false;
        let mut frames_elapsed = 1;
        if let Some(prev_telemetry) = &prev_frame_telemetry {
            if frame_telemetry.frame_num != prev_telemetry.frame_num + 1 {
                let skipped_frames =
                    (frame_telemetry.frame_num as i32 - prev_telemetry.frame_num as i32) - 1;
                frames_elapsed += skipped_frames.min(2);
                if skipped_frames > 0 && frame_telemetry.frame_num < 1_000_000 {
                    // TODO: Never going to see more than 1_000_000 frames between reboots?
                    //  What about continuous recording mode?
                    warn!(
                        "Lost {} frame(s), got {}, prev was {}",
                        skipped_frames, frame_telemetry.frame_num, prev_telemetry.frame_num
                    );
                } else {
                    //
                }
            }
        }
        if too_close_to_ffc_event && motion_detection.is_some() {
            warn!("Resetting motion detection due to FFC event");
            frames_seen = 0;
            motion_detection = None;
        }

        // NOTE: In low power mode, don't try to start recordings/motion detection until frames have stabilised.
        if !too_close_to_ffc_event
            && frames_seen > WAIT_N_FRAMES_FOR_STABLE
            && device_config.use_low_power_mode
        {
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
                should_start_new_recording = !flash_storage.is_too_full_to_start_new_recordings()
                    && this_frame_motion_detection.got_new_trigger()
                    && cptv_stream.is_none(); // wait until lepton stabilises before recording

                // Time out after 10 mins?
                should_end_current_recording = cptv_stream.is_some()
                    && (this_frame_motion_detection.triggering_ended()
                        || frames_written >= max_length_in_frames
                        || flash_storage.is_nearly_full());
            }
            motion_detection = this_frame_motion_detection;

            if should_start_new_recording {
                // Begin CPTV file

                // NOTE: Rather than trying to get the RTC time right as we're trying to start a CPTV file,
                //  we just get it periodically, and then each frame add to it, then re-sync it
                // (when we do our once a minute checks) when we're *not* trying to start a recording.
                let date_time_utc = last_datetime.as_ref().unwrap();
                let is_inside_recording_window =
                    device_config.time_is_in_recording_window(&date_time_utc);
                if is_inside_recording_window {
                    warn!("Setting recording flag on attiny");
                    // TODO: Do we actually want to do this?  It's really there so the RPi/Attiny doesn't shut us down while
                    //  we're writing to the flash.  Needs implementation on Attiny side.
                    let _ = shared_i2c
                        .set_recording_flag(&mut delay, true)
                        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
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
            }

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

        if let Some((transfer, transfer_end_address, transfer_start_address)) = transfer {
            let did_abort_transfer = pi_spi.end_message(
                &mut peripherals.DMA,
                transfer_end_address,
                transfer_start_address,
                transfer,
            );
            if did_abort_transfer {
                warn!("Transfer aborted, pi must be asleep?");
            }
        }
        // TODO: Is here the best place to swap buffers?
        critical_section::with(|cs| {
            // Now we just swap the buffers?
            let buffer = if selected_frame_buffer == 0 {
                frame_buffer_local
            } else {
                frame_buffer_local_2
            };
            *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
        });

        if should_start_new_recording && cptv_stream.is_some() {
            info!("Send start recording message to core0");
            sio.fifo.write(Core1Task::StartRecording.into());
        }
        if ended_recording && cptv_stream.is_none() {
            info!("Send end recording message to core0");
            sio.fifo.write(Core1Task::EndRecording.into());
        }

        // NOTE: Check if we need to go to sleep etc.
        if device_config.use_low_power_mode {
            if frames_seen % (10 * 9) == 0 {
                info!("Got frame #{}", frame_num);
            }

            // Once per minute, if we're not currently recording, tell the RPi it can shut-down, as it's not
            // needed in low-power mode unless it's offloading/uploading CPTV data.
            if (should_reboot || (frames_seen > 1 && frames_seen % (60 * 9) == 0))
                && cptv_stream.is_none()
            {
                // NOTE: We only advise the RPi that it can shut down if we're not currently recording –
                //  since the change in frame times can affect our frame sync.  It's fine to call this repeatedly,
                //  the RPi will shut down when it wants to.

                //advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
                last_datetime = match shared_i2c.get_datetime(&mut delay) {
                    Ok(now) => Some(get_naive_datetime(now)),
                    Err(err_str) => {
                        error!("Unable to get DateTime from RTC: {}", err_str);
                        last_datetime
                    }
                };

                // TODO: Only do this if not in 24/7 mode - which we don't support yet.
                if let Some(date_time_utc) = last_datetime {
                    let is_outside_recording_window =
                        !device_config.time_is_in_recording_window(&date_time_utc);
                    let flash_storage_nearly_full =
                        flash_storage.is_too_full_to_start_new_recordings();
                    if is_outside_recording_window || flash_storage_nearly_full {
                        if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(&mut delay) {
                            if pi_is_powered_down {
                                info!("Pi is now powered down: {}", pi_is_powered_down);
                                // NOTE: Calculate the start of the next recording window, set the RTC wake-up alarm,
                                //  and ask for the rp2040 to be put to sleep.
                                let next_recording_window_start = if flash_storage_nearly_full
                                    || (is_outside_recording_window
                                        && flash_storage.has_files_to_offload())
                                {
                                    // If flash storage is nearly full, or we're now outside the recording window,
                                    //  restart in 2 minutes so we can offload files.
                                    date_time_utc + chrono::Duration::minutes(2)
                                } else {
                                    // Otherwise, restart at the start of the next recording window.
                                    device_config.next_recording_window_start(&date_time_utc)
                                };

                                // NOTE: For DEBUG purposes
                                let next_recording_window_start =
                                    date_time_utc + chrono::Duration::minutes(2);
                                shared_i2c.clear_alarm();
                                if let Ok(_) = shared_i2c
                                    .set_wakeup_alarm(&next_recording_window_start, &mut delay)
                                {
                                    let alarm_enabled = shared_i2c.alarm_interrupt_enabled();
                                    info!("Wake up alarm interrupt enabled {}", alarm_enabled);
                                    if alarm_enabled {
                                        info!("Tell core0 to get ready to sleep");
                                        // Tell core0 we're exiting the recording loop, and it should
                                        // down the lepton module, and wait for reply.
                                        sio.fifo.write(Core1Task::ReadyToSleep.into());
                                        loop {
                                            if let Some(result) = sio.fifo.read() {
                                                if result == 255 {
                                                    break;
                                                }
                                            }
                                        }
                                        info!("Ask Attiny to power down rp2040");
                                        if let Ok(_) =
                                            shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay)
                                        {
                                            info!("Sleeping");
                                        } else {
                                            error!("Failed sending sleep request to attiny");
                                        }
                                    } else {
                                        error!("Alarm was not properly enabled");
                                    }
                                    // Now we can put ourselves to sleep.
                                } else {
                                    error!("Failed setting wake alarm, can't go to sleep");
                                }
                            } else {
                                warn!("Pi is still awake, so rp2040 must stay awake");
                            }
                        }
                    }
                } else {
                    unreachable!("No last DateTime set!");
                }
            } else {
                // Increment the datetime n frame's worth.
                if let Some(last_datetime) = &mut last_datetime {
                    *last_datetime += chrono::Duration::milliseconds(115 * frames_elapsed as i64);
                }
                // Spend the same time as we would otherwise use querying the RTC to keep frame-times
                //  about the same
                delay.delay_us(2150);
            }
        }
        prev_frame_telemetry = Some(frame_telemetry);
        frames_seen += 1;
        info!("Loop took {}", (timer.get_counter() - start).to_micros());
        sio.fifo.write(Core1Task::FrameProcessingComplete.into());
    }
}
