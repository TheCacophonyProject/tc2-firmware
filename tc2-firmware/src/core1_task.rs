#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::cptv_encoder::huffman::{HuffmanEntry, HUFFMAN_TABLE};
use crate::cptv_encoder::streaming_cptv::{make_crc_table, CptvStream};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{read_telemetry, FFCStatus, Telemetry};
use crate::motion_detector::{track_motion, MotionTracking};
use crate::onboard_flash::OnboardFlash;
use crate::utils::u8_slice_to_u16;
use crate::{bsp, FrameBuffer, FIRMWARE_VERSION};
use byteorder::{ByteOrder, LittleEndian};
use core::cell::RefCell;
use cortex_m::singleton;
use crc::{Crc, CRC_16_XMODEM};
use critical_section::Mutex;
use defmt::{error, info, warn, Format};
use fugit::RateExtU32;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio5, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, PullNone, SioOutput};
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
pub fn wake_raspberry_pi() {
    // TODO: Send a wake signal to the attiny, and poll its registers
    //  until we see that the raspberry pi is awake (and ideally that tc2-agent is running).
    //  Actually, tc2-agent will restart this firmware, so maybe we don't need to poll, we just
    //  block indefinitely?
    info!("Sent wake signal to raspberry pi");
}

pub fn core_1_task(
    frame_buffer_local: &'static Mutex<RefCell<FrameBuffer>>,
    frame_buffer_local_2: &'static Mutex<RefCell<FrameBuffer>>,
    clock_freq: u32,
    pins: Core1Pins,
) {
    let crc_buf = singleton!(: [u8; 32] = [0x42; 32]).unwrap();
    let payload_buf = singleton!(: [u8; 2066] = [0x42; 2066]).unwrap();
    let flash_page_buf = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();
    let flash_page_buf_2 = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();

    let mut peripherals = unsafe { Peripherals::steal() };
    let timer = bsp::hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
    let dma_channels = peripherals.DMA.split(&mut peripherals.RESETS);

    let (pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);

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
        false,
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

    // Don't actually record to flash if in recording mode, while debugging.
    let should_record_to_flash = false;
    // For P2, we just say rPi is always awake.
    let raspberry_pi_is_awake = true;
    // Don't record, for updated P2 firmware mode.
    let is_in_non_recording_mode = true;

    // This is the raw frame buffer which can be sent to the rPi as is: it has 18 bytes
    // reserved at the beginning for a header, and 2 bytes of padding to make it align to 32bits
    let mut thread_local_frame_buffer: [u8; FRAME_LENGTH + 20] = [0u8; FRAME_LENGTH + 20];

    // Create the GZIP crc table once on startup, then reuse, since it's expensive to
    // re-create inline.
    let crc_table = make_crc_table();

    // Copy huffman table into ram for faster access.
    let mut huffman_table = [HuffmanEntry { code: 0, bits: 0 }; 257];
    huffman_table.copy_from_slice(&HUFFMAN_TABLE[..]);

    let mut peripherals = unsafe { Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    sio.fifo.write_blocking(Core1Task::Ready.into());
    let radiometry_enabled = sio.fifo.read_blocking();
    info!("Core 1 got radiometry enabled: {}", radiometry_enabled == 1);
    let lepton_version = if radiometry_enabled == 1 { 35 } else { 3 };

    if raspberry_pi_is_awake {
        let mut payload = [0u8; 8];
        let free_spi = flash_storage.free_spi();
        pi_spi.enable(free_spi, &mut peripherals.RESETS);

        LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
        LittleEndian::write_u32(&mut payload[4..8], FIRMWARE_VERSION);

        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        pi_spi.send_message(
            ExtTransferMessage::CameraConnectInfo,
            &payload,
            crc,
            &mut peripherals.DMA,
        );

        let spi_free = pi_spi.disable();
        flash_storage.take_spi(spi_free, &mut peripherals.RESETS, clock_freq.Hz());
    }
    // TODO: Switch modes here.
    //  If there are files to upload, do that.
    //  If the pi requests raw frames, do that (as well as being able to write to flash)
    //  Possibly we can get away with doing this via the PIO transfers?  Let's try!
    //  And then we switch to spi at the end to do the read-back?

    if flash_storage.has_files_to_offload() {
        warn!("There are files to offload!");
        if !raspberry_pi_is_awake {
            wake_raspberry_pi();
        }
        if raspberry_pi_is_awake {
            // do some offloading.
            let mut file_count = 0;
            flash_storage.begin_offload();
            let mut file_start = true;
            let mut part_count = 0;

            // TODO: Could speed this up slightly using cache_random_read interleaving on flash storage.
            while let Some(((part, crc, block_index, page_index), is_last, spi)) =
                flash_storage.get_file_part()
            {
                pi_spi.enable(spi, &mut peripherals.RESETS);
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
                let tt = match transfer_type {
                    ExtTransferMessage::BeginFileTransfer => "BEGIN FILE TRANSFER",
                    ExtTransferMessage::ResumeFileTransfer => "CONTINUE FILE TRANSFER",
                    ExtTransferMessage::EndFileTransfer => "END FILE TRANSFER",
                    ExtTransferMessage::BeginAndEndFileTransfer => "BEGIN & END FILE TRANSFER",
                    _ => crate::unreachable!("Invalid"),
                };
                //info!("tt {}, part len {}", tt, part.len());

                let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
                let current_crc = crc_check.checksum(&part);
                if current_crc != crc {
                    warn!(
                        "Data corrupted at part #{} ({}:{}) in transfer to or from flash memory",
                        part_count, block_index, page_index
                    );
                }
                pi_spi.send_message(transfer_type, &part, current_crc, &mut peripherals.DMA);

                part_count += 1;
                if is_last {
                    file_count += 1;
                    info!("Offloaded {} file(s)", file_count);
                    file_start = true;
                } else {
                    file_start = false;
                }

                // Give spi peripheral back to flash storage.
                let spi = pi_spi.disable();
                flash_storage.take_spi(spi, &mut peripherals.RESETS, clock_freq.Hz());
            }
            info!("Completed file offload, transferred {} files", file_count);
            // TODO: Some validation from the raspberry pi that the transfer completed
            //  without errors, in the form of a hash, and if we have errors, we'd re-transmit.

            // Once we've successfully offloaded all files, we can erase the flash and we're
            // ready to start recording new CPTV files there.

            info!("Erasing after successful offload");
            flash_storage.erase_all_good_used_blocks();
        }
    }

    pi_spi.enable_pio_spi();
    // NOTE: Let Core0 know that it can start the frame loop
    // TODO: This could potentially start earlier to get lepton up and synced,
    //  we just need core 0 to not block on core 1 until this message is sent.
    warn!("Core 1 is ready to receive frames");
    sio.fifo.write_blocking(Core1Task::Ready.into());

    let mut cptv_stream: Option<CptvStream> = None;
    let mut prev_frame: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1] =
        [0u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1];

    let mut frames_written = 0;
    let mut frames_seen = 0usize;
    let mut prev_frame_telemetry: Option<Telemetry> = None;
    let mut motion_detection: Option<MotionTracking> = None;

    loop {
        let input = sio.fifo.read_blocking();
        crate::assert_eq!(
            input,
            Core1Task::ReceiveFrame.into(),
            "Got unknown fifo input to core1 task loop {}",
            input
        );
        let selected_frame_buffer = sio.fifo.read_blocking();
        let start = timer.get_counter();

        // TODO: Could we indeed have a local double-buffer, and just allow the use of this
        // memory unsafely without copying it?  I think we surely could.

        // Are we stomping all over memory sometimes with this?
        // Maybe we do indeed need to double-buffer?
        critical_section::with(|cs| {
            let buffer = if selected_frame_buffer == 0 {
                frame_buffer_local
            } else {
                frame_buffer_local_2
            };
            // TODO: Can we get rid of this copying, and just use the buffer itself?
            thread_local_frame_buffer[18..18 + 39040]
                .copy_from_slice(&buffer.borrow_ref(cs).as_u8_slice());
        });
        //info!("Got frame in {}µs", (e - start).to_micros());
        //sio.fifo.write(Core1Task::GotFrame.into());
        // // Transfer RAW frame to pi if it is available.
        let (transfer, transfer_end_address) = pi_spi.begin_message(
            ExtTransferMessage::CameraRawFrameTransfer,
            &mut thread_local_frame_buffer,
            0,
            cptv_stream.is_some(),
            &mut peripherals.DMA,
        );
        let frame_buffer = &mut thread_local_frame_buffer[18..];
        // Read the telemetry:
        let frame_telemetry = read_telemetry(&frame_buffer);
        let too_close_to_ffc_event = frame_telemetry.msec_since_last_ffc < 5000
            || frame_telemetry.ffc_status == FFCStatus::Imminent
            || frame_telemetry.ffc_status == FFCStatus::InProgress
            || is_in_non_recording_mode;
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        let mut should_end_current_recording = false;
        if let Some(prev_telemetry) = &prev_frame_telemetry {
            if frame_telemetry.frame_num != prev_telemetry.frame_num + 1 {
                let skipped_frames =
                    (frame_telemetry.frame_num as i32 - prev_telemetry.frame_num as i32) - 1;
                if skipped_frames > 0 {
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
        if !too_close_to_ffc_event && frames_seen > WAIT_N_FRAMES_FOR_STABLE {
            let frame_buffer = &mut thread_local_frame_buffer[18..];
            let current_raw_frame =
                unsafe { &u8_slice_to_u16(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT] }; // Telemetry skipped

            let this_frame_motion_detection = Some(track_motion(
                &current_raw_frame,
                &prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT],
                &motion_detection,
            ));

            if let Some(this_frame_motion_detection) = &this_frame_motion_detection {
                should_start_new_recording =
                    this_frame_motion_detection.got_new_trigger() && cptv_stream.is_none(); // wait until lepton stabilises before recording

                should_end_current_recording =
                    this_frame_motion_detection.triggering_ended() && cptv_stream.is_some();
            }
            motion_detection = this_frame_motion_detection;

            if should_start_new_recording {
                error!("Starting new recording, {:?}", &frame_telemetry);
                // Begin cptv file
                // TODO: Record the current time when recording starts
                // TODO: Pass in various cptv header info bits.
                let mut cptv_streamer = CptvStream::new(
                    0,
                    lepton_version,
                    &mut flash_storage,
                    &huffman_table,
                    &crc_table,
                );
                cptv_streamer.init_gzip_stream(&mut flash_storage, false);
                cptv_stream = Some(cptv_streamer);

                // TODO: Write the prev frame before the trigger.

                // info!("Created CPTV stream");
                // Write out the cptv header, with placeholders, then write out the previous
                // frame and the current frame.  Going dormant on the main thread gets turned off?  Maybe, we'll see
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
                }
                cptv_stream = None;
            }
            prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT].copy_from_slice(current_raw_frame);
        }
        prev_frame_telemetry = Some(frame_telemetry);
        frames_seen += 1;
        // Check if we need to trigger:  Mostly at the moment we want to see what frame data
        // structures can be shared with encoding.
        pi_spi.end_message(&mut peripherals.DMA, transfer_end_address, transfer);
        if should_start_new_recording && cptv_stream.is_some() {
            info!("Send start recording message to core0");
            sio.fifo.write(Core1Task::StartRecording.into());
        }
        if ended_recording && cptv_stream.is_none() {
            info!("Send end recording message to core0");
            sio.fifo.write(Core1Task::EndRecording.into());
        }
        sio.fifo.write(Core1Task::FrameProcessingComplete.into());
    }

    // TODO: Need a way to exit this loop, and then offload files, and then re-enter the loop.
}
