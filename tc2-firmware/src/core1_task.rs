#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::cptv_encoder::huffman::{HuffmanEntry, HUFFMAN_TABLE};
use crate::cptv_encoder::streaming_cptv::{make_crc_table, CptvStream};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::read_telemetry;
use crate::onboard_flash::OnboardFlash;
use crate::utils::u8_slice_to_u16;
use crate::{bsp, FrameSeg, FIRMWARE_VERSION};
use byteorder::{ByteOrder, LittleEndian};
use core::cell::RefCell;
use cortex_m::singleton;
use crc::{Crc, CRC_16_XMODEM};
use critical_section::Mutex;
use defmt::{error, info, warn};
use fugit::RateExtU32;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio5, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, PullNone, SioOutput};
use rp2040_hal::pio::PIOExt;
use rp2040_hal::Sio;

#[repr(u32)]
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

pub fn wake_raspberry_pi() {
    // TODO: Send a wake signal to the attiny, and poll its registers
    //  until we see that the raspberry pi is awake (and ideally that tc2-agent is running).
    //  Actually, tc2-agent will restart this firmware, so maybe we don't need to poll, we just
    //  block indefinitely?
    info!("Sent wake signal to raspberry pi");
}

pub fn core_1_task(
    frame_buffer_local: &'static Mutex<RefCell<[FrameSeg; 4]>>,
    clock_freq: u32,
    pins: Core1Pins,
) {
    let crc_buf = singleton!(: [u8; 32] = [0x42; 32]).unwrap();
    let payload_buf = singleton!(: [u8; 2066] = [0x42; 2066]).unwrap();
    let flash_page_buf = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();
    let flash_page_buf_2 = singleton!(: [u8; 4 + 2048 + 128] = [0xff; 4 + 2048 + 128]).unwrap();

    let mut peripherals = unsafe { Peripherals::steal() };
    let mut watchdog = bsp::hal::Watchdog::new(peripherals.WATCHDOG);
    watchdog.enable_tick_generation((clock_freq / 1_000_000) as u8);
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

    let should_record_new = true;
    let should_record_to_flash = false;
    let num_seconds = 10;
    let num_frames_to_record = num_seconds * 9;
    // FIXME: Allocate all the buffers we need in this thread up front.

    // This is the raw frame buffer which can be sent to the rPi as is: it has 18 bytes
    // reserved at the beginning for a header, and 2 bytes of padding to make it align to 32bits
    let mut thread_local_frame_buffer: [u8; FRAME_LENGTH + 20] = [0u8; FRAME_LENGTH + 20];

    // Create the GZIP crc table once on startup, then reuse, since it's expensive to
    // re-create inline.
    let crc_table = make_crc_table();

    // Copy huffman table into ram for faster access.
    let mut huffman_table = [HuffmanEntry { code: 0, bits: 0 }; 257];
    huffman_table.copy_from_slice(&HUFFMAN_TABLE[..]);

    // Cptv Stream also holds another buffer of 1 frame length.
    // There is a 2K buffer for pi_spi transfers
    // There is another 2K+ buffer for flash_storage page
    // There is a 1K buffer for crc table in cptv stream.
    // It may be worth copying the huffman/lz77 table into RAM for speed also.
    // Try to put the FrameSeg buffer in ram on core 0?

    let raspberry_pi_is_awake = true;
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
            if should_record_new {
                info!("Erasing after successful offload");
                flash_storage.erase_all_good_used_blocks();
            }
        }
    }

    pi_spi.enable_pio_spi();
    // NOTE: Let Core0 know that it can start the frame loop
    // TODO: This could potentially start earlier to get lepton up and synced,
    //  we just need core 0 to not block on core 1 until this message is sent.
    warn!("Core 1 is ready to receive frames");
    sio.fifo.write_blocking(Core1Task::Ready.into());

    //pi_ping.set_low().unwrap();
    let mut motion_has_triggered = false;
    let mut this_frame_has_motion = should_record_new;
    let mut cptv_stream: Option<CptvStream> = None;

    let mut motion_left_frame = false;
    let mut five_seconds_have_passed_since_motion_left_frame = false;
    let mut thirty_seconds_have_passed_since_motion_disappeared_in_frame = false;
    let mut motion_disappeared_in_frame = false;
    let mut paused_cptv_recording_with_static_target_in_frame = false;
    let mut frames_written = 0;
    let mut frames_seen = 0;
    let mut prev_frame_number = 0;
    let mut prev_time_on_ms = 0;
    let mut slowest_frame = 0;
    loop {
        let input = sio.fifo.read_blocking();
        crate::assert_eq!(
            input,
            Core1Task::ReceiveFrame.into(),
            "Got unknown fifo input to core1 task loop {}",
            input
        );
        let start = *&timer.get_counter();
        // TODO: Could we indeed have a local double-buffer, and just allow the use of this
        // memory unsafely without copying it?
        critical_section::with(|cs| {
            for (seg_num, frame_seg) in frame_buffer_local.borrow_ref(cs).iter().enumerate() {
                let slice = frame_seg.as_u8_slice();
                // Write the slice length to read
                // FIXME - This was here because we were using frame_buffer to transmit
                //  directly to rPi raw frames.
                let start = seg_num * slice.len();
                let end = start + slice.len();
                let frame_buffer = &mut thread_local_frame_buffer[18..];
                frame_buffer[start..end].copy_from_slice(slice);
            }
        });
        let e = *&timer.get_counter();
        info!("Got frame in {}µs", (e - start).to_micros());
        sio.fifo.write(Core1Task::GotFrame.into());
        // // Transfer RAW frame to pi if it is available.
        let (transfer, transfer_end_address) = pi_spi.begin_message(
            ExtTransferMessage::CameraRawFrameTransfer,
            &mut thread_local_frame_buffer,
            0,
            cptv_stream.is_some(),
            &mut peripherals.DMA,
        );
        // Read the telemetry:
        let frame_buffer = &mut thread_local_frame_buffer[18..];
        let frame_telemetry = read_telemetry(&frame_buffer);
        frames_seen += 1;
        let should_start_new_recording = !motion_has_triggered
            && this_frame_has_motion
            && cptv_stream.is_none()
            && frames_seen > 20; // wait until lepton stabilises before recording
        let should_end_current_recording = ((motion_left_frame
            && five_seconds_have_passed_since_motion_left_frame)
            || (motion_disappeared_in_frame
                && thirty_seconds_have_passed_since_motion_disappeared_in_frame))
            && cptv_stream.is_some();

        let raw_frame =
            unsafe { &u8_slice_to_u16(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT] }; // Telemetry skipped
        let mut ended_recording = false;
        if should_start_new_recording && frames_written < num_frames_to_record {
            error!("Starting new recording");
            motion_has_triggered = true;
            // Begin cptv file
            // TODO: Record the current time when recording starts

            // TODO - compute crc table and copt huffman table as part of startup, and only
            // do it once, so that we won't stutter on new streams.

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
            info!("Created CPTV stream");
            // Write out the cptv header, with placeholders, then write out the previous
            // frame and the current frame.  Going dormant on the main thread gets turned off?  Maybe, we'll see
        }
        if !should_end_current_recording && frames_written != num_frames_to_record {
            if let Some(cptv_stream) = &mut cptv_stream {
                // TODO: double buffer prev/current raw frames?
                // At this point, do we really even want two cores running in tandem?
                //wake_interrupt_pin.set_high().unwrap();
                if frame_telemetry.frame_num == prev_frame_number {
                    warn!("Duplicate frame {}", frame_telemetry.frame_num);
                }
                if frame_telemetry.msec_on == prev_time_on_ms {
                    warn!(
                        "Duplicate frame {} (same time {})",
                        frame_telemetry.frame_num, frame_telemetry.msec_on
                    );
                }
                prev_frame_number = frame_telemetry.frame_num;
                prev_time_on_ms = frame_telemetry.msec_on;
                cptv_stream.push_frame(raw_frame, &frame_telemetry, &mut flash_storage);
                //wake_interrupt_pin.set_low().unwrap();
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
        // Check if we need to trigger:  Mostly at the moment we want to see what frame data
        // structures can be shared with encoding.
        pi_spi.end_message(&mut peripherals.DMA, transfer_end_address, transfer);
        let end = *&timer.get_counter();
        let frame_time_us = (end - start).to_micros();
        slowest_frame = slowest_frame.max(frame_time_us);
        if frames_seen % 100 == 0 {
            info!(
                "Frame processing {}µs, worst case {}µs",
                (end - start).to_micros(),
                slowest_frame
            );
        }
        if should_start_new_recording && cptv_stream.is_some() {
            sio.fifo.write(Core1Task::StartRecording.into());
        }
        if ended_recording && cptv_stream.is_none() {
            sio.fifo.write(Core1Task::EndRecording.into());
        }
        sio.fifo.write(Core1Task::FrameProcessingComplete.into());
    }

    // TODO: Need a way to exit this loop, and then offload files, and then re-enter the loop.
}
