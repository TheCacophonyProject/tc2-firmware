#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::SharedI2C;
use crate::bsp::pac::DMA;
use crate::cptv_encoder::huffman::{HUFFMAN_TABLE, HuffmanEntry};
use crate::cptv_encoder::streaming_cptv::{CptvStream, make_crc_table};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{AudioMode, DeviceConfig, get_datetime_utc};
use crate::event_logger::{Event, EventLogger, WakeReason};
use core::cell::RefCell;

use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage, RPI_TRANSFER_HEADER_LENGTH};
use crate::lepton::FFCStatus;
use crate::motion_detector::{MotionTracking, track_motion};
use crate::onboard_flash::OnboardFlash;
use chrono::{DateTime, Datelike, Duration, Timelike, Utc};

use crate::lepton_telemetry::Telemetry;
use crate::rpi_power::{advise_raspberry_pi_it_may_shutdown, wake_raspberry_pi};
use crate::synced_date_time::SyncedDateTime;
use cortex_m::asm::nop;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::{Format, error, info, warn};
use rp2040_hal::{Sio, Timer};

pub const NUM_LEPTON_SEGMENTS: usize = 4;
pub const NUM_LINES_PER_LEPTON_SEGMENT: usize = 61;
const FRAME_BUFFER_ALIGNMENT_PADDING: usize = 2;
const LEPTON_RAW_FRAME_PAYLOAD_LENGTH: usize =
    FRAME_WIDTH * NUM_LINES_PER_LEPTON_SEGMENT * NUM_LEPTON_SEGMENTS;
const FRAME_BUFFER_LENGTH: usize =
    RPI_TRANSFER_HEADER_LENGTH + LEPTON_RAW_FRAME_PAYLOAD_LENGTH + FRAME_BUFFER_ALIGNMENT_PADDING;
const NUM_STATUS_RECORDING_FRAMES: u32 = 18;
const DEV_MODE: bool = false;
#[repr(C, align(32))]
pub struct FrameBuffer([u8; FRAME_BUFFER_LENGTH]);
pub type StaticFrameBuffer = &'static Mutex<RefCell<Option<&'static mut FrameBuffer>>>;

impl Default for FrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameBuffer {
    #[allow(clippy::large_stack_arrays)]
    pub const fn new() -> FrameBuffer {
        // NOTE: Put an 18 byte padding at the start, and a 2 byte padding at the end, to make it 32bit aligned
        FrameBuffer([0u8; FRAME_BUFFER_LENGTH])
    }

    pub fn as_u8_slice(&self) -> &[u8] {
        &self.0[RPI_TRANSFER_HEADER_LENGTH
            ..RPI_TRANSFER_HEADER_LENGTH + LEPTON_RAW_FRAME_PAYLOAD_LENGTH]
    }

    pub fn as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }

    pub fn ffc_imminent(&mut self, is_imminent: bool) {
        let val = u8::from(is_imminent);
        self.0[RPI_TRANSFER_HEADER_LENGTH + 636] = val;
        self.0[RPI_TRANSFER_HEADER_LENGTH + 637] = val;
    }

    pub fn frame_data_as_u8_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0[RPI_TRANSFER_HEADER_LENGTH
            ..RPI_TRANSFER_HEADER_LENGTH + LEPTON_RAW_FRAME_PAYLOAD_LENGTH]
    }

    pub fn packet(&mut self, segment: usize, packet_id: usize) -> &mut [u8] {
        let segment_offset = FRAME_WIDTH * NUM_LINES_PER_LEPTON_SEGMENT * segment;
        let packet_offset = FRAME_WIDTH * packet_id;
        let offset = segment_offset + packet_offset;
        &mut self.0[RPI_TRANSFER_HEADER_LENGTH..][offset..offset + FRAME_WIDTH]
    }
}

#[repr(u32)]
#[derive(Format)]
pub enum Core0Task {
    Ready = 0xdb,
    ReadyToReceiveLeptonConfig = 0xec,
    SendIntercoreArray = 0x0c,
    FrameProcessingComplete = 0xee,
    ReceiveFrame = 0xae,
    GotFrame = 0xab,
    StartRecording = 0xbc,
    EndRecording = 0xbe,
    StartFileOffload = 0xfa,
    EndFileOffload = 0xfb,
    ReadyToSleep = 0xef,
    RequestReset = 0xea,
    HighPowerMode = 0xeb,
    ReceiveFrameWithPendingFFC = 0xac,
}
#[derive(Format, PartialEq, Eq)]
enum StatusRecording {
    StartupStatus = 0,
    ShutdownStatus = 1,
    NotPending = 3,
}

impl StatusRecording {
    pub fn is_pending(&self) -> bool {
        *self != StatusRecording::NotPending
    }
    pub fn is_not_pending(&self) -> bool {
        !self.is_pending()
    }
}

impl From<Core0Task> for u32 {
    fn from(task: Core0Task) -> Self {
        task as u32
    }
}

const WAIT_N_FRAMES_FOR_STABLE: usize = 45;

#[allow(clippy::struct_excessive_bools)]
pub struct BookkeepingState {
    audio_pending: bool,
    frame_checkpoint: u32,
    is_recording_on_rpi: bool,
    made_shutdown_status_recording: bool,
    made_startup_status_recording: bool,
    making_status_recording: bool,
    lost_frame_count: u32,
    next_audio_alarm: Option<DateTime<Utc>>,
    low_power_mode: bool,

    logged_frame_transfer: Option<()>,
    logged_told_rpi_to_sleep: Option<()>,
    logged_pi_powered_down: Option<()>,
    logged_fs_nearly_full: Option<()>,
}

impl BookkeepingState {
    pub fn twenty_seconds_elapsed_since_last_check(&self, telemetry: &Telemetry) -> bool {
        telemetry.frame_num - self.frame_checkpoint > 9 * 20
    }
    pub fn reset_checkpoint(&mut self) {
        self.frame_checkpoint = 0;
    }
    pub fn use_low_power_mode(&self) -> bool {
        self.low_power_mode
    }
    pub fn use_high_power_mode(&self) -> bool {
        !self.low_power_mode
    }
}

// FIXME: Clean up the logic around status recordings

#[allow(clippy::too_many_lines)]
pub fn thermal_motion_task(
    mut delay: Delay,
    mut sio: Sio,
    mut dma: DMA,
    mut i2c: SharedI2C,
    mut pi_spi: ExtSpiTransfers,
    mut fs: OnboardFlash,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
    config: &DeviceConfig,
    mut events: EventLogger,
    mut time: SyncedDateTime,
    next_audio_alarm: Option<DateTime<Utc>>,
) -> ! {
    info!("=== Core 0 Thermal Motion start ===");
    if DEV_MODE {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }
    let timer = time.get_timer();

    // TODO: Do we want to have a max recording length timeout, or just pause recording if a
    //  subject stays in the frame but doesn't move for a while?  Maybe if a subject is stationary
    //  for 1 minute, we pause, and only resume recording if there is new movement, or it moves again?
    //  If the night ends in this way, we end the recording then.
    //  In continuous recording mode we'd have a really long timeout perhaps?  Needs more thought.
    //  Also consider the case where we have a mask region to ignore or pay attention to.

    events.log(Event::ThermalMode, &time, &mut fs);
    if let Ok(is_recording) = i2c.get_is_recording(&mut delay) {
        if is_recording {
            events.log(Event::RecordingNotFinished, &time, &mut fs);
        }
    }
    // Unset the is_recording flag on attiny on startup
    let _ = i2c
        .set_recording_flag(&mut delay, false)
        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
    let startup_date_time_utc: DateTime<Utc> = time.get_date_time();

    // This is the 'raw' frame buffer which can be sent to the rPi as is: it has 18 bytes
    // reserved at the beginning for a header, and 2 bytes of padding to make it align to 32bits
    // It includes the lepton-provided header block as well as the raw pixel frame data.
    // At any given time during frame processing, it will hold one of the two frame-buffer structs,
    // and the core1 lepton frame acquisition task will write pixels to the other.
    let mut thread_local_frame_buffer: Option<&mut FrameBuffer> = None;

    // Create the GZIP crc table once on startup, then reuse, since it's expensive to
    // re-create inline.
    let crc_table = make_crc_table();

    // Copy huffman table into ram for faster access.
    let mut huffman_table = [HuffmanEntry { code: 0, bits: 0 }; 257];
    huffman_table.copy_from_slice(&HUFFMAN_TABLE[..]);

    sio.fifo
        .write_blocking(Core0Task::ReadyToReceiveLeptonConfig.into());
    let cmd = sio.fifo.read_blocking();
    assert_eq!(cmd, Core0Task::SendIntercoreArray.into());
    let length = sio.fifo.read_blocking();
    assert_eq!(length, 4);
    let radiometry_enabled = sio.fifo.read_blocking();
    let lepton_serial = sio.fifo.read_blocking();
    let lepton_firmware_main_version = sio.fifo.read_blocking();
    let lepton_firmware_dsp_version = sio.fifo.read_blocking();
    let firmware_main_parts = lepton_firmware_main_version.to_le_bytes();
    let firmware_dsp_parts = lepton_firmware_dsp_version.to_le_bytes();
    let lepton_version = if radiometry_enabled == 2 { 35 } else { 3 };
    let lepton_firmware_version = (
        (
            firmware_main_parts[0],
            firmware_main_parts[1],
            firmware_main_parts[2],
        ),
        (
            firmware_dsp_parts[0],
            firmware_dsp_parts[1],
            firmware_dsp_parts[2],
        ),
    );

    // TODO: Send CameraConnectInfo
    let has_cptv_files_saved = fs.has_cptv_files();

    let current_recording_window = config.next_or_current_recording_window(&time.get_date_time());

    info!(
        "Has cptv files? {} has files? {}",
        has_cptv_files_saved,
        fs.has_files_to_offload()
    );

    if let Some(audio_alarm) = next_audio_alarm {
        info!(
            "Alarm scheduled for {} {}:{}",
            audio_alarm.day(),
            audio_alarm.hour(),
            audio_alarm.minute()
        );
    }

    let mut bk = {
        let made_shutdown_status_recording = !config
            .time_is_in_recording_window(&time.get_date_time(), Some(current_recording_window));
        let made_startup_status_recording = has_cptv_files_saved;
        BookkeepingState {
            audio_pending: false,
            frame_checkpoint: 0,
            is_recording_on_rpi: false,
            made_shutdown_status_recording,
            making_status_recording: false,
            lost_frame_count: 0,
            made_startup_status_recording,
            next_audio_alarm,
            low_power_mode: config.use_low_power_mode(),

            logged_frame_transfer: Some(()),
            logged_told_rpi_to_sleep: Some(()),
            logged_pi_powered_down: Some(()),
            logged_fs_nearly_full: Some(()),
        }
    };

    warn!("Core 0 is ready to receive frames");
    sio.fifo.write_blocking(Core0Task::Ready.into());
    if !config.config().use_low_power_mode {
        sio.fifo.write_blocking(Core0Task::HighPowerMode.into());
    }
    let mut cptv_stream: Option<CptvStream> = None;

    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];
    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame_2: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];

    let mut frames_written = 0u32;
    let mut frames_seen = 0usize;
    let max_length_in_frames = if DEV_MODE { 60 * 9 } else { 60 * 10 * 9 };
    let mut prev_telemetry: Option<Telemetry> = None;
    let mut stable_telemetry_tracker = ([0u8, 0u8], -1);

    let mut is_daytime = config.time_is_in_daylight(&time.get_date_time());

    info!(
        "Current time is in recording window? {}",
        config.time_is_in_recording_window(&time.get_date_time(), None)
    );

    let mut motion_detection: Option<MotionTracking> = None;

    // NOTE: If there are already recordings on the flash memory,
    //  assume we've already made the startup status recording during this recording window.

    let mut status_recording_state = if bk.made_startup_status_recording {
        StatusRecording::NotPending
    } else {
        StatusRecording::StartupStatus
    };
    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    info!(
        "Entering frame loop made start up? {}",
        bk.made_startup_status_recording
    );
    loop {
        let input = sio.fifo.read_blocking();
        let needs_ffc = if input == Core0Task::ReceiveFrameWithPendingFFC.into() {
            true
        } else {
            crate::assert_eq!(
                input,
                Core0Task::ReceiveFrame.into(),
                "Got unknown fifo input to core1 task loop {}",
                input
            );
            false
        };

        // Get the currently selected buffer to transfer/write to disk.
        let selected_frame_buffer = sio.fifo.read_blocking();
        critical_section::with(|cs| {
            // Now we just swap the buffers
            let buffer = if selected_frame_buffer == 0 {
                static_frame_buffer_a
            } else {
                static_frame_buffer_b
            };
            thread_local_frame_buffer = buffer.borrow_ref_mut(cs).take();
        });

        let (telemetry, frame_is_valid) = process_frame_telemetry(
            &mut bk,
            &mut thread_local_frame_buffer,
            &prev_telemetry,
            &mut stable_telemetry_tracker,
            cptv_stream.is_some(),
        );

        // if in high power mode need to check thermal-recorder hasn't made a recording
        if needs_ffc && config.use_high_power_mode() {
            // only check status every 20 seconds
            if bk.twenty_seconds_elapsed_since_last_check(&telemetry) {
                if let Ok(is_recording) = i2c.get_is_recording(&mut delay) {
                    bk.is_recording_on_rpi = is_recording;
                    bk.frame_checkpoint = telemetry.frame_num;
                    if bk.is_recording_on_rpi {
                        sio.fifo.write(Core0Task::StartRecording.into());
                    } else {
                        sio.fifo.write(Core0Task::EndRecording.into());
                        thread_local_frame_buffer
                            .as_mut()
                            .unwrap()
                            .ffc_imminent(true);
                    }
                }
            } else if !bk.is_recording_on_rpi {
                // depending on timing might get 2 frames with needs_ffc event after said ok
                if let Some(fb) = thread_local_frame_buffer.as_mut() {
                    fb.ffc_imminent(true);
                }
            }
        } else if bk.is_recording_on_rpi {
            bk.is_recording_on_rpi = false;
            bk.reset_checkpoint();
        }

        let frame_transfer_start = timer.get_counter();
        // Transfer RAW frame to pi if it is available.
        let transfer = if frame_is_valid {
            pi_spi.begin_message_pio(
                ExtTransferMessage::CameraRawFrameTransfer,
                thread_local_frame_buffer
                    .as_mut()
                    .unwrap()
                    .as_u8_slice_mut(),
                0,
                cptv_stream.is_some(),
                &mut dma,
            )
        } else {
            None
        };
        if frame_is_valid && bk.logged_frame_transfer.take().is_some() {
            if transfer.is_some() {
                events.log(Event::StartedSendingFramesToRpi, &time, &mut fs);
            } else {
                events.log(Event::StartedGettingFrames, &time, &mut fs);
            }
        }

        let frame_buffer = &mut thread_local_frame_buffer
            .as_mut()
            .unwrap()
            .frame_data_as_u8_slice_mut();

        // Telemetry skipped
        let current_raw_frame =
            &bytemuck::cast_slice(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT];

        let too_close_to_ffc_event = telemetry.msec_since_last_ffc < 20000
            || telemetry.ffc_status == FFCStatus::Imminent
            || telemetry.ffc_status == FFCStatus::InProgress;
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        if too_close_to_ffc_event && motion_detection.is_some() {
            warn!("Resetting motion detection due to FFC event");
            frames_seen = 0;
            motion_detection = None;
        }
        let frame_output_stable = frames_seen >= WAIT_N_FRAMES_FOR_STABLE;
        // NOTE: In low power mode, don't try to start recordings/motion detection until frames have stabilised.
        let should_record_to_flash =
            frame_output_stable && config.use_low_power_mode() && frame_is_valid;
        let past_ffc_event = !too_close_to_ffc_event;
        // FIXME: What happens if we're in the middle of recording a cptv file and
        //  we get an invalid frame header?  Maybe this doesn't happen in practice?

        if should_record_to_flash {
            // just want 1 previous frame for first status
            if past_ffc_event {
                let this_frame_motion_detection = track_motion(
                    current_raw_frame,
                    &prev_frame,
                    &motion_detection,
                    is_daytime,
                    &config.motion_detection_mask,
                );

                should_start_new_recording = fs.can_begin_new_cptv_recordings()
                    && this_frame_motion_detection.got_new_trigger()
                    && cptv_stream.is_none();

                if bk.made_startup_status_recording
                    && !bk.made_shutdown_status_recording
                    && status_recording_state.is_not_pending()
                {
                    if DEV_MODE {
                        if time.get_date_time() + Duration::minutes(1)
                            > startup_date_time_utc + Duration::minutes(4)
                        {
                            warn!("Make shutdown status recording");
                            status_recording_state = StatusRecording::ShutdownStatus;
                        }
                    } else {
                        let (_, window_end) = &current_recording_window;
                        if &(time.get_date_time() + Duration::minutes(1)) > window_end {
                            warn!("Make shutdown status recording");
                            status_recording_state = StatusRecording::ShutdownStatus;
                        }
                    }
                }

                if status_recording_state.is_pending()
                    && !should_start_new_recording
                    && cptv_stream.is_none()
                {
                    should_start_new_recording = true;
                    bk.making_status_recording = true;
                }

                let should_end_current_recording = if cptv_stream.is_some() {
                    let (recording_is_max_length, storage_insufficient, motion_ended) =
                        if bk.making_status_recording {
                            (frames_written >= max_length_in_frames, fs.is_full(), false)
                        } else {
                            (
                                frames_written >= NUM_STATUS_RECORDING_FRAMES,
                                fs.is_nearly_full_for_thermal_recordings(),
                                this_frame_motion_detection.triggering_ended(),
                            )
                        };
                    motion_ended || recording_is_max_length || storage_insufficient
                } else {
                    false
                };

                motion_detection = Some(this_frame_motion_detection);

                if should_start_new_recording {
                    // NOTE: Rather than trying to get the RTC time right as we're trying to start a CPTV file,
                    //  we just get it periodically, and then each frame add to it, then re-sync it
                    // (when we do our once a minute checks) when we're *not* trying to start a recording.
                    let is_inside_recording_window = if DEV_MODE {
                        // Recording window is 5 minutes from startup time in dev mode.
                        time.get_date_time() < startup_date_time_utc + Duration::minutes(5)
                    } else {
                        config.time_is_in_recording_window(
                            &time.get_date_time(),
                            Some(current_recording_window),
                        )
                    };
                    // start recording below so frame buffer is out of scope
                    should_start_new_recording =
                        should_start_new_recording && is_inside_recording_window;
                    if is_inside_recording_window {
                        // Should we make a 2-second status recording at the beginning or end of the window?
                        // if !made_startup_status_recording && !motion_detection_triggered_this_frame
                        // {
                        //     warn!("Make startup status recording");
                        //     made_startup_status_recording = true;
                        //     making_status_recording = true;
                        // } else {
                        //     // We're making a shutdown recording.
                        // }
                    } else if !bk.making_status_recording {
                        info!("Would start recording, but outside recording window");
                    } else if bk.made_startup_status_recording && !bk.made_shutdown_status_recording
                    {
                        should_start_new_recording = true;
                        // force shutdown status recording even outside of window
                    } else {
                        bk.making_status_recording = false;
                    }
                    if bk.making_status_recording {
                        info!("Making status recording {}", status_recording_state);
                    }
                } else if !should_end_current_recording {
                    if let Some(cptv_stream) = &mut cptv_stream {
                        warn_on_duplicate_frames(&prev_telemetry, &telemetry);
                        cptv_stream.push_frame(
                            current_raw_frame,
                            &mut prev_frame,
                            &telemetry,
                            &mut fs,
                        );
                        frames_written += 1;
                    }
                } else {
                    // Finalise on a different frame period to writing out the prev/last frame,
                    // to give more breathing room.
                    if let Some(cptv_stream) = &mut cptv_stream {
                        let cptv_start_block_index = cptv_stream.starting_block_index;

                        if !bk.making_status_recording
                            && motion_detection.as_ref().unwrap().was_false_positive()
                        {
                            info!(
                                "Discarding as a false-positive {}:{} ",
                                cptv_start_block_index, fs.last_used_block_index
                            );
                            let _ = fs.erase_last_file();
                            events.log(Event::WouldDiscardAsFalsePositive, &time, &mut fs);
                        } else {
                            cptv_stream.finalise(&mut fs);
                            error!(
                                "Ending current recording start block {} end block{}",
                                cptv_start_block_index, fs.last_used_block_index
                            );
                            events.log(Event::EndedRecording, &time, &mut fs);

                            if bk.lost_frame_count > 0 {
                                events.log(
                                    Event::LostFrames(u64::from(bk.lost_frame_count)),
                                    &time,
                                    &mut fs,
                                );
                            }
                        }

                        // Clear out prev frame before starting a new recording stream.
                        prev_frame_2.fill(0);

                        ended_recording = true;
                        let _ = i2c
                            .set_recording_flag(&mut delay, false)
                            .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));

                        if bk.making_status_recording {
                            bk.making_status_recording = false;
                            match status_recording_state {
                                StatusRecording::StartupStatus => {
                                    bk.made_startup_status_recording = true;
                                    // only make a shutdown if we made a startup
                                    bk.made_shutdown_status_recording = false;
                                }
                                StatusRecording::ShutdownStatus => {
                                    bk.made_shutdown_status_recording = true;
                                }
                                StatusRecording::NotPending => {}
                            }
                            status_recording_state = StatusRecording::NotPending;
                        }
                    }
                    cptv_stream = None;
                    frames_written = 0;
                    motion_detection = None;
                }
            } else {
                should_start_new_recording = false;
            }

            // if starting a new recording will handle this differently below
            if !should_start_new_recording {
                prev_frame.copy_from_slice(current_raw_frame);
            }
        }

        if let Some((transfer, transfer_end_address, transfer_start_address)) = transfer {
            let did_abort_transfer =
                pi_spi.end_message(&dma, transfer_end_address, transfer_start_address, transfer);
            if did_abort_transfer {
                warn!(
                    "Transfer aborted for frame #{}, pi must be asleep?",
                    telemetry.frame_num
                );
            }
        }
        let frame_transfer_end = timer.get_counter();
        if should_start_new_recording {
            // Since we write 2 frames every new recording, this can take too long and
            // we drop a frame so cache the current frame and tell core1 to keep getting
            // lepton frames, this will spread the initial load over the first 2 frames.
            warn!("Setting recording flag on attiny");
            let _ = i2c
                .set_recording_flag(&mut delay, true)
                .map_err(|e| error!("Error setting recording flag on attiny: {}", e));

            error!("Starting new recording, {:?}", &telemetry);
            let mut cptv_streamer = CptvStream::new(
                time.get_timestamp_micros(), // Microseconds
                lepton_version,
                lepton_serial,
                lepton_firmware_version,
                config,
                &mut fs,
                &huffman_table,
                &crc_table,
                bk.making_status_recording,
            );
            cptv_streamer.init_gzip_stream(&mut fs, false);

            // Write out the initial frame from *before* the time when the motion detection triggered.
            cptv_streamer.push_frame(
                &prev_frame,
                &mut prev_frame_2, // This should be zeroed out before starting a new clip.
                prev_telemetry.as_ref().unwrap(),
                &mut fs,
            );
            frames_written += 1;

            events.log(Event::StartedRecording, &time, &mut fs);

            prev_frame_2.copy_from_slice(current_raw_frame);
            // release the buffer before writing the frame so core1 can continue working
            critical_section::with(|cs| {
                // Now we just swap the buffers?
                let buffer = if selected_frame_buffer == 0 {
                    static_frame_buffer_a
                } else {
                    static_frame_buffer_b
                };
                *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
            });
            sio.fifo.write(Core0Task::StartRecording.into());
            info!("Sent start recording message to core1");
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());

            warn_on_duplicate_frames(&prev_telemetry, &telemetry);
            // now write the second/current frame
            cptv_streamer.push_frame(&prev_frame_2, &mut prev_frame, &telemetry, &mut fs);
            frames_written += 1;
            prev_frame.copy_from_slice(&prev_frame_2);
            cptv_stream = Some(cptv_streamer);
        } else {
            critical_section::with(|cs| {
                // Now we just swap the buffers?
                let buffer = if selected_frame_buffer == 0 {
                    static_frame_buffer_a
                } else {
                    static_frame_buffer_b
                };
                *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
            });
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());
        }

        if ended_recording && cptv_stream.is_none() {
            info!("Send end recording message to core0");
            sio.fifo.write(Core0Task::EndRecording.into());
        }

        if frames_seen % (10 * 9) == 0 && frame_is_valid {
            info!("Got frame #{}", telemetry.frame_num);
        }

        let expected_rtc_sync_time_us = 4200u32; // using slower clock speed
        let not_recording_and_every_minute_interval_arrived =
            (frames_seen > 1 && frames_seen % (60 * 9) == 0) && cptv_stream.is_none();
        // INFO RTC Sync time took 1350µs
        // NOTE: Every minute when we're not recording, we try to re-sync our time with the RTC module,
        //  as well as checking if we need to tell the rPi to shutdown if we're in low-power mode.
        //  We also check if the flash storage is nearly full, and if so, we request a restart to offload
        //  files to the rPi.
        //  We also check if we need to restart to make an audio recording.
        //  If we're in low power mode and outside the recording window, we set an alarm and ask
        //  the Attiny to put us to sleep.
        //  The reason we only do all this once per minute is because the i2c comms can take longer than
        //  expected, and cause us to lose sync.

        if not_recording_and_every_minute_interval_arrived {
            is_daytime = config.time_is_in_daylight(&time.get_date_time());
            let sync_rtc_start = timer.get_counter();
            // NOTE: We only advise the RPi that it can shut down if we're not currently recording –
            //  since the change in frame times can affect our frame sync.  It's fine to call this repeatedly,
            //  the RPi will shut down when it wants to.

            if config.use_low_power_mode() {
                // Once per minute, if we're not currently recording, tell the RPi it can shut down, as it's not
                // needed in low-power mode unless it's offloading/uploading CPTV data.
                advise_raspberry_pi_it_may_shutdown(&mut i2c, &mut delay);
                if bk.logged_told_rpi_to_sleep.take().is_some() {
                    events.log(Event::ToldRpiToSleep, &time, &mut fs);
                }
                info!(
                    "Advise pi to shutdown took {}µs",
                    (timer.get_counter() - sync_rtc_start).to_micros()
                );
            }
            let sync_rtc_start_real = timer.get_counter();

            match i2c.get_datetime(&mut delay) {
                Ok(now) => time.set(get_datetime_utc(now)),
                Err(err_str) => {
                    events.log(Event::RtcCommError, &time, &mut fs);
                    error!("Unable to get DateTime from RTC: {}", err_str);
                }
            }

            info!(
                "RTC Sync time took {}µs",
                (timer.get_counter() - sync_rtc_start_real).to_micros()
            );

            // NOTE: In continuous recording mode, the device will only shut down briefly when the flash storage
            //  is nearly full, and it needs to offload files.  Or, in the case of non-low-power-mode, it will
            //  never shut down.
            let is_outside_recording_window = if DEV_MODE {
                let is_inside_recording_window =
                    time.get_date_time() < startup_date_time_utc + Duration::minutes(5);
                !is_inside_recording_window
            } else {
                !config.time_is_in_recording_window(&time.get_date_time(), None)
            };
            let is_inside_recording_window = !is_outside_recording_window;

            let fs_nearly_full = fs.is_too_full_to_start_new_cptv_recordings();
            if fs_nearly_full && bk.logged_fs_nearly_full.take().is_some() {
                events.log(Event::FlashStorageNearlyFull, &time, &mut fs);
            }
            if fs_nearly_full {
                // Request restart to offload.
                restart(&mut sio);
            }

            if is_outside_recording_window
                && (bk.use_high_power_mode() || bk.made_shutdown_status_recording)
            {
                if fs.has_files_to_offload() {
                    // If flash storage is nearly full, or we're now outside the recording window,
                    // Trigger a restart now via the watchdog timer, so that flash storage will
                    // be offloaded during the startup sequence.
                    warn!("Recording window ended with files to offload, request restart");
                    restart(&mut sio);
                }

                if bk.use_high_power_mode() {
                    // Tell rPi it is outside its recording window in *non*-low-power mode, and can go to sleep.
                    advise_raspberry_pi_it_may_shutdown(&mut i2c, &mut delay);
                    if bk.logged_told_rpi_to_sleep.take().is_some() {
                        events.log(Event::ToldRpiToSleep, &time, &mut fs);
                    }
                }
                shutdown_if_possible(
                    &timer,
                    &mut i2c,
                    &time,
                    &mut fs,
                    &mut events,
                    &mut delay,
                    &mut bk,
                    config,
                    &mut sio,
                );
            } else if is_inside_recording_window && bk.use_high_power_mode() {
                // This obviously blocks for a long time, if the rPi is asleep,
                // but is essentially instant if it is already awake and ready.
                if wake_raspberry_pi(&mut i2c, &mut delay) {
                    events.log(
                        Event::ToldRpiToWake(WakeReason::ThermalHighPower),
                        &time,
                        &mut fs,
                    );
                }
            } else if is_outside_recording_window && !bk.made_shutdown_status_recording {
                bk.making_status_recording = true;
                // force shutdown recording outside of window
            }

            // Make sure timing is as close as possible to the non-sync case
            let sync_rtc_end = timer.get_counter();
            #[allow(clippy::cast_possible_truncation)]
            let sync_time = (sync_rtc_end - sync_rtc_start).to_micros() as u32;
            let additional_wait = expected_rtc_sync_time_us.saturating_sub(sync_time);
            if additional_wait > 0 {
                warn!(
                    "Additional wait after RTC sync {}µs, total sync time {}",
                    additional_wait, sync_time
                );
                delay.delay_us(additional_wait);
            } else {
                warn!("I2C messages took {}µs", sync_time);
            }
        } else {
            advance_time_by_n_frames(&telemetry, &prev_telemetry, &mut time, frame_is_valid);
            // Spend the same time as we would otherwise use querying the RTC to keep frame-times
            // about the same
            delay.delay_us(expected_rtc_sync_time_us);
        }

        if should_restart_to_make_an_audio_recording(
            &mut bk,
            cptv_stream.is_some(),
            &time,
            &current_recording_window.1,
            &telemetry,
            &mut i2c,
            &mut delay,
        ) {
            info!("Taking audio recording");
            // make audio rec now
            if i2c.tc2_agent_take_audio_rec(&mut delay).is_ok() {
                restart(&mut sio);
            }
        }

        if frame_is_valid {
            prev_telemetry = Some(telemetry);
        }
        frames_seen += 1;
    }
}

#[allow(clippy::ref_option)]
fn advance_time_by_n_frames(
    telemetry: &Telemetry,
    prev_telemetry: &Option<Telemetry>,
    time: &mut SyncedDateTime,
    frame_is_valid: bool,
) {
    // Increment the datetime n frame's worth.
    // NOTE: We only get the actual date from the RTC every minutes' worth of frames, so that we
    //  don't have too many stalls trying to communicate via I2C with the RTC.
    let frames_elapsed = if frame_is_valid {
        get_frames_elapsed(telemetry, prev_telemetry)
    } else {
        1
    };
    if frames_elapsed > 100 {
        warn!("Got {} elapsed frames, ignoring", frames_elapsed);
    } else {
        time.set(time.get_date_time() + Duration::milliseconds(115 * i64::from(frames_elapsed)));
    }
}

fn shutdown_if_possible(
    timer: &Timer,
    i2c: &mut SharedI2C,
    time: &SyncedDateTime,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    delay: &mut Delay,
    bk: &mut BookkeepingState,
    config: &DeviceConfig,
    sio: &mut Sio,
) {
    let check_power_down_state_start = timer.get_counter();
    if let Ok(pi_is_powered_down) = i2c.pi_is_powered_down(delay, true) {
        if pi_is_powered_down {
            if bk.logged_pi_powered_down.take().is_some() {
                info!("Pi is now powered down: {}", pi_is_powered_down);
                events.log(Event::GotRpiPoweredDown, time, fs);
            }

            // FIXME: I don't understand this logic
            // only reboot into audio mode if pi is powered down and rp2040 is asleep
            // so we can keep the thermal preview up as long as the PI is on.
            match config.audio_mode() {
                AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                    // just reboot and it will go into audio branch
                    info!("Reset as thermal finished and time for audio");
                    restart(sio);
                }
                _ => (),
            }

            // NOTE: Calculate the start of the next recording window, set the RTC wake-up alarm,
            //  and ask for the rp2040 to be put to sleep.
            let next_recording_window_start = if DEV_MODE {
                // In dev mode, we always set the restart alarm for 2 minutes time.
                time.get_date_time() + Duration::minutes(2)
            } else {
                config.next_recording_window_start(&time.get_date_time())
            };
            let enabled_alarm = i2c.enable_alarm(delay);
            if enabled_alarm.is_err() {
                error!("Failed enabling alarm");
                events.log(Event::RtcCommError, time, fs);
            }
            if i2c
                .set_wakeup_alarm(&next_recording_window_start, delay)
                .is_ok()
            {
                let alarm_enabled = i2c.alarm_interrupt_enabled(delay).unwrap_or(false);
                info!("Wake up alarm interrupt enabled {}", alarm_enabled);
                if alarm_enabled {
                    events.log(
                        Event::SetAlarm(next_recording_window_start.timestamp_micros()),
                        time,
                        fs,
                    );

                    info!("Tell core1 to get ready to sleep");
                    // Tell core1 we're exiting the recording loop, and it should
                    // power down the lepton module, and wait for reply.
                    sio.fifo.write(Core0Task::ReadyToSleep.into());
                    loop {
                        if let Some(result) = sio.fifo.read() {
                            if result == 255 {
                                // FIXME: Document 255
                                break;
                            }
                        }
                    }
                    info!("Ask Attiny to power down rp2040");
                    events.log(Event::Rp2040Sleep, time, fs);
                    if i2c.tell_attiny_to_power_down_rp2040(delay).is_ok() {
                        info!("Sleeping");
                    } else {
                        error!("Failed sending sleep request to attiny");
                        events.log(Event::AttinyCommError, time, fs);
                    }
                } else {
                    error!("Alarm was not properly enabled");
                    events.log(Event::RtcCommError, time, fs);
                }
                // Now we can put ourselves to sleep.
            } else {
                error!("Failed setting wake alarm, can't go to sleep");
                events.log(Event::RtcCommError, time, fs);
            }
        } else {
            warn!("Pi is still awake, so rp2040 must stay awake");
        }
    } else {
        warn!("Failed to get Pi powered down state from Attiny");
    }
    warn!(
        "Check pi power down state took {}µs",
        (timer.get_counter() - check_power_down_state_start).to_micros()
    );
}

fn should_restart_to_make_an_audio_recording(
    bk: &mut BookkeepingState,
    is_recording_cptv: bool,
    time: &SyncedDateTime,
    window_end: &DateTime<Utc>,
    telemetry: &Telemetry,
    i2c: &mut SharedI2C,
    delay: &mut Delay,
) -> bool {
    if let Some(next_audio_alarm) = bk.next_audio_alarm {
        if !bk.audio_pending {
            let current_time = time.get_date_time();
            bk.audio_pending = current_time > next_audio_alarm;
            info!(
                "Audio recording is pending because time {}:{} is after {}:{} ",
                current_time.hour(),
                current_time.minute(),
                next_audio_alarm.hour(),
                next_audio_alarm.minute()
            );
        }

        if !is_recording_cptv
            && bk.audio_pending
            && bk.twenty_seconds_elapsed_since_last_check(telemetry)
        {
            bk.frame_checkpoint = telemetry.frame_num;
            let duration_until_window_end = *window_end - time.get_date_time();
            // If within 2 minutes of the window end, make status before doing audio recording:
            // this ensures we always make the status recording.
            let need_to_make_shutdown_status_recording = bk.use_low_power_mode()
                && !bk.made_shutdown_status_recording
                && duration_until_window_end <= Duration::minutes(2);

            if need_to_make_shutdown_status_recording {
                info!("Pending audio recording, but deferring until after shutdown status");
            } else {
                // handles case where thermal recorder on the rPi is doing recording
                bk.is_recording_on_rpi = bk.use_high_power_mode()
                    && i2c
                        .get_is_recording(delay)
                        .is_ok_and(|is_recording| is_recording);
                if !bk.is_recording_on_rpi {
                    return true;
                }
            }
        }
    }
    false
}

fn restart(sio: &mut Sio) {
    sio.fifo.write(Core0Task::RequestReset.into());
    loop {
        // Wait to be reset
        nop();
    }
}

#[allow(clippy::ref_option)]
fn warn_on_duplicate_frames(prev_frame_telemetry: &Option<Telemetry>, frame_telemetry: &Telemetry) {
    if let Some(prev_telemetry) = &prev_frame_telemetry {
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
}

#[allow(clippy::ref_option)]
fn process_frame_telemetry(
    bk: &mut BookkeepingState,
    frame_buffer: &mut Option<&mut FrameBuffer>,
    prev_telemetry: &Option<Telemetry>,
    stable_telemetry_tracker: &mut ([u8; 2], i8),
    is_recording: bool,
) -> (Telemetry, bool) {
    let frame_buffer = frame_buffer.as_mut().unwrap().frame_data_as_u8_slice_mut();
    // Read the telemetry:
    let frame_telemetry = Telemetry::from_bytes(frame_buffer);
    let mut skipped_frames = 0;
    if let Some(prev_telemetry) = &prev_telemetry {
        let frame_diff = frame_telemetry.frame_num - prev_telemetry.frame_num - 1;
        // over a 100 is probably corrupt telemetry
        if frame_diff > 0 && frame_diff < 100 {
            skipped_frames = frame_diff;
        }
    }

    if is_recording {
        // if recording accumulate
        bk.lost_frame_count += skipped_frames;
    } else {
        bk.lost_frame_count = skipped_frames;
    }
    // Sometimes we get an invalid frame header on this thread; we detect and ignore these frames.
    let frame_header_is_valid = frame_telemetry.is_valid(stable_telemetry_tracker);
    (frame_telemetry, frame_header_is_valid)
}

#[allow(clippy::ref_option)]
fn get_frames_elapsed(
    frame_telemetry: &Telemetry,
    prev_frame_telemetry: &Option<Telemetry>,
) -> u32 {
    if let Some(prev_telemetry) = prev_frame_telemetry {
        let frames_elapsed = frame_telemetry
            .frame_num
            .saturating_sub(prev_telemetry.frame_num);
        if frames_elapsed > 1 {
            warn!(
                "Lost {} frame(s), got {}, prev was {}",
                frames_elapsed - 1,
                frame_telemetry.frame_num,
                prev_telemetry.frame_num
            );
        }
        frames_elapsed
    } else {
        1
    }
}
