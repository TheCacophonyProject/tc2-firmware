#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::SharedI2C;
use crate::bsp::pac::DMA;
use crate::cptv_encoder::huffman::{HUFFMAN_TABLE, HuffmanEntry};
use crate::cptv_encoder::streaming_cptv::{CptvStream, make_crc_table};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{AudioMode, DeviceConfig, get_datetime_utc};
use crate::event_logger::{EventLogger, LoggerEvent, LoggerEventKind, WakeReason};
use core::cell::RefCell;

use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage, RPI_TRANSFER_HEADER_LENGTH};
use crate::lepton::{FFCStatus, Telemetry, read_telemetry};
use crate::motion_detector::{MotionTracking, track_motion};
use crate::onboard_flash::OnboardFlash;
use chrono::{DateTime, Datelike, Duration, NaiveDateTime, Timelike, Utc};

use crate::synced_date_time::SyncedDateTime;
use core::ops::Add;
use cortex_m::asm::nop;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::{Format, error, info, warn};
use rp2040_hal::Sio;

pub const NUM_LEPTON_SEGMENTS: usize = 4;
pub const NUM_LINES_PER_LEPTON_SEGMENT: usize = 61;
const FRAME_BUFFER_ALIGNMENT_PADDING: usize = 2;
const LEPTON_RAW_FRAME_PAYLOAD_LENGTH: usize =
    FRAME_WIDTH * NUM_LINES_PER_LEPTON_SEGMENT * NUM_LEPTON_SEGMENTS;
const FRAME_BUFFER_LENGTH: usize =
    RPI_TRANSFER_HEADER_LENGTH + LEPTON_RAW_FRAME_PAYLOAD_LENGTH + FRAME_BUFFER_ALIGNMENT_PADDING;

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

/// Returns `true` if it did wake the rPI
pub fn wake_raspberry_pi(shared_i2c: &mut SharedI2C, delay: &mut Delay) -> bool {
    if let Ok(true) = shared_i2c.pi_is_powered_down(delay, false) {
        if shared_i2c.tell_pi_to_wakeup(delay).is_ok() {
            // TODO: Log here if this was an unexpected wakeup
            warn!("Sent wake signal to raspberry pi");
            // Poll to see when tc2-agent is ready.
            loop {
                if let Ok(pi_is_awake) = shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, true)
                {
                    if pi_is_awake {
                        break;
                    }
                    // Try to wake it again, just in case it was shutdown behind our backs.
                    let _ = shared_i2c.tell_pi_to_wakeup(delay);
                }
                delay.delay_ms(1000);
            }
            true
        } else {
            warn!("Failed to send wake signal to raspberry pi");
            false
        }
    } else {
        loop {
            if let Ok(pi_is_awake) = shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, false) {
                if pi_is_awake {
                    break;
                }
                // Try to wake it again, just in case it was shutdown behind our back.
                let _ = shared_i2c.tell_pi_to_wakeup(delay);
            }
            delay.delay_ms(1000);
        }
        false
    }
}

pub fn advise_raspberry_pi_it_may_shutdown(shared_i2c: &mut SharedI2C, delay: &mut Delay) {
    if shared_i2c.tell_pi_to_shutdown(delay).is_err() {
        error!("Error sending power-down advice to raspberry pi");
    } else {
        info!("Sent power-down advice to raspberry pi");
    }
}

fn is_frame_telemetry_is_valid(
    frame_telemetry: &Telemetry,
    telemetry_revision_stable: &mut ([u8; 2], i8),
) -> bool {
    // Sometimes the header is invalid, but the frame becomes valid and gets sync.
    // Because the telemetry revision is static across frame headers we can detect this
    // case and not send the frame, as it may cause false triggers.
    if telemetry_revision_stable.1 > -1 && telemetry_revision_stable.1 <= 2 {
        if telemetry_revision_stable.0[0] == frame_telemetry.revision[0]
            && telemetry_revision_stable.0[1] == frame_telemetry.revision[1]
        {
            telemetry_revision_stable.1 += 1;
        } else {
            telemetry_revision_stable.1 = -1;
        }
        if telemetry_revision_stable.1 > 2 {
            info!("Got stable telemetry revision (core 0) {:?}", frame_telemetry.revision);
        }
    }
    if telemetry_revision_stable.1 == -1 {
        // Initialise seen telemetry revision.
        telemetry_revision_stable.0 = [frame_telemetry.revision[0], frame_telemetry.revision[1]];
        telemetry_revision_stable.1 += 1;
    }
    if telemetry_revision_stable.1 < 2 {
        false
    } else if telemetry_revision_stable.1 > 2
        && (telemetry_revision_stable.0[0] != frame_telemetry.revision[0]
            || telemetry_revision_stable.0[1] != frame_telemetry.revision[1])
    {
        // We have a misaligned/invalid frame.
        warn!("Misaligned header (core 1)");
        false
    } else {
        true
    }
}

#[allow(clippy::too_many_lines)]
pub fn thermal_motion_task(
    mut delay: Delay,
    mut sio: Sio,
    mut dma: DMA,
    mut shared_i2c: SharedI2C,
    mut pi_spi: ExtSpiTransfers,
    mut flash_storage: OnboardFlash,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
    device_config: &DeviceConfig,
    mut event_logger: EventLogger,
    mut synced_date_time: SyncedDateTime,
    next_audio_alarm: Option<DateTime<Utc>>,
) -> ! {
    let dev_mode = false;
    info!("=== Core 0 Thermal Motion start ===");
    if dev_mode {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }
    let timer = synced_date_time.get_timer();
    {
        if flash_storage.has_files_to_offload() {
            info!("Finished scan, has files to offload");
        }
    }

    if event_logger.has_events_to_offload() {
        info!("There are {} event(s) to offload", event_logger.count());
    }

    event_logger.log_event(
        LoggerEvent::new(LoggerEventKind::ThermalMode, &synced_date_time),
        &mut flash_storage,
    );
    if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
        if is_recording {
            event_logger.log_event(
                LoggerEvent::new(LoggerEventKind::RecordingNotFinished, &synced_date_time),
                &mut flash_storage,
            );
        }
    }
    // Unset the is_recording flag on attiny on startup
    let _ = shared_i2c
        .set_recording_flag(&mut delay, false)
        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
    let startup_date_time_utc: DateTime<Utc> = synced_date_time.get_date_time();

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

    sio.fifo.write_blocking(Core0Task::ReadyToReceiveLeptonConfig.into());
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
        (firmware_main_parts[0], firmware_main_parts[1], firmware_main_parts[2]),
        (firmware_dsp_parts[0], firmware_dsp_parts[1], firmware_dsp_parts[2]),
    );

    // TODO: Send CameraConnectInfo

    let record_audio_any_time = device_config.config().audio_mode == AudioMode::AudioAndThermal;
    let record_audio_outside_thermal_window =
        device_config.config().audio_mode == AudioMode::AudioOrThermal;
    let record_audio = record_audio_outside_thermal_window || record_audio_any_time;
    let mut audio_pending: bool = false;

    let has_cptv_files_saved = flash_storage.has_cptv_files(false);
    let mut made_startup_status_recording = has_cptv_files_saved;

    let current_recording_window =
        device_config.next_or_current_recording_window(&synced_date_time.get_date_time());

    let mut made_shutdown_status_recording = !device_config.time_is_in_recording_window(
        &synced_date_time.get_date_time(),
        Some(current_recording_window),
    );

    info!(
        "Has cptv files? {} has files? {}",
        has_cptv_files_saved,
        flash_storage.has_files_to_offload()
    );

    if record_audio {
        if let Some(audio_alarm) = next_audio_alarm {
            info!(
                "Alarm scheduled for {} {}:{}",
                audio_alarm.day(),
                audio_alarm.hour(),
                audio_alarm.minute()
            );
        }
    }

    warn!("Core 0 is ready to receive frames");
    sio.fifo.write_blocking(Core0Task::Ready.into());
    if !device_config.config().use_low_power_mode {
        sio.fifo.write_blocking(Core0Task::HighPowerMode.into());
    }
    let mut cptv_stream: Option<CptvStream> = None;

    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];
    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame_2: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];

    let mut frames_written = 0;
    let mut frames_seen = 0usize;
    let mut prev_frame_telemetry: Option<Telemetry> = None;
    let mut stable_telemetry_tracker = ([0u8, 0u8], -1);

    let mut is_daytime = device_config.time_is_in_daylight(&synced_date_time.get_date_time());

    info!(
        "Current time is in recording window? {}",
        device_config.time_is_in_recording_window(&synced_date_time.get_date_time(), None)
    );

    let mut motion_detection: Option<MotionTracking> = None;
    let mut logged_frame_transfer = false;
    let mut logged_told_rpi_to_sleep = false;
    let mut logged_pi_powered_down = false;
    let mut logged_flash_storage_nearly_full = false;
    // NOTE: If there are already recordings on the flash memory,
    //  assume we've already made the startup status recording during this recording window.

    let mut making_status_recording = false;
    let mut status_recording_state = if made_startup_status_recording {
        StatusRecording::NotPending
    } else {
        StatusRecording::StartupStatus
    };

    let mut high_power_recording = false;

    // TODO: last_rec_check means?
    let mut last_rec_check = 0;
    let mut lost_frames = 0;
    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    info!("Entering frame loop made start up? {}", made_startup_status_recording);
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

        let start = timer.get_counter();
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
        let frame_swap_time = timer.get_counter();

        let (frame_telemetry, frame_header_is_valid) = {
            let frame_buffer =
                &mut thread_local_frame_buffer.as_mut().unwrap().frame_data_as_u8_slice_mut();
            // Read the telemetry:
            let frame_telemetry = read_telemetry(frame_buffer);
            let mut skipped_frames = 0;
            if let Some(prev_telemetry) = &prev_frame_telemetry {
                let frame_diff = frame_telemetry.frame_num - prev_telemetry.frame_num - 1;
                // over a 100 is probably corrupt telemetry
                if frame_diff > 0 && frame_diff < 100 {
                    skipped_frames = frame_diff;
                }
            }

            if cptv_stream.is_some() {
                // if recording accumulate
                lost_frames += skipped_frames;
            } else {
                lost_frames = skipped_frames;
            }
            // Sometimes we get an invalid frame header on this thread; we detect and ignore these frames.
            let frame_header_is_valid =
                is_frame_telemetry_is_valid(&frame_telemetry, &mut stable_telemetry_tracker);
            (frame_telemetry, frame_header_is_valid)
        };

        // if in high power mode need to check thermal-recorder hasn't made a recording
        if needs_ffc && !device_config.use_low_power_mode() {
            // only check status every 20 seconds
            if frame_telemetry.frame_num - last_rec_check > 9 * 20 {
                if let Ok(is_recording) = shared_i2c.tc2_agent_is_recording(&mut delay) {
                    high_power_recording = is_recording;
                    last_rec_check = frame_telemetry.frame_num;
                    info!(
                        "Checking if recording {} am recording ?? {}",
                        is_recording, high_power_recording
                    );
                    if high_power_recording {
                        sio.fifo.write(Core0Task::StartRecording.into());
                        high_power_recording = true;
                    } else {
                        sio.fifo.write(Core0Task::EndRecording.into());
                        high_power_recording = false;
                        thread_local_frame_buffer.as_mut().unwrap().ffc_imminent(true);
                    }
                }
            } else if !high_power_recording {
                // depending on timing might get 2 frames with needs_ffc event after said ok
                thread_local_frame_buffer.as_mut().unwrap().ffc_imminent(true);
            }
        } else if high_power_recording {
            high_power_recording = false;
            last_rec_check = 0;
        }

        let frame_transfer_start = timer.get_counter();
        // Transfer RAW frame to pi if it is available.
        let transfer = if frame_header_is_valid {
            pi_spi.begin_message_pio(
                ExtTransferMessage::CameraRawFrameTransfer,
                thread_local_frame_buffer.as_mut().unwrap().as_u8_slice_mut(),
                0,
                cptv_stream.is_some(),
                &mut dma,
            )
        } else {
            None
        };
        if !logged_frame_transfer && frame_header_is_valid {
            if transfer.is_some() {
                event_logger.log_event(
                    LoggerEvent::new(LoggerEventKind::StartedSendingFramesToRpi, &synced_date_time),
                    &mut flash_storage,
                );
            } else {
                event_logger.log_event(
                    LoggerEvent::new(LoggerEventKind::StartedGettingFrames, &synced_date_time),
                    &mut flash_storage,
                );
            }
            logged_frame_transfer = true;
        }

        let frame_buffer =
            &mut thread_local_frame_buffer.as_mut().unwrap().frame_data_as_u8_slice_mut();

        // Telemetry skipped
        let current_raw_frame =
            &bytemuck::cast_slice(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT];

        let frame_num = frame_telemetry.frame_num;
        let too_close_to_ffc_event = frame_telemetry.msec_since_last_ffc < 20000
            || frame_telemetry.ffc_status == FFCStatus::Imminent
            || frame_telemetry.ffc_status == FFCStatus::InProgress;
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        if too_close_to_ffc_event && motion_detection.is_some() {
            warn!("Resetting motion detection due to FFC event");
            frames_seen = 0;
            motion_detection = None;
        }
        // NOTE: In low power mode, don't try to start recordings/motion detection until frames have stabilised.
        let should_record_to_flash = frames_seen >= WAIT_N_FRAMES_FOR_STABLE
            && device_config.use_low_power_mode()
            && frame_header_is_valid;
        if should_record_to_flash {
            // just want 1 previous frame for first status
            if !too_close_to_ffc_event && frames_seen > WAIT_N_FRAMES_FOR_STABLE {
                let this_frame_motion_detection = track_motion(
                    current_raw_frame,
                    &prev_frame,
                    &motion_detection,
                    is_daytime,
                    &device_config.motion_detection_mask,
                );
                let max_length_in_frames = if dev_mode { 60 * 9 } else { 60 * 10 * 9 };
                let status_recording_length_in_frames = 18;
                let motion_detection_triggered_this_frame =
                    this_frame_motion_detection.got_new_trigger();

                should_start_new_recording = !flash_storage.is_too_full_to_start_new_recordings()
                    && motion_detection_triggered_this_frame
                    && cptv_stream.is_none(); // wait until lepton stabilises before recording

                if made_startup_status_recording
                    && !made_shutdown_status_recording
                    && status_recording_state.is_not_pending()
                {
                    if dev_mode {
                        if synced_date_time.get_date_time() + Duration::minutes(1)
                            > startup_date_time_utc + Duration::minutes(4)
                        {
                            warn!("Make shutdown status recording");
                            status_recording_state = StatusRecording::ShutdownStatus;
                        }
                    } else {
                        let (_, window_end) = &current_recording_window;
                        if &(synced_date_time.get_date_time() + Duration::minutes(1)) > window_end {
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
                    making_status_recording = true;
                }

                // TODO: Do we want to have a max recording length timeout, or just pause recording if a subject stays in the frame
                //  but doesn't move for a while?  Maybe if a subject is stationary for 1 minute, we pause, and only resume
                //  recording if there is new movement, or it moves again?  If the night ends in this way, we end the recording then.
                //  In continuous recording mode we'd have a really long timeout perhaps?  Needs more thought.
                //  Also consider the case where we have a mask region to ignore or pay attention to.
                let should_end_current_recording = cptv_stream.is_some()
                    && ((this_frame_motion_detection.triggering_ended()
                        || frames_written >= max_length_in_frames
                        || (!making_status_recording && flash_storage.is_nearly_full()))
                        || (making_status_recording
                            && (frames_written >= status_recording_length_in_frames
                                || flash_storage.is_full())));

                motion_detection = Some(this_frame_motion_detection);

                if should_start_new_recording {
                    // NOTE: Rather than trying to get the RTC time right as we're trying to start a CPTV file,
                    //  we just get it periodically, and then each frame add to it, then re-sync it
                    // (when we do our once a minute checks) when we're *not* trying to start a recording.
                    let is_inside_recording_window = if dev_mode {
                        // Recording window is 5 minutes from startup time in dev mode.
                        synced_date_time.get_date_time()
                            < startup_date_time_utc + Duration::minutes(5)
                    } else {
                        device_config.time_is_in_recording_window(
                            &synced_date_time.get_date_time(),
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
                    } else if !making_status_recording {
                        info!("Would start recording, but outside recording window");
                    } else if made_startup_status_recording && !made_shutdown_status_recording {
                        should_start_new_recording = true;
                        // force shutdown status recording even outside of window
                    } else {
                        making_status_recording = false;
                    }
                    if making_status_recording {
                        info!("Making status recording {}", status_recording_state);
                    }
                } else if !should_end_current_recording {
                    if let Some(cptv_stream) = &mut cptv_stream {
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
                        cptv_stream.push_frame(
                            current_raw_frame,
                            &mut prev_frame,
                            &frame_telemetry,
                            &mut flash_storage,
                        );
                        frames_written += 1;
                    }
                } else {
                    // Finalize on a different frame period to writing out the prev/last frame,
                    // to give more breathing room.
                    if let Some(cptv_stream) = &mut cptv_stream {
                        let cptv_start_block_index = cptv_stream.starting_block_index;

                        if !making_status_recording
                            && motion_detection.as_ref().unwrap().was_false_positive()
                        // && cptv_stream.num_frames <= 100
                        {
                            info!(
                                "Discarding as a false-positive {}:{} ",
                                cptv_start_block_index, flash_storage.last_used_block_index
                            );
                            let _ = flash_storage.erase_last_file();
                            event_logger.log_event(
                                LoggerEvent::new(
                                    LoggerEventKind::WouldDiscardAsFalsePositive,
                                    &synced_date_time,
                                ),
                                &mut flash_storage,
                            );
                        } else {
                            cptv_stream.finalise(&mut flash_storage);
                            error!(
                                "Ending current recording start block {} end block{}",
                                cptv_start_block_index, flash_storage.last_used_block_index
                            );
                            event_logger.log_event(
                                LoggerEvent::new(
                                    LoggerEventKind::EndedRecording,
                                    &synced_date_time,
                                ),
                                &mut flash_storage,
                            );

                            if lost_frames > 0 {
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::LostFrames(u64::from(lost_frames)),
                                        &synced_date_time,
                                    ),
                                    &mut flash_storage,
                                );
                            }
                        }

                        // Clear out prev frame before starting a new recording stream.
                        prev_frame_2.fill(0);

                        ended_recording = true;
                        let _ = shared_i2c
                            .set_recording_flag(&mut delay, false)
                            .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));

                        if making_status_recording {
                            making_status_recording = false;
                            match status_recording_state {
                                StatusRecording::StartupStatus => {
                                    made_startup_status_recording = true;
                                    //only make a shutdown if we made a startup
                                    made_shutdown_status_recording = false;
                                }
                                StatusRecording::ShutdownStatus => {
                                    made_shutdown_status_recording = true;
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
                warn!("Transfer aborted for frame #{}, pi must be asleep?", frame_num);
            }
        }
        let frame_transfer_end = timer.get_counter();
        if should_start_new_recording {
            // Since we write 2 frames every new recording, this can take too long and
            // we drop a frame so cache the current frame and tell core1 to keep getting
            // lepton frames, this will spread the initial load over the first 2 frames.

            warn!("Setting recording flag on attiny");
            // TODO: Do we actually want to do this?  It's really there so the RPi/Attiny doesn't shut us down while
            //  we're writing to the flash.  Needs implementation on Attiny side.  But actually, nobody but the rp2040 should
            //  be shutting down the rp2040, so maybe we *don't* need this?  Still nice to have for UI concerns (show recording indicator)
            let _ = shared_i2c
                .set_recording_flag(&mut delay, true)
                .map_err(|e| error!("Error setting recording flag on attiny: {}", e));

            error!("Starting new recording, {:?}", &frame_telemetry);
            let mut cptv_streamer = CptvStream::new(
                synced_date_time.get_timestamp_micros(), // Microseconds
                lepton_version,
                lepton_serial,
                lepton_firmware_version,
                device_config,
                &mut flash_storage,
                &huffman_table,
                &crc_table,
                making_status_recording,
            );
            cptv_streamer.init_gzip_stream(&mut flash_storage, false);

            // Write out the initial frame from *before* the time when the motion detection triggered.
            cptv_streamer.push_frame(
                &prev_frame,
                &mut prev_frame_2, // This should be zeroed out before starting a new clip.
                prev_frame_telemetry.as_ref().unwrap(),
                &mut flash_storage,
            );
            frames_written += 1;

            event_logger.log_event(
                LoggerEvent::new(LoggerEventKind::StartedRecording, &synced_date_time),
                &mut flash_storage,
            );

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
            // now write the frame
            cptv_streamer.push_frame(
                &prev_frame_2,
                &mut prev_frame,
                &frame_telemetry,
                &mut flash_storage,
            );
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

        if frames_seen % (10 * 9) == 0 && frame_header_is_valid {
            info!("Got frame #{}", frame_num);
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
            let sync_rtc_start = timer.get_counter();
            // NOTE: We only advise the RPi that it can shut down if we're not currently recording –
            //  since the change in frame times can affect our frame sync.  It's fine to call this repeatedly,
            //  the RPi will shut down when it wants to.

            if device_config.use_low_power_mode() {
                // Once per minute, if we're not currently recording, tell the RPi it can shut down, as it's not
                // needed in low-power mode unless it's offloading/uploading CPTV data.
                advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
                if !logged_told_rpi_to_sleep {
                    event_logger.log_event(
                        LoggerEvent::new(LoggerEventKind::ToldRpiToSleep, &synced_date_time),
                        &mut flash_storage,
                    );
                    logged_told_rpi_to_sleep = true;
                }
                info!(
                    "Advise pi to shutdown took {}µs",
                    (timer.get_counter() - sync_rtc_start).to_micros()
                );
            }
            let sync_rtc_start_real = timer.get_counter();

            match shared_i2c.get_datetime(&mut delay) {
                Ok(now) => synced_date_time.set(get_datetime_utc(now)),
                Err(err_str) => {
                    event_logger.log_event(
                        LoggerEvent::new(LoggerEventKind::RtcCommError, &synced_date_time),
                        &mut flash_storage,
                    );
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
            is_daytime = device_config.time_is_in_daylight(&synced_date_time.get_date_time());
            let is_outside_recording_window = if dev_mode {
                let is_inside_recording_window =
                    synced_date_time.get_date_time() < startup_date_time_utc + Duration::minutes(5);
                !is_inside_recording_window
            } else {
                !device_config.time_is_in_recording_window(&synced_date_time.get_date_time(), None)
            };

            let flash_storage_nearly_full = flash_storage.is_too_full_to_start_new_recordings();
            if flash_storage_nearly_full && !logged_flash_storage_nearly_full {
                event_logger.log_event(
                    LoggerEvent::new(LoggerEventKind::FlashStorageNearlyFull, &synced_date_time),
                    &mut flash_storage,
                );
                logged_flash_storage_nearly_full = true;
            }
            if ((!device_config.use_low_power_mode() || made_shutdown_status_recording)
                && is_outside_recording_window)
                || flash_storage_nearly_full
            {
                if flash_storage_nearly_full
                    || (is_outside_recording_window && flash_storage.has_files_to_offload())
                {
                    warn!("Recording window ended with files to offload, request restart");
                    // If flash storage is nearly full, or we're now outside the recording window,
                    // Trigger a restart now via the watchdog timer, so that flash storage will
                    // be offloaded during the startup sequence.
                    sio.fifo.write(Core0Task::RequestReset.into());
                    loop {
                        // Wait to be reset
                        nop();
                    }
                }

                if !device_config.use_low_power_mode() {
                    // Tell rPi it is outside its recording window in *non*-low-power mode, and can go to sleep.
                    advise_raspberry_pi_it_may_shutdown(&mut shared_i2c, &mut delay);
                    if !logged_told_rpi_to_sleep {
                        event_logger.log_event(
                            LoggerEvent::new(LoggerEventKind::ToldRpiToSleep, &synced_date_time),
                            &mut flash_storage,
                        );
                        logged_told_rpi_to_sleep = true;
                    }
                }

                let check_power_down_state_start = timer.get_counter();
                if let Ok(pi_is_powered_down) = shared_i2c.pi_is_powered_down(&mut delay, true) {
                    if pi_is_powered_down {
                        if !logged_pi_powered_down {
                            info!("Pi is now powered down: {}", pi_is_powered_down);
                            event_logger.log_event(
                                LoggerEvent::new(
                                    LoggerEventKind::GotRpiPoweredDown,
                                    &synced_date_time,
                                ),
                                &mut flash_storage,
                            );
                            logged_pi_powered_down = true;
                        }

                        // FIXME: I don't understand this logic
                        // only reboot into audio mode if pi is powered down and rp2040 is asleep
                        // so we can keep the thermal preview up as long as the PI is on.
                        match device_config.config().audio_mode {
                            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                                // just reboot and it will go into audio branch
                                info!("Reset as thermal finished and time for audio");
                                sio.fifo.write(Core0Task::RequestReset.into());

                                loop {
                                    nop();
                                }
                            }
                            _ => (),
                        }

                        // NOTE: Calculate the start of the next recording window, set the RTC wake-up alarm,
                        //  and ask for the rp2040 to be put to sleep.
                        let next_recording_window_start = if dev_mode {
                            // In dev mode, we always set the restart alarm for 2 minutes time.
                            synced_date_time.get_date_time() + Duration::minutes(2)
                        } else {
                            device_config
                                .next_recording_window_start(&synced_date_time.get_date_time())
                        };
                        let enabled_alarm = shared_i2c.enable_alarm(&mut delay);
                        if enabled_alarm.is_err() {
                            error!("Failed enabling alarm");
                            event_logger.log_event(
                                LoggerEvent::new(LoggerEventKind::RtcCommError, &synced_date_time),
                                &mut flash_storage,
                            );
                        }
                        if shared_i2c
                            .set_wakeup_alarm(&next_recording_window_start, &mut delay)
                            .is_ok()
                        {
                            let alarm_enabled =
                                shared_i2c.alarm_interrupt_enabled(&mut delay).unwrap_or(false);
                            info!("Wake up alarm interrupt enabled {}", alarm_enabled);
                            if alarm_enabled {
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::SetAlarm(
                                            next_recording_window_start.timestamp_micros(),
                                        ),
                                        &synced_date_time,
                                    ),
                                    &mut flash_storage,
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
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::Rp2040Sleep,
                                        &synced_date_time,
                                    ),
                                    &mut flash_storage,
                                );
                                if shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay).is_ok() {
                                    info!("Sleeping");
                                } else {
                                    error!("Failed sending sleep request to attiny");
                                    event_logger.log_event(
                                        LoggerEvent::new(
                                            LoggerEventKind::AttinyCommError,
                                            &synced_date_time,
                                        ),
                                        &mut flash_storage,
                                    );
                                }
                            } else {
                                error!("Alarm was not properly enabled");
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::RtcCommError,
                                        &synced_date_time,
                                    ),
                                    &mut flash_storage,
                                );
                            }
                            // Now we can put ourselves to sleep.
                        } else {
                            error!("Failed setting wake alarm, can't go to sleep");
                            event_logger.log_event(
                                LoggerEvent::new(LoggerEventKind::RtcCommError, &synced_date_time),
                                &mut flash_storage,
                            );
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
            } else if !is_outside_recording_window && !device_config.use_low_power_mode() {
                if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::ToldRpiToWake(WakeReason::ThermalHighPower),
                            &synced_date_time,
                        ),
                        &mut flash_storage,
                    );
                }
            } else if is_outside_recording_window && !made_shutdown_status_recording {
                making_status_recording = true;
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
            // Increment the datetime n frame's worth.
            // NOTE: We only get the actual date from the RTC every minutes' worth of frames, so that we
            // don't have too many stalls trying to communicate via I2C with the RTC.

            let mut incremented_datetime = synced_date_time.get_date_time();

            let frames_elapsed = if frame_header_is_valid {
                get_frames_elapsed(&frame_telemetry, &prev_frame_telemetry)
            } else {
                1
            };
            if frames_elapsed > 100 {
                warn!("Got {} elapsed frames, ignoring", frames_elapsed);
            } else {
                incremented_datetime += Duration::milliseconds(115 * frames_elapsed);
                if incremented_datetime > synced_date_time.get_date_time() {
                    synced_date_time.set(incremented_datetime);
                }
            }

            // Spend the same time as we would otherwise use querying the RTC to keep frame-times
            //  about the same
            delay.delay_us(expected_rtc_sync_time_us);
        }
        // let one_min_check_end = timer.get_counter();

        // info!(
        //     "Loop took {}µs, 1min check {}µs, frame transfer {}µs",
        //     (timer.get_counter() - start).to_micros(),
        //     (one_min_check_end - one_min_check_start).to_micros(),
        //     (frame_transfer_end - frame_transfer_start).to_micros()
        // );
        if record_audio {
            if let Some(next_audio_alarm) = next_audio_alarm {
                if !audio_pending && synced_date_time.get_date_time() > next_audio_alarm {
                    // Should we be checking alarm triggered? or just use this and clear alarm
                    audio_pending = true;
                    let cur_time = &synced_date_time.get_date_time();
                    info!(
                        "Audio recording is pending because time {}:{} is after {}:{} ",
                        cur_time.hour(),
                        cur_time.minute(),
                        next_audio_alarm.hour(),
                        next_audio_alarm.minute()
                    );
                }
            }

            if cptv_stream.is_none()
                && audio_pending
                && (frame_telemetry.frame_num - last_rec_check) > 9 * 20
            {
                let (_, window_end) = &current_recording_window;

                // if within 2 minutes of end of a window make status before doing audio rec
                // this ensures we always make the status recording

                if !device_config.use_low_power_mode()
                    || made_shutdown_status_recording
                    || synced_date_time.get_date_time() > *window_end - Duration::minutes(2)
                {
                    // FIXME: Was this flag meant to be used to see if the high-power mode was recording?
                    //  I thought rp2040 set it to let tc2-agent know when *it* is recording?

                    // handles case where thermal recorder on the rPi is doing recording
                    if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
                        if !is_recording {
                            info!("Taking audio recording");
                            // make audio rec now
                            let _ = shared_i2c.tc2_agent_take_audio_rec(&mut delay);
                            sio.fifo.write(Core0Task::RequestReset.into());
                            loop {
                                // Wait to be reset
                                nop();
                            }
                        }
                    }
                } else {
                    info!("Not doing audio until after shutdown status");
                }
                last_rec_check = frame_telemetry.frame_num;
            }
        }

        if frame_header_is_valid {
            prev_frame_telemetry = Some(frame_telemetry);
        }
        frames_seen += 1;
    }
}

fn current_time(synced_time: &NaiveDateTime, offset: Duration) -> NaiveDateTime {
    synced_time.add(offset)
}

#[allow(clippy::ref_option)]
fn get_frames_elapsed(
    frame_telemetry: &Telemetry,
    prev_frame_telemetry: &Option<Telemetry>,
) -> i64 {
    if let Some(prev_telemetry) = prev_frame_telemetry {
        let frames_elapsed =
            i64::from(frame_telemetry.frame_num) - i64::from(prev_telemetry.frame_num);
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
