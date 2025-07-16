#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::{AlarmMode, MainI2C, ScheduledAlarmTime};
use crate::bsp::pac::{DMA, Peripherals};
use crate::cptv_encoder::huffman::{HUFFMAN_TABLE, HuffmanEntry};
use crate::cptv_encoder::streaming_cptv::{CptvStream, make_crc_table};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::DeviceConfig;
use crate::event_logger::{Event, EventLogger, LoggerEvent, WakeReason};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage, RPI_TRANSFER_HEADER_LENGTH};
use crate::lepton::{FFCStatus, LeptonPins};
use crate::motion_detector::{MotionTracking, track_motion};
use crate::onboard_flash::{
    OnboardFlash, RecordingFileType, RecordingFileTypeDetails, extend_lifetime_generic,
    extend_lifetime_generic_mut,
};
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Duration, Timelike, Utc};
use core::cell::RefCell;

use crate::bsp;
use crate::frame_processing::Core0Task::LeptonReadyToSleep;
use crate::lepton_task::lepton_core1_task;
use crate::lepton_telemetry::Telemetry;
use crate::rpi_power::wake_raspberry_pi;
use crate::synced_date_time::SyncedDateTime;
use cortex_m::asm::nop;
use crc::{CRC_16_XMODEM, Crc};
use critical_section::Mutex;
use defmt::{Format, error, info, warn};
use embedded_hal::delay::DelayNs;
use fugit::HertzU32;
use rp2040_hal::clocks::ClocksManager;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pac::RESETS;
use rp2040_hal::rosc::RingOscillator;
use rp2040_hal::{Clock, Sio, Timer, Watchdog};

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
    LeptonReadyToSleep = 0xbf,
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
    is_recording_on_rpi: bool,
    made_shutdown_status_recording: bool,
    made_startup_status_recording: bool,
    making_status_recording: bool,
    lost_frame_count: u32,
    low_power_mode: bool,
    scheduled_alarm: Option<ScheduledAlarmTime>,

    logged_frame_transfer: Option<()>,
    logged_told_rpi_to_sleep: Option<()>,
    logged_pi_powered_down: Option<()>,
    logged_fs_nearly_full: Option<()>,
}

impl BookkeepingState {
    pub fn use_low_power_mode(&self) -> bool {
        self.low_power_mode
    }
    pub fn use_high_power_mode(&self) -> bool {
        !self.low_power_mode
    }
}

fn twenty_seconds_elapsed_since_last_check(telemetry: &Telemetry) -> bool {
    // Use a prime number close to 20 seconds, so that we never ask for this on the same
    // frame as we sync the time.
    telemetry.frame_num % 197 == 0
}

fn send_camera_connect_info(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    dma: &mut DMA,
    resets: &mut RESETS,
    radiometry_enabled: u32,
    lepton_serial: u32,
) {
    if let Some(free_spi) = fs.free_spi() {
        let mut payload = [0u8; 16];
        pi_spi.enable(free_spi, resets);
        LittleEndian::write_u32(&mut payload[0..4], radiometry_enabled);
        LittleEndian::write_u32(&mut payload[8..12], lepton_serial);
        LittleEndian::write_u32(&mut payload[12..16], 0); // Thermal mode
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let crc = crc_check.checksum(&payload);
        info!("Sending camera connect info");
        let success = pi_spi.send_message_over_spi(
            ExtTransferMessage::CameraConnectInfo,
            &payload,
            crc,
            dma,
            resets,
            None,
        );
        if let Some(spi) = pi_spi.disable() {
            fs.take_spi(spi, resets);
        }
    }
}

/// # Panics
///
/// TODO
pub fn record_thermal(
    i2c: MainI2C,
    pi_spi: ExtSpiTransfers,
    onboard_flash: OnboardFlash,
    lepton_pins: LeptonPins,
    watchdog: Watchdog,
    system_clock_freq: HertzU32,
    clocks: &ClocksManager,
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    config: &DeviceConfig,
    events: EventLogger,
    time: SyncedDateTime,
    scheduled_alarm: Option<ScheduledAlarmTime>,
) -> ! {
    let mut peripherals = unsafe { Peripherals::steal() };
    let mut sio = Sio::new(peripherals.SIO);
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);

    let cores = mc.cores();
    let core_1 = &mut cores[1];
    // NOTE: We're allocating the stack memory for core1 on our core0 stack rather than using
    //  a `static` var so that the memory isn't used when we're in the audio mode code-path.
    let core1_stack: Stack<470> = Stack::new();
    let mut fb0 = FrameBuffer::new();
    let mut fb1 = FrameBuffer::new();

    let frame_buffer_a = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb0)
    })));
    let frame_buffer_b = Mutex::new(RefCell::new(Some(unsafe {
        extend_lifetime_generic_mut(&mut fb1)
    })));
    let static_frame_buffer_a = unsafe { extend_lifetime_generic(&frame_buffer_a) };
    let static_frame_buffer_b = unsafe { extend_lifetime_generic(&frame_buffer_b) };
    let peripheral_clock_freq = clocks.peripheral_clock.freq();

    watchdog.feed();
    watchdog.disable();
    let is_high_power_mode = config.use_high_power_mode();
    let timer = time.get_timer();
    let _ = core_1.spawn(core1_stack.take().unwrap(), move || {
        lepton_core1_task(
            lepton_pins,
            watchdog,
            system_clock_freq,
            peripheral_clock_freq,
            &rosc,
            static_frame_buffer_a,
            static_frame_buffer_b,
            is_high_power_mode,
            timer,
        );
    });

    thermal_motion_task(
        sio,
        peripherals.DMA,
        i2c,
        pi_spi,
        onboard_flash,
        static_frame_buffer_a,
        static_frame_buffer_b,
        config,
        events,
        time,
        scheduled_alarm,
    );
}

#[allow(clippy::too_many_lines)]
pub fn thermal_motion_task(
    mut sio: Sio,
    mut dma: DMA,
    mut i2c: MainI2C,
    mut pi_spi: ExtSpiTransfers,
    mut fs: OnboardFlash,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
    config: &DeviceConfig,
    mut events: EventLogger,
    mut time: SyncedDateTime,
    scheduled_alarm: Option<ScheduledAlarmTime>,
) -> ! {
    info!("=== Core 0 Thermal Motion start ===");
    if DEV_MODE {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }
    let mut timer = time.get_timer();
    let mut peripherals = unsafe { Peripherals::steal() };

    // TODO: Do we want to have a max recording length timeout, or just pause recording if a
    //  subject stays in the frame but doesn't move for a while?  Maybe if a subject is stationary
    //  for 1 minute, we pause, and only resume recording if there is new movement, or it moves again?
    //  If the night ends in this way, we end the recording then.
    //  In continuous recording mode we'd have a really long timeout perhaps?  Needs more thought.
    //  Also consider the case where we have a mask region to ignore or pay attention to.
    events.log(Event::ThermalMode, &time, &mut fs);
    let startup_date_time_utc = time.date_time();

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
    // TODO: See if this actually affects write speed.
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

    send_camera_connect_info(
        &mut fs,
        &mut pi_spi,
        &mut dma,
        &mut peripherals.RESETS,
        radiometry_enabled,
        lepton_serial,
    );

    let has_cptv_files_saved = fs.has_cptv_files();

    let current_recording_window = config.next_or_current_recording_window(&time.date_time());
    if current_recording_window.is_err() {
        error!("Invalid recording window");
        request_restart(&mut sio, timer);
    }
    let current_recording_window = current_recording_window.unwrap();
    let mut bk = {
        // This might be true if we woke up in thermal mode having been randomly shutdown?
        // FIXME: If the user cancels offload, then we might think we've made a status recording
        //  this session when we actually haven't?
        //  Maybe we also write the current time to the metadata of written files,
        //  then we can see when they were made?
        let made_shutdown_status_recording = !config
            .time_is_in_supplied_recording_window(&time.date_time(), current_recording_window);

        // NOTE: Assume that if we have any cptv files saved we're in the active window
        //  recording session, and therefore we've already made a startup recording in this session.
        let made_startup_status_recording = has_cptv_files_saved;
        BookkeepingState {
            audio_pending: false,
            is_recording_on_rpi: false,
            made_shutdown_status_recording,
            making_status_recording: false,
            lost_frame_count: 0,
            made_startup_status_recording,
            scheduled_alarm,
            low_power_mode: config.use_low_power_mode(),

            logged_frame_transfer: Some(()),
            logged_told_rpi_to_sleep: Some(()),
            logged_pi_powered_down: Some(()),
            logged_fs_nearly_full: Some(()),
        }
    };
    // FIXME: Put into bk, model as state machine transitions.
    let mut status_recording_state = if bk.made_startup_status_recording {
        StatusRecording::NotPending
    } else {
        StatusRecording::StartupStatus
    };

    //warn!("Core 0 is ready to receive frames");
    sio.fifo.write_blocking(Core0Task::Ready.into());
    let mut cptv_stream: Option<CptvStream> = None;
    let mut motion_detection: Option<MotionTracking> = None;

    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];
    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame_2: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];

    let mut frames_written = 0u32;
    let mut frames_seen = 0usize;
    let max_length_in_frames = if DEV_MODE { 60 * 9 } else { 60 * 10 * 9 };

    let mut prev_telemetry: Option<Telemetry> = None;
    let mut stable_telemetry_tracker = ([0u8, 0u8], -1);
    let mut safe_to_execute_ffc = config.use_low_power_mode();

    let mut is_daytime = config.time_is_in_daylight(&time.date_time());
    info!(
        "Current time is in recording window? {}",
        config.time_is_in_configured_recording_window(&time.date_time())
    );

    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    info!(
        "Entering recording loop.  Made start up status recording: {}",
        bk.made_startup_status_recording
    );
    #[allow(unused_labels)]
    'recording_loop: loop {
        let input = sio.fifo.read_blocking();
        let frame_start_time = timer.get_counter();
        crate::assert_eq!(
            input,
            Core0Task::ReceiveFrame.into(),
            "Got unknown fifo input to core1 task loop {}",
            input
        );

        // Get the currently selected buffer to transfer/write to disk.
        let selected_frame_buffer = sio.fifo.read_blocking();
        sio.fifo.write(u32::from(safe_to_execute_ffc));
        if config.use_high_power_mode() && safe_to_execute_ffc {
            // NOTE: At most, this will delay a desired FFC 20 seconds in high-power mode.
            safe_to_execute_ffc = false;
        }
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

        let frame_transfer_start = timer.get_counter();
        // Transfer RAW frame including telemetry header to pi if it is available.
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

        let too_close_to_ffc_event = frame_is_valid
            && (telemetry.msec_since_last_ffc < 20000
                || telemetry.ffc_status == FFCStatus::InProgress);
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        if too_close_to_ffc_event && motion_detection.is_some() {
            warn!("Resetting motion detection due to FFC event");
            frames_seen = 0;
            motion_detection = None;
        }
        let frame_output_stable = telemetry.msec_since_last_ffc > 5000;
        // NOTE: In low power mode, don't try to start recordings/motion detection
        //  until frames have stabilised.
        let should_record_to_flash =
            frame_output_stable && config.use_low_power_mode() && frame_is_valid;
        let past_ffc_event = !too_close_to_ffc_event;

        // FIXME: What happens if we're in the middle of recording a cptv file and
        //  we get an invalid frame header?  Maybe this doesn't happen in practice?
        //  At least now we don't reset motion detection as if for an FFC event, but
        //  maybe we should reset motion detection, or stop recording?

        if should_record_to_flash {
            if past_ffc_event {
                // TIME for track_motion: 12.5ms, max 13?
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
                        if time.date_time() + Duration::minutes(1)
                            > startup_date_time_utc + Duration::minutes(4)
                        {
                            warn!("Make shutdown status recording");
                            status_recording_state = StatusRecording::ShutdownStatus;
                        }
                    } else {
                        let (_, window_end) = &current_recording_window;
                        if time.date_time() + Duration::minutes(1) > *window_end {
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
                            (
                                frames_written >= NUM_STATUS_RECORDING_FRAMES,
                                fs.is_full(),
                                false,
                            )
                        } else {
                            (
                                frames_written >= max_length_in_frames,
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
                        time.date_time() < startup_date_time_utc + Duration::minutes(5)
                    } else {
                        config.time_is_in_supplied_recording_window(
                            &time.date_time(),
                            current_recording_window,
                        )
                    };
                    // start recording below so frame buffer is out of scope
                    should_start_new_recording =
                        should_start_new_recording && is_inside_recording_window;
                    if is_inside_recording_window {
                        // Should we make a 2-second status recording at the beginning or end of the window?
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
                        let ps_start = timer.get_counter();
                        // 37ms
                        cptv_stream.push_frame(
                            current_raw_frame,
                            &mut prev_frame,
                            &telemetry,
                            &mut fs,
                            RecordingFileType::Cptv(RecordingFileTypeDetails {
                                user_requested: false,
                                status: bk.making_status_recording,
                            }),
                        );
                        let ps_end = timer.get_counter();
                        if frames_seen % 45 == 0 {
                            warn!("Push frame {}µs", (ps_end - ps_start).to_micros());
                        }
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
                            cptv_stream.finalise(
                                &mut fs,
                                RecordingFileType::Cptv(RecordingFileTypeDetails {
                                    user_requested: false,
                                    status: bk.making_status_recording,
                                }),
                                &time,
                            );
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
                        if let Err(e) = i2c.stopped_recording() {
                            error!("Error clearing recording flag on attiny: {}", e);
                        }

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
                // TIME: 2ms
                prev_frame.copy_from_slice(current_raw_frame);
            }
        }

        // TIME to await transfer complete to pi: 27ms when not recording, 2ms when recording
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
        if should_start_new_recording {
            // Since we write 2 frames every new recording, this can take too long
            // to write out to flash memory within our time budget, so we end up
            // dropping a frame. Cache the current frame and tell core1 to keep getting
            // lepton frames, this will spread the initial load over the first 2 frames.
            // TIME to set started_recording flag: < 1-2ms
            if let Err(e) = i2c.started_recording() {
                error!("Error setting recording flag on attiny: {}", e);
            }

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
            cptv_streamer.init_gzip_stream(
                &mut fs,
                false,
                RecordingFileType::Cptv(RecordingFileTypeDetails {
                    user_requested: false,
                    status: bk.making_status_recording,
                }),
            );

            // Write out the initial frame from *before* the time when the motion detection triggered.
            cptv_streamer.push_frame(
                &prev_frame,
                &mut prev_frame_2, // This should be zeroed out before starting a new clip.
                prev_telemetry.as_ref().unwrap(),
                &mut fs,
                RecordingFileType::Cptv(RecordingFileTypeDetails {
                    user_requested: false,
                    status: bk.making_status_recording,
                }),
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

            // FIXME: Satisfy myself this will actually work with dormant mode
            info!("Sent start recording message to core1");
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());

            warn_on_duplicate_frames(&prev_telemetry, &telemetry);
            // now write the second/current frame
            cptv_streamer.push_frame(
                &prev_frame_2,
                &mut prev_frame,
                &telemetry,
                &mut fs,
                RecordingFileType::Cptv(RecordingFileTypeDetails {
                    user_requested: false,
                    status: bk.making_status_recording,
                }),
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
        if frames_seen % (30 * 9) == 0 && frame_is_valid {
            info!("Got frame #{}", telemetry.frame_num);
        }

        // FIXME: Check these time estimates
        let expected_rtc_sync_time_us = 4200u32; // using slower clock speed
        let not_recording_and_every_minute_interval_arrived =
            (frames_seen > 1 && frames_seen % (60 * 9) == 0) && cptv_stream.is_none();
        // INFO RTC Sync time took 1350µs

        // NOTE: Every minute when we're not recording, we try to re-sync our time with the RTC,
        //  as well as checking if we need to tell the rPi to shutdown if we're in low-power mode.
        //  We also check if the flash storage is nearly full, and if so, we request a restart to
        //  offload files to the rPi.
        //  We also check if we need to restart to make an audio recording.
        //  If we're in low power mode and outside the recording window, we set an alarm and ask
        //  the Attiny to put us to sleep.
        //  The reason we only do all this once per minute is because the i2c comms can take
        //  longer than expected, and cause us to lose sync.

        if not_recording_and_every_minute_interval_arrived {
            is_daytime = config.time_is_in_daylight(&time.date_time());
            let sync_rtc_start = timer.get_counter();
            // NOTE: We only advise the RPi that it can shut down if we're not currently recording –
            //  since the change in frame times can affect our frame sync.
            //  It's fine to call this repeatedly, the RPi will shut down when it wants to.
            if config.use_low_power_mode() {
                // Once per minute, if we're not currently recording,
                // tell the RPi it can shut down, as it's not needed in
                // low-power mode unless it's sending preview frames.
                if i2c.advise_raspberry_pi_it_may_shutdown().is_ok()
                    && bk.logged_told_rpi_to_sleep.take().is_some()
                {
                    events.log(Event::ToldRpiToSleep, &time, &mut fs);
                }
            }
            time.resync_with_rtc(&mut i2c, &mut events, &mut fs);

            // NOTE: In continuous recording mode, the device will only shut down briefly
            //  when the flash storage is nearly full, and it needs to offload files.
            //  Or, in the case of high-power-mode, it will  never shut down.
            let is_outside_recording_window = if DEV_MODE {
                let is_inside_recording_window =
                    time.date_time() < startup_date_time_utc + Duration::minutes(5);
                !is_inside_recording_window
            } else {
                !config.time_is_in_configured_recording_window(&time.date_time())
            };
            let is_inside_recording_window = !is_outside_recording_window;

            if fs.is_too_full_to_start_new_cptv_recordings() {
                if bk.logged_fs_nearly_full.take().is_some() {
                    events.log(Event::FlashStorageNearlyFull, &time, &mut fs);
                }
                // Request restart to offload.
                request_restart(&mut sio, timer);
            }

            if is_outside_recording_window
                && (bk.use_high_power_mode() || bk.made_shutdown_status_recording)
            {
                if fs.has_recordings_to_offload() {
                    // If we're now outside the recording window,
                    // Trigger a restart now via the watchdog timer, so that flash storage will
                    // be offloaded during the startup sequence.
                    warn!("Recording window ended with files to offload, request restart");
                    request_restart(&mut sio, timer);
                }
                if bk.use_high_power_mode() {
                    // Tell rPi it is outside its recording window in high-power
                    // mode, and can go to sleep.
                    if i2c.advise_raspberry_pi_it_may_shutdown().is_ok()
                        && bk.logged_told_rpi_to_sleep.take().is_some()
                    {
                        events.log(Event::ToldRpiToSleep, &time, &mut fs);
                    }
                }
                shutdown_rp2040_if_possible(
                    &timer,
                    &mut i2c,
                    &time,
                    &mut fs,
                    &mut events,
                    &mut bk,
                    config,
                    &mut sio,
                );
            } else if is_inside_recording_window && bk.use_high_power_mode() {
                // This obviously blocks for a long time, if the rPi is asleep,
                // but is essentially instant if it is already awake and ready.
                // Even if we send the wake signal, we'll probably get killed by
                // the watchdog timer on the frame acquisition thread here.
                // FIXME: Maybe rework the flow of this wake function?
                let _wait_for_wake = wake_raspberry_pi(
                    &mut i2c,
                    timer,
                    None,
                    Some((
                        &mut fs,
                        &mut events,
                        LoggerEvent::new(Event::ToldRpiToWake(WakeReason::ThermalHighPower), &time),
                    )),
                );
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
                timer.delay_us(additional_wait);
            } else {
                warn!("I2C messages took {}µs", sync_time);
            }
        } else {
            advance_time_by_n_frames(&telemetry, &prev_telemetry, &mut time, frame_is_valid);
            // Spend the same time as we would otherwise use querying the RTC to keep frame-times
            // about the same
            timer.delay_us(expected_rtc_sync_time_us);
        }

        // if in high power mode need to check thermal-recorder isn't recording
        if config.use_high_power_mode() && twenty_seconds_elapsed_since_last_check(&telemetry) {
            // only check status every ~20 seconds
            // NOTE: This might be better to replace with the non retry version, so we don't
            //  loop too long doing i2c
            if let Ok(is_recording) = i2c.get_is_recording() {
                if !is_recording {
                    safe_to_execute_ffc = true;
                }
                bk.is_recording_on_rpi = is_recording;
            }
        }

        let recording_in_high_power_mode = bk.is_recording_on_rpi;
        if should_restart_to_make_an_audio_recording(
            &mut bk,
            cptv_stream.is_some() || recording_in_high_power_mode,
            &time,
            &current_recording_window.1,
        ) {
            // FIXME: We can just rely on the alarm trigger.
            if i2c.tc2_agent_take_audio_rec().is_ok() {
                info!("Taking audio recording");
                request_restart(&mut sio, timer);
            } else {
                error!("Failed to write 'take audio recording' flag");
            }
        }

        if frame_is_valid {
            prev_telemetry = Some(telemetry);
        }
        frames_seen += 1;

        // // ===== Frame timings
        // let frame_end_time = timer.get_counter();
        // let frame_time_elapsed_ms = (frame_end_time - frame_start_time).to_millis();
        // let every_5_seconds = frames_seen % 45 == 0;
        // let avg = fpt_rolling.iter().map(|&x| x as u32).sum::<u32>() / fpt_rolling.len() as u32;
        // if every_5_seconds {
        //     info!(
        //         "Frame processing took {}ms, avg {}ms",
        //         frame_time_elapsed_ms, avg,
        //     );
        // }
        // if frame_time_elapsed_ms.saturating_sub(avg as u64) > 5 {
        //     warn!(
        //         "Frame processing time spike {}ms vs avg {}ms",
        //         frame_time_elapsed_ms, avg
        //     );
        // }
        // fpt_rolling[frames_seen % fpt_rolling.len()] = frame_time_elapsed_ms.min(111) as u8;
        //
        // if frame_time_elapsed_ms > 111 {
        //     warn!("Frame processing exceeded budget {}", frame_time_elapsed_ms);
        // }
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
        time.set(time.date_time() + Duration::milliseconds(115 * i64::from(frames_elapsed)));
    }
}

// FIXME: This has similarities with the audio shutdown function, can we share logic?
fn shutdown_rp2040_if_possible(
    timer: &Timer,
    i2c: &mut MainI2C,
    time: &SyncedDateTime,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    bk: &mut BookkeepingState,
    config: &DeviceConfig,
    sio: &mut Sio,
) {
    if let Ok(state) = i2c.get_camera_state() {
        if state.pi_is_powered_off() {
            if bk.logged_pi_powered_down.take().is_some() {
                info!("Pi is now powered down");
                events.log(Event::GotRpiPoweredDown, time, fs);
            }

            // Only reboot into audio mode if pi is powered down and rp2040 is asleep
            // so we can keep the thermal preview up as long as the PI is on.
            if config.records_audio_and_thermal() && !DEV_MODE {
                // Just reboot and it will go into audio branch.
                // Since there is no alarm set, the audio branch should then set an alarm
                // without taking a recording, and then go to sleep.
                info!("Reset as thermal finished and time for audio");
                request_restart(sio, *timer);
            }

            // NOTE: Calculate the start of the next recording window, set the RTC wake-up alarm,
            //  and ask for the rp2040 to be put to sleep.
            if !DEV_MODE
                && config
                    .next_recording_window_start(&time.date_time())
                    .is_err()
            {
                request_restart(sio, *timer);
            }
            let next_recording_window_start = if DEV_MODE {
                // In dev mode, we always set the restart alarm for 2 minutes time.
                time.date_time() + Duration::minutes(2)
            } else {
                config
                    .next_recording_window_start(&time.date_time())
                    .unwrap()
            };
            if i2c
                .set_wakeup_alarm(&next_recording_window_start, AlarmMode::Thermal, time)
                .is_ok()
            {
                events.log(
                    Event::SetThermalAlarm(next_recording_window_start.timestamp_micros()),
                    time,
                    fs,
                );
                power_down_frame_acquisition(sio);
                info!("Ask Attiny to power down rp2040");
                events.log(Event::Rp2040Sleep, time, fs);
                if i2c.tell_attiny_to_power_down_rp2040().is_ok() {
                    info!("Sleeping");
                } else {
                    error!("Failed sending sleep request to attiny");
                    events.log(Event::AttinyCommError, time, fs);
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
    // warn!(
    //     "Check pi power down state took {}µs",
    //     (timer.get_counter() - check_power_down_state_start).to_micros()
    // );
}

fn power_down_frame_acquisition(sio: &mut Sio) {
    info!("Tell core1 to get ready to sleep");
    // Tell core1 we're exiting the recording loop, and it should
    // power down the lepton module, and wait for reply.
    sio.fifo.write(Core0Task::ReadyToSleep.into());
    loop {
        // Wait until core1 tells us that the message was received and the lepton
        // module has been put to sleep.
        if let Some(result) = sio.fifo.read() {
            if result == LeptonReadyToSleep.into() {
                break;
            }
        }
    }
}

fn check_if_rpi_is_recording_in_high_power_mode(bk: &mut BookkeepingState, i2c: &mut MainI2C) {
    if bk.use_high_power_mode() {
        // If i2c comms fail, assume we're not recording on the pi
        bk.is_recording_on_rpi = i2c
            .get_is_recording()
            .is_ok_and(|is_recording| is_recording);
    }
}

fn should_restart_to_make_an_audio_recording(
    bk: &mut BookkeepingState,
    is_recording_cptv: bool,
    time: &SyncedDateTime,
    window_end: &DateTime<Utc>,
) -> bool {
    if let Some(scheduled_alarm) = &bk.scheduled_alarm
        && scheduled_alarm.is_audio_alarm()
    {
        if !bk.audio_pending {
            let current_time = time.date_time();
            bk.audio_pending = current_time > scheduled_alarm.time();
            if bk.audio_pending {
                info!(
                    "Audio recording is pending because time {}:{} is after {}:{} ",
                    current_time.hour(),
                    current_time.minute(),
                    scheduled_alarm.time().hour(),
                    scheduled_alarm.time().minute()
                );
            }
        }
        if is_recording_cptv {
            false
        } else if bk.audio_pending {
            let duration_until_window_end = *window_end - time.date_time();

            // If within 2 minutes of the window end, make status before doing audio recording:
            // this ensures we always make the status recording.
            let need_to_make_shutdown_status_recording = bk.use_low_power_mode()
                && !bk.made_shutdown_status_recording
                && duration_until_window_end <= Duration::minutes(2);

            if need_to_make_shutdown_status_recording {
                info!("Pending audio recording, but deferring until after shutdown status");
                false
            } else {
                true
            }
        } else {
            false
        }
    } else {
        false
    }
}

fn request_restart(sio: &mut Sio, mut delay: Timer) {
    power_down_frame_acquisition(sio);
    // Delay so we actually get to see the shutdown message in the console.
    delay.delay_ms(500);
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
