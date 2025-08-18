#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::{
    CameraState, MainI2C, RecordingMode, RecordingRequestType, RecordingTypeDetail,
};
use crate::bsp::pac::{DMA, Peripherals};
use crate::cptv_encoder::huffman::{HUFFMAN_TABLE, HuffmanEntry};
use crate::cptv_encoder::streaming_cptv::{CptvStream, make_crc_table};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{DeviceConfig, RecordingWindow};
use crate::event_logger::{DiscardedRecordingInfo, Event, EventLogger};
use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage, RPI_TRANSFER_HEADER_LENGTH};
use crate::lepton::{FFCStatus, LeptonPins};
use crate::motion_detector::{MotionTracking, track_motion};
use crate::onboard_flash::{FileType, NUM_RECORDING_BLOCKS, OnboardFlash};
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, Duration, Utc};
use core::cell::RefCell;

use crate::bsp;
use crate::lepton_task::lepton_core1_task;
use crate::lepton_telemetry::Telemetry;
use crate::sub_tasks::FormattedNZTime;
use crate::synced_date_time::SyncedDateTime;
use crate::utils::{extend_lifetime_generic, extend_lifetime_generic_mut};
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
use rp2040_hal::{Clock, Sio, Watchdog};

pub const NUM_LEPTON_SEGMENTS: usize = 4;
pub const NUM_LINES_PER_LEPTON_SEGMENT: usize = 61;
const FRAME_BUFFER_ALIGNMENT_PADDING: usize = 2;
const LEPTON_RAW_FRAME_PAYLOAD_LENGTH: usize =
    FRAME_WIDTH * NUM_LINES_PER_LEPTON_SEGMENT * NUM_LEPTON_SEGMENTS;
const FRAME_BUFFER_LENGTH: usize =
    RPI_TRANSFER_HEADER_LENGTH + LEPTON_RAW_FRAME_PAYLOAD_LENGTH + FRAME_BUFFER_ALIGNMENT_PADDING;

pub(crate) const THERMAL_DEV_MODE: bool = false;
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
    SafeToFfc = 0xcf,
    UnsafeToFfc = 0xca,
    LeptonReadyToSleep = 0xbf,
}
#[derive(Format, PartialEq, Eq)]
enum StatusRecordingState {
    NeedsStartup,
    MakingStartup,
    MadeStartup,
    NeedsShutdown,
    MakingShutdown,
    AllDone,
}

impl StatusRecordingState {
    pub fn next_state(&mut self) {
        *self = match self {
            StatusRecordingState::NeedsStartup => StatusRecordingState::MakingStartup,
            StatusRecordingState::MakingStartup => StatusRecordingState::MadeStartup,
            StatusRecordingState::MadeStartup => StatusRecordingState::NeedsShutdown,
            StatusRecordingState::NeedsShutdown => StatusRecordingState::MakingShutdown,
            StatusRecordingState::MakingShutdown | StatusRecordingState::AllDone => {
                StatusRecordingState::AllDone
            }
        };
    }

    pub fn is_pending(&self) -> bool {
        *self == StatusRecordingState::NeedsStartup || *self == StatusRecordingState::NeedsShutdown
    }

    pub fn in_progress(&self) -> bool {
        *self == StatusRecordingState::MakingStartup
            || *self == StatusRecordingState::MakingShutdown
    }

    pub fn is_not_in_progress(&self) -> bool {
        !self.in_progress()
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

pub struct BookkeepingState {
    safe_to_execute_ffc: bool,
    lost_frame_count: u32,
    startup_date_time: DateTime<Utc>,
    frames_seen: u32,
    status_recording: StatusRecordingState,
    current_recording_window: RecordingWindow,

    making_status_recording: Option<()>,
    logged_frame_transfer: Option<()>,
    logged_told_rpi_to_sleep: Option<()>,
    logged_pi_powered_down: Option<()>,
    logged_fs_nearly_full: Option<()>,
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
        let _success = pi_spi.send_message_over_spi(
            ExtTransferMessage::CameraConnectInfo,
            &payload,
            crc,
            dma,
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
    current_recording_window: RecordingWindow,
    recording_mode: RecordingMode,
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
        current_recording_window,
        recording_mode,
    );
}
type LeptonVersion = ((u8, u8, u8), (u8, u8, u8));
fn maybe_do_startup_handshake(
    fs: &mut OnboardFlash,
    pi_spi: &mut ExtSpiTransfers,
    dma: &mut DMA,
    resets: &mut RESETS,
    sio: &mut Sio,
) -> (u8, LeptonVersion, u32) {
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

    send_camera_connect_info(fs, pi_spi, dma, resets, radiometry_enabled, lepton_serial);
    (lepton_version, lepton_firmware_version, lepton_serial)
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
    current_recording_window: RecordingWindow,
    recording_mode: RecordingMode,
) -> ! {
    info!("=== Core 0 Thermal Motion start ===");
    if THERMAL_DEV_MODE {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }
    let mut timer = time.get_timer();
    let mut peripherals = unsafe { Peripherals::steal() };

    let next_alarm = i2c.get_scheduled_alarm(&time).unwrap();

    // TODO: Maybe log the #of FFC events in a recording window, so we can know the average/total
    //  'blackout' time when we might have missed triggers?

    // TODO: Do we want to have a max recording length timeout, or just pause recording if a
    //  subject stays in the frame but doesn't move for a while?  Maybe if a subject is stationary
    //  for 1 minute, we pause, and only resume recording if there is new movement, or it moves again?
    //  If the night ends in this way, we end the recording then.
    //  In continuous recording mode we'd have a really long timeout perhaps?  Needs more thought.
    //  Also consider the case where we have a mask region to ignore or pay attention to.
    events.log(Event::ThermalMode, &time, &mut fs);

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

    let (lepton_version, lepton_firmware_version, lepton_serial) = maybe_do_startup_handshake(
        &mut fs,
        &mut pi_spi,
        &mut dma,
        &mut peripherals.RESETS,
        &mut sio,
    );

    let mut bk = {
        let made_startup_status_recording = (fs.file_types_found.has_type(FileType::CptvStartup)
            && fs.last_startup_recording_time.is_some()
            && fs.last_startup_recording_time.unwrap() > current_recording_window.0)
            || events
                .latest_event_of_kind(Event::OffloadedRecording(FileType::CptvScheduled), &mut fs)
                .is_some_and(|e| {
                    e.timestamp()
                        .is_some_and(|time| time > current_recording_window.0)
                });

        let made_shutdown_recording = fs.file_types_found.has_type(FileType::CptvShutdown);
        let status_recording_state = if made_startup_status_recording && made_shutdown_recording {
            StatusRecordingState::AllDone
        } else if !made_startup_status_recording {
            StatusRecordingState::NeedsStartup
        } else {
            StatusRecordingState::MadeStartup
        };

        info!(
            "Initial status recording state: {:?}",
            status_recording_state
        );
        if config.use_high_power_mode() {
            info!("High power mode");
        } else {
            info!("Low power mode");
        }

        BookkeepingState {
            status_recording: status_recording_state,
            lost_frame_count: 0,
            safe_to_execute_ffc: config.use_low_power_mode(),
            startup_date_time: time.date_time(),
            frames_seen: 0,

            making_status_recording: None,
            logged_frame_transfer: Some(()),
            logged_told_rpi_to_sleep: Some(()),
            logged_pi_powered_down: Some(()),
            logged_fs_nearly_full: Some(()),
            current_recording_window,
        }
    };

    sio.fifo.write_blocking(Core0Task::Ready.into());
    let mut cptv_stream: Option<CptvStream> = None;
    let mut motion_detection: Option<MotionTracking> = None;

    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];
    #[allow(clippy::large_stack_arrays)]
    let mut prev_frame_2: [u16; FRAME_WIDTH * FRAME_HEIGHT] = [0u16; FRAME_WIDTH * FRAME_HEIGHT];
    let mut prev_telemetry: Option<Telemetry> = None;
    let mut stable_telemetry_tracker = ([0u8, 0u8], -1);

    let mut started = Some(());
    let mut test_recording_in_progress = None;
    let mut test_recording =
        if let RecordingMode::Thermal(RecordingRequestType::Test(detail)) = recording_mode {
            Some(detail)
        } else {
            None
        };

    info!(
        "Current recording window {} - {}, Window is active: {}",
        FormattedNZTime(current_recording_window.0),
        FormattedNZTime(current_recording_window.1),
        config.time_is_in_supplied_recording_window(&time.date_time(), current_recording_window)
    );

    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    info!("Entering recording loop.");
    #[allow(unused_labels)]
    'recording_loop: loop {
        let input = sio.fifo.read_blocking();
        crate::assert_eq!(
            input,
            Core0Task::ReceiveFrame.into(),
            "Got unknown fifo input to core1 task loop {}",
            input
        );

        // Get the currently selected buffer to transfer/write to disk.
        let selected_frame_buffer = sio.fifo.read_blocking();
        if bk.safe_to_execute_ffc {
            sio.fifo.write(Core0Task::SafeToFfc.into());
        } else {
            sio.fifo.write(Core0Task::UnsafeToFfc.into());
        }
        if config.use_high_power_mode() && bk.safe_to_execute_ffc {
            // NOTE: At most, this will delay a desired FFC 20 seconds in high-power mode.
            bk.safe_to_execute_ffc = false;
        }

        take_front_buffer(
            &mut thread_local_frame_buffer,
            selected_frame_buffer,
            static_frame_buffer_a,
            static_frame_buffer_b,
        );

        let (telemetry, frame_is_valid, telemetry_is_valid) = process_frame_telemetry(
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

        let mut past_ffc_event = true;
        if frame_is_valid
            && telemetry_is_valid
            && (telemetry.msec_since_last_ffc < 20000
                || telemetry.ffc_status == FFCStatus::InProgress)
        {
            past_ffc_event = false;
        }
        let mut ended_recording = false;
        let mut should_start_new_recording = false;
        if !past_ffc_event && motion_detection.is_some() && cptv_stream.is_none() {
            warn!(
                "Resetting motion detection due to FFC event {:?}",
                telemetry
            );
            motion_detection = None;
        }
        // NOTE: In low power mode, don't try to start recordings/motion detection
        //  until frames have stabilised.
        let should_record_to_flash = frame_is_valid
            && telemetry_is_valid
            && past_ffc_event
            && (config.use_low_power_mode()
                || (test_recording.is_some() || test_recording_in_progress.is_some()));

        // FIXME: What happens if we're in the middle of recording a cptv file and
        //  we get an invalid frame header?  Maybe this doesn't happen in practice?
        //  At least now we don't reset motion detection as if for an FFC event, but
        //  maybe we should reset motion detection, or stop recording?

        if should_record_to_flash {
            if started.take().is_some() {
                info!(
                    "Motion detection active, {:?} frame is valid, {}, telemetry_is_valid {}",
                    telemetry, frame_is_valid, telemetry_is_valid
                );
            }
            // TIME for track_motion: 12.5ms, max 13?
            let this_frame_motion_detection = track_motion(
                current_raw_frame,
                &prev_frame,
                &motion_detection,
                &config.motion_detection_mask,
            );
            let is_inside_recording_window = config
                .time_is_in_supplied_recording_window(&time.date_time(), current_recording_window);
            let is_outside_recording_window = !is_inside_recording_window;

            should_start_new_recording = fs.can_begin_new_cptv_recordings()
                && this_frame_motion_detection.got_new_trigger()
                && cptv_stream.is_none();

            if bk.status_recording.is_pending()
                && !should_start_new_recording
                && cptv_stream.is_none()
                && !fs.is_nearly_full_for_thermal_recordings()
                && is_inside_recording_window
            {
                bk.status_recording.next_state();
                should_start_new_recording = true;
            }
            if this_frame_motion_detection.triggering_ended()
                && cptv_stream.is_none()
                && is_outside_recording_window
            {
                info!("Would end recording, but outside window");
            }

            if !should_start_new_recording
                && test_recording.is_some()
                && !fs.is_nearly_full_for_thermal_recordings()
            {
                // Take test recording.
                test_recording_in_progress = test_recording.take();
                should_start_new_recording = true;
            }

            let should_end_current_recording = if let Some(cptv_stream) = &cptv_stream {
                let (recording_is_max_length, storage_insufficient, motion_ended) =
                    if let Some(RecordingTypeDetail {
                        duration_seconds, ..
                    }) = test_recording_in_progress
                    {
                        (
                            cptv_stream.len() == duration_seconds * 9,
                            fs.is_nearly_full_for_thermal_recordings(),
                            false,
                        )
                    } else if bk.status_recording.in_progress() {
                        (cptv_stream.reached_max_length(), fs.is_full(), false)
                    } else {
                        (
                            cptv_stream.reached_max_length(),
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
                // start recording below so frame buffer is out of scope
                if is_outside_recording_window
                    && bk.status_recording.is_not_in_progress()
                    && test_recording_in_progress.is_none()
                {
                    should_start_new_recording = false;
                    info!("Would start recording, but outside recording window");
                } else if bk.status_recording.in_progress() {
                    info!("Making status recording {}", bk.status_recording);
                } else if test_recording_in_progress.is_some() {
                    info!(
                        "Making user-requested test recording {}",
                        test_recording_in_progress
                    );
                }
            } else if should_end_current_recording {
                // Finalise on a different frame period to writing out the prev/last frame,
                // to give more breathing room.
                if let Some(cptv_stream) = &mut cptv_stream {
                    let cptv_start_block_index = cptv_stream.starting_block_index;

                    if bk.status_recording.is_not_in_progress()
                        && test_recording_in_progress.is_none()
                        && motion_detection.as_ref().unwrap().was_false_positive()
                    {
                        info!(
                            "Discarding as a false-positive {}..={} ",
                            cptv_start_block_index,
                            fs.last_used_block_index.unwrap()
                        );
                        let _ = fs.erase_last_file();
                        #[allow(clippy::cast_possible_truncation)]
                        events.log(
                            Event::WouldDiscardAsFalsePositive(DiscardedRecordingInfo {
                                recording_type: FileType::CptvScheduled,
                                num_frames: cptv_stream.len() as u16,
                                seconds_since_last_ffc: (telemetry.msec_since_last_ffc / 1000)
                                    as u16,
                            }),
                            &time,
                            &mut fs,
                        );
                    } else {
                        cptv_stream.finalise(&mut fs, &time);
                        let blocks_used = cptv_start_block_index..fs.last_used_block_index.unwrap();
                        let num_blocks_used = blocks_used.len() + 1;
                        #[allow(clippy::cast_precision_loss)]
                        let space_used = (num_blocks_used * 2048 * 64) as f32 / 1024.0 / 1024.0;

                        let total_blocks_used = 0..fs.last_used_block_index.unwrap();
                        let total_num_blocks_used = total_blocks_used.len() + 1;
                        #[allow(clippy::cast_precision_loss)]
                        let total_space_used =
                            (total_num_blocks_used * 2048 * 64) as f32 / 1024.0 / 1024.0;
                        #[allow(clippy::cast_precision_loss)]
                        let total_space = (usize::from(NUM_RECORDING_BLOCKS) * 2048 * 64) as f32
                            / 1024.0
                            / 1024.0;
                        error!(
                            "Ending recording, used {:?} ({} blocks), space used {}MB.",
                            blocks_used, num_blocks_used, space_used
                        );
                        info!(
                            "- Total space used {}MB, {}MB free.",
                            total_space_used,
                            total_space - total_space_used
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
                    if test_recording_in_progress.take().is_some() {
                        // Finished requested test recording, restart now.
                        if let Err(e) = i2c.tc2_agent_clear_mode_flags() {
                            error!("Failed to clear mode flags {}", e);
                        }
                        timer.delay_ms(100);
                        request_restart(&mut sio);
                    } else if let Err(e) = i2c.stopped_recording() {
                        error!("Error clearing recording flag on attiny: {}", e);
                    }

                    if bk.status_recording.in_progress() {
                        bk.status_recording.next_state();
                    }
                }
                if is_outside_recording_window && !bk.status_recording.in_progress() {
                    info!("Would end recording, but outside window");
                }
                ended_recording = cptv_stream.take().is_some();
                motion_detection = None;
            } else if let Some(cptv_stream) = &mut cptv_stream {
                // Continue recording in progress (~37ms)
                cptv_stream.push_frame(current_raw_frame, &mut prev_frame, &telemetry, &mut fs);
            }
        }
        // if starting a new recording will handle this differently below
        if !should_start_new_recording {
            // TIME: 2ms
            prev_frame.copy_from_slice(current_raw_frame);
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
            let is_startup_status = test_recording_in_progress.is_none()
                && bk.status_recording == StatusRecordingState::MakingStartup;
            let is_shutdown_status = test_recording_in_progress.is_none()
                && bk.status_recording == StatusRecordingState::MakingShutdown;

            error!("Starting new recording at frame #{}", &telemetry.frame_num);
            info!("{:?}", telemetry);
            let mut cptv_streamer = CptvStream::new(
                time.get_timestamp_micros(), // Microseconds
                lepton_version,
                lepton_serial,
                lepton_firmware_version,
                config,
                &mut fs,
                &huffman_table,
                &crc_table,
                is_startup_status,
                is_shutdown_status,
                test_recording_in_progress.is_some(),
            );
            cptv_streamer.init_gzip_stream(&mut fs, false);
            // Write out the initial frame from *before* the time when the motion detection triggered.
            cptv_streamer.push_frame(
                &prev_frame,
                &mut prev_frame_2, // This should be zeroed out before starting a new clip.
                prev_telemetry.as_ref().unwrap(),
                &mut fs,
            );
            prev_frame_2.copy_from_slice(current_raw_frame);
            // release the buffer before writing the frame so core1 can continue working
            restore_front_buffer(
                &mut thread_local_frame_buffer,
                selected_frame_buffer,
                static_frame_buffer_a,
                static_frame_buffer_b,
            );
            sio.fifo.write(Core0Task::StartRecording.into());

            // FIXME: Satisfy myself this will actually work with dormant mode
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());
            // now write the second/current frame
            cptv_streamer.push_frame(&prev_frame_2, &mut prev_frame, &telemetry, &mut fs);
            prev_frame.copy_from_slice(&prev_frame_2);
            cptv_stream = Some(cptv_streamer);
            events.log(Event::StartedRecording, &time, &mut fs);
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());
        } else {
            // Should not start new recording
            restore_front_buffer(
                &mut thread_local_frame_buffer,
                selected_frame_buffer,
                static_frame_buffer_a,
                static_frame_buffer_b,
            );
            sio.fifo.write(Core0Task::FrameProcessingComplete.into());
        }

        if ended_recording {
            sio.fifo.write(Core0Task::EndRecording.into());
        }
        if bk.frames_seen % (60 * 9) == 0 && frame_is_valid {
            let (_, window_end) = bk.current_recording_window;
            let duration_until_window_end = window_end - time.date_time();
            let inside_recording_window = config
                .time_is_in_supplied_recording_window(&time.date_time(), current_recording_window);
            let duration_until_next_alarm = next_alarm.date_time() - time.date_time();
            let next_alarm_mode = next_alarm.mode;
            if inside_recording_window {
                if duration_until_next_alarm.num_minutes() < 0 {
                    info!(
                        "Got frame #{}, inside recording window",
                        telemetry.frame_num,
                    );
                } else if duration_until_next_alarm.num_minutes() != 0 {
                    info!(
                        "Got frame #{}, inside recording window, window ends in {}mins, next alarm ({}) in {}mins",
                        telemetry.frame_num,
                        duration_until_window_end.num_minutes(),
                        next_alarm_mode,
                        duration_until_next_alarm.num_minutes()
                    );
                } else {
                    info!(
                        "Got frame #{}, inside recording window, window ends in {}mins, next alarm ({}) in {}s",
                        telemetry.frame_num,
                        duration_until_window_end.num_minutes(),
                        next_alarm_mode,
                        duration_until_next_alarm.num_minutes()
                    );
                }
            } else if duration_until_next_alarm.num_minutes() != 0 {
                info!(
                    "Got frame #{}, outside recording window, next alarm ({}) in {}mins",
                    telemetry.frame_num,
                    next_alarm_mode,
                    duration_until_next_alarm.num_minutes()
                );
            } else {
                info!(
                    "Got frame #{}, outside recording window, next alarm ({}) in {}s",
                    telemetry.frame_num,
                    next_alarm_mode,
                    duration_until_next_alarm.num_seconds()
                );
            }
        }

        let is_not_recording_in_low_power_mode = cptv_stream.is_none();
        if is_not_recording_in_low_power_mode && fs.is_too_full_to_start_new_cptv_recordings() {
            if bk.logged_fs_nearly_full.take().is_some() {
                events.log(Event::FlashStorageNearlyFull, &time, &mut fs);
            }
            // Request restart to offload.
            request_restart(&mut sio);
        }

        let expected_i2c_io_time_us = 2400u32;
        let start = timer.get_counter();
        if is_not_recording_in_low_power_mode
            && do_periodic_bookkeeping(
                &mut bk,
                &mut i2c,
                &mut fs,
                &mut events,
                &mut time,
                &mut sio,
                config,
                &test_recording_in_progress,
            )
        {
            #[allow(clippy::cast_possible_truncation)]
            let elapsed = (timer.get_counter() - start).to_micros() as u32;
            let additional_wait = expected_i2c_io_time_us.saturating_sub(elapsed);
            if additional_wait > 0 {
                timer.delay_us(additional_wait);
            } else {
                warn!("I2C messages took {}µs", elapsed);
            }
        } else {
            let frames_elapsed = if frame_is_valid && telemetry_is_valid {
                get_frames_elapsed(&telemetry, &prev_telemetry)
            } else {
                1
            };
            advance_time_by_n_frames(frames_elapsed, &mut time);
            // Spend the same time as we would otherwise use querying the RTC to keep frame-times
            // about the same
            timer.delay_us(expected_i2c_io_time_us);
        }

        if frame_is_valid && telemetry_is_valid {
            prev_telemetry = Some(telemetry);
        }
        bk.frames_seen += 1;
    }
}

fn restore_front_buffer(
    thread_local_frame_buffer: &mut Option<&'static mut FrameBuffer>,
    selected_frame_buffer: u32,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
) {
    critical_section::with(|cs| {
        // Now we just swap the buffers?
        let buffer = if selected_frame_buffer == 0 {
            static_frame_buffer_a
        } else {
            static_frame_buffer_b
        };
        *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
    });
}

fn take_front_buffer(
    thread_local_frame_buffer: &mut Option<&'static mut FrameBuffer>,
    selected_frame_buffer: u32,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
) {
    critical_section::with(|cs| {
        // Now we just swap the buffers
        let buffer = if selected_frame_buffer == 0 {
            static_frame_buffer_a
        } else {
            static_frame_buffer_b
        };
        *thread_local_frame_buffer = buffer.borrow_ref_mut(cs).take();
    });
}

#[allow(clippy::ref_option)]
#[allow(clippy::too_many_lines)]
fn do_periodic_bookkeeping(
    bk: &mut BookkeepingState,
    i2c: &mut MainI2C,
    fs: &mut OnboardFlash,
    events: &mut EventLogger,
    time: &mut SyncedDateTime,
    sio: &mut Sio,
    config: &DeviceConfig,
    user_requested_recording: &Option<RecordingTypeDetail>,
) -> bool {
    // Do periodic checks that require I2C I/O, or are otherwise computationally expensive
    // and can have variable effects on frame time.
    // NOTE: Use approximate prime numbers as the modulus to avoid all of these checks falling
    //  on the same frame.
    let frames_seen = bk.frames_seen;
    let every_minute = frames_seen > 0 && frames_seen % 541 == 0;
    let every_twenty_seconds_in_high_power_mode =
        config.use_high_power_mode() && frames_seen > 0 && frames_seen % 197 == 0;
    let every_ten_seconds = frames_seen > 0 && frames_seen % 47 == 0;
    let every_minute_and_a_half = frames_seen > 0 && frames_seen % 810 == 0;
    if every_minute
        || every_minute_and_a_half
        || every_twenty_seconds_in_high_power_mode
        || every_ten_seconds
    {
        time.resync_with_rtc(i2c, events, fs, every_minute);
    }
    if every_ten_seconds {
        let (_, window_end) = bk.current_recording_window;
        let duration_until_window_end = window_end - time.date_time();
        // If within 2 minutes of the window end, make status before doing audio recording:
        // this ensures we always make the status recording.
        let need_to_make_shutdown_status_recording = config.use_low_power_mode()
            && bk.status_recording == StatusRecordingState::MadeStartup
            && duration_until_window_end <= Duration::minutes(2);
        if need_to_make_shutdown_status_recording {
            warn!("Make shutdown status recording");
            bk.status_recording.next_state();
        } else if user_requested_recording.is_some() {
            // Do nothing, there's a test recording in progress
        } else if let Some(scheduled_alarm) = i2c.get_scheduled_alarm(time) {
            if scheduled_alarm.is_audio_alarm() && scheduled_alarm.has_triggered() {
                let _ = i2c.enter_audio_mode();
                request_restart(sio);
            }
        }
    } else if every_twenty_seconds_in_high_power_mode {
        // if in high power mode need to check thermal-recorder isn't recording
        if let Ok(is_recording) = i2c.get_is_recording() {
            if !is_recording {
                // info!("Safe to execute FFC in high power mode");
                bk.safe_to_execute_ffc = true;
            }
        }
    } else if every_minute {
        // NOTE: Every minute when we're not recording, we try to re-sync our time with the RTC,
        //  as well as checking if we need to tell the rPi to shutdown if we're in low-power mode.
        //  We also check if the flash storage is nearly full, and if so, we request a restart to
        //  offload files to the rPi.
        //  We also check if we need to restart to make an audio recording.
        //  If we're in low power mode and outside the recording window, we set an alarm and ask
        //  the Attiny to put us to sleep.
        //  The reason we only do all this once per minute is because the i2c comms can take
        //  longer than expected, and cause us to lose sync.
        // NOTE: We only advise the RPi that it can shut down if we're not currently recording –
        //  since the change in frame times can affect our frame sync.
        //  It's fine to call this repeatedly, the RPi will shut down when it wants to.
        let camera_state = i2c.get_camera_state();
        if config.use_low_power_mode() {
            // Once per minute, if we're not currently recording,
            // tell the RPi it can shut down, as it's not needed in
            // low-power mode unless it's sending preview frames.
            if camera_state.is_ok_and(|state| !state.pi_is_powered_off())
                && i2c.advise_raspberry_pi_it_may_shutdown().is_ok()
                && bk.logged_told_rpi_to_sleep.take().is_some()
            {
                events.log(Event::ToldRpiToSleep, time, fs);
            }
        }
        let is_inside_recording_window = config
            .time_is_in_supplied_recording_window(&time.date_time(), bk.current_recording_window);
        let is_outside_recording_window = !is_inside_recording_window;
        if is_outside_recording_window
            && (config.use_high_power_mode()
                || fs.file_types_found.has_type(FileType::CptvShutdown))
        {
            // NOTE: Even in continuous recording mode, there is still an "outside the window"
            //  where the current 24hr window ends, and we will restart and offload recordings.

            // FIXME: If there is a user getting frames, they'll be disconnected here.
            //  We could fix that by deferring, if we had a way to check if we still want to prioritise
            //  getting frames?
            if fs.has_recordings_to_offload() {
                warn!("Recording window ended with files to offload, request restart");
                request_restart(sio);
            }
            if config.use_high_power_mode() {
                // Tell rPi it is outside its recording window in high-power
                // mode, and can go to sleep.

                if !config.is_continuous_recorder() {
                    if camera_state.is_ok_and(|state| !state.pi_is_powered_off())
                        && i2c.advise_raspberry_pi_it_may_shutdown().is_ok()
                        && bk.logged_told_rpi_to_sleep.take().is_some()
                    {
                        events.log(Event::ToldRpiToSleep, time, fs);
                    }
                } else if events.is_nearly_full() {
                    // Restart to clear events
                    request_restart(sio);
                }
            }
            if i2c
                .get_camera_state()
                .is_ok_and(CameraState::pi_is_powered_off)
            {
                // Restart so that we'll ask for us to be put to sleep after any file offloading.
                request_restart(sio);
            } else {
                warn!("Pi is still awake, so rp2040 must stay awake");
            }
        } else if is_inside_recording_window && config.use_high_power_mode() {
            if camera_state.is_ok_and(CameraState::pi_is_powered_off) {
                info!("In high power window and pi is powered off, so restart");
                time.get_timer().delay_ms(1000);
                // Restart so that we'll power the rPi back on
                request_restart(sio);
            }
        } else if is_outside_recording_window
            && bk.status_recording == StatusRecordingState::NeedsShutdown
        {
            info!("Should make shutdown recording");
            bk.status_recording.next_state();
        } else if is_outside_recording_window {
            if camera_state.is_ok_and(CameraState::pi_is_powered_off) {
                // Restart so that we'll ask for us to be put to sleep.
                request_restart(sio);
            } else {
                warn!("Pi is still awake, so rp2040 must stay awake");
            }
        }
    } else if every_minute_and_a_half {
        if let Err(e) = i2c.attiny_keep_alive() {
            error!("Failed to send keep alive: {}", e);
        }
    }

    every_ten_seconds
        || every_twenty_seconds_in_high_power_mode
        || every_minute
        || every_minute_and_a_half
}

#[allow(clippy::ref_option)]
fn advance_time_by_n_frames(frames_elapsed: u32, time: &mut SyncedDateTime) {
    // Increment the datetime n frame's worth.
    // NOTE: We only get the actual date from the RTC every minutes' worth of frames, so that we
    //  don't have too many stalls trying to communicate via I2C with the RTC.
    if frames_elapsed > 100 {
        warn!("Got {} elapsed frames, ignoring", frames_elapsed);
    } else {
        time.set(time.date_time_utc + Duration::milliseconds(115 * i64::from(frames_elapsed)));
    }
}

fn request_restart(sio: &mut Sio) {
    info!("Tell core1 to get ready to sleep");
    // Power down frame acquisition
    // Tell core1 we're exiting the recording loop, and it should
    // power down the lepton module, then restart us.
    sio.fifo.write(Core0Task::ReadyToSleep.into());
    loop {
        // Wait to be reset
        nop();
    }
}

#[allow(clippy::ref_option)]
fn process_frame_telemetry(
    bk: &mut BookkeepingState,
    frame_buffer: &mut Option<&mut FrameBuffer>,
    prev_telemetry: &Option<Telemetry>,
    stable_telemetry_tracker: &mut ([u8; 2], i8),
    is_recording: bool,
) -> (Telemetry, bool, bool) {
    let frame_buffer = frame_buffer.as_mut().unwrap().frame_data_as_u8_slice_mut();
    // Read the telemetry:
    let frame_telemetry = Telemetry::from_bytes(frame_buffer);
    let mut skipped_frames = 0;
    let mut telemetry_is_valid = true;
    if let Some(prev_telemetry) = &prev_telemetry {
        let frame_diff = frame_telemetry.frame_num - prev_telemetry.frame_num - 1;
        // over a 100 is probably corrupt telemetry
        if frame_diff > 0 && frame_diff < 100 {
            skipped_frames = frame_diff;
        } else if frame_diff >= 100 {
            telemetry_is_valid = false;
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
    (frame_telemetry, frame_header_is_valid, telemetry_is_valid)
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
