#![allow(dead_code)]
#![allow(unused_variables)]

use crate::attiny_rtc_i2c::{I2CConfig, SharedI2C};
use crate::bsp::pac;
use crate::bsp::pac::Peripherals;
use crate::cptv_encoder::huffman::{HuffmanEntry, HUFFMAN_TABLE};
use crate::cptv_encoder::streaming_cptv::{make_crc_table, CptvStream};
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::{get_naive_datetime, AudioMode, DeviceConfig};
use crate::event_logger::{
    clear_audio_alarm, get_audio_alarm, EventLogger, LoggerEvent, LoggerEventKind, WakeReason,
};
use crate::sub_tasks::{maybe_offload_events, offload_flash_storage_and_events};

use crate::ext_spi_transfers::{ExtSpiTransfers, ExtTransferMessage};
use crate::lepton::{read_telemetry, FFCStatus, Telemetry};
use crate::motion_detector::{track_motion, MotionTracking};
use crate::onboard_flash::OnboardFlash;
use crate::utils::u8_slice_to_u16;
use crate::FrameBuffer;
use chrono::{Datelike, Duration, NaiveDateTime, Timelike};

use core::cell::RefCell;
use core::ops::Add;
use cortex_m::asm::nop;
use cortex_m::delay::Delay;
use critical_section::Mutex;
use defmt::{error, info, warn, Format};
use rp2040_hal::gpio::bank0::{
    Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio3, Gpio5, Gpio8, Gpio9,
};
use rp2040_hal::gpio::{FunctionNull, FunctionSio, Pin, PullDown, PullNone, SioInput, SioOutput};
use rp2040_hal::timer::Instant;
use rp2040_hal::{Sio, Timer};

use crate::audio_task::{
    check_alarm_still_valid_with_thermal_window, schedule_audio_rec, AlarmMode, MAX_GAP_MIN,
};
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
    RequestReset = 0xea,
    HighPowerMode = 0xeb,
    ReceiveFrameWithPendingFFC = 0xac,
}
#[derive(Format)]

enum StatusRecording {
    StartupStatus = 0,
    ShutdownStatus = 1,
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

/// Returns `true` if it did wake the rPI
pub fn wake_raspberry_pi(shared_i2c: &mut SharedI2C, delay: &mut Delay) -> bool {
    match shared_i2c.pi_is_powered_down(delay, false) {
        Ok(true) => {
            if shared_i2c.tell_pi_to_wakeup(delay).is_ok() {
                // TODO: Log here if this was an unexpected wakeup
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
                true
            } else {
                warn!("Failed to send wake signal to raspberry pi");
                false
            }
        }
        _ => {
            loop {
                if let Ok(pi_is_awake) = shared_i2c.pi_is_awake_and_tc2_agent_is_ready(delay, false)
                {
                    if pi_is_awake {
                        break;
                    } else {
                        // Try to wake it again, just in case it was shutdown behind our back.
                        let _ = shared_i2c.tell_pi_to_wakeup(delay);
                    }
                }
                delay.delay_ms(1000);
            }
            false
        }
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
            info!(
                "Got stable telemetry revision (core 1) {:?}",
                frame_telemetry.revision
            );
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

// NOTE: Important: If we start using dormant states again, the timer will be incorrect
pub struct SyncedDateTime {
    pub date_time_utc: NaiveDateTime,
    pub timer_offset: Instant,
}

impl Default for SyncedDateTime {
    fn default() -> Self {
        SyncedDateTime {
            date_time_utc: NaiveDateTime::default(),
            timer_offset: Instant::from_ticks(0),
        }
    }
}

impl SyncedDateTime {
    pub fn get_timestamp_micros(&self, timer: &rp2040_hal::Timer) -> u64 {
        (self.date_time_utc
            + chrono::Duration::microseconds(
                (timer.get_counter() - self.timer_offset).to_micros() as i64
            ))
        .and_utc()
        .timestamp_micros() as u64
    }

    pub fn get_adjusted_dt(&self, timer: &rp2040_hal::Timer) -> NaiveDateTime {
        self.date_time_utc
            + chrono::Duration::microseconds(
                (timer.get_counter() - self.timer_offset).to_micros() as i64
            )
    }

    pub fn set(&mut self, date_time: NaiveDateTime, timer: &rp2040_hal::Timer) {
        self.date_time_utc = date_time;
        self.timer_offset = timer.get_counter();
    }

    pub fn new(date_time: NaiveDateTime, timer: &rp2040_hal::Timer) -> SyncedDateTime {
        SyncedDateTime {
            date_time_utc: date_time,
            timer_offset: timer.get_counter(),
        }
    }
}

pub fn thermal_motion_task(
    mut pi_spi: ExtSpiTransfers,
    mut flash_storage: OnboardFlash,
    frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    clock_freq: u32,
    i2c_config: I2CConfig,
    unlocked_pin: Pin<Gpio3, FunctionSio<SioInput>, PullDown>,
    lepton_serial: Option<u32>,
    lepton_firmware_version: Option<((u8, u8, u8), (u8, u8, u8))>,
    woken_by_alarm: bool,
    mut timer: Timer,
    device_config: &DeviceConfig,
    event_logger: &mut EventLogger,
    synced_date_time: &mut SyncedDateTime,
) -> ! {
    let dev_mode = false;
    info!("=== Core 0 Thermal Motion start ===");
    if dev_mode {
        warn!("DEV MODE");
    } else {
        warn!("FIELD MODE");
    }

    let mut peripherals = unsafe { Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut delay = Delay::new(core.SYST, clock_freq);
    let mut sio = Sio::new(peripherals.SIO);
    let mut shared_i2c = SharedI2C::new(i2c_config, unlocked_pin, &mut delay);

    let should_record_to_flash = true;

    {
        if flash_storage.has_files_to_offload() {
            info!("Finished scan, has files to offload");
        }
    }

    if event_logger.has_events_to_offload() {
        info!("There are {} event(s) to offload", event_logger.count());
    }

    event_logger.log_event(
        LoggerEvent::new(
            LoggerEventKind::ThermalMode,
            synced_date_time.get_timestamp_micros(&timer),
        ),
        &mut flash_storage,
    );
    if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
        if is_recording {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::RecordingNotFinished,
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
        }
    }
    // Unset the is_recording flag on attiny on startup
    let _ = shared_i2c
        .set_recording_flag(&mut delay, false)
        .map_err(|e| error!("Error setting recording flag on attiny: {}", e));
    let startup_date_time_utc: NaiveDateTime = synced_date_time.date_time_utc.clone();

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

    sio.fifo.write_blocking(Core1Task::Ready.into());

    let radiometry_enabled = sio.fifo.read_blocking();
    info!("Core 1 got radiometry enabled: {}", radiometry_enabled == 2);
    let lepton_version = if radiometry_enabled == 2 { 35 } else { 3 };

    if woken_by_alarm {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::Rp2040WokenByAlarm,
                synced_date_time.get_timestamp_micros(&timer),
            ),
            &mut flash_storage,
        );
    }

    let record_audio: bool;
    let mut audio_pending: bool = false;
    let mut next_audio_alarm: Option<NaiveDateTime> = None;

    match device_config.config().audio_mode {
        AudioMode::AudioOrThermal | AudioMode::AudioAndThermal => {
            let (audio_mode, audio_alarm) = get_audio_alarm(&mut flash_storage);
            record_audio = true;
            let mut schedule_alarm = true;
            // if audio alarm is set check it's within 60 minutes and before or on thermal window start
            if let Some(audio_alarm) = audio_alarm {
                if let Ok(audio_mode) = audio_mode {
                    if audio_mode == AlarmMode::AUDIO {
                        let synced = synced_date_time.date_time_utc;
                        let until_alarm = (audio_alarm - synced).num_minutes();
                        if until_alarm <= MAX_GAP_MIN as i64 {
                            info!(
                                "Audio alarm already scheduled for {}-{}-{} {}:{}",
                                audio_alarm.year(),
                                audio_alarm.month(),
                                audio_alarm.day(),
                                audio_alarm.hour(),
                                audio_alarm.minute()
                            );
                            if check_alarm_still_valid_with_thermal_window(
                                &audio_alarm,
                                &synced,
                                &device_config,
                            ) {
                                next_audio_alarm = Some(audio_alarm);
                                schedule_alarm = false;
                            } else {
                                schedule_alarm = true;
                                //if window time changed and alarm is after rec window start
                                info!("Rescehduling as alarm is after window start");
                            }
                        } else {
                            info!("Alarm is missed");
                        }
                    }
                }
            }
            if schedule_alarm {
                if let Ok(next_alarm) = schedule_audio_rec(
                    &mut delay,
                    &synced_date_time,
                    &mut shared_i2c,
                    &mut flash_storage,
                    &mut timer,
                    event_logger,
                    &device_config,
                ) {
                    next_audio_alarm = Some(next_alarm);
                    info!("Setting a pending audio alarm");
                } else {
                    error!("Couldn't schedule alarm");
                }
            }
        }
        _ => {
            info!("Clearing audio alarm");
            clear_audio_alarm(&mut flash_storage);
            record_audio = false
        }
    }

    if !device_config.use_low_power_mode() {
        if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToWake(WakeReason::ThermalHighPower),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
        }
        maybe_offload_events(
            &mut pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            &mut delay,
            &mut timer,
            event_logger,
            &mut flash_storage,
            clock_freq,
            &synced_date_time,
            None,
        );
    }
    // NOTE: We'll only wake the pi if we have files to offload, and it is *outside* the recording
    //  window, or the previous offload happened more than 24 hours ago, or the flash is nearly full.
    //  Otherwise, if the rp2040 happens to restart, we'll pretty much
    // always start the pi up, which we don't want.
    let has_files_to_offload = flash_storage.has_files_to_offload();
    let should_offload = (has_files_to_offload
        && !device_config.time_is_in_recording_window(&synced_date_time.date_time_utc, &None))
        || flash_storage.is_too_full_to_start_new_recordings()
        || (has_files_to_offload && flash_storage.file_start_block_index.is_none());
    //means old file system offload once

    if should_offload {
        event_logger.log_event(
            LoggerEvent::new(
                LoggerEventKind::ToldRpiToWake(WakeReason::ThermalOffload),
                synced_date_time.get_timestamp_micros(&timer),
            ),
            &mut flash_storage,
        );
    }
    let should_offload = if !should_offload {
        let previous_offload_time = event_logger
            .latest_event_of_kind(LoggerEventKind::OffloadedRecording, &mut flash_storage)
            .map(|event| event.timestamp());
        let duration_since_prev_offload: Duration =
            if let Some(previous_offload_time) = previous_offload_time {
                if let Some(timestamp) = previous_offload_time {
                    synced_date_time.date_time_utc.clone() - timestamp
                } else {
                    Duration::minutes(0)
                }
            } else {
                Duration::minutes(0)
            };
        if has_files_to_offload && duration_since_prev_offload > Duration::hours(24) {
            event_logger.log_event(
                LoggerEvent::new(
                    LoggerEventKind::ToldRpiToWake(WakeReason::ThermalOffloadAfter24Hours),
                    synced_date_time.get_timestamp_micros(&timer),
                ),
                &mut flash_storage,
            );
            true
        } else {
            false
        }
    } else {
        should_offload
    };

    let is_cptv = flash_storage.has_cptv_files(false);
    let mut made_startup_status_recording = is_cptv;

    let current_recording_window =
        device_config.next_or_current_recording_window(&synced_date_time.date_time_utc);

    let mut made_shutdown_status_recording = !device_config.time_is_in_recording_window(
        &synced_date_time.date_time_utc,
        &Some(current_recording_window),
    );

    info!(
        "Has cptv files? {} has files? {}",
        is_cptv,
        flash_storage.has_files_to_offload()
    );

    let did_offload_files = if should_offload {
        offload_flash_storage_and_events(
            &mut flash_storage,
            &mut pi_spi,
            &mut peripherals.RESETS,
            &mut peripherals.DMA,
            clock_freq,
            &mut shared_i2c,
            &mut delay,
            &mut timer,
            event_logger,
            synced_date_time,
            None,
            false,
        )
    } else {
        false
    };

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

    warn!("Core 1 is ready to receive frames");
    sio.fifo.write_blocking(Core1Task::Ready.into());
    if !device_config.config().use_low_power_mode {
        sio.fifo.write_blocking(Core1Task::HighPowerMode.into());
    }
    let mut cptv_stream: Option<CptvStream> = None;
    let mut prev_frame: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1] =
        [0u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1];

    let mut prev_frame_2: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1] =
        [0u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1];

    let mut frames_written = 0;
    let mut frames_seen = 0usize;
    let mut prev_frame_telemetry: Option<Telemetry> = None;
    let mut stable_telemetry_tracker = ([0u8, 0u8], -1);

    let mut is_daytime = device_config.time_is_in_daylight(&synced_date_time.date_time_utc);

    info!(
        "Current time is in recording window? {}",
        device_config.time_is_in_recording_window(&synced_date_time.date_time_utc, &None)
    );

    let mut motion_detection: Option<MotionTracking> = None;
    let mut logged_frame_transfer = false;
    let mut logged_told_rpi_to_sleep = false;
    let mut logged_pi_powered_down = false;
    let mut logged_flash_storage_nearly_full = false;
    // NOTE: If there are already recordings on the flash memory,
    //  assume we've already made the startup status recording during this recording window.

    let mut making_status_recording = false;
    let mut status_recording_pending = if !made_startup_status_recording {
        Some(StatusRecording::StartupStatus)
    } else {
        None
    };

    let mut high_power_recording = false;
    let mut last_rec_check = 0;
    let mut lost_frames = 0;
    // Enable raw frame transfers to pi – if not already enabled.
    pi_spi.enable_pio_spi();
    info!(
        "Entering frame loop made start up? {}",
        made_startup_status_recording
    );
    loop {
        let input = sio.fifo.read_blocking();
        let needs_ffc: bool;
        if input == Core1Task::ReceiveFrameWithPendingFFC.into() {
            needs_ffc = true;
        } else {
            crate::assert_eq!(
                input,
                Core1Task::ReceiveFrame.into(),
                "Got unknown fifo input to core1 task loop {}",
                input
            );
            needs_ffc = false;
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
        let frame_swap_time = timer.get_counter();

        let (frame_telemetry, frame_header_is_valid) = {
            let frame_buffer = &mut thread_local_frame_buffer
                .as_mut()
                .unwrap()
                .frame_data_as_u8_slice_mut();
            // Read the telemetry:
            let frame_telemetry = read_telemetry(&frame_buffer);
            let mut skipped_frames = 0;
            if let Some(prev_telemetry) = &prev_frame_telemetry {
                let frame_diff = frame_telemetry.frame_num - prev_telemetry.frame_num - 1;
                //over a 100 is probably corrupt telemtry
                if frame_diff > 0 && frame_diff < 100 {
                    skipped_frames = frame_diff;
                }
            }

            if cptv_stream.is_some() {
                //if recording accumulate
                lost_frames += skipped_frames;
            } else {
                lost_frames = skipped_frames;
            }
            // Sometimes we get an invalid frame header on this thread; we detect and ignore these frames.
            let frame_header_is_valid =
                is_frame_telemetry_is_valid(&frame_telemetry, &mut stable_telemetry_tracker);
            (frame_telemetry, frame_header_is_valid)
        };

        //if in high power mode need to check thermal-recorder hasn't made a recording
        if needs_ffc && !device_config.use_low_power_mode() {
            //only check status every 20 seconds
            if frame_telemetry.frame_num - last_rec_check > 9 * 20 {
                if let Ok(is_recording) = shared_i2c.tc2_agent_is_recording(&mut delay) {
                    high_power_recording = is_recording;
                    last_rec_check = frame_telemetry.frame_num;
                    info!(
                        "Checking if recording {} am recording ?? {}",
                        is_recording, high_power_recording
                    );
                    if high_power_recording {
                        sio.fifo.write(Core1Task::StartRecording.into());
                        high_power_recording = true;
                    } else {
                        sio.fifo.write(Core1Task::EndRecording.into());
                        high_power_recording = false;
                        thread_local_frame_buffer
                            .as_mut()
                            .unwrap()
                            .ffc_imminent(true);
                    }
                }
            } else if !high_power_recording {
                //depending on timing might get 2 frames with needs_ffc event after said ok
                thread_local_frame_buffer
                    .as_mut()
                    .unwrap()
                    .ffc_imminent(true);
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
                &mut thread_local_frame_buffer
                    .as_mut()
                    .unwrap()
                    .as_u8_slice_mut(),
                0,
                cptv_stream.is_some(),
                &mut peripherals.DMA,
                &mut timer,
            )
        } else {
            None
        };
        if !logged_frame_transfer && frame_header_is_valid {
            if transfer.is_some() {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::StartedSendingFramesToRpi,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
            } else {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::StartedGettingFrames,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );
            }
            logged_frame_transfer = true;
        }

        let frame_buffer = &mut thread_local_frame_buffer
            .as_mut()
            .unwrap()
            .frame_data_as_u8_slice_mut();

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
        if frames_seen >= WAIT_N_FRAMES_FOR_STABLE
            && device_config.use_low_power_mode()
            && frame_header_is_valid
        {
            let current_raw_frame =
                unsafe { &u8_slice_to_u16(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT] }; // Telemetry skipped

            //just want 1 previous frame for first status
            if !too_close_to_ffc_event && frames_seen > WAIT_N_FRAMES_FOR_STABLE {
                let this_frame_motion_detection = track_motion(
                    &current_raw_frame,
                    &prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT],
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
                    && status_recording_pending.is_none()
                {
                    if dev_mode {
                        if synced_date_time.date_time_utc + Duration::minutes(1)
                            > startup_date_time_utc + Duration::minutes(4)
                        {
                            warn!("Make shutdown status recording");
                            status_recording_pending = Some(StatusRecording::ShutdownStatus);
                        }
                    } else {
                        let (_, window_end) = &current_recording_window;
                        if &(synced_date_time.date_time_utc + Duration::minutes(1)) > window_end {
                            warn!("Make shutdown status recording");
                            status_recording_pending = Some(StatusRecording::ShutdownStatus);
                        }
                    }
                }

                if status_recording_pending.is_some()
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
                    let is_inside_recording_window = if !dev_mode {
                        device_config.time_is_in_recording_window(
                            &synced_date_time.date_time_utc,
                            &Some(current_recording_window),
                        )
                    } else {
                        // Recording window is 5 minutes from startup time in dev mode.
                        synced_date_time.date_time_utc
                            < startup_date_time_utc + chrono::Duration::minutes(5)
                    };
                    //start recording bellow so frame buffer is out of scope
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
                        //force shutdown status recording even outside of window
                    } else {
                        making_status_recording = false;
                    }
                    if making_status_recording {
                        info!("Making status recording {}", status_recording_pending);
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
                    // Finalise on a different frame period to writing out the prev/last frame,
                    // to give more breathing room.
                    if let Some(cptv_stream) = &mut cptv_stream {
                        let cptv_start_block_index = cptv_stream.starting_block_index as isize;

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
                                    synced_date_time.get_timestamp_micros(&timer),
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
                                    synced_date_time.get_timestamp_micros(&timer),
                                ),
                                &mut flash_storage,
                            );

                            if lost_frames > 0 {
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::LostFrames(lost_frames as u64),
                                        synced_date_time.get_timestamp_micros(&timer),
                                    ),
                                    &mut flash_storage,
                                );
                            }
                        }
                        prev_frame_2.fill(0);

                        ended_recording = true;
                        let _ = shared_i2c
                            .set_recording_flag(&mut delay, false)
                            .map_err(|e| error!("Error clearing recording flag on attiny: {}", e));

                        if making_status_recording {
                            making_status_recording = false;
                            match status_recording_pending.unwrap() {
                                StatusRecording::StartupStatus => {
                                    made_startup_status_recording = true;
                                    //only make a shutdown if we made a startup
                                    made_shutdown_status_recording = false;
                                }
                                StatusRecording::ShutdownStatus => {
                                    made_shutdown_status_recording = true;
                                }
                            }
                            status_recording_pending = None;
                        }
                    }
                    cptv_stream = None;
                    frames_written = 0;
                    motion_detection = None;
                }
            } else {
                should_start_new_recording = false;
            }

            //if starting a new recording will handle this differently below
            if !should_start_new_recording {
                prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT].copy_from_slice(current_raw_frame);
            }
        }

        if let Some((transfer, transfer_end_address, transfer_start_address)) = transfer {
            let did_abort_transfer = pi_spi.end_message(
                &mut peripherals.DMA,
                transfer_end_address,
                transfer_start_address,
                transfer,
            );
            if did_abort_transfer {
                warn!(
                    "Transfer aborted for frame #{}, pi must be asleep?",
                    frame_num
                );
            }
        }
        let frame_transfer_end = timer.get_counter();
        if should_start_new_recording {
            {
                //Since we write 2 frames every new recording, this can take too long and we drop a frame
                //so cache the current frame and tell core0 to keep getting lepton frames, this will spread the initial
                //load over the first 2 frames.

                warn!("Setting recording flag on attiny");
                // TODO: Do we actually want to do this?  It's really there so the RPi/Attiny doesn't shut us down while
                //  we're writing to the flash.  Needs implementation on Attiny side.  But actually, nobody but the rp2040 should
                //  be shutting down the rp2040, so maybe we *don't* need this?  Still nice to have for UI concerns (show recording indicator)
                let _ = shared_i2c
                    .set_recording_flag(&mut delay, true)
                    .map_err(|e| error!("Error setting recording flag on attiny: {}", e));

                error!("Starting new recording, {:?}", &frame_telemetry);
                // TODO: Pass in various cptv header info bits.
                let mut cptv_streamer = CptvStream::new(
                    synced_date_time.get_timestamp_micros(&timer), // Microseconds
                    lepton_version,
                    lepton_serial.clone(),
                    lepton_firmware_version.clone(),
                    &device_config,
                    &mut flash_storage,
                    &huffman_table,
                    &crc_table,
                    making_status_recording,
                );
                cptv_streamer.init_gzip_stream(&mut flash_storage, false);
                cptv_streamer.push_frame(
                    &prev_frame,
                    &mut prev_frame_2, // This should be zeroed out before starting a new clip.
                    &prev_frame_telemetry.as_ref().unwrap(),
                    &mut flash_storage,
                );
                frames_written += 1;

                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::StartedRecording,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
                    &mut flash_storage,
                );

                cptv_stream = Some(cptv_streamer);
                prev_frame_2[FRAME_WIDTH * FRAME_HEIGHT] = 0;
                prev_frame_2[0..FRAME_WIDTH * FRAME_HEIGHT].copy_from_slice(unsafe {
                    &u8_slice_to_u16(&frame_buffer[640..])[0..FRAME_WIDTH * FRAME_HEIGHT]
                }); // Telemetry skipped
                    // release the buffer before writing the frame so core0 can continue working
                critical_section::with(|cs| {
                    // Now we just swap the buffers?
                    let buffer = if selected_frame_buffer == 0 {
                        frame_buffer_local
                    } else {
                        frame_buffer_local_2
                    };
                    *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
                });
                sio.fifo.write(Core1Task::StartRecording.into());
                info!("Sent start recording message to core0");
                sio.fifo.write(Core1Task::FrameProcessingComplete.into());

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
                //now write the frame
                cptv_stream.as_mut().unwrap().push_frame(
                    &mut prev_frame_2,
                    &mut prev_frame,
                    &frame_telemetry,
                    &mut flash_storage,
                );
                frames_written += 1;
                prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT]
                    .copy_from_slice(&prev_frame_2[0..FRAME_WIDTH * FRAME_HEIGHT]);
            }
        } else {
            critical_section::with(|cs| {
                // Now we just swap the buffers?
                let buffer = if selected_frame_buffer == 0 {
                    frame_buffer_local
                } else {
                    frame_buffer_local_2
                };
                *buffer.borrow_ref_mut(cs) = thread_local_frame_buffer.take();
            });
            sio.fifo.write(Core1Task::FrameProcessingComplete.into());
        }

        let swap_buffer = timer.get_counter();
        if ended_recording && cptv_stream.is_none() {
            info!("Send end recording message to core0");
            sio.fifo.write(Core1Task::EndRecording.into());
        }

        if frames_seen % (10 * 9) == 0 && frame_header_is_valid {
            info!("Got frame #{}", frame_num);
        }

        // let one_min_check_start = timer.get_counter();
        // let expected_rtc_sync_time_us = 3500;
        let expected_rtc_sync_time_us = 4200; //using slower clock speed

        //INFO  RTC Sync time took 1350µs
        if (frames_seen > 1 && frames_seen % (60 * 9) == 0) && cptv_stream.is_none() {
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
                        LoggerEvent::new(
                            LoggerEventKind::ToldRpiToSleep,
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
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
                Ok(now) => synced_date_time.set(get_naive_datetime(now), &timer),
                Err(err_str) => {
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::RtcCommError,
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
                        &mut flash_storage,
                    );
                    error!("Unable to get DateTime from RTC: {}", err_str);
                }
            };

            info!(
                "RTC Sync time took {}µs",
                (timer.get_counter() - sync_rtc_start_real).to_micros()
            );

            // NOTE: In continuous recording mode, the device will only shut down briefly when the flash storage
            // is nearly full, and it needs to offload files.  Or, in the case of non-low-power-mode, it will
            // never shut down.
            is_daytime = device_config.time_is_in_daylight(&synced_date_time.date_time_utc);
            let is_outside_recording_window = if !dev_mode {
                !device_config.time_is_in_recording_window(&synced_date_time.date_time_utc, &None)
            } else {
                // !device_config.time_is_in_recording_window(&synced_date_time.date_time_utc, &None)

                let is_inside_recording_window =
                    synced_date_time.date_time_utc < startup_date_time_utc + Duration::minutes(5);
                !is_inside_recording_window
            };

            let flash_storage_nearly_full = flash_storage.is_too_full_to_start_new_recordings();
            if flash_storage_nearly_full && !logged_flash_storage_nearly_full {
                event_logger.log_event(
                    LoggerEvent::new(
                        LoggerEventKind::FlashStorageNearlyFull,
                        synced_date_time.get_timestamp_micros(&timer),
                    ),
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

                    sio.fifo.write(Core1Task::RequestReset.into());
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
                            LoggerEvent::new(
                                LoggerEventKind::ToldRpiToSleep,
                                synced_date_time.get_timestamp_micros(&timer),
                            ),
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
                                    synced_date_time.get_timestamp_micros(&timer),
                                ),
                                &mut flash_storage,
                            );
                            logged_pi_powered_down = true;
                        }

                        //only reboot into audio mode if pi is powered down and rp2040 is asleep
                        //so we can keep the thermal preview up as long as the PI is on.
                        match device_config.config().audio_mode {
                            AudioMode::AudioAndThermal | AudioMode::AudioOrThermal => {
                                //just reboot and it will go into audio branch
                                info!("Reset as thermal finished and time for audio");
                                sio.fifo.write(Core1Task::RequestReset.into());

                                loop {
                                    nop();
                                }
                            }
                            _ => (),
                        }
                        // NOTE: Calculate the start of the next recording window, set the RTC wake-up alarm,
                        //  and ask for the rp2040 to be put to sleep.
                        let next_recording_window_start = if !dev_mode {
                            device_config
                                .next_recording_window_start(&synced_date_time.date_time_utc)
                        } else {
                            // In dev mode, we always set the restart alarm for 2 minutes time.
                            synced_date_time.date_time_utc + chrono::Duration::minutes(2)
                        };
                        let enabled_alarm = shared_i2c.enable_alarm(&mut delay);
                        if enabled_alarm.is_err() {
                            error!("Failed enabling alarm");
                            event_logger.log_event(
                                LoggerEvent::new(
                                    LoggerEventKind::RtcCommError,
                                    synced_date_time.get_timestamp_micros(&timer),
                                ),
                                &mut flash_storage,
                            );
                        }
                        if let Ok(_) =
                            shared_i2c.set_wakeup_alarm(&next_recording_window_start, &mut delay)
                        {
                            let alarm_enabled = shared_i2c
                                .alarm_interrupt_enabled(&mut delay)
                                .unwrap_or(false);
                            info!("Wake up alarm interrupt enabled {}", alarm_enabled);
                            if alarm_enabled {
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::SetAlarm(
                                            next_recording_window_start.and_utc().timestamp_micros()
                                                as u64,
                                        ),
                                        synced_date_time.get_timestamp_micros(&timer),
                                    ),
                                    &mut flash_storage,
                                );

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
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::Rp2040Sleep,
                                        synced_date_time.get_timestamp_micros(&timer),
                                    ),
                                    &mut flash_storage,
                                );
                                if let Ok(_) =
                                    shared_i2c.tell_attiny_to_power_down_rp2040(&mut delay)
                                {
                                    info!("Sleeping");
                                } else {
                                    error!("Failed sending sleep request to attiny");
                                    event_logger.log_event(
                                        LoggerEvent::new(
                                            LoggerEventKind::AttinyCommError,
                                            synced_date_time.get_timestamp_micros(&timer),
                                        ),
                                        &mut flash_storage,
                                    );
                                }
                            } else {
                                error!("Alarm was not properly enabled");
                                event_logger.log_event(
                                    LoggerEvent::new(
                                        LoggerEventKind::RtcCommError,
                                        synced_date_time.get_timestamp_micros(&timer),
                                    ),
                                    &mut flash_storage,
                                );
                            }
                            // Now we can put ourselves to sleep.
                        } else {
                            error!("Failed setting wake alarm, can't go to sleep");
                            event_logger.log_event(
                                LoggerEvent::new(
                                    LoggerEventKind::RtcCommError,
                                    synced_date_time.get_timestamp_micros(&timer),
                                ),
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
                    "Check pi power down state took {}",
                    (timer.get_counter() - check_power_down_state_start).to_micros()
                );
            } else if !is_outside_recording_window && !device_config.use_low_power_mode() {
                if wake_raspberry_pi(&mut shared_i2c, &mut delay) {
                    event_logger.log_event(
                        LoggerEvent::new(
                            LoggerEventKind::ToldRpiToWake(WakeReason::ThermalHighPower),
                            synced_date_time.get_timestamp_micros(&timer),
                        ),
                        &mut flash_storage,
                    );
                }
            } else if is_outside_recording_window && !made_shutdown_status_recording {
                making_status_recording = true;
                //force shutdown recording outside of window
            }

            // Make sure timing is as close as possible to the non-sync case
            let sync_rtc_end = timer.get_counter();
            let sync_time = (sync_rtc_end - sync_rtc_start).to_micros() as i32;
            let additional_wait = (expected_rtc_sync_time_us - sync_time).min(0);
            if additional_wait > 0 {
                warn!(
                    "Additional wait after RTC sync {}µs, total sync time {}",
                    additional_wait, sync_time
                );
                delay.delay_us(additional_wait as u32);
            } else {
                warn!("I2C messages took {}µs", sync_time)
            }
        } else {
            // Increment the datetime n frame's worth.
            // NOTE: We only get the actual date from the RTC every minutes' worth of frames, so that we
            // don't have too many stalls trying to communicate via I2C with the RTC.

            let mut incremented_datetime = synced_date_time.date_time_utc.clone();

            let frames_elapsed = if frame_header_is_valid {
                get_frames_elapsed(&frame_telemetry, &prev_frame_telemetry)
            } else {
                1
            };
            if frames_elapsed > 100 {
                warn!("Got {} elapsed frames, ignoring", frames_elapsed);
            } else {
                incremented_datetime += Duration::milliseconds(115 * frames_elapsed as i64);
                if incremented_datetime > synced_date_time.date_time_utc {
                    synced_date_time.set(incremented_datetime, &timer);
                }
            }

            // Spend the same time as we would otherwise use querying the RTC to keep frame-times
            //  about the same
            delay.delay_us(expected_rtc_sync_time_us as u32);
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
                if !audio_pending && synced_date_time.date_time_utc > next_audio_alarm {
                    //Should we be checking alarm triggered? or just us ethis and clear alarm
                    audio_pending = true;
                    let cur_time = &synced_date_time.date_time_utc;

                    info!(
                            "Audio recording is pending because time {}:{} from {}:{} and timer {:?} startup {:?} is after {}:{} ",
                            cur_time.hour(),
                            cur_time.minute(),
                            synced_date_time.date_time_utc.hour(),
                            synced_date_time.date_time_utc.minute(),
                            timer.get_counter().ticks(),
                            synced_date_time.timer_offset.ticks(),
                            next_audio_alarm.hour(),
                            next_audio_alarm.minute()
                            )
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
                    || &(synced_date_time.date_time_utc + Duration::minutes(2)) < window_end
                {
                    //hanldes case where thermal recorder is doing recording
                    if let Ok(is_recording) = shared_i2c.get_is_recording(&mut delay) {
                        if !is_recording {
                            info!("Taking audio recording");
                            //make audio rec now
                            let _ = shared_i2c.tc2_agent_take_audio_rec(&mut delay);
                            sio.fifo.write(Core1Task::RequestReset.into());
                            loop {
                                // Wait to be reset
                                nop();
                            }
                        }
                    }
                } else {
                    info!("Not doing audio until after shutdown status")
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

fn get_frames_elapsed(
    frame_telemetry: &Telemetry,
    prev_frame_telemetry: &Option<Telemetry>,
) -> i32 {
    if let Some(prev_telemetry) = prev_frame_telemetry {
        if frame_telemetry.frame_num != prev_telemetry.frame_num + 1 {
            let frames_elapsed = frame_telemetry.frame_num as i32 - prev_telemetry.frame_num as i32;
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
    } else {
        1
    }
}
