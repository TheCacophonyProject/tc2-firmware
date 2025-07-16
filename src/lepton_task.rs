use crate::bsp::pac::Peripherals;
use crate::cptv_encoder::FRAME_WIDTH;
use crate::frame_processing::{
    Core0Task, FrameBuffer, NUM_LEPTON_SEGMENTS, NUM_LINES_PER_LEPTON_SEGMENT, StaticFrameBuffer,
};
use crate::lepton::{FFCStatus, LeptonFirmwareInfo, LeptonModule, LeptonPins, init_lepton_module};
use crate::lepton_telemetry::Telemetry;
use crate::utils::restart;
use crate::{FFC_INTERVAL_MS, bsp};
use bsp::hal::gpio::{FunctionSio, Interrupt, Pin, PinId, PullNone, SioInput};
use bsp::hal::pac::RESETS;
use bsp::hal::rosc::RingOscillator;
use bsp::hal::sio::SioFifo;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use core::cell::RefCell;
use crc::{CRC_16_XMODEM, Crc};
use critical_section::Mutex;
use defmt::{info, warn};
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::{Sio, Timer, Watchdog};

pub type FramePacketData = [u8; FRAME_WIDTH];
pub type FrameSegments = [[FramePacketData; NUM_LINES_PER_LEPTON_SEGMENT]; NUM_LEPTON_SEGMENTS];

pub const LEPTON_SPI_CLOCK_FREQ: u32 = 40_000_000;
fn go_dormant_until_next_vsync(
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    lepton: &mut LeptonModule,
    rosc_freq: HertzU32,
    got_sync: bool,
) -> RingOscillator<bsp::hal::rosc::Enabled> {
    if got_sync && lepton.is_awake() {
        lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
        unsafe { rosc.dormant() };
        let disabled_rosc = RingOscillator::new(rosc.free());
        let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq);
        lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
        initialized_rosc
    } else {
        rosc
    }
}

fn go_dormant_until_woken<T: PinId>(
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    wake_pin: &mut Pin<T, FunctionSio<SioInput>, PullNone>,
    lepton: &mut LeptonModule,
    rosc_freq: HertzU32,
) -> RingOscillator<bsp::hal::rosc::Enabled> {
    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    wake_pin.set_dormant_wake_enabled(Interrupt::EdgeHigh, true);
    unsafe { rosc.dormant() };
    // Woken by pin
    let disabled_rosc = RingOscillator::new(rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq);
    wake_pin.set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    initialized_rosc
}

pub fn lepton_core1_task(
    lepton_pins: LeptonPins,
    mut watchdog: Watchdog,
    system_clock_freq: HertzU32,
    peripheral_clock_freq: HertzU32,
    rosc: &RingOscillator<bsp::hal::rosc::Enabled>,
    static_frame_buffer_a: StaticFrameBuffer,
    static_frame_buffer_b: StaticFrameBuffer,
    is_high_power_mode: bool,
    timer: Timer,
) -> ! {
    // This task runs on the second core, so we need to steal the peripherals.
    // Core peripherals are per core, so we can just take our copy (but the current cortex-m API
    // makes us steal it anyway)

    let mut peripherals = unsafe { Peripherals::steal() };
    let sio = Sio::new(peripherals.SIO);
    let mut fifo = sio.fifo;

    let mut lepton = init_lepton_module(
        peripherals.SPI0,
        peripherals.I2C0,
        system_clock_freq,
        &mut peripherals.RESETS,
        timer,
        lepton_pins,
    );

    let radiometric_mode = lepton.radiometric_mode_enabled().unwrap_or(false);
    let lepton_serial = lepton.get_camera_serial().map_or(0, |x| x);
    let lepton_firmware_version = lepton
        .get_firmware_version()
        .map_or(LeptonFirmwareInfo::default(), |x| x);
    let LeptonFirmwareInfo {
        gpp_major,
        gpp_minor,
        gpp_build,
        dsp_major,
        dsp_minor,
        dsp_build,
    } = lepton_firmware_version;

    info!(
        "Camera firmware versions: main: {}.{}.{}, dsp: {}.{}.{}, serial #{}",
        gpp_major, gpp_minor, gpp_build, dsp_major, dsp_minor, dsp_build, lepton_serial
    );

    let result = fifo.read_blocking();
    defmt::assert_eq!(result, Core0Task::ReadyToReceiveLeptonConfig.into());
    let main_lepton_firmware = LittleEndian::read_u32(&[gpp_major, gpp_minor, gpp_build, 0]);
    let dsp_lepton_firmware = LittleEndian::read_u32(&[dsp_major, dsp_minor, dsp_build, 0]);
    fifo.write_blocking(Core0Task::SendIntercoreArray.into());
    fifo.write_blocking(4);
    fifo.write_blocking(if radiometric_mode { 2 } else { 1 });
    fifo.write_blocking(lepton_serial);
    fifo.write_blocking(main_lepton_firmware);
    fifo.write_blocking(dsp_lepton_firmware);

    let result = fifo.read_blocking();
    if result == Core0Task::RequestReset.into() {
        info!("Got reset request before frame acquisition startup");
        restart(&mut watchdog);
    }
    defmt::assert_eq!(result, Core0Task::Ready.into());
    frame_acquisition_loop(
        rosc,
        &mut lepton,
        &mut fifo,
        peripheral_clock_freq,
        &mut peripherals.RESETS,
        static_frame_buffer_a,
        static_frame_buffer_b,
        watchdog,
        is_high_power_mode,
    );
}

/// # Panics
///
/// Does not panic, spi read is actually infallible
#[allow(clippy::too_many_lines)]
pub fn frame_acquisition_loop(
    rosc: &RingOscillator<bsp::hal::rosc::Enabled>, // NOTE: not using dormant at the moment, so don't need mut
    lepton: &mut LeptonModule,
    sio_fifo: &mut SioFifo,
    peripheral_clock_freq: HertzU32,
    resets: &mut RESETS,
    frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    mut watchdog: Watchdog,
    is_high_power_mode: bool,
) -> ! {
    let mut selected_frame_buffer = 0;
    let mut frame_counter = 0u32;
    let mut unverified_frame_counter = 0;
    let mut prev_frame_counter = 0;

    // NOTE: Do we want to do CRC checks?  What is the energy implication of this?
    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
    let do_crc_check = true;
    let mut crc_buffer = [0u8; 164];
    let mut prev_segment_was_4 = false;
    let mut scanline_buffer = [0u16; 82];

    let mut initial_ffc = Some(());
    let mut got_sync = false;
    let mut valid_frame_current_segment_num = 0u16;
    let mut started_segment = false;
    let mut attempt = 1;
    let mut transferring_prev_frame = false;
    let mut is_recording = false;
    let mut recording_ended = false;
    let mut prev_frame_needs_transfer = false;

    // Each frame has 4 equally sized segments.
    let last_segment_num_for_frame = 4;
    // Each frame segment is comprised of 61 packets, zero based.
    let last_packet_id_for_segment = 60;
    // There are 2 "discard" frames between each real frame, each with 4 segments
    let total_segments_including_dummy_frames = 12;
    // Only packet 20 has a valid segment number, for some reason.
    let packet_id_with_valid_segment_num = 20;
    // Track the frame segment num including "discard" frames.
    let mut current_segment_num = 0;
    // Do FFC every 20 mins?
    let mut ffc_requested_frame = None;
    let mut ffc_pending_time = None;
    let mut safe_to_execute_ffc = true;
    let mut seen_telemetry_revision = [0u8, 0u8];
    let mut times_telemetry_revision_stable = -1;
    let mut frames_seen = 0;
    let mut last_frame_seen = None;

    // should not feed watchdog if we dont receive a message and are in low power mode
    'frame_loop: loop {
        if got_sync || is_recording {
            watchdog.feed();
        }
        if got_sync {
            current_segment_num += 1;
            if current_segment_num > total_segments_including_dummy_frames {
                current_segment_num = 1;
            }
        }
        if got_sync
            && current_segment_num > last_segment_num_for_frame
            && !is_recording
            && !transferring_prev_frame
        {
            // Go to sleep, skip dummy segment

            // FIXME - Whenever recording starts, we lose a frame here. It's the all important first frame of the recording.
            //  Let's just use a bit more power for now until we figure out how to fix this.
            //rosc = go_dormant_until_next_vsync(rosc, lepton, clocks.system_clock.freq(), got_sync);
            continue 'frame_loop;
        }
        if !transferring_prev_frame && prev_frame_needs_transfer {
            // Initiate the transfer of the previous frame
            sio_fifo.write(Core0Task::ReceiveFrame.into());
            sio_fifo.write(selected_frame_buffer);
            safe_to_execute_ffc = sio_fifo.read_blocking() == 1;
            if selected_frame_buffer == 0 {
                selected_frame_buffer = 1;
            } else {
                selected_frame_buffer = 0;
            }
            transferring_prev_frame = true;
            prev_frame_needs_transfer = false;
        }

        // Read the next frame
        let mut prev_packet_id = 0;
        'scanline: loop {
            // This is one scanline
            let packet = lepton.transfer(&mut scanline_buffer).unwrap();
            let packet_header = packet[0];
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            if is_discard_packet {
                continue 'scanline;
            }
            let packet_id = usize::from(packet_header & 0x0fff);
            let is_valid_packet_num = packet_id <= last_packet_id_for_segment;

            if packet_id == 0 {
                prev_packet_id = 0;
                started_segment = true;
                // If we don't know, always start at segment 1 so that things will be
                // written out.
                if !got_sync || valid_frame_current_segment_num == 0 || prev_segment_was_4 {
                    valid_frame_current_segment_num = 1;
                }
                let telemetry = Telemetry::from_bytes(bytemuck::cast_slice(&scanline_buffer[2..]));
                unverified_frame_counter = telemetry.frame_num;

                if got_sync && valid_frame_current_segment_num == 1 {
                    // Check if we need an FFC
                    if unverified_frame_counter == prev_frame_counter + 2
                        && ffc_requested_frame.is_none()
                        && !is_recording
                        && telemetry.msec_on != 0
                        && (telemetry.time_at_last_ffc != 0 || initial_ffc.is_some())
                    {
                        let temperature_shifted_too_much_since_last_ffc =
                            (telemetry.fpa_temp_c_at_last_ffc - telemetry.fpa_temp_c).abs() > 1.5;
                        let ffc_frame_diff =
                            telemetry.msec_on.saturating_sub(telemetry.time_at_last_ffc);
                        let time_for_ffc = ffc_frame_diff > FFC_INTERVAL_MS;
                        // If time on ms is zero, that indicates a corrupt/invalid frame.
                        if telemetry.msec_on < telemetry.time_at_last_ffc {
                            warn!(
                                "Time on less than last FFC: time_on_ms: {}, last_ffc_ms: {}",
                                telemetry.msec_on, telemetry.time_at_last_ffc
                            );
                        } else if (time_for_ffc || temperature_shifted_too_much_since_last_ffc)
                            && telemetry.ffc_status != FFCStatus::InProgress
                        {
                            if temperature_shifted_too_much_since_last_ffc {
                                // NOTE: If the sensor temperature has shifted by >1.5C° since the
                                //  previous FFC, we also want to trigger an FFC.
                                info!(
                                    "Temp diff between FFCs {}:{} {}",
                                    telemetry.fpa_temp_c,
                                    telemetry.fpa_temp_c_at_last_ffc,
                                    (telemetry.fpa_temp_c_at_last_ffc - telemetry.fpa_temp_c).abs()
                                );
                            }
                            if let Some(pending_frame) = ffc_pending_time {
                                if frame_counter.saturating_sub(pending_frame) > 20 {
                                    ffc_pending_time = None;
                                    ffc_requested_frame = Some(frame_counter);
                                }
                            } else {
                                ffc_requested_frame = Some(frame_counter);
                            }
                        }

                        // Sometimes the header is invalid, but the frame becomes valid and gets sync.
                        // Because the telemetry revision is static across frame headers we can detect this
                        // case and not send the frame, as it may cause false triggers.
                        if times_telemetry_revision_stable > -1
                            && times_telemetry_revision_stable <= 2
                        {
                            if seen_telemetry_revision[0] == telemetry.revision[0]
                                && seen_telemetry_revision[1] == telemetry.revision[1]
                            {
                                times_telemetry_revision_stable += 1;
                            } else {
                                times_telemetry_revision_stable = -1;
                            }
                        }
                        if times_telemetry_revision_stable == -1 {
                            // Initialise seen telemetry revision.
                            seen_telemetry_revision =
                                [telemetry.revision[0], telemetry.revision[1]];
                            times_telemetry_revision_stable += 1;
                        }
                        if times_telemetry_revision_stable > 2
                            && (seen_telemetry_revision[0] != telemetry.revision[0]
                                || seen_telemetry_revision[1] != telemetry.revision[1])
                        {
                            // We have a misaligned/invalid frame.
                            warn!("Got misaligned frame header");
                            got_sync = false;
                        }
                    }
                }
            } else if packet_id == packet_id_with_valid_segment_num {
                // Packet 20 is the only one that contains a meaningful segment number
                let segment_num = packet_header >> 12;
                if prev_packet_id == packet_id_with_valid_segment_num - 1 && segment_num == 1 {
                    prev_frame_counter = frame_counter;
                    if unverified_frame_counter < prev_frame_counter {
                        warn!("Frames appear to be out of sync / offset");
                        got_sync = false;
                    }
                    if unverified_frame_counter > frame_counter + 1000 {
                        warn!("(2) Frames appear to be out of sync / offset");
                        got_sync = false;
                    }
                    frame_counter = unverified_frame_counter;
                }
                if frames_seen % 9 == 0 {
                    // FIXME We seem to be able to get into this state, without out of sync ever triggering,
                    //  and frame_counter is the same each iteration through the loop.
                    // info!(
                    //     "Core0 got frame #{} {}, synced {}",
                    //     frames_seen, frame_counter, got_sync
                    // );
                }
                frames_seen += 1;

                // See if we're starting a frame, or ending it.
                if valid_frame_current_segment_num > 1
                    && valid_frame_current_segment_num < 5
                    && segment_num != valid_frame_current_segment_num
                {
                    // Segment order mismatch.
                    warn!(
                        "Segment order mismatch error: stored {}, this {}",
                        current_segment_num, segment_num
                    );
                    started_segment = false;
                    prev_segment_was_4 = false;

                    lepton.wait_for_ready(false);
                    lepton.reset_spi(
                        resets,
                        peripheral_clock_freq,
                        LEPTON_SPI_CLOCK_FREQ.Hz(),
                        true,
                    );
                    if !is_recording && initial_ffc.take().is_some() {
                        let _success = lepton.do_ffc();
                    }
                    break 'scanline;
                }
                valid_frame_current_segment_num = segment_num;
                if valid_frame_current_segment_num == 0 {
                    started_segment = false;
                    break 'scanline;
                }
            }

            let is_valid_segment_num = valid_frame_current_segment_num > 0
                && valid_frame_current_segment_num <= last_segment_num_for_frame;
            let packets_are_in_order = packet_id == 0 || packet_id == prev_packet_id + 1;
            if is_valid_segment_num
                && is_valid_packet_num
                && started_segment
                && packets_are_in_order
            {
                if do_crc_check {
                    let crc = scanline_buffer[1].to_le();
                    BigEndian::write_u16_into(&scanline_buffer, &mut crc_buffer);
                    crc_buffer[0] &= 0x0f;
                    crc_buffer[2] = 0;
                    crc_buffer[3] = 0;
                    if crc_check.checksum(&crc_buffer) != crc
                        && packet_id != 0
                        && valid_frame_current_segment_num != 1
                    {
                        warn!(
                            "Checksum fail on packet {}, segment {}",
                            packet_id, current_segment_num
                        );
                    }
                }

                // Copy the line out to the appropriate place in the current segment buffer.
                critical_section::with(|cs| {
                    let segment_index =
                        usize::from(valid_frame_current_segment_num.clamp(1, 4) - 1);
                    // NOTE: We may be writing the incorrect seg number here initially, but it will always be
                    //  set correctly when we reach packet 20, assuming we do manage to write out a full segment.
                    let buffer = if selected_frame_buffer == 0 {
                        frame_buffer_local
                    } else {
                        frame_buffer_local_2
                    };

                    if let Some(buffer) = buffer.borrow_ref_mut(cs).as_mut() {
                        LittleEndian::write_u16_into(
                            &scanline_buffer[2..],
                            buffer.packet(segment_index, packet_id),
                        );
                    } else {
                        defmt::error!("Failed to write to frame buffer");
                    }
                });

                let is_last_segment = valid_frame_current_segment_num == 4;
                prev_segment_was_4 = is_last_segment;
                if packet_id == last_packet_id_for_segment {
                    if is_last_segment {
                        if safe_to_execute_ffc && ffc_requested_frame.take().is_some() {
                            info!("Requesting needed FFC");
                            match lepton.do_ffc() {
                                Ok(success) => {
                                    if success {
                                        ffc_pending_time = Some(frame_counter);
                                    } else {
                                        info!("Failed to request FFC (i2c err)");
                                    }
                                }
                                Err(e) => {
                                    ffc_requested_frame = Some(frame_counter);
                                    info!("Failed to request FFC {:?}", e);
                                }
                            }
                        } else if initial_ffc.take().is_some() {
                            info!("Requesting initial FFC");
                            let success = lepton.do_ffc();
                            if let Err(e) = lepton.do_ffc() {
                                initial_ffc = Some(());
                                info!("Failed to request FFC {:?}", e);
                            }
                        }
                        if !got_sync && frame_counter == prev_frame_counter + 1 {
                            // Only set got sync if frame_count is = frame_count + 1 from previous frame.
                            got_sync = true;
                            current_segment_num = valid_frame_current_segment_num;
                            warn!(
                                "Got sync at seg {} frame {} prev frame {}",
                                current_segment_num, frame_counter, prev_frame_counter
                            );
                        }

                        attempt = 0;
                        prev_frame_needs_transfer = true;

                        if let Some(last_frame_seen) = last_frame_seen {
                            if got_sync && last_frame_seen != frame_counter - 1 {
                                warn!("Looks like we lost sync");
                                got_sync = false;
                                prev_frame_needs_transfer = false;
                            }
                        }
                        last_frame_seen = Some(frame_counter);
                    } else {
                        // Increment in good faith if we're on the last packet of a valid segment
                        valid_frame_current_segment_num += 1;
                    }
                    started_segment = false;
                    break 'scanline;
                }
            }

            // We only mark a segment as started if the packet num was 0 or 20.
            if started_segment && !packets_are_in_order {
                if got_sync {
                    got_sync = false;
                    warn!(
                        "Lost sync at seg {}, frame {}, prev frame {}",
                        current_segment_num, frame_counter, prev_frame_counter
                    );
                    current_segment_num = 0;
                }
                // Packet order mismatch
                attempt += 1;
                valid_frame_current_segment_num = 0;
                started_segment = false;
                prev_segment_was_4 = false;

                if attempt > 250 && attempt % 5 == 0 {
                    if got_sync {
                        warn!(
                            "Packet order mismatch current: {}, prev: {}, seg {} #{}",
                            packet_id, prev_packet_id, current_segment_num, attempt
                        );
                    }
                    if attempt < 500 {
                        lepton.wait_for_ready(false);
                        lepton.reset_spi(
                            resets,
                            peripheral_clock_freq,
                            LEPTON_SPI_CLOCK_FREQ.Hz(),
                            false,
                        );
                        if initial_ffc.take().is_some() {
                            info!("Requesting FFC");
                            let success = lepton.do_ffc();
                            match success {
                                Ok(success) => {
                                    info!("Success requesting FFC");
                                }
                                Err(e) => {
                                    initial_ffc = Some(());
                                    info!("Failed to request FFC {:?}", e);
                                }
                            }
                        }
                    } else {
                        initial_ffc = Some(());
                        current_segment_num = 0;
                        frames_seen = 0;
                        last_frame_seen = None;
                        lepton.power_down_sequence();
                        lepton.power_on_sequence();
                    }
                    break 'scanline;
                }
                continue 'scanline;
            }
            prev_packet_id = packet_id;

            'message_loop: loop {
                match sio_fifo.read() {
                    None => break 'message_loop,
                    Some(message) => {
                        if message == Core0Task::StartRecording.into() {
                            is_recording = true;
                        } else if message == Core0Task::EndRecording.into() {
                            recording_ended = true;
                        } else if message == Core0Task::ReadyToSleep.into() {
                            info!("Powering down lepton module");
                            lepton.power_down_sequence();
                            sio_fifo.write(Core0Task::LeptonReadyToSleep.into());
                        } else if message == Core0Task::FrameProcessingComplete.into() {
                            transferring_prev_frame = false;
                            prev_frame_needs_transfer = false;
                        } else if message == Core0Task::RequestReset.into() {
                            warn!("Request reset");
                            restart(&mut watchdog);
                        }
                    }
                }
            }
        }

        // This block prevents a frame sync issue when we *end* recording

        // NOTE: If we're not transferring the previous frame, and the current segment is the second
        //  to last for a real frame, we can go dormant until the next vsync interrupt.
        if recording_ended {
            // && !transferring_prev_frame && current_segment_num == 3 {
            is_recording = false;
            recording_ended = false;
        }
        // if !is_recording && !transferring_prev_frame && current_segment_num == 3 {
        //     rosc =
        //         go_dormant_until_next_vsync(rosc, lepton, clocks.system_clock.freq(), got_sync);
        // } else if current_segment_num == 3 {
        //     //warn!("Overrunning frame time");
        // }
    }
}
