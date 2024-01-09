use crate::core1_task::Core1Task;
use crate::lepton::{read_telemetry, FFCStatus, LeptonModule};
use crate::utils::u16_slice_to_u8;
use crate::{bsp, FrameBuffer, FFC_INTERVAL_MS};
use bsp::hal::clocks::ClocksManager;
use bsp::hal::gpio::{FunctionSio, Interrupt, Pin, PinId, PullNone, SioInput};
use bsp::hal::pac::RESETS;
use bsp::hal::rosc::RingOscillator;
use bsp::hal::sio::SioFifo;
use bsp::hal::Clock;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use core::cell::RefCell;
use cortex_m::delay::Delay;
use crc::{Crc, CRC_16_XMODEM};
use critical_section::Mutex;
use defmt::{info, warn};
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::Timer;

pub const LEPTON_SPI_CLOCK_FREQ: u32 = 40_000_000;
fn go_dormant_until_next_vsync(
    rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    lepton: &mut LeptonModule,
    rosc_freq: HertzU32,
    got_sync: bool,
) -> RingOscillator<bsp::hal::rosc::Enabled> {
    if got_sync && lepton.is_awake() {
        lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
        let dormant_rosc = unsafe { rosc.dormant() };
        let disabled_rosc = RingOscillator::new(dormant_rosc.free());
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
    let dormant_rosc = unsafe { rosc.dormant() };
    // Woken by pin
    let disabled_rosc = RingOscillator::new(dormant_rosc.free());
    let initialized_rosc = disabled_rosc.initialize_with_freq(rosc_freq);
    wake_pin.set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    initialized_rosc
}

pub fn frame_acquisition_loop(
    mut rosc: RingOscillator<bsp::hal::rosc::Enabled>,
    lepton: &mut LeptonModule,
    sio_fifo: &mut SioFifo,
    clocks: &ClocksManager,
    delay: &mut Delay,
    resets: &mut RESETS,
    frame_buffer_local: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    frame_buffer_local_2: &'static Mutex<RefCell<Option<&mut FrameBuffer>>>,
    timer: &Timer,
) -> ! {
    let mut selected_frame_buffer = 0;
    let mut frame_counter = 0;
    let mut unverified_frame_counter = 0;
    let mut prev_frame_counter = 0;

    // NOTE: Do we want to do CRC checks?  What is the energy implication of this?
    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
    let do_crc_check = true;
    let mut crc_buffer = [0u8; 164];

    let mut prev_segment_was_4 = false;
    let mut scanline_buffer = [0u16; 82];

    let mut has_done_initial_ffc = false;
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
    let mut needs_ffc = false;
    let mut ffc_requested = false;

    let mut frames_seen = 0;
    let mut last_frame_seen = None;
    let start = timer.get_counter();
    let end = timer.get_counter();

    'frame_loop: loop {
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

            // FIXME - Whenever recording starts, we lose a frame here.  Let's just use a bit more power
            //  for now until we figure out how to fix this.
            // rosc = go_dormant_until_next_vsync(rosc, lepton, clocks.system_clock.freq());
            continue 'frame_loop;
        }
        if !transferring_prev_frame && prev_frame_needs_transfer {
            // Initiate the transfer of the previous frame
            sio_fifo.write(Core1Task::ReceiveFrame.into());
            sio_fifo.write(selected_frame_buffer);
            if selected_frame_buffer == 0 {
                selected_frame_buffer = 1;
            } else {
                selected_frame_buffer = 0;
            }
            transferring_prev_frame = true;
            prev_frame_needs_transfer = false;
        }
        {
            // Read the next frame
            let mut prev_packet_id = -1;
            'scanline: loop {
                // This is one scanline
                let packet = lepton.transfer(&mut scanline_buffer).unwrap();
                let packet_header = packet[0];
                let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
                let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
                if is_discard_packet {
                    continue 'scanline;
                }
                let packet_id = (packet_header & 0x0fff) as isize;
                let is_valid_packet_num = packet_id >= 0 && packet_id <= last_packet_id_for_segment;

                if packet_id == 0 {
                    prev_packet_id = -1;
                    started_segment = true;
                    // If we don't know, always start at segment 1 so that things will be
                    // written out.
                    if !got_sync || valid_frame_current_segment_num == 0 || prev_segment_was_4 {
                        valid_frame_current_segment_num = 1;
                    }
                    let telemetry =
                        read_telemetry(unsafe { u16_slice_to_u8(&scanline_buffer[2..]) });
                    unverified_frame_counter = telemetry.frame_num;

                    if got_sync && valid_frame_current_segment_num == 1 {
                        // Check if we need an FFC
                        if unverified_frame_counter == prev_frame_counter + 2 {
                            if !is_recording
                                && telemetry.msec_on != 0
                                && (telemetry.time_at_last_ffc != 0 || !has_done_initial_ffc)
                            {
                                // If time on ms is zero, that indicates a corrupt/invalid frame.
                                if telemetry.msec_on < telemetry.time_at_last_ffc {
                                    warn!(
                                        "Time on less than last FFC: time_on_ms: {}, last_ffc_ms: {}",
                                        telemetry.msec_on, telemetry.time_at_last_ffc
                                    );
                                } else if telemetry.msec_on - telemetry.time_at_last_ffc
                                    > FFC_INTERVAL_MS
                                    && telemetry.ffc_status != FFCStatus::Imminent
                                    && telemetry.ffc_status != FFCStatus::InProgress
                                {
                                    needs_ffc = true;
                                    ffc_requested = false;
                                }
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
                            delay,
                            resets,
                            clocks.peripheral_clock.freq(),
                            LEPTON_SPI_CLOCK_FREQ.Hz(),
                            true,
                        );
                        if !is_recording && !has_done_initial_ffc {
                            let _success = lepton.do_ffc();
                            has_done_initial_ffc = true;
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
                let packets_are_in_order = packet_id == prev_packet_id + 1;
                if is_valid_segment_num
                    && is_valid_packet_num
                    && started_segment
                    && packets_are_in_order
                {
                    if do_crc_check {
                        let crc = scanline_buffer[1].to_le();
                        BigEndian::write_u16_into(&scanline_buffer, &mut crc_buffer);
                        crc_buffer[0] = crc_buffer[0] & 0x0f;
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
                            ((valid_frame_current_segment_num as u8).max(1).min(4) - 1) as usize;
                        // NOTE: We may be writing the incorrect seg number here initially, but it will always be
                        //  set correctly when we reach packet 20, assuming we do manage to write out a full segment.
                        let buffer = if selected_frame_buffer == 0 {
                            frame_buffer_local
                        } else {
                            frame_buffer_local_2
                        };
                        LittleEndian::write_u16_into(
                            &scanline_buffer[2..],
                            buffer
                                .borrow_ref_mut(cs)
                                .as_mut()
                                .unwrap()
                                .packet(segment_index, packet_id as usize),
                        );
                    });

                    let is_last_segment = valid_frame_current_segment_num == 4;
                    prev_segment_was_4 = is_last_segment;
                    if packet_id == last_packet_id_for_segment {
                        if is_last_segment {
                            if needs_ffc && !ffc_requested {
                                ffc_requested = true;
                                info!("Requesting needed FFC");
                                let success = lepton.do_ffc();
                                match success {
                                    Ok(success) => {
                                        needs_ffc = false;
                                        info!("Success requesting needed FFC");
                                    }
                                    Err(e) => {
                                        info!("Failed to request FFC {:?}", e);
                                    }
                                }
                            } else if !has_done_initial_ffc {
                                info!("Requesting initial FFC");
                                let success = lepton.do_ffc();
                                match success {
                                    Ok(success) => {
                                        has_done_initial_ffc = true;
                                        info!("Success requesting initial FFC");
                                    }
                                    Err(e) => {
                                        info!("Failed to request FFC {:?}", e);
                                    }
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
                                if last_frame_seen != frame_counter - 1 {
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
                        warn!(
                            "Packet order mismatch current: {}, prev: {}, seg {} #{}",
                            packet_id, prev_packet_id, current_segment_num, attempt
                        );
                        lepton.wait_for_ready(false);
                        lepton.reset_spi(
                            delay,
                            resets,
                            clocks.peripheral_clock.freq(),
                            LEPTON_SPI_CLOCK_FREQ.Hz(),
                            false,
                        );
                        if !has_done_initial_ffc {
                            info!("Requesting FFC");
                            let success = lepton.do_ffc();
                            match success {
                                Ok(success) => {
                                    info!("Success requesting FFC");
                                }
                                Err(e) => {
                                    info!("Failed to request FFC {:?}", e);
                                }
                            }
                            has_done_initial_ffc = true;
                        }
                        break 'scanline;
                    } else {
                        continue 'scanline;
                    }
                }
                prev_packet_id = packet_id;

                if transferring_prev_frame {
                    // Could read blocking, but need to increment current_segment_num appropriately?
                    if let Some(message) = sio_fifo.read() {
                        if message == Core1Task::StartRecording.into() {
                            is_recording = true;
                            if let Some(message) = sio_fifo.read() {
                                if message == Core1Task::FrameProcessingComplete.into() {
                                    transferring_prev_frame = false;
                                    prev_frame_needs_transfer = false;
                                }
                            }
                        } else if message == Core1Task::EndRecording.into() {
                            recording_ended = true;
                            if let Some(message) = sio_fifo.read() {
                                if message == Core1Task::FrameProcessingComplete.into() {
                                    transferring_prev_frame = false;
                                    prev_frame_needs_transfer = false;
                                }
                            }
                        } else if message == Core1Task::ReadyToSleep.into() {
                            info!("Powering down lepton module");
                            lepton.power_down_sequence(delay);
                            if let Some(message) = sio_fifo.read() {
                                if message == Core1Task::FrameProcessingComplete.into() {
                                    transferring_prev_frame = false;
                                    prev_frame_needs_transfer = false;
                                }
                            }
                            sio_fifo.write(255);
                        } else if message == Core1Task::FrameProcessingComplete.into() {
                            transferring_prev_frame = false;
                            prev_frame_needs_transfer = false;
                        }
                    }
                    // if !transferring_prev_frame || recording_started {
                    //     end = timer.get_counter();
                    //     if recording_started
                    //         || (is_recording && frame_counter % 10 == 0)
                    //         || (frame_counter % 10 == 0 && current_segment_num == 3)
                    //     {
                    //         info!(
                    //             "#{} Frame time {}µs, current seg {} (r started {})",
                    //             frame_counter,
                    //             (end - start).to_micros(),
                    //             current_segment_num,
                    //             recording_started
                    //         );
                    //     }
                    // }
                }
            }
        }
        {
            // This block prevents a frame sync issue when we *end* recording

            // NOTE: If we're not transferring the previous frame, and the current segment is the second
            //  to last for a real frame, we can go dormant until the next vsync interrupt.
            if recording_ended && !transferring_prev_frame && current_segment_num == 3 {
                is_recording = false;
                recording_ended = false;
            }
            if !is_recording && !transferring_prev_frame && current_segment_num == 3 {
                rosc =
                    go_dormant_until_next_vsync(rosc, lepton, clocks.system_clock.freq(), got_sync);
            } else if current_segment_num == 3 {
                //warn!("Overrunning frame time");
            }
        }
    }
}
