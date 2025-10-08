use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::frame_processing::FrameBuffer;
use crate::re_exports::log::info;
use byteorder::{ByteOrder, LittleEndian};
use chrono::Duration;
use codec::decode::CptvDecoder;
extern crate std;
use crate::formatted_time::FormattedNZTime;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use std::collections::VecDeque;
use std::fs::File;
use std::path::Path;

pub struct SioFifo {
    inner: VecDeque<u32>,
    selected_buffer: u8,
}
impl SioFifo {
    pub fn read_blocking(&mut self) -> u32 {
        let peek_next = self.inner.front();
        match peek_next {
            None => self.next_frame(),
            Some(_peek_next) => {}
        }
        self.inner.pop_front().unwrap_or(0)
    }

    pub fn read(&self) -> Option<u32> {
        None
    }

    pub fn next_frame(&mut self) {
        advance_one_frame();
        // TODO, if the current timestamp is a recording time, advance cptv frame, otherwise use static frame.

        self.inner.push_back(0xae);
        if self.selected_buffer == 1 {
            self.selected_buffer = 0;
        } else {
            self.selected_buffer = 1;
        }
        self.inner.push_back(self.selected_buffer as u32); // frame buffer 1
    }

    pub fn write_blocking(&mut self, value: u32) {
        match value {
            0xec => {
                // Core0Task::ReadyToReceiveLeptonConfig
                self.inner.push_back(0x0c);
                self.inner.push_back(4);
                self.inner.push_back(2);
                self.inner.push_back(1234567);
                let main_lepton_firmware = LittleEndian::read_u32(&[1, 2, 3, 0]);
                let dsp_lepton_firmware = LittleEndian::read_u32(&[4, 5, 6, 0]);
                self.inner.push_back(main_lepton_firmware);
                self.inner.push_back(dsp_lepton_firmware);
            }
            0xdb => {
                // Tell other imaginary core we're ready

                //ReceiveFrame
                // self.inner.push_back(0xae);
                // self.inner.push_back(1); // frame buffer 1
            }
            _ => panic!("Unknown value to write to fifo:"),
        }
    }
    pub fn write(&mut self, value: u32) {
        if value == 0xef {
            // Respond with Lepton ready to sleep
            self.inner.push_back(0xbf);
        }
    }
}

#[allow(non_camel_case_types)]
pub struct GPIO_BANK0;
pub struct Sio {
    pub gpio_bank0: GPIO_BANK0,
    pub fifo: SioFifo,
}

impl Sio {
    pub fn new(_sio: crate::re_exports::bsp::pac::SIO) -> Sio {
        Sio {
            gpio_bank0: GPIO_BANK0,
            fifo: SioFifo {
                inner: VecDeque::new(),
                selected_buffer: 0,
            },
        }
    }
}

pub fn advance_one_frame() {
    // Advance time 1 frame (or maybe that already happens for us)
    // if the current time is a time to trigger recording, advance cptv frame, otherwise
    // keep the same frame.
    TEST_SIM_STATE.with(|s| {
        let mut s = s.borrow_mut();
        s.current_time += Duration::milliseconds(115);
        if s.cptv_decoder.is_none() {
            let next_file_path = if s.current_cptv_file.is_none() {
                s.cptv_files.as_ref().unwrap().first().unwrap().clone()
            } else {
                let cur = s.current_cptv_file.as_ref().unwrap();
                let paths = &s.cptv_files;
                let p = if let Some(paths) = paths.as_ref() {
                    let current_index =
                        paths.iter().enumerate().find(|(_, x)| x == &cur).unwrap().0;
                    let p = if current_index + 1 < paths.len() {
                        paths[current_index + 1].clone()
                    } else {
                        paths[0].clone()
                    };
                    info!("Setting next CPTV file path: {}", cur);
                    p
                } else {
                    unreachable!("Should have path");
                };
                p
            };
            s.cptv_decoder
                .replace(CptvDecoder::<File>::from_path(&Path::new(&next_file_path)).unwrap());
            s.current_cptv_file = Some(next_file_path);
        }
        if s.last_frame.is_none() {
            let mut frame = s.cptv_decoder.as_mut().unwrap().next_frame();
            if frame.as_ref().is_ok_and(|frame| frame.is_background_frame) {
                frame = s.cptv_decoder.as_mut().unwrap().next_frame();
            }
            if let Ok(frame) = frame {
                s.last_frame = Some(frame.clone());
            }
        } else {
            // Advance frame if we're in a trigger period, and inside the recording window?
            let mut start_time = s.current_time;
            let minutes_into_thermal_window =
                if let Some((window_start_time, _end)) = s.current_thermal_window {
                    start_time = window_start_time;
                    s.current_time
                        .signed_duration_since(window_start_time)
                        .num_minutes()
                } else {
                    1_000_000
                };
            if minutes_into_thermal_window > 0 {
                let current_time = s.current_time;
                let mut triggers = s.thermal_trigger_offsets_mins.clone();
                if let Some(triggers) = &mut triggers {
                    let has_trigger = triggers.contains(&(minutes_into_thermal_window as u32));
                    if has_trigger {
                        if let Ok(frame) = s.cptv_decoder.as_mut().unwrap().next_frame() {
                            // info!(
                            //     "Trigger frame {}, minutes_info_window {}, now {}",
                            //     frame.time_on,
                            //     minutes_into_thermal_window,
                            //     FormattedNZTime(current_time)
                            // );
                            s.last_frame = Some(frame.clone());
                        } else {
                            // Clip ended, load up a new decoder with the next file,
                            // and pause on the last frame of this file so that motion detection stalls out.

                            // Remove the trigger.
                            info!(
                                "Trigger {} reached end, loading new file",
                                minutes_into_thermal_window
                            );
                            info!(
                                "Current time: {:?}, window start time {:?}",
                                current_time, start_time
                            );
                            triggers.retain(|x| *x != minutes_into_thermal_window as u32);
                            s.cptv_decoder = None;
                            s.last_frame = None;
                        }
                    }
                }
                s.thermal_trigger_offsets_mins = triggers;
            }
        }
    });
}

pub fn copy_last_frame(frame_buffer: &mut Option<&'static mut FrameBuffer>) {
    if let Some(frame_buffer) = frame_buffer {
        if TEST_SIM_STATE.with(|s| s.borrow().last_frame.is_none()) {
            advance_one_frame();
        }
        TEST_SIM_STATE.with(|s| {
            let mut s = s.borrow_mut();
            let mut frame_num = s.frame_num;
            if let Some(frame) = s.last_frame.as_ref() {
                let header = frame_buffer.frame_data_as_u8_slice_mut();
                // Telemetry revision
                LittleEndian::write_u16(&mut header[0..2], 14);
                // Frame number
                LittleEndian::write_u32(&mut header[40..44], frame_num);

                // Msec on:
                let msec_on = frame_num * 115;
                LittleEndian::write_u32(&mut header[2..6], msec_on);

                let adjusted_time_since_last_ffc =
                    msec_on.saturating_sub(frame.time_on.saturating_sub(frame.last_ffc_time));
                // Time at last FFC:
                LittleEndian::write_u32(&mut header[60..64], adjusted_time_since_last_ffc);

                // Always just say FFC is done
                // status_bits
                LittleEndian::write_u32(&mut header[6..10], 3 << 4);
                let fpa_temp_kelvin_x_100 = ((frame.frame_temp_c + 273.15) * 100.0) as u16;
                LittleEndian::write_u16(&mut header[48..=49], fpa_temp_kelvin_x_100);

                // FIXME?  Check that these values come out correctly in the written file.
                let fpa_temp_kelvin_x_100_at_last_ffc =
                    ((frame.last_ffc_temp_c + 273.15) * 100.0) as u16;
                LittleEndian::write_u16(&mut header[58..=59], fpa_temp_kelvin_x_100_at_last_ffc);

                header[640..][0..(FRAME_WIDTH * FRAME_HEIGHT) * 2]
                    .copy_from_slice(frame.image_data.as_slice());
                frame_num += 1;
            }
            s.frame_num = frame_num;
        });
    }
}
