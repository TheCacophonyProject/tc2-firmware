use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use defmt::{Format, info};

const SEG_DIV: usize = 8;
const SEG_WIDTH: usize = FRAME_WIDTH / SEG_DIV;
const SEG_HEIGHT: usize = FRAME_HEIGHT / SEG_DIV;
const ATTENUATION_OFFSET: u16 = 60;
// TODO: These are lepton 3.5 values, probably need to come up with new ones for lepton3
const NIGHTTIME_TRIGGER_THRESHOLD: u16 = 30; //50; //30
const DAYTIME_TRIGGER_THRESHOLD: u16 = 35; //49;
fn trigger_threshold(is_daytime: bool) -> u16 {
    if is_daytime { DAYTIME_TRIGGER_THRESHOLD } else { NIGHTTIME_TRIGGER_THRESHOLD }
}
type HotMap = [(bool, u16); 64];
pub struct MotionTracking {
    pub hot_map: HotMap,
    pub hot_count: u8,      // How many total segments are hot
    pub hot_edge_count: u8, // How many edge segments are hot
    timeout_in_n_frames: Option<u16>,
    triggered_this_frame: bool,
    cumulative_motion: usize,
    max_delta_variance: u16,
}

impl MotionTracking {
    pub fn new() -> MotionTracking {
        MotionTracking {
            hot_edge_count: 0,
            hot_count: 0,
            hot_map: [(false, 0u16); 64],
            timeout_in_n_frames: None,
            triggered_this_frame: false,
            max_delta_variance: 0,
            cumulative_motion: 0,
        }
    }

    fn with_previous(other: &MotionTracking) -> MotionTracking {
        let next_timeout = other
            .timeout_in_n_frames
            .and_then(|timeout| if timeout == 0 { None } else { Some(timeout) });
        MotionTracking {
            hot_edge_count: 0,
            hot_count: 0,
            hot_map: [(false, 0u16); 64],
            timeout_in_n_frames: next_timeout,
            triggered_this_frame: false,
            cumulative_motion: other.cumulative_motion,
            max_delta_variance: other.max_delta_variance,
        }
    }

    fn is_triggering(&self) -> bool {
        self.hot_count != 0
    }

    fn is_triggering_on_apron_only(&self) -> bool {
        self.hot_count != 0 && self.hot_edge_count == self.hot_count
    }

    pub fn triggering_ended(&self) -> bool {
        self.timeout_in_n_frames.is_some_and(|count| count == 0)
    }

    pub fn has_triggered(&self) -> bool {
        self.timeout_in_n_frames.is_some()
    }

    pub fn got_new_trigger(&self) -> bool {
        self.triggered_this_frame
    }

    pub fn hot_bits(&self) -> [u8; 8] {
        let mut bytes = [0u8; 8];
        for (y, item) in bytes.iter_mut().enumerate() {
            for x in 0..SEG_DIV {
                if self.hot_map[(y * SEG_DIV) + x].0 {
                    *item |= 1 << x;
                }
            }
        }
        bytes
    }

    pub fn was_false_positive(&self) -> bool {
        self.max_delta_variance < 60 && self.cumulative_motion < 10
    }
}

// NOTE: If there was an FFC event in the last second, don't try to call this.
//  FFC events will never be called during a recording (though maybe during a pause?)
#[allow(clippy::too_many_lines)]
#[allow(clippy::ref_option)]
pub fn track_motion(
    current_frame: &[u16],
    prev_frame: &[u16],
    prev_frame_stats: &Option<MotionTracking>,
    is_daytime: bool,
    mask: &DetectionMask,
) -> MotionTracking {
    //  The hot map stores whether a segment is currently triggering, and what it's previous
    //  (pre-triggering) max value was.
    let mut motion_tracking = match prev_frame_stats {
        Some(stats) => MotionTracking::with_previous(stats),
        None => MotionTracking::new(),
    };
    let trigger_threshold_val = trigger_threshold(is_daytime);

    for seg_y in 0..SEG_DIV {
        let line = seg_y * SEG_DIV;
        for seg_x in 0..SEG_DIV {
            let segment_index = line + seg_x;
            let x = seg_x * SEG_WIDTH;
            let y = seg_y * SEG_HEIGHT;
            let mut motion_count = 0;
            let mut seg_max = u16::MIN;
            for yy in y..y + SEG_HEIGHT {
                let is_topmost = yy == 0;
                let is_bottommost = yy == 119;
                let y_edge = is_topmost || is_bottommost;
                let mod_y = if y_edge { 30 } else { 0 };
                let row = yy * 160;
                for xx in x..x + SEG_WIDTH {
                    let idx = row + xx;
                    // NOTE: This mask check contributes to half the time spent tracking.
                    if !mask.is_masked_at_index(idx) {
                        let is_leftmost = xx < 2;
                        let is_rightmost = xx == 159;
                        let x_edge = is_leftmost || is_rightmost;
                        let mod_x = if x_edge { 30 } else { 0 };
                        let curr_px = current_frame[idx];
                        let prev_px = prev_frame[idx];
                        let delta = curr_px - prev_px;
                        // TODO: Adjust thresholds for lepton3, since it has different scaling.
                        //  Check dynamic range of lepton3 vs lepton3.5 recordings to understand the mapping.
                        let delta_is_over_threshold = if x_edge || y_edge {
                            seg_max = seg_max.max(curr_px.saturating_sub(ATTENUATION_OFFSET));
                            delta > 50 + mod_x + mod_y
                        } else {
                            seg_max = seg_max.max(curr_px);
                            delta > 50
                        };
                        if delta_is_over_threshold {
                            motion_count += 1;
                        }
                    }
                }
            }

            // Because things are usually smaller/further away towards the top of the frame,
            // we may want to allow triggering on smaller amounts of motion at the top of the
            // frame.  This could be a more graduated falloff.
            #[allow(clippy::if_same_then_else)]
            let motion_threshold = {
                if y < 30 {
                    2
                } else if y < 60 {
                    2
                } else {
                    3
                }
            };

            let seg_val = seg_max;
            if let Some(prev_frame_stats) = &prev_frame_stats {
                let (mut hot, mut pre_hot_val) = prev_frame_stats.hot_map[segment_index];
                // TODO: Possibly the segments should overlap each other slightly?
                let segment_diff = seg_val.saturating_sub(pre_hot_val);

                motion_tracking.max_delta_variance =
                    motion_tracking.max_delta_variance.max(segment_diff);
                let triggers =
                    segment_diff > trigger_threshold_val && motion_count >= motion_threshold;
                if triggers {
                    motion_tracking.cumulative_motion += motion_count;
                }
                if !hot {
                    if triggers {
                        hot = true;
                        // val gets used from previous frame: keeping the value pre-hot,
                        // so that we keep triggering
                    } else {
                        // Update pre-hot val
                        pre_hot_val = seg_val;
                    }
                } else if hot && seg_val < pre_hot_val + 10 && motion_count < 2 {
                    // TODO: Tune values for lepton3
                    // Needs to drop by 10 to un-trigger hot, and have very little motion.
                    hot = false;
                    // Update the max value with the new value.
                    pre_hot_val = seg_val;
                }
                // NOTE: implicit else when hot – Stays hot, and value stays the same, which should be the previous value before it
                // went hot.
                if hot {
                    motion_tracking.hot_count += 1;
                    // We got motion, so reset timeout
                    if motion_tracking.timeout_in_n_frames.is_some() {
                        info!("New motion, reset timeout");
                        motion_tracking.timeout_in_n_frames = None;
                    }
                    let is_edge =
                        seg_x == 0 || seg_x == SEG_DIV - 1 || seg_y == 0 || seg_y == SEG_DIV - 1;

                    if is_edge {
                        motion_tracking.hot_edge_count += 1;
                    }
                }
                motion_tracking.hot_map[segment_index] = (hot, pre_hot_val);
            } else {
                // Seed the initial values
                motion_tracking.hot_map[segment_index] = (false, seg_val);
            }
        }
    }

    if let Some(prev_frame_stats) = &prev_frame_stats {
        // If we're no longer triggering, maybe start decrementing a timeout to stop recording.
        if !motion_tracking.is_triggering() {
            // All the previous hot segments here on edges, so the subject has left the frame.
            if prev_frame_stats.is_triggering_on_apron_only() {
                // TODO: Actually when recording, we'd pause here for a while without
                //  recording more frames, to see if the animal returns - otherwise,
                //  we'd wait for a much longer timeout before ending the recording.
                info!("Subject left frame: set 5 second recording timeout");
                motion_tracking.timeout_in_n_frames = Some(9 * 5); // 5 second timeout
            } else if prev_frame_stats.is_triggering() {
                // Hot disappeared, but not via the edge of the frame, so we wait longer.
                info!("Subject disappeared in frame: set 10 second recording timeout");
                motion_tracking.timeout_in_n_frames = Some(9 * 10); // 10 second timeout
            }
            if let Some(timeout_in_n_frames) = motion_tracking.timeout_in_n_frames.as_mut() {
                *timeout_in_n_frames -= 1;
            }
        } else if !prev_frame_stats.is_triggering() {
            motion_tracking.triggered_this_frame = true;
        }
    }

    // TODO: If we have persistently hot frames, but no motion, we know there's still an
    //  animal there, and we'd pause the recording.
    motion_tracking
}

#[derive(Format, PartialEq)]
pub struct DetectionMask {
    pub inner: [u8; 2400],
    length: usize,
}

impl DetectionMask {
    pub fn set_empty(&mut self) {
        self.length = 0;
    }
    pub fn append_piece(&mut self, piece: &[u8]) {
        self.inner[self.length..self.length + piece.len()].copy_from_slice(piece);
        self.length += piece.len();
    }
    #[allow(clippy::large_types_passed_by_value)]
    pub fn new(mask: Option<[u8; 2400]>) -> DetectionMask {
        let length = if mask.is_some() { 2400 } else { 0 };
        DetectionMask { inner: mask.unwrap_or([0u8; 2400]), length }
    }
    pub fn is_masked_at_pos(&self, x: usize, y: usize) -> bool {
        let index = (y * 160) + x;
        self.inner[index >> 3] & (1 << (index % 8)) != 0
    }

    pub fn set_index(&mut self, index: usize) {
        self.inner[index >> 3] |= 1 << (index % 8);
    }

    pub fn set_pos(&mut self, x: usize, y: usize) {
        let i = (y * 160) + x;
        self.inner[i >> 3] |= 1 << (i % 8);
    }

    #[inline(always)]
    #[allow(clippy::inline_always)]
    pub fn is_masked_at_index(&self, index: usize) -> bool {
        let group = self.inner[index >> 3];
        group != 0 && group & (1 << (index % 8)) != 0
    }
}
