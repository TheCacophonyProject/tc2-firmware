use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use core::mem;

#[repr(u8)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum FieldType {
    // K remaining
    Header = b'H',
    Timestamp = b'T',
    Width = b'X',
    Height = b'Y',
    Compression = b'C',
    DeviceName = b'D',
    MotionConfig = b'M',
    PreviewSecs = b'P',
    Latitude = b'L',
    Longitude = b'O',
    LocTimestamp = b'S',
    Altitude = b'A',
    Accuracy = b'U',
    Model = b'E',
    Brand = b'B',
    DeviceID = b'I',
    FirmwareVersion = b'V',
    CameraSerial = b'N',
    FrameRate = b'Z',
    BackgroundFrame = b'g',

    // TODO: Other header fields I've added to V2
    MinValue = b'Q',
    MaxValue = b'K',
    NumFrames = b'J',
    FramesPerIframe = b'G',
    FrameHeader = b'F',

    BitsPerPixel = b'w',
    FrameSize = b'f',
    LastFfcTime = b'c',
    FrameTempC = b'a',
    LastFfcTempC = b'b',
    TimeOn = b't',
    Unknown = b';',
}

impl From<char> for FieldType {
    fn from(val: char) -> Self {
        use FieldType::*;
        match val {
            'H' => Header,
            'T' => Timestamp,
            'X' => Width,
            'Y' => Height,
            'C' => Compression,
            'D' => DeviceName,
            'E' => Model,
            'B' => Brand,
            'I' => DeviceID,
            'M' => MotionConfig,
            'P' => PreviewSecs,
            'L' => Latitude,
            'O' => Longitude,
            'S' => LocTimestamp,
            'A' => Altitude,
            'U' => Accuracy,
            'Q' => MinValue,
            'K' => MaxValue,
            'N' => CameraSerial,
            'V' => FirmwareVersion,
            'J' => NumFrames,
            'Z' => FrameRate,
            'G' => FramesPerIframe,
            'F' => FrameHeader,
            'g' => BackgroundFrame,
            'w' => BitsPerPixel,
            'f' => FrameSize,
            'c' => LastFfcTime,
            't' => TimeOn,
            'a' => FrameTempC,
            'b' => LastFfcTempC,
            _ => Unknown,
        }
    }
}

#[derive(Clone)]
pub struct FrameData {
    data: [u16; 160 * 120],
    min: u16,
    max: u16,
}

#[derive(Clone)]
pub struct CptvFrame {
    pub time_on: u32,

    pub bit_width: u8,
    pub frame_size: u32,

    pub last_ffc_time: u32,
    pub last_ffc_temp_c: f32,
    pub frame_temp_c: f32,

    // Raw image data?
    pub image_data: FrameData,
}

impl CptvFrame {
    pub fn new_with_dimensions(width: usize, height: usize) -> CptvFrame {
        CptvFrame {
            time_on: 0,
            bit_width: 0,
            frame_size: 0,
            last_ffc_time: 0,
            last_ffc_temp_c: 0.0,
            frame_temp_c: 0.0,
            image_data: FrameData::new(),
        }
    }
}

impl FrameData {
    pub fn new() -> FrameData {
        FrameData {
            data: [0; FRAME_WIDTH * FRAME_HEIGHT],
            min: u16::MAX,
            max: u16::MIN,
        }
    }

    pub fn view(&self, x: usize, y: usize, w: usize, h: usize) -> impl Iterator<Item = &u16> {
        let start_row = y * self.width;
        let end_row = (y + h) * self.width;
        self.data[start_row..end_row]
            .chunks_exact(self.width)
            .into_iter()
            .flat_map(move |row| row.iter().skip(x).take(w))
    }

    pub fn view_mut(
        &mut self,
        x: usize,
        y: usize,
        w: usize,
        h: usize,
    ) -> impl Iterator<Item = &mut u16> {
        let start_row = y * self.width;
        let end_row = (y + h) * self.width;
        self.data[start_row..end_row]
            .chunks_exact_mut(self.width)
            .into_iter()
            .flat_map(move |row| row.iter_mut().skip(x).take(w))
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn data(&self) -> &[u16] {
        &self.data
    }

    pub fn set(&mut self, x: usize, y: usize, val: u16) {
        // Ignore edge pixels for this?
        self.max = u16::max(self.max, val);
        self.min = u16::min(self.min, val);
        self[y][x] = val;
    }

    pub fn as_slice(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                &self.data[0] as *const u16 as *const u8,
                core::mem::size_of::<u16>() * self.data.len(),
            )
        }
    }
}

fn delta_encode_frame(
    prev_frame: &CptvFrame,
    frame: &CptvFrame,
    output: &mut [i32],
    prev_intra: &mut [i32],
    cutoff: i32,
) -> (u8, u16, u16) {
    delta_encode_frame_data(
        prev_frame.image_data.data(),
        frame.image_data.data(),
        output,
        prev_intra,
        cutoff,
    )
}

fn delta_encode_frame_data(
    prev_frame: &[u16],
    curr: &[u16],
    output: &mut [i32],
    prev_intra: &mut [i32],
    cutoff: i32,
) -> (u8, u16, u16) {
    // We need to work out after the delta encoding what the range is, and how many bits we can pack
    // this into.

    // Here we are doing intra-frame delta encoding between this frame and the previous frame if
    // present.

    // FIXME - Rather than collecting here, we should be able to stream this out to the next step
    //  for large speed gains.  Or just use a scratch buffer that gets reused.
    let mut snaking_iterator = (0..FRAME_WIDTH)
        .chain((0..FRAME_WIDTH).rev())
        .cycle()
        .take(FRAME_WIDTH * FRAME_HEIGHT)
        .enumerate()
        .map(|(index, i)| (index, ((index / FRAME_WIDTH) * FRAME_WIDTH) + i));
    let mut max: i32 = 0;
    let mut prev_val = 0;

    // IDEA: Images from lepton often seem to have vertical noise bands that are similar
    // - so rotating the image by 90 degrees before compressing could be a win.

    // NOTE: This decimates the output, taking total compression from 3.x:1 to 5:1
    //let cutoff = CUTOFF; // Is the cutoff fine being symmetrical around zero?
    // TODO: Once we know our warm body cutoff, we can also apply this only to cooler pixels?

    let mut max_value = u16::MIN;
    let mut min_value = u16::MAX;

    for (i, (output_index, input_index)) in snaking_iterator.enumerate() {
        assert!(input_index < curr.len());
        assert!(input_index < prev_frame.len());
        assert!(input_index < prev_intra.len());
        assert!(output_index < output.len());
        let curr_px = curr[input_index];
        let mut prev_px = prev_intra[input_index];
        let prev_raw = prev_frame[input_index];
        let mut val = curr_px as i32 - prev_raw as i32;

        //prev_px += val;

        //let mut val2 = curr_px as i32 - prev_raw as i32;
        // Figure out how to not accumulate the error
        let val_tmp = val;

        // 12, 10, 15, 16
        // 10 - 12 = -2
        // write 0, write -2 to actual prev.
        // 15 - 10 = 5
        // 15 - 10 - -2 = 3 = 0 or 15 - 10 = 5
        // 16 - 15 = 1 = 0

        // -2 + 1 + 1 + 2 = 2 // right, so keep an accumulation buffer.  If the buffer gets above
        // the actual diff, then don't use the adjusted value.

        // 12, 10, 11, 12, 14, 16
        // 12
        // 10 - 12 = -2 = 0
        // 11 - 10 = 1 = 0
        // 12 - 11 = 1 = 0
        // 14 - 12 = 2 = 0
        // 16 - 14 = 2 = 0

        // TODO: Maybe only do this if the previous px has a diff of zero.
        if (val + prev_px).abs() < cutoff {
            // && prev_diff == 0r
            val = 0;
            max_value = max_value.max(prev_px as u16);
            min_value = min_value.min(prev_px as u16);
        } else {
            prev_intra[input_index] = val; // TODO: Should this be += val?
            max_value = max_value.max(curr_px);
            min_value = min_value.min(curr_px);
        }
        prev_intra[input_index] += val_tmp; //(curr_px as i32 + val) as u16;
        let delta = val - prev_val;
        output[output_index] = delta;
        if i != 0 {
            // NOTE: We can ignore the first pixel when working out our range, since that is always a literal u32
            max = delta.abs().max(max);
        }
        prev_val = val;
    }
    // Now we pack into either 8 or 16 bits, depending on the range present in the frame
    //println!("Hist {:?}", hist);
    // NOTE: If we go from 65535 to 0 in one step, that's a delta of -65535 which doesn't fit into 16 bits.
    //  Can this happen ever with real input?  How should we guard against it?
    //  Are there more realistic scenarios which don't work?  Let's get a bunch of lepton 3.5 files
    //  and work out the ranges there.\

    // NOTE: To play nice with lz77, we only want to pack to bytes
    let mut bits_per_pixel = (((mem::size_of::<i32>() as u32 * 8) - max.leading_zeros()) as u8 + 1); // Allow for sign bit
    if bits_per_pixel >= 8 {
        bits_per_pixel = 16
    } else {
        bits_per_pixel = 8
    };
    (bits_per_pixel, min_value, max_value)
}

struct FieldIterator {
    state: [u8; 256], // TODO: What is the actual high-water mark for field sizes?
    size: u8,
    code: u8,
    cursor: u16,
}

impl Iterator for FieldIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        self.cursor += 1;
        if self.cursor == 1 {
            Some(self.size)
        } else if self.cursor == 2 {
            Some(self.code)
        } else if self.cursor <= (self.size as u16) + 2 {
            Some(self.state[self.cursor as usize - 3])
        } else {
            None
        }
    }
}

struct FrameHeaderIterator {
    state: [u8; 2],
    cursor: u8,
}

impl Iterator for FrameHeaderIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor == 0 {
            self.cursor += 1;
            Some(self.state[0])
        } else if self.cursor == 1 {
            self.cursor += 1;
            Some(self.state[1])
        } else {
            None
        }
    }
}

fn push_field_iterator<T: Sized>(value: &T, code: FieldType) -> FieldIterator {
    let size = mem::size_of_val(value);
    let size = mem::size_of_val(value);
    let size = if size == 257 {
        // This is a string
        unsafe { core::slice::from_raw_parts(value as *const T as *const u8, size)[size - 1] }
    } else {
        size as u8
    };
    assert!(size <= 255);
    let mut iter_state = FieldIterator {
        state: [0u8; 256],
        size: size as u8,
        code: code as u8,
        cursor: 0,
    };

    iter_state.state[0..size as usize].copy_from_slice(unsafe {
        core::slice::from_raw_parts(value as *const T as *const u8, size as usize)
    });
    iter_state
}

struct PackBitsIterator<'a> {
    input: &'a [i32],
    width: u8,
    cursor: usize,
    byte: u8,
    frame_num: i32,
}

impl<'a> Iterator for PackBitsIterator<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor == self.input.len() {
            None
        } else {
            let px = self.input[self.cursor];
            if self.cursor == 0 {
                self.byte += 1;
                // Initial pixel written out as a u32

                match self.byte {
                    1 => Some((((px as u32) & 0x000000ff) >> 0) as u8),
                    2 => Some((((px as u32) & 0x0000ff00) >> 8) as u8),
                    3 => Some((((px as u32) & 0x00ff0000) >> 16) as u8),
                    4 => {
                        self.byte = 0;
                        self.cursor += 1;
                        Some((((px as u32) & 0xff000000) >> 24) as u8)
                    }
                    _ => unreachable!(),
                }
            } else {
                if self.width == 8 {
                    self.cursor += 1;
                    Some(px as u8)
                } else if self.width == 16 {
                    if self.byte == 1 {
                        self.cursor += 1;
                        self.byte = 0;
                        Some(px as u8)
                    } else {
                        self.byte += 1;
                        Some((px >> 8) as u8)
                    }
                } else {
                    unreachable!()
                }
            }
        }
    }
}

fn pack_bits_fast_iterator(input: &[i32], width: u8, frame_num: i32) -> PackBitsIterator {
    PackBitsIterator {
        input,
        width,
        cursor: 0,
        byte: 0,
        frame_num,
    }
}
fn pack_frame_iterator<'a>(
    frame: &'a CptvFrame,
    delta_encoded_frame: &'a [i32], // Could this be an iterator too, and we just evaluate as needed?
    bits_per_pixel: u8,
    frame_num: i32,
) -> impl Iterator<Item = u8> + 'a {
    // Write the frame header
    // We know the number of fields up front:
    let frame_size = 4 + (delta_encoded_frame[1..].len() * (bits_per_pixel as usize / 8));
    FrameHeaderIterator {
        state: [b'F', 6],
        cursor: 0,
    }
    .chain(push_field_iterator(&frame_size, FieldType::FrameSize))
    .chain(push_field_iterator(
        &bits_per_pixel,
        FieldType::BitsPerPixel,
    ))
    .chain(push_field_iterator(&frame.time_on, FieldType::TimeOn))
    .chain(push_field_iterator(
        &frame.last_ffc_time,
        FieldType::LastFfcTime,
    ))
    .chain(push_field_iterator(
        &frame.last_ffc_temp_c,
        FieldType::LastFfcTempC,
    ))
    .chain(push_field_iterator(
        &frame.frame_temp_c,
        FieldType::FrameTempC,
    ))
    .chain(pack_bits_fast_iterator(
        &delta_encoded_frame,
        bits_per_pixel,
        frame_num,
    ))
}
