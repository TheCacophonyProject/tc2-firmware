use crate::cptv_encoder::bit_cursor::BitCursor;
use crate::cptv_encoder::huffman::HUFFMAN_TABLE;
use crate::cptv_encoder::streaming_cptv::FieldType::PreviewSecs;
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::onboard_flash::OnboardFlash;
use crate::utils::{u16_slice_to_u8, u16_slice_to_u8_mut};
use byteorder::{ByteOrder, LittleEndian};
use core::mem;
use core::ops::{Index, IndexMut};
use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::bank0::{Gpio4, Gpio5};
use rp2040_hal::gpio::{Pin, PushPullOutput};

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

impl Index<usize> for FrameData {
    type Output = [u16];

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[(index * FRAME_WIDTH)..(index * FRAME_WIDTH) + FRAME_WIDTH]
    }
}

impl IndexMut<usize> for FrameData {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[(index * FRAME_WIDTH)..(index * FRAME_WIDTH) + FRAME_WIDTH]
    }
}

#[derive(Clone)]
pub struct CptvFrameHeader {
    pub time_on: u32,

    pub bit_width: u8,
    pub frame_size: u32,

    pub last_ffc_time: u32,
    pub last_ffc_temp_c: f32,
    pub frame_temp_c: f32,
}

impl CptvFrameHeader {
    pub fn new() -> CptvFrameHeader {
        CptvFrameHeader {
            time_on: 0,
            bit_width: 0,
            frame_size: 0,
            last_ffc_time: 0,
            last_ffc_temp_c: 0.0,
            frame_temp_c: 0.0,
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
        let start_row = y * FRAME_WIDTH;
        let end_row = (y + h) * FRAME_WIDTH;
        self.data[start_row..end_row]
            .chunks_exact(FRAME_WIDTH)
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
        let start_row = y * FRAME_WIDTH;
        let end_row = (y + h) * FRAME_WIDTH;
        self.data[start_row..end_row]
            .chunks_exact_mut(FRAME_WIDTH)
            .into_iter()
            .flat_map(move |row| row.iter_mut().skip(x).take(w))
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
}

fn delta_encode_frame_data(
    prev_frame: &mut [u16],
    curr: &[u16],
    output: &mut [i32],
) -> (u8, u16, u16) {
    // We need to work out after the delta encoding what the range is, and how many bits we can pack
    // this into.

    // Here we are doing intra-frame delta encoding between this frame and the previous frame if
    // present.
    let snake_iter = (0..FRAME_WIDTH)
        .chain((0..FRAME_WIDTH).rev())
        .cycle()
        .take(FRAME_WIDTH * FRAME_HEIGHT)
        .enumerate()
        .map(|(index, i)| (index / FRAME_WIDTH) * FRAME_WIDTH + i)
        .skip(1);
    let mut prev_val = 0;

    // IDEA: Images from lepton often seem to have vertical noise bands that are similar
    // - so rotating the image by 90 degrees before compressing could be a win.

    // NOTE: This decimates the output, taking total compression from 3.x:1 to 5:1
    //let cutoff = CUTOFF; // Is the cutoff fine being symmetrical around zero?
    // TODO: Once we know our warm body cutoff, we can also apply this only to cooler pixels?

    let mut max_value = u16::MIN;
    let mut min_value = u16::MAX;

    // NOTE: We can ignore the first pixel when working out our range, since that is always a literal u32
    let curr_px = unsafe { curr.get_unchecked(0) };
    let prev_raw = unsafe { prev_frame.get_unchecked(0) };
    let mut val = *curr_px as i32 - *prev_raw as i32;
    let delta = val - prev_val;
    *unsafe { output.get_unchecked_mut(0) } = delta;
    prev_val = val;

    let mut i = 1;
    for input_index in snake_iter {
        let curr_px = unsafe { curr.get_unchecked(input_index) };
        let prev_raw = unsafe { prev_frame.get_unchecked(input_index) };
        let mut val = *curr_px as i32 - *prev_raw as i32;
        let delta = val - prev_val;
        *unsafe { output.get_unchecked_mut(i) } = delta;
        i += 1;
        prev_val = val;
    }
    // NOTE: If we go from 65535 to 0 in one step, that's a delta of -65535 which doesn't fit into 16 bits.
    //  Can this happen ever with real input?  How should we guard against it?
    //  Are there more realistic scenarios which don't work?  Let's get a bunch of lepton 3.5 files
    //  and work out the ranges there.\

    // NOTE: To play nice with lz77, we only want to pack to bytes
    let bits_per_pixel = if output[1..].iter().find(|&&x| x < -127 || x > 127).is_some() {
        16
    } else {
        8
    };
    {
        let px = output[0];
        let mut prev_as_u8 = unsafe { u16_slice_to_u8_mut(prev_frame) };
        LittleEndian::write_u32(&mut prev_as_u8[0..4], px as u32);
    }
    if bits_per_pixel >= 8 {
        for i in 1..output.len() {
            let px = unsafe { *output.get_unchecked(i) } as u16;
            prev_frame[i + 1] = px;
        }
    } else {
        let mut prev_as_u8 = unsafe { u16_slice_to_u8_mut(prev_frame) };
        for i in 1..output.len() {
            let px = unsafe { *output.get_unchecked(i) } as u8;
            prev_as_u8[i + 3] = px;
        }
    }

    (bits_per_pixel, min_value, max_value)
}

struct FieldIterator {
    state: [u8; 20], // TODO: What is the actual high-water mark for field sizes?
    size: u8,
    code: u8,
    cursor: u16,
}

impl Iterator for FieldIterator {
    type Item = u8;

    #[inline(always)]
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
    let size = if size > 8 {
        // This is a 0 terminated string
        let slice = unsafe { core::slice::from_raw_parts(value as *const T as *const u8, size) };
        let length = slice.iter().position(|&x| x == 0).unwrap_or(size - 1);
        length as u8
    } else {
        size as u8
    };
    assert!(size <= 20);
    let mut iter_state = FieldIterator {
        state: [0u8; 20],
        size,
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
    bits_per_pixel: u8,
    cursor: usize,
    byte: u8,
}

impl<'a> Iterator for PackBitsIterator<'a> {
    type Item = u8;

    #[inline(always)]
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
                if self.bits_per_pixel == 8 {
                    self.cursor += 1;
                    Some(px as u8)
                } else if self.bits_per_pixel == 16 {
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

fn pack_bits_fast_iterator(input: &[i32], bits_per_pixel: u8) -> PackBitsIterator {
    PackBitsIterator {
        input,
        bits_per_pixel,
        cursor: 0,
        byte: 0,
    }
}
fn pack_frame_iterator<'a>(
    frame: &'a CptvFrameHeader,
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
    // .chain(pack_bits_fast_iterator(
    //     &delta_encoded_frame,
    //     bits_per_pixel,
    // ))
}

const STATIC_HUFFMAN_AND_BLOCK_HEADER: [u8; 46] = [
    5, 224, 3, 152, 109, 105, 182, 6, 106, 190, 255, 24, 107, 70, 196, 222, 153, 89, 85, 231, 182,
    109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 119, 223, 83, 200, 220, 59,
    34, 230, 26, 227, 235, 7,
];
const BITS_UNUSED: u16 = 3;

pub struct CptvStream {
    cptv_header: Cptv2Header,
    pub cursor: BitCursor,
    crc_table: [u32; 256],
    crc_val: u32,
    total_uncompressed: u32,
    starting_block_index: u16,
    prev_frame: [u16; (FRAME_WIDTH * FRAME_HEIGHT) + 1],
}

fn make_crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for n in 0..256 {
        let mut c = n as u32;
        for _ in 0..8 {
            if c & 1 == 1 {
                c = 0xedb88320 ^ (c >> 1);
            } else {
                c = c >> 1;
            }
        }
        table[n] = c;
    }
    table
}
impl CptvStream {
    pub fn new(current_time: u64, flash_storage: &mut OnboardFlash) -> CptvStream {
        let starting_block_index = flash_storage.start_file();
        CptvStream {
            cursor: BitCursor::new(),
            crc_table: make_crc_table(), // TODO - This only needs to be computed at startup
            crc_val: 0,
            total_uncompressed: 0,
            starting_block_index: starting_block_index as u16,
            prev_frame: [0u16; FRAME_WIDTH * FRAME_HEIGHT + 1],
            cptv_header: Cptv2Header::new(current_time),
        }
    }

    pub fn init(&mut self, flash_storage: &mut OnboardFlash, at_header_location: bool) {
        let gzip_header: [u8; 10] = [0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff];
        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index as isize), Some(0isize))
        } else {
            (None, None)
        };

        for byte in gzip_header {
            self.cursor.write_bits_fast(byte as u32, 8);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
            }
        }

        for byte in &STATIC_HUFFMAN_AND_BLOCK_HEADER[..STATIC_HUFFMAN_AND_BLOCK_HEADER.len() - 1] {
            self.cursor.write_bits_fast(*byte as u32, 8);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
            }
        }
        self.cursor.write_bits_fast(
            STATIC_HUFFMAN_AND_BLOCK_HEADER[STATIC_HUFFMAN_AND_BLOCK_HEADER.len() - 1] as u32,
            5,
        );
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
        }
    }

    pub fn push_frame(
        &mut self,
        raw_frame: &[u16],
        flash_storage: &mut OnboardFlash,
        pin: &mut Pin<Gpio5, PushPullOutput>,
        pin2: &mut Pin<Gpio4, PushPullOutput>,
    ) {
        let mut scratch = [0i32; FRAME_WIDTH * FRAME_HEIGHT];
        let frame_header = CptvFrameHeader::new();
        let (min, max, bits_per_pixel, frame_iter) = push_frame_iterator(
            raw_frame,
            &frame_header,
            &mut self.prev_frame,
            &mut scratch,
            self.cptv_header.total_frame_count as i32,
            pin2,
        );
        self.cptv_header.min_value = self.cptv_header.min_value.min(min);
        self.cptv_header.max_value = self.cptv_header.max_value.max(max);
        self.cptv_header.total_frame_count += 1;
        // Delta encode iterator.
        {
            //let frame_bytes = unsafe { u16_slice_to_u8(raw_frame) };
            if self.cptv_header.total_frame_count % 10 == 0 {
                info!("Write frame #{}", self.cptv_header.total_frame_count);
            }
            for byte in frame_iter {
                self.total_uncompressed += 1;
                self.crc_val = self.crc_val ^ 0xffffffff;
                self.crc_val = self.crc_table[((self.crc_val ^ byte as u32) & 0xff) as usize]
                    ^ (self.crc_val >> 8);
                self.crc_val = self.crc_val ^ 0xffffffff;
                let entry = &HUFFMAN_TABLE[byte as usize];
                self.cursor
                    .write_bits_fast(entry.code as u32, entry.bits as u32);
                if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                    pin.set_high().unwrap();
                    flash_storage.append_file_bytes(to_flush, num_bytes, false, None, None);
                    pin.set_low().unwrap();
                }
            }
            let frame_size = 4 + (((FRAME_HEIGHT * FRAME_WIDTH) - 1) * (bits_per_pixel / 8));
            let delta_encoded = if bits_per_pixel == 8 {
                unsafe { &u16_slice_to_u8(&self.prev_frame)[0..frame_size] }
            } else {
                unsafe { &u16_slice_to_u8(&self.prev_frame)[0..frame_size] }
            };
            for byte in delta_encoded {
                self.total_uncompressed += 1;
                self.crc_val = self.crc_val ^ 0xffffffff;
                self.crc_val = self.crc_table[((self.crc_val ^ *byte as u32) & 0xff) as usize]
                    ^ (self.crc_val >> 8);
                self.crc_val = self.crc_val ^ 0xffffffff;
                let entry = &HUFFMAN_TABLE[*byte as usize];
                self.cursor
                    .write_bits_fast(entry.code as u32, entry.bits as u32);
                // TODO: Do we need to somehow do this async to keep times down?
                if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                    pin.set_high().unwrap();
                    flash_storage.append_file_bytes(to_flush, num_bytes, false, None, None);
                    pin.set_low().unwrap();
                }
            }
        }
        self.prev_frame[0..FRAME_WIDTH * FRAME_HEIGHT].copy_from_slice(raw_frame);
    }

    fn write_trailer(&mut self, flash_storage: &mut OnboardFlash, at_header_location: bool) {
        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index as isize), Some(0isize))
        } else {
            (None, None)
        };

        let entry = &HUFFMAN_TABLE[256];
        self.cursor
            .write_bits_fast(entry.code as u32, entry.bits as u32);
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
        }

        let needs_flush = self.cursor.end_aligned();
        if needs_flush {
            let (to_flush, num_bytes) = self.cursor.flush();
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
        }
        let mut buf = [0u8; 4];
        LittleEndian::write_u32(&mut buf, self.crc_val);
        for b in buf {
            self.cursor.write_bits_fast(b as u32, 8);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
            }
        }
        LittleEndian::write_u32(&mut buf, self.total_uncompressed);
        for b in buf {
            self.cursor.write_bits_fast(b as u32, 8);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
            }
        }
        let _ = self.cursor.end_aligned();
        let (to_flush, num_bytes) = self.cursor.flush();

        flash_storage.append_file_bytes(
            to_flush,
            num_bytes,
            !at_header_location,
            block_index,
            page_index,
        );
    }
    pub fn finalise(&mut self, flash_storage: &mut OnboardFlash) {
        self.write_trailer(flash_storage, false);
        // TODO: Now write the header into the first page of the starting block as a separate gzip member

        self.init(flash_storage, true);
        // Now write header
        self.crc_val = 0;
        self.total_uncompressed = 0;
        let mut header_iter = push_header_iterator(&self.cptv_header);
        for byte in header_iter {
            self.total_uncompressed += 1;
            self.crc_val = self.crc_val ^ 0xffffffff;
            self.crc_val = self.crc_table[((self.crc_val ^ byte as u32) & 0xff) as usize]
                ^ (self.crc_val >> 8);
            self.crc_val = self.crc_val ^ 0xffffffff;
            let entry = &HUFFMAN_TABLE[byte as usize];
            self.cursor
                .write_bits_fast(entry.code as u32, entry.bits as u32);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(to_flush, num_bytes, false, None, None);
            }
        }
        self.write_trailer(flash_storage, true);
    }
}

struct HeaderIterator {
    state: [u8; 7],
    cursor: u8,
}

impl HeaderIterator {
    pub fn new(num_fields: u8) -> HeaderIterator {
        HeaderIterator {
            state: [b'C', b'P', b'T', b'V', 2, b'H', num_fields],
            cursor: 0,
        }
    }
}

pub fn push_frame_iterator<'a>(
    frame: &'a [u16],
    frame_header: &'a CptvFrameHeader,
    prev_frame: &'a mut [u16],
    scratch: &'a mut [i32],
    frame_num: i32,
    pin2: &mut Pin<Gpio4, PushPullOutput>,
) -> (u16, u16, usize, impl Iterator<Item = u8> + 'a) {
    pin2.set_high().unwrap();
    let (bits_per_pixel, max_value, min_value) =
        delta_encode_frame_data(prev_frame, frame, scratch);
    pin2.set_low().unwrap();
    (
        min_value,
        max_value,
        bits_per_pixel as usize,
        pack_frame_iterator(&frame_header, scratch, bits_per_pixel, frame_num),
    )
}

fn push_optional_field_iterator<T: Sized>(value: &Option<T>, code: FieldType) -> FieldIterator {
    if let Some(value) = value {
        let size = mem::size_of_val(value);
        let size = if size == 257 {
            // This is a string
            unsafe { core::slice::from_raw_parts(value as *const T as *const u8, size)[size - 1] }
        } else {
            size as u8
        };
        assert!(size <= 20);
        let mut iter_state = FieldIterator {
            state: [0u8; 20],
            size,
            code: code as u8,
            cursor: 0,
        };
        iter_state.state[0..size as usize].copy_from_slice(unsafe {
            core::slice::from_raw_parts(value as *const T as *const u8, size as usize)
        });
        iter_state
    } else {
        FieldIterator {
            state: [0u8; 20],
            size: 0,
            code: 0,
            cursor: 0,
        }
    }
}

impl Iterator for HeaderIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if (self.cursor as usize) < self.state.len() {
            let val = self.state[self.cursor as usize];
            self.cursor += 1;
            Some(val)
        } else {
            None
        }
    }
}
pub fn push_header_iterator(header: &Cptv2Header) -> impl Iterator<Item = u8> {
    let num_header_fields = 9u8;
    let num_optional_header_fields = [
        header.device_id.is_some(),
        header.serial_number.is_some(),
        header.firmware_version.is_some(),
        header.latitude.is_some(),
        header.longitude.is_some(),
        header.loc_timestamp.is_some(),
        header.altitude.is_some(),
        header.accuracy.is_some(),
        header.model.is_some(),
    ]
    .iter()
    .filter(|x| **x)
    .count() as u8;

    HeaderIterator::new(num_header_fields + num_optional_header_fields)
        .chain(push_field_iterator(&header.min_value, FieldType::MinValue))
        .chain(push_field_iterator(&header.max_value, FieldType::MaxValue))
        .chain(push_field_iterator(
            &header.total_frame_count,
            FieldType::NumFrames,
        ))
        .chain(push_field_iterator(&header.timestamp, FieldType::Timestamp))
        .chain(push_field_iterator(&(FRAME_WIDTH as u32), FieldType::Width))
        .chain(push_field_iterator(
            &(FRAME_HEIGHT as u32),
            FieldType::Height,
        ))
        .chain(push_field_iterator(&1, FieldType::Compression))
        .chain(push_field_iterator(&9, FieldType::FrameRate))
        .chain(push_field_iterator(
            &header.device_name,
            FieldType::DeviceName,
        ))
        .chain(push_field_iterator(&"flir", FieldType::Brand))
        .chain(push_optional_field_iterator(
            &header.model.map_or(None, |x| Some(x)),
            FieldType::Model,
        ))
        .chain(push_optional_field_iterator(
            &header.device_id,
            FieldType::DeviceID,
        ))
        .chain(push_optional_field_iterator(
            &header.serial_number,
            FieldType::CameraSerial,
        ))
        .chain(push_optional_field_iterator(
            &header.firmware_version.map_or(None, |x| Some(x)),
            FieldType::FirmwareVersion,
        ))
        .chain(push_optional_field_iterator(
            &header.latitude,
            FieldType::Latitude,
        ))
        .chain(push_optional_field_iterator(
            &header.longitude,
            FieldType::Longitude,
        ))
        .chain(push_optional_field_iterator(
            &header.loc_timestamp,
            FieldType::LocTimestamp,
        ))
        .chain(push_optional_field_iterator(
            &header.altitude,
            FieldType::Altitude,
        ))
        .chain(push_optional_field_iterator(
            &header.accuracy,
            FieldType::Accuracy,
        ))
        .into_iter()
}

// "flir"
// "lepton3.5"
// "<unknown>"
pub struct Cptv2Header {
    pub timestamp: u64,
    pub device_name: [u8; 20],
    pub model: Option<[u8; 20]>,
    pub device_id: Option<u32>,
    pub serial_number: Option<u32>,
    pub firmware_version: Option<[u8; 20]>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub loc_timestamp: Option<u64>,
    pub altitude: Option<f32>,
    pub accuracy: Option<f32>,

    pub total_frame_count: u16,
    pub min_value: u16,
    pub max_value: u16,
}

impl Cptv2Header {
    pub fn new(timestamp: u64) -> Cptv2Header {
        // NOTE: Set default values for things not included in
        // older CPTVv1 files, which can otherwise be decoded as
        // v2.
        let mut header = Cptv2Header {
            timestamp,
            device_name: [0; 20],
            model: None,
            device_id: None,
            serial_number: None,
            firmware_version: None,
            latitude: None,
            longitude: None,
            loc_timestamp: None,
            altitude: None,
            accuracy: None,
            total_frame_count: 0,
            min_value: u16::MAX,
            max_value: u16::MIN,
        };
        let device_name = b"<unknown>";
        header.device_name[0..device_name.len()].copy_from_slice(device_name);

        header
    }
}
