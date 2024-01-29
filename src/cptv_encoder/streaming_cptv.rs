use crate::byte_slice_cursor::CursorMut;
use crate::cptv_encoder::bit_cursor::BitCursor;
use crate::cptv_encoder::huffman::HuffmanEntry;
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
use crate::device_config::DeviceConfig;
use crate::lepton::Telemetry;
use crate::onboard_flash::OnboardFlash;
use crate::utils::{u16_slice_to_u8, u16_slice_to_u8_mut};
use crate::FIRMWARE_VERSION;
use byteorder::{ByteOrder, LittleEndian};
use core::fmt::Write;
use core::mem;
use core::ops::{Index, IndexMut};
use defmt::{info, Format};

#[repr(u8)]
#[derive(PartialEq, Debug, Copy, Clone, Format)]
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

fn delta_encode_frame_data(prev_frame: &mut [u16], curr: &[u16]) -> (u8, u16, u16) {
    let mut output = [0i32; FRAME_WIDTH * FRAME_HEIGHT];
    // We need to work out after the delta encoding what the range is, and how many bits we can pack
    // this into.

    // Here we are doing intra-frame delta encoding between this frame and the previous frame if
    // present.
    let snake_iter = (0..FRAME_WIDTH)
        .chain((0..FRAME_WIDTH).rev())
        .cycle()
        .take(FRAME_WIDTH * FRAME_HEIGHT)
        .enumerate()
        .map(|(index, i)| (index / FRAME_WIDTH) * FRAME_WIDTH + i);
    //.skip(1);
    let mut prev_val = 0;

    // IDEA: Images from lepton often seem to have vertical noise bands that are similar
    // - so rotating the image by 90 degrees before compressing could be a win.

    // NOTE: This decimates the output, taking total compression from 3.x:1 to 5:1
    //let cutoff = CUTOFF; // Is the cutoff fine being symmetrical around zero?
    // TODO: Once we know our warm body cutoff, we can also apply this only to cooler pixels?

    let mut max_value = u16::MIN;
    let mut min_value = u16::MAX;

    // NOTE: We can ignore the first pixel when working out our range, since that is always a literal u32
    // let curr_px = unsafe { curr.get_unchecked(0) };
    // info!("First frame px {}", *curr_px);
    // let prev_raw = unsafe { prev_frame.get_unchecked(0) };
    // let mut val = *curr_px as i32 - *prev_raw as i32;
    // let delta = val - prev_val;
    // info!("First frame delta {}", delta);
    // *unsafe { output.get_unchecked_mut(0) } = delta;
    // prev_val = val;

    // TODO: Try having a separate buffer for prev_frame?
    let max = i16::MAX as i32;
    let min = i16::MIN as i32;
    let mut i = 0;
    for input_index in snake_iter {
        let curr_px = unsafe { curr.get_unchecked(input_index) };
        max_value = max_value.max(*curr_px);
        min_value = min_value.min(*curr_px);
        let prev_raw = unsafe { prev_frame.get_unchecked(input_index) };
        let val = *curr_px as i32 - *prev_raw as i32;
        let delta = val - prev_val;
        assert!(
            delta >= min,
            "delta {}, min {}, val {}, prev_val {}, curr_px {}, prev_px {}",
            delta,
            min,
            val,
            prev_val,
            curr_px,
            prev_raw
        );
        assert!(
            delta <= max,
            "delta {}, max {}, val {}, prev_val {}, curr_px {}, prev_px {}",
            delta,
            max,
            val,
            prev_val,
            curr_px,
            prev_raw
        );
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
        let prev_as_u8 = unsafe { u16_slice_to_u8_mut(prev_frame) };
        LittleEndian::write_u32(&mut prev_as_u8[0..4], px as u32);
    }
    if bits_per_pixel == 16 {
        for i in 1..output.len() {
            let px = unsafe { *output.get_unchecked(i) } as u16;
            prev_frame[i + 1] = px;
        }
    } else {
        let prev_as_u8 = unsafe { u16_slice_to_u8_mut(prev_frame) };
        for i in 1..output.len() {
            let px = unsafe { *output.get_unchecked(i) } as u8;
            prev_as_u8[i + 3] = px;
        }
    }

    (bits_per_pixel, min_value, max_value)
}

struct FieldIterator {
    state: [u8; 30], // TODO: What is the actual high-water mark for field sizes?
    size: u8,
    code: u8,
    cursor: u16,
}

impl Iterator for FieldIterator {
    type Item = u8;

    #[inline(always)]
    fn next(&mut self) -> Option<Self::Item> {
        if self.size == 0 {
            None
        } else {
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
    //assert!(size <= 30);
    let mut iter_state = FieldIterator {
        state: [0u8; 30],
        size,
        code: code as u8,
        cursor: 0,
    };

    iter_state.state[0..size as usize].copy_from_slice(unsafe {
        core::slice::from_raw_parts(value as *const T as *const u8, size as usize)
    });
    iter_state
}

/// This is a pre-computed dynamic LZ77 (deflate) header.
/// It starts with the block type b10 (dynamic) and is followed by the pre-computed huffman table
/// code length descriptions see: (Deflate spec)[https://www.ietf.org/rfc/rfc1951.txt]
/// For our purposes of writing out CPTV files, we only use a 1:1 byte mapping of literal values to
/// huffman codes, we don't attempt to do matching of longer patterns in the data stream,
/// since this would a) take more memory, and b) have a less constant performance profile.
///
/// Important: the last item of this array is only a 5 bit value.
const STATIC_LZ77_DYNAMIC_BLOCK_HEADER: [u8; 46] = [
    5, 224, 3, 152, 109, 105, 182, 6, 106, 190, 255, 24, 107, 70, 196, 222, 153, 89, 85, 231, 182,
    109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 119, 223, 83, 200, 220, 59,
    34, 230, 26, 227, 235, 7,
];

pub struct CptvStream<'a> {
    cptv_header: Cptv2Header,
    pub cursor: BitCursor,
    crc_table: &'a [u32; 256],
    huffman_table: &'a [HuffmanEntry; 257],
    crc_val: u32,
    total_uncompressed: u32,
    pub starting_block_index: u16,
    pub num_frames: u32,
}

pub fn make_crc_table() -> [u32; 256] {
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

impl<'a> CptvStream<'a> {
    pub fn new(
        current_time: u64,
        lepton_version: u8,
        lepton_serial: Option<u32>,
        lepton_firmware_version: Option<((u8, u8, u8), (u8, u8, u8))>,
        device_config: &DeviceConfig,
        flash_storage: &mut OnboardFlash,
        huffman_table: &'a [HuffmanEntry; 257],
        crc_table: &'a [u32; 256],
    ) -> CptvStream<'a> {
        let starting_block_index = flash_storage.start_file();

        // Camera serial, camera firmware, location_altitude, location_timestamp, location_accuracy
        let cptv_header = Cptv2Header::new(
            current_time,
            lepton_version,
            lepton_serial,
            lepton_firmware_version,
            device_config,
        );
        CptvStream {
            cursor: BitCursor::new(),
            crc_table,
            huffman_table,
            crc_val: 0,
            total_uncompressed: 0,
            starting_block_index: starting_block_index as u16,
            cptv_header,
            num_frames: 0,
        }
    }

    /// Writes out the standard gzip header, which says that the following block is the *last* block in
    /// the gzip 'member' - which is a self-contained valid gzip stream.  Then write out the beginning
    /// of a 'dynamic' gzip block, with pre-calculated huffman tables describing only literals
    /// (not match lengths or distances).  This will always be smaller than our page size of 2048 bytes
    /// so there's no need to check if we need to flush out to flash memory.
    pub fn init_gzip_stream(&mut self, flash_storage: &mut OnboardFlash, at_header_location: bool) {
        self.crc_val = 0;
        self.total_uncompressed = 0;

        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index as isize), Some(0isize))
        } else {
            (None, None)
        };

        let gzip_header: [u8; 10] = [0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff];
        for byte in gzip_header
            .iter()
            .chain(&STATIC_LZ77_DYNAMIC_BLOCK_HEADER[..STATIC_LZ77_DYNAMIC_BLOCK_HEADER.len() - 1])
        {
            self.cursor.write_byte(*byte);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
                self.cursor.flush_residual_bits();
            }
        }
        self.cursor.write_bits(
            STATIC_LZ77_DYNAMIC_BLOCK_HEADER[STATIC_LZ77_DYNAMIC_BLOCK_HEADER.len() - 1] as u32,
            5,
        );
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
            self.cursor.flush_residual_bits();
        }
    }

    /// Add a new frame to the current CPTV stream, flushing out pages to the flash storage as needed.
    pub fn push_frame(
        &mut self,
        current_frame: &[u16],
        prev_frame: &mut [u16],
        frame_telemetry: &Telemetry,
        flash_storage: &mut OnboardFlash,
    ) {
        let (bit_width, min_value, max_value) = delta_encode_frame_data(prev_frame, current_frame);
        let frame_size = 4 + ((FRAME_HEIGHT * FRAME_WIDTH) - 1) as u32 * (bit_width as u32 / 8);
        let frame_header = CptvFrameHeader {
            time_on: frame_telemetry.msec_on,
            last_ffc_time: frame_telemetry.time_at_last_ffc,
            bit_width,
            frame_size,
            last_ffc_temp_c: frame_telemetry.fpa_temp_c_at_last_ffc,
            frame_temp_c: frame_telemetry.fpa_temp_c,
        };
        let frame_header_iter = frame_header_iter(&frame_header);
        let delta_encoded = unsafe { &u16_slice_to_u8(&prev_frame)[0..frame_size as usize] };
        self.cptv_header.min_value = self.cptv_header.min_value.min(min_value);
        self.cptv_header.max_value = self.cptv_header.max_value.max(max_value);
        self.cptv_header.total_frame_count += 1;

        if self.cptv_header.total_frame_count % 10 == 0 {
            info!(
                "Write frame #{}, {}",
                self.cptv_header.total_frame_count, frame_telemetry.frame_num
            );
        }
        for byte in frame_header_iter.chain(delta_encoded.iter().map(|&x| x)) {
            self.total_uncompressed += 1;
            self.crc_val = self.crc_val ^ 0xffffffff;
            self.crc_val = self.crc_table[((self.crc_val ^ byte as u32) & 0xff) as usize]
                ^ (self.crc_val >> 8);
            self.crc_val = self.crc_val ^ 0xffffffff;
            let entry = &self.huffman_table[byte as usize];
            self.cursor.write_bits(entry.code as u32, entry.bits as u32);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(to_flush, num_bytes, false, None, None);
                self.cursor.flush_residual_bits();
            }
        }
        self.num_frames += 1;
    }

    fn write_gzip_trailer(&mut self, flash_storage: &mut OnboardFlash, at_header_location: bool) {
        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index as isize), Some(0isize))
        } else {
            (None, None)
        };

        // End the actual data stream by writing out the end of stream literal.
        let entry = &self.huffman_table[256];
        self.cursor.write_bits(entry.code as u32, entry.bits as u32);
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
            self.cursor.flush_residual_bits();
        }

        // Align to the nearest full byte after finishing the stream.
        let needs_flush = self.cursor.end_aligned();
        if needs_flush {
            let (to_flush, num_bytes) = self.cursor.flush();
            flash_storage.append_file_bytes(to_flush, num_bytes, false, block_index, page_index);
            self.cursor.flush_residual_bits();
        }

        // Write the CRC value for the uncompressed input date of the gzip stream.
        let mut buf = [0u8; 4];
        LittleEndian::write_u32(&mut buf, self.crc_val);
        for b in buf {
            self.cursor.write_byte(b);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
                self.cursor.flush_residual_bits();
            }
        }
        // Write the total uncompressed length of input data
        LittleEndian::write_u32(&mut buf, self.total_uncompressed);
        for b in buf {
            self.cursor.write_byte(b);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(
                    to_flush,
                    num_bytes,
                    false,
                    block_index,
                    page_index,
                );
                self.cursor.flush_residual_bits();
            }
        }
        // Flush out any remaining bits in the accumulator, align them to the nearest full byte,
        // and write out to storage.
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

    /// End the current CPTV stream, and write out the CPTV header as a separate gzip member which
    /// will be prepended to the main data stream when transferred out to the raspberry pi.  These
    /// two gzip 'members' or self-contained gip streams when concatenated are themselves a valid
    /// gzip stream, and therefore make up a valid CPTV file.
    pub fn finalise(&mut self, flash_storage: &mut OnboardFlash) {
        // Write out the gzip trailer for the main data - the CPTV frames.
        self.write_gzip_trailer(flash_storage, false);

        // Now write the CPTV header into the first page of the starting block as a separate gzip member,
        // now that we know the total frame count for the recording, and the min/max pixel values.
        self.init_gzip_stream(flash_storage, true);
        for byte in push_header_iterator(&self.cptv_header) {
            self.total_uncompressed += 1;
            self.crc_val = self.crc_val ^ 0xffffffff;
            self.crc_val = self.crc_table[((self.crc_val ^ byte as u32) & 0xff) as usize]
                ^ (self.crc_val >> 8);
            self.crc_val = self.crc_val ^ 0xffffffff;
            let entry = &self.huffman_table[byte as usize];
            self.cursor.write_bits(entry.code as u32, entry.bits as u32);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                flash_storage.append_file_bytes(to_flush, num_bytes, false, None, None);
                self.cursor.flush_residual_bits();
            }
        }
        self.write_gzip_trailer(flash_storage, true);
    }

    pub fn discard(
        &mut self,
        flash_storage: &mut OnboardFlash,
        start_block_index: isize,
        end_block_index: isize,
    ) {
        // NOTE: In the case that the block erase fails, that just means we won't reclaim the space at the moment.
        let _ = flash_storage.erase_block_range(start_block_index, end_block_index);
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

pub fn frame_header_iter<'a>(frame_header: &'a CptvFrameHeader) -> impl Iterator<Item = u8> + 'a {
    // Write the frame header
    // We know the number of fields up front:
    [b'F', 6]
        .iter()
        .map(|x| *x)
        .chain(push_field_iterator(
            &frame_header.frame_size,
            FieldType::FrameSize,
        ))
        .chain(push_field_iterator(
            &frame_header.bit_width,
            FieldType::BitsPerPixel,
        ))
        .chain(push_field_iterator(
            &frame_header.time_on,
            FieldType::TimeOn,
        ))
        .chain(push_field_iterator(
            &frame_header.last_ffc_time,
            FieldType::LastFfcTime,
        ))
        .chain(push_field_iterator(
            &frame_header.last_ffc_temp_c,
            FieldType::LastFfcTempC,
        ))
        .chain(push_field_iterator(
            &frame_header.frame_temp_c,
            FieldType::FrameTempC,
        ))
}

fn push_optional_field_iterator<T: Sized>(value: &Option<T>, code: FieldType) -> FieldIterator {
    if let Some(value) = value {
        push_field_iterator(value, code)
    } else {
        FieldIterator {
            state: [0u8; 30],
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
    let num_header_fields = 12u8;
    let num_optional_header_fields = [
        header.serial_number.is_some(),
        header.firmware_version.is_some(),
        header.latitude.is_some(),
        header.longitude.is_some(),
        header.loc_timestamp.is_some(),
        header.altitude.is_some(),
        header.accuracy.is_some(),
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
        .chain(push_field_iterator(&1u8, FieldType::Compression))
        .chain(push_field_iterator(&9u8, FieldType::FrameRate))
        .chain(push_field_iterator(
            &header.device_name,
            FieldType::DeviceName,
        ))
        .chain(push_field_iterator(
            &[b'f', b'l', b'i', b'r'],
            FieldType::Brand,
        ))
        .chain(push_field_iterator(&header.model, FieldType::Model))
        .chain(push_field_iterator(&header.device_id, FieldType::DeviceID))
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
    pub device_name: [u8; 63],
    pub model: [u8; 30],
    pub device_id: u32,
    pub serial_number: Option<u32>,
    pub firmware_version: Option<[u8; 30]>,
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
    pub fn new(
        timestamp: u64,
        lepton_version: u8,
        lepton_serial: Option<u32>,
        lepton_firmware_version: Option<((u8, u8, u8), (u8, u8, u8))>,
        device_config: &DeviceConfig,
    ) -> Cptv2Header {
        // NOTE: Set default values for things not included in
        // older CPTVv1 files, which can otherwise be decoded as
        // v2.
        let mut firmware = [0u8; 30];
        let mut cursor = CursorMut::new(&mut firmware);
        if let Some(((m_major, m_minor, m_build), (d_major, d_minor, d_build))) =
            lepton_firmware_version
        {
            let _ = write!(
                &mut cursor,
                "{}.{}.{}/{}.{}.{}/{}",
                m_major, m_minor, m_build, d_major, d_minor, d_build, FIRMWARE_VERSION
            );
        } else {
            let _ = write!(&mut cursor, "{}", FIRMWARE_VERSION);
        }

        let (lat, lng) = device_config.config().location;
        let mut header = Cptv2Header {
            timestamp,
            device_name: [0; 63],
            model: [0; 30],
            device_id: device_config.config().device_id,
            serial_number: lepton_serial,
            firmware_version: Some(firmware),
            latitude: Some(lat),
            longitude: Some(lng),
            loc_timestamp: device_config.config().location_timestamp,
            altitude: device_config.config().location_altitude,
            accuracy: device_config.config().location_accuracy,
            total_frame_count: 0,
            min_value: u16::MAX,
            max_value: u16::MIN,
        };
        // NOTE: Device names longer than 30 chars are truncated
        header.device_name[0..device_config.device_name_bytes().len()].copy_from_slice(
            &device_config.device_name_bytes()[0..device_config.device_name_bytes().len()],
        );
        let model = if lepton_version == 35 {
            &b"lepton3.5"[..]
        } else {
            &b"lepton3"[..]
        };
        header.model[0..model.len()].copy_from_slice(model);

        header
    }
}
