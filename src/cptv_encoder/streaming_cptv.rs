use crate::byte_slice_cursor::CursorMut;
use crate::constants::FIRMWARE_VERSION;
use crate::cptv_encoder::bit_cursor::BitCursor;
use crate::cptv_encoder::huffman::HuffmanEntry;
use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_HEIGHT_U32, FRAME_WIDTH, FRAME_WIDTH_U32};
use crate::device_config::DeviceConfig;
use crate::lepton_telemetry::Telemetry;
use crate::onboard_flash::{
    BlockIndex, FileType, OnboardFlash, RecordingFileType, RecordingFileTypeDetails,
};
use crate::re_exports::log::info;
use crate::synced_date_time::SyncedDateTime;
use byteorder::{ByteOrder, LittleEndian};
use core::fmt::Write;

const NUM_STATUS_RECORDING_FRAMES: u32 = 18;

#[repr(u8)]
#[derive(PartialEq, Debug, Copy, Clone)]
#[cfg_attr(feature = "no-std", derive(defmt::Format))]
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

// This is *either* a zero-terminated string,
// *or* a string taking the full length of the `data` array.
#[derive(Copy, Clone)]
pub struct MaybeZeroTerminatedString<const LENGTH: usize> {
    data: [u8; LENGTH],
}

impl<const LENGTH: usize> MaybeZeroTerminatedString<LENGTH> {
    pub fn new() -> Self {
        Self { data: [0; LENGTH] }
    }

    pub fn new_with_bytes(bytes: &[u8]) -> Self {
        let mut s = Self::new();
        s.set_bytes(bytes);
        s
    }

    pub fn set_bytes(&mut self, bytes: &[u8]) {
        assert!(bytes.len() <= LENGTH);
        self.data[0..bytes.len()].copy_from_slice(bytes);
    }

    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.data
            .iter()
            .position(|&x| x == 0)
            .unwrap_or(self.data.len())
    }

    pub fn as_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

impl From<char> for FieldType {
    #[allow(clippy::enum_glob_use)]
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
pub struct CptvFrameHeader {
    pub time_on: u32,

    pub bit_width: u8,
    pub frame_size: u32,

    pub last_ffc_time: u32,
    pub last_ffc_temp_c: f32,
    pub frame_temp_c: f32,
}

fn delta_encode_frame_data(prev_frame: &mut [u16], curr: &[u16]) -> (u16, u16, i32) {
    // Here we are doing *intra-frame* delta encoding between this frame `curr` and the previous
    // frame if present.  If there was no previous frame, `prev_frame` will be all zeros.

    // We need to work out during the delta encoding what the min and max frame values are
    // for eventual playback normalization purposes.

    // IDEA: Images from lepton often seem to have vertical noise bands that are similar
    // - so rotating the image by 90 degrees before compressing could be a win.

    let mut max_value = u16::MIN;
    let mut min_value = u16::MAX;

    let snake_iter = (0..FRAME_WIDTH)
        .chain((0..FRAME_WIDTH).rev())
        .cycle()
        .take(FRAME_WIDTH * FRAME_HEIGHT)
        .enumerate()
        .map(|(index, i)| (index / FRAME_WIDTH) * FRAME_WIDTH + i);

    // NOTE: We can ignore the first pixel when working out our range, since that is always a literal u32
    let max = i32::from(i16::MAX);
    let min = i32::from(i16::MIN);
    let initial_value = i32::from(curr[0]) - i32::from(prev_frame[0]);
    let mut prev_val = 0;
    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_sign_loss)]
    for offset in snake_iter {
        let curr_px = unsafe { *curr.get_unchecked(offset) };
        let prev_px = unsafe { prev_frame.get_unchecked_mut(offset) };
        max_value = max_value.max(curr_px);
        min_value = min_value.min(curr_px);
        let val = i32::from(curr_px) - i32::from(*prev_px);
        // The actual delta is a delta of deltas.  So first intra-frame, then iter-pixel.
        let delta = val - prev_val;
        debug_assert!(
            delta >= min,
            "delta {delta}, min {min}, val {val}, prev_val {prev_val}, curr_px {curr_px}, prev_px {prev_px}",
        );
        debug_assert!(
            delta <= max,
            "delta {delta}, max {max}, val {val}, prev_val {prev_val}, curr_px {curr_px}, prev_px {prev_px}",
        );
        *prev_px = delta as u16;
        prev_val = val;
    }
    (min_value, max_value, initial_value)
}
struct FieldIterator {
    state: [u8; 30], // TODO: What is the actual high-water mark for field sizes?
    size: u8,
    code: u8,
    cursor: u16,
}

impl Iterator for FieldIterator {
    type Item = u8;

    #[allow(clippy::inline_always)]
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
            } else if self.cursor <= u16::from(self.size) + 2 {
                Some(self.state[self.cursor as usize - 3])
            } else {
                None
            }
        }
    }
}

fn push_field_iterator<T: Sized>(value: &T, code: FieldType) -> FieldIterator {
    let size = size_of_val(value);
    #[allow(clippy::cast_possible_truncation)]
    let size = if size > 8 {
        // This is a zero-terminated string - or if the slice is full, it just has the length of the
        // slice as its termination/length.
        let slice = unsafe {
            core::slice::from_raw_parts(core::ptr::from_ref::<T>(value).cast::<u8>(), size)
        };
        let length = slice.iter().position(|&x| x == 0).unwrap_or(size - 1);
        assert!(u8::try_from(length).is_ok());
        length as u8
    } else {
        size as u8
    };

    // FIXME: Longer device names are truncated, so why allow longer device names elsewhere at all?
    //assert!(size <= 30);
    let mut iter_state = FieldIterator {
        state: [0u8; 30],
        size,
        code: code as u8,
        cursor: 0,
    };

    iter_state.state[0..size as usize].copy_from_slice(unsafe {
        core::slice::from_raw_parts(core::ptr::from_ref::<T>(value).cast::<u8>(), size as usize)
    });
    iter_state
}

/// This is a pre-computed dynamic LZ77 (deflate) header.
/// It starts with the block type b10 (dynamic) and is followed by the pre-computed huffman table
/// code length descriptions see: [Deflate spec](https://www.ietf.org/rfc/rfc1951.txt)
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
    pub starting_block_index: BlockIndex,
    num_frames: u32,
    recording_type: RecordingFileTypeDetails,
}

pub fn make_crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for (n, slot) in table.iter_mut().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let mut c = n as u32;
        for _ in 0..8 {
            if c & 1 == 1 {
                c = 0xedb8_8320 ^ (c >> 1);
            } else {
                c >>= 1;
            }
        }
        *slot = c;
    }
    table
}

impl<'a> CptvStream<'a> {
    pub fn new(
        current_time: i64,
        lepton_version: u8,
        lepton_serial: u32,
        lepton_firmware_version: ((u8, u8, u8), (u8, u8, u8)),
        device_config: &DeviceConfig,
        fs: &mut OnboardFlash,
        huffman_table: &'a [HuffmanEntry; 257],
        crc_table: &'a [u32; 256],
        is_startup_status_recording: bool,
        is_shutdown_status_recording: bool,
        is_user_requested_recording: bool,
    ) -> CptvStream<'a> {
        let starting_block_index = fs.start_file(1);

        // Camera serial, camera firmware, location_altitude, location_timestamp, location_accuracy
        let cptv_header = Cptv2Header::new(
            current_time,
            lepton_version,
            lepton_serial,
            lepton_firmware_version,
            device_config,
            is_startup_status_recording,
            is_shutdown_status_recording,
            is_user_requested_recording,
        );
        let recording_type = RecordingFileTypeDetails {
            startup_status: is_startup_status_recording,
            shutdown_status: is_shutdown_status_recording,
            user_requested: is_user_requested_recording,
        };
        CptvStream {
            cursor: BitCursor::new(),
            crc_table,
            huffman_table,
            crc_val: 0,
            total_uncompressed: 0,
            starting_block_index,
            cptv_header,
            num_frames: 0,
            recording_type,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn push_byte(&mut self, byte: u8, fs: &mut OnboardFlash) {
        self.total_uncompressed += 1;
        self.crc_val ^= 0xffff_ffff;
        self.crc_val = self.crc_table[((self.crc_val ^ u32::from(byte)) & 0xff) as usize]
            ^ (self.crc_val >> 8);
        self.crc_val ^= 0xffff_ffff;
        let entry = &self.huffman_table[usize::from(byte)];
        self.cursor
            .write_bits(u32::from(entry.code), u32::from(entry.bits));

        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            _ = fs.append_recording_bytes(
                to_flush,
                num_bytes,
                RecordingFileType::Cptv(self.recording_type),
            );
            self.cursor.flush_residual_bits();
        }
    }

    /// Writes out the standard gzip header, which says that the following block is the *last* block in
    /// the gzip 'member' - which is a self-contained valid gzip stream.  Then write out the beginning
    /// of a 'dynamic' gzip block, with pre-calculated huffman tables describing only literals
    /// (not match lengths or distances).  This will always be smaller than our page size of 2048 bytes
    /// so there's no need to check if we need to flush out to flash memory.
    pub fn init_gzip_stream(&mut self, fs: &mut OnboardFlash, at_header_location: bool) {
        self.crc_val = 0;
        self.total_uncompressed = 0;

        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index), Some(0))
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
                _ = fs.append_recording_bytes_at_location(
                    to_flush,
                    num_bytes,
                    block_index,
                    page_index,
                    RecordingFileType::Cptv(self.recording_type),
                );
                self.cursor.flush_residual_bits();
            }
        }
        self.cursor.write_bits(
            u32::from(STATIC_LZ77_DYNAMIC_BLOCK_HEADER[STATIC_LZ77_DYNAMIC_BLOCK_HEADER.len() - 1]),
            5,
        );
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            _ = fs.append_recording_bytes_at_location(
                to_flush,
                num_bytes,
                block_index,
                page_index,
                RecordingFileType::Cptv(self.recording_type),
            );
            self.cursor.flush_residual_bits();
        }
    }

    /// Add a new frame to the current CPTV stream, flushing out pages to the flash storage as needed.
    /// This takes two "raw" frames of u16 values, and writes the delta-encoded result into the
    /// `prev_frame` buffer.
    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_possible_wrap)]
    pub fn push_frame(
        &mut self,
        current_frame: &[u16],
        prev_frame: &mut [u16],
        frame_telemetry: &Telemetry,
        fs: &mut OnboardFlash,
    ) {
        let (min_value, max_value, initial_px) = delta_encode_frame_data(prev_frame, current_frame);
        let delta_encoded_frame = prev_frame;

        // We need to work out what the max delta range is and how many bits we can pack this into.
        // In this implementation, we represent the delta-encoded pixels as either
        // 8 or 16 bit values, since we want to pack nicely into full bytes to maximize entropy
        // so that the lz77 algorithm has a lot of the same values to work with.
        // The CPTV file format has these delta pixel values encoded in a zig-zag or snake
        // pattern for each row of pixels: Note that the first row has the first 2 u16 values
        // actually representing an u32 value of the starting value for the frame
        // ...for purely historical reasons.

        // ↓↓ : u32 starting value
        // -----------┐
        //  ┌---------┘
        //  └---------┐
        //  ┌---------┘
        //  └----------
        // ... etc
        let i8_min = i16::from(i8::MIN);
        let i8_max = i16::from(i8::MAX);
        let bits_per_pixel = if delta_encoded_frame[1..]
            .iter()
            .map(|&x| x as i16)
            .any(|x| x < i8_min || x > i8_max)
        {
            16
        } else {
            8
        };
        let frame_size_bytes =
            4 + ((FRAME_HEIGHT * FRAME_WIDTH) - 1) as u32 * (u32::from(bits_per_pixel) / 8);
        let frame_header = CptvFrameHeader {
            time_on: frame_telemetry.msec_on,
            last_ffc_time: frame_telemetry.time_at_last_ffc,
            bit_width: bits_per_pixel,
            frame_size: frame_size_bytes,
            last_ffc_temp_c: frame_telemetry.fpa_temp_c_at_last_ffc,
            frame_temp_c: frame_telemetry.fpa_temp_c,
        };
        let frame_header_iter = frame_header_iter(&frame_header);
        self.cptv_header.min_value = self.cptv_header.min_value.min(min_value);
        self.cptv_header.max_value = self.cptv_header.max_value.max(max_value);
        self.cptv_header.total_frame_count += 1;
        if self.cptv_header.total_frame_count.is_multiple_of(10) {
            info!(
                "Write frame #{}, (#{} in telemetry)",
                self.cptv_header.total_frame_count, frame_telemetry.frame_num
            );
        }

        let snake_iter = delta_encoded_frame
            .chunks_exact(FRAME_WIDTH)
            .step_by(2)
            .zip(
                delta_encoded_frame
                    .chunks_exact(FRAME_WIDTH)
                    .skip(1)
                    .step_by(2),
            )
            .flat_map(|(even_row, odd_row)| even_row.iter().chain(odd_row.iter().rev()))
            .skip(1);
        // NOTE: Here we can insert the i32 start pixel at the beginning of each frame.
        for byte in frame_header_iter.chain(initial_px.to_le_bytes()) {
            self.push_byte(byte, fs);
        }
        if bits_per_pixel == 16 {
            for px in snake_iter {
                for byte in px.to_be_bytes() {
                    self.push_byte(byte, fs);
                }
            }
        } else {
            for &px in snake_iter {
                self.push_byte(px as u8, fs);
            }
        }
        self.num_frames += 1;
    }

    pub fn len(&self) -> u32 {
        self.num_frames
    }

    pub fn reached_max_length(&self) -> bool {
        if self.recording_type.startup_status || self.recording_type.shutdown_status {
            self.num_frames >= NUM_STATUS_RECORDING_FRAMES
        } else {
            let max_length_in_frames = if crate::frame_processing::THERMAL_DEV_MODE {
                60 * 9
            } else {
                60 * 10 * 9
            };
            self.num_frames >= max_length_in_frames
        }
    }

    fn write_gzip_trailer(
        &mut self,
        fs: &mut OnboardFlash,
        at_header_location: bool,
        time: Option<&SyncedDateTime>,
    ) {
        let (block_index, page_index) = if at_header_location {
            (Some(self.starting_block_index), Some(0))
        } else {
            (None, None)
        };

        // End the actual data stream by writing out the end of stream literal.
        let entry = &self.huffman_table[256];
        self.cursor
            .write_bits(u32::from(entry.code), u32::from(entry.bits));
        if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
            _ = fs.append_recording_bytes_at_location(
                to_flush,
                num_bytes,
                block_index,
                page_index,
                RecordingFileType::Cptv(self.recording_type),
            );
            self.cursor.flush_residual_bits();
        }

        // Align to the nearest full byte after finishing the stream.
        let needs_flush = self.cursor.end_aligned();
        if needs_flush {
            let (to_flush, num_bytes) = self.cursor.flush();
            _ = fs.append_recording_bytes_at_location(
                to_flush,
                num_bytes,
                block_index,
                page_index,
                RecordingFileType::Cptv(self.recording_type),
            );
            self.cursor.flush_residual_bits();
        }

        // Write the CRC value for the uncompressed input date of the gzip stream.
        let mut buf = [0u8; 4];
        LittleEndian::write_u32(&mut buf, self.crc_val);
        for b in buf {
            self.cursor.write_byte(b);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                let _ = fs.append_recording_bytes_at_location(
                    to_flush,
                    num_bytes,
                    block_index,
                    page_index,
                    RecordingFileType::Cptv(self.recording_type),
                );
                self.cursor.flush_residual_bits();
            }
        }
        // Write the total uncompressed length of input data
        LittleEndian::write_u32(&mut buf, self.total_uncompressed);
        for b in buf {
            self.cursor.write_byte(b);
            if let Some((to_flush, num_bytes)) = self.cursor.should_flush() {
                let _ = fs.append_recording_bytes_at_location(
                    to_flush,
                    num_bytes,
                    block_index,
                    page_index,
                    RecordingFileType::Cptv(self.recording_type),
                );
                self.cursor.flush_residual_bits();
            }
        }
        // Flush out any remaining bits in the accumulator, align them to the nearest full byte,
        // and write out to storage.

        // FIXME:
        let _ = self.cursor.end_aligned();
        let (to_flush, num_bytes) = self.cursor.flush();
        if at_header_location {
            let _ = fs.append_recording_bytes_at_location_with_time(
                to_flush,
                num_bytes,
                block_index,
                page_index,
                RecordingFileType::Cptv(self.recording_type),
                time,
            );
        } else {
            // Write the last part of the file
            let _ = fs.append_last_recording_bytes_at_location(
                to_flush,
                num_bytes,
                block_index,
                page_index,
                RecordingFileType::Cptv(self.recording_type),
            );
        }
    }

    /// End the current CPTV stream, and write out the CPTV header as a separate gzip member which
    /// will be prepended to the main data stream when transferred out to the raspberry pi.  These
    /// two gzip 'members' or self-contained gip streams when concatenated are themselves a valid
    /// gzip stream, and therefore make up a valid CPTV file.
    pub fn finalise(&mut self, fs: &mut OnboardFlash, time: &SyncedDateTime) {
        // Write out the gzip trailer for the main data - the CPTV frames.
        self.write_gzip_trailer(fs, false, None);

        // Now write the CPTV header into the first page of the starting block as a separate gzip member,
        // now that we know the total frame count for the recording, and the min/max pixel values.
        self.init_gzip_stream(fs, true);
        for byte in push_header_iterator(&self.cptv_header.clone()) {
            self.push_byte(byte, fs);
        }
        self.write_gzip_trailer(fs, true, Some(time));

        if self.recording_type.user_requested {
            fs.file_types_found.add_type(FileType::CptvUserRequested);
        } else if self.recording_type.shutdown_status {
            fs.file_types_found.add_type(FileType::CptvShutdown);
        } else if self.recording_type.startup_status {
            fs.file_types_found.add_type(FileType::CptvStartup);
            fs.last_startup_recording_time = Some(time.date_time());
        } else {
            fs.file_types_found.add_type(FileType::CptvScheduled);
        }
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

pub fn frame_header_iter(frame_header: &CptvFrameHeader) -> impl Iterator<Item = u8> + '_ {
    // Write the frame header
    // We know the number of fields up front:
    [b'F', 6]
        .iter()
        .copied()
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

#[allow(clippy::ref_option)]
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
    let num_header_fields = 14u8;
    #[allow(clippy::cast_possible_truncation)]
    let num_optional_header_fields = [
        header.latitude.is_some(),
        header.longitude.is_some(),
        header.loc_timestamp.is_some(),
        header.altitude.is_some(),
        header.accuracy.is_some(),
        header.status_recording.is_some(),
    ]
    .into_iter()
    .filter(|&x| x)
    .count() as u8;
    HeaderIterator::new(num_header_fields + num_optional_header_fields)
        .chain(push_field_iterator(&header.min_value, FieldType::MinValue))
        .chain(push_field_iterator(&header.max_value, FieldType::MaxValue))
        .chain(push_field_iterator(
            &header.total_frame_count,
            FieldType::NumFrames,
        ))
        .chain(push_field_iterator(&header.timestamp, FieldType::Timestamp))
        .chain(push_field_iterator(&FRAME_WIDTH_U32, FieldType::Width))
        .chain(push_field_iterator(&FRAME_HEIGHT_U32, FieldType::Height))
        .chain(push_field_iterator(&1u8, FieldType::Compression))
        .chain(push_field_iterator(&9u8, FieldType::FrameRate))
        .chain(push_field_iterator(
            &header.device_name,
            FieldType::DeviceName,
        ))
        .chain(push_field_iterator(b"flir", FieldType::Brand))
        .chain(push_field_iterator(&header.model, FieldType::Model))
        .chain(push_field_iterator(&header.device_id, FieldType::DeviceID))
        .chain(push_field_iterator(
            &header.serial_number,
            FieldType::CameraSerial,
        ))
        .chain(push_field_iterator(
            &header.firmware_version,
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
        .chain(push_optional_field_iterator(
            &header.status_recording,
            FieldType::MotionConfig,
        ))
}

// "flir"
// "lepton3.5"
// "<unknown>"
#[derive(Copy, Clone)]
pub struct Cptv2Header {
    pub timestamp: i64,
    pub device_name: MaybeZeroTerminatedString<63>,
    pub model: MaybeZeroTerminatedString<30>,
    pub device_id: u32,
    pub serial_number: u32,
    pub firmware_version: MaybeZeroTerminatedString<30>,
    pub status_recording: Option<MaybeZeroTerminatedString<30>>,
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
        timestamp: i64,
        lepton_version: u8,
        lepton_serial: u32,
        lepton_firmware_version: ((u8, u8, u8), (u8, u8, u8)),
        config: &DeviceConfig,
        is_startup_status_recording: bool,
        is_shutdown_status_recording: bool,
        is_user_requested: bool,
    ) -> Cptv2Header {
        // NOTE: Set default values for things not included in
        // older CPTVv1 files, which can otherwise be decoded as
        // v2.
        let mut firmware = MaybeZeroTerminatedString::<30>::new();
        let mut cursor = CursorMut::new(firmware.as_mut());
        let ((m_major, m_minor, m_build), (d_major, d_minor, d_build)) = lepton_firmware_version;

        let _ = write!(
            &mut cursor,
            "{m_major}.{m_minor}.{m_build}:{d_major}.{d_minor}.{d_build}:{FIRMWARE_VERSION}",
        );

        let status_recording =
            if is_startup_status_recording || is_shutdown_status_recording || is_user_requested {
                let status = if is_startup_status_recording {
                    info!("Creating startup status recording");
                    MaybeZeroTerminatedString::new_with_bytes("status: startup".as_bytes())
                } else if is_shutdown_status_recording {
                    info!("Creating shutdown status recording");
                    MaybeZeroTerminatedString::new_with_bytes("status: shutdown".as_bytes())
                } else {
                    info!("Creating user requested tests recording");
                    MaybeZeroTerminatedString::new_with_bytes("test: true".as_bytes())
                };

                Some(status)
            } else {
                None
            };

        let (lat, lng) = config.config().location;
        Cptv2Header {
            status_recording,
            timestamp,
            device_name: MaybeZeroTerminatedString::new_with_bytes(config.device_name_bytes()),
            model: MaybeZeroTerminatedString::new_with_bytes(if lepton_version == 35 {
                &b"lepton3.5"[..]
            } else {
                &b"lepton3"[..]
            }),
            device_id: config.config().device_id,
            serial_number: lepton_serial,
            firmware_version: firmware,
            latitude: Some(lat),
            longitude: Some(lng),
            loc_timestamp: config.config().location_timestamp,
            altitude: config.config().location_altitude,
            accuracy: config.config().location_accuracy,
            total_frame_count: 0,
            min_value: u16::MAX,
            max_value: u16::MIN,
        }
    }
}
