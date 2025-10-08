use byteorder::{ByteOrder, LittleEndian};

const PAGE_COMMAND_ADDRESS: usize = 4;
const USER_BUFFER_LENGTH: usize = 2048;

// The total buffer length is the same as a single page to be written to our onboard nand flash.
// LZ encoding only uses the first 2048 bytes, which is the user data section.  Other metadata
// is written in the remaining 128 bytes.  Here we only care about the first metadata section,
// so the buffer is truncated from the full page+metadata size.
const BUFFER_LENGTH: usize = PAGE_COMMAND_ADDRESS + 2112;
type PageBuffer = [u8; BUFFER_LENGTH];
pub struct BitCursor {
    used_bits: u32,
    accumulator: u32,
    cursor: usize,
    buffer: PageBuffer,
}

#[allow(clippy::inline_always)]
impl BitCursor {
    // Writing 16 bits needs to be able to succeed, but then we need to
    pub fn new() -> BitCursor {
        BitCursor {
            used_bits: 0,
            accumulator: 0,
            cursor: 0,
            buffer: [0xff; BUFFER_LENGTH], // Initialised to 0xff, mirroring nand flash memory.
        }
    }

    #[inline(always)]
    pub fn write_byte(&mut self, value: u8) {
        self.write_bits(u32::from(value), 8);
    }

    #[inline(always)]
    pub fn write_bits(&mut self, bits: u32, len: u32) {
        self.accumulator |= bits.overflowing_shl(self.used_bits).0;
        self.used_bits += len;
    }

    #[inline(always)]
    pub fn do_flush(&mut self) -> bool {
        #[allow(clippy::cast_possible_truncation)]
        while self.used_bits >= 8 {
            self.buffer[PAGE_COMMAND_ADDRESS..PAGE_COMMAND_ADDRESS + USER_BUFFER_LENGTH]
                [self.cursor] = self.accumulator as u8;
            self.cursor += 1;
            self.accumulator >>= 8;
            self.used_bits -= 8;
            if self.cursor == USER_BUFFER_LENGTH {
                return true;
            }
        }
        self.is_full()
    }

    #[inline(always)]
    pub fn flush(&mut self) -> (&mut PageBuffer, usize) {
        let num_bytes = self.cursor;
        self.cursor = 0;
        (&mut self.buffer, num_bytes)
    }

    #[inline(always)]
    pub fn flush_residual_bits(&mut self, completely: bool) {
        if self.used_bits > 17 || completely {
            let _ = self.do_flush();
        }
    }

    #[inline(always)]
    pub fn is_full(&self) -> bool {
        self.cursor == USER_BUFFER_LENGTH
    }

    #[inline(always)]
    pub fn should_flush(&mut self) -> Option<(&mut PageBuffer, usize)> {
        if self.used_bits >= 17 && self.do_flush() {
            let num_bytes = self.cursor;
            self.cursor = 0;
            Some((&mut self.buffer, num_bytes))
        } else {
            None
        }
    }

    pub fn end_aligned(&mut self) -> (bool, ([u8; 4], u8)) {
        #[allow(clippy::cast_possible_truncation)]
        let num_free_bytes = USER_BUFFER_LENGTH.saturating_sub(self.cursor) as u32;
        // Write as many of these extra bytes as possible into the buffer.
        // We may end up with some remaining bytes we need to flush into the next page.
        let extra_bytes_to_write: u32 = if self.used_bits == 0 {
            0
        } else if self.used_bits <= 8 {
            1
        } else if self.used_bits <= 16 {
            2
        } else if self.used_bits <= 24 {
            3
        } else {
            4
        };

        let num_bits = (extra_bytes_to_write * 8) - self.used_bits;
        if num_bits != 0 {
            self.write_bits(0, num_bits);
        }
        let available_extra_bytes = num_free_bytes.min(extra_bytes_to_write);
        let mut parts = [0u8; 4];
        LittleEndian::write_u32(&mut parts, self.accumulator);
        self.accumulator = 0;
        self.used_bits = 0;
        if available_extra_bytes != 0 {
            for i in 0..available_extra_bytes {
                self.buffer[PAGE_COMMAND_ADDRESS..PAGE_COMMAND_ADDRESS + USER_BUFFER_LENGTH]
                    [self.cursor] = parts[i as usize];
                self.cursor += 1;
            }
        }
        let num_remaining_bytes = (extra_bytes_to_write - available_extra_bytes) as usize;
        let mut remaining_bytes = [0u8; 4];
        if num_remaining_bytes != 0 {
            remaining_bytes[0..num_remaining_bytes].copy_from_slice(
                &parts[available_extra_bytes as usize
                    ..available_extra_bytes as usize + num_remaining_bytes],
            );
        }
        #[allow(clippy::cast_possible_truncation)]
        (self.is_full(), (remaining_bytes, num_remaining_bytes as u8))
    }
}
