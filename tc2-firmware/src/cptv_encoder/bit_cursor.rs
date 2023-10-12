use byteorder::{ByteOrder, LittleEndian};
use defmt::info;

const PAGE_COMMAND_ADDRESS: usize = 4;
const USER_BUFFER_LENGTH: usize = 2048;
const META_BUFFER_LENGTH: usize = 128;

// Total buffer length is the same as a single page to be written to our onboard nand flash.
// LZ encoding only uses the first 2048 bytes, which is the user data section.  Other meta data
// is written in the remaining 128 bytes.
const BUFFER_LENGTH: usize = PAGE_COMMAND_ADDRESS + 2112; //USER_BUFFER_LENGTH + META_BUFFER_LENGTH;
type PageBuffer = [u8; BUFFER_LENGTH];
pub struct BitCursor {
    used_bits: u32,
    accumulator: u32,
    cursor: usize,
    buffer: PageBuffer,
}

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
        self.write_bits(value as u32, 8)
    }

    #[inline(always)]
    pub fn write_bits(&mut self, bits: u32, len: u32) {
        self.accumulator |= bits.overflowing_shl(self.used_bits).0;
        self.used_bits += len;
    }

    pub fn do_flush(&mut self) -> bool {
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
    pub fn flush(&mut self) -> (&mut PageBuffer, usize) {
        let num_bytes = self.cursor;
        self.cursor = 0;
        (&mut self.buffer, num_bytes)
    }

    // Always call this after a flush to reset to 0xff
    pub fn reset(&mut self) {
        self.buffer = [0xff; BUFFER_LENGTH];
    }

    #[inline(always)]
    pub fn is_full(&self) -> bool {
        self.cursor == USER_BUFFER_LENGTH
    }

    pub fn should_flush(&mut self) -> Option<(&mut PageBuffer, usize)> {
        if self.used_bits >= 17 && self.do_flush() {
            let num_bytes = self.cursor;
            self.cursor = 0;
            Some((&mut self.buffer, num_bytes))
        } else {
            None
        }
    }

    pub fn end_aligned(&mut self) -> bool {
        //info!("End aligned with {} bits in acc", self.used_bits);
        if self.used_bits != 0 {
            let extra_bytes = if self.used_bits <= 8 {
                1
            } else if self.used_bits <= 16 {
                2
            } else if self.used_bits <= 24 {
                3
            } else {
                4
            };

            let num_bits = (extra_bytes * 8) - self.used_bits;
            if num_bits != 0 {
                self.write_bits(0, num_bits);
            }
            // info!(
            //     "Padding with {} bits, then flushing out {} bytes",
            //     num_bits, extra_bytes
            // );

            let mut parts = [0u8; 4];
            LittleEndian::write_u32(&mut parts, self.accumulator);
            self.accumulator = 0;
            self.used_bits = 0;
            for i in 0..extra_bytes {
                self.buffer[PAGE_COMMAND_ADDRESS..PAGE_COMMAND_ADDRESS + USER_BUFFER_LENGTH]
                    [self.cursor] = parts[i as usize];
                self.cursor += 1;
            }
        }
        self.is_full()
    }
}
