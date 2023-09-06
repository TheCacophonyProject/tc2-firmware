use byteorder::{ByteOrder, LittleEndian};

const PAGE_COMMAND_ADDRESS: usize = 4;
const USER_BUFFER_LENGTH: usize = 2048;
const META_BUFFER_LENGTH: usize = 128;

// Total buffer length is the same as a single page to be written to our onboard nand flash.
// LZ encoding only uses the first 2048 bytes, which is the user data section.  Other meta data
// is written in the remaining 128 bytes.
const BUFFER_LENGTH: usize = PAGE_COMMAND_ADDRESS + USER_BUFFER_LENGTH + META_BUFFER_LENGTH;
type PageBuffer = [u8; BUFFER_LENGTH];
pub struct BitCursor {
    //unused_bits: u16,
    pub used_bits: u32,
    accumulator: u32,
    cursor: usize,
    front: u8,
    buffer: PageBuffer,
    bytes_flushed: usize,
}

impl BitCursor {
    // Writing 16 bits needs to be able to succeed, but then we need to
    pub fn new() -> BitCursor {
        BitCursor {
            used_bits: 0,
            //unused_bits: 16,
            accumulator: 0,
            cursor: 0,
            front: 0,
            buffer: [0xff; BUFFER_LENGTH], // Initialised to 0xff, mirroring nand flash memory.
            bytes_flushed: 0,
        }
    }

    #[inline(always)]
    fn buffer_user(&mut self) -> &mut [u8] {
        &mut self.buffer[PAGE_COMMAND_ADDRESS..PAGE_COMMAND_ADDRESS + USER_BUFFER_LENGTH]
    }

    #[inline(always)]
    pub fn write_byte(&mut self, value: u8) -> bool {
        self.write_bits(value as u32, 8)
    }

    pub fn write_bits_fast(&mut self, bits: u32, len: u32) {
        self.accumulator |= bits << self.used_bits;
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
        self.cursor == USER_BUFFER_LENGTH
    }

    pub fn write_bits(&mut self, value: u32, num_bits: u32) -> bool {
        // let mut val = value;
        // let mut ub = self.used_bits;
        // let mut ac = self.accumulator;
        // let mut cursor = self.cursor;
        //
        // if num_bits < 32 - ub {
        //     // We can just write the whole lot of bits in, surely.
        //     val <<= ub;
        //     ac |= val;
        //     ub += num_bits;
        // } else {
        //     for _ in 0..num_bits {
        //         let bit = val & 1;
        //         if ub < 32 {
        //             ac |= bit << ub;
        //             ub += 1;
        //         } else {
        //             ub = 0;
        //             LittleEndian::write_u32(&mut self.buffer[4..2052][cursor..cursor + 4], ac);
        //             cursor += 4;
        //             ac = bit;
        //             ub = 1;
        //         }
        //         val >>= 1;
        //     }
        //     ub += num_bits;
        // }
        // self.cursor = cursor;
        // self.used_bits = ub;
        // self.accumulator = ac;
        // self.cursor == USER_BUFFER_LENGTH

        let mut val = value;
        if num_bits < 32 - self.used_bits {
            val <<= self.used_bits;
            self.accumulator |= val;
            self.used_bits += num_bits;
        } else {
            for _ in 0..num_bits {
                let bit = val & 1;
                if self.used_bits < 32 {
                    self.accumulator |= bit << self.used_bits;
                    self.used_bits += 1;
                } else {
                    self.used_bits = 0;
                    let mut parts = [0u8; 4];
                    LittleEndian::write_u32(&mut parts, self.accumulator);
                    self.accumulator = 0;
                    self.buffer[4..2052][self.cursor] = parts[0];
                    self.cursor += 1;
                    self.buffer[4..2052][self.cursor] = parts[1];
                    self.cursor += 1;
                    self.buffer[4..2052][self.cursor] = parts[2];
                    self.cursor += 1;
                    self.buffer[4..2052][self.cursor] = parts[3];
                    self.cursor += 1;

                    self.accumulator |= bit << (self.used_bits as u32);
                    self.used_bits += 1;
                }
                val >>= 1;
            }
        }
        self.is_full()
    }

    pub fn used_bits(&self) -> u32 {
        self.used_bits
    }

    pub fn unused_bits(&self) -> u32 {
        32 - self.used_bits
    }
    pub fn flush<'a>(&'a mut self) -> (&'a mut PageBuffer, usize) {
        let num_bytes = self.cursor;
        self.bytes_flushed += self.cursor;
        self.cursor = 0;
        (&mut self.buffer, num_bytes)
    }

    // Always call this after a flush to reset to 0xff
    pub fn reset(&mut self) {
        self.buffer = [0xff; BUFFER_LENGTH];
    }
    #[inline(always)]
    pub fn bytes_written(&self) -> usize {
        self.bytes_flushed
    }

    #[inline(always)]
    pub fn is_full(&self) -> bool {
        self.cursor == USER_BUFFER_LENGTH
    }

    #[inline(always)]
    pub fn write_bit(&mut self, bit: u32) {
        //debug_assert!(bit == 1 || bit == 0);
        if self.used_bits < 32 {
            self.accumulator |= bit << self.used_bits;
            self.used_bits += 1;
        } else {
            self.used_bits = 0;
            LittleEndian::write_u32(
                &mut self.buffer[4..2052][self.cursor..self.cursor + 4],
                self.accumulator,
            );
            self.accumulator = 0;
            self.cursor += 4;
            self.accumulator |= bit << self.used_bits;
            self.used_bits += 1;
        }
        //self.cursor == USER_BUFFER_LENGTH
    }

    // pub fn end_aligned(&mut self) -> bool {
    //     if self.used_bits() != 0 {
    //         if self.used_bits() <= 8 {
    //             while self.unused_bits() > 8 {
    //                 self.write_bit(0);
    //             }
    //             let mut parts = [0u8; 4];
    //             LittleEndian::write_u32(&mut parts, self.accumulator);
    //             self.accumulator = 0;
    //             let cursor = self.cursor;
    //             self.buffer_user()[cursor] = parts[0];
    //             self.cursor += 1;
    //             self.used_bits = 0;
    //         } else {
    //             while self.unused_bits() > 0 {
    //                 self.write_bit(0);
    //             }
    //             let mut parts = [0u8; 4];
    //             LittleEndian::write_u32(&mut parts, self.accumulator);
    //             self.accumulator = 0;
    //             let mut cursor = self.cursor;
    //             self.buffer_user()[cursor] = parts[0];
    //             cursor += 1;
    //             self.buffer_user()[cursor] = parts[1];
    //             cursor += 1;
    //             self.buffer_user()[cursor] = parts[0];
    //             cursor += 1;
    //             self.buffer_user()[cursor] = parts[1];
    //             cursor += 1;
    //             self.cursor = cursor;
    //             self.used_bits = 0;
    //         }
    //     }
    //     self.is_full()
    // }

    pub fn should_flush<'a>(&'a mut self) -> Option<(&'a mut PageBuffer, usize)> {
        if self.used_bits >= 17 && self.do_flush() {
            let num_bytes = self.cursor;
            self.bytes_flushed += self.cursor;
            self.cursor = 0;
            Some((&mut self.buffer, num_bytes))
        } else {
            None
        }
    }

    pub fn end_aligned(&mut self) -> bool {
        if self.used_bits != 0 {
            let extra_bytes = if self.used_bits < 8 {
                1
            } else if self.used_bits < 16 {
                2
            } else if self.used_bits < 24 {
                3
            } else {
                4
            };

            let num_bits = (extra_bytes * 8) - self.used_bits;
            self.write_bits_fast(0, num_bits);

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
