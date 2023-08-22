use byteorder::{ByteOrder, LittleEndian};

const BUFFER_LENGTH: usize = 4;
type Buffer = [u8; BUFFER_LENGTH];
pub struct BitCursor {
    unused_bits: u8,
    accumulator: u16,
    cursor: usize,
    front: u8,
    buffer: Buffer,
    bytes_flushed: usize,
}

impl BitCursor {
    // Writing 16 bits needs to be able to succeed, but then we need to
    pub fn new() -> BitCursor {
        BitCursor {
            unused_bits: 16,
            accumulator: 0,
            cursor: 0,
            front: 0,
            buffer: [0u8; BUFFER_LENGTH],
            bytes_flushed: 0,
        }
    }

    pub fn write_byte(&mut self, value: u8) -> bool {
        self.write_bits(value as u16, 8)
    }

    pub fn write_bits(&mut self, value: u16, num_bits: u8) -> bool {
        let mut val = value;
        let mut nb = num_bits;
        while nb > 0 {
            self.write_bit((val & 1) as u8);
            val >>= 1;
            nb -= 1;
        }
        self.is_full()
    }

    pub fn used_bits(&self) -> u8 {
        16 - self.unused_bits
    }

    pub fn unused_bits(&self) -> u8 {
        self.unused_bits
    }
    pub fn flush(&mut self) -> (Buffer, usize) {
        let num_bytes = self.cursor;
        self.bytes_flushed += self.cursor;
        self.cursor = 0;
        // NOTE: We might decide to just block on the write, and reuse this
        //  buffer to save memory.
        (self.buffer.clone(), num_bytes)
    }
    #[inline(always)]
    pub fn bytes_written(&self) -> usize {
        self.bytes_flushed
    }

    #[inline(always)]
    pub fn is_full(&self) -> bool {
        self.cursor == BUFFER_LENGTH
    }

    #[inline(always)]
    pub fn write_bit(&mut self, bit: u8) -> bool {
        debug_assert!(bit == 1 || bit == 0);
        if self.unused_bits > 0 {
            self.accumulator |= (bit as u16) << (16 - self.unused_bits as u16);
            self.unused_bits -= 1;
            self.is_full()
        } else {
            self.unused_bits = 16;
            let mut parts = [0u8; 2];
            LittleEndian::write_u16(&mut parts, self.accumulator);
            self.accumulator = 0;
            self.buffer[self.cursor] = parts[0];
            self.cursor += 1;
            self.buffer[self.cursor] = parts[1];
            self.cursor += 1;
            self.write_bit(bit);
            self.is_full()
        }
    }

    pub fn end_aligned(&mut self) -> bool {
        if self.used_bits() != 0 {
            if self.used_bits() <= 8 {
                while self.unused_bits > 8 {
                    self.write_bit(0);
                }
                let mut parts = [0u8; 2];
                LittleEndian::write_u16(&mut parts, self.accumulator);
                self.accumulator = 0;
                self.buffer[self.cursor] = parts[0];
                self.cursor += 1;
                self.unused_bits = 16;
            } else {
                while self.unused_bits > 0 {
                    self.write_bit(0);
                }
                let mut parts = [0u8; 2];
                LittleEndian::write_u16(&mut parts, self.accumulator);
                self.accumulator = 0;
                self.buffer[self.cursor] = parts[0];
                self.cursor += 1;
                self.buffer[self.cursor] = parts[1];
                self.cursor += 1;
                self.unused_bits = 16;
            }
        }
        self.is_full()
    }
}
