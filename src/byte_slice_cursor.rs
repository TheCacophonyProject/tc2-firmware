use byteorder::{ByteOrder, LittleEndian};
use core::fmt;
use embedded_io::{ErrorType, Read, ReadExactError};

pub struct Cursor<'a> {
    inner: &'a [u8],
    pos: usize,
}

pub struct CursorMut<'a> {
    inner: &'a mut [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    pub const fn new(inner: &'a [u8]) -> Cursor<'a> {
        Cursor { pos: 0, inner }
    }
    pub const fn position(&self) -> usize {
        self.pos
    }
    pub fn set_position(&mut self, pos: usize) {
        self.pos = pos;
    }
    pub fn remaining_slice(&self) -> &[u8] {
        let len = self.pos.min(self.inner.as_ref().len());
        &self.inner.as_ref()[len..]
    }

    pub fn read_u32(&mut self) -> u32 {
        let result = LittleEndian::read_u32(&self.inner[self.pos..self.pos + 4]);
        self.pos += 4;
        result
    }
    pub fn read_f32(&mut self) -> f32 {
        let result = LittleEndian::read_f32(&self.inner[self.pos..self.pos + 4]);
        self.pos += 4;
        result
    }
    pub fn read_i32(&mut self) -> i32 {
        let result = LittleEndian::read_i32(&self.inner[self.pos..self.pos + 4]);
        self.pos += 4;
        result
    }
    pub fn read_u8(&mut self) -> u8 {
        let result = self.inner[self.pos];
        self.pos += 1;
        result
    }
    pub fn read_bool(&mut self) -> bool {
        self.read_u8() == 1
    }
    pub fn read_u64(&mut self) -> u64 {
        let result = LittleEndian::read_u64(&self.inner[self.pos..self.pos + 8]);
        self.pos += 8;
        result
    }

    pub fn read_i64(&mut self) -> i64 {
        let result = LittleEndian::read_i64(&self.inner[self.pos..self.pos + 8]);
        self.pos += 8;
        result
    }
}

impl<'a> CursorMut<'a> {
    pub fn new(inner: &'a mut [u8]) -> CursorMut<'a> {
        CursorMut { pos: 0, inner }
    }
    pub const fn position(&self) -> usize {
        self.pos
    }
    pub fn set_position(&mut self, pos: usize) {
        self.pos = pos;
    }
    pub fn remaining_slice(&mut self) -> &mut [u8] {
        let len = self.pos.min(self.inner.as_mut().len());
        &mut self.inner.as_mut()[len..]
    }
}

impl<'a> ErrorType for Cursor<'a> {
    type Error = embedded_io::ErrorKind;
}

impl<'a> ErrorType for CursorMut<'a> {
    type Error = embedded_io::ErrorKind;
}

impl<'a> Read for Cursor<'a> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = Read::read(&mut self.remaining_slice(), buf).unwrap();
        self.pos += n;
        Ok(n)
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        let n = buf.len();
        Read::read_exact(&mut self.remaining_slice(), buf).unwrap();
        self.pos += n;
        Ok(())
    }
}

impl<'a> fmt::Write for CursorMut<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();

        // Skip over already-copied data
        let remainder = &mut self.inner[self.pos..];
        // Check if there is space remaining (return error instead of panicking)
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }
        // Make the two slices the same length
        let remainder = &mut remainder[..bytes.len()];
        // Copy
        remainder.copy_from_slice(bytes);

        // Update offset to avoid overwriting
        self.pos += bytes.len();

        Ok(())
    }
}
