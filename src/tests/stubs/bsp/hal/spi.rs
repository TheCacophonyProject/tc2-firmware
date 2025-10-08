extern crate std;
use crate::onboard_flash::{
    BLOCK_ERASE, BlockIndex, CACHE_READ, FEATURE_STATUS, FLASH_SPI_HEADER, GET_FEATURES, PAGE_READ,
    PROGRAM_EXECUTE, PROGRAM_LOAD, PageIndex, RESET, SET_FEATURES, WRITE_ENABLE,
};
use crate::re_exports::bsp::pac::RESETS;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;
use fugit::HertzU32;
use log::error;
use std::convert::Infallible;
use std::println;
use std::vec::Vec;

pub struct Enabled;
pub struct Disabled;
pub struct Spi<E, S, P, const N: usize> {
    slave: bool,
    inner: (E, S, P),
    pub(crate) buffer: Vec<u8>,
    last_read_address: (BlockIndex, PageIndex),
    last_page_offset: usize,
}

pub trait SpiMarker {}
impl<E, S, P> SpiMarker for Spi<E, S, P, 8> {}

impl<S, P> Spi<Disabled, S, P, 8> {
    pub fn new(peripheral: S, p: P) -> Spi<Disabled, S, P, 8> {
        Spi {
            slave: false,
            inner: (Disabled, peripheral, p),
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }

    pub fn init(
        self,
        _resets: &RESETS,
        _freq: HertzU32,
        _p_freq: HertzU32,
        _mode: embedded_hal::spi::Mode,
    ) -> Spi<Enabled, S, P, 8> {
        Spi {
            slave: false,
            inner: (Enabled, self.inner.1, self.inner.2),
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }

    pub fn init_slave(
        self,
        _resets: &RESETS,
        _mode: embedded_hal::spi::Mode,
    ) -> Spi<Enabled, S, P, 8> {
        Spi {
            slave: true,
            inner: (Enabled, self.inner.1, self.inner.2),
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }
    pub fn free(self) -> (S, P) {
        (self.inner.1, self.inner.2)
    }
}

impl<S, P> Spi<Disabled, S, P, 16> {
    pub fn new(peripheral: S, p: P) -> Spi<Disabled, S, P, 16> {
        Spi {
            slave: false,
            inner: (Disabled, peripheral, p),
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }

    pub fn init(
        self,
        _resets: &RESETS,
        _freq: HertzU32,
        _p_freq: HertzU32,
        _mode: embedded_hal::spi::Mode,
    ) -> Spi<Enabled, S, P, 16> {
        Spi {
            slave: false,
            inner: (Enabled, self.inner.1, self.inner.2),
            buffer: Vec::new(),
            last_read_address: (0, 0),
            last_page_offset: 0,
        }
    }

    pub fn free(self) -> (S, P) {
        (self.inner.1, self.inner.2)
    }
}

impl<S, P> Spi<Enabled, S, P, 16> {
    pub fn disable(self) -> Spi<Disabled, S, P, 16> {
        Spi {
            slave: self.slave,
            inner: (Disabled, self.inner.1, self.inner.2),
            buffer: self.buffer,
            last_read_address: self.last_read_address,
            last_page_offset: self.last_page_offset,
        }
    }
    pub fn transfer<'a>(&mut self, dst: &'a mut [u16]) -> Result<&'a [u16], Infallible> {
        Ok(dst)
    }
}

impl<S, P> Spi<Enabled, S, P, 8> {
    pub fn disable(self) -> Spi<Disabled, S, P, 8> {
        Spi {
            slave: self.slave,
            inner: (Disabled, self.inner.1, self.inner.2),
            buffer: self.buffer,
            last_read_address: self.last_read_address,
            last_page_offset: self.last_page_offset,
        }
    }

    pub fn transfer(&mut self, dst: &mut [u8]) -> Result<(), ()> {
        match dst[0] {
            CACHE_READ => {
                let col_offset_0 = dst[1] as u16;
                let col_offset_1 = dst[2] as u16;
                let col_offset = ((col_offset_0 & 0x0f) << 8 | col_offset_1) as usize;
                let len = col_offset..col_offset + dst.len() - FLASH_SPI_HEADER;
                dst[FLASH_SPI_HEADER..].copy_from_slice(&self.buffer[len]);
            }
            GET_FEATURES => {
                let feature_type = dst[1];
                match feature_type {
                    FEATURE_STATUS => {
                        if TEST_SIM_STATE.with(|s| {
                            s.borrow()
                                .ecc_error_addresses
                                .contains(&self.last_read_address)
                        }) {
                            println!("ECC Error @ {:?}", self.last_read_address);
                            dst[2] = 0b0010_0010;
                        } else {
                            dst[2] = 0b0000_0010;
                        }
                    }
                    _ => error!("Unhandled feature_type {:?}", feature_type),
                }
            }
            _ => error!("unhandled command 0x{:02x?}", dst[0]),
        }
        Ok(())
    }

    fn get_block_and_page_from_address(address_bytes: [u8; 3]) -> (BlockIndex, PageIndex) {
        // Reconstruct the 32-bit address from the 3 bytes
        let address: u32 = (u32::from(address_bytes[0]) << 16)
            | (u32::from(address_bytes[1]) << 8)
            | u32::from(address_bytes[2]);

        // Extract page from lower 6 bits
        let page = (address & 0x3F) as PageIndex; // 0x3F = 0b111111 (6 bits)

        // Extract block from upper bits (shift right by 6)
        let block = (address >> 6) as BlockIndex;

        (block, page)
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<(), ()> {
        match bytes[0] {
            PROGRAM_LOAD => {
                // Set the plane to write some bytes to, nop
                // FIXME: Load needs to also handle offsets into the page
                let offset_in_page: u16 = (bytes[1] as u16 & 0x0f) << 8 | bytes[2] as u16;
                self.last_page_offset = offset_in_page as usize;
                self.buffer.drain(..);
                self.buffer
                    .extend_from_slice(&bytes[FLASH_SPI_HEADER - 1..]);
            }
            PROGRAM_EXECUTE => {
                // 11 bits for block, 6 bits for page
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);

                // NOTE: was 2112 instead of buffer.len()
                TEST_SIM_STATE.with(|s| {
                    s.borrow_mut().flash_backing_storage[block_index as usize].inner
                        [page_index as usize]
                        .inner[self.last_page_offset..self.last_page_offset + self.buffer.len()]
                        .copy_from_slice(&self.buffer);
                });
            }
            PAGE_READ => {
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);
                self.last_read_address = (block_index, page_index);
                self.buffer.drain(..);
                TEST_SIM_STATE.with(|s| {
                    let s = s.borrow();
                    self.buffer.extend_from_slice(
                        &s.flash_backing_storage[block_index as usize].inner[page_index as usize]
                            .inner,
                    );

                    if s.ecc_error_addresses.contains(&self.last_read_address) {
                        //println!("Corrupting buffer");
                        self.buffer[1] = 0x42;
                    }
                });
            }
            RESET => {
                self.buffer.drain(..);
            }
            WRITE_ENABLE => {
                // Do nothing
            }
            SET_FEATURES => {
                // Do nothing
            }
            BLOCK_ERASE => {
                let address_0 = bytes[1];
                let address_1 = bytes[2];
                let address_2 = bytes[3];
                let (block_index, _page_index) =
                    Self::get_block_and_page_from_address([address_0, address_1, address_2]);
                TEST_SIM_STATE.with(|s| {
                    let mut s = s.borrow_mut();
                    s.ecc_error_addresses.retain(|(b, p)| *b != block_index);
                    self.last_read_address = (0, 0);
                    for page in &mut s.flash_backing_storage[block_index as usize].inner {
                        page.inner.fill(0xff);
                    }
                });
            }
            _ => panic!("Unhandled command 0x{:02x?}", bytes[0]),
        }
        Ok(())
    }
}
