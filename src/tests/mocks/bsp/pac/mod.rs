use crate::re_exports::bsp::hal::dma::{CH0, CH1, CH2, CH3, CH4, Channel, Channels, DMAExt};
use crate::re_exports::bsp::hal::pio::PIOExt;
use crate::tests::test_state::test_global_state::TEST_SIM_STATE;

#[allow(non_camel_case_types)]
pub struct DMA;

impl DMA {
    pub fn inte0(&self) -> CLOCKS {
        CLOCKS
    }
    pub fn inte1(&self) -> CLOCKS {
        CLOCKS
    }
    pub fn chan_abort(&self) -> RW_STUB {
        RW_STUB
    }

    pub fn ch(&self, _ch_num: usize) -> DMA {
        DMA
    }

    pub fn ch_read_addr(&self) -> KHZ_STUB {
        KHZ_STUB
    }

    pub fn ch_ctrl_trig(&self) -> RW_STUB {
        RW_STUB
    }
}

impl DMAExt for DMA {
    fn split(self, _resets: &mut RESETS) -> Channels {
        Channels {
            ch0: Channel(CH0),
            ch1: Channel(CH1),
            ch2: Channel(CH2),
            ch3: Channel(CH3),
            ch4: Channel(CH4),
        }
    }
}

#[allow(non_camel_case_types)]
pub enum Interrupt {
    IO_IRQ_BANK0,
    TIMER_IRQ_0,
}
#[allow(non_camel_case_types)]
pub struct NVIC;

impl NVIC {
    pub fn unmask(_int: Interrupt) {}
    pub fn mask(_int: Interrupt) {}
}

#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
pub struct PIO0;

#[derive(Copy, Clone)]
pub struct PIO1;

impl PIOExt for PIO0 {}
impl PIOExt for PIO1 {}

#[allow(non_camel_case_types)]
pub struct RESETS;
#[allow(non_camel_case_types)]
pub struct SPI0;
#[allow(non_camel_case_types)]
pub struct SPI1;
#[allow(non_camel_case_types)]
pub struct CLOCKS;
#[allow(non_camel_case_types)]
pub struct RW_STUB;
#[allow(non_camel_case_types)]
pub struct KHZ_STUB;
#[allow(non_camel_case_types)]
pub struct BITS_STUB;
impl KHZ_STUB {
    pub fn write<F>(&self, f: F)
    where
        F: FnOnce(&mut KHZ_STUB) -> (),
    {
        f(&mut KHZ_STUB)
    }

    pub fn read(&self) -> KHZ_STUB {
        KHZ_STUB
    }
    pub fn done(&self) -> CLOCKS {
        CLOCKS
    }

    pub fn bits(&self) -> u32 {
        125_000
    }
    pub fn running(&self) -> CLOCKS {
        CLOCKS
    }
    pub fn khz(&self) -> KHZ_STUB {
        KHZ_STUB
    }
}
impl BITS_STUB {
    pub fn freq_range(&self) -> RW_STUB {
        RW_STUB
    }
    pub fn bits(&self) -> u8 {
        1
    }

    pub fn busy(&self) -> RW_STUB {
        RW_STUB
    }
    pub fn ds0(&self) -> BITS_STUB {
        BITS_STUB
    }

    pub fn ds1(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds2(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds3(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds4(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds5(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds6(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn ds7(&self) -> BITS_STUB {
        BITS_STUB
    }
}
impl RW_STUB {
    pub fn write<F>(&self, f: F)
    where
        F: FnOnce(&mut RW_STUB) -> (),
    {
        f(&mut RW_STUB)
    }
    pub fn read(&self) -> BITS_STUB {
        BITS_STUB
    }
    pub fn running(&self) -> Self {
        RW_STUB
    }
    pub fn freq_range(&self) -> RW_STUB {
        RW_STUB
    }

    pub fn bit_is_set(&self) -> bool {
        false
    }

    pub fn low(&self) -> () {
        ()
    }
    pub fn medium(&self) -> () {
        ()
    }
    pub fn high(&self) -> () {
        ()
    }

    pub fn variant(&self) -> Option<rosc::ctrl::FREQ_RANGE_A> {
        TEST_SIM_STATE.with(|s| {
            let mut s = s.borrow_mut();
            if s.rosc_drive_iterator == 0 {
                s.rosc_drive_iterator += 1;
                None
            } else {
                Some(rosc::ctrl::FREQ_RANGE_A::HIGH)
            }
        })
    }

    pub fn bits(&mut self, _b: u32) {}
}

impl CLOCKS {
    pub fn write<F>(&self, f: F) -> u32
    where
        F: FnOnce(&mut CLOCKS) -> u32,
    {
        f(&mut CLOCKS)
    }
    pub fn bits(&mut self, _b: u32) -> u32 {
        1
    }

    pub fn khz(&self) -> KHZ_STUB {
        KHZ_STUB
    }
    pub fn read(&self) -> KHZ_STUB {
        KHZ_STUB
    }
    pub fn running(&self) -> Self {
        CLOCKS
    }
    pub fn bit_is_set(&self) -> bool {
        false
    }
    pub fn bit_is_clear(&self) -> bool {
        false
    }
    pub fn done(&self) -> CLOCKS {
        CLOCKS
    }

    pub(crate) fn clk_ref_ctrl(&self) -> CLOCKS {
        CLOCKS
    }

    pub(crate) fn src(&self) -> CLOCKS {
        CLOCKS
    }

    pub fn xosc_clksrc(&self) -> u32 {
        1
    }

    pub(crate) fn fc0_result(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_src(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_max_khz(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_min_khz(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_interval(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_status(&self) -> CLOCKS {
        CLOCKS
    }
    pub(crate) fn fc0_ref_khz(&self) -> CLOCKS {
        CLOCKS
    }
}

#[allow(non_camel_case_types)]
pub struct XOSC;
#[allow(non_camel_case_types)]
pub struct ROSC;
#[allow(non_camel_case_types)]
pub struct WATCHDOG;
#[allow(non_camel_case_types)]
pub struct TIMER;
#[allow(non_camel_case_types)]
pub struct IO_BANK0;
#[allow(non_camel_case_types)]
pub struct PADS_BANK0;
#[allow(non_camel_case_types)]
pub struct I2C0;
#[allow(non_camel_case_types)]
pub struct I2C1;
#[allow(non_camel_case_types)]
pub struct SIO;

impl ROSC {
    pub fn div(&self) -> RW_STUB {
        RW_STUB
    }

    pub fn ctrl(&self) -> RW_STUB {
        RW_STUB
    }

    pub fn freqa(&self) -> RW_STUB {
        RW_STUB
    }
    pub fn freqb(&self) -> RW_STUB {
        RW_STUB
    }
}

pub struct PSM;
pub struct PPB;

#[allow(non_snake_case)]
pub struct Peripherals {
    pub(crate) DMA: DMA,
    pub(crate) PIO0: PIO0,
    pub(crate) RESETS: RESETS,
    pub(crate) CLOCKS: CLOCKS,
    pub(crate) XOSC: XOSC,
    pub(crate) ROSC: ROSC,
    pub(crate) WATCHDOG: WATCHDOG,
    pub(crate) TIMER: TIMER,
    pub(crate) SIO: SIO,
    pub(crate) IO_BANK0: IO_BANK0,
    pub(crate) PADS_BANK0: PADS_BANK0,
    pub(crate) I2C0: I2C0,
    pub(crate) I2C1: I2C1,
    pub(crate) SPI1: SPI1,
    pub(crate) SPI0: SPI0,
    pub(crate) PIO1: PIO1,
    pub(crate) PSM: PSM,
    pub(crate) PPB: PPB,
}

impl Peripherals {
    pub fn take() -> Option<Self> {
        Some(Peripherals {
            DMA,
            PIO0,
            RESETS,
            CLOCKS,
            XOSC,
            ROSC,
            WATCHDOG,
            TIMER,
            SIO,
            IO_BANK0,
            PADS_BANK0,
            I2C0,
            I2C1,
            SPI0,
            SPI1,
            PIO1,
            PSM,
            PPB,
        })
    }

    pub unsafe fn steal() -> Self {
        Self::take().unwrap()
    }
}

pub fn interrupt() {}

pub mod rosc {
    pub mod ctrl {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(u16)]
        #[allow(non_camel_case_types)]
        pub enum FREQ_RANGE_A {
            #[doc = "4004: `111110100100`"]
            LOW = 4004,
            #[doc = "4005: `111110100101`"]
            MEDIUM = 4005,
            #[doc = "4007: `111110100111`"]
            HIGH = 4007,
            #[doc = "4006: `111110100110`"]
            TOOHIGH = 4006,
        }
    }
}
