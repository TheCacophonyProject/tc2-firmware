extern crate std;
pub mod i2c;
pub mod sio;
pub mod spi;

use crate::re_exports::bsp::hal::clocks::ClocksManager;
use crate::re_exports::bsp::pac::{RESETS, TIMER};
use byteorder::ByteOrder;
use chrono::{Datelike, Timelike};
use fugit::{HertzU32, TimerInstantU64};

pub type Instant = TimerInstantU64<1_000_000>;

pub mod timer {
    pub use crate::re_exports::bsp::hal::Instant;
    use crate::tests::test_state::test_global_state::{CAMERA_STATE, TC2_AGENT_STATE};
    use fugit::MicrosDurationU32;

    pub trait Alarm {
        fn schedule(&self, when: MicrosDurationU32) -> Result<(), ()> {
            Ok(())
        }

        fn enable_interrupt(&self) {}
        fn finished(&self) -> bool {
            let camera_state = CAMERA_STATE.lock().unwrap();
            if camera_state.pi_is_powered_on() {
                // Pi is ready to receive
                !TC2_AGENT_STATE.lock().unwrap().is_ready()
            } else {
                true
            }
        }
        fn clear_interrupt(&self) {}
        fn cancel(&self) {}
        fn disable_interrupt(&self) {}
    }
    pub struct Alarm0;

    impl Alarm for Alarm0 {}
}

#[derive(Copy, Clone)]
pub struct Timer {
    pub counter: u64,
}

impl Timer {
    pub fn new(timer: TIMER, resets: &RESETS, clocks: &ClocksManager) -> Timer {
        Timer { counter: 0 }
    }

    pub fn get_counter(&self) -> Instant {
        // TODO: Need to make counter increase?
        TimerInstantU64::from_ticks(self.counter)
    }

    pub fn delay_us(&mut self, micros: u32) {
        //info!("Delaying for {}µs", micros);
    }
    pub fn delay_ms(&self, ms: u64) {
        //info!("Delaying for {}ms", ms);
    }

    pub fn alarm_0(&mut self) -> Option<timer::Alarm0> {
        Some(timer::Alarm0)
    }
}
pub trait Clock {
    fn configure_clock<S>(&self, src: &S, freq: HertzU32) -> Result<(), ()> {
        Ok(())
    }

    fn disable(&self) {}

    fn freq(&self) -> HertzU32 {
        HertzU32::MHz(125)
    }
}
//pub struct Clock;
pub mod clocks {
    pub(crate) use crate::re_exports::bsp::hal::Clock;
    use crate::re_exports::bsp::pac::CLOCKS;
    use fugit::HertzU32;

    pub struct ClocksManager {
        pub system_clock: SystemClock,
        pub peripheral_clock: PeripheralClock,
        pub reference_clock: ReferenceClock,
        pub usb_clock: ClockSource,
        pub adc_clock: ClockSource,
        pub gpio_output0_clock: ClockSource,
        pub gpio_output1_clock: ClockSource,
        pub gpio_output2_clock: ClockSource,
        pub gpio_output3_clock: ClockSource,
        pub rtc_clock: ClockSource,
    }

    impl ClocksManager {
        pub fn new(clocks: CLOCKS) -> ClocksManager {
            ClocksManager {
                system_clock: SystemClock,
                peripheral_clock: PeripheralClock,
                reference_clock: ReferenceClock,
                usb_clock: ClockSource,
                adc_clock: ClockSource,
                gpio_output0_clock: ClockSource,
                gpio_output1_clock: ClockSource,
                gpio_output2_clock: ClockSource,
                gpio_output3_clock: ClockSource,
                rtc_clock: ClockSource,
            }
        }
    }

    pub struct ClockSource;
    pub struct StoppableClock;
    pub struct SystemClock;

    impl Into<HertzU32> for SystemClock {
        fn into(self) -> HertzU32 {
            HertzU32::millis(1)
        }
    }

    pub struct PeripheralClock;
    pub struct ReferenceClock;

    impl Clock for SystemClock {}
    impl Clock for PeripheralClock {}
    impl Clock for ReferenceClock {}
    impl Clock for ClockSource {}
}

pub use sio::Sio;

pub mod multicore {
    use crate::re_exports::bsp::hal::sio::SioFifo;
    use crate::re_exports::bsp::pac::{PPB, PSM};

    pub struct Core;

    impl Core {
        pub fn spawn<const N: usize, F: FnOnce() + Send + 'static>(
            &mut self,
            stack: Stack<N>,
            f: F,
        ) {
        }
    }

    pub struct Multicore {
        cores: [Core; 2],
    }

    impl Multicore {
        pub fn new(psm: &mut PSM, ppb: &mut PPB, fifo: &mut SioFifo) -> Multicore {
            Multicore {
                cores: [Core, Core],
            }
        }

        pub fn cores(&mut self) -> &mut [Core] {
            &mut self.cores
        }
    }

    pub struct Stack<const N: usize> {
        pub size: usize,
    }

    impl<const SIZE: usize> Stack<SIZE> {
        pub const fn new() -> Stack<SIZE> {
            Stack { size: SIZE }
        }

        pub fn take(self) -> Option<Stack<SIZE>> {
            Some(self)
        }
    }
}

pub mod rosc {
    use fugit::HertzU32;
    pub struct Enabled;
    pub struct RingOscillator<T>(pub T);
    impl<T> RingOscillator<T> {
        pub fn new(r: T) -> RingOscillator<T> {
            RingOscillator(r)
        }
        pub fn initialize_with_freq(self, freq: HertzU32) -> RingOscillator<Enabled> {
            RingOscillator::<Enabled>(Enabled)
        }
        pub fn get_freq(&self) -> HertzU32 {
            HertzU32::millis(1)
        }
        pub fn disable(&self) {}

        pub unsafe fn dormant(&self) {}

        pub fn free(self) -> Self {
            self
        }
    }
}
pub struct CrystalOcillator;

impl CrystalOcillator {
    pub fn disable(&self) {}
}
pub mod xosc {
    use crate::re_exports::bsp::hal::CrystalOcillator;

    pub fn setup_xosc_blocking(
        x: crate::re_exports::bsp::pac::XOSC,
        hz: fugit::HertzU32,
    ) -> Result<CrystalOcillator, ()> {
        Ok(CrystalOcillator)
    }
}

pub mod gpio {
    pub enum Interrupt {
        LevelLow,
        EdgeHigh,
    }

    pub mod bank0 {
        use crate::re_exports::bsp::hal::gpio::PinNum;

        pub struct Gpio0;
        pub struct Gpio1;
        //pub struct Gpio2;
        pub struct Gpio3;
        pub struct Gpio5;
        pub struct Gpio6;
        pub struct Gpio7;
        pub struct Gpio8;
        pub struct Gpio9;
        pub struct Gpio10;
        pub struct Gpio11;
        pub struct Gpio12;
        pub struct Gpio13;
        pub struct Gpio14;
        pub struct Gpio15;

        pub struct Gpio18;
        pub struct Gpio19;
        pub struct Gpio20;
        pub struct Gpio21;
        pub struct Gpio22;
        pub struct Gpio23;
        pub struct Gpio24;
        pub struct Gpio25;
        pub struct Gpio26;
        pub struct Gpio27;
        pub struct Gpio28;
        pub struct Gpio29;

        impl PinNum for Gpio0 {}
        impl PinNum for Gpio1 {}
        impl PinNum for Gpio3 {}
        impl PinNum for Gpio5 {}
        impl PinNum for Gpio6 {}
        impl PinNum for Gpio7 {}
        impl PinNum for Gpio8 {}
        impl PinNum for Gpio9 {}
        impl PinNum for Gpio10 {}
        impl PinNum for Gpio11 {}
        impl PinNum for Gpio12 {}
        impl PinNum for Gpio13 {}
        impl PinNum for Gpio14 {}
        impl PinNum for Gpio15 {}

        impl PinNum for Gpio18 {}
        impl PinNum for Gpio19 {}
        impl PinNum for Gpio20 {}
        impl PinNum for Gpio21 {}
        impl PinNum for Gpio22 {}
        impl PinNum for Gpio23 {}
        impl PinNum for Gpio24 {}
        impl PinNum for Gpio25 {}
        impl PinNum for Gpio26 {}
        impl PinNum for Gpio27 {}
        impl PinNum for Gpio28 {}
        impl PinNum for Gpio29 {}
    }

    use crate::re_exports::bsp::hal::gpio::bank0::{
        Gpio0, Gpio1, Gpio3, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9, Gpio10, Gpio11, Gpio12, Gpio13,
        Gpio14, Gpio15, Gpio18, Gpio19, Gpio20, Gpio21, Gpio22, Gpio23, Gpio24, Gpio25, Gpio26,
        Gpio27, Gpio28, Gpio29,
    };
    use crate::re_exports::bsp::hal::sio::GPIO_BANK0;
    use crate::re_exports::bsp::pac::{IO_BANK0, PADS_BANK0, RESETS};
    extern crate std;
    use std::cmp::PartialEq;

    enum PullType {
        PullUp,
        PullDown,
        PullNone,
    }

    enum FunctionType {
        I2C,
        Null,
        SioInput,
        SioOutput,
        Spi,
        Pio,
    }

    pub trait PinFunction {
        fn func_type(&self) -> FunctionType;
        fn from(f: FunctionType) -> Self;
    }
    pub trait PinPullType {
        fn pull_type(&self) -> PullType;
        fn from(p: PullType) -> Self;
    }
    pub trait PinNum {}

    impl PinFunction for FunctionI2C {
        fn func_type(&self) -> FunctionType {
            FunctionType::I2C
        }
        fn from(f: FunctionType) -> FunctionI2C {
            FunctionI2C
        }
    }
    impl PinFunction for FunctionPio0 {
        fn func_type(&self) -> FunctionType {
            FunctionType::Pio
        }
        fn from(f: FunctionType) -> FunctionPio0 {
            FunctionPio0
        }
    }

    impl PinFunction for FunctionPio1 {
        fn func_type(&self) -> FunctionType {
            FunctionType::Pio
        }
        fn from(f: FunctionType) -> FunctionPio1 {
            FunctionPio1
        }
    }
    impl PinFunction for FunctionNull {
        fn func_type(&self) -> FunctionType {
            FunctionType::Null
        }
        fn from(f: FunctionType) -> FunctionNull {
            FunctionNull
        }
    }
    impl PinFunction for FunctionSio<SioInput> {
        fn func_type(&self) -> FunctionType {
            FunctionType::SioInput
        }
        fn from(f: FunctionType) -> FunctionSio<SioInput> {
            FunctionSio::new(SioInput)
        }
    }
    impl PinFunction for FunctionSio<SioOutput> {
        fn func_type(&self) -> FunctionType {
            FunctionType::SioOutput
        }
        fn from(f: FunctionType) -> FunctionSio<SioOutput> {
            FunctionSio::new(SioOutput)
        }
    }
    impl PinFunction for FunctionSpi {
        fn func_type(&self) -> FunctionType {
            FunctionType::Spi
        }
        fn from(f: FunctionType) -> FunctionSpi {
            FunctionSpi
        }
    }
    impl PinPullType for PullDown {
        fn pull_type(&self) -> PullType {
            PullType::PullDown
        }
        fn from(p: PullType) -> PullDown {
            PullDown
        }
    }
    impl PinPullType for PullUp {
        fn pull_type(&self) -> PullType {
            PullType::PullUp
        }
        fn from(p: PullType) -> PullUp {
            PullUp
        }
    }

    impl PinPullType for PullNone {
        fn pull_type(&self) -> PullType {
            PullType::PullNone
        }
        fn from(p: PullType) -> PullNone {
            PullNone
        }
    }

    pub struct FunctionI2C;
    pub struct FunctionNull;
    pub struct FunctionSpi;
    pub struct FunctionPio0;
    pub struct FunctionPio1;
    pub struct FunctionSio<T>(T);
    impl<T> FunctionSio<T> {
        pub fn new(i: T) -> FunctionSio<T> {
            FunctionSio(i)
        }
    }
    pub struct PullDown;
    pub struct PullUp;
    pub struct PullNone;
    pub struct SioInput;
    pub struct SioOutput;
    pub struct Pin<G, F, P>(G, F, P);

    pub struct PinId {
        pub num: u8,
    }

    impl<G: PinNum, F: PinFunction, P: PinPullType> Pin<G, F, P> {
        pub fn id(&self) -> PinId {
            PinId { num: 1 }
        }
        pub fn is_high(&mut self) -> Result<bool, ()> {
            Ok(true)
        }

        pub fn is_low(&mut self) -> Result<bool, ()> {
            Ok(true)
        }

        pub fn reconfigure<F2: PinFunction, P2: PinPullType>(mut self) -> Pin<G, F2, P2> {
            let p = self.2.pull_type();
            let f = self.1.func_type();
            Pin::new(self.0, F2::from(f), P2::from(p))
        }

        pub fn set_interrupt_enabled(&self, level: Interrupt, enabled: bool) {}

        pub fn set_dormant_wake_enabled(&self, level: Interrupt, enabled: bool) {}
        pub fn clear_interrupt(&mut self, level: Interrupt) {}

        pub fn new(pin_num: G, func: F, pull_type: P) -> Pin<G, F, P> {
            Self(pin_num, func, pull_type)
        }

        pub fn set_high(&self) -> Result<(), ()> {
            Ok(())
        }

        pub fn set_low(&self) -> Result<(), ()> {
            Ok(())
        }
    }

    pub struct Pins {
        pub gpio0: Pin<Gpio0, FunctionNull, PullNone>,
        pub gpio1: Pin<Gpio1, FunctionNull, PullNone>,

        pub gpio5: Pin<Gpio5, FunctionI2C, PullDown>,
        pub gpio6: Pin<Gpio6, FunctionI2C, PullDown>,
        pub gpio7: Pin<Gpio7, FunctionI2C, PullDown>,
        pub gpio3: Pin<Gpio3, FunctionSio<SioInput>, PullDown>,
        pub gpio8: Pin<Gpio8, FunctionNull, PullNone>,
        pub gpio9: Pin<Gpio9, FunctionNull, PullNone>,
        pub gpio10: Pin<Gpio10, FunctionNull, PullNone>,
        pub gpio11: Pin<Gpio11, FunctionNull, PullNone>,
        pub gpio12: Pin<Gpio12, FunctionNull, PullNone>,
        pub gpio13: Pin<Gpio13, FunctionNull, PullNone>,
        pub gpio14: Pin<Gpio14, FunctionNull, PullNone>,
        pub gpio15: Pin<Gpio15, FunctionNull, PullNone>,

        pub gpio18: Pin<Gpio18, FunctionNull, PullNone>,
        pub gpio19: Pin<Gpio19, FunctionNull, PullNone>,
        pub gpio20: Pin<Gpio20, FunctionNull, PullNone>,
        pub gpio21: Pin<Gpio21, FunctionNull, PullNone>,
        pub gpio22: Pin<Gpio22, FunctionNull, PullNone>,
        pub gpio23: Pin<Gpio23, FunctionNull, PullNone>,
        pub gpio24: Pin<Gpio24, FunctionNull, PullNone>,
        pub gpio25: Pin<Gpio25, FunctionNull, PullNone>,
        pub gpio26: Pin<Gpio26, FunctionNull, PullNone>,
        pub gpio27: Pin<Gpio27, FunctionNull, PullNone>,
        pub gpio28: Pin<Gpio28, FunctionNull, PullNone>,
        pub gpio29: Pin<Gpio29, FunctionNull, PullNone>,
    }
    impl Pins {
        pub fn new(
            io_bank0: IO_BANK0,
            pads_bank0: PADS_BANK0,
            gpio_bank0: GPIO_BANK0,
            resets: &mut RESETS,
        ) -> Pins {
            Pins {
                gpio0: Pin::new(Gpio0, FunctionNull, PullNone),
                gpio1: Pin::new(Gpio1, FunctionNull, PullNone),
                gpio3: Pin::new(Gpio3, FunctionSio::new(SioInput), PullDown),
                gpio5: Pin::new(Gpio5, FunctionI2C, PullDown),
                gpio6: Pin::new(Gpio6, FunctionI2C, PullDown),
                gpio7: Pin::new(Gpio7, FunctionI2C, PullDown),
                gpio8: Pin::new(Gpio8, FunctionNull, PullNone),
                gpio9: Pin::new(Gpio9, FunctionNull, PullNone),
                gpio10: Pin::new(Gpio10, FunctionNull, PullNone),
                gpio11: Pin::new(Gpio11, FunctionNull, PullNone),
                gpio12: Pin::new(Gpio12, FunctionNull, PullNone),
                gpio13: Pin::new(Gpio13, FunctionNull, PullNone),
                gpio14: Pin::new(Gpio14, FunctionNull, PullNone),
                gpio15: Pin::new(Gpio15, FunctionNull, PullNone),

                gpio18: Pin::new(Gpio18, FunctionNull, PullNone),
                gpio19: Pin::new(Gpio19, FunctionNull, PullNone),
                gpio20: Pin::new(Gpio20, FunctionNull, PullNone),
                gpio21: Pin::new(Gpio21, FunctionNull, PullNone),
                gpio22: Pin::new(Gpio22, FunctionNull, PullNone),
                gpio23: Pin::new(Gpio23, FunctionNull, PullNone),
                gpio24: Pin::new(Gpio24, FunctionNull, PullNone),
                gpio25: Pin::new(Gpio25, FunctionNull, PullNone),
                gpio26: Pin::new(Gpio26, FunctionNull, PullNone),
                gpio27: Pin::new(Gpio27, FunctionNull, PullNone),
                gpio28: Pin::new(Gpio28, FunctionNull, PullNone),
                gpio29: Pin::new(Gpio29, FunctionNull, PullNone),
            }
        }
    }
}

pub mod dma {
    pub struct CH0;
    pub struct CH1;
    pub struct CH2;
    pub struct CH3;
    pub struct CH4;
    pub struct Channels {
        pub ch0: Channel<CH0>,
        pub ch1: Channel<CH1>,
        pub ch2: Channel<CH2>,
        pub ch3: Channel<CH3>,
        pub ch4: Channel<CH4>,
    }

    pub struct Channel<T>(pub T);

    pub mod bidirectional {
        use crate::onboard_flash::{CACHE_READ, FLASH_SPI_HEADER};
        use crate::re_exports::bsp::hal::spi::{Enabled, Spi};
        use crate::re_exports::bsp::pac::SPI1;
        pub struct Transfer<CH1, CH2, SpiPins> {
            ch: (CH1, CH2),
            from: &'static [u8],
            spi: Spi<Enabled, SPI1, SpiPins, 8>,
            to: &'static mut [u8],
        }
        impl<CH1, CH2, SpiPins> Transfer<CH1, CH2, SpiPins> {
            pub fn wait(
                self,
            ) -> (
                (CH1, CH2),
                &'static [u8],
                Spi<Enabled, SPI1, SpiPins, 8>,
                &'static [u8],
            ) {
                if self.from[0] == CACHE_READ {
                    let col_offset_0 = self.from[1] as u16;
                    let col_offset_1 = self.from[2] as u16;

                    let col_offset = ((col_offset_0 & 0x0f) << 8 | col_offset_1) as usize;
                    assert_eq!(self.to.len(), self.from.len());
                    let len = col_offset..col_offset + self.from.len() - FLASH_SPI_HEADER;
                    self.to[FLASH_SPI_HEADER..].copy_from_slice(&self.spi.buffer[len]);
                }

                ((self.ch.0, self.ch.1), self.from, self.spi, self.to)
            }
        }

        pub struct Config<CH1, CH2, SpiPins> {
            ch: (CH1, CH2),
            from: &'static [u8],
            spi: Spi<Enabled, SPI1, SpiPins, 8>,
            to: &'static mut [u8],
        }

        impl<CH1, CH2, SpiPins> Config<CH1, CH2, SpiPins> {
            pub fn new(
                ch: (CH1, CH2),
                from: &'static [u8],
                spi: Spi<Enabled, SPI1, SpiPins, 8>,
                to: &'static mut [u8],
            ) -> Self {
                Self { ch, from, spi, to }
            }

            pub fn start(self) -> Transfer<CH1, CH2, SpiPins> {
                Transfer {
                    ch: self.ch,
                    from: self.from,
                    spi: self.spi,
                    to: self.to,
                }
            }
        }
    }

    pub mod double_buffer {
        pub struct Config<CH1, CH2, T, BUF: 'static> {
            ch: (CH1, CH2),
            rx: T,
            buf: &'static mut BUF,
        }

        impl<CH1, CH2, T, BUF> Config<CH1, CH2, T, BUF> {
            pub fn new(ch: (CH1, CH2), rx: T, buf: &'static mut BUF) -> Self {
                Self { ch, rx, buf }
            }

            pub fn start(self) -> Transfer<(CH1, CH2), &'static mut BUF, T, ()> {
                Transfer((self.ch, self.buf, self.rx, ()))
            }
        }

        pub struct Transfer<CHANNEL, BUF, T, NEXT>((CHANNEL, BUF, T, NEXT));

        impl<CH1, CH2, BUF, T> Transfer<(CH1, CH2), BUF, T, ()> {
            pub fn write_next<NEXT>(self, buf: NEXT) -> Transfer<(CH1, CH2), BUF, T, NEXT> {
                Transfer((self.0.0, self.0.1, self.0.2, buf))
            }
        }

        impl<CH1, CH2, BUF, T, NEXT> Transfer<(CH1, CH2), BUF, T, NEXT> {
            pub fn is_done(&self) -> bool {
                false
            }

            pub fn wait(self) -> (NEXT, Transfer<(CH1, CH2), BUF, T, ()>) {
                (self.0.3, Transfer((self.0.0, self.0.1, self.0.2, ())))
            }
        }
    }

    pub mod single_buffer {
        use crate::ext_spi_transfers::SpiEnabledPeripheral;
        use crate::re_exports::bsp::hal::dma::{CH0, Channel};
        use crate::re_exports::bsp::hal::pio::{SM0, Tx};
        use crate::re_exports::bsp::pac::PIO0;
        use crate::tests::stubs::fake_shared_spi::{read_from_rpi, write_to_rpi};

        pub trait TransferExt {
            type Channel;
            type RX;
            type TX;

            fn wait(self) -> (Self::Channel, Self::RX, Self::TX);
        }

        pub struct Config<CH1, RX, TX> {
            ch: CH1,
            tx: TX,
            rx: RX,
        }

        impl<CH1, RX, TX> Config<CH1, RX, TX> {
            pub fn new(ch: CH1, rx: RX, tx: TX) -> Self {
                Self { ch, tx, rx }
            }

            pub fn bswap(&mut self, enabled: bool) {}

            pub fn start(self) -> Transfer<CH1, RX, TX> {
                Transfer {
                    ch: self.ch,
                    tx: self.tx,
                    rx: self.rx,
                }
            }
        }

        pub struct Transfer<CHANNEL, RX, TX> {
            ch: CHANNEL,
            rx: RX,
            tx: TX,
        }

        // Implement for PIO DMA transfers (&'static [u32] -> Tx)
        impl TransferExt for Transfer<Channel<CH0>, &'static [u32], Tx<(PIO0, SM0)>> {
            type Channel = Channel<CH0>;
            type RX = &'static [u32];
            type TX = Tx<(PIO0, SM0)>;

            fn wait(self) -> (Self::Channel, Self::RX, Self::TX) {
                // Custom logic for PIO transfers
                // Add any PIO-specific logic here if needed
                (self.ch, self.rx, self.tx)
            }
        }

        // Implement for SPI write transfers (buffer -> SPI)
        impl<const N: usize> TransferExt
            for Transfer<Channel<CH0>, &'static mut [u8; N], SpiEnabledPeripheral>
        {
            type Channel = Channel<CH0>;
            type RX = &'static mut [u8; N];
            type TX = SpiEnabledPeripheral;

            fn wait(self) -> (Self::Channel, Self::RX, Self::TX) {
                let _ = write_to_rpi(self.rx);
                (self.ch, self.rx, self.tx)
            }
        }

        // Implement for SPI read transfers (SPI -> buffer)
        impl<const N: usize> TransferExt
            for Transfer<Channel<CH0>, SpiEnabledPeripheral, &'static mut [u8; N]>
        {
            type Channel = Channel<CH0>;
            type RX = SpiEnabledPeripheral;
            type TX = &'static mut [u8; N];

            fn wait(self) -> (Self::Channel, Self::RX, Self::TX) {
                let _ = read_from_rpi(self.tx);
                (self.ch, self.rx, self.tx)
            }
        }
    }

    pub trait DMAExt {
        /// Splits the DMA unit into its individual channels.
        fn split(self, resets: &mut crate::re_exports::bsp::pac::RESETS) -> Channels;
    }
}

pub mod watchdog {
    use crate::re_exports::bsp::pac::WATCHDOG;
    use fugit::Duration;

    pub struct Watchdog;

    impl Watchdog {
        pub fn new(watchdog: WATCHDOG) -> Self {
            Watchdog
        }
        pub fn feed(&self) {}
        pub fn disable(&self) {}

        pub fn start(&self, timeout_ms: Duration<u32, 1, 1000>) {
            //info!("set {timeout_ms} timeout");
        }
        pub fn pause_on_debug(&mut self, pause: bool) {}
        pub fn enable_tick_generation(&mut self, freq: u8) {}
    }
}

pub use watchdog::Watchdog;

pub mod pio {
    extern crate std;
    use crate::re_exports::bsp::pac::PIO0;

    pub struct PIO<P: PIOExt>(pub P);
    pub struct SM0;
    pub struct SM1;
    pub struct PIOBuilder;
    pub struct PIO_STUB;
    pub enum PinDir {
        Input,
        Output,
    }

    impl<P> StateMachine<P, Stopped> {
        pub fn set_pindirs(&self, dirs: impl IntoIterator<Item = (u8, PinDir)>) {}
        pub fn start(self) -> StateMachine<P, Running> {
            StateMachine(self.0, Running)
        }
    }

    impl PIO_STUB {
        pub fn in_pin_base(&self, pin_id: u8) -> PIO_STUB {
            PIO_STUB
        }
        pub fn clock_divisor_fixed_point(&self, c: u16, f: u8) -> PIO_STUB {
            PIO_STUB
        }
        pub fn side_set_pin_base(&self, pin_id: u8) -> PIO_STUB {
            PIO_STUB
        }
        pub fn out_pins(self, pin_id: u8, x: i32) -> PIO_STUB {
            PIO_STUB
        }
        pub fn jmp_pin(self, pin_id: u8) -> PIO_STUB {
            PIO_STUB
        }
        pub fn out_shift_direction(self, direction: ShiftDirection) -> PIO_STUB {
            PIO_STUB
        }
        pub fn in_shift_direction(self, direction: ShiftDirection) -> PIO_STUB {
            PIO_STUB
        }
        pub fn push_threshold(self, amount: i32) -> PIO_STUB {
            PIO_STUB
        }
        pub fn pull_threshold(self, amount: i32) -> PIO_STUB {
            PIO_STUB
        }
        pub fn autopush(self, enabled: bool) -> PIO_STUB {
            PIO_STUB
        }
        pub fn autopull(self, enabled: bool) -> PIO_STUB {
            PIO_STUB
        }
        pub fn build<P, SM>(
            self,
            uninit_state_machine: UninitStateMachine<(P, SM)>,
        ) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
            (
                StateMachine(uninit_state_machine.0, Stopped),
                Rx(core::marker::PhantomData),
                Tx(core::marker::PhantomData),
            )
        }
    }

    impl PIOBuilder {
        pub(crate) fn from_installed_program<P: PIOExt>(p0: InstalledProgram<P>) -> PIO_STUB {
            PIO_STUB
        }
    }

    pub struct Running;
    pub struct Stopped;
    pub struct Rx<T>(core::marker::PhantomData<T>);
    pub enum ShiftDirection {
        Left,
        Right,
    }
    pub struct StateMachine<A, B>(A, B);

    impl<A> StateMachine<A, Running> {
        pub fn uninit(
            self,
            rx: Rx<A>,
            tx: Tx<A>,
        ) -> (UninitStateMachine<A>, InstalledProgram<PIO0>) {
            (
                UninitStateMachine(self.0),
                InstalledProgram(core::marker::PhantomData),
            )
        }

        pub fn stop(&self) {}

        pub fn clock_divisor_fixed_point(&self, c: u16, f: u8) {}
    }

    pub struct Tx<T>(core::marker::PhantomData<T>);
    pub struct UninitStateMachine<T>(pub T);
    pub struct InstalledProgram<P>(core::marker::PhantomData<P>);

    impl<P: PIOExt> InstalledProgram<P> {}

    impl<P: PIOExt> PIO<P> {
        pub(crate) fn install<T>(&mut self, program: T) -> Result<InstalledProgram<P>, ()> {
            Ok(InstalledProgram(core::marker::PhantomData))
        }

        pub fn uninstall(&mut self, program: InstalledProgram<PIO0>) {}
    }

    pub trait PIOExt: Sized + Copy {
        fn split(
            self,
            resets: &mut crate::re_exports::bsp::pac::RESETS,
        ) -> (
            PIO<Self>,
            UninitStateMachine<(Self, SM0)>,
            UninitStateMachine<(Self, SM1)>,
            (),
            (),
        ) {
            (
                PIO(self),
                UninitStateMachine((self, SM0)),
                UninitStateMachine((self, SM1)),
                (),
                (),
            )
        }
    }
}

pub use i2c::I2C;
pub use spi::Spi;
