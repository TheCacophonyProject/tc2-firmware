pub mod pac {
    use crate::re_exports::bsp::hal::dma::{CH0, CH1, CH2, CH3, CH4, Channel, Channels, DMAExt};
    use crate::re_exports::bsp::hal::pio::PIOExt;

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

        pub fn ch(&self, ch_num: usize) -> DMA {
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
        fn split(self, resets: &mut RESETS) -> Channels {
            Channels {
                ch0: Channel(CH0),
                ch1: Channel(CH1),
                ch2: Channel(CH2),
                ch3: Channel(CH3),
                ch4: Channel(CH4),
            }
        }
    }

    pub enum Interrupt {
        IO_IRQ_BANK0,
        TIMER_IRQ_0,
    }

    pub struct NVIC;

    impl NVIC {
        pub fn unmask(int: Interrupt) {}
        pub fn mask(int: Interrupt) {}
    }

    #[derive(Copy, Clone)]
    pub struct PIO0;

    #[derive(Copy, Clone)]
    pub struct PIO1;

    impl PIOExt for PIO0 {}
    impl PIOExt for PIO1 {}

    pub struct RESETS;
    pub struct SPI0;
    pub struct SPI1;

    pub struct CLOCKS;

    pub struct RW_STUB;
    pub struct KHZ_STUB;
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
            1
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
            // FIXME: Might need to fake some logic here to drive things forwards.
            None
        }

        pub fn bits(&mut self, b: u32) {}
    }

    impl CLOCKS {
        pub fn write<F>(&self, f: F) -> u32
        where
            F: FnOnce(&mut CLOCKS) -> u32,
        {
            f(&mut CLOCKS)
        }
        pub fn bits(&mut self, b: u32) -> u32 {
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

    pub struct XOSC;
    pub struct ROSC;
    pub struct WATCHDOG;
    pub struct TIMER;
    pub struct IO_BANK0;
    pub struct PADS_BANK0;
    pub struct I2C0;
    pub struct I2C1;
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

    // impl PIO0 {
    //     pub fn split(
    //         &mut self,
    //         resets: &mut RESETS,
    //     ) -> (PIO<PIO0>, UninitStateMachine<(PIO0, SM0)>, (), (), ()) {
    //         (PIO(PIO0), UninitStateMachine((PIO0, SM0)), (), (), ())
    //     }
    // }
    //
    // impl PIO1 {
    //     pub fn split(
    //         &mut self,
    //         resets: &mut RESETS,
    //     ) -> (PIO<PIO1>, (), UninitStateMachine<(PIO1, SM1)>, (), ()) {
    //         (PIO(PIO1), (), UninitStateMachine((PIO1, SM1)), (), ())
    //     }
    // }

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
}

pub mod hal {
    extern crate std;
    use crate::attiny_rtc_i2c::{
        ATTINY_ADDRESS, ATTINY_REG_CAMERA_STATE, ATTINY_REG_KEEP_ALIVE,
        ATTINY_REG_RP2040_PI_POWER_CTRL, ATTINY_REG_TC2_AGENT_STATE, ATTINY_REG_VERSION,
        CRC_AUG_CCITT, CameraState, EEPROM_I2C_ADDRESS, I2CConfig, RTC_ADDRESS,
        RTC_REG_ALARM_CONTROL, RTC_REG_ALARM_MINUTES, RTC_REG_DATETIME_SECONDS, tc2_agent_state,
    };
    use crate::re_exports::bsp::hal::clocks::{ClocksManager, SystemClock};
    use crate::re_exports::bsp::hal::gpio::bank0::{Gpio6, Gpio7, Gpio24, Gpio25};
    use crate::re_exports::bsp::hal::gpio::{FunctionI2C, Pin, PullUp};
    use crate::re_exports::bsp::pac::{I2C0, I2C1, RESETS, TIMER};
    use byteorder::{BigEndian, ByteOrder};
    use chrono::{Datelike, Timelike};
    use crc::Crc;
    use fugit::{HertzU32, Rate, TimerInstantU64};
    use log::debug;

    pub type Instant = TimerInstantU64<1_000_000>;

    pub mod i2c {
        #[non_exhaustive]
        #[derive(Debug)]
        pub enum Error {
            /// I2C abort with error
            Abort(u32),
            /// User passed in a read buffer that was 0 length
            ///
            /// This is a limitation of the RP2040 I2C peripheral.
            /// If the slave ACKs its address, the I2C peripheral must read
            /// at least one byte before sending the STOP condition.
            InvalidReadBufferLength,
            /// User passed in a write buffer that was 0 length
            ///
            /// This is a limitation of the RP2040 I2C peripheral.
            /// If the slave ACKs its address, the I2C peripheral must write
            /// at least one byte before sending the STOP condition.
            InvalidWriteBufferLength,
            /// Target i2c address is out of range
            AddressOutOfRange(u16),
            /// Target i2c address is reserved
            AddressReserved(u16),
        }
    }

    pub mod timer {
        pub use crate::re_exports::bsp::hal::Instant;
        use fugit::MicrosDurationU32;
        pub trait Alarm {
            fn schedule(&self, when: MicrosDurationU32) -> Result<(), ()> {
                Ok(())
            }

            fn enable_interrupt(&self) {}
            fn finished(&self) -> bool {
                true
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
            HertzU32::millis(1)
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

    pub mod sio {
        use crate::cptv_encoder::{FRAME_HEIGHT, FRAME_WIDTH};
        use crate::frame_processing::FrameBuffer;
        use crate::tests::test_state::test_global_state::CURRENT_TIME;
        use byteorder::{ByteOrder, LittleEndian};
        use chrono::Duration;
        use codec::decode::{CptvDecoder, CptvFrame};
        extern crate std;
        use std::collections::VecDeque;
        use std::fs::File;
        use std::path::Path;

        pub struct SioFifo {
            inner: VecDeque<u32>,
            decoder: CptvDecoder<File>,
            last_frame: Option<CptvFrame>,
            selected_buffer: u8,
            frame_num: u32,
        }
        impl SioFifo {
            pub fn read_blocking(&mut self) -> u32 {
                self.inner.pop_front().unwrap_or(0)
            }

            pub fn read(&self) -> Option<u32> {
                None
            }

            pub fn copy_last_frame(&mut self, frame_buffer: &mut Option<&'static mut FrameBuffer>) {
                if let Some(frame_buffer) = frame_buffer {
                    if let Some(frame) = &self.last_frame {
                        let header = frame_buffer.frame_data_as_u8_slice_mut();
                        // Telemetry revision
                        LittleEndian::write_u16(&mut header[0..2], 14);
                        // Frame number
                        LittleEndian::write_u32(&mut header[40..44], self.frame_num);

                        // Msec on:
                        let msec_on = self.frame_num * 115;
                        LittleEndian::write_u32(&mut header[2..6], msec_on);

                        // Time at last FFC:
                        LittleEndian::write_u32(&mut header[60..64], frame.last_ffc_time);

                        // Always just say FFC is done
                        let status_bits = LittleEndian::write_u32(&mut header[6..10], 3 << 4);
                        let fpa_temp_kelvin_x_100 = LittleEndian::write_u16(
                            &mut header[48..=49],
                            ((frame.frame_temp_c * 100.0) + 273.15) as u16,
                        );
                        let fpa_temp_kelvin_x_100_at_last_ffc = LittleEndian::write_u16(
                            &mut header[58..=59],
                            ((frame.last_ffc_temp_c * 100.0) + 273.15) as u16,
                        );
                        header[640..][0..(FRAME_WIDTH * FRAME_HEIGHT) * 2]
                            .copy_from_slice(frame.image_data.as_slice());
                        self.frame_num += 1;
                    }
                }
            }
            pub fn next_frame(&mut self) {
                // Advance time 1 frame (or maybe that already happens for us)
                // if the current time is a time to trigger recording, advance cptv frame, otherwise
                // keep the same frame.
                *CURRENT_TIME.lock().unwrap() += Duration::milliseconds(115);
                //if *RTC_ALARM_STATE.lock().unwrap()

                if self.last_frame.is_none() {
                    if let Ok(frame) = self.decoder.next_frame() {
                        self.last_frame = Some(frame.clone());
                    }
                }
                // TODO, if the current timestamp is a recording time, advance cptv frame, otherwise use static frame.

                self.inner.push_back(0xae);
                if self.selected_buffer == 1 {
                    self.selected_buffer = 0;
                } else {
                    self.selected_buffer = 1;
                }
                self.inner.push_back(self.selected_buffer as u32); // frame buffer 1
            }

            pub fn write_blocking(&mut self, value: u32) {
                match value {
                    0xec => {
                        //Core0Task::ReadyToReceiveLeptonConfig
                        self.inner.push_back(0x0c);
                        self.inner.push_back(4);
                        self.inner.push_back(2);
                        self.inner.push_back(1234567);
                        let main_lepton_firmware = LittleEndian::read_u32(&[1, 2, 3, 0]);
                        let dsp_lepton_firmware = LittleEndian::read_u32(&[4, 5, 6, 0]);
                        self.inner.push_back(main_lepton_firmware);
                        self.inner.push_back(dsp_lepton_firmware);
                    }
                    0xdb => {
                        // Tell other imaginary core we're ready

                        //ReceiveFrame
                        // self.inner.push_back(0xae);
                        // self.inner.push_back(1); // frame buffer 1
                    }
                    _ => panic!("Unknown value to write to fifo:"),
                }
            }
            pub fn write(&mut self, value: u32) {}
        }

        pub struct GPIO_BANK0;
        pub struct Sio {
            pub gpio_bank0: GPIO_BANK0,
            pub fifo: SioFifo,
        }

        impl Sio {
            pub fn new(sio: crate::re_exports::bsp::pac::SIO) -> Sio {
                Sio {
                    gpio_bank0: GPIO_BANK0,
                    fifo: SioFifo {
                        inner: VecDeque::new(),
                        decoder: CptvDecoder::<File>::from_path(&Path::new(
                            "./recording-2334134-29-05-2025--07-14-59.cptv",
                        ))
                        .unwrap(),
                        last_frame: None,
                        selected_buffer: 0,
                        frame_num: 0,
                    },
                }
            }
        }
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

        // impl<G, F, P> embedded_hal::digital::ErrorType for Pin<G, F, P> {
        //     type Error = ();
        // }

        // impl<G, F, P> embedded_hal::digital::InputPin for Pin<G, F, P> {
        //     fn is_high(&mut self) -> Result<bool, Self::Error> {
        //         Ok(true)
        //     }
        //
        //     fn is_low(&mut self) -> Result<bool, Self::Error> {
        //         Ok(true)
        //     }
        // }

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
                    true
                }

                pub fn wait(self) -> (NEXT, Transfer<(CH1, CH2), BUF, T, ()>) {
                    (self.0.3, Transfer((self.0.0, self.0.1, self.0.2, ())))
                }
            }
        }

        pub mod single_buffer {
            pub struct Config<CH1, BUF, TX> {
                ch: CH1,
                spi: TX,
                to: BUF,
            }

            impl<CH1, BUF, TX> Config<CH1, BUF, TX> {
                pub fn new(ch: CH1, to: BUF, spi: TX) -> Self {
                    Self { ch, spi, to }
                }

                pub fn bswap(&mut self, enabled: bool) {}

                pub fn start(self) -> Transfer<CH1, BUF, TX> {
                    Transfer {
                        ch: self.ch,
                        spi: self.spi,
                        to: self.to,
                    }
                }
            }

            pub struct Transfer<CHANNEL, BUF, TX> {
                ch: CHANNEL,
                to: BUF,
                spi: TX,
            }

            impl<CHANNEL, BUF, TX> Transfer<CHANNEL, BUF, TX> {
                pub fn wait(self) -> (CHANNEL, BUF, TX) {
                    (self.ch, self.to, self.spi)
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

    pub struct I2C<A, B>((A, B));

    #[derive(Debug)]
    pub struct I2cError;
    impl I2cError {
        pub fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    impl<A, B> I2C<I2C0, (A, B)> {
        pub(crate) fn i2c0(
            p0: I2C0,
            p1: A,
            p2: B,
            p3: Rate<u32, 1, 1>,
            p4: &mut RESETS,
            p5: HertzU32,
        ) -> I2C<I2C0, (A, B)> {
            I2C((p0, (p1, p2)))
        }

        pub fn write(&mut self, address: u8, payload: &[u8]) -> Result<(), i2c::Error> {
            Ok(())
        }

        pub fn write_read(
            &mut self,
            address: u8,
            payload: &[u8],
            dst: &mut [u8],
        ) -> Result<(), i2c::Error> {
            Ok(())
        }

        pub fn tx_fifo_used(&self) -> u32 {
            0
        }
    }

    impl<A, B> I2C<I2C1, (A, B)> {
        pub(crate) fn i2c1(
            p0: I2C1,
            p1: A,
            p2: B,
            p3: Rate<u32, 1, 1>,
            p4: &mut RESETS,
            p5: HertzU32,
        ) -> I2C<I2C1, (A, B)> {
            I2C((p0, (p1, p2)))
        }

        pub fn write(&mut self, address: u8, payload: &[u8]) -> Result<(), I2cError> {
            // NOTE: In between writes we might want to advance the state of the external simulated devices.

            // We might actually want to write a state to some external device (rpi power on state)
            match address {
                ATTINY_ADDRESS => match payload[0] {
                    ATTINY_REG_KEEP_ALIVE => match payload[1] {
                        0x01 => {
                            // FIXME: Make this be time from our synced clock.
                            *ATTINY_KEEP_ALIVE.lock().unwrap() = std::time::Instant::now();
                        }
                        _ => panic!("Unsupported attiny keep alive 0x{:02x?}", payload[1]),
                    },
                    ATTINY_REG_VERSION => {
                        // nop
                    }
                    ATTINY_REG_CAMERA_STATE => {
                        // nop
                    }
                    ATTINY_REG_RP2040_PI_POWER_CTRL => {
                        // Set power control
                        *ATTINY_POWER_CTRL_STATE.lock().unwrap() = payload[1];
                        match payload[1] {
                            0x00 => {
                                // Tell rPi it can shutdown.
                                let mut camera_state = CAMERA_STATE.lock().unwrap();
                                if *camera_state == CameraState::PoweredOn {
                                    *camera_state = CameraState::PoweringOff;
                                }
                            }
                            0x01 => {
                                // Tell rPi to wake up.
                                let mut camera_state = CAMERA_STATE.lock().unwrap();
                                if *camera_state == CameraState::PoweredOff {
                                    *camera_state = CameraState::PoweringOn;
                                }
                            }
                            0x02 => {
                                // Tell attiny to shut US down.  Can we simulate this?
                                // Maybe it's the same as a restart, with the time advanced?
                                //*CAMERA_STATE.lock().unwrap() = CameraState::PoweringOf;
                            }
                            _ => panic!(
                                "Unsupported attiny power control state 0x{:02x?}",
                                payload[1]
                            ),
                        }
                    }
                    ATTINY_REG_TC2_AGENT_STATE => {
                        *TC2_AGENT_STATE.lock().unwrap() = payload[1].into();
                    }
                    _ => panic!("Unsupported attiny command 0x{:02x?}", payload[0]),
                },
                RTC_ADDRESS => match payload[0] {
                    RTC_REG_ALARM_MINUTES => {
                        let mut alarm = RTC_ALARM_STATE.lock().unwrap();
                        alarm.minutes = payload[1];
                        alarm.hours = payload[2];
                        alarm.day = payload[3];
                        alarm.weekday_alarm_mode = payload[4];
                    }
                    RTC_REG_ALARM_CONTROL => {
                        let mut alarm = RTC_ALARM_STATE.lock().unwrap();
                        alarm.enabled = payload[1];
                    }
                    _ => panic!("Unsupported RTC command"),
                },
                _ => panic!("Unsupported i2c peripheral 0x{:02x?}", address),
            }
            Ok(())
        }

        pub fn write_read(
            &mut self,
            address: u8,
            request: &[u8],
            payload: &mut [u8],
        ) -> Result<(), I2cError> {
            // Maybe time, camera state etc can be mutable static vars that we can changes from outside?
            // Advance external CAMERA_STATE
            match address {
                EEPROM_I2C_ADDRESS => match request[0] {
                    _ => panic!("Unsupported EEPROM write: {:02x?}", request[0]),
                },
                RTC_ADDRESS => {
                    let current_time = CURRENT_TIME.lock().unwrap();
                    match request[0] {
                        RTC_REG_DATETIME_SECONDS => {
                            // FIXME: If we want to compromise time integrity:
                            // if payload[0] & 0b1000_0000 == 0b1000_0000 {
                            //     // Time integrity compromised
                            //     return Err("Time integrity compromised");
                            // }
                            let seconds = current_time.second() as u8;
                            let minutes = current_time.minute() as u8;
                            let hours = current_time.hour() as u8;
                            let day = current_time.day() as u8;
                            let month = current_time.month() as u8;
                            let year = (current_time.year() - 2000) as u8;

                            // info!("Setting RTC to: {:?} ", current_time);
                            // info!("Seconds: {:?} ", seconds);
                            // info!("Minutes: {:?} ", minutes);
                            // info!("Hours: {:?} ", hours);
                            // info!("Day: {:?} ", day);
                            // info!("Month: {:?} ", month);
                            // info!("Year: {:?} ", year);

                            let year = crate::attiny_rtc_i2c::encode_bcd(year);
                            let month = crate::attiny_rtc_i2c::encode_bcd(month);
                            // let weekday = decode_bcd(data[4] & 0x07);
                            let day = crate::attiny_rtc_i2c::encode_bcd(day);
                            let hours = crate::attiny_rtc_i2c::encode_bcd(hours);
                            let minutes = crate::attiny_rtc_i2c::encode_bcd(minutes);
                            let seconds = crate::attiny_rtc_i2c::encode_bcd(seconds);
                            payload[0] = seconds;
                            payload[1] = minutes;
                            payload[2] = hours;
                            payload[3] = day;
                            payload[5] = month;
                            payload[6] = year;
                        }
                        RTC_REG_ALARM_CONTROL => {
                            let alarm = RTC_ALARM_STATE.lock().unwrap();
                            payload[0] = alarm.enabled;
                            debug!(
                                "RTC alarm enabled: {:?}",
                                alarm.enabled & 0b0000_0010 == 0b0000_0010
                            );
                        }
                        RTC_REG_ALARM_MINUTES => {
                            let alarm = RTC_ALARM_STATE.lock().unwrap();
                            payload[0] = alarm.minutes;
                            payload[1] = alarm.hours;
                            payload[2] = alarm.day;
                            payload[3] = alarm.weekday_alarm_mode;
                        }
                        _ => {
                            panic!("Unsupported RTC write: {:02x?}", request);
                        }
                    }
                }
                ATTINY_ADDRESS => match request[0] {
                    ATTINY_REG_KEEP_ALIVE => {
                        payload[0] = 0x01; // Always return that the keep alive was set.
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    ATTINY_REG_VERSION => {
                        let attiny_firmware_version = ATTINY_FIRMWARE_VERSION.lock().unwrap();
                        payload[0] = *attiny_firmware_version; // Set ATTINY FIRMWARE_VERSION to 1
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    ATTINY_REG_CAMERA_STATE => {
                        // Write the camera state:
                        {
                            let camera_state = CAMERA_STATE.lock().unwrap();
                            payload[0] = *camera_state as u8;
                            let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                            BigEndian::write_u16(&mut payload[1..=2], crc);
                        }
                        // Maybe advance the simulation?
                        {
                            let mut camera_state = CAMERA_STATE.lock().unwrap();
                            debug!("Got camera state {:?}", camera_state);
                            if *camera_state == CameraState::PoweringOn {
                                *camera_state = CameraState::PoweredOn;
                            }
                            if *camera_state == CameraState::PoweringOff {
                                *camera_state = CameraState::PoweredOff;
                                TC2_AGENT_STATE
                                    .lock()
                                    .unwrap()
                                    .unset_flag(tc2_agent_state::READY);
                            }
                        }
                        {
                            let camera_state = CAMERA_STATE.lock().unwrap();
                            debug!("Got updated camera state {:?}", camera_state);
                        }
                    }
                    ATTINY_REG_RP2040_PI_POWER_CTRL => {
                        let power_state = ATTINY_POWER_CTRL_STATE.lock().unwrap();
                        payload[0] = *power_state;
                        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                        BigEndian::write_u16(&mut payload[1..=2], crc);
                    }
                    ATTINY_REG_TC2_AGENT_STATE => {
                        // Once we're powered on, we care about the tc2-agent state.
                        {
                            let agent_state = TC2_AGENT_STATE.lock().unwrap();
                            payload[0] = u8::from(*agent_state);
                            let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&[payload[0]]);
                            BigEndian::write_u16(&mut payload[1..=2], crc);
                        }
                        {
                            //Advance state
                            let mut agent_state = TC2_AGENT_STATE.lock().unwrap();
                            debug!("Got tc2-agent state {}", agent_state);
                            if !agent_state.flag_is_set(tc2_agent_state::READY) {
                                agent_state.set_flag(tc2_agent_state::READY);
                            }
                        }
                    }
                    _ => panic!("Unsupported attiny command 0x{:02x?}", request[0]),
                },
                _ => panic!("Unsupported i2c peripheral 0x{:02x?}", address),
            }
            Ok(())
        }
        pub fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError> {
            Ok(())
        }

        pub fn tx_fifo_used(&self) -> u32 {
            0
        }
    }

    pub mod spi {
        extern crate std;
        use crate::onboard_flash::{
            BLOCK_ERASE, BlockIndex, CACHE_READ, FEATURE_STATUS, FLASH_SPI_HEADER, GET_FEATURES,
            PAGE_READ, PROGRAM_EXECUTE, PROGRAM_LOAD, PageIndex, RESET, SET_FEATURES, WRITE_ENABLE,
        };
        use crate::re_exports::bsp::pac::RESETS;
        use crate::tests::test_state::test_global_state::{
            ECC_ERROR_ADDRESSES, FLASH_BACKING_STORAGE,
        };
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
                resets: &RESETS,
                freq: HertzU32,
                p_freq: HertzU32,
                mode: embedded_hal::spi::Mode,
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
                resets: &RESETS,
                mode: embedded_hal::spi::Mode,
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
                resets: &RESETS,
                freq: HertzU32,
                p_freq: HertzU32,
                mode: embedded_hal::spi::Mode,
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
                                let ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                                if ecc_error_addresses.contains(&self.last_read_address) {
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
                        let (block_index, page_index) = Self::get_block_and_page_from_address([
                            address_0, address_1, address_2,
                        ]);

                        // NOTE: was 2112 instead of buffer.len()
                        let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                        storage[block_index as usize].inner[page_index as usize].inner
                            [self.last_page_offset..self.last_page_offset + self.buffer.len()]
                            .copy_from_slice(&self.buffer);
                    }
                    PAGE_READ => {
                        let address_0 = bytes[1];
                        let address_1 = bytes[2];
                        let address_2 = bytes[3];
                        let (block_index, page_index) = Self::get_block_and_page_from_address([
                            address_0, address_1, address_2,
                        ]);
                        self.last_read_address = (block_index, page_index);
                        self.buffer.drain(..);
                        let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                        self.buffer.extend_from_slice(
                            &storage[block_index as usize].inner[page_index as usize].inner,
                        );

                        let ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                        if ecc_error_addresses.contains(&self.last_read_address) {
                            //println!("Corrupting buffer");
                            self.buffer[1] = 0x42;
                        }
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
                        let (block_index, _page_index) = Self::get_block_and_page_from_address([
                            address_0, address_1, address_2,
                        ]);
                        let mut ecc_error_addresses = ECC_ERROR_ADDRESSES.lock().unwrap();
                        ecc_error_addresses.retain(|(b, p)| *b != block_index);
                        self.last_read_address = (0, 0);
                        let mut storage = FLASH_BACKING_STORAGE.lock().unwrap();
                        for page in &mut storage[block_index as usize].inner {
                            page.inner.fill(0xff);
                        }
                    }
                    _ => panic!("Unhandled command 0x{:02x?}", bytes[0]),
                }
                Ok(())
            }
        }
    }
    use crate::tests::test_state::test_global_state::{
        ATTINY_FIRMWARE_VERSION, ATTINY_KEEP_ALIVE, ATTINY_POWER_CTRL_STATE, CAMERA_STATE,
        CURRENT_TIME, RTC_ALARM_STATE, TC2_AGENT_STATE,
    };
    pub use spi::Spi;
}

pub fn entry() {}
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000u32;
