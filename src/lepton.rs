use crate::bsp;
use crate::bsp::hal::gpio::FunctionSpi;
use crate::bsp::hal::gpio::bank0::{Gpio19, Gpio24, Gpio25, Gpio26, Gpio27, Gpio28, Gpio29};
use crate::bsp::hal::spi::Enabled;
use crate::bsp::hal::{I2C as I2CInterface, Spi};
use crate::bsp::pac::{RESETS, SPI0};
use crate::bsp::{hal::gpio::Pin, pac::I2C0};
use crate::lepton_task::LEPTON_SPI_CLOCK_FREQ;
use crate::utils::any_as_u8_slice;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use core::convert::Infallible;
use core::mem;
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_spi_Transfer};
use cortex_m::{delay::Delay, prelude::_embedded_hal_blocking_i2c_WriteRead};
use defmt::{Format, trace};
use defmt::{error, info, panic, warn};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::MODE_3;
use fugit::{HertzU32, RateExtU32};
use rp2040_hal::gpio::bank0::{Gpio18, Gpio20, Gpio21, Gpio22, Gpio23};
use rp2040_hal::gpio::{
    FunctionI2C, FunctionSio, Interrupt, PullDown, PullNone, PullUp, SioInput, SioOutput,
};
use rp2040_hal::i2c::Error;

#[allow(clippy::unusual_byte_groupings)]
pub enum LeptonCommandType {
    Get = 0b0000_0000_0000_00_00,
    Set = 0b0000_0000_0000_00_01,
    Run = 0b0000_0000_0000_00_10,
}

pub type LeptonRegister = [u8; 2];
pub type LeptonSynthesizedCommand = ([u8; 2], u16);
pub type LeptonCommand = (u16, u16);
const LEPTON_BOOTED: u16 = 0b100; // If the camera successfully boots up, this bit is set to 1. If this bit is 0, then the camera has not booted. A host can monitor this bit to learn when the camera has booted.
const LEPTON_BUSY: u16 = 0b001;
const LEPTON_BOOT_MODE: u16 = 0b010; // For normal operation, this bit will be set to 1, indicating successful boot from internal ROM.
const LEPTON_ADDRESS: u8 = 0x2a;

const LEPTON_POWER_ON_REGISTER: LeptonRegister = lepton_register_val(0x0000);
const LEPTON_STATUS_REGISTER: LeptonRegister = lepton_register_val(0x0002);
const LEPTON_COMMAND_ID_REGISTER: LeptonRegister = lepton_register_val(0x0004);
const LEPTON_DATA_LENGTH_REGISTER: LeptonRegister = lepton_register_val(0x0006);
const LEPTON_DATA_0_REGISTER: LeptonRegister = lepton_register_val(0x0008);

const LEPTON_COMMAND_OEM_BIT: u16 = 0b0100_0000_0000_0000;

const LEPTON_SUB_SYSTEM_VID: u16 = 0b0000_0011_0000_0000;
const LEPTON_SUB_SYSTEM_OEM: u16 = 0b0000_1000_0000_0000; // | LEPTON_COMMAND_OEM_BIT; // Requires protection bit set
const LEPTON_SUB_SYSTEM_RAD: u16 = 0x0e00; //0b0000_1110_0000_0000; // | LEPTON_COMMAND_OEM_BIT; // Requires protection bit set
const LEPTON_SUB_SYSTEM_AGC: u16 = 0b0000_0001_0000_0000;
const LEPTON_SUB_SYSTEM_SYS: u16 = 0b0000_0010_0000_0000;

/*
    4.3.1 AGC, VID, and SYS Module Command ID Generation
    AGC, VID, and SYS modules no not require a protection bit to be set before the camera will
    recognize it as a valid command so the protection bit value is 0x0000. For example, the AGC
    Module ID is 0x0100; the ACG Enable command ID Base is 0x00. To retrieve the current AGC
    enable state, issue a Get command specifying command type of 0x0.
    The AGC module protection bit not defined so the value is 0x0000.
    The Command ID is synthesized as follows: Module ID + Command ID Base + Type + Protection Bit value= Command ID.
    So in this example, 0x0100 + 0x00 + 0x0 + 0x0000 = 0x0100 and this is the Get AGC Enable State Command ID.
    To set the AGC enable state to enabled, the command type is 0x1 and
    thus the Command ID is 0x100 + 0x00 + 0x1 + 0x0000 = 0x0101.
*/

const LEPTON_SYS_PING_CAMERA: LeptonCommand = (0x0000, 0);
const LEPTON_SYS_STATUS: LeptonCommand = (0x0004, 4);
const LEPTON_SYS_GET_SERIAL: LeptonCommand = (0x0008, 4);
const LEPTON_SYS_NUM_FRAMES_TO_AVERAGE: LeptonCommand = (0x0024, 2);
const LEPTON_SYS_FFC_STATUS: LeptonCommand = (0x0044, 2);
const LEPTON_SYS_STATS: LeptonCommand = (0x002C, 4);
const LEPTON_SYS_TELEMETRY_ENABLE_STATE: LeptonCommand = (0x0018, 2);
const LEPTON_SYS_TELEMETRY_LOCATION: LeptonCommand = (0x001C, 2);
const LEPTON_SYS_FFC_SHUTTER_MODE: LeptonCommand = (0x03C, 16);
const LEPTON_SYS_RUN_FFC: LeptonCommand = (0x040, 0);
const LEPTON_VID_OUTPUT_FORMAT: LeptonCommand = (0x0030, 2);
const LEPTON_VID_FOCUS_METRIC: LeptonCommand = (0x0018, 2);
const LEPTON_VID_FOCUS_ENABLE_STATE: LeptonCommand = (0x000C, 2);
const LEPTON_AGC_ENABLE_STATE: LeptonCommand = (0x0000, 2);
const LEPTON_OEM_GPIO_MODE: LeptonCommand = (0x0054, 2);
const LEPTON_OEM_GPIO_VSYNC_PHASE_DELAY: LeptonCommand = (0x0058, 2);
const LEPTON_RAD_SPOT_METER_ROI: LeptonCommand = (0x00CC, 4);
const LEPTON_RAD_ENABLE_STATE: LeptonCommand = (0x0010, 2);
const LEPTON_RAD_TLINEAR_ENABLE_STATE: LeptonCommand = (0x00C0, 2);
const LEPTON_OEM_REBOOT: LeptonCommand = (0x0040, 0);
const LEPTON_OEM_BAD_PIXEL_REPLACEMENT: LeptonCommand = (0x006C, 2);
const LEPTON_OEM_CAMERA_SOFTWARE_REVISION: LeptonCommand = (0x0020, 4);
const LEPTON_OEM_TEMPORAL_FILTER: LeptonCommand = (0x0070, 2);
const LEPTON_OEM_COLUMN_NOISE_FILTER: LeptonCommand = (0x0074, 2);
const LEPTON_OEM_PIXEL_NOISE_FILTER: LeptonCommand = (0x0078, 2);
const LEP_OEM_VIDEO_OUTPUT_SOURCE: LeptonCommand = (0x002c, 2);
const LEPTON_OEM_POWER_DOWN: LeptonCommand = (0x0000, 0);
const LEPTON_OEM_VIDEO_OUTPUT_ENABLE: LeptonCommand = (0x0024, 2);

// Should be able to just read into these automatically.
const LEPTON_DATA_1_REGISTER: LeptonRegister = lepton_register_val(0x000A);
// const LEPTON_DATA_2_REGISTER: LeptonRegister = lepton_register_val(0x000C);
// const LEPTON_DATA_3_REGISTER: LeptonRegister = lepton_register_val(0x000E);
// const LEPTON_DATA_4_REGISTER: LeptonRegister = lepton_register_val(0x0010);
// const LEPTON_DATA_5_REGISTER: LeptonRegister = lepton_register_val(0x0012);
// const LEPTON_DATA_6_REGISTER: LeptonRegister = lepton_register_val(0x0014);
// const LEPTON_DATA_7_REGISTER: LeptonRegister = lepton_register_val(0x0016);
// const LEPTON_DATA_8_REGISTER: LeptonRegister = lepton_register_val(0x0018);
// const LEPTON_DATA_9_REGISTER: LeptonRegister = lepton_register_val(0x001A);
// const LEPTON_DATA_10_REGISTER: LeptonRegister = lepton_register_val(0x001C);
// const LEPTON_DATA_11_REGISTER: LeptonRegister = lepton_register_val(0x001E);
// const LEPTON_DATA_12_REGISTER: LeptonRegister = lepton_register_val(0x0020);
// const LEPTON_DATA_13_REGISTER: LeptonRegister = lepton_register_val(0x0022);
// const LEPTON_DATA_14_REGISTER: LeptonRegister = lepton_register_val(0x0024);
// const LEPTON_DATA_15_REGISTER: LeptonRegister = lepton_register_val(0x0026);

#[derive(PartialEq, Debug, Format)]
pub enum LeptonError {
    Ok = 0,                      // Camera ok
    Error = -1,                  // Camera general error
    NotReady = -2,               // Camera not ready error
    RangeError = -3,             // Camera range error
    ChecksumError = -4,          // Camera checksum error
    BadArgPointerError = -5,     // Camera Bad argument error
    DataSizeError = -6,          // Camera byte count error
    UndefinedFunctionError = -7, // Camera undefined function error
    FunctionNotSupported = -8,   // Camera function not yet supported error
    DataOutOfRange = -9,         // Camera input DATA is out of valid range error
    CommandNotAllowed = -11,     // Camera unable to execute command due to current camera state

    OtpWriteError = -15,         // Camera OTP write error
    OtpReadError = -16,          // double bit error detected (uncorrectible)
    OtpNotProgrammedError = -18, // Flag read as non-zero

    ErrorI2CBusNotReady = -20,     // I2C Bus Error - Bus Not Avaialble
    ErrorI2CBufferOverflow = -22,  // I2C Bus Error - Buffer Overflow
    ErrorI2CArbitrationLost = -23, // I2C Bus Error - Bus Arbitration Lost
    ErrorI2CBusError = -24,        // I2C Bus Error - General Bus Error
    ErrorI2CNackReceived = -25,    // I2C Bus Error - NACK Received
    ErrorI2CFail = -26,            // I2C Bus Error - General Failure

    DivZeroError = -80, // Attempted div by zero

    CommPortNotOpen = -101,      // Comm port not open
    CommInvalidPortError = -102, // Comm port no such port error
    CommRangeError = -103,       // Comm port range error
    ErrorCreatingComm = -104,    // Error creating comm
    ErrorStartingComm = -105,    // Error starting comm
    ErrorClosingComm = -106,     // Error closing comm
    CommChecksumError = -107,    // Comm checksum error
    CommNoDev = -108,            // No comm device
    TimoutError = -109,          // Comm timeout error
    CommErrorWaitingComm = -110, // Error writing comm
    CommErrorReadingComm = -111, // Error reading comm
    CommCountError = -112,       // Comm byte count error

    OperationCanceled = -126,  // Camera operation canceled
    UndefinedErrorCode = -127, // Undefined error
}

impl LeptonError {
    #[allow(clippy::enum_glob_use)]
    fn from_i8(value: i8) -> LeptonError {
        use LeptonError::*;
        match value {
            0 => Ok,                      // Camera ok
            -1 => Error,                  // Camera general error
            -2 => NotReady,               // Camera not ready error
            -3 => RangeError,             // Camera range error
            -4 => ChecksumError,          // Camera checksum error
            -5 => BadArgPointerError,     // Camera Bad argument error
            -6 => DataSizeError,          // Camera byte count error
            -7 => UndefinedFunctionError, // Camera undefined function error
            -8 => FunctionNotSupported,   // Camera function not yet supported error
            -9 => DataOutOfRange,         // Camera input DATA is out of valid range error
            -11 => CommandNotAllowed, // Camera unable to execute command due to current camera state

            -15 => OtpWriteError,         // Camera OTP write error
            -16 => OtpReadError,          // double bit error detected (uncorrectible)
            -18 => OtpNotProgrammedError, // Flag read as non-zero

            -20 => ErrorI2CBusNotReady, // I2C Bus Error - Bus Not Avaialble
            -22 => ErrorI2CBufferOverflow, // I2C Bus Error - Buffer Overflow
            -23 => ErrorI2CArbitrationLost, // I2C Bus Error - Bus Arbitration Lost
            -24 => ErrorI2CBusError,    // I2C Bus Error - General Bus Error
            -25 => ErrorI2CNackReceived, // I2C Bus Error - NACK Received
            -26 => ErrorI2CFail,        // I2C Bus Error - General Failure

            -80 => DivZeroError, // Attempted div by zero

            -101 => CommPortNotOpen,      // Comm port not open
            -102 => CommInvalidPortError, // Comm port no such port error
            -103 => CommRangeError,       // Comm port range error
            -104 => ErrorCreatingComm,    // Error creating comm
            -105 => ErrorStartingComm,    // Error starting comm
            -106 => ErrorClosingComm,     // Error closing comm
            -107 => CommChecksumError,    // Comm checksum error
            -108 => CommNoDev,            // No comm device
            -109 => TimoutError,          // Comm timeout error
            -110 => CommErrorWaitingComm, // Error writing comm
            -111 => CommErrorReadingComm, // Error reading comm
            -112 => CommCountError,       // Comm byte count error

            -126 => OperationCanceled,  // Camera operation canceled
            -127 => UndefinedErrorCode, // Undefined error
            _ => {
                error!("Unknown lepton error code: {}", value);
                UndefinedErrorCode
            }
        }
    }
}

type Vsync = Pin<Gpio19, FunctionSio<SioInput>, PullDown>;
type SpiCs = Pin<Gpio21, FunctionSpi, PullDown>;
type SpiTx = Pin<Gpio23, FunctionSpi, PullDown>;
type SpiRx = Pin<Gpio20, FunctionSpi, PullDown>;
type SpiClk = Pin<Gpio22, FunctionSpi, PullDown>;
type Sda = Pin<Gpio24, FunctionI2C, PullUp>;
type Scl = Pin<Gpio25, FunctionI2C, PullUp>;
type LeptonCciI2c = I2CInterface<I2C0, (Sda, Scl)>;
type PowerEnable = Pin<Gpio18, FunctionSio<SioOutput>, PullDown>;
type PowerDown = Pin<Gpio28, FunctionSio<SioOutput>, PullDown>;
type Reset = Pin<Gpio29, FunctionSio<SioOutput>, PullDown>;
type ClkDisable = Pin<Gpio27, FunctionSio<SioOutput>, PullDown>;
type MasterClk = Pin<Gpio26, FunctionSio<SioInput>, PullNone>;
pub struct LeptonModule {
    spi: Option<Spi<Enabled, SPI0, (SpiTx, SpiRx, SpiClk), 16>>,
    cs: Option<SpiCs>,
    pub vsync: Vsync,
    cci: LeptonCciI2c,
    power_enable: PowerEnable,
    power_down: PowerDown,
    reset: Reset,
    clk_disable: ClkDisable,
    master_clk: MasterClk,
    is_powered_on: bool,
}

#[repr(C)]
enum FFCShutterMode {
    Manual = 0,
    Auto = 1,
    External = 2,
}

#[repr(C)]
enum FFCTempLockoutState {
    Inactive = 0,
    High = 1,
    Low = 2,
}

#[repr(C)]
#[derive(PartialEq, Format, Debug)]
pub enum FFCStatus {
    NeverCommanded = 0,
    Imminent = 1,
    InProgress = 2,
    Done = 3,
}

impl From<u8> for FFCStatus {
    fn from(value: u8) -> Self {
        match value {
            0 => FFCStatus::NeverCommanded,
            1 => FFCStatus::Imminent,
            2 => FFCStatus::InProgress,
            3 => FFCStatus::Done,
            _ => unreachable!("Invalid FFC value {}", value),
        }
    }
}

#[repr(C)]
#[derive(PartialEq)]
enum LepSysEnable {
    Disable = 0,
    Enable = 1,
}

#[repr(C)]
pub struct FFCState {
    shutter_mode: FFCShutterMode,
    temp_lockout_state: FFCTempLockoutState,
    video_freeze_during_ffc: LepSysEnable,
    ffc_desired: LepSysEnable,
    elapsed_time_since_last_ffc: u32,
    desired_ffc_period_ms: u32,
    explicit_command_to_open: bool,
    desired_ffc_temp_delta: u16, // Kelvin x 100
    imminent_delay: u16,         // in frame counts
}

impl Default for FFCState {
    fn default() -> Self {
        FFCState {
            shutter_mode: FFCShutterMode::Manual,
            temp_lockout_state: FFCTempLockoutState::Inactive,
            video_freeze_during_ffc: LepSysEnable::Disable, // Keep video going during FFC, and discard frames based on their telemetry - this makes it easier for us to maintain sync.
            ffc_desired: LepSysEnable::Disable,
            elapsed_time_since_last_ffc: 0,
            desired_ffc_period_ms: 0,
            explicit_command_to_open: false,
            desired_ffc_temp_delta: 0,
            imminent_delay: 0,
        }
    }
}

impl FFCState {
    pub fn ffc_is_needed(&self) -> bool {
        self.ffc_desired == LepSysEnable::Enable
    }
}

#[derive(Default)]
pub struct LeptonFirmwareInfo {
    pub gpp_major: u8,
    pub gpp_minor: u8,
    pub gpp_build: u8,
    pub dsp_major: u8,
    pub dsp_minor: u8,
    pub dsp_build: u8,
}

type EnabledLeptonSpiPeripheral = Spi<
    Enabled,
    SPI0,
    (
        Pin<Gpio23, FunctionSpi, PullDown>,
        Pin<Gpio20, FunctionSpi, PullDown>,
        Pin<Gpio22, FunctionSpi, PullDown>,
    ),
    16,
>;

impl LeptonModule {
    pub fn new(
        i2c: LeptonCciI2c,
        spi: EnabledLeptonSpiPeripheral,
        cs: Pin<Gpio21, FunctionSpi, PullDown>,
        vsync: Vsync,
        power_enable: Pin<Gpio18, FunctionSio<SioOutput>, PullDown>,
        power_down: Pin<Gpio28, FunctionSio<SioOutput>, PullDown>,
        reset: Pin<Gpio29, FunctionSio<SioOutput>, PullDown>,
        clk_disable: Pin<Gpio27, FunctionSio<SioOutput>, PullDown>,
        master_clk: Pin<Gpio26, FunctionSio<SioInput>, PullNone>,
    ) -> LeptonModule {
        LeptonModule {
            spi: Some(spi),
            cs: Some(cs),
            vsync,
            power_enable,
            power_down,
            reset,
            clk_disable,
            master_clk,
            cci: i2c,
            is_powered_on: false,
        }
    }

    pub fn init(&mut self) -> Result<(), &'static str> {
        /*
            a. Wait 950 milliseconds minimum after power on, clocks applied, and RESET_L de-asserted
                • If the camera has an attached shutter, the minimum wait period should be
                extended to 5 seconds to allow the camera’s automatic execution of a flat-field
                correction (Auto FFC mode).
            b. Read the STATUS register (register address 0x0002) bit 2
                • If Bit 2 is 0, then the camera has not booted yet, extend the wait period.
                • If Bit 2 is 1, then the camera has booted, I2C interface is available
            c. Read the STATUS register (register address 0x0002) bit 0
                • If Bit 0 is 1, then the interface is busy, poll until Bit 0 becomes 0
                • If Bit 0 is 0, then the interface is ready for receiving commands.
        */
        info!("=== Init lepton module ===");
        // NOTE: It's actually quite good if we don't have FFC interrupt the video feed - we can
        //  just discard those frames later based on the telemetry, and this helps with sync.
        let success = self.disable_automatic_ffc();
        if success.is_err() {
            return Err("Could not disable automatic FFC");
        }

        //let mut t_enabled = false;
        let success = self.enable_telemetry();
        if success.is_err() {
            warn!("Could not enable telemetry");
        }
        let telemetry_enabled = self.telemetry_enabled();
        if let Ok(telemetry_enabled) = telemetry_enabled {
            if !telemetry_enabled {
                return Err("Telemetry not set");
            }
        } else {
            return Err("Telemetry not set");
        }
        let telemetry_location = self.telemetry_location();
        if let Ok(telemetry_location) = telemetry_location {
            if telemetry_location != TelemetryLocation::Header {
                return Err("Telemetry not in header");
            }
        } else {
            return Err("Telemetry not in header");
        }
        let success = self.enable_post_processing();
        if success.is_err() {
            warn!("Could not set post processing");
        }
        let radiometry_enabled = self.radiometric_mode_enabled();
        if let Ok(radiometry_enabled) = radiometry_enabled {
            if !radiometry_enabled {
                return Err("Radiometry not enabled");
            }
        } else {
            return Err("Radiometry not enabled");
        }

        let success = self.enable_vsync();
        if success.is_err() {
            warn!("Could not enable vsync");
        }
        let vsync_enabled = self.vsync_enabled();
        if let Ok(vsync_enabled) = vsync_enabled {
            if !vsync_enabled {
                return Err("Vsync not enabled");
            }
        } else {
            return Err("Vsync not enabled");
        }
        Ok(())
    }

    fn setup_spot_meter_roi(&mut self) -> Result<bool, LeptonError> {
        // Change spot meter ROI so we can get scene stats per frame
        self.set_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_RAD,
                LEPTON_RAD_SPOT_METER_ROI,
                LeptonCommandType::Set,
                true,
            ),
            //  [startRow, startCol, endRow, endCol]
            //&[0, 0, 119, 159],
            &[79, 59, 80, 60], // If this is set to anything but the default 2x2 pixels, we get visual glitches in the video feed :-/
        )
    }

    fn enable_post_processing(&mut self) -> Result<bool, LeptonError> {
        // This is probably the default post-processing settings on lepton startup,
        // but it doesn't hurt to set them anyway
        let success = self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_BAD_PIXEL_REPLACEMENT,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );
        if success.is_err() {
            warn!("{}", success);
        }

        let success = self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_TEMPORAL_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );
        if success.is_err() {
            warn!("{}", success);
        }

        let success = self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_COLUMN_NOISE_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );
        if success.is_err() {
            warn!("{}", success);
        }

        let success = self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_PIXEL_NOISE_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );
        if success.is_err() {
            warn!("{}", success);
        }
        success
    }

    pub fn enable_video_output(&mut self, enabled: bool) -> Result<bool, LeptonError> {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_VIDEO_OUTPUT_ENABLE,
                LeptonCommandType::Set,
                true,
            ),
            i32::from(enabled),
        )
    }

    pub fn power_down(&mut self) -> Result<bool, LeptonError> {
        self.execute_command(lepton_command(
            LEPTON_SUB_SYSTEM_OEM,
            LEPTON_OEM_POWER_DOWN,
            LeptonCommandType::Run,
            true,
        ))
    }

    fn disable_t_linear(&mut self) -> Result<bool, LeptonError> {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_RAD,
                LEPTON_RAD_TLINEAR_ENABLE_STATE,
                LeptonCommandType::Set,
                true,
            ),
            0,
        )
    }

    pub fn reset_spi(
        &mut self,
        delay: &mut Delay,
        resets: &mut RESETS,
        freq: HertzU32,
        baudrate: HertzU32,
        print: bool,
    ) {
        if print {
            info!("Resetting spi");
        }
        let spi = self.spi.take().unwrap();
        let spi = spi.disable();
        let (spi, (tx, rx, sck)) = spi.free();
        let mut cs = self.cs.take().unwrap().into_push_pull_output();
        let mut sck = sck.into_push_pull_output();
        cs.set_high().unwrap();
        sck.set_high().unwrap(); // SCK is high when idle
        delay.delay_ms(200);
        cs.set_low().unwrap();
        sck.set_low().unwrap();
        self.cs = Some(cs.into_function::<FunctionSpi>());
        let sck = sck.into_function::<FunctionSpi>();

        self.spi = Some(Spi::new(spi, (tx, rx, sck)).init(resets, freq, baudrate, MODE_3));

        if print {
            info!("Finished resetting spi");
        }
    }

    pub fn transfer<'w>(&mut self, words: &'w mut [u16]) -> Result<&'w [u16], Infallible> {
        self.spi.as_mut().unwrap().transfer(words)
    }

    pub fn wait_for_ffc_status_ready(&mut self, delay: &mut Delay) -> bool {
        loop {
            match self.get_attribute(lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_FFC_STATUS,
                LeptonCommandType::Get,
                false,
            )) {
                Ok((ffc_state, length)) => {
                    let ffc_state = BigEndian::read_i32(&ffc_state[..((length * 2) as usize)]);
                    info!("FFC state {}", ffc_state);
                    delay.delay_ms(1);
                    if ffc_state == 0 {
                        return true;
                    }
                }
                Err(err) => {
                    return false;
                }
            }
        }
    }

    pub fn get_camera_serial(&mut self) -> Result<u32, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_GET_SERIAL,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((serial, length)) => {
                let serial = LittleEndian::read_u64(&serial[..((length * 2) as usize)]);
                // In practice, we hope serial numbers are less than 32bits.
                #[allow(clippy::cast_possible_truncation)]
                Ok(serial as u32)
            }
            Err(err) => Err(err),
        }
    }

    pub fn get_firmware_version(&mut self) -> Result<LeptonFirmwareInfo, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_OEM,
            LEPTON_OEM_CAMERA_SOFTWARE_REVISION,
            LeptonCommandType::Get,
            true,
        )) {
            Ok((firmware_version, length)) => {
                let firmware_version = &firmware_version[..((length * 2) as usize)];
                let gpp_major = firmware_version[0];
                let gpp_minor = firmware_version[1];
                let gpp_build = firmware_version[2];

                let dsp_major = firmware_version[0];
                let dsp_minor = firmware_version[1];
                let dsp_build = firmware_version[2];
                Ok(LeptonFirmwareInfo {
                    gpp_major,
                    gpp_minor,
                    gpp_build,
                    dsp_major,
                    dsp_minor,
                    dsp_build,
                })
            }
            Err(err) => {
                error!("Err {}", err);
                Err(err)
            }
        }
    }

    pub fn telemetry_location(&mut self) -> Result<TelemetryLocation, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_TELEMETRY_LOCATION,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((telemetry_location, length)) => {
                let telemetry_location_state =
                    LittleEndian::read_u32(&telemetry_location[..((length * 2) as usize)]);
                if telemetry_location_state == 0 {
                    Ok(TelemetryLocation::Header)
                } else {
                    Ok(TelemetryLocation::Footer)
                }
            }
            Err(err) => Err(err),
        }
    }

    pub fn enable_telemetry(&mut self) -> Result<bool, LeptonError> {
        // Enable telemetry
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_TELEMETRY_ENABLE_STATE,
                LeptonCommandType::Set,
                false,
            ),
            1,
        )?;
        // Set location to header
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_TELEMETRY_LOCATION,
                LeptonCommandType::Set,
                false,
            ),
            0,
        )
    }

    pub fn disable_automatic_ffc(&mut self) -> Result<bool, LeptonError> {
        let ffc_struct = FFCState::default();
        let ffc_struct_bytes = bytemuck::cast_slice(unsafe { any_as_u8_slice(&ffc_struct) });
        // Enable telemetry
        self.set_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_FFC_SHUTTER_MODE,
                LeptonCommandType::Set,
                false,
            ),
            ffc_struct_bytes,
        )
    }

    pub fn get_ffc_state(&mut self) -> Result<FFCState, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_FFC_SHUTTER_MODE,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((state, _length)) => {
                let struct_size = size_of::<FFCState>();
                let mut ffc_state = [0u8; 20];
                ffc_state.copy_from_slice(&state[0..struct_size]);
                let ffc_state: FFCState = unsafe { mem::transmute(ffc_state) };
                Ok(ffc_state)
            }
            Err(err) => Err(err),
        }
    }

    pub fn get_ffc_status(&mut self) -> Result<FFCStatus, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_FFC_STATUS,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((state, length)) => {
                let ffc_status = LittleEndian::read_u32(&state[..((length * 2) as usize)]);
                let ffc_status: FFCStatus = match ffc_status {
                    0 => FFCStatus::NeverCommanded,
                    1 => FFCStatus::Imminent,
                    2 => FFCStatus::InProgress,
                    3 => FFCStatus::Done,
                    _ => panic!("Unknown FFC Status {}", ffc_status),
                };
                Ok(ffc_status)
            }
            Err(err) => Err(err),
        }
    }

    pub fn do_ffc(&mut self) -> Result<bool, LeptonError> {
        self.execute_command_non_blocking(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_RUN_FFC,
            LeptonCommandType::Run,
            false,
        ))
    }

    pub fn telemetry_enabled(&mut self) -> Result<bool, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_TELEMETRY_ENABLE_STATE,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((telemetry_enabled, length)) => {
                Ok(LittleEndian::read_u32(&telemetry_enabled[..((length * 2) as usize)]) == 1)
            }
            Err(err) => Err(err),
        }
    }

    pub fn output_raw14(&mut self) -> Result<bool, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_VID,
            LEPTON_VID_OUTPUT_FORMAT,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((output_state, length)) => {
                let output_state = LittleEndian::read_u32(&output_state[..((length * 2) as usize)]);
                Ok(output_state == 7)
            }
            Err(err) => Err(err),
        }
    }

    pub fn status(&mut self) -> Result<u32, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_STATUS,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((output_state, _length)) => Ok(LittleEndian::read_u32(&output_state[..4usize])),
            Err(err) => Err(err),
        }
    }

    pub fn enable_focus_metric(&mut self) -> Result<bool, LeptonError> {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_VID,
                LEPTON_VID_FOCUS_ENABLE_STATE,
                LeptonCommandType::Set,
                false,
            ),
            1,
        )
    }

    pub fn focus_metric_enabled(&mut self) -> Result<bool, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_VID,
            LEPTON_VID_FOCUS_ENABLE_STATE,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((enabled_focus_state, l)) => {
                Ok(LittleEndian::read_u32(&enabled_focus_state[..((l * 2) as usize)]) == 1)
            }
            Err(err) => Err(err),
        }
    }

    pub fn get_focus_metric(&mut self) -> Result<u32, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_VID,
            LEPTON_VID_FOCUS_METRIC,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((focus_metric, l)) => {
                Ok(LittleEndian::read_u32(&focus_metric[..((l * 2) as usize)]))
            }
            Err(err) => Err(err),
        }
    }

    pub fn acg_enabled(&mut self) -> Result<bool, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_AGC,
            LEPTON_AGC_ENABLE_STATE,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((agc_state, length)) => {
                let acg_enabled_state =
                    LittleEndian::read_u32(&agc_state[..((length * 2) as usize)]);
                Ok(acg_enabled_state == 1)
            }
            Err(err) => Err(err),
        }
    }

    pub fn ping(&mut self) -> Result<bool, LeptonError> {
        self.execute_command(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_PING_CAMERA,
            LeptonCommandType::Run,
            false,
        ))
    }

    pub fn reboot(&mut self, delay: &mut Delay) -> bool {
        warn!("Rebooting lepton module");
        let success = self.execute_command(lepton_command(
            LEPTON_SUB_SYSTEM_OEM,
            LEPTON_OEM_REBOOT,
            LeptonCommandType::Run,
            true,
        ));
        if success.is_err() {
            warn!("Error rebooting lepton");
        }

        info!("Waiting 5 seconds");
        delay.delay_ms(5000);
        while let Err(e) = self.init() {
            self.reboot(delay);
        }
        success.is_ok()
    }

    pub fn is_awake(&self) -> bool {
        self.is_powered_on
    }

    pub fn power_down_sequence(&mut self, delay: &mut Delay) {
        // Putting lepton into standby mode, uses about 5mW in standby mode.
        trace!("power down asserted");
        self.power_down.set_low().unwrap();

        // Datasheet says to wait at least 100ms before turning off clock after power down.
        delay.delay_ms(100);
        trace!("clk disabled");
        self.clk_disable.set_low().unwrap();

        // power off disables the 3.0V, 2.8V and 1.2V
        trace!("power off");
        self.power_enable.set_low().unwrap();
        self.is_powered_on = false;
        delay.delay_ms(200);
    }

    pub fn power_on(&mut self, delay: &mut Delay) {
        self.power_enable.set_low().unwrap();
        delay.delay_ms(100);
        self.clk_disable.set_low().unwrap();
        self.power_enable.set_high().unwrap();
        delay.delay_ms(100);
    }

    // Page 18 https://www.flir.com/globalassets/imported-assets/document/flir-lepton-engineering-datasheet.pdf
    pub fn start_up_sequence(&mut self, delay: &mut Delay) {
        self.power_down.set_high().unwrap();
        delay.delay_ms(1);
        self.reset.set_low().unwrap();
        delay.delay_ms(1);
        self.clk_disable.set_high().unwrap();
        delay.delay_ms(1);
        self.reset.set_high().unwrap();
    }

    pub fn cci_init(&mut self, delay: &mut Delay) {
        trace!("de-assert reset");
        delay.delay_ms(2000);
        trace!("Wait for ready");
        self.wait_for_ready(false);
        while let Err(e) = self.init() {
            error!("{:?}", e);
            self.cci_init(delay);
        }
    }

    pub fn power_on_sequence(&mut self, delay: &mut Delay) {
        self.power_on(delay);
        self.start_up_sequence(delay);
        self.cci_init(delay);
        self.is_powered_on = true;
    }

    pub fn radiometric_mode_enabled(&mut self) -> Result<bool, LeptonError> {
        // Try to enable radiometric mode if available.
        let success = match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_RAD,
            LEPTON_RAD_ENABLE_STATE,
            LeptonCommandType::Get,
            true,
        )) {
            Ok((radiometry_enabled, length)) => {
                Ok(LittleEndian::read_u32(&radiometry_enabled[..((length * 2) as usize)]) == 1)
            }
            Err(err) => Err(err),
        };
        if let Ok(true) = success {
            match self.get_attribute(lepton_command(
                LEPTON_SUB_SYSTEM_RAD,
                LEPTON_RAD_TLINEAR_ENABLE_STATE,
                LeptonCommandType::Get,
                true,
            )) {
                Ok((radiometry_enabled, length)) => {
                    Ok(LittleEndian::read_u32(&radiometry_enabled[..((length * 2) as usize)]) == 1)
                }
                Err(err) => Err(err),
            }
        } else {
            success
        }
    }

    pub fn enable_vsync(&mut self) -> Result<bool, LeptonError> {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_GPIO_VSYNC_PHASE_DELAY,
                LeptonCommandType::Set,
                true,
            ),
            0,
        )?;

        // Set the phase to -1 line, and see if we have less reboots
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_GPIO_MODE,
                LeptonCommandType::Set,
                true,
            ),
            5,
        )
    }

    pub fn vsync_enabled(&mut self) -> Result<bool, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_OEM,
            LEPTON_OEM_GPIO_MODE,
            LeptonCommandType::Get,
            true,
        )) {
            Ok((vsync_state, length)) => {
                Ok(LittleEndian::read_u32(&vsync_state[..((length * 2) as usize)]) == 5)
            }
            Err(err) => Err(err),
        }
    }

    pub fn scene_stats(&mut self) -> Result<SceneStats, LeptonError> {
        match self.get_attribute(lepton_command(
            LEPTON_SUB_SYSTEM_SYS,
            LEPTON_SYS_STATS,
            LeptonCommandType::Get,
            false,
        )) {
            Ok((scene_stats, _length)) => Ok(SceneStats {
                avg: LittleEndian::read_u16(&scene_stats[0..2]),
                max: LittleEndian::read_u16(&scene_stats[2..4]),
                min: LittleEndian::read_u16(&scene_stats[4..6]),
                num_pixels: LittleEndian::read_u16(&scene_stats[6..8]),
            }),
            Err(err) => Err(err),
        }
    }

    pub fn wait_for_ready(&mut self, print: bool) -> LeptonError {
        let mut readbuf: [u8; 2];
        let mut camera_status = 0u16;
        let mut failures = 0;
        loop {
            readbuf = [0; 2];
            if match self
                .cci
                .write_read(LEPTON_ADDRESS, &LEPTON_STATUS_REGISTER, &mut readbuf)
            {
                Ok(()) => {
                    camera_status = BigEndian::read_u16(&readbuf);
                    if print {
                        info!("Booted {}", camera_status & LEPTON_BOOTED == LEPTON_BOOTED);
                        info!("Busy {}", camera_status & LEPTON_BUSY == LEPTON_BUSY);
                        info!(
                            "Boot mode {}",
                            camera_status & LEPTON_BOOT_MODE == LEPTON_BOOT_MODE
                        );
                        info!("Camera status {:#018b}", camera_status);
                    }
                    camera_status & (LEPTON_BOOTED | LEPTON_BOOT_MODE | LEPTON_BUSY)
                        == LEPTON_BOOTED | LEPTON_BOOT_MODE
                }
                Err(e) => {
                    warn!("i2c Err {:?}", e);
                    false
                }
            } {
                return LeptonError::from_i8((camera_status >> 8) as i8);
            }
            failures += 1;
            if failures > 100 {
                return LeptonError::from_i8((camera_status >> 8) as i8);
            }
            //warn!("i2c error");
            if print {
                info!("Booted {:#018b}", camera_status & LEPTON_BOOTED);
                info!("Boot mode {:#018b}", camera_status & LEPTON_BOOT_MODE);
                info!("Busy {:#018b}", camera_status & LEPTON_BUSY);
            }
        }
    }

    fn execute_command(
        &mut self,
        (command, _length): LeptonSynthesizedCommand,
    ) -> Result<bool, LeptonError> {
        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }
        let success = self.write(&[
            LEPTON_COMMAND_ID_REGISTER[0],
            LEPTON_COMMAND_ID_REGISTER[1],
            command[0],
            command[1],
        ]);
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok(false);
        }

        let success = self.wait_for_ready(false);
        if success == LeptonError::Ok {
            Ok(true)
        } else {
            Err(success)
        }
    }

    fn execute_command_non_blocking(
        &mut self,
        (command, _length): LeptonSynthesizedCommand,
    ) -> Result<bool, LeptonError> {
        self.wait_for_ready(false);
        let success = self.write(&[
            LEPTON_COMMAND_ID_REGISTER[0],
            LEPTON_COMMAND_ID_REGISTER[1],
            command[0],
            command[1],
        ]);
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok(false);
        }
        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }
        Ok(true)
    }

    fn write(&mut self, payload: &[u8]) -> Result<(), Error> {
        let result = self.cci.write(LEPTON_ADDRESS, payload);
        while self.cci.tx_fifo_used() != 0 {}
        result
    }

    fn get_attribute(
        &mut self,
        (command, length): LeptonSynthesizedCommand,
    ) -> Result<([u8; 32], u16), LeptonError> {
        self.wait_for_ready(false);
        let len = lepton_register_val(length);
        let mut buf = [0u8; 32];
        let success = self.write(&[
            LEPTON_DATA_LENGTH_REGISTER[0],
            LEPTON_DATA_LENGTH_REGISTER[1],
            len[0],
            len[1],
        ]);
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok((buf, 0));
        }
        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            //warn!("Set attribute error {}", success);
            return Err(success);
        }
        let success = self.write(&[
            LEPTON_COMMAND_ID_REGISTER[0],
            LEPTON_COMMAND_ID_REGISTER[1],
            command[0],
            command[1],
        ]);
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok((buf, 0));
        }
        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            //warn!("Set attribute error {}", success);
            return Err(success);
        }

        // Now read out length registers, and assemble them into the return type that we want.
        let success = self.cci.write_read(
            LEPTON_ADDRESS,
            &LEPTON_DATA_0_REGISTER,
            &mut buf[..(length as usize * 2)],
        );
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok((buf, 0));
        }

        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            //warn!("Set attribute error {}", success);
            return Err(success);
        }

        // Each u16 needs to be read in order before we can read out a u32 properly
        for chunk in buf.chunks_exact_mut(2) {
            chunk.swap(0, 1);
        }
        Ok((buf, length))
    }

    fn set_attribute_i32(
        &mut self,
        command: LeptonSynthesizedCommand,
        val: i32,
    ) -> Result<bool, LeptonError> {
        #[allow(clippy::cast_sign_loss)]
        self.set_attribute(
            command,
            &[(val & 0xffff) as u16, (val >> 16 & 0xffff) as u16],
        )
    }

    fn set_attribute(
        &mut self,
        (command, length): LeptonSynthesizedCommand,
        words: &[u16],
    ) -> Result<bool, LeptonError> {
        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }
        //self.wait_for_ffc_status_ready();
        let len = lepton_register_val(length);

        let success = self.write(&[
            LEPTON_DATA_LENGTH_REGISTER[0],
            LEPTON_DATA_LENGTH_REGISTER[1],
            len[0],
            len[1],
        ]);

        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok(false);
        }

        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }

        for (index, word) in words.iter().enumerate() {
            // Data reg 0
            #[allow(clippy::cast_possible_truncation)]
            let register_address = (0x0008u16 + (index * 2) as u16).to_be_bytes();
            let word = word.to_be_bytes();
            let success = self.write(&[register_address[0], register_address[1], word[0], word[1]]);
            if success.is_err() {
                warn!("i2c Write error {:?}", success.err().unwrap());
                return Ok(false);
            }
        }

        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }

        let success = self.write(&[
            LEPTON_COMMAND_ID_REGISTER[0],
            LEPTON_COMMAND_ID_REGISTER[1],
            command[0],
            command[1],
        ]);
        if success.is_err() {
            warn!("i2c Write error {:?}", success.err().unwrap());
            return Ok(false);
        }

        let success = self.wait_for_ready(false);
        if success != LeptonError::Ok {
            return Err(success);
        }
        // Now read out length registers, and assemble them into the return type that we want.

        // Now read the status register to see if it worked?
        Ok(true)
    }
}

#[allow(clippy::cast_possible_truncation)]
const fn lepton_register_val(cmd: u16) -> LeptonRegister {
    let mut buf = [0; 2];
    buf[0] = (cmd >> 8) as u8;
    buf[1] = cmd as u8;
    buf
}

pub const fn lepton_command(
    sub_system: u16,
    command: LeptonCommand,
    command_type: LeptonCommandType,
    is_oem: bool,
) -> LeptonSynthesizedCommand {
    let mut synthesized_command = sub_system | command.0 | command_type as u16;
    if is_oem {
        synthesized_command |= LEPTON_COMMAND_OEM_BIT;
    }
    (lepton_register_val(synthesized_command), command.1)
}

pub struct SceneStats {
    pub min: u16,
    pub max: u16,
    pub avg: u16,
    pub num_pixels: u16,
}

#[derive(PartialEq, Format)]
pub enum TelemetryLocation {
    Header,
    Footer,
}

struct CentiK {
    inner: u16,
}

pub struct LeptonPins {
    pub(crate) tx: SpiTx,
    pub(crate) rx: SpiRx,
    pub(crate) clk: SpiClk,
    pub(crate) cs: SpiCs,

    pub(crate) vsync: Vsync,

    pub(crate) sda: Sda,
    pub(crate) scl: Scl,

    pub(crate) power_down: PowerDown,
    pub(crate) power_enable: PowerEnable,
    pub(crate) reset: Reset,
    pub(crate) clk_disable: ClkDisable,
    pub(crate) master_clk: MasterClk,
}

pub fn init_lepton_module(
    spi_peripheral: SPI0,
    ic2_peripheral: I2C0,
    system_clock_freq_hz: HertzU32,
    resets: &mut RESETS,
    delay: &mut Delay,
    pins: LeptonPins,
) -> LeptonModule {
    let spi = Spi::new(spi_peripheral, (pins.tx, pins.rx, pins.clk)).init(
        resets,
        system_clock_freq_hz,
        LEPTON_SPI_CLOCK_FREQ.Hz(),
        MODE_3,
    );
    let mut lepton = LeptonModule::new(
        bsp::hal::I2C::i2c0(
            ic2_peripheral,
            pins.sda,
            pins.scl,
            100.kHz(),
            resets,
            system_clock_freq_hz,
        ),
        spi,
        pins.cs,
        pins.vsync,
        pins.power_enable,
        pins.power_down,
        pins.reset,
        pins.clk_disable,
        pins.master_clk,
    );

    // TODO: When going dormant, can we make sure we don't have any gpio pins in a pullup/down mode.
    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, false);
    info!("Lepton startup sequence");
    lepton.power_down_sequence(delay);
    lepton.power_on_sequence(delay);
    // Set wake from dormant on vsync
    lepton.vsync.clear_interrupt(Interrupt::EdgeHigh);
    lepton
        .vsync
        .set_dormant_wake_enabled(Interrupt::EdgeHigh, true);
    lepton
}
