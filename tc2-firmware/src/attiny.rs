use embedded_hal::blocking::i2c::{Write, Read};
use core::fmt::Debug;

/// Camera states corresponding to the attiny1616 src/CameraState.h file
#[derive(Debug, PartialEq)]
pub enum CameraState {
    PoweringOn,
    PoweredOn,
    PoweringOff,
    PoweredOff,
    PowerOnTimeout,
}

#[derive(Debug)]
pub enum AttinyError<E> {
    I2CError(E),
    UnexpectedCameraState(u8),
}

impl<E> From<E> for AttinyError<E> {
    fn from(error: E) -> Self {
        AttinyError::I2CError(error)
    }
}

pub struct Attiny<I2C, E> {
    i2c: I2C,
    _phantom: core::marker::PhantomData<E>,
}

impl<I2C, E> Attiny<I2C, E>
where
    I2C: Read<Error = E> + Write<Error = E>,
    E: Debug,
{
    const I2C_ADDRESS: u8 = 0x25;
    const CAMERA_STATE_REGISTER: u8 = 0x02;

    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn check_camera_state(&mut self) -> Result<CameraState, AttinyError<E>> {
        match self.read_register(Self::CAMERA_STATE_REGISTER)? {
            0 => Ok(CameraState::PoweringOn),
            1 => Ok(CameraState::PoweredOn),
            2 => Ok(CameraState::PoweringOff),
            3 => Ok(CameraState::PoweredOff),
            4 => Ok(CameraState::PowerOnTimeout),
            value => Err(AttinyError::UnexpectedCameraState(value)),
        }
    }

    fn read_register(&mut self, register: u8) -> Result<u8, AttinyError<E>> {
        let write_buffer: [u8; 1] = [register];
        self.i2c.write(Self::I2C_ADDRESS, &write_buffer).map_err(AttinyError::from)?;
        let mut read_buffer: [u8; 1] = [0];
        self.i2c.read(Self::I2C_ADDRESS, &mut read_buffer).map_err(AttinyError::from)?;
        Ok(read_buffer[0])
    }
}
