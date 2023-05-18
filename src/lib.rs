#![no_std]

extern crate embedded_hal;
extern crate micromath;

use core::f32::consts::PI;

use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};

use micromath::F32Ext;

pub const DEFAULT_SLAVE_ADDR: u8 = 0x1E;

#[derive(Debug, Clone, Eq, PartialEq)]
pub enum HMC5883LError<E> {
    I2c(E),
}

pub struct HMC5883L<I> {
    i2c: I,
    slave_addr: u8,
}

impl<I, E> HMC5883L<I>
where
    I: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I, slave_addr: Option<u8>) -> Self {
        HMC5883L {
            i2c,
            slave_addr: slave_addr.unwrap_or(DEFAULT_SLAVE_ADDR),
        }
    }

    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), HMC5883LError<E>> {
        // set gain to +/1 1.3 Gauss
        self.write_byte(0x01, 0x20)?;
        // set in continuous-measurement mode
        self.write_byte(0x02, 0x00)?;

        delay.delay_ms(100u8);

        Ok(())
    }

    pub fn heading(&mut self) -> Result<f32, HMC5883LError<E>> {
        let (x, y, z) = self.read_raw_mag()?;

        let gauss_lsb_xy = 1100.0;
        let gauss_lsb_z = 980.0;
        let (x, y, z) = (x as f32, y as f32, z as f32);

        let (x, y, _z) = (
            x / gauss_lsb_xy * 100.0,
            y / gauss_lsb_xy * 100.0,
            z / gauss_lsb_z * 100.0,
        );

        // You need to determine the correct magnetic declination for your location for accurate
        // readings. Find yours at http://www.magnetic-declination.com/
        // Warsaw is 6.45 degree, it is 0.11780972450961724 Radians
        let declination_angle = 0.11780972450961724; // in radians, not degrees

        let mut heading = y.atan2(x) + declination_angle;

        if heading < 0.0 {
            heading += 2.0 * PI;
        }

        if heading > 2.0 * PI {
            heading -= 2.0 * PI;
        }

        // Convert radians to degrees for readability.
        heading = heading * 180.0 / PI;

        Ok(heading)
    }

    pub fn read_raw_mag(&mut self) -> Result<(i16, i16, i16), HMC5883LError<E>> {
        // parse (x, z, y)
        let x_msb = self.read_byte(0x03)?;
        let x_lsb = self.read_byte(0x04)?;
        let x: i16 = ((x_msb as i16) << 8) as i16 | x_lsb as i16;

        let z_msb = self.read_byte(0x05)?;
        let z_lsb = self.read_byte(0x06)?;
        let z: i16 = ((z_msb as i16) << 8) as i16 | z_lsb as i16;

        let y_msb = self.read_byte(0x07)?;
        let y_lsb = self.read_byte(0x08)?;
        let y: i16 = ((y_msb as i16) << 8) as i16 | y_lsb as i16;

        // rearrange tuple x, y, z values
        Ok((x, y, z))
    }

    /// Writes byte to register
    fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), HMC5883LError<E>> {
        self.i2c
            .write(self.slave_addr, &[reg, byte])
            .map_err(HMC5883LError::I2c)?;
        Ok(())
    }

    /// Reads byte from register
    fn read_byte(&mut self, reg: u8) -> Result<u8, HMC5883LError<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.slave_addr, &[reg], &mut byte)
            .map_err(HMC5883LError::I2c)?;
        Ok(byte[0])
    }
}
