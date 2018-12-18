//! This is a platform agnostic Rust driver for the HDC1080 temperature
//! sensor and thermal watchdog, based on the [`I2CDevice`] traits.
//!
//! [`I2CDevice`]: https://github.com/rust-embedded/i2cdev
//!
//! This driver allows you to:
//! - Read the temperature.
//! - Read the relative humidity.
//! - Read both temperature and humidity.
//! - Test low battery condition.
//! - Set temperature resolution.
//! - Set humidity resolution.
//! - Set heater on/off.
//! - Reset device.
//!
//! ## The device
//!
//! The HDC1080 is a digital humidity sensor with integrated temperature sensor. The host can
//! query the HDC1080 through its I2C interface to start a temperature and/or humidity reading at any
//! time.
//!
//! Datasheet:
//! - [hdc1080](http://www.ti.com/lit/ds/symlink/hdc1080.pdf)
//!
//!
//! ## Usage examples (see also examples folder)
//!
//! To use this driver, import this crate and an `I2CDevice` implementation,
//! then instantiate the device.
//!
//! ### Read temperature
//!
//! ```no_run
//! extern crate i2cdev;
//! extern crate hdc1080;
//!
//! use i2cdev::linux::LinuxI2CDevice;
//! use std::{thread, time};
//! use hdc1080::{Config, Hdc1080};
//!
//! const HDC1080_SLAVE_ADDR: u16 = 0x40;
//!
//! # fn main() {
//! let dev = LinuxI2CDevice::new("/dev/i2c-0", HDC1080_SLAVE_ADDR).unwrap();
//! let mut sensor = Hdc1080::new(dev);
//! sensor.read_temperature_start().unwrap();
//! thread::sleep(time::Duration::from_millis(500u64));
//! let temp_celsius = sensor.read_temperature_finish().unwrap();
//! println!("Temperature: {}ºC", temp_celsius);
//! # }
//! ```
//!
//! ### Read relative humidity
//!
//! ```no_run
//! extern crate i2cdev;
//! extern crate hdc1080;
//!
//! use i2cdev::linux::LinuxI2CDevice;
//! use std::{thread, time};
//! use hdc1080::{Config, Hdc1080};
//!
//! const HDC1080_SLAVE_ADDR: u16 = 0x40;
//!
//! # fn main() {
//! let dev = LinuxI2CDevice::new("/dev/i2c-0", HDC1080_SLAVE_ADDR).unwrap();
//! let mut sensor = Hdc1080::new(dev);
//! sensor.read_humidity_start().unwrap();
//! thread::sleep(time::Duration::from_millis(500u64));
//! let humidity = sensor.read_humidity_finish().unwrap();
//! println!("Temperature: {}ºC", humidity);
//! # }
//! ```
//! 
//! 
//! ### Read temperature and relative humidity in sequence
//! 
//! ```no_run
//! extern crate i2cdev;
//! extern crate hdc1080;
//!
//! use i2cdev::linux::LinuxI2CDevice;
//! use std::{thread, time};
//! use hdc1080::{Config, Hdc1080, Mode};
//!
//! const HDC1080_SLAVE_ADDR: u16 = 0x40;
//!
//! # fn main() {
//! let dev = LinuxI2CDevice::new("/dev/i2c-0", HDC1080_SLAVE_ADDR).unwrap();
//! let mut sensor = Hdc1080::new(dev);
//! let mut config: Config = hdc1080::Config::default();
//! config.set_mode(Mode::TAndH);
//! sensor.write_config(config).unwrap();
//! sensor.read_temperature_start().unwrap();
//! thread::sleep(time::Duration::from_millis(500u64));
//! let (temp,humidity) = sensor.read_temperature_humidity_finish().unwrap();
//! println!("Temperature: {}ºC, Humidity: {}RH", temp,humidity);
//! # }
//! ```


#![deny(missing_docs, unsafe_code, warnings)]
#![no_std]

extern crate byteorder;
extern crate i2cdev;

use byteorder::{BigEndian, ByteOrder, ReadBytesExt};
use i2cdev::core::I2CDevice;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// IÂ²C bus error
    I2C(E),
    /// Invalid input data
    InvalidInputData,
}

/// Resolution of temperature
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TResolution {
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}


/// Resolution of humidity
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HResolution {
    /// 8 bits
    _8,
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}

/// Heater operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeaterMode {
    /// Disabled (default)
    Disabled,
    /// Enabled
    Enabled,
}


/// Heater operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    /// Temperature or Humidity (default)
    TOrH,
    /// Temperature and Humidity
    TAndH,
}

struct Register;

impl Register {
    const TEMPERATURE: u8 = 0x00;
    const HUMIDITY: u8 = 0x01;
    const CONFIGURATION: u8 = 0x02;
    const SERIAL_ID1: u8 = 0xFB;
    const SERIAL_ID2: u8 = 0xFC;
    const SERIAL_ID3: u8 = 0xFD;
    const MANUFACTURER: u8 = 0xFE;
    const DEVICE_ID: u8 = 0xFF;
}

struct ConfigBitFlags;

impl ConfigBitFlags {
   const RST : u16 = 0b1000_0000;
   const HEAT: u16 = 0b0010_0000;
   const MODE: u16 = 0b0001_0000;
   const BTST :u16 = 0b0000_1000_0000_0000;
   const T_MODE: u16 = 0b0000_0100;
   const H_MODE9: u16 = 0b000_0010;
   const H_MODE8: u16 = 0b000_0001;
}

/// Config register content to be sent
#[derive(Debug, Clone, Copy)]
pub struct Config {
    bits: u16,
}


impl Default for Config {
    fn default() -> Self {
        Config { bits: 0x00 }
    }
}

impl Config{
    fn with_high(&mut self, mask: u16) {
        self.bits |= mask;
        
    }
    
    fn with_low(&mut self, mask: u16) {
        self.bits &= !mask;
    }

    /// Set heater mode
    pub fn set_heater(&mut self, mode: HeaterMode )
    {
        match mode {
            HeaterMode::Enabled => self.with_high(ConfigBitFlags::HEAT),
            HeaterMode::Disabled => self.with_low(ConfigBitFlags::HEAT),
        };
    }

    /// Enable reset
    pub fn set_reset(&mut self)
    {
        self.with_high(ConfigBitFlags::RST);
    }

    /// Set Temperature or/and Humidity
    pub fn set_mode(&mut self, mode: Mode)
    {
        match mode {
            Mode::TAndH => self.with_high(ConfigBitFlags::MODE),        
            Mode::TOrH => self.with_low(ConfigBitFlags::MODE),
        };
    }

    /// Set temperature resolution
    pub fn set_temp_resolution(&mut self, res: TResolution)
    {
        match res {
            TResolution::_11 => self.with_high(ConfigBitFlags::T_MODE),       
            TResolution::_14 => self.with_low(ConfigBitFlags::T_MODE),
        };
    }

    /// Set humidity resolution
    pub fn set_humidity_resolution(&mut self, res: HResolution)
    {
        match res {
            HResolution::_8 => {self.with_low(ConfigBitFlags::H_MODE8);
                                self.with_high(ConfigBitFlags::H_MODE9);
                                },        
            HResolution::_11 => {self.with_high(ConfigBitFlags::H_MODE9);
                                self.with_low(ConfigBitFlags::H_MODE8);
        },
            HResolution::_14 => {self.with_low(ConfigBitFlags::H_MODE9);
                                self.with_low(ConfigBitFlags::H_MODE8);},
        };
    }

}

/// Hdc1080 device driver.
#[derive(Debug, Default)]
pub struct Hdc1080<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
}

impl<I2C> Hdc1080<I2C>
where
    I2C: I2CDevice,
{
    /// Create new instance of the hdc1080 device.
    pub fn new(i2c: I2C) -> Self {
        Hdc1080 {
            i2c,
            }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    fn read_u16_be_data(&mut self) -> Result<u16, I2C::Error> {
        let mut data = [0u8; 2];
        self.i2c.read(&mut data)?;
        let mut current = &data[..];
        let result = current.read_u16::<BigEndian>().unwrap();
        Ok(result)
    }

    fn compute_temp(&mut self, data: u16) -> f32 {
        f32::from(data) * 165.0 / 65536.0 - 40.0
    }

    fn compute_humidity(&mut self, data: u16) -> f32 {
        f32::from(data) * 100.0 / 65536.0
    }

    /// Read the temperature from the sensor (celsius).
    pub fn read_temperature_finish(&mut self) -> Result<f32, I2C::Error> {
        let data = self.read_u16_be_data().unwrap();
        Ok(self.compute_temp(data))
    }

    /// Read humidity from the sensor (RH%).
    pub fn read_humidity_finish(&mut self) -> Result<f32, I2C::Error> {
        let data = self.read_u16_be_data().unwrap();
        Ok(self.compute_humidity(data))
    }

    /// Read the temperature from the sensor (celsius).
    pub fn read_temperature_humidity_finish(&mut self) -> Result<(f32, f32), I2C::Error> {
        let mut data = [0u8; 4];
        self.i2c.read(&mut data)?;
        let mut current = &data[..];
        let t = current.read_u16::<BigEndian>().unwrap();
        let h = current.read_u16::<BigEndian>().unwrap();
        Ok((self.compute_temp(t), self.compute_humidity(h)))
    }

    /// Read the temperature from the sensor (celsius).
    pub fn read_temperature_start(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(&[Register::TEMPERATURE])
    }

    /// Read the humidity from the sensor (RH).
    pub fn read_humidity_start(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(&[Register::HUMIDITY])
    }

    /// Write config register, 0x02
    pub fn write_config(&mut self,config: Config) -> Result<(), I2C::Error> {
        let write_bits = config.bits & 0xB7;
        self.i2c
            .smbus_write_word_data(Register::CONFIGURATION, write_bits)
    }


    /// Read config, register 0x02
    pub fn read_config(&mut self) -> Result<u16, I2C::Error> {
        let mut data = [0u16; 1];
        data[0] = self
            .i2c
            .smbus_read_word_data(Register::CONFIGURATION)
            .unwrap();
        BigEndian::from_slice_u16(&mut data);
        Ok(data[0])
    }

    /// Returns true if battery voltage is under 2.8v
    pub fn battery_low(&mut self) -> Result<bool, I2C::Error> {
        let config = self.read_config().unwrap();
        Ok((config & ConfigBitFlags::BTST) == ConfigBitFlags::BTST)
    }

    /// Read manufacturer, register 0xfe
    pub fn read_manufacturer(&mut self) -> Result<u16, I2C::Error> {
        let mut data = [0u16; 1];
        data[0] = self
            .i2c
            .smbus_read_word_data(Register::MANUFACTURER)
            .unwrap();
        BigEndian::from_slice_u16(&mut data);
        Ok(data[0])
    }

    /// Read device id, register 0xff
    pub fn read_device_id(&mut self) -> Result<u16, I2C::Error> {
        let mut data = [0u16; 1];
        data[0] = self.i2c.smbus_read_word_data(Register::DEVICE_ID).unwrap();
        BigEndian::from_slice_u16(&mut data);
        Ok(data[0])
    }

    /// Read serial registers 0xfb, 0xfc, 0xfd.
    pub fn read_serial(&mut self) -> Result<u64, I2C::Error> {
        let mut serial = [0u16; 3];
        serial[2] = self.i2c.smbus_read_word_data(Register::SERIAL_ID1).unwrap();
        serial[1] = self.i2c.smbus_read_word_data(Register::SERIAL_ID2).unwrap();
        serial[0] = self.i2c.smbus_read_word_data(Register::SERIAL_ID3).unwrap();
        BigEndian::from_slice_u16(&mut serial);
        let mut result = u64::from(serial[2]) * 65536 * 65536
            + (u64::from(serial[1]) * 65536)
            + u64::from(serial[0]);
        result >>= 7;//this might be an error in TI docs, section 8.6.4. It is 40 bits or 41?
        Ok(result)
    }
}

