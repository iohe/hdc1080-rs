extern crate hdc1080;
extern crate i2cdev;

use i2cdev::linux::LinuxI2CDevice;
use std::{thread, time};
use hdc1080::{Config, Hdc1080, Mode, HeaterMode, TResolution, HResolution};

const HDC1080_SLAVE_ADDR: u16 = 0x40;

fn main() {
    let dev = LinuxI2CDevice::new("/dev/i2c-0", HDC1080_SLAVE_ADDR).unwrap();
    let mut sensor = Hdc1080::new(dev);


    let mut config: Config = hdc1080::Config::default();
    config.set_heater(HeaterMode::Enabled);
    config.set_mode(Mode::TAndH);
    config.set_temp_resolution(TResolution::_11);
    config.set_humidity_resolution(HResolution::_8);
    println!("Bits = {:x}", config.bits);
    sensor.write_config(config).unwrap();

    let config2 = sensor.read_config().unwrap();
    println!("Config = 0x{:x}", config2);

    sensor.read_humidity_start().unwrap();
    thread::sleep(time::Duration::from_millis(500u64));
    let humidity = sensor.read_humidity_finish().unwrap();
    println!("Humidity: {}", humidity);

    sensor.read_temperature_start().unwrap();
    thread::sleep(time::Duration::from_millis(500u64));
    let temp = sensor.read_temperature_finish().unwrap();
    println!("Temperature: {} --- ", temp);

    config.set_heater(HeaterMode::Enabled);
    config.set_mode(Mode::TAndH);
    config.set_temp_resolution(TResolution::_14);
    config.set_humidity_resolution(HResolution::_14);
    println!("Reading TH {:b} --- ", config.bits);

    sensor.write_config(config).unwrap();
    
    thread::sleep(time::Duration::from_millis(500u64));
    
    sensor.read_temperature_start().unwrap();
    thread::sleep(time::Duration::from_millis(500u64));
    let (temp, humidity) = sensor.read_temperature_humidity_finish().unwrap();
    println!("temp = {}, humidity = {}",temp, humidity);

    config.set_mode(Mode::TOrH);
    sensor.write_config(config).unwrap();
    

    let manufacturer = sensor.read_manufacturer().unwrap();
    println!("Manufacturer = 0x{:x}", manufacturer);
    let device_id = sensor.read_device_id().unwrap();
    println!("Device id = 0x{:x}", device_id);
    let serial = sensor.read_serial().unwrap();
    println!("Serial = 0x{:x}", serial);

    let config2 = sensor.read_config().unwrap();
    println!("Config = 0x{:x}", config2);

    let bat = sensor.battery_low().unwrap();
    if bat { println!("Batter low"); }
}
