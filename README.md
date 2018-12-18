# Rust HDC1080 Temperature and Humidity Sensor

[![crates.io](https://img.shields.io/crates/v/hdc1080.svg)](https://crates.io/crates/hdc1080)
[![Docs](https://docs.rs/hdc1080/badge.svg)](https://docs.rs/hdc1080)
[![Build Status](https://travis-ci.org/iohe/hdc1080-rs.svg?branch=master)](https://travis-ci.org/iohe/hdc1080-rs)
[![Coverage Status](https://coveralls.io/repos/github/iohe/hdc1080-rs/badge.svg?branch=master)](https://coveralls.io/github/iohe/hdc1080-rs?branch=master)
![Maintenance Intention](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg)

This is based on LM75 work of Diego Barrios Romero

This is a platform agnostic Rust driver for the HDC1080 temperature and humidity
sensor, based on the
[`I2CDevice`](https://github.com/rust-embedded/rust-i2cdev/) traits.


This driver allows you to:
- Enable/disable the device.
- Read the temperature.

## The device
The HDC1080 temperature sensor includes a delta-sigma analog-to-digital
converter, and a digital overtemperature detector. The host can
query the HDC1080 through its I2C interface to read temperature at any
time. 

Datasheet:
- [HDC1080](http://www.ti.com/lit/ds/symlink/hdc1080.pdf)

### Usage

```rust
extern crate i2cdev;
extern crate hdc1080;

use hal::I2cdev;
use hdc1080::{ Hdc1080, SlaveAddr };

const HDC1080_SLAVE_ADDR: u16 = 0x40;

fn main() {
    let dev = LinuxI2CDevice::new("/dev/i2c-0", HDC1080_SLAVE_ADDR).unwrap();
    let mut sensor = Hdc1080::new(dev);

    sensor.read_temperature_start().unwrap();
    thread::sleep(time::Duration::from_millis(500u64));
    let temp_celsius = sensor.read_temperature_finish().unwrap();
    println!("Temperature: {}ºC", temp_celsius);
}
```

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

