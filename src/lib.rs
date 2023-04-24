#![no_std]

#[cfg(feature = "embassy-rp")]
pub mod dshot_embassy_rp;

#[cfg(feature = "rp2040-hal")]
pub mod dshot_rp2040_hal;

pub trait QuadDshotTrait {
    fn reverse(&mut self, reverse: (bool, bool, bool, bool)) -> (bool, bool, bool, bool);
    fn throttle_clamp(&mut self, throttle: (u16, u16, u16, u16)) -> (bool, bool, bool, bool);
    fn throttle_minimum(&mut self) -> (bool, bool, bool, bool);
}