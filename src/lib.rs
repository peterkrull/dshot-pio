#![no_std]

#[cfg(feature = "embassy-rp")]
pub mod dshot_embassy_rp;

#[cfg(feature = "rp2040-hal")]
pub mod dshot_rp2040_hal;

pub trait DshotPioTrait<const N: usize> {
    fn reverse(&mut self, reverse: [bool;N]);
    fn throttle_clamp(&mut self, throttle: [u16;N]);
    fn throttle_minimum(&mut self);
}