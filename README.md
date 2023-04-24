# Dshot implementation for RP2040 using PIO

This crate utilizes a single PIO block of the RP2040, enable it to send Dshot commands to four ESCs simultaneously, making it a great fit for quad-copters. The crate supports both the `embassy-rp` and  `rp2040-hal` hardware abstraction layers (HAL). To use either HAL, the corresponding feature *must* be enabled, using either feature below.

```toml
quad-dshot-pio = { git = "https://github.com/peterkrull/quad-dshot-pio", features = ["embassy-rp"] }
quad-dshot-pio = { git = "https://github.com/peterkrull/quad-dshot-pio", features = ["rp2040-hal"] }
```
Then creating the `QuadShotPio` struct is simply a matter of giving the constructor the PIO block to use, as well as which four pins to use. The last argument is the clock divider. In order to get reliable transmission to the ESCs, it is important to set the clock divider correctly. The formula for doing so is the following, where *dshot speed* is the number indicating speed, eg 150, 300, 600 and so on. The system clock can vary from board to board, but generally 120 Mhz to 133 Mhz is common.

$$\text{clock divider} = \frac { \text{system clock} }{8 \cdot \text{dshot speed} \cdot 1000} $$

This clock divider is passed to the constructor in two parts, consisting of the integer part, and the fraction. Generally stuff after the decimal point of the *clock divider* can be ignored, meaning that is should be good enough to pass only the integer part. Otherwise the remainder should be passed as `(remainder * 256 ) as u8`

---

## Construction

Constructing the motor struct looks slightly different, depending on whether the `embassy-rp` or  `rp2040-hal` HAL is used. The functionality of the resulting struct is shared through a trait.

```rust
use quad_dshot_pio::rp2040_hal::*;
let quad_motors_rp2040_hal = QuadDshotPio::new(
    pac.PIO0,
    &mut pac.RESETS,
    pins.gpio13.into_mode(),
    pins.gpio7.into_mode(),
    pins.gpio6.into_mode(),
    pins.gpio12.into_mode(),
    (52, 0) // clock divider
);
```

```rust
use quad_dshot_pio::embassy_rp::*;
let quad_motors_embassy_rp = QuadDshotPio::new(
    p.PIO0.split(),
    p.PIN_13.degrade(),
    p.PIN_7.degrade(),
    p.PIN_6.degrade(),
    p.PIN_12.degrade(),
    (52, 0) // clock divider
);
```
