use dshot_encoder as dshot;
pub use super::QuadDshotTrait;

use rp2040_hal::{
    gpio::{bank0::BankPinId, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{PIOExt, PinDir, Tx, SM0, SM1, SM2, SM3, PIOBuilder},
    pac::RESETS,
};

#[allow(dead_code)]
pub struct QuadDshotPio<P: PIOExt + FunctionConfig> {
    pub motor: ( Tx<(P, SM0)>, Tx<(P, SM1)>, Tx<(P, SM2)>, Tx<(P, SM3)> )
}

#[allow(dead_code)]
impl<P: PIOExt + FunctionConfig> QuadDshotPio<P> {
    pub fn new<I0, I1, I2, I3>(
        pio_block: P,
        resets: &mut RESETS,
        pin0 : Pin<I0, Function<P>>,
        pin1 : Pin<I1, Function<P>>,
        pin2 : Pin<I2, Function<P>>,
        pin3 : Pin<I3, Function<P>>,
        clk_div: (u16, u8),
    ) -> QuadDshotPio<P>
    where
        I0: PinId + BankPinId,
        I1: PinId + BankPinId,
        I2: PinId + BankPinId,
        I3: PinId + BankPinId,
        Function<P>: ValidPinMode<I0>,
        Function<P>: ValidPinMode<I1>,
        Function<P>: ValidPinMode<I2>,
        Function<P>: ValidPinMode<I3>,
    {
        // Split the PIO block into individual state machines
        let (mut pio, sm0, sm1, sm2, sm3) = pio_block.split(resets);

        // Program that generates DSHOT signal in PIO state machine
        let dshot_pio_program = pio_proc::pio_asm!(
            "entry:"
            "   pull"
            "   out null 16"
            "   set x 15"
            "loop:"
            "   set pins 1"
            "   out y 1"
            "   jmp !y zero"
            "   nop [2]"
            "one:" // 6 and 2
            "   set pins 0"
            "   jmp x-- loop"
            "   jmp reset"
            "zero:" // 3 and 5
            "   set pins 0 [3]"
            "   jmp x-- loop"
            "   jmp reset"
            "reset:" // Blank frame
            "   nop [31]"
            "   nop [31]"
            "   nop [31]"
            "   jmp entry [31]"
        );

        // Install dshot program into PIO block
        let installed = pio.install(&dshot_pio_program.program).expect("Unable to install program into PIO block");

        // Configure the four state machines
        let (mut sm0x, _, tx0) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin0.id().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm0);
        sm0x.set_pindirs([(pin0.id().num, PinDir::Output)]);
        sm0x.start();

        let (mut sm1x, _, tx1) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin1.id().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm1);
        sm1x.set_pindirs([(pin1.id().num, PinDir::Output)]);
        sm1x.start();

        let (mut sm2x, _, tx2) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin2.id().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm2);
        sm2x.set_pindirs([(pin2.id().num, PinDir::Output)]);
        sm2x.start();

        let (mut sm3x, _, tx3) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin3.id().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm3);
        sm3x.set_pindirs([(pin3.id().num, PinDir::Output)]);
        sm3x.start();

        // Return struct of four configured DSHOT state machines
        QuadDshotPio { motor: ( tx0, tx1, tx2, tx3 ) }
    }
}
impl<P: PIOExt + FunctionConfig> super::QuadDshotTrait for QuadDshotPio<P> {

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: (bool, bool, bool, bool)) -> (bool, bool, bool, bool) {
        (
            self.motor.0.write(dshot::reverse(reverse.0) as u32),
            self.motor.1.write(dshot::reverse(reverse.1) as u32),
            self.motor.2.write(dshot::reverse(reverse.2) as u32),
            self.motor.3.write(dshot::reverse(reverse.3) as u32),
        )
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: (u16, u16, u16, u16)) -> (bool, bool, bool, bool) {
        (
            self.motor.0.write(dshot::throttle_clamp(throttle.0, false) as u32),
            self.motor.1.write(dshot::throttle_clamp(throttle.1, false) as u32),
            self.motor.2.write(dshot::throttle_clamp(throttle.2, false) as u32),
            self.motor.3.write(dshot::throttle_clamp(throttle.3, false) as u32),
        )
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self) -> (bool, bool, bool, bool) {
        (
            self.motor.0.write(dshot::throttle_minimum(false) as u32),
            self.motor.1.write(dshot::throttle_minimum(false) as u32),
            self.motor.2.write(dshot::throttle_minimum(false) as u32),
            self.motor.3.write(dshot::throttle_minimum(false) as u32),
        )
    }
}


