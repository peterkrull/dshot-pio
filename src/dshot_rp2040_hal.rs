use dshot_encoder as dshot;
pub use super::DshotPioTrait;

use rp2040_hal::{
    gpio::{PinId, AnyPin},
    pio::{PIOExt, PinDir, Tx, SM0, SM1, SM2, SM3, PIOBuilder},
    pac::RESETS,
};

pub struct DshotPio<const N : usize, P: PIOExt> {
    sm0: Tx<(P, SM0)>,
    sm1: Tx<(P, SM1)>,
    sm2: Tx<(P, SM2)>,
    sm3: Tx<(P, SM3)>
}

fn configure_pio_instance<P: PIOExt>(
    pio_block: P,
    resets: &mut RESETS,
) -> (rp2040_hal::pio::InstalledProgram<P>, rp2040_hal::pio::UninitStateMachine<(P, SM0)>, rp2040_hal::pio::UninitStateMachine<(P, SM1)>, rp2040_hal::pio::UninitStateMachine<(P, SM2)>, rp2040_hal::pio::UninitStateMachine<(P, SM3)>) {
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
    (
        pio.install(&dshot_pio_program.program).expect("Unable to install program into PIO block"),
        sm0, sm1, sm2, sm3
    )
}

///
/// Defining constructor functions
/// 

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<1,P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0 : impl AnyPin + PinId,
        clk_div: (u16, u8),
    ) -> DshotPio<1,P> {

        // Install dshot program into PIO block
        let (installed,sm0,sm1,sm2,sm3) = configure_pio_instance(pio_block, resets);

        // Configure the four state machines
        let (mut sm0x, _, tx0) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin0.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm0);
        sm0x.set_pindirs([(pin0.as_dyn().num, PinDir::Output)]);
        sm0x.start();

        // Setup dummy program for unused state machine
        let (_, _, tx1) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm1);
        let (_, _, tx2) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm2);
        let (_, _, tx3) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm3);

        // Return struct of four configured DSHOT state machines
        DshotPio { sm0: tx0, sm1: tx1, sm2: tx2, sm3: tx3 }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<2,P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0 : impl AnyPin + PinId,
        pin1 : impl AnyPin + PinId,
        clk_div: (u16, u8),
    ) -> DshotPio<2,P> {

        // Install dshot program into PIO block
        let (installed,sm0,sm1,sm2,sm3) = configure_pio_instance(pio_block, resets);

        // Configure the four state machines
        let (mut sm0x, _, tx0) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin0.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm0);
        sm0x.set_pindirs([(pin0.as_dyn().num, PinDir::Output)]);
        sm0x.start();

        let (mut sm1x, _, tx1) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin1.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm1);
        sm1x.set_pindirs([(pin1.as_dyn().num, PinDir::Output)]);
        sm1x.start();

        // Setup dummy program for unused state machine
        let (_, _, tx2) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm2);
        let (_, _, tx3) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm3);

        // Return struct of four configured DSHOT state machines
        DshotPio { sm0: tx0, sm1: tx1, sm2: tx2, sm3: tx3 }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<3,P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0 : impl AnyPin + PinId,
        pin1 : impl AnyPin + PinId,
        pin2 : impl AnyPin + PinId,
        clk_div: (u16, u8),
    ) -> DshotPio<3,P> {

        // Install dshot program into PIO block
        let (installed,sm0,sm1,sm2,sm3) = configure_pio_instance(pio_block, resets);

        // Configure the four state machines
        let (mut sm0x, _, tx0) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin0.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm0);
        sm0x.set_pindirs([(pin0.as_dyn().num, PinDir::Output)]);
        sm0x.start();

        let (mut sm1x, _, tx1) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin1.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm1);
        sm1x.set_pindirs([(pin1.as_dyn().num, PinDir::Output)]);
        sm1x.start();

        let (mut sm2x, _, tx2) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin2.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm2);
        sm2x.set_pindirs([(pin2.as_dyn().num, PinDir::Output)]);
        sm2x.start();

        // Setup dummy program for unused state machine
        let (_, _, tx3) = PIOBuilder::from_program(unsafe { installed.share() }).build(sm3);

        // Return struct of four configured DSHOT state machines
        DshotPio { sm0: tx0, sm1: tx1, sm2: tx2, sm3: tx3 }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<4,P> {
    pub fn new(
        pio_block: P,
        resets: &mut RESETS,
        pin0 : impl AnyPin + PinId,
        pin1 : impl AnyPin + PinId,
        pin2 : impl AnyPin + PinId,
        pin3 : impl AnyPin + PinId,
        clk_div: (u16, u8),
    ) -> DshotPio<4,P> {

        // Install dshot program into PIO block
        let (installed,sm0,sm1,sm2,sm3) = configure_pio_instance(pio_block, resets);

        // Configure the four state machines
        let (mut sm0x, _, tx0) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin0.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm0);
        sm0x.set_pindirs([(pin0.as_dyn().num, PinDir::Output)]);
        sm0x.start();

        let (mut sm1x, _, tx1) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin1.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm1);
        sm1x.set_pindirs([(pin1.as_dyn().num, PinDir::Output)]);
        sm1x.start();

        let (mut sm2x, _, tx2) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin2.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm2);
        sm2x.set_pindirs([(pin2.as_dyn().num, PinDir::Output)]);
        sm2x.start();

        let (mut sm3x, _, tx3) =
            PIOBuilder::from_program(unsafe { installed.share() })
                .set_pins(pin3.as_dyn().num, 1)
                .clock_divisor_fixed_point(clk_div.0, clk_div.1)
                .pull_threshold(32)
                .autopull(true)
                .build(sm3);
        sm3x.set_pindirs([(pin3.as_dyn().num, PinDir::Output)]);
        sm3x.start();

        // Return struct of four configured DSHOT state machines
        DshotPio { sm0: tx0, sm1: tx1, sm2: tx2, sm3: tx3 }
    }
}

///
/// Implementing DshotPioTrait
/// 

impl<P: PIOExt> super::DshotPioTrait<1> for DshotPio<1,P> {

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool;1]) {
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;1])  {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<2> for DshotPio<2,P> {

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool;2]) {
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
        self.sm1.write(dshot::reverse(reverse[1]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;2]) {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
        self.sm1.write(dshot::throttle_clamp(throttle[1], false) as u32);
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self){
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<3> for DshotPio<3,P> {

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool;3]){
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
        self.sm1.write(dshot::reverse(reverse[1]) as u32);
        self.sm2.write(dshot::reverse(reverse[2]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;3]) {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
        self.sm1.write(dshot::throttle_clamp(throttle[1], false) as u32);
        self.sm2.write(dshot::throttle_clamp(throttle[2], false) as u32);
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
        self.sm2.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<4> for DshotPio<4,P> {

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool;4]) {
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
        self.sm1.write(dshot::reverse(reverse[1]) as u32);
        self.sm2.write(dshot::reverse(reverse[2]) as u32);
        self.sm3.write(dshot::reverse(reverse[3]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;4]) {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
        self.sm1.write(dshot::throttle_clamp(throttle[1], false) as u32);
        self.sm2.write(dshot::throttle_clamp(throttle[2], false) as u32);
        self.sm3.write(dshot::throttle_clamp(throttle[3], false) as u32);
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
        self.sm2.write(dshot::throttle_minimum(false) as u32);
        self.sm3.write(dshot::throttle_minimum(false) as u32);
    }
}
