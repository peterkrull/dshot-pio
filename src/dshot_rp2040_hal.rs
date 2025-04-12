pub use super::DshotPioTrait;
use dshot_encoder as dshot;

use rp2040_hal::{
    gpio::{Function, Pin, PinId, PullType},
    pac::RESETS,
    pio::{
        InstalledProgram, PIOBuilder, PIOExt, PinDir, ShiftDirection, StateMachineIndex, Tx,
        UninitStateMachine, SM0, SM1, SM2, SM3,
    },
};

pub struct DshotPio<const N: usize, P: PIOExt> {
    sm0: Tx<(P, SM0)>,
    sm1: Tx<(P, SM1)>,
    sm2: Tx<(P, SM2)>,
    sm3: Tx<(P, SM3)>,
}

fn configure_pio_instance<P: PIOExt>(
    pio_block: P,
    resets: &mut RESETS,
) -> (
    InstalledProgram<P>,
    (
        UninitStateMachine<(P, SM0)>,
        UninitStateMachine<(P, SM1)>,
        UninitStateMachine<(P, SM2)>,
        UninitStateMachine<(P, SM3)>,
    ),
) {
    // Split the PIO block into individual state machines
    let (mut pio, sm0, sm1, sm2, sm3) = pio_block.split(resets);

    // Program that generates DShot signal in PIO state machine
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

    // Install DShot program into PIO block
    (
        pio.install(&dshot_pio_program.program)
            .expect("Unable to install program into PIO block"),
        (sm0, sm1, sm2, sm3),
    )
}

///
/// Defining constructor functions
///

fn setup_state_machine<P: PIOExt, SM: StateMachineIndex>(
    installed: &InstalledProgram<P>,
    sm: UninitStateMachine<(P, SM)>,
    clk_div: (u16, u8),
    pin: impl PinId,
) -> Tx<(P, SM)> {
    // SAFETY: We never uninstall the program, so all unsafety considerations are met
    let (mut smx, _, tx) = PIOBuilder::from_installed_program(unsafe { installed.share() })
        .set_pins(pin.as_dyn().num, 1)
        .clock_divisor_fixed_point(clk_div.0, clk_div.1)
        .out_shift_direction(ShiftDirection::Left)
        .pull_threshold(32)
        .autopull(true)
        .build(sm);

    smx.set_pindirs([(pin.as_dyn().num, PinDir::Output)]);
    smx.start(); // NOTE: This consumes the state machine
    tx
}

fn dummy_state_machine<P: PIOExt, SM: StateMachineIndex>(
    installed: &InstalledProgram<P>,
    sm: UninitStateMachine<(P, SM)>,
) -> Tx<(P, SM)> {
    let (_, _, tx) = PIOBuilder::from_installed_program(unsafe { installed.share() }).build(sm);
    tx
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<1, P> {
    pub fn new<FN: Function, PT: PullType>(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId, FN, PT>,
        clk_div: (u16, u8),
    ) -> DshotPio<1, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0.id());

        // Setup dummy program for unused state machines
        let tx1 = dummy_state_machine(&installed, sm.1);
        let tx2 = dummy_state_machine(&installed, sm.2);
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<2, P> {
    pub fn new<FN: Function, PT: PullType>(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId, FN, PT>,
        pin1: Pin<impl PinId, FN, PT>,
        clk_div: (u16, u8),
    ) -> DshotPio<2, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0.id());
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1.id());

        // Setup dummy program for unused state machines
        let tx2 = dummy_state_machine(&installed, sm.2);
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<3, P> {
    pub fn new<FN: Function, PT: PullType>(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId, FN, PT>,
        pin1: Pin<impl PinId, FN, PT>,
        pin2: Pin<impl PinId, FN, PT>,
        clk_div: (u16, u8),
    ) -> DshotPio<3, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0.id());
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1.id());
        let tx2 = setup_state_machine(&installed, sm.2, clk_div, pin2.id());

        // Setup dummy program for unused state machines
        let tx3 = dummy_state_machine(&installed, sm.3);

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
        }
    }
}

#[allow(dead_code)]
impl<P: PIOExt> DshotPio<4, P> {
    pub fn new<FN: Function, PT: PullType>(
        pio_block: P,
        resets: &mut RESETS,
        pin0: Pin<impl PinId, FN, PT>,
        pin1: Pin<impl PinId, FN, PT>,
        pin2: Pin<impl PinId, FN, PT>,
        pin3: Pin<impl PinId, FN, PT>,
        clk_div: (u16, u8),
    ) -> DshotPio<4, P> {
        // Install DShot program into PIO block
        let (installed, sm) = configure_pio_instance(pio_block, resets);

        // Configure the state machine
        let tx0 = setup_state_machine(&installed, sm.0, clk_div, pin0.id());
        let tx1 = setup_state_machine(&installed, sm.1, clk_div, pin1.id());
        let tx2 = setup_state_machine(&installed, sm.2, clk_div, pin2.id());
        let tx3 = setup_state_machine(&installed, sm.3, clk_div, pin3.id());

        // Return struct of four configured DShot state machines
        DshotPio {
            sm0: tx0,
            sm1: tx1,
            sm2: tx2,
            sm3: tx3,
        }
    }
}

///
/// Implementing DshotPioTrait
///

impl<P: PIOExt> super::DshotPioTrait<1> for DshotPio<1, P> {
    /// Send any valid DShot value to the ESC.
    fn command(&mut self, command: [u16; 1]) {
        self.sm0.write(command[0].min(dshot::THROTTLE_MAX) as u32);
    }

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool; 1]) {
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;1])  {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
    }

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<2> for DshotPio<2, P> {
    /// Send any valid DShot value to the ESC.
    fn command(&mut self, command: [u16; 2]) {
        self.sm0.write(command[0].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[1].min(dshot::THROTTLE_MAX) as u32);
    }

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool; 2]) {
        self.sm0.write(dshot::reverse(reverse[0]) as u32);
        self.sm1.write(dshot::reverse(reverse[1]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16;2]) {
        self.sm0.write(dshot::throttle_clamp(throttle[0], false) as u32);
        self.sm1.write(dshot::throttle_clamp(throttle[1], false) as u32);
    }

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<3> for DshotPio<3, P> {
    /// Send any valid DShot value to the ESC.
    fn command(&mut self, command: [u16; 3]) {
        self.sm0.write(command[0].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[1].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[2].min(dshot::THROTTLE_MAX) as u32);
    }

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool; 3]) {
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

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
        self.sm2.write(dshot::throttle_minimum(false) as u32);
    }
}

impl<P: PIOExt> super::DshotPioTrait<4> for DshotPio<4, P> {
    /// Send any valid DShot value to the ESC.
    fn command(&mut self, command: [u16; 4]) {
        self.sm0.write(command[0].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[1].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[2].min(dshot::THROTTLE_MAX) as u32);
        self.sm0.write(command[3].min(dshot::THROTTLE_MAX) as u32);
    }

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool; 4]) {
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

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&mut self) {
        self.sm0.write(dshot::throttle_minimum(false) as u32);
        self.sm1.write(dshot::throttle_minimum(false) as u32);
        self.sm2.write(dshot::throttle_minimum(false) as u32);
        self.sm3.write(dshot::throttle_minimum(false) as u32);
    }
}
