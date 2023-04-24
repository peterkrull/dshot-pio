use dshot_encoder as dshot;
pub use super::QuadDshotTrait;

use embassy_rp::{
    gpio::{AnyPin, Pin},
    pio::{PioStateMachine, PioStateMachineInstance as PSMI, PioInstance, SmInstance, PioCommonInstance, ShiftDirection},
    pio_instr_util,
    relocate::RelocatedProgram
};

#[allow(dead_code)]
pub struct QuadDshotPio<PIO,SM0,SM1,SM2,SM3>
where
    PIO : PioInstance,
    SM0 : SmInstance,
    SM1 : SmInstance,
    SM2 : SmInstance,
    SM3 : SmInstance,
{
    pub motor: ( PSMI<PIO,SM0>, PSMI<PIO,SM1>, PSMI<PIO,SM2>, PSMI<PIO,SM3> )
}


#[allow(dead_code)]
impl <PIO,SM0,SM1,SM2,SM3> QuadDshotPio<PIO,SM0,SM1,SM2,SM3>
where
    PIO : PioInstance,
    SM0 : SmInstance,
    SM1 : SmInstance,
    SM2 : SmInstance,
    SM3 : SmInstance,
{
    pub fn new(
        pio_split: (PioCommonInstance<PIO>, PSMI<PIO,SM0>, PSMI<PIO,SM1>, PSMI<PIO,SM2>, PSMI<PIO,SM3> ),
        pin0: AnyPin, pin1: AnyPin, pin2: AnyPin, pin3: AnyPin,
        clk_div: (u16, u8),
    ) -> QuadDshotPio<PIO,SM0,SM1,SM2,SM3> {

        let (_,sm0,sm1,sm2,sm3) = pio_split;
        let mut sm = (sm0,sm1,sm2,sm3);

        let dshot_pio_program = pio_proc::pio_asm!(
            "set pindirs, 1",
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

        let relocated = RelocatedProgram::new(&dshot_pio_program.program);
        let pio::Wrap { source, target } = relocated.wrap();

        sm.0.write_instr(relocated.origin() as usize, relocated.code());

        pio_instr_util::exec_jmp(&mut sm.0, relocated.origin());
        sm.0.set_set_range(pin0.pin(), 1);
        sm.0.set_set_pins(&[&sm.0.make_pio_pin(pin0)]);
        sm.0.set_clkdiv((( clk_div.0 as u32 ) << 8) | (( clk_div.1 as u32 )) );
        sm.0.set_wrap(source, target);
        sm.0.set_pull_threshold(32);
        sm.0.set_autopull(true);
        sm.0.set_out_shift_dir(ShiftDirection::Left);
        sm.0.set_enable(true);

        pio_instr_util::exec_jmp(&mut sm.1, relocated.origin());
        sm.1.set_set_range(pin1.pin(), 1);
        sm.1.set_set_pins(&[&sm.1.make_pio_pin(pin1)]);
        sm.1.set_clkdiv((( clk_div.0 as u32 ) << 8) | (( clk_div.1 as u32 )) );
        sm.1.set_wrap(source, target);
        sm.1.set_pull_threshold(32);
        sm.1.set_autopull(true);
        sm.1.set_out_shift_dir(ShiftDirection::Left);
        sm.1.set_enable(true);

        pio_instr_util::exec_jmp(&mut sm.2, relocated.origin());
        sm.2.set_set_range(pin2.pin(), 1);
        sm.2.set_set_pins(&[&sm.2.make_pio_pin(pin2)]);
        sm.2.set_clkdiv((( clk_div.0 as u32 ) << 8) | (( clk_div.1 as u32 )) );
        sm.2.set_wrap(source, target);
        sm.2.set_pull_threshold(32);
        sm.2.set_autopull(true);
        sm.2.set_out_shift_dir(ShiftDirection::Left);
        sm.2.set_enable(true);

        pio_instr_util::exec_jmp(&mut sm.3, relocated.origin());
        sm.3.set_set_range(pin3.pin(), 1);
        sm.3.set_set_pins(&[&sm.3.make_pio_pin(pin3)]);
        sm.3.set_clkdiv((( clk_div.0 as u32 ) << 8) | (( clk_div.1 as u32 )) );
        sm.3.set_wrap(source, target);
        sm.3.set_pull_threshold(32);
        sm.3.set_autopull(true);
        sm.3.set_out_shift_dir(ShiftDirection::Left);
        sm.3.set_enable(true);

        // Return struct of four configured DSHOT state machines
        QuadDshotPio { motor : sm }
    }

}
impl<PIO,SM0,SM1,SM2,SM3>  super::QuadDshotTrait for QuadDshotPio<PIO,SM0,SM1,SM2,SM3>
where
    PIO : PioInstance,
    SM0 : SmInstance,
    SM1 : SmInstance,
    SM2 : SmInstance,
    SM3 : SmInstance,
{

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: (bool, bool, bool, bool)) -> (bool, bool, bool, bool) {
        (
            self.motor.0.try_push_tx(dshot::reverse(reverse.0) as u32),
            self.motor.1.try_push_tx(dshot::reverse(reverse.1) as u32),
            self.motor.2.try_push_tx(dshot::reverse(reverse.2) as u32),
            self.motor.3.try_push_tx(dshot::reverse(reverse.3) as u32),
        )
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: (u16, u16, u16, u16)) -> (bool, bool, bool, bool) {
        (
            self.motor.0.try_push_tx(dshot::throttle_clamp(throttle.0, false) as u32),
            self.motor.1.try_push_tx(dshot::throttle_clamp(throttle.1, false) as u32),
            self.motor.2.try_push_tx(dshot::throttle_clamp(throttle.2, false) as u32),
            self.motor.3.try_push_tx(dshot::throttle_clamp(throttle.3, false) as u32),
        )
    }

    /// Set the throttle for each motor to zero (Dshot command 48)
    fn throttle_minimum(&mut self) -> (bool, bool, bool, bool) {
        (
            self.motor.0.try_push_tx(dshot::throttle_minimum(false) as u32),
            self.motor.1.try_push_tx(dshot::throttle_minimum(false) as u32),
            self.motor.2.try_push_tx(dshot::throttle_minimum(false) as u32),
            self.motor.3.try_push_tx(dshot::throttle_minimum(false) as u32),
        )
    }
}