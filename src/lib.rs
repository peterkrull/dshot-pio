use rp2040_hal as hal;
use hal::gpio::{Pin, PinId, ValidPinMode, Function, FunctionConfig, bank0::BankPinId};
use hal::pio::{PIOExt, PinDir, Tx, SM0, SM1, SM2, SM3};
use dshot_encoder as dshot;
use hal::pac;


#[allow(dead_code)]
pub struct QuadDshotPio <P : PIOExt + FunctionConfig> {
    pub motor0 : Tx<(P,SM0)>,
    pub motor1 : Tx<(P,SM1)>,
    pub motor2 : Tx<(P,SM2)>,
    pub motor3 : Tx<(P,SM3)>,
}

#[allow(dead_code)]
impl<P : PIOExt + FunctionConfig> QuadDshotPio<P> {
    pub fn new<I0,I1,I2,I3>(
        pio_block: P,
        resets : & mut pac::RESETS,
        _pin0 : Pin<I0,Function<P>>,
        _pin1 : Pin<I1,Function<P>>,
        _pin2 : Pin<I2,Function<P>>,
        _pin3 : Pin<I3,Function<P>>,
        clk_div : (u16,u8)
    ) -> Self
    where
        I0 : PinId+BankPinId,
        I1 : PinId+BankPinId,
        I2 : PinId+BankPinId,
        I3 : PinId+BankPinId,
        Function<P> : ValidPinMode<I0>,
        Function<P> : ValidPinMode<I1>,
        Function<P> : ValidPinMode<I2>,
        Function<P> : ValidPinMode<I3>,
    {

        // Split the PIO block into individual state machines
        let (
            mut pio, 
            sm0,
            sm1,
            sm2,
            sm3)
            = pio_block.split(resets);

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

        I0::DYN.num;

        // Install dshot program into PIO block
        let installed = pio.install(&dshot_pio_program.program).unwrap();

        // Configure the four state machines
        let (mut sm0x, _, tx0)=
            rp2040_hal::pio::PIOBuilder::from_program( unsafe { installed.share() } )
            .set_pins(I0::DYN.num, 1)
            .clock_divisor_fixed_point(clk_div.0,clk_div.1)
            .pull_threshold(32)
            .autopull(true)
            .build(sm0);
        sm0x.set_pindirs([(I0::DYN.num, PinDir::Output)]);
        sm0x.start();
        
        let (mut sm1x, _, tx1) =
            rp2040_hal::pio::PIOBuilder::from_program( unsafe { installed.share() } )
            .set_pins(I1::DYN.num, 1)
            .clock_divisor_fixed_point(clk_div.0,clk_div.1)
            .pull_threshold(32)
            .autopull(true)
            .build(sm1);
        sm1x.set_pindirs([(I1::DYN.num, PinDir::Output)]);
        sm1x.start();
        
        let (mut sm2x, _, tx2) =
            rp2040_hal::pio::PIOBuilder::from_program( unsafe { installed.share() } )
            .set_pins(I2::DYN.num, 1)
            .clock_divisor_fixed_point(clk_div.0,clk_div.1)
            .pull_threshold(32)
            .autopull(true)
            .build(sm2);
        sm2x.set_pindirs([(I2::DYN.num, PinDir::Output)]);
        sm2x.start();
        
        let (mut sm3x, _, tx3) =
            rp2040_hal::pio::PIOBuilder::from_program( unsafe { installed.share() } )
            .set_pins(I3::DYN.num, 1)
            .clock_divisor_fixed_point(clk_div.0,clk_div.1)
            .pull_threshold(32)
            .autopull(true)
            .build(sm3);
        sm3x.set_pindirs([(I3::DYN.num, PinDir::Output)]);
        sm3x.start();

        // Return struct of four configured DSHOT state machines
        QuadDshotPio{ motor0: tx0, motor1: tx1, motor2: tx2, motor3: tx3 }
    }

    pub fn reverse(& mut self,reverse_0:bool,reverse_1:bool,reverse_2:bool,reverse_3:bool) -> (bool,bool,bool,bool) {
        (
            self.motor0.write( dshot::reverse(reverse_0) as u32 ),
            self.motor1.write( dshot::reverse(reverse_1) as u32 ),
            self.motor2.write( dshot::reverse(reverse_2) as u32 ),
            self.motor3.write( dshot::reverse(reverse_3) as u32 )
        )
    }

    pub fn throttle_clamp(& mut self,throttle_0:u16,throttle_1:u16,throttle_2:u16,throttle_3:u16) -> (bool,bool,bool,bool) {
        (
            self.motor0.write( dshot::throttle_clamp(throttle_0, false) as u32 ),
            self.motor1.write( dshot::throttle_clamp(throttle_1, false) as u32 ),
            self.motor2.write( dshot::throttle_clamp(throttle_2, false) as u32 ),
            self.motor3.write( dshot::throttle_clamp(throttle_3, false) as u32 )
        )
    }

    pub fn throttle_minimum(& mut self) -> (bool,bool,bool,bool) {
        (
            self.motor0.write( dshot::throttle_minimum(false) as u32 ),
            self.motor1.write( dshot::throttle_minimum(false) as u32 ),
            self.motor2.write( dshot::throttle_minimum(false) as u32 ),
            self.motor3.write( dshot::throttle_minimum(false) as u32 )
        )
    }
}