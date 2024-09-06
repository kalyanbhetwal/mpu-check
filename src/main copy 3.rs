#![no_main]
#![no_std]
#![feature(naked_functions)]
use panic_halt as _;

use core::sync::atomic::{AtomicUsize, Ordering};


// TODO(6) Import your HAL
use stm32f3xx_hal_v2 as _; // memory layout
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SCB;
use cortex_m_rt::ExceptionFrame;
use cortex_m_rt::exception;
use cortex_m_semihosting::{hprintln,debug}; // Import the `hprintln!` macro
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use core::arch::asm;
use core::ptr;
use cortex_m::peripheral::{SYST, NVIC};
use cortex_m::peripheral::CPUID;
use cortex_m::asm;

use stm32f3xx_hal_v2::interrupt;

use stm32f3xx_hal_v2::{pac::Peripherals, pac::Interrupt, timer::{Event, Timer}};

use stm32f3xx_hal_v2::{pac, prelude::*};
#[entry]
fn main() -> ! {
     // Setup and initialization code here
     let dp = Peripherals::take().unwrap();
     // Set up the system clock
     let mut flash = dp.FLASH.constrain();
     let mut rcc = dp.RCC.constrain();
     let clocks = rcc.cfgr.freeze(&mut flash.acr);
 
     let mut timer = Timer::tim2(dp.TIM2, 5.hz(), clocks, &mut rcc.apb1);
     timer.listen(Event::Update);
     cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
     unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::TIM2) };

    loop{
        hprintln!("rest").unwrap();
       // debug::exit(debug::EXIT_SUCCESS);
    }
}

#[naked]
#[no_mangle]
#[export_name = "TIM2"]
pub unsafe extern "C" fn __cortex_m_rt_TIM2_trampoline() {
   //unsafe{asm!("NOP", options(noreturn))};
   unsafe{asm!("cpsid i; nop; mov r0,sp; mov r1, lr; bl _timer2;", options(noreturn))};
}
// #[export_name = "TIM2"]
// pub unsafe extern "C" fn __cortex_m_rt_TIM2_trampoline() {
//     hprintln!("rest").unwrap();
// }

#[no_mangle]
pub unsafe extern "C" fn _timer2(){
    let mut r0_value: u32;
    let mut lr_value: u32;

    unsafe {
        // Inline assembly to move the value of R0 into the variable `r0_value`
        asm!(
            "MOV {0}, R0",     // Move the value of R0 into the output operand
            out(reg) r0_value  // Define `r0_value` as an output operand
        );
    }

    unsafe {
        // Inline assembly to move the value of R0 into the variable `r0_value`
        asm!(
            "MOV {0}, R1",     // Move the value of R0 into the output operand
            out(reg) lr_value  // Define `r0_value` as an output operand
        );
    }
    hprintln!("I am in a differnt function").unwrap();


    if lr_value == 0xFFFF_FFF9{
        asm!(
            "MSR msp, R0",  // Move the value from R0 into the PSP (Process Stack Pointer)
            in("r0") r0_value
        );
        unsafe{asm!(" pop {{r0-r3}}; pop {{r12}}; pop {{r5}}; pop {{r6}}; pop {{r5}};mov pc, r6 ;nop;", options(noreturn))};
    }
    asm!(
        "MSR msp, R0",  // Move the value from R0 into the PSP (Process Stack Pointer)
        in("r0") r0_value
    );
    unsafe{asm!(" pop {{r7,lr}}", options(noreturn))};
}