#![no_main]
#![no_std]
 #![feature(naked_functions)]
use core::arch;
use cortex_m::register::basepri::read;
// #![feature(naked_functions)]
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

use cortex_m::peripheral;
use stm32f3xx_hal_v2::interrupt;

use stm32f3xx_hal_v2::{pac::Peripherals, pac::Interrupt, timer::{Event, Timer}};

use stm32f3xx_hal_v2::{pac, prelude::*};

use crate::checkpoint::restore;


mod checkpoint;

#[entry]
unsafe fn main()->!{
    //checkpoint::delete_all_pg();
    restore();
   // hprintln!("before pendsv").unwrap();
//    unsafe {cortex_m::peripheral::NVIC::unmask(Interrupt::TIM4);}
//    cortex_m::peripheral::NVIC::pend(Interrupt::TIM4);
    let mut core = cortex_m::peripheral::Peripherals::steal();
    core.NVIC.set_priority(Interrupt::TIM4, 50);
    core.NVIC.set_priority(Interrupt::TIM3, 45);
    peripheral::NVIC::unmask(Interrupt::TIM3);
    peripheral::NVIC::unmask(Interrupt::TIM4);
    cortex_m::interrupt::enable();
    peripheral::NVIC::pend(stm32f3xx_hal_v2::pac::Interrupt::TIM4);
   // unsafe {arch::asm!("svc 0");}
    // SCB::set_pendsv(); 
    // hprintln!("after pendsv").unwrap();
    loop{
        hprintln!("rest st in loop");
       unsafe { _rest();}
       unsafe { _test();}
    }

}

// #[exception]
// fn PendSV(){
//     read_regs();
//     hprintln!("In pendsv").unwrap();
// }

// #[allow(non_snake_case)]
// #[no_mangle]
// pub unsafe extern "C" fn PendSV() {
//     hprintln!("In pendsv").unwrap();
//     read_regs();
//     arch::asm!( " mov r1, #0xFFFFFFF3" );
//     arch::asm!("msr APSR, r1");
//     arch::asm!("msr XPSR, r1");
//     read_regs();
//     arch::asm!("bl _test;");
// }
#[no_mangle]
pub unsafe extern "C" fn _rest(){
    arch::asm!("NOP");
    hprintln!("rest my");
}


#[no_mangle]
pub unsafe extern "C" fn _test(){
    hprintln!("test").unwrap();
    hprintln!("test").unwrap();
    hprintln!("test").unwrap();
    hprintln!("test").unwrap();
    asm!(
        "movw r0, #0x0000",           // Load lower 16 bits of 0x080005AA into R0
        "movt r0, #0x4100",           // Load upper 16 bits of 0x080005AA into R0
        "push {{r0}}",
        "movw r0, #0x0204",           // Load lower 16 bits of 0x080005AA into R0
        "movt r0, #0x0800",  
        "push {{r0}}",            // Push R0 and R1 onto the stack
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "mov lr, #0xFFFFFFF9",        // Load 0xFFFFFFF1 into LR
        "bx lr",                    // Branch with Link to _test
        options(noreturn)             // Indicate that this code does not return
    );

}
#[no_mangle]
pub unsafe extern "C" fn PendSV() {
   // unsafe {arch::asm!("svc #11");}
}

// // #[naked]
// #[no_mangle]
// pub unsafe extern "C" fn SVCall(){
//     asm!(
//         "movw r0, #0x21a",           // Load lower 16 bits of 0x080005AA into R0
//         "movt r0, #0x0800",           // Load upper 16 bits of 0x080005AA into R0
//         "mov pc, r0",
//         options(noreturn)             // Indicate that this code does not return
//     );
//     //arch::asm!("bl _rest;");
//     // change program counter and go to diffenre location and see how it behaves
//     //arch::asm!("bl _test;");
// }

#[no_mangle]
extern "C" fn TIM4(){
    hprintln!("In tim4").unwrap();
    peripheral::NVIC::pend(stm32f3xx_hal_v2::pac::Interrupt::TIM3);
   // checkpoint::checkpoint(true);
}


#[no_mangle]
extern "C" fn TIM3(){
    hprintln!("In tim3").unwrap();
    checkpoint::checkpoint(true);
}