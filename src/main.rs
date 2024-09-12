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

use stm32f3xx_hal_v2::interrupt;

use stm32f3xx_hal_v2::{pac::Peripherals, pac::Interrupt, timer::{Event, Timer}};

use stm32f3xx_hal_v2::{pac, prelude::*};

fn read_regs(){
    let control_reg_value: u32;
    unsafe {
        asm!(
            "MRS {0}, CONTROL",
            out(reg) control_reg_value
        );
    }
    let ipsr: u32;
    unsafe {
        asm!(
            "MRS {0}, IPSR",
            out(reg) ipsr
        );
    }

    // Read APSR (Application Program Status Register)
    let apsr: u32;
    unsafe {
        asm!(
            "MRS {0}, APSR",
            out(reg) apsr
        );
    }

    // Read EPSR (Execution Program Status Register)
    let epsr: u32;
    unsafe {
        asm!(
            "MRS {0}, EPSR",
            out(reg) epsr
        );
    }
    let t_bit = (epsr >> 24) & 0x1;

    let xpsr: u32;
    unsafe {
        asm!(
            "MRS {0}, XPSR",
            out(reg) xpsr
        );
    }
    let msp: u32;
    unsafe {
        asm!(
            "MRS {0}, msp",
            out(reg) msp
        );
    }
    let cpsr: u32;
    unsafe {
        asm!(
            "MRS {0}, cpsr",
            out(reg) cpsr
        );
    }


    hprintln!("control {:x}", control_reg_value).unwrap(); 
    hprintln!("apsr {:x}", apsr).unwrap();
    hprintln!("epsr {:x}", epsr).unwrap();
    hprintln!("tbit {:x}", epsr).unwrap();
    hprintln!("xpsr {:x}", xpsr).unwrap();
    hprintln!("msp {:x}", msp).unwrap();
    hprintln!("cpsr {:x}", cpsr).unwrap();

}

#[entry]
fn main()->!{
    hprintln!("before pendsv").unwrap();
    //SCB::set_pendsv(); 
    unsafe {arch::asm!("svc #11");}
    hprintln!("after pendsv").unwrap();
    loop{
        hprintln!("rest st");
       unsafe { _rest();}
    }
}

// #[exception]
// fn PendSV(){
//     read_regs();
//     hprintln!("In pendsv").unwrap();
// }

#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn PendSV() {
    hprintln!("In pendsv").unwrap();
    read_regs();
    arch::asm!( " mov r1, #0xFFFFFFF3" );
    arch::asm!("msr APSR, r1");
    arch::asm!("msr XPSR, r1");
    read_regs();
    arch::asm!("bl _test;");
}
#[no_mangle]
pub unsafe extern "C" fn _rest(){
    hprintln!("rest");
}


#[no_mangle]
pub unsafe extern "C" fn _test(){
    hprintln!("test").unwrap();
    asm!(
        "movw r0, #0x0000",           // Load lower 16 bits of 0x080005AA into R0
        "movt r0, #0x4100",           // Load upper 16 bits of 0x080005AA into R0
        "push {{r0}}",
        "movw r0, #0x05aa",           // Load lower 16 bits of 0x080005AA into R0
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
#[naked]
#[no_mangle]
pub unsafe extern "C" fn SVCall(){
    //arch::asm!("add sp, sp ,#32; movw r0,#05aa;movt r0,0x0800; push {{r0,r1}} ;mov lr, #0xFFFFFFF1; bl _test", options(noreturn));

    asm!(
        "add sp, sp, #32",            // Add 32 to the stack pointer
        "movw r0, #0x000b",           // Load lower 16 bits of 0x080005AA into R0
        "movt r0, #0x4100",           // Load upper 16 bits of 0x080005AA into R0
        "push {{r0}}",
        "movw r0, #0x05be",           // Load lower 16 bits of 0x080005AA into R0
        "movt r0, #0x0800",  
        "push {{r0}}",            // Push R0 and R1 onto the stack
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "push {{r0}}",
        "mov lr, #0xFFFFFFF1",        // Load 0xFFFFFFF1 into LR
        "bx lr",                    // Branch with Link to _test
        options(noreturn)             // Indicate that this code does not return
    );
    //arch::asm!("NOP;bl _test;", options(noreturn));
    //arch::asm!("bl _test;");
}
