#![no_main]
#![no_std]
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
    read_regs();
    loop{
       // hprintln!("rest").unwrap();
       //debug::exit(debug::EXIT_SUCCESS);
    }
}
#[no_mangle]
#[export_name = "TIM2"]
pub unsafe extern "C" fn __cortex_m_rt_TIM2_trampoline() {
    hprintln!("I am in interrupt").unwrap();
    unsafe {asm!("cpsid i")};
    read_regs();
   //unsafe{asm!("NOP", options(noreturn))};
   //unsafe{asm!("cpsid i; nop; mov r0,sp; mov r1, lr; bl _timer2;", options(noreturn))};
}
