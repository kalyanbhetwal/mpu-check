#![no_main]
#![no_std]

use panic_halt as _;

use core::sync::atomic::{AtomicUsize, Ordering};


// TODO(6) Import your HAL
use stm32f3xx_hal_v2 as _; // memory layout
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SCB;
use cortex_m_rt::ExceptionFrame;
use cortex_m_rt::exception;
use cortex_m_semihosting::hprintln; // Import the `hprintln!` macro
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

mod checkpoint;
use checkpoint::{checkpoint, restore, delete_all_pg, delete_pg};

#[no_mangle]
pub extern "C" fn svc_handler() {
    hprintln!("SVC handler called").unwrap();
    unsafe {
        asm!(
            "mov r0, #0",     // Prepare to set CONTROL register to 0
            "msr control, r0", // Write 0 to CONTROL register to set privileged mode
            options(nostack)
        );
    }

}

fn disable_mpu() {
    unsafe {
        // MPU base address
        const MPU_BASE: usize = 0xE000_ED94;
        // Pointer to the MPU control register
        let mpu_ctrl = MPU_BASE as *mut u32;
        
        // Read the current control register value
        let ctrl = ptr::read_volatile(mpu_ctrl);
        
        // Disable the MPU by clearing the ENABLE bit (bit 0)
        ptr::write_volatile(mpu_ctrl, ctrl & !(1 << 0));
    }
}
const CONTROL_REGISTER: *const u32 = 0xE000ED14 as *const u32;

// Function to get the current mode of the processor
fn get_current_mode() -> bool {
    let control_value: u32;
    unsafe{
    // Use inline assembly to read the CONTROL register
    asm!(
        "mrs {0}, control",  // 'control' is the register to read
        out(reg) control_value  // Output the result into 'control_value'
    );


    hprintln!("CONTROL register value: {:#X}", control_value).unwrap();
}
    unsafe {
        let control_reg = core::ptr::read_volatile(CONTROL_REGISTER);
        control_reg & 0x1 != 0
    }

}

// Address of the MPU Region Base Address Register (MPU_RBAR)
const MPU_RBAR_ADDRESS: *mut u32 = 0xE000ED9C as *mut u32;
// Address of the MPU Region Attribute and Size Register (MPU_RASR)
const MPU_RASR_ADDRESS: *mut u32 = 0xE000EDA0 as *mut u32;
// Address of the MPU Control Register (MPU_CTRL)
const MPU_CTRL_ADDRESS: *mut u32 = 0xE000ED94 as *mut u32;

fn configure_mpu_allow_all() {
    unsafe {
        // Disable MPU
        let mut mpu_ctrl = ptr::read_volatile(MPU_CTRL_ADDRESS);
        mpu_ctrl &= !(1 << 0); // Clear ENABLE bit
        ptr::write_volatile(MPU_CTRL_ADDRESS, mpu_ctrl);

        // // Configure the first MPU region to cover all address space
        // ptr::write_volatile(MPU_RBAR_ADDRESS, 0); // Set base address to 0
        // ptr::write_volatile(MPU_RASR_ADDRESS, 
        //     (0b11 << 24) | // Full access
        //     (0b1111 << 1)  // Size 4GB (assuming a 32-bit address space)
        // );

        // // Enable MPU
        // mpu_ctrl |= 1 << 0; // Set ENABLE bit
        // ptr::write_volatile(MPU_CTRL_ADDRESS, mpu_ctrl);
    }
}

#[entry]
fn main() -> ! {
    //delete_all_pg();
    //restore();
    //configure_mpu_allow_all();
     // Trigger an SVC call to switch to privileged mode
    //  unsafe {
    //     asm!(
    //         "svc 0", // Trigger SVC instruction
    //         options(nostack)
    //     );
    // }
    
    let dp = Peripherals::take().unwrap();
    // Set up the system clock
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut timer = Timer::tim2(dp.TIM2, 5.hz(), clocks, &mut rcc.apb1);
    timer.listen(Event::Update);

    // let is_handler_mode = get_current_mode();
    // if is_handler_mode {
    //     // Handle the case where the processor is in Handler mode
    //     hprintln!("h mode").unwrap();
    // } else {
    //     // Handle the case where the processor is in Thread mode
    //     hprintln!("th mode").unwrap();

    // }

    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);

    unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::TIM2) };

    loop{}
}
// #[cortex_m_rt::interrupt]
// fn TIM2() {
//     // ..
//     // Clear reason for the generated interrupt request
//     hprintln!("test").unwrap();
// }

#[export_name = "TIM2"]
pub unsafe extern "C" fn __cortex_m_rt_TIM2_trampoline() {
    cortex_m::interrupt::disable();
    let icsr = unsafe { ptr::read(&(*SCB::PTR).icsr as *const _ as *const u32) };
    hprintln!("icsr: {:x}", icsr).unwrap();
    hprintln!("test").unwrap();

    asm!(
        "mov r0, #1",     // Load 0 into R0
        "msr control, r0", // Set CONTROL register to 0
        options(nostack, nomem, preserves_flags)
    );

    //checkpoint(false);
    hprintln!("rest").unwrap();
    //cortex_m::interrupt::enable();
}


#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    let scb = unsafe { &*(SCB::PTR as *const SCB) };
    
    // Read fault status registers
    let hfsr = scb.hfsr.read();
    let cfsr = scb.cfsr.read();
    let mmfar = scb.mmfar.read();
    let bfar = scb.bfar.read();

    // Extract fault information
    let mem_manage_fault = (cfsr & 0xFF) as u8;
    let bus_fault = ((cfsr >> 8) & 0xFF) as u8;
    let usage_fault = ((cfsr >> 16) & 0xFF) as u8;

    // Print detailed fault information
    hprintln!("HardFault occurred!").ok();
    hprintln!("HFSR: 0x{:08X}", hfsr).ok();
    hprintln!("CFSR: 0x{:08X}", cfsr).ok();
    hprintln!("MMFAR: 0x{:08X}", mmfar).ok();
    hprintln!("BFAR: 0x{:08X}", bfar).ok();
    hprintln!("ExceptionFrame: {:?}", ef).ok();

    // Print detailed explanations of the faults
    print_cfsr_explanations(mem_manage_fault, bus_fault, usage_fault);
    
    // Halt execution
    loop {
        cortex_m::asm::bkpt(); // Trigger a breakpoint for debugging
    }
}

fn print_cfsr_explanations(mem_manage_fault: u8, bus_fault: u8, usage_fault: u8) {
    hprintln!("Configurable Fault Status Register (CFSR):").ok();

    // Memory Management Fault
    hprintln!("Memory Management Fault Status:").ok();
    if (mem_manage_fault & 0x01) != 0 {
        hprintln!("  - Memory Management Fault occurred due to a failed MPU check.").ok();
    }
    if (mem_manage_fault & 0x02) != 0 {
        hprintln!("  - Access violation caused by a failed MPU region.").ok();
    }
    if (mem_manage_fault & 0x04) != 0 {
        hprintln!("  - DWT Unit: Debug Watchpoint Unit event caused a fault.").ok();
    }
    
    // Bus Fault
    hprintln!("Bus Fault Status:").ok();
    if (bus_fault & 0x01) != 0 {
        hprintln!("  - Instruction Bus Error").ok();
    }
    if (bus_fault & 0x02) != 0 {
        hprintln!("  - Data Bus Error").ok();
    }
    if (bus_fault & 0x08) != 0 {
        hprintln!("  - Unstacking Error").ok();
    }
    if (bus_fault & 0x10) != 0 {
        hprintln!("  - Stack Error").ok();
    }
    
    // Usage Fault
    hprintln!("Usage Fault Status:").ok();
    if (usage_fault & 0x01) != 0 {
        hprintln!("  - Undefined Instruction").ok();
    }
    if (usage_fault & 0x02) != 0 {
        hprintln!("  - Invalid State").ok();
    }
    if (usage_fault & 0x08) != 0 {
        hprintln!("  - Invalid PC Load").ok();
    }
    if (usage_fault & 0x10) != 0 {
        hprintln!("  - Division by Zero").ok();
    }
}
