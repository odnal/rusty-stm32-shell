#![no_main]
#![no_std]
use core::ptr::{read_volatile, write_volatile};
use core::panic::PanicInfo;

extern "C" {
    fn _start();
    #[allow(dead_code)]
    fn dummy(_: u32);
}

#[panic_handler]
fn panic_handler(_: &PanicInfo) -> ! {
    loop {}
}

const RCC_BASE: u32 = 0x40021000;
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x4C) as *mut u32;
const RCC_AB1ENR1: *mut u32 = (RCC_BASE + 0x58) as *mut u32;

const GPIOA_BASE: u32 = 0x48000000;
const UART4_BASE: u32 = 0x40004C00;

struct GPIO_t {
    mode_r: *mut u32,
}

enum GpioPort {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
}

trait GPIOops {
    fn enable_clock(rcc_bus: *mut u32, port_bit: GpioPort);
    fn set_mode(&self, pin: u8, mode: u8);
}

impl GPIOops for GPIO_t {
    fn enable_clock(rcc_bus: *mut u32, port_bit: GpioPort) {
        unsafe {
            let ra = read_volatile(rcc_bus);
            write_volatile(rcc_bus, ra | (1 << (port_bit as u8)));
        }
    }

    fn set_mode(&self, pin: u8, mode: u8) {
        unsafe {
            let mut ra = read_volatile(self.mode_r);
            ra &= !(3 << (pin * 2));          // clear - 2bit field
            ra |= (mode as u32) << (pin * 2); // set mode - 2bit field 
            write_volatile(self.mode_r, ra);
        }
    }
}

#[no_mangle]
pub extern "C" fn main() {
    GPIO_t::enable_clock(RCC_AHB2ENR, GpioPort::A);
    let gpio_a = GPIO_t { mode_r: (GPIOA_BASE + 0x00) as *mut u32 };
    gpio_a.set_mode(0, 1); // pin A0 set to output mode '01'

    let running = true;
    while running {}
}
