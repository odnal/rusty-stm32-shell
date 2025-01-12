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
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x4C) as *mut u32;  // GPIO Clock Bus
const RCC_APB1ENR1: *mut u32 = (RCC_BASE + 0x58) as *mut u32; // UART Clock Bus

const GPIOA_BASE: u32 = 0x48000000;
const UART4_BASE: u32 = 0x40004C00;

struct GPIO_t {
    mode_r: *mut u32,
    aflr_r: *mut u32,
}

enum GpioPort {
    A, B, C, D, E, F, G, H, I,
}

trait GPIOops {
    fn enable_clock(rcc_bus: *mut u32, port_bit: u8);
    fn set_mode(&self, pin: u8, mode: u8);
    fn config_aflr(&self, pin: u8, af: u8);
}

impl GPIOops for GPIO_t {
    fn enable_clock(rcc_bus: *mut u32, port_bit: u8) {
        unsafe {
            let ra = read_volatile(rcc_bus);
            write_volatile(rcc_bus, ra | (1 << port_bit));
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

    fn config_aflr(&self, pin: u8, af: u8) {
        unsafe {
            let bit_field = (pin * 4) as u32;
            write_volatile(self.aflr_r, (af as u32) << bit_field);
        }
    }
}

struct UART {
    usart_cr1: *mut u32,
    usart_cr2: *mut u32,
    usart_cr3: *mut u32,
    usart_brr: *mut u32,
    usart_isr: *mut u32,
    usart_tdr: *mut u32,
}

trait UARTops {
    fn config_cr_reg(&self, cr: *mut u32, reg_bit: u8);
    fn config_buadrate(&self, usart_div: u16); // some math formula in ds to figure this. #todo for later.
    fn config_data_length(&self, m0_bit: u8, m1_bit: u8);
    fn send_byte(&self, byte: u8);
}

impl UARTops for UART {
    fn config_cr_reg(&self, cr: *mut u32, reg_bit: u8) {
        unsafe {
            write_volatile(cr, 1 << reg_bit);
        }
    }

    fn config_buadrate(&self, usart_div: u16) {
        unsafe {
            write_volatile(self.usart_brr, usart_div as u32);
        }
    }

    fn config_data_length(&self, m0_bit: u8, m1_bit: u8) {
        unsafe {
            todo!();
        }
    }

    fn send_byte(&self, byte: u8) {
        unsafe {
            write_volatile(self.usart_tdr, byte as u32);
        }
    }
}

#[no_mangle]
pub extern "C" fn main() {
    let mut ra;

    unsafe {
        // Init clocks
        GPIO_t::enable_clock(RCC_APB1ENR1, 19);
        GPIO_t::enable_clock(RCC_AHB2ENR, GpioPort::A as u8);

        let gpio_a = GPIO_t { 
            mode_r: (GPIOA_BASE + 0x00) as *mut u32,
            aflr_r: (GPIOA_BASE + 0x20) as *mut u32,
        };

        // Configure GPIO
        gpio_a.set_mode(0, 2); // pin A0 (tx) set to alternate function mode b'10'
        gpio_a.config_aflr(GpioPort::A as u8, 8);

        // Configure UART Only TX side to transmit data out the tx pin to an FTDI device so that we can see the data printed in a virtual com port.
        let uart = UART {
            usart_cr1: (UART4_BASE + 0x00) as *mut u32,
            usart_cr2: (UART4_BASE + 0x04) as *mut u32,
            usart_cr3: (UART4_BASE + 0x08) as *mut u32,
            usart_brr: (UART4_BASE + 0x0C) as *mut u32,
            usart_isr: (UART4_BASE + 0x1C) as *mut u32,
            usart_tdr: (UART4_BASE + 0x28) as *mut u32,
        };

        // transmission flowcontrol 8 data bits
        ra = read_volatile(uart.usart_cr1);
        ra |= (1<<28) | (1<<12);
        ra ^= (1<<28) | (1<<12);
        write_volatile(uart.usart_cr1, ra);

        uart.config_buadrate(0x0022); // 115200 did the maths awhile ago
        uart.config_cr_reg(uart.usart_cr1, 0); // Enable UART

        // Set TE bit to send idle frame
        ra = read_volatile(uart.usart_cr1);
        ra |= 1<<3;
        write_volatile(uart.usart_cr1, ra);

        let running = true;
        while running {
            while (read_volatile(uart.usart_isr) & 0x40) == 0 {}
            uart.send_byte(0x55);
            while (read_volatile(uart.usart_isr) & 0x40) == 0 {}
            uart.send_byte(0x0D);
            while (read_volatile(uart.usart_isr) & 0x40) == 0 {}
            uart.send_byte(0x0A);

            for _ in 0..10000 {}
        }
    }
}
