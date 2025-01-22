#![no_main]
#![no_std]
use core::ptr::{read_volatile, write_volatile};
use core::panic::PanicInfo;

extern "C" {
    fn _start();
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
            let ra = read_volatile(self.aflr_r);
            let bit_field = (pin * 4) as u32;
            write_volatile(self.aflr_r, ra | (af as u32) << bit_field);
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
    usart_rdr: *mut u32,
}

trait UARTops {
    fn config_cr_reg(&self, cr: *mut u32, reg_bit: u8, val: u8);
    fn config_buadrate(&self, usart_div: u16);
    fn send_byte(&self, byte: u8);
    fn read_byte(&self) -> u8;
    fn send_str(&self, s: &str);
}

impl UARTops for UART {
    fn config_cr_reg(&self, cr: *mut u32, reg_bit: u8, val: u8) {
        unsafe {
            let mut ra = read_volatile(cr);
            if val == 0 {
                ra |= (1 << reg_bit) as u32;
                ra ^= (1 << reg_bit) as u32;
            } else {
                ra |= (val << reg_bit) as u32;
            }
            write_volatile(cr, ra);
        }
    }

    fn config_buadrate(&self, usart_div: u16) {
        unsafe {
            write_volatile(self.usart_brr, usart_div as u32);
        }
    }

    fn send_byte(&self, byte: u8) {
        unsafe {
            while (read_volatile(self.usart_isr) & 0x40) == 0 {};
            write_volatile(self.usart_tdr, byte as u32);
        }
    }

    fn read_byte(&self) -> u8 {
        unsafe {
            while (read_volatile(self.usart_isr) & 0x20) == 0 {}
            return (read_volatile(self.usart_rdr) & 0xFF) as u8;
        }
    }

    fn send_str(&self, s: &str) {
        for c in s.as_bytes() {
            self.send_byte(*c);
        }
    }
}

const BS: u8 = 0x08;
const DEL: u8 = 0x7F;
const CTRL_U: u8 = 0x15;
const CTRL_W: u8 = 0x17;

fn reset_buffer(line_buffer: &mut [u8]) {
    for i in 0..line_buffer.len() {
        line_buffer[i] = b'\0';
    }
}

fn handle_backspace(uart: &UART, line_buffer: &mut [u8], cursor_pos: &mut usize) {
    if *cursor_pos > 0 {
        uart.send_str("\x08 \x08");
        *cursor_pos -= 1;
        line_buffer[*cursor_pos] = b'\0';
    }
}

fn handle_newline(uart: &UART, line_buffer: &mut [u8], cursor_pos: &mut usize) {
    uart.send_str("\r\n");
    if *cursor_pos > 0 {
        *cursor_pos = 0;
        reset_buffer(line_buffer);
    }
}

fn clear_line(uart: &UART, line_buffer: &mut [u8], cursor_pos: &mut usize) {
    uart.send_str("\x1b[2K"); // Clear entire line
    uart.send_str("\x1b[0G"); // Move cursor to 0th column
    reset_buffer(line_buffer);
    *cursor_pos = 0;
}


fn delete_word(uart: &UART, line_buffer: &mut [u8], cursor_pos: &mut usize) {
    if *cursor_pos == 0 {
        return;
    }
    let mut temp_pos = *cursor_pos;
    while temp_pos != 0 && line_buffer[temp_pos] != b' ' {
        temp_pos -= 1;
    }

    if temp_pos == 0 {
        clear_line(&uart, line_buffer, cursor_pos);
        return;
    } else {
        while temp_pos > 0 && line_buffer[temp_pos-1] == b' ' {
            temp_pos -= 1;
        }

        if temp_pos == 0 {
            clear_line(&uart, line_buffer, cursor_pos);
            return;
        }

        uart.send_str("\x1b[2K"); // Clear entire line
        uart.send_str("\x1b[0G"); // Move cursor to 0th column

        // update line_buffer content
        for i in temp_pos+1..*cursor_pos {
            line_buffer[i] = b'\0';
        }
        // display the updated content
        for i in 0..temp_pos+1 {
            uart.send_byte(line_buffer[i]);
        }
    }

    *cursor_pos = temp_pos+1;
}

fn display_char(uart: &UART, byte: u8, line_buffer: &mut [u8], cursor_pos: &mut usize) {
    uart.send_byte(byte); 
    line_buffer[*cursor_pos] = byte;
    *cursor_pos += 1;
}

#[no_mangle]
pub extern "C" fn main() {

    let mut cursor_pos = 0;
    let mut line_buffer: [u8; 528] = [b'\0'; 528];

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
        gpio_a.set_mode(1, 2); // pin A1 (rx) set to alternate function mode b'10'
        gpio_a.config_aflr(0, 8);
        gpio_a.config_aflr(1, 8);

        let uart = UART {
            usart_cr1: (UART4_BASE + 0x00) as *mut u32,
            usart_cr2: (UART4_BASE + 0x04) as *mut u32,
            usart_cr3: (UART4_BASE + 0x08) as *mut u32,
            usart_brr: (UART4_BASE + 0x0C) as *mut u32,
            usart_isr: (UART4_BASE + 0x1C) as *mut u32,
            usart_rdr: (UART4_BASE + 0x24) as *mut u32,
            usart_tdr: (UART4_BASE + 0x28) as *mut u32,
        };

        // transmission flowcontrol 8 data bits
        uart.config_cr_reg(uart.usart_cr1, 28, 0);
        uart.config_cr_reg(uart.usart_cr1, 12, 0);

        uart.config_buadrate(0x1a0); // 9600 buad
        uart.config_cr_reg(uart.usart_cr1, 0, 1); // Enable UART bit
        uart.config_cr_reg(uart.usart_cr1, 3, 1); // Enable TE bit
        uart.config_cr_reg(uart.usart_cr1, 2, 1); // Enable RE bit

        let running = true;
        while running {
            let byte = uart.read_byte();
            while (read_volatile(uart.usart_isr) & 0x40) == 0 {}

            match byte {
                BS | DEL => {
                    handle_backspace(&uart, &mut line_buffer, &mut cursor_pos);
                }
                b'\r' | b'\n' => {
                    handle_newline(&uart, &mut line_buffer, &mut cursor_pos);
                }
                CTRL_U => {
                    clear_line(&uart, &mut line_buffer, &mut cursor_pos);
                }
                CTRL_W => {
                    delete_word(&uart, &mut line_buffer, &mut cursor_pos);
                }
                _ => {
                    display_char(&uart, byte, &mut line_buffer, &mut cursor_pos);
                }
            }
        }
    }
}
