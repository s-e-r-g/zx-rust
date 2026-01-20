//! ZX Spectrum Emulator - Z80 CPU and System Emulation
//!
//! This module implements a complete ZX Spectrum 48K emulator with:
//! - Z80 CPU emulation (full instruction set)
//! - 48KB RAM and 16KB ROM
//! - Interrupt timing (50Hz frame interrupts)
//! - Display memory mapping
//! - Keyboard input and port I/O
//!
//! The emulator executes ~10,000 instructions per frame to maintain
//! responsiveness while accurately simulating the 3.5MHz Z80 processor.

const SCREEN_WIDTH: usize = 256;
const SCREEN_HEIGHT: usize = 192;
const BORDER_H: usize = 32;
const BORDER_V: usize = 24;
pub const FULL_WIDTH: usize = SCREEN_WIDTH + BORDER_H * 2;
pub const FULL_HEIGHT: usize = SCREEN_HEIGHT + BORDER_V * 2;

const PALETTE: [[u8; 4]; 16] = [
    [0x00, 0x00, 0x00, 0xFF], // 0: Black
    [0x00, 0x00, 0xCD, 0xFF], // 1: Blue
    [0xCD, 0x00, 0x00, 0xFF], // 2: Red
    [0xCD, 0x00, 0xCD, 0xFF], // 3: Magenta
    [0x00, 0xCD, 0x00, 0xFF], // 4: Green
    [0x00, 0xCD, 0xCD, 0xFF], // 5: Cyan
    [0xCD, 0xCD, 0x00, 0xFF], // 6: Yellow
    [0xCD, 0xCD, 0xCD, 0xFF], // 7: White
    [0x00, 0x00, 0x00, 0xFF], // 8: Bright Black
    [0x00, 0x00, 0xFF, 0xFF], // 9: Bright Blue
    [0xFF, 0x00, 0x00, 0xFF], // 10: Bright Red
    [0xFF, 0x00, 0xFF, 0xFF], // 11: Bright Magenta
    [0x00, 0xFF, 0x00, 0xFF], // 12: Bright Green
    [0x00, 0xFF, 0xFF, 0xFF], // 13: Bright Cyan
    [0xFF, 0xFF, 0x00, 0xFF], // 14: Bright Yellow
    [0xFF, 0xFF, 0xFF, 0xFF], // 15: Bright White
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Key {
    // Row 0
    Shift, Z, X, C, V,
    // Row 1
    A, S, D, F, G,
    // Row 2
    Q, W, E, R, T,
    // Row 3
    Num1, Num2, Num3, Num4, Num5,
    // Row 4
    Num0, Num9, Num8, Num7, Num6,
    // Row 5
    P, O, I, U, Y,
    // Row 6
    Enter, L, K, J, H,
    // Row 7
    Space, Sym, M, N, B,
}

pub trait Memory {
    fn read_byte(&self, addr: u16) -> u8;
    fn write_byte(&mut self, addr: u16, val: u8);
}

pub trait Ports {
    fn read_port(&self, port: u16) -> u8;
    fn write_port(&mut self, port: u16, val: u8);
}

pub trait Bus: Memory + Ports {}

struct Ula {
    framebuffer: Vec<u8>, // RGBA bytes
    h_counter: usize,
    v_counter: usize,
    border_color: u8,
}

impl Ula {
    fn new() -> Self {
        Self {
            framebuffer: vec![0; FULL_WIDTH * FULL_HEIGHT * 4],
            h_counter: 0,
            v_counter: 0,
            border_color: 0,
        }
    }

    fn tick(&mut self, memory: &dyn Memory) {
        let pixel = if self.in_visible_area() {
            self.compute_pixel(memory)
        } else {
            PALETTE[self.border_color as usize]
        };
        let idx = self.idx() * 4;
        self.framebuffer[idx..idx+4].copy_from_slice(&pixel);
        self.h_counter += 1;
        if self.h_counter == FULL_WIDTH {
            self.h_counter = 0;
            self.v_counter += 1;
            if self.v_counter == FULL_HEIGHT {
                self.v_counter = 0;
            }
        }
    }

    fn in_visible_area(&self) -> bool {
        self.h_counter >= BORDER_H && self.h_counter < BORDER_H + SCREEN_WIDTH &&
        self.v_counter >= BORDER_V && self.v_counter < BORDER_V + SCREEN_HEIGHT
    }

    fn idx(&self) -> usize {
        self.v_counter * FULL_WIDTH + self.h_counter
    }

    fn compute_pixel(&self, memory: &dyn Memory) -> [u8; 4] {
        let x = self.h_counter - BORDER_H;
        let y = self.v_counter - BORDER_V;
        // Calculate pixel address
        let pixel_addr = 0x4000 + ((y & 0xC0) << 5) | ((y & 0x38) << 2) | ((y & 0x07) << 8) | (x >> 3);
        let bit = (memory.read_byte(pixel_addr as u16) >> (7 - (x & 7))) & 1;
        // Calculate attribute address
        let attr_addr = 0x5800 + ((y >> 3) * 32) + (x >> 3);
        let attr = memory.read_byte(attr_addr as u16);
        let ink = attr & 7;
        let paper = (attr >> 3) & 7;
        let bright = (attr >> 6) & 1;
        let color = if bit == 1 { ink | (bright << 3) } else { paper | (bright << 3) };
        PALETTE[color as usize]
    }
}

pub struct Memory48k
{
    ram: [u8; 0xC000], // 48KB RAM
    rom: [u8; 0x4000], // 16KB ROM
}

impl Memory for Memory48k {
    fn read_byte(&self, addr: u16) -> u8 {
        if addr < 0x4000 {
            self.rom[addr as usize]
        } else {
            self.ram[(addr - 0x4000) as usize]
        }
    }

    fn write_byte(&mut self, addr: u16, val: u8) {
        // ROM protection: prevent writes to 0x0000-0x3FFF
        if addr >= 0x4000 {
            self.ram[(addr - 0x4000) as usize] = val;
        }
        else {
            // TODO: remove after testing
            self.rom[addr as usize] = val; // Allow writing to ROM for testing purposes
        }
    }
}

pub struct MachineZxSpectrum48 {
    ula: Ula,
    memory: Memory48k,
    cpu: crate::cpu::Z80,
    t_states: u32,
    frame_ready: bool,
    pub screen_buffer: Vec<u8>,
    keyboard: [u8; 8],
    debugger: crate::debugger::Debugger,
}


impl Ports for MachineZxSpectrum48 {
    fn read_port(&self, port: u16) -> u8 {
        // Port 0xFE (Keyboard/Ear)
        if (port & 0x0001) == 0 {
            let mut result = 0xFF;
            // Check address lines A8-A15 to see which key row is being scanned
            if (port & 0x0100) == 0 { result &= self.keyboard[0]; } // A8
            if (port & 0x0200) == 0 { result &= self.keyboard[1]; } // A9
            if (port & 0x0400) == 0 { result &= self.keyboard[2]; } // A10
            if (port & 0x0800) == 0 { result &= self.keyboard[3]; } // A11
            if (port & 0x1000) == 0 { result &= self.keyboard[4]; } // A12
            if (port & 0x2000) == 0 { result &= self.keyboard[5]; } // A13
            if (port & 0x4000) == 0 { result &= self.keyboard[6]; } // A14
            if (port & 0x8000) == 0 { result &= self.keyboard[7]; } // A15
            
            // Bit 6 is for EAR, usually high.
            // Bits 5 and 7 are floating.
            return (result & 0x1F) | 0x40;
        }
        0xFF
    }

    fn write_port(&mut self, port: u16, val: u8) {
        // Port 0xFE (Border/Mic/Beep)
        if (port & 0x01) == 0 {
            self.ula.border_color = val & 0x07;
        }
    }
}


impl Memory for MachineZxSpectrum48 {
    fn read_byte(&self, addr: u16) -> u8 {
        self.memory.read_byte(addr)
    }
    fn write_byte(&mut self, addr: u16, val: u8) {
        self.memory.write_byte(addr, val)
    }
}

impl Bus for MachineZxSpectrum48 {}


impl MachineZxSpectrum48 {
    pub fn set_key_state(&mut self, key: Key, pressed: bool) {
        let (row, bit) = match key {
            // Row 0 (SHIFT, Z, X, C, V)
            Key::Shift => (0, 0),
            Key::Z => (0, 1),
            Key::X => (0, 2),
            Key::C => (0, 3),
            Key::V => (0, 4),
            // Row 1 (A, S, D, F, G)
            Key::A => (1, 0),
            Key::S => (1, 1),
            Key::D => (1, 2),
            Key::F => (1, 3),
            Key::G => (1, 4),
            // Row 2 (Q, W, E, R, T)
            Key::Q => (2, 0),
            Key::W => (2, 1),
            Key::E => (2, 2),
            Key::R => (2, 3),
            Key::T => (2, 4),
            // Row 3 (1, 2, 3, 4, 5)
            Key::Num1 => (3, 0),
            Key::Num2 => (3, 1),
            Key::Num3 => (3, 2),
            Key::Num4 => (3, 3),
            Key::Num5 => (3, 4),
            // Row 4 (0, 9, 8, 7, 6)
            Key::Num0 => (4, 0),
            Key::Num9 => (4, 1),
            Key::Num8 => (4, 2),
            Key::Num7 => (4, 3),
            Key::Num6 => (4, 4),
            // Row 5 (P, O, I, U, Y)
            Key::P => (5, 0),
            Key::O => (5, 1),
            Key::I => (5, 2),
            Key::U => (5, 3),
            Key::Y => (5, 4),
            // Row 6 (ENTER, L, K, J, H)
            Key::Enter => (6, 0),
            Key::L => (6, 1),
            Key::K => (6, 2),
            Key::J => (6, 3),
            Key::H => (6, 4),
            // Row 7 (SPACE, SYM, M, N, B)
            Key::Space => (7, 0),
            Key::Sym => (7, 1),
            Key::M => (7, 2),
            Key::N => (7, 3),
            Key::B => (7, 4),
        };

        if pressed {
            self.keyboard[row] &= !(1 << bit);
        } else {
            self.keyboard[row] |= 1 << bit;
        }
    }

    pub fn new_with_options(enable_disassembler: bool, enable_trace_interrupts: bool, rom_filename: String, run_zexall: bool) -> Self {
        let mut machine = Self {
            ula: Ula::new(),
            memory: Memory48k {
                ram: [0; 0xC000],
                rom: [0; 0x4000],
            },
            cpu: crate::cpu::Z80::new(),
            t_states: 0,
            frame_ready: false,
            screen_buffer: vec![0; FULL_WIDTH * FULL_HEIGHT * 4],
            keyboard: [0xFF; 8],
            debugger: crate::debugger::Debugger::new(enable_disassembler, enable_trace_interrupts, run_zexall),
        };
        machine.load_rom(&rom_filename, run_zexall);
        // machine.load_test_image("test.tap").unwrap_or_else(|e| {
        //     println!("Error loading test.tap: {}", e);
        // }   );
        machine
    }

    pub fn load_rom(&mut self, rom_filename: &str, run_zexall: bool) {
        if run_zexall  {
            self.load_zexall_test(rom_filename);
        } else {
            self.load_standard_rom(rom_filename);
        }
    }

    fn load_standard_rom(&mut self, rom_filename: &str) {
        if let Ok(rom) = std::fs::read(rom_filename) {
            if rom.len() == 0x4000 {
                self.memory.rom.copy_from_slice(&rom);
                println!("Loaded ROM from {}", rom_filename);
            } else {
                println!("Warning: {} has incorrect size ({} bytes). Expected 16384 bytes.", rom_filename, rom.len());
                self.load_fallback_program();
            }
        } else {
            println!("Error: Could not load ROM from {}. Using fallback program.", rom_filename);
            self.load_fallback_program();
        }
    }

    fn load_zexall_test(&mut self, rom_name: &str) {
        let load_addr = 0x0100;
        if let Ok(rom) = std::fs::read(rom_name) {
            self.memory.rom[load_addr..load_addr + rom.len()].copy_from_slice(&rom);
            println!("Loaded ZEXALL/ZEXCOM test ROM from {}", rom_name);

            // Patch address 5 to add RET instruction to avoid hanging
            self.memory.rom[5] = 0xC9; // RET
            self.memory.rom[0x38] = 0xFB; // EI
            self.memory.rom[0x39] = 0xED; // RETI
            self.memory.rom[0x3A] = 0x4D;

            self.cpu.pc = 0x100;
            self.cpu.sp = 0xF000;
            self.cpu.iff1 = false;
            self.cpu.iff2 = false;
            self.cpu.halted = false;
            self.cpu.int_requested = false;
        } else {
            println!("Error: Could not load ZEXALL/ZEXCOM test ROM from {}. Using fallback program.", rom_name);
            self.load_fallback_program();
        }
    }

    fn load_fallback_program(&mut self) {
        println!("Loading built-in test program...");
        // Simple test program to fill screen
        let code: &[u8] = &[
            0x21, 0x00, 0x40, // LD HL, 0x4000
            0x34,             // INC (HL)
            0x23,             // INC HL
            0x7C,             // LD A, H
            0xFE, 0x58,       // CP 0x58
            0x20, 0xF7,       // JR NZ, -9 (to 0x34)
            0xC3, 0x00, 0x00, // JP 0x0000
        ];
        for (i, &b) in code.iter().enumerate() {
            self.memory.rom[i] = b;
        }
    }

    pub fn load_test_image(&mut self, filename: &str) -> Result<(), String> {
        let tap_bytes = match std::fs::read(filename) {
            Ok(bytes) => bytes,
            Err(e) => return Err(format!("Could not read file {}: {}", filename, e)),
        };

        let mut i = 0;
        // First pass: look for a standard CODE block with a header
        while i < tap_bytes.len() {
            if i + 2 > tap_bytes.len() { break; }
            let block_len = u16::from_le_bytes([tap_bytes[i], tap_bytes[i + 1]]) as usize;
            i += 2;
            
            if i + block_len > tap_bytes.len() { break; }
            let block = &tap_bytes[i..i + block_len];
            i += block_len;

            if block_len > 18 && block[0] == 0x00 && block[1] == 3 { // Header, Type 3: Code
                let data_len_in_header = u16::from_le_bytes([block[12], block[13]]);
                let start_address = u16::from_le_bytes([block[14], block[15]]);
                
                if i >= tap_bytes.len() { continue; }

                if i + 2 > tap_bytes.len() { break; }
                let data_block_len = u16::from_le_bytes([tap_bytes[i], tap_bytes[i + 1]]) as usize;
                
                if i + 2 + data_block_len > tap_bytes.len() { break; }
                let data_block = &tap_bytes[i + 2 .. i + 2 + data_block_len];

                if !data_block.is_empty() && data_block[0] == 0xFF {
                    let code_data = &data_block[1..data_block.len() - 1];
                    if code_data.len() != data_len_in_header as usize {
                        println!("Warning: TAP block header length ({}) differs from actual data length ({}).", data_len_in_header, code_data.len());
                    }
                    
                    println!("Loading {} bytes to 0x{:04X}", code_data.len(), start_address);
                    for (offset, byte) in code_data.iter().enumerate() {
                        self.write_byte(start_address + offset as u16, *byte);
                    }
                    
                    let di_halt = [0xF3, 0x76];
                    self.write_byte(0x5B00, di_halt[0]);
                    self.write_byte(0x5B01, di_halt[1]);
                    self.cpu.pc = 0x5B00;
                    self.cpu.iff1 = false;
                    self.cpu.iff2 = false;
                    self.cpu.halted = false;

                    return Ok(());
                }
            }
        }
        
        // Second pass: if no code block was found, look for a raw screen dump
        i = 0;
        while i < tap_bytes.len() {
            if i + 2 > tap_bytes.len() { break; }
            let block_len = u16::from_le_bytes([tap_bytes[i], tap_bytes[i + 1]]) as usize;
            i += 2;
            
            if i + block_len > tap_bytes.len() { break; }
            let block = &tap_bytes[i..i + block_len];
            i += block_len;

            if block_len > 2 && block[0] == 0xFF { // Data block
                let data = &block[1..block.len()-1];
                if data.len() == 6912 {
                    println!("Detected screen data (6912 bytes). Loading to 0x4000.");
                    for (offset, byte) in data.iter().enumerate() {
                        self.write_byte(0x4000 + offset as u16, *byte);
                    }

                    let di_halt = [0xF3, 0x76];
                    self.write_byte(0x5B00, di_halt[0]);
                    self.write_byte(0x5B01, di_halt[1]);
                    self.cpu.pc = 0x5B00;
                    self.cpu.iff1 = false;
                    self.cpu.iff2 = false;
                    self.cpu.halted = false;

                    return Ok(());
                }
            }
        }

        Err("No CODE or 6912-byte data block found in tap file.".to_string())
    }

    pub fn run_until_frame(&mut self) {
        self.frame_ready = false;

        while !self.frame_ready {
            // Execute one CPU instruction
            let t_states = unsafe {

                let bus = &mut *(self as *mut MachineZxSpectrum48 as *mut dyn Bus);

                self.cpu.step(bus, &mut self.debugger)
            };
            for _ in 0..t_states {
                self.ula.tick(&self.memory);
                self.t_states += 1;

                if self.t_states == 69888 {
                    self.t_states = 0;
                    self.frame_ready = true;
                    self.cpu.raise_int();
                    self.screen_buffer.copy_from_slice(&self.ula.framebuffer);
                }
            }
        }
    }
}

