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
    }
}

pub struct MachineZxSpectrum48 {
    ula: Ula,
    memory: Memory48k,
    cpu: crate::cpu::Z80,
    t_states: u32,
    frame_ready: bool,
    pub screen_buffer: Vec<u8>,
}


impl Ports for MachineZxSpectrum48 {
    fn read_port(&self, port: u16) -> u8 {
        // Simple mock for 48K Spectrum
        // Port 0xFE (Keyboard/Ear)
        if (port & 0x01) == 0 {
            return 0xFF; // No keys pressed
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
    pub fn new() -> Self {
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
        };
        machine.load_rom();
        // machine.load_test_image("test.tap").unwrap_or_else(|e| {
        //     println!("Error loading test.tap: {}", e);
        // }   );
        machine
    }

    pub fn load_rom(&mut self) {
        let mut loaded = false;
        let rom_name = "48.rom";
        if let Ok(rom) = std::fs::read(rom_name) {
            // Check if it looks like a valid ROM (exact size for 48K ROM)
            if rom.len() == 16384 {
                    self.memory.rom[..16384].copy_from_slice(&rom);
                    loaded = true;
            } else {
                println!("Warning: {} has incorrect size ({} bytes). Expected 16384 bytes.", rom_name, rom.len());
            }
        }
        if !loaded {
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

                self.cpu.step(bus)

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

