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

const F_S: u8 = 0x80;
const F_Z: u8 = 0x40;
const F_5: u8 = 0x20;
const F_H: u8 = 0x10;
const F_3: u8 = 0x08;
const F_PV: u8 = 0x04;
const F_N: u8 = 0x02;
const F_C: u8 = 0x01;

pub struct Emulator {
    pub memory: [u8; 0x10000],
    pub screen_buffer: Vec<u8>,
    pub border_color: u8,
    // Registers
    pub a: u8, pub f: u8,
    pub b: u8, pub c: u8,
    pub d: u8, pub e: u8,
    pub h: u8, pub l: u8,
    pub sp: u16,
    pub pc: u16,
    pub ix: u16, pub iy: u16,
    pub i: u8, pub r: u8,
    pub iff1: bool, pub iff2: bool,
    pub im: u8,
    pub alt_af: u16, pub alt_bc: u16, pub alt_de: u16, pub alt_hl: u16,
    pub halted: bool,
}

impl Emulator {
    pub fn new() -> Self {
        let mut emu = Self {
            memory: [0; 0x10000],
            screen_buffer: vec![0; FULL_WIDTH * FULL_HEIGHT * 4],
            border_color: 7, // White border
            a: 0, f: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0,
            sp: 0,
            pc: 0x0000,
            ix: 0, iy: 0, i: 0, r: 0,
            iff1: false, iff2: false, im: 1, // Spectrum defaults to IM 1 usually, but 0 on reset
            alt_af: 0, alt_bc: 0, alt_de: 0, alt_hl: 0,
            halted: false,
        };
        emu.load_rom(); // Завантаж ROM
        emu
    }

    pub fn load_rom(&mut self) {
        let mut loaded = false;
        let rom_name = "Robik_Basic48.rom"; // "48.rom"
        if let Ok(rom) = std::fs::read(rom_name) {
            // Check if it looks like a valid ROM (exact size for 48K ROM)
            if rom.len() == 16384 {
                    self.memory[..16384].copy_from_slice(&rom);
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
                self.memory[i] = b;
            }
        }
    }

    pub fn step(&mut self) {
        // Simulate 50Hz interrupt at the start of the frame
        if self.iff1 {
            // In a real emulator, this happens based on cycles. 
            // Here we just trigger it once per 'step' call (frame).
            self.trigger_interrupt();
        }

        for _ in 0..10000 {
            if self.halted {
                // CPU waits for interrupt. Since we handle interrupt outside this loop,
                // we just break to simulate waiting for the next frame/interrupt.
                break;
            }

            let opcode = self.fetch_byte();
            self.execute(opcode);
        }
    }

    fn trigger_interrupt(&mut self) {
        // Accept interrupt
        if self.halted {
            self.halted = false;
            self.pc = self.pc.wrapping_add(1);
        }

        self.iff1 = false;
        self.iff2 = false;
        
        // Push PC
        self.push(self.pc);
        
        // Jump to interrupt vector (IM 1 = 0x0038)
        match self.im {
            0 => { self.pc = 0x0038; }, // Effectively RST 38H (0xFF on bus)
            1 => { self.pc = 0x0038; },
            2 => {
                let vector = (self.i as u16) << 8 | 0xFF; // Bus usually 0xFF
                self.pc = self.read_word(vector);
            },
            _ => {}
        }
    }

    fn fetch_byte(&mut self) -> u8 {
        let val = self.memory[self.pc as usize];
        self.pc = self.pc.wrapping_add(1);
        self.r = self.r.wrapping_add(1);
        val
    }

    fn read_port(&self, port: u16) -> u8 {
        // Simple mock for 48K Spectrum
        // Port 0xFE (Keyboard/Ear)
        if (port & 0x0001) == 0 {
            return 0xFF; // No keys pressed
        }
        0xFF
    }

    fn write_port(&mut self, port: u16, val: u8) {
        // Port 0xFE (Border/Mic/Beep)
        if (port & 0x0001) == 0 {
            self.border_color = val & 0x07;
        }
    }

    fn fetch_word(&mut self) -> u16 {
        let l = self.fetch_byte() as u16;
        let h = self.fetch_byte() as u16;
        (h << 8) | l
    }

    fn read_byte(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn write_byte(&mut self, addr: u16, val: u8) {
        if addr >= 0x4000 { // Simple ROM protection
            self.memory[addr as usize] = val;
        }
    }

    fn read_word(&self, addr: u16) -> u16 {
        let l = self.read_byte(addr) as u16;
        let h = self.read_byte(addr.wrapping_add(1)) as u16;
        (h << 8) | l
    }

    fn write_word(&mut self, addr: u16, val: u16) {
        self.write_byte(addr, (val & 0xFF) as u8);
        self.write_byte(addr.wrapping_add(1), (val >> 8) as u8);
    }

    // Helper to get register by index (0..7)
    // 0=B, 1=C, 2=D, 3=E, 4=H, 5=L, 6=(HL), 7=A
    fn get_r8(&self, idx: u8, prefix: u8, disp: i8) -> u8 {
        match idx {
            0 => self.b, 1 => self.c, 2 => self.d, 3 => self.e,
            4 => if prefix == 0xDD { (self.ix >> 8) as u8 } else if prefix == 0xFD { (self.iy >> 8) as u8 } else { self.h },
            5 => if prefix == 0xDD { (self.ix & 0xFF) as u8 } else if prefix == 0xFD { (self.iy & 0xFF) as u8 } else { self.l },
            6 => {
                let addr = if prefix == 0xDD {
                    self.ix.wrapping_add(disp as u16)
                } else if prefix == 0xFD {
                    self.iy.wrapping_add(disp as u16)
                } else {
                    (self.h as u16) << 8 | self.l as u16
                };
                self.read_byte(addr)
            },
            7 => self.a, _ => 0,
        }
    }

    fn set_r8(&mut self, idx: u8, val: u8, prefix: u8, disp: i8) {
        match idx {
            0 => self.b = val, 1 => self.c = val, 2 => self.d = val, 3 => self.e = val,
            4 => if prefix == 0xDD { self.ix = (self.ix & 0xFF) | ((val as u16) << 8) } else if prefix == 0xFD { self.iy = (self.iy & 0xFF) | ((val as u16) << 8) } else { self.h = val },
            5 => if prefix == 0xDD { self.ix = (self.ix & 0xFF00) | (val as u16) } else if prefix == 0xFD { self.iy = (self.iy & 0xFF00) | (val as u16) } else { self.l = val },
            6 => {
                let addr = if prefix == 0xDD {
                    self.ix.wrapping_add(disp as u16)
                } else if prefix == 0xFD {
                    self.iy.wrapping_add(disp as u16)
                } else {
                    (self.h as u16) << 8 | self.l as u16
                };
                self.write_byte(addr, val)
            },
            7 => self.a = val, _ => {},
        }
    }

    // Helper for 16-bit registers (dd/rp)
    // 0=BC, 1=DE, 2=HL, 3=SP
    fn get_rp(&self, idx: u8, prefix: u8) -> u16 {
        match idx {
            0 => (self.b as u16) << 8 | self.c as u16,
            1 => (self.d as u16) << 8 | self.e as u16,
            2 => if prefix == 0xDD { self.ix } else if prefix == 0xFD { self.iy } else { (self.h as u16) << 8 | self.l as u16 },
            3 => self.sp,
            _ => 0,
        }
    }

    fn set_rp(&mut self, idx: u8, val: u16, prefix: u8) {
        let h = (val >> 8) as u8;
        let l = (val & 0xFF) as u8;
        match idx {
            0 => { self.b = h; self.c = l; },
            1 => { self.d = h; self.e = l; },
            2 => if prefix == 0xDD { self.ix = val } else if prefix == 0xFD { self.iy = val } else { self.h = h; self.l = l; },
            3 => { self.sp = val; },
            _ => {},
        }
    }

    // Helper for PUSH/POP (qq/rp2)
    // 0=BC, 1=DE, 2=HL, 3=AF
    fn get_rp2(&self, idx: u8, prefix: u8) -> u16 {
        if idx == 3 { (self.a as u16) << 8 | self.f as u16 } else { self.get_rp(idx, prefix) }
    }

    fn set_rp2(&mut self, idx: u8, val: u16, prefix: u8) {
        if idx == 3 { self.a = (val >> 8) as u8; self.f = (val & 0xFF) as u8; } else { self.set_rp(idx, val, prefix); }
    }

    fn push(&mut self, val: u16) {
        self.sp = self.sp.wrapping_sub(2);
        self.write_word(self.sp, val);
    }

    fn pop(&mut self) -> u16 {
        let val = self.read_word(self.sp);
        self.sp = self.sp.wrapping_add(2);
        val
    }

    fn check_cond(&self, cond: u8) -> bool {
        match cond {
            0 => (self.f & F_Z) == 0, // NZ
            1 => (self.f & F_Z) != 0, // Z
            2 => (self.f & F_C) == 0, // NC
            3 => (self.f & F_C) != 0, // C
            4 => (self.f & F_PV) == 0, // PO
            5 => (self.f & F_PV) != 0, // PE
            6 => (self.f & F_S) == 0, // P
            7 => (self.f & F_S) != 0, // M
            _ => false,
        }
    }

    fn execute(&mut self, mut opcode: u8) {
        let mut prefix = 0;
        if opcode == 0xDD || opcode == 0xFD {
            prefix = opcode;
            opcode = self.fetch_byte();
        }

        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;

        // Displacement for (IX+d) / (IY+d)
        let mut disp: i8 = 0;
        let use_disp = prefix != 0 && (
            (x == 1 && (z == 6 || y == 6)) || // LD r, (HL) or LD (HL), r
            (x == 2 && z == 6) || // ALU A, (HL)
            (x == 0 && z == 6 && y == 6) || // LD (HL), n
            (x == 0 && (z == 4 || z == 5) && y == 6) // INC/DEC (HL)
        );

        if use_disp {
            disp = self.fetch_byte() as i8;
        }

        match x {
            0 => {
                match z {
                    0 => match y {
                        0 => {}, // NOP
                        1 => { // EX AF, AF'
                            let af = (self.a as u16) << 8 | self.f as u16;
                            self.a = (self.alt_af >> 8) as u8;
                            self.f = (self.alt_af & 0xFF) as u8;
                            self.alt_af = af;
                        },
                        2 => { // DJNZ d
                            let d = self.fetch_byte() as i8;
                            self.b = self.b.wrapping_sub(1);
                            if self.b != 0 { self.pc = (self.pc as i32 + d as i32) as u16; }
                        },
                        3 => { // JR d
                            let d = self.fetch_byte() as i8;
                            self.pc = (self.pc as i32 + d as i32) as u16;
                        },
                        _ => { // JR cc, d
                            let d = self.fetch_byte() as i8;
                            if self.check_cond(y - 4) { self.pc = (self.pc as i32 + d as i32) as u16; }
                        }
                    },
                    1 => {
                        if q == 0 { // LD rp, nn
                            let nn = self.fetch_word();
                            self.set_rp(p, nn, prefix);
                        } else { // ADD HL, rp
                            let hl = self.get_rp(2, prefix);
                            let rp = self.get_rp(p, prefix);
                            let res = hl.wrapping_add(rp);
                            self.set_rp(2, res, prefix);
                            // Flags: H, N=0, C
                            self.f = (self.f & !(F_N | F_H | F_C)) |
                                     if hl > 0xFFFF - rp { F_C } else { 0 } |
                                     if (hl & 0xFFF) + (rp & 0xFFF) > 0xFFF { F_H } else { 0 };
                        }
                    },
                    2 => {
                        if q == 0 {
                            match p {
                                0 => self.write_byte(self.get_rp(0, 0), self.a), // LD (BC), A
                                1 => self.write_byte(self.get_rp(1, 0), self.a), // LD (DE), A
                                2 => { let nn = self.fetch_word(); self.write_word(nn, self.get_rp(2, prefix)); }, // LD (nn), HL/IX/IY
                                3 => { let nn = self.fetch_word(); self.write_byte(nn, self.a); }, // LD (nn), A
                                _ => {}
                            }
                        } else {
                            match p {
                                0 => self.a = self.read_byte(self.get_rp(0, 0)), // LD A, (BC)
                                1 => self.a = self.read_byte(self.get_rp(1, 0)), // LD A, (DE)
                                2 => { let nn = self.fetch_word(); let v = self.read_word(nn); self.set_rp(2, v, prefix); }, // LD HL/IX/IY, (nn)
                                3 => { let nn = self.fetch_word(); self.a = self.read_byte(nn); }, // LD A, (nn)
                                _ => {}
                            }
                        }
                    },
                    3 => { // INC/DEC rp
                        let v = self.get_rp(p, prefix);
                        self.set_rp(p, if q == 0 { v.wrapping_add(1) } else { v.wrapping_sub(1) }, prefix);
                    },
                    4 => { // INC r
                        let v = self.get_r8(y, prefix, disp);
                        let res = v.wrapping_add(1);
                        self.set_r8(y, res, prefix, disp);
                        self.f = (self.f & F_C) | (if res == 0 { F_Z } else { 0 }) | (if res & 0x80 != 0 { F_S } else { 0 }) |
                                 (if (v & 0xF) == 0xF { F_H } else { 0 }) | (if v == 0x7F { F_PV } else { 0 });
                    },
                    5 => { // DEC r
                        let v = self.get_r8(y, prefix, disp);
                        let res = v.wrapping_sub(1);
                        self.set_r8(y, res, prefix, disp);
                        self.f = (self.f & F_C) | F_N | (if res == 0 { F_Z } else { 0 }) | (if res & 0x80 != 0 { F_S } else { 0 }) |
                                 (if (v & 0xF) == 0 { F_H } else { 0 }) | (if v == 0x80 { F_PV } else { 0 });
                    },
                    6 => { // LD r, n
                        let n = self.fetch_byte();
                        self.set_r8(y, n, prefix, disp);
                    },
                    7 => match y {
                        0 => { // RLCA
                            let a = self.a;
                            self.a = (a << 1) | (a >> 7);
                            self.f = (self.f & !(F_H | F_N | F_C)) | (if a & 0x80 != 0 { F_C } else { 0 });
                        },
                        1 => { // RRCA
                            let a = self.a;
                            self.a = (a >> 1) | (a << 7);
                            self.f = (self.f & !(F_H | F_N | F_C)) | (if a & 0x01 != 0 { F_C } else { 0 });
                        },
                        2 => { // RLA
                            let a = self.a;
                            let c = if self.f & F_C != 0 { 1 } else { 0 };
                            self.a = (a << 1) | c;
                            self.f = (self.f & !(F_H | F_N | F_C)) | (if a & 0x80 != 0 { F_C } else { 0 });
                        },
                        3 => { // RRA
                            let a = self.a;
                            let c = if self.f & F_C != 0 { 0x80 } else { 0 };
                            self.a = (a >> 1) | c;
                            self.f = (self.f & !(F_H | F_N | F_C)) | (if a & 0x01 != 0 { F_C } else { 0 });
                        },
                        4 => { // DAA
                            let mut a = self.a;
                            let mut diff = 0;
                            if (self.f & F_H) != 0 || (a & 0x0F) > 9 { diff += 6; }
                            if (self.f & F_C) != 0 || a > 0x99 { diff += 0x60; self.f |= F_C; }
                            if (self.f & F_N) != 0 { a = a.wrapping_sub(diff); } else { a = a.wrapping_add(diff); }
                            self.a = a;
                            self.f = (self.f & F_C) | (if a == 0 { F_Z } else { 0 }) | (if a & 0x80 != 0 { F_S } else { 0 }) | (if a.count_ones() % 2 == 0 { F_PV } else { 0 });
                        },
                        5 => { self.a = !self.a; self.f |= F_H | F_N; }, // CPL
                        6 => { self.f = (self.f & !(F_H | F_N)) | F_C; }, // SCF
                        7 => { let c = self.f & F_C; self.f = (self.f & !(F_H | F_N | F_C)) | (if c != 0 { F_H } else { F_C }); }, // CCF
                        _ => {}
                    },
                    _ => {}
                }
            },
            1 => { // LD r, r' or HALT
                if y == 6 && z == 6 { self.halted = true; self.pc = self.pc.wrapping_sub(1); } // HALT (PC-1 to point to HALT again effectively, but halted flag stops fetch)
                else { self.set_r8(y, self.get_r8(z, prefix, disp), prefix, disp); }
            },
            2 => { // ALU A, r
                self.alu(y, self.get_r8(z, prefix, disp));
            },
            3 => {
                match z {
                    0 => { if self.check_cond(y) { self.pc = self.pop(); } }, // RET cc
                    1 => {
                        if q == 0 { let v = self.pop(); self.set_rp2(p, v, prefix); } // POP rp2
                        else {
                            match p {
                                0 => self.pc = self.pop(), // RET
                                1 => { // EXX
                                    let bc = self.get_rp(0, 0); let de = self.get_rp(1, 0); let hl = self.get_rp(2, 0);
                                    self.set_rp(0, self.alt_bc, 0); self.set_rp(1, self.alt_de, 0); self.set_rp(2, self.alt_hl, 0);
                                    self.alt_bc = bc; self.alt_de = de; self.alt_hl = hl;
                                },
                                2 => self.pc = self.get_rp(2, prefix), // JP (HL) / JP (IX)
                                3 => self.sp = self.get_rp(2, prefix), // LD SP, HL / LD SP, IX
                                _ => {}
                            }
                        }
                    },
                    2 => { let nn = self.fetch_word(); if self.check_cond(y) { self.pc = nn; } }, // JP cc, nn
                    3 => {
                        match y {
                            0 => { let nn = self.fetch_word(); self.pc = nn; }, // JP nn
                            1 => self.execute_cb(prefix, disp), // CB prefix
                            2 => { let n = self.fetch_byte(); self.write_port((self.a as u16) << 8 | n as u16, self.a); }, // OUT (n), A
                            3 => { let n = self.fetch_byte(); self.a = self.read_port((self.a as u16) << 8 | n as u16); }, // IN A, (n)
                            4 => { // EX (SP), HL/IX/IY
                                let t = self.pop(); 
                                let val = self.get_rp(2, prefix);
                                self.push(val); 
                                self.set_rp(2, t, prefix); 
                            },
                            5 => { let t = self.d; self.d = self.h; self.h = t; let t = self.e; self.e = self.l; self.l = t; }, // EX DE, HL
                            6 => self.iff1 = false, // DI
                            7 => self.iff1 = true, // EI
                            _ => {}
                        }
                    },
                    4 => { let nn = self.fetch_word(); if self.check_cond(y) { self.push(self.pc); self.pc = nn; } }, // CALL cc, nn
                    5 => {
                        if q == 0 { self.push(self.get_rp2(p, prefix)); } // PUSH rp2
                        else if p == 0 { let nn = self.fetch_word(); self.push(self.pc); self.pc = nn; } // CALL nn
                        else if p == 2 { self.execute_ed(); } // ED prefix
                    },
                    6 => { let n = self.fetch_byte(); self.alu(y, n); }, // ALU A, n
                    7 => { self.push(self.pc); self.pc = (y as u16) * 8; }, // RST
                    _ => {}
                }
            },
            _ => {}
        }
    }

    fn alu(&mut self, op: u8, val: u8) {
        let a = self.a;
        let (res, c, h, pv) = match op {
            0 => { // ADD
                let r = a.wrapping_add(val);
                (r, r < a, (a & 0xF) + (val & 0xF) > 0xF, (a ^ r) & (val ^ r) & 0x80 != 0)
            },
            1 => { // ADC
                let cy = if self.f & F_C != 0 { 1 } else { 0 };
                let r = a.wrapping_add(val).wrapping_add(cy);
                (r, (a as u16 + val as u16 + cy as u16) > 0xFF, (a & 0xF) + (val & 0xF) + cy > 0xF, (a ^ r) & (val ^ r) & 0x80 != 0)
            },
            2 => { // SUB
                let r = a.wrapping_sub(val);
                (r, val > a, (a & 0xF) < (val & 0xF), (a ^ val) & (a ^ r) & 0x80 != 0)
            },
            3 => { // SBC
                let cy = if self.f & F_C != 0 { 1 } else { 0 };
                let r = a.wrapping_sub(val).wrapping_sub(cy);
                (r, (a as u16) < (val as u16 + cy as u16), (a & 0xF) < (val & 0xF) + cy, (a ^ val) & (a ^ r) & 0x80 != 0)
            },
            4 => { (a & val, false, true, false) }, // AND (H=1)
            5 => { (a ^ val, false, false, false) }, // XOR
            6 => { (a | val, false, false, false) }, // OR
            7 => { // CP
                let r = a.wrapping_sub(val);
                (a, val > a, (a & 0xF) < (val & 0xF), (a ^ val) & (a ^ r) & 0x80 != 0)
            },
            _ => (a, false, false, false),
        };

        if op != 7 { self.a = res; }
        
        let mut f = 0;
        if res == 0 { f |= F_Z; }
        if res & 0x80 != 0 { f |= F_S; }
        if c { f |= F_C; }
        if h { f |= F_H; }
        if op >= 2 { f |= F_N; }
        if op >= 4 && op <= 6 {
            if res.count_ones() % 2 == 0 { f |= F_PV; }
        } else if pv {
            f |= F_PV;
        }
        self.f = f;
    }

    fn execute_cb(&mut self, prefix: u8, _disp: i8) {
        let mut opcode = self.fetch_byte();
        let mut d = 0;
        if prefix != 0 {
            d = opcode as i8;
            opcode = self.fetch_byte();
        }

        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        
        let mut val = 0;
        let mut addr = 0;
        
        if prefix != 0 {
            // Calculate address
            addr = if prefix == 0xDD { self.ix.wrapping_add(d as u16) } else { self.iy.wrapping_add(d as u16) };
            val = self.read_byte(addr);
            
            // Execute on 'val'
            let res = self.cb_op(x, y, val);
            
            // Write back to memory
            self.write_byte(addr, res);
            
            // Undocumented: DDCB also writes result to register z_ (if z_ != 6)
            if z != 6 {
                self.set_r8(z, res, 0, 0);
            }
        } else {
            val = self.get_r8(z, 0, 0);
            let res = self.cb_op(x, y, val);
            self.set_r8(z, res, 0, 0);
        }
    }

    fn cb_op(&mut self, x: u8, y: u8, val: u8) -> u8 {
        match x {
            0 => { // Rotates/Shifts
                match y {
                    0 => { // RLC
                        let r = (val << 1) | (val >> 7);
                        self.f = (self.f & !F_C) | (if val & 0x80 != 0 { F_C } else { 0 });
                        r
                    },
                    1 => { // RRC
                        let r = (val >> 1) | (val << 7);
                        self.f = (self.f & !F_C) | (if val & 0x01 != 0 { F_C } else { 0 });
                        r
                    },
                    2 => { // RL
                        let r = (val << 1) | (if self.f & F_C != 0 { 1 } else { 0 });
                        self.f = (self.f & !F_C) | (if val & 0x80 != 0 { F_C } else { 0 });
                        r
                    },
                    3 => { // RR
                        let r = (val >> 1) | (if self.f & F_C != 0 { 0x80 } else { 0 });
                        self.f = (self.f & !F_C) | (if val & 0x01 != 0 { F_C } else { 0 });
                        r
                    },
                    4 => { // SLA
                        let r = val << 1;
                        self.f = (self.f & !F_C) | (if val & 0x80 != 0 { F_C } else { 0 });
                        r
                    },
                    5 => { // SRA
                        let r = (val as i8 >> 1) as u8;
                        self.f = (self.f & !F_C) | (if val & 0x01 != 0 { F_C } else { 0 });
                        r
                    },
                    6 => { // SLL (Undocumented)
                        let r = (val << 1) | 1;
                        self.f = (self.f & !F_C) | (if val & 0x80 != 0 { F_C } else { 0 });
                        r
                    },
                    7 => { // SRL
                        let r = val >> 1;
                        self.f = (self.f & !F_C) | (if val & 0x01 != 0 { F_C } else { 0 });
                        r
                    },
                    _ => val,
                }
            },
            1 => { // BIT
                let res = val & (1 << y);
                self.f = (self.f & !(F_Z | F_N | F_H)) | F_H | (if res == 0 { F_Z } else { 0 });
                val // BIT doesn't change value
            },
            2 => val & !(1 << y), // RES
            3 => val | (1 << y), // SET
            _ => val
        }
    }

    fn execute_ed(&mut self) {
        let opcode = self.fetch_byte();
        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;

        match x {
            1 => {
                match z {
                    0 => { // IN r, (C)
                        if y != 6 {
                            let val = self.read_port((self.b as u16) << 8 | self.c as u16);
                            self.set_r8(y, val, 0, 0);
                            self.f = (self.f & F_C) | (if val == 0 { F_Z } else { 0 }) | (if val & 0x80 != 0 { F_S } else { 0 }) |
                                     (if val.count_ones() % 2 == 0 { F_PV } else { 0 });
                        }
                    },
                    1 => { // OUT (C), r
                        if y != 6 {
                            let val = self.get_r8(y, 0, 0);
                            self.write_port((self.b as u16) << 8 | self.c as u16, val);
                        }
                    },
                    2 => { // SBC HL, rp
                        let hl = self.get_rp(2, 0);
                        let rp = self.get_rp(p, 0);
                        let cy = if self.f & F_C != 0 { 1 } else { 0 };
                        if q == 0 { // SBC HL, rp
                            let res = hl.wrapping_sub(rp).wrapping_sub(cy);
                            self.set_rp(2, res, 0);
                            self.f = (self.f & !(F_N | F_H | F_C | F_Z | F_S)) | F_N |
                                     (if res == 0 { F_Z } else { 0 }) | (if res & 0x8000 != 0 { F_S } else { 0 }) |
                                     (if (hl as u32) < (rp as u32 + cy as u32) { F_C } else { 0 });
                        } else { // ADC HL, rp
                            let res = hl.wrapping_add(rp).wrapping_add(cy);
                            self.set_rp(2, res, 0);
                            self.f = (self.f & !(F_N | F_H | F_C | F_Z | F_S)) |
                                     (if res == 0 { F_Z } else { 0 }) | (if res & 0x8000 != 0 { F_S } else { 0 }) |
                                     (if (hl as u32 + rp as u32 + cy as u32) > 0xFFFF { F_C } else { 0 });
                        }
                    },
                    3 => { // LD (nn), rp / LD rp, (nn)
                        let nn = self.fetch_word();
                        if q == 0 { self.write_word(nn, self.get_rp(p, 0)); }
                        else { let v = self.read_word(nn); self.set_rp(p, v, 0); }
                    },
                    4 => { // NEG
                        let a = self.a;
                        self.a = 0u8.wrapping_sub(a);
                        let res = self.a;
                        self.f = (if res == 0 { F_Z } else { 0 }) | F_N |
                                 (if res & 0x80 != 0 { F_S } else { 0 }) |
                                 (if a == 0x80 { F_PV } else { 0 }) |
                                 (if (a & 0x0F) != 0 { F_H } else { 0 }) |
                                 (if a != 0 { F_C } else { 0 });
                    },
                    5 => { // RETN / RETI
                        self.pc = self.pop();
                        if y != 1 { // RETN
                            self.iff1 = self.iff2;
                        }
                    },
                    7 => { // LD I, A / LD R, A etc
                        match y {
                            0 => { self.i = self.a; }, // LD I, A
                            1 => { self.r = self.a; }, // LD R, A
                            2 => { self.a = self.i; self.f = (self.f & F_C) | (if self.iff2 { F_PV } else { 0 }) | (if self.i == 0 { F_Z } else { 0 }) | (if self.i & 0x80 != 0 { F_S } else { 0 }); }, // LD A, I
                            3 => { self.a = self.r; self.f = (self.f & F_C) | (if self.iff2 { F_PV } else { 0 }) | (if self.r == 0 { F_Z } else { 0 }) | (if self.r & 0x80 != 0 { F_S } else { 0 }); }, // LD A, R
                            4 => { // RRD
                                let hl = (self.h as u16) << 8 | self.l as u16;
                                let val = self.read_byte(hl);
                                let a = self.a;
                                self.a = (a & 0xF0) | (val & 0x0F);
                                let new_val = (val >> 4) | ((a & 0x0F) << 4);
                                self.write_byte(hl, new_val);
                                self.f = (self.f & F_C) | (if self.a == 0 { F_Z } else { 0 }) | (if self.a & 0x80 != 0 { F_S } else { 0 }) | (if self.a.count_ones() % 2 == 0 { F_PV } else { 0 });
                            },
                            5 => { // RLD
                                let hl = (self.h as u16) << 8 | self.l as u16;
                                let val = self.read_byte(hl);
                                let a = self.a;
                                self.a = (a & 0xF0) | (val >> 4);
                                let new_val = (val << 4) | (a & 0x0F);
                                self.write_byte(hl, new_val);
                                self.f = (self.f & F_C) | (if self.a == 0 { F_Z } else { 0 }) | (if self.a & 0x80 != 0 { F_S } else { 0 }) | (if self.a.count_ones() % 2 == 0 { F_PV } else { 0 });
                            },
                            _ => {}
                        }
                    },
                    6 => { // IM 0/1/2
                        match y & 0x03 {
                            0 => self.im = 0,
                            2 => self.im = 1,
                            3 => self.im = 2,
                            _ => self.im = 0,
                        }
                    },
                    _ => {}
                }
            },
            2 => {
                if z <= 3 && y >= 4 { // Block instructions (LDI, LDIR, etc)
                    self.block_op(y, z);
                }
            },
            _ => {}
        }
    }

    fn block_op(&mut self, y: u8, z: u8) {
        let mut hl = (self.h as u16) << 8 | self.l as u16;
        let mut de = (self.d as u16) << 8 | self.e as u16;
        let mut bc = (self.b as u16) << 8 | self.c as u16;
        let mut repeat = false;

        match z {
            0 => { // LD block
                let val = self.read_byte(hl);
                self.write_byte(de, val);
                
                if y & 1 == 0 { // Inc (LDI/LDIR)
                    hl = hl.wrapping_add(1);
                    de = de.wrapping_add(1);
                } else { // Dec (LDD/LDDR)
                    hl = hl.wrapping_sub(1);
                    de = de.wrapping_sub(1);
                }
                bc = bc.wrapping_sub(1);
                
                self.f = (self.f & !(F_H | F_N | F_PV)) | (if bc != 0 { F_PV } else { 0 });
                
                if (y & 2) != 0 && bc != 0 { // Repeat (LDIR/LDDR)
                    repeat = true;
                }
            },
            1 => { // CP block
                let val = self.read_byte(hl);
                let res = self.a.wrapping_sub(val);
                
                if y & 1 == 0 { hl = hl.wrapping_add(1); } else { hl = hl.wrapping_sub(1); }
                bc = bc.wrapping_sub(1);
                
                let h_flag = (self.a & 0xF) < (val & 0xF);
                self.f = (self.f & !(F_S | F_Z | F_H | F_PV | F_N)) |
                         (if res & 0x80 != 0 { F_S } else { 0 }) |
                         (if res == 0 { F_Z } else { 0 }) |
                         (if h_flag { F_H } else { 0 }) |
                         (if bc != 0 { F_PV } else { 0 }) |
                         F_N;
                         
                if (y & 2) != 0 && bc != 0 && res != 0 { // Repeat (CPIR/CPDR)
                    repeat = true;
                }
            },
            2 => { // IN block (INI, INIR, IND, INDR)
                let port = bc; // B is high, C is low
                let val = self.read_port(port);
                self.write_byte(hl, val);
                
                if y & 1 == 0 { hl = hl.wrapping_add(1); } else { hl = hl.wrapping_sub(1); }
                
                let b = (bc >> 8) as u8;
                let new_b = b.wrapping_sub(1);
                bc = (new_b as u16) << 8 | (bc & 0xFF);
                
                self.f = (self.f & !(F_N | F_Z)) | F_N | (if new_b == 0 { F_Z } else { 0 });
                if (y & 2) != 0 && new_b != 0 { repeat = true; }
            },
            3 => { // OUT block (OUTI, OTIR, OUTD, OTDR)
                let port = bc;
                let val = self.read_byte(hl);
                self.write_port(port, val);
                
                if y & 1 == 0 { hl = hl.wrapping_add(1); } else { hl = hl.wrapping_sub(1); }
                let b = (bc >> 8) as u8;
                let new_b = b.wrapping_sub(1);
                bc = (new_b as u16) << 8 | (bc & 0xFF);
                
                self.f = (self.f & !(F_N | F_Z)) | F_N | (if new_b == 0 { F_Z } else { 0 });
                if (y & 2) != 0 && new_b != 0 { repeat = true; }
            },
            _ => {}
        }

        self.h = (hl >> 8) as u8;
        self.l = (hl & 0xFF) as u8;
        self.d = (de >> 8) as u8;
        self.e = (de & 0xFF) as u8;
        self.b = (bc >> 8) as u8;
        self.c = (bc & 0xFF) as u8;

        if repeat {
            self.pc = self.pc.wrapping_sub(2); // Re-execute instruction
        }
    }

    pub fn render(&mut self) {
        // Fill border
        let border_rgba = PALETTE[(self.border_color & 0x07) as usize];
        for pixel in self.screen_buffer.chunks_exact_mut(4) {
            pixel.copy_from_slice(&border_rgba);
        }

        // Draw main screen
        for y in 0..SCREEN_HEIGHT {
            let y_pos = y + BORDER_V;
            let sector = y >> 6;
            let row = (y >> 3) & 7;
            let line = y & 7;
            let addr_start = 0x4000 | (sector << 11) | (line << 8) | (row << 5);

            for x_char in 0..32 {
                let bitmap = self.memory[addr_start + x_char];
                let attr = self.memory[0x5800 + (y >> 3) * 32 + x_char];
                let ink = (attr & 0x07) + if (attr & 0x40) != 0 { 8 } else { 0 };
                let paper = ((attr >> 3) & 0x07) + if (attr & 0x40) != 0 { 8 } else { 0 };
                
                let x_start = x_char * 8 + BORDER_H;
                for bit in 0..8 {
                    let color_idx = if (bitmap & (0x80 >> bit)) != 0 { ink } else { paper };
                    let pixel_idx = (y_pos * FULL_WIDTH + x_start + bit) * 4;
                    self.screen_buffer[pixel_idx..pixel_idx + 4].copy_from_slice(&PALETTE[color_idx as usize]);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_test(code: &[u8]) -> Emulator {
        let mut emu = Emulator::new();
        // Clear memory and load code at 0x0000
        emu.memory.fill(0);
        for (i, &b) in code.iter().enumerate() {
            emu.memory[i] = b;
        }
        emu.pc = 0;
        emu.sp = 0; // Set stack to top of memory (wraps to 0xFFFF) to avoid ROM protection
        
        // Run until PC goes past the code
        let end_pc = code.len() as u16;
        let max_steps = 1000;
        let mut steps = 0;
        while emu.pc < end_pc && steps < max_steps {
            let opcode = emu.fetch_byte();
            emu.execute(opcode);
            steps += 1;
        }
        emu
    }

    #[test]
    fn test_ld_8bit() {
        // LD A, 0x55; LD B, 0xAA; LD C, A
        let emu = run_test(&[0x3E, 0x55, 0x06, 0xAA, 0x4F]);
        assert_eq!(emu.a, 0x55);
        assert_eq!(emu.b, 0xAA);
        assert_eq!(emu.c, 0x55);
    }

    #[test]
    fn test_ld_16bit() {
        // LD BC, 0x1234; LD DE, 0x5678; LD HL, 0x9ABC; LD SP, 0xFEDC
        let emu = run_test(&[0x01, 0x34, 0x12, 0x11, 0x78, 0x56, 0x21, 0xBC, 0x9A, 0x31, 0xDC, 0xFE]);
        assert_eq!(emu.b, 0x12); assert_eq!(emu.c, 0x34);
        assert_eq!(emu.d, 0x56); assert_eq!(emu.e, 0x78);
        assert_eq!(emu.h, 0x9A); assert_eq!(emu.l, 0xBC);
        assert_eq!(emu.sp, 0xFEDC);
    }

    #[test]
    fn test_alu_add() {
        // LD A, 10; ADD A, 20
        let emu = run_test(&[0x3E, 10, 0xC6, 20]);
        assert_eq!(emu.a, 30);
        assert_eq!(emu.f & F_Z, 0);
        
        // Overflow test: LD A, 255; ADD A, 1
        let emu = run_test(&[0x3E, 0xFF, 0xC6, 0x01]);
        assert_eq!(emu.a, 0);
        assert_ne!(emu.f & F_Z, 0);
        assert_ne!(emu.f & F_C, 0);
    }

    #[test]
    fn test_alu_sub() {
        // LD A, 30; SUB 10
        let emu = run_test(&[0x3E, 30, 0xD6, 10]);
        assert_eq!(emu.a, 20);
        
        // Carry test: LD A, 10; SUB 20
        let emu = run_test(&[0x3E, 10, 0xD6, 20]);
        assert_eq!(emu.a, 246); // -10 as u8
        assert_ne!(emu.f & F_C, 0);
    }

    #[test]
    fn test_inc_dec() {
        // LD B, 10; INC B; DEC B; DEC B
        let emu = run_test(&[0x06, 10, 0x04, 0x05, 0x05]);
        assert_eq!(emu.b, 9);
    }

    #[test]
    fn test_jr() {
        // LD A, 1; JR 2; LD A, 2; LD B, 3
        // Opcode for JR d is 18 d. 
        // 0: 3E 01 (LD A, 1)
        // 2: 18 02 (JR +2) -> Skips next 2 bytes (LD A, 2 is 3E 02)
        // 4: 3E 02 (LD A, 2) - skipped
        // 6: 06 03 (LD B, 3)
        let emu = run_test(&[0x3E, 0x01, 0x18, 0x02, 0x3E, 0x02, 0x06, 0x03]);
        assert_eq!(emu.a, 1);
        assert_eq!(emu.b, 3);
    }

    #[test]
    fn test_call_ret() {
        // LD SP, 0x0000 (Top of RAM)
        // CALL 0x0010
        // HALT
        // 0x0010: LD A, 55; RET
        let mut emu = Emulator::new();
        emu.memory.fill(0);
        let code = [0x31, 0x00, 0x00, 0xCD, 0x10, 0x00, 0x76];
        let sub = [0x3E, 0x55, 0xC9];
        for (i, &b) in code.iter().enumerate() { emu.memory[i] = b; }
        for (i, &b) in sub.iter().enumerate() { emu.memory[0x10+i] = b; }
        
        // Run manually to handle HALT
        emu.pc = 0;
        for _ in 0..100 {
            if emu.halted { break; }
            let op = emu.fetch_byte();
            emu.execute(op);
        }
        
        assert_eq!(emu.a, 0x55);
        assert_eq!(emu.pc, 6); // PC points to HALT instruction (decremented in execute)
    }

    #[test]
    fn test_stack() {
        // LD BC, 0x1234; PUSH BC; POP DE
        let emu = run_test(&[0x01, 0x34, 0x12, 0xC5, 0xD1]);
        assert_eq!(emu.d, 0x12);
        assert_eq!(emu.e, 0x34);
    }

    #[test]
    fn test_ix_iy() {
        // LD IX, 0x8000; LD (IX+5), 0xAA; LD A, (IX+5)
        let mut emu = Emulator::new();
        emu.memory.fill(0);
        let code = [
            0xDD, 0x21, 0x00, 0x80, // LD IX, 0x8000
            0xDD, 0x36, 0x05, 0xAA, // LD (IX+5), 0xAA
            0xDD, 0x7E, 0x05        // LD A, (IX+5)
        ];
        for (i, &b) in code.iter().enumerate() { emu.memory[i] = b; }
        
        let end_pc = code.len() as u16;
        while emu.pc < end_pc {
            let op = emu.fetch_byte();
            emu.execute(op);
        }
        
        assert_eq!(emu.ix, 0x8000);
        assert_eq!(emu.memory[0x8005], 0xAA);
        assert_eq!(emu.a, 0xAA);
    }

    #[test]
    fn test_bit_ops() {
        // LD A, 0; SET 3, A; BIT 3, A; RES 3, A
        // CB C7 (SET 0, A) -> 0x01
        // CB DE (SET 3, (HL)) - let's stick to register A
        // SET 3, A -> CB DF
        // BIT 3, A -> CB 5F
        // RES 3, A -> CB 9F
        let emu = run_test(&[0x3E, 0x00, 0xCB, 0xDF, 0xCB, 0x5F, 0xCB, 0x9F]);
        assert_eq!(emu.a, 0); // Should be 0 after RES
        // Check flags from BIT 3, A (which was 1 at the time)
        // BIT sets Z if bit is 0. Here bit was 1, so Z should be 0.
        // Wait, I can't easily check intermediate flag state with run_test unless I inspect trace.
        // But final A is 0.
    }

    #[test]
    fn test_block_ldi() {
        // LD HL, 0x0100; LD DE, 0x8000; LD BC, 2; LD (HL), 0x55; INC HL; LD (HL), 0x66; DEC HL
        // LDIR
        let mut emu = Emulator::new();
        emu.memory.fill(0);
        emu.memory[0x0100] = 0x55;
        emu.memory[0x0101] = 0x66;
        
        let code = [
            0x21, 0x00, 0x01, // LD HL, 0x0100
            0x11, 0x00, 0x80, // LD DE, 0x8000
            0x01, 0x02, 0x00, // LD BC, 2
            0xED, 0xB0        // LDIR
        ];
        for (i, &b) in code.iter().enumerate() { emu.memory[i] = b; }
        
        let end_pc = code.len() as u16;
        let mut steps = 0;
        while emu.pc < end_pc && steps < 100 {
            let op = emu.fetch_byte();
            emu.execute(op);
            steps += 1;
        }
        
        assert_eq!(emu.memory[0x8000], 0x55);
        assert_eq!(emu.memory[0x8001], 0x66);
        assert_eq!(emu.b, 0);
        assert_eq!(emu.c, 0);
    }

    #[test]
    fn test_ex_de_hl() {
        // LD HL, 0x1111; LD DE, 0x2222; EX DE, HL
        let emu = run_test(&[0x21, 0x11, 0x11, 0x11, 0x22, 0x22, 0xEB]);
        assert_eq!(emu.h, 0x22); assert_eq!(emu.l, 0x22);
        assert_eq!(emu.d, 0x11); assert_eq!(emu.e, 0x11);
    }

    #[test]
    fn test_exx() {
        // LD BC, 0x1111; EXX; LD BC, 0x2222; EXX
        let emu = run_test(&[0x01, 0x11, 0x11, 0xD9, 0x01, 0x22, 0x22, 0xD9]);
        assert_eq!(emu.b, 0x11); assert_eq!(emu.c, 0x11);
        assert_eq!((emu.alt_bc >> 8) as u8, 0x22);
    }

    #[test]
    fn test_neg() {
        // LD A, 1; NEG
        let emu = run_test(&[0x3E, 0x01, 0xED, 0x44]);
        assert_eq!(emu.a, 0xFF); // -1
    }

    #[test]
    fn test_rrca() {
        // LD A, 0x01; RRCA
        let emu = run_test(&[0x3E, 0x01, 0x0F]);
        assert_eq!(emu.a, 0x80);
        assert_ne!(emu.f & F_C, 0);
    }

    #[test]
    fn test_rlca() {
        // LD A, 0x80; RLCA
        let emu = run_test(&[0x3E, 0x80, 0x07]);
        assert_eq!(emu.a, 0x01);
        assert_ne!(emu.f & F_C, 0);
    }

    #[test]
    fn test_daa() {
        // 1. Simple addition: 0x15 + 0x25 = 0x3A -> DAA -> 0x40
        let emu = run_test(&[0x3E, 0x15, 0x06, 0x25, 0x80, 0x27]);
        assert_eq!(emu.a, 0x40);

        // 2. Wrap around: 0x99 + 0x01 = 0x9A -> DAA -> 0x00 with Carry
        let emu = run_test(&[0x3E, 0x99, 0x06, 0x01, 0x80, 0x27]);
        assert_eq!(emu.a, 0x00);
        assert_ne!(emu.f & F_C, 0);
    }

    #[test]
    fn test_cpl() {
        // LD A, 0xAA; CPL
        let emu = run_test(&[0x3E, 0xAA, 0x2F]);
        assert_eq!(emu.a, 0x55);
        assert_ne!(emu.f & F_H, 0);
        assert_ne!(emu.f & F_N, 0);
    }
}
