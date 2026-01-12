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
            iff1: false, iff2: false, im: 0,
            alt_af: 0, alt_bc: 0, alt_de: 0, alt_hl: 0,
        };
        emu.load_rom(); // Завантаж ROM
        emu
    }

    pub fn load_rom(&mut self) {
        // Завантажити ROM у верхню частину памʼяті
        // (Ви можете зчитати .rom файл пізніше)
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

    pub fn step(&mut self) {
        for _ in 0..10000 {
            let opcode = self.fetch_byte();
            self.execute(opcode);
        }
    }

    fn fetch_byte(&mut self) -> u8 {
        let val = self.memory[self.pc as usize];
        self.pc = self.pc.wrapping_add(1);
        self.r = self.r.wrapping_add(1);
        val
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
    fn get_r8(&self, idx: u8) -> u8 {
        match idx {
            0 => self.b, 1 => self.c, 2 => self.d, 3 => self.e,
            4 => self.h, 5 => self.l, 6 => self.read_byte((self.h as u16) << 8 | self.l as u16),
            7 => self.a, _ => 0,
        }
    }

    fn set_r8(&mut self, idx: u8, val: u8) {
        match idx {
            0 => self.b = val, 1 => self.c = val, 2 => self.d = val, 3 => self.e = val,
            4 => self.h = val, 5 => self.l = val, 6 => self.write_byte((self.h as u16) << 8 | self.l as u16, val),
            7 => self.a = val, _ => {},
        }
    }

    // Helper for 16-bit registers (dd/rp)
    // 0=BC, 1=DE, 2=HL, 3=SP
    fn get_rp(&self, idx: u8) -> u16 {
        match idx {
            0 => (self.b as u16) << 8 | self.c as u16,
            1 => (self.d as u16) << 8 | self.e as u16,
            2 => (self.h as u16) << 8 | self.l as u16,
            3 => self.sp,
            _ => 0,
        }
    }

    fn set_rp(&mut self, idx: u8, val: u16) {
        let h = (val >> 8) as u8;
        let l = (val & 0xFF) as u8;
        match idx {
            0 => { self.b = h; self.c = l; },
            1 => { self.d = h; self.e = l; },
            2 => { self.h = h; self.l = l; },
            3 => { self.sp = val; },
            _ => {},
        }
    }

    // Helper for PUSH/POP (qq/rp2)
    // 0=BC, 1=DE, 2=HL, 3=AF
    fn get_rp2(&self, idx: u8) -> u16 {
        if idx == 3 { (self.a as u16) << 8 | self.f as u16 } else { self.get_rp(idx) }
    }

    fn set_rp2(&mut self, idx: u8, val: u16) {
        if idx == 3 { self.a = (val >> 8) as u8; self.f = (val & 0xFF) as u8; } else { self.set_rp(idx, val); }
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

    fn execute(&mut self, opcode: u8) {
        let x = opcode >> 6;
        let y = (opcode >> 3) & 7;
        let z = opcode & 7;
        let p = y >> 1;
        let q = y & 1;

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
                            self.set_rp(p, nn);
                        } else { // ADD HL, rp
                            let hl = self.get_rp(2);
                            let rp = self.get_rp(p);
                            let res = hl.wrapping_add(rp);
                            self.set_rp(2, res);
                            // Flags: H, N=0, C
                            self.f = (self.f & !(F_N | F_H | F_C)) |
                                     if hl > 0xFFFF - rp { F_C } else { 0 } |
                                     if (hl & 0xFFF) + (rp & 0xFFF) > 0xFFF { F_H } else { 0 };
                        }
                    },
                    2 => {
                        if q == 0 {
                            match p {
                                0 => self.write_byte(self.get_rp(0), self.a), // LD (BC), A
                                1 => self.write_byte(self.get_rp(1), self.a), // LD (DE), A
                                2 => { let nn = self.fetch_word(); self.write_word(nn, self.get_rp(2)); }, // LD (nn), HL
                                3 => { let nn = self.fetch_word(); self.write_byte(nn, self.a); }, // LD (nn), A
                                _ => {}
                            }
                        } else {
                            match p {
                                0 => self.a = self.read_byte(self.get_rp(0)), // LD A, (BC)
                                1 => self.a = self.read_byte(self.get_rp(1)), // LD A, (DE)
                                2 => { let nn = self.fetch_word(); let v = self.read_word(nn); self.set_rp(2, v); }, // LD HL, (nn)
                                3 => { let nn = self.fetch_word(); self.a = self.read_byte(nn); }, // LD A, (nn)
                                _ => {}
                            }
                        }
                    },
                    3 => { // INC/DEC rp
                        let v = self.get_rp(p);
                        self.set_rp(p, if q == 0 { v.wrapping_add(1) } else { v.wrapping_sub(1) });
                    },
                    4 => { // INC r
                        let v = self.get_r8(y);
                        let res = v.wrapping_add(1);
                        self.set_r8(y, res);
                        self.f = (self.f & F_C) | (if res == 0 { F_Z } else { 0 }) | (if res & 0x80 != 0 { F_S } else { 0 }) |
                                 (if (v & 0xF) == 0xF { F_H } else { 0 }) | (if v == 0x7F { F_PV } else { 0 });
                    },
                    5 => { // DEC r
                        let v = self.get_r8(y);
                        let res = v.wrapping_sub(1);
                        self.set_r8(y, res);
                        self.f = (self.f & F_C) | F_N | (if res == 0 { F_Z } else { 0 }) | (if res & 0x80 != 0 { F_S } else { 0 }) |
                                 (if (v & 0xF) == 0 { F_H } else { 0 }) | (if v == 0x80 { F_PV } else { 0 });
                    },
                    6 => { // LD r, n
                        let n = self.fetch_byte();
                        self.set_r8(y, n);
                    },
                    7 => { /* RLCA, RRCA, etc. - simplified */ },
                    _ => {}
                }
            },
            1 => { // LD r, r' or HALT
                if y == 6 && z == 6 { self.pc = self.pc.wrapping_sub(1); } // HALT
                else { self.set_r8(y, self.get_r8(z)); }
            },
            2 => { // ALU A, r
                self.alu(y, self.get_r8(z));
            },
            3 => {
                match z {
                    0 => { if self.check_cond(y) { self.pc = self.pop(); } }, // RET cc
                    1 => {
                        if q == 0 { let v = self.pop(); self.set_rp2(p, v); } // POP rp2
                        else {
                            match p {
                                0 => self.pc = self.pop(), // RET
                                1 => { // EXX
                                    let bc = self.get_rp(0); let de = self.get_rp(1); let hl = self.get_rp(2);
                                    self.set_rp(0, self.alt_bc); self.set_rp(1, self.alt_de); self.set_rp(2, self.alt_hl);
                                    self.alt_bc = bc; self.alt_de = de; self.alt_hl = hl;
                                },
                                2 => self.pc = self.get_rp(2), // JP (HL)
                                3 => self.sp = self.get_rp(2), // LD SP, HL
                                _ => {}
                            }
                        }
                    },
                    2 => { let nn = self.fetch_word(); if self.check_cond(y) { self.pc = nn; } }, // JP cc, nn
                    3 => {
                        match y {
                            0 => { let nn = self.fetch_word(); self.pc = nn; }, // JP nn
                            2 => { let _n = self.fetch_byte(); /* OUT */ },
                            3 => { let _n = self.fetch_byte(); /* IN */ },
                            _ => {}
                        }
                    },
                    4 => { let nn = self.fetch_word(); if self.check_cond(y) { self.push(self.pc); self.pc = nn; } }, // CALL cc, nn
                    5 => {
                        if q == 0 { self.push(self.get_rp2(p)); } // PUSH rp2
                        else if p == 0 { let nn = self.fetch_word(); self.push(self.pc); self.pc = nn; } // CALL nn
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
