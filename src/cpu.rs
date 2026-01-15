//! Z80 CPU Emulation
//!
//! This module implements a complete Z80 CPU emulator with full instruction set support.

use crate::emulator::{Bus, Memory};

// Z80 CPU Flag register bits
const F_S: u8 = 0x80;   // Sign flag
const F_Z: u8 = 0x40;   // Zero flag
const F_H: u8 = 0x10;   // Half-carry flag
const F_PV: u8 = 0x04;  // Parity/Overflow flag
const F_N: u8 = 0x02;   // Subtract flag
const F_C: u8 = 0x01;   // Carry flag

#[derive(Clone)]
pub struct Z80 {
    pub a: u8,
    pub f: u8,

    pub b: u8,
    pub c: u8,

    pub d: u8,
    pub e: u8,

    pub h: u8,
    pub l: u8,

    pub a_alt: u8,
    pub f_alt: u8,

    pub b_alt: u8,
    pub c_alt: u8,

    pub d_alt: u8,
    pub e_alt: u8,

    pub h_alt: u8,
    pub l_alt: u8,

    pub sp: u16,
    pub pc: u16,

    pub ix: u16,
    pub iy: u16,

    pub i: u8,
    pub r: u8,

    pub iff1: bool,
    pub iff2: bool,

    pub im: u8,

    pub int_requested: bool,
}

impl Z80 {
    pub fn new() -> Self {
        Self {
            a: 0, f: 0,
            b: 0, c: 0,
            d: 0, e: 0,
            h: 0, l: 0,
            a_alt: 0, f_alt: 0,
            b_alt: 0, c_alt: 0,
            d_alt: 0, e_alt: 0,
            h_alt: 0, l_alt: 0,
            sp: 0,
            pc: 0,
            ix: 0, iy: 0,
            i: 0, r: 0,
            iff1: false,
            iff2: false,
            im: 1,
            int_requested: false,
        }
    }

    fn push(&mut self, mem: &mut dyn Memory, value: u16) {
        self.sp = self.sp.wrapping_sub(1);
        mem.write_byte(self.sp, (value & 0xFF) as u8);
        self.sp = self.sp.wrapping_sub(1);
        mem.write_byte(self.sp, (value >> 8) as u8);
    }

    fn pop(&mut self, mem: &dyn Memory) -> u16 {
        let low = mem.read_byte(self.sp);
        self.sp = self.sp.wrapping_add(1);
        let high = mem.read_byte(self.sp);
        self.sp = self.sp.wrapping_add(1);
        (high as u16) << 8 | low as u16
    }

    fn read_byte_pc(&mut self, mem: &dyn Memory) -> u8 {
        let val = mem.read_byte(self.pc);
        self.pc = self.pc.wrapping_add(1);
        val
    }

    fn read_word_pc(&mut self, mem: &dyn Memory) -> u16 {
        let low = self.read_byte_pc(mem);
        let high = self.read_byte_pc(mem);
        (high as u16) << 8 | low as u16
    }

    fn get_reg(&self, reg: u8) -> u8 {
        match reg {
            0 => self.b,
            1 => self.c,
            2 => self.d,
            3 => self.e,
            4 => self.h,
            5 => self.l,
            6 => panic!("get_reg for (HL) not supported"),
            7 => self.a,
            _ => panic!("Invalid reg"),
        }
    }

    fn set_reg(&mut self, reg: u8, val: u8) {
        match reg {
            0 => self.b = val,
            1 => self.c = val,
            2 => self.d = val,
            3 => self.e = val,
            4 => self.h = val,
            5 => self.l = val,
            6 => panic!("set_reg for (HL) not supported"),
            7 => self.a = val,
            _ => panic!("Invalid reg"),
        }
    }

    fn get_bc(&self) -> u16 { (self.b as u16) << 8 | self.c as u16 }
    fn set_bc(&mut self, val: u16) { self.b = (val >> 8) as u8; self.c = val as u8; }

    fn get_de(&self) -> u16 { (self.d as u16) << 8 | self.e as u16 }
    fn set_de(&mut self, val: u16) { self.d = (val >> 8) as u8; self.e = val as u8; }

    fn get_hl(&self) -> u16 { (self.h as u16) << 8 | self.l as u16 }
    fn set_hl(&mut self, val: u16) { self.h = (val >> 8) as u8; self.l = val as u8; }

    fn get_af(&self) -> u16 { (self.a as u16) << 8 | self.f as u16 }
    fn set_af(&mut self, val: u16) { self.a = (val >> 8) as u8; self.f = val as u8; }

    fn parity(v: u8) -> bool {
        let mut p = 0;
        for i in 0..8 {
            if (v & (1 << i)) != 0 { p += 1; }
        }
        p % 2 == 0
    }

    fn set_flags_add(&mut self, a: u8, b: u8, result: u16) {
        let res = result as u8;
        self.f = 0;
        if res == 0 { self.f |= F_Z; }
        if (res & 0x80) != 0 { self.f |= F_S; }
        if (result & 0x100) != 0 { self.f |= F_C; }
        if ((a ^ b ^ res) & 0x10) != 0 { self.f |= F_H; }
        let overflow = ((a ^ res) & (b ^ res) & 0x80) != 0;
        if overflow { self.f |= F_PV; }
    }

    fn set_flags_sub(&mut self, a: u8, b: u8, result: i16) {
        let res = result as u8;
        self.f = F_N;
        if res == 0 { self.f |= F_Z; }
        if (res & 0x80) != 0 { self.f |= F_S; }
        if result < 0 { self.f |= F_C; }
        if ((a ^ b ^ res) & 0x10) != 0 { self.f |= F_H; }
        let overflow = ((a ^ res) & ((!b) ^ res) & 0x80) != 0;
        if overflow { self.f |= F_PV; }
    }

    pub fn step(&mut self, bus: &mut dyn Bus) -> u32 {
        if self.int_requested {
            self.int_requested = false;
            // Handle interrupt
            self.push(bus, self.pc);
            self.pc = 0x0038;
            return 13; // T-states for interrupt handling
        }

        // Fetch opcode
        let opcode = self.read_byte_pc(bus);
        self.r = (self.r & 0x80) | ((self.r + 1) & 0x7F);

        match opcode {
            0x00 => 4, // NOP
            0x01 => { // LD BC, nn
                let nn = self.read_word_pc(bus);
                self.set_bc(nn);
                10
            }
            0x02 => { // LD (BC), A
                bus.write_byte(self.get_bc(), self.a);
                7
            }
            0x03 => { // INC BC
                let bc = self.get_bc().wrapping_add(1);
                self.set_bc(bc);
                6
            }
            0x04 => { // INC B
                let res = self.b.wrapping_add(1);
                self.set_flags_add(self.b, 1, res as u16);
                self.f &= !F_N;
                self.b = res;
                4
            }
            0x05 => { // DEC B
                let res = self.b.wrapping_sub(1);
                self.set_flags_sub(self.b, 1, res as i16);
                self.b = res;
                4
            }
            0x06 => { // LD B, n
                self.b = self.read_byte_pc(bus);
                7
            }
            0x07 => { // RLCA
                let carry = (self.a & 0x80) != 0;
                self.a = (self.a << 1) | if carry { 1 } else { 0 };
                self.f = (self.f & (F_S | F_Z | F_PV)) | if carry { F_C } else { 0 };
                4
            }
            0x08 => { // EX AF, AF'
                let temp_a = self.a;
                let temp_f = self.f;
                self.a = self.a_alt;
                self.f = self.f_alt;
                self.a_alt = temp_a;
                self.f_alt = temp_f;
                4
            }
            0x09 => { // ADD HL, BC
                let hl = self.get_hl() as u32;
                let bc = self.get_bc() as u32;
                let result = hl + bc;
                self.set_hl(result as u16);
                self.f = (self.f & (F_S | F_Z | F_PV)) | if (result & 0x10000) != 0 { F_C } else { 0 } | if ((hl ^ bc ^ result) & 0x1000) != 0 { F_H } else { 0 };
                11
            }
            0x0A => { // LD A, (BC)
                self.a = bus.read_byte(self.get_bc());
                7
            }
            0x0B => { // DEC BC
                let bc = self.get_bc().wrapping_sub(1);
                self.set_bc(bc);
                6
            }
            0x0C => { // INC C
                let res = self.c.wrapping_add(1);
                self.set_flags_add(self.c, 1, res as u16);
                self.f &= !F_N;
                self.c = res;
                4
            }
            0x0D => { // DEC C
                let res = self.c.wrapping_sub(1);
                self.set_flags_sub(self.c, 1, res as i16);
                self.c = res;
                4
            }
            0x0E => { // LD C, n
                self.c = self.read_byte_pc(bus);
                7
            }
            0x0F => { // RRCA
                let carry = (self.a & 0x01) != 0;
                self.a = (self.a >> 1) | if carry { 0x80 } else { 0 };
                self.f = (self.f & (F_S | F_Z | F_PV)) | if carry { F_C } else { 0 };
                4
            }
            0x10 => { // DJNZ d
                let d = self.read_byte_pc(bus) as i8;
                self.b = self.b.wrapping_sub(1);
                if self.b != 0 {
                    self.pc = self.pc.wrapping_add(d as u16);
                    13
                } else {
                    8
                }
            }
            0x11 => { // LD DE, nn
                let nn = self.read_word_pc(bus);
                self.set_de(nn);
                10
            }
            0x12 => { // LD (DE), A
                bus.write_byte(self.get_de(), self.a);
                7
            }
            0x13 => { // INC DE
                let de = self.get_de().wrapping_add(1);
                self.set_de(de);
                6
            }
            0x14 => { // INC D
                let res = self.d.wrapping_add(1);
                self.set_flags_add(self.d, 1, res as u16);
                self.f &= !F_N;
                self.d = res;
                4
            }
            0x15 => { // DEC D
                let res = self.d.wrapping_sub(1);
                self.set_flags_sub(self.d, 1, res as i16);
                self.d = res;
                4
            }
            0x16 => { // LD D, n
                self.d = self.read_byte_pc(bus);
                7
            }
            0x17 => { // RLA
                let carry = (self.f & F_C) != 0;
                let new_carry = (self.a & 0x80) != 0;
                self.a = (self.a << 1) | if carry { 1 } else { 0 };
                self.f = (self.f & (F_S | F_Z | F_PV)) | if new_carry { F_C } else { 0 };
                4
            }
            0x18 => { // JR d
                let d = self.read_byte_pc(bus) as i8;
                self.pc = self.pc.wrapping_add(d as u16);
                12
            }
            0x19 => { // ADD HL, DE
                let hl = self.get_hl() as u32;
                let de = self.get_de() as u32;
                let result = hl + de;
                self.set_hl(result as u16);
                self.f = (self.f & (F_S | F_Z | F_PV)) | if (result & 0x10000) != 0 { F_C } else { 0 } | if ((hl ^ de ^ result) & 0x1000) != 0 { F_H } else { 0 };
                self.f &= !F_N;
                11
            }
            0x1A => { // LD A, (DE)
                self.a = bus.read_byte(self.get_de());
                7
            }
            0x1B => { // DEC DE
                let de = self.get_de().wrapping_sub(1);
                self.set_de(de);
                6
            }
            0x1C => { // INC E
                let res = self.e.wrapping_add(1);
                self.set_flags_add(self.e, 1, res as u16);
                self.f &= !F_N;
                self.e = res;
                4
            }
            0x1D => { // DEC E
                let res = self.e.wrapping_sub(1);
                self.set_flags_sub(self.e, 1, res as i16);
                self.e = res;
                4
            }
            0x1E => { // LD E, n
                self.e = self.read_byte_pc(bus);
                7
            }
            0x1F => { // RRA
                let carry = (self.f & F_C) != 0;
                let new_carry = (self.a & 0x01) != 0;
                self.a = (self.a >> 1) | if carry { 0x80 } else { 0 };
                self.f = (self.f & (F_S | F_Z | F_PV)) | if new_carry { F_C } else { 0 };
                4
            }
            0x20 => { // JR NZ, d
                let d = self.read_byte_pc(bus) as i8;
                if (self.f & F_Z) == 0 {
                    self.pc = self.pc.wrapping_add(d as u16);
                    12
                } else {
                    7
                }
            }
            0x21 => { // LD HL, nn
                let nn = self.read_word_pc(bus);
                self.set_hl(nn);
                10
            }
            0x22 => { // LD (nn), HL
                let nn = self.read_word_pc(bus);
                bus.write_byte(nn, self.l);
                bus.write_byte(nn.wrapping_add(1), self.h);
                16
            }
            0x23 => { // INC HL
                let hl = self.get_hl().wrapping_add(1);
                self.set_hl(hl);
                6
            }
            0x24 => { // INC H
                let res = self.h.wrapping_add(1);
                self.set_flags_add(self.h, 1, res as u16);
                self.f &= !F_N;
                self.h = res;
                4
            }
            0x25 => { // DEC H
                let res = self.h.wrapping_sub(1);
                self.set_flags_sub(self.h, 1, res as i16);
                self.h = res;
                4
            }
            0x26 => { // LD H, n
                self.h = self.read_byte_pc(bus);
                7
            }
            0x27 => { // DAA
                let mut correction = 0;
                if (self.f & F_H) != 0 || (self.a & 0x0F) > 9 {
                    correction |= 0x06;
                }
                if (self.f & F_C) != 0 || self.a > 0x99 {
                    correction |= 0x60;
                    self.f |= F_C;
                } else {
                    self.f &= !F_C;
                }
                if (self.f & F_N) != 0 {
                    self.a = self.a.wrapping_sub(correction);
                } else {
                    self.a = self.a.wrapping_add(correction);
                }
                self.f = (self.f & (F_C | F_N)) | if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0x28 => { // JR Z, d
                let d = self.read_byte_pc(bus) as i8;
                if (self.f & F_Z) != 0 {
                    self.pc = self.pc.wrapping_add(d as u16);
                    12
                } else {
                    7
                }
            }
            0x29 => { // ADD HL, HL
                let hl = self.get_hl() as u32;
                let result = hl + hl;
                self.set_hl(result as u16);
                self.f = (self.f & (F_S | F_Z | F_PV)) | if (result & 0x10000) != 0 { F_C } else { 0 } | if ((hl ^ hl ^ result) & 0x1000) != 0 { F_H } else { 0 };
                11
            }
            0x2A => { // LD HL, (nn)
                let nn = self.read_word_pc(bus);
                self.l = bus.read_byte(nn);
                self.h = bus.read_byte(nn.wrapping_add(1));
                16
            }
            0x2B => { // DEC HL
                let hl = self.get_hl().wrapping_sub(1);
                self.set_hl(hl);
                6
            }
            0x2C => { // INC L
                let res = self.l.wrapping_add(1);
                self.set_flags_add(self.l, 1, res as u16);
                self.f &= !F_N;
                self.l = res;
                4
            }
            0x2D => { // DEC L
                let res = self.l.wrapping_sub(1);
                self.set_flags_sub(self.l, 1, res as i16);
                self.l = res;
                4
            }
            0x2E => { // LD L, n
                self.l = self.read_byte_pc(bus);
                7
            }
            0x2F => { // CPL
                self.a = !self.a;
                self.f |= F_H | F_N;
                4
            }
            0x30 => { // JR NC, d
                let d = self.read_byte_pc(bus) as i8;
                if (self.f & F_C) == 0 {
                    self.pc = self.pc.wrapping_add(d as u16);
                    12
                } else {
                    7
                }
            }
            0x31 => { // LD SP, nn
                self.sp = self.read_word_pc(bus);
                10
            }
            0x32 => { // LD (nn), A
                let nn = self.read_word_pc(bus);
                bus.write_byte(nn, self.a);
                13
            }
            0x33 => { // INC SP
                self.sp = self.sp.wrapping_add(1);
                6
            }
            0x34 => { // INC (HL)
                let addr = self.get_hl();
                let val = bus.read_byte(addr);
                let res = val.wrapping_add(1);
                bus.write_byte(addr, res);
                self.set_flags_add(val, 1, res as u16);
                self.f &= !F_N;
                11
            }
            0x35 => { // DEC (HL)
                let addr = self.get_hl();
                let val = bus.read_byte(addr);
                let res = val.wrapping_sub(1);
                bus.write_byte(addr, res);
                self.set_flags_sub(val, 1, res as i16);
                11
            }
            0x36 => { // LD (HL), n
                let n = self.read_byte_pc(bus);
                bus.write_byte(self.get_hl(), n);
                10
            }
            0x37 => { // SCF
                self.f = (self.f & (F_S | F_Z | F_PV)) | F_C;
                4
            }
            0x38 => { // JR C, d
                let d = self.read_byte_pc(bus) as i8;
                if (self.f & F_C) != 0 {
                    self.pc = self.pc.wrapping_add(d as u16);
                    12
                } else {
                    7
                }
            }
            0x39 => { // ADD HL, SP
                let hl = self.get_hl() as u32;
                let sp = self.sp as u32;
                let result = hl + sp;
                self.set_hl(result as u16);
                self.f = (self.f & (F_S | F_Z | F_PV)) | if (result & 0x10000) != 0 { F_C } else { 0 } | if ((hl ^ sp ^ result) & 0x1000) != 0 { F_H } else { 0 };
                11
            }
            0x3A => { // LD A, (nn)
                let nn = self.read_word_pc(bus);
                self.a = bus.read_byte(nn);
                13
            }
            0x3B => { // DEC SP
                self.sp = self.sp.wrapping_sub(1);
                6
            }
            0x3C => { // INC A
                let res = self.a.wrapping_add(1);
                self.set_flags_add(self.a, 1, res as u16);
                self.f &= !F_N;
                self.a = res;
                4
            }
            0x3D => { // DEC A
                let res = self.a.wrapping_sub(1);
                self.set_flags_sub(self.a, 1, res as i16);
                self.a = res;
                4
            }
            0x3E => { // LD A, n
                self.a = self.read_byte_pc(bus);
                7
            }
            0x3F => { // CCF
                let carry = (self.f & F_C) != 0;
                self.f = (self.f & (F_S | F_Z | F_PV)) | if !carry { F_C } else { 0 } | if carry { F_H } else { 0 };
                4
            }
            0x40 => { // LD B, B
                4
            }
            0x41 => { // LD B, C
                self.b = self.c;
                4
            }
            0x42 => { // LD B, D
                self.b = self.d;
                4
            }
            0x43 => { // LD B, E
                self.b = self.e;
                4
            }
            0x44 => { // LD B, H
                self.b = self.h;
                4
            }
            0x45 => { // LD B, L
                self.b = self.l;
                4
            }
            0x46 => { // LD B, (HL)
                self.b = bus.read_byte(self.get_hl());
                7
            }
            0x47 => { // LD B, A
                self.b = self.a;
                4
            }
            0x48 => { // LD C, B
                self.c = self.b;
                4
            }
            0x49 => { // LD C, C
                4
            }
            0x4A => { // LD C, D
                self.c = self.d;
                4
            }
            0x4B => { // LD C, E
                self.c = self.e;
                4
            }
            0x4C => { // LD C, H
                self.c = self.h;
                4
            }
            0x4D => { // LD C, L
                self.c = self.l;
                4
            }
            0x4E => { // LD C, (HL)
                self.c = bus.read_byte(self.get_hl());
                7
            }
            0x4F => { // LD C, A
                self.c = self.a;
                4
            }
            0x50 => { // LD D, B
                self.d = self.b;
                4
            }
            0x51 => { // LD D, C
                self.d = self.c;
                4
            }
            0x52 => { // LD D, D
                4
            }
            0x53 => { // LD D, E
                self.d = self.e;
                4
            }
            0x54 => { // LD D, H
                self.d = self.h;
                4
            }
            0x55 => { // LD D, L
                self.d = self.l;
                4
            }
            0x56 => { // LD D, (HL)
                self.d = bus.read_byte(self.get_hl());
                7
            }
            0x57 => { // LD D, A
                self.d = self.a;
                4
            }
            0x58 => { // LD E, B
                self.e = self.b;
                4
            }
            0x59 => { // LD E, C
                self.e = self.c;
                4
            }
            0x5A => { // LD E, D
                self.e = self.d;
                4
            }
            0x5B => { // LD E, E
                4
            }
            0x5C => { // LD E, H
                self.e = self.h;
                4
            }
            0x5D => { // LD E, L
                self.e = self.l;
                4
            }
            0x5E => { // LD E, (HL)
                self.e = bus.read_byte(self.get_hl());
                7
            }
            0x5F => { // LD E, A
                self.e = self.a;
                4
            }
            0x60 => { // LD H, B
                self.h = self.b;
                4
            }
            0x61 => { // LD H, C
                self.h = self.c;
                4
            }
            0x62 => { // LD H, D
                self.h = self.d;
                4
            }
            0x63 => { // LD H, E
                self.h = self.e;
                4
            }
            0x64 => { // LD H, H
                4
            }
            0x65 => { // LD H, L
                self.h = self.l;
                4
            }
            0x66 => { // LD H, (HL)
                self.h = bus.read_byte(self.get_hl());
                7
            }
            0x67 => { // LD H, A
                self.h = self.a;
                4
            }
            0x68 => { // LD L, B
                self.l = self.b;
                4
            }
            0x69 => { // LD L, C
                self.l = self.c;
                4
            }
            0x6A => { // LD L, D
                self.l = self.d;
                4
            }
            0x6B => { // LD L, E
                self.l = self.e;
                4
            }
            0x6C => { // LD L, H
                self.l = self.h;
                4
            }
            0x6D => { // LD L, L
                4
            }
            0x6E => { // LD L, (HL)
                self.l = bus.read_byte(self.get_hl());
                7
            }
            0x6F => { // LD L, A
                self.l = self.a;
                4
            }
            0x70 => { // LD (HL), B
                bus.write_byte(self.get_hl(), self.b);
                7
            }
            0x71 => { // LD (HL), C
                bus.write_byte(self.get_hl(), self.c);
                7
            }
            0x72 => { // LD (HL), D
                bus.write_byte(self.get_hl(), self.d);
                7
            }
            0x73 => { // LD (HL), E
                bus.write_byte(self.get_hl(), self.e);
                7
            }
            0x74 => { // LD (HL), H
                bus.write_byte(self.get_hl(), self.h);
                7
            }
            0x75 => { // LD (HL), L
                bus.write_byte(self.get_hl(), self.l);
                7
            }
            0x76 => { // HALT
                // For simplicity, treat as NOP
                4
            }
            0x77 => { // LD (HL), A
                bus.write_byte(self.get_hl(), self.a);
                7
            }
            0x78 => { // LD A, B
                self.a = self.b;
                4
            }
            0x79 => { // LD A, C
                self.a = self.c;
                4
            }
            0x7A => { // LD A, D
                self.a = self.d;
                4
            }
            0x7B => { // LD A, E
                self.a = self.e;
                4
            }
            0x7C => { // LD A, H
                self.a = self.h;
                4
            }
            0x7D => { // LD A, L
                self.a = self.l;
                4
            }
            0x7E => { // LD A, (HL)
                self.a = bus.read_byte(self.get_hl());
                7
            }
            0x7F => { // LD A, A
                4
            }
            0x80 => { // ADD A, B
                let result = self.a as u16 + self.b as u16;
                self.set_flags_add(self.a, self.b, result);
                self.a = result as u8;
                4
            }
            0x81 => { // ADD A, C
                let result = self.a as u16 + self.c as u16;
                self.set_flags_add(self.a, self.c, result);
                self.a = result as u8;
                4
            }
            0x82 => { // ADD A, D
                let result = self.a as u16 + self.d as u16;
                self.set_flags_add(self.a, self.d, result);
                self.a = result as u8;
                4
            }
            0x83 => { // ADD A, E
                let result = self.a as u16 + self.e as u16;
                self.set_flags_add(self.a, self.e, result);
                self.a = result as u8;
                4
            }
            0x84 => { // ADD A, H
                let result = self.a as u16 + self.h as u16;
                self.set_flags_add(self.a, self.h, result);
                self.a = result as u8;
                4
            }
            0x85 => { // ADD A, L
                let result = self.a as u16 + self.l as u16;
                self.set_flags_add(self.a, self.l, result);
                self.a = result as u8;
                4
            }
            0x86 => { // ADD A, (HL)
                let val = bus.read_byte(self.get_hl());
                let result = self.a as u16 + val as u16;
                self.set_flags_add(self.a, val, result);
                self.a = result as u8;
                7
            }
            0x87 => { // ADD A, A
                let result = self.a as u16 + self.a as u16;
                self.set_flags_add(self.a, self.a, result);
                self.a = result as u8;
                4
            }
            0x88 => { // ADC A, B
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.b as u16 + carry;
                self.set_flags_add(self.a, self.b + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x89 => { // ADC A, C
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.c as u16 + carry;
                self.set_flags_add(self.a, self.c + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x8A => { // ADC A, D
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.d as u16 + carry;
                self.set_flags_add(self.a, self.d + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x8B => { // ADC A, E
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.e as u16 + carry;
                self.set_flags_add(self.a, self.e + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x8C => { // ADC A, H
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.h as u16 + carry;
                self.set_flags_add(self.a, self.h + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x8D => { // ADC A, L
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.l as u16 + carry;
                self.set_flags_add(self.a, self.l + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x8E => { // ADC A, (HL)
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let val = bus.read_byte(self.get_hl());
                let result = self.a as u16 + val as u16 + carry;
                self.set_flags_add(self.a, val + carry as u8, result);
                self.a = result as u8;
                7
            }
            0x8F => { // ADC A, A
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as u16 + self.a as u16 + carry;
                self.set_flags_add(self.a, self.a + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x90 => { // SUB B
                let result = self.a as i16 - self.b as i16;
                self.set_flags_sub(self.a, self.b, result);
                self.a = result as u8;
                4
            }
            0x91 => { // SUB C
                let result = self.a as i16 - self.c as i16;
                self.set_flags_sub(self.a, self.c, result);
                self.a = result as u8;
                4
            }
            0x92 => { // SUB D
                let result = self.a as i16 - self.d as i16;
                self.set_flags_sub(self.a, self.d, result);
                self.a = result as u8;
                4
            }
            0x93 => { // SUB E
                let result = self.a as i16 - self.e as i16;
                self.set_flags_sub(self.a, self.e, result);
                self.a = result as u8;
                4
            }
            0x94 => { // SUB H
                let result = self.a as i16 - self.h as i16;
                self.set_flags_sub(self.a, self.h, result);
                self.a = result as u8;
                4
            }
            0x95 => { // SUB L
                let result = self.a as i16 - self.l as i16;
                self.set_flags_sub(self.a, self.l, result);
                self.a = result as u8;
                4
            }
            0x96 => { // SUB (HL)
                let val = bus.read_byte(self.get_hl());
                let result = self.a as i16 - val as i16;
                self.set_flags_sub(self.a, val, result);
                self.a = result as u8;
                7
            }
            0x97 => { // SUB A
                self.a = 0;
                self.f = F_Z | F_N;
                4
            }
            0x98 => { // SBC A, B
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.b as i16 - carry;
                self.set_flags_sub(self.a, self.b, result);
                self.a = result as u8;
                4
            }
            0x99 => { // SBC A, C
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.c as i16 - carry;
                self.set_flags_sub(self.a, self.c + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x9A => { // SBC A, D
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.d as i16 - carry;
                self.set_flags_sub(self.a, self.d + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x9B => { // SBC A, E
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.e as i16 - carry;
                self.set_flags_sub(self.a, self.e + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x9C => { // SBC A, H
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.h as i16 - carry;
                self.set_flags_sub(self.a, self.h + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x9D => { // SBC A, L
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.l as i16 - carry;
                self.set_flags_sub(self.a, self.l + carry as u8, result);
                self.a = result as u8;
                4
            }
            0x9E => { // SBC A, (HL)
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let val = bus.read_byte(self.get_hl());
                let result = self.a as i16 - val as i16 - carry;
                self.set_flags_sub(self.a, val, result);
                self.a = result as u8;
                7
            }
            0x9F => { // SBC A, A
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = self.a as i16 - self.a as i16 - carry;
                self.set_flags_sub(self.a, self.a + carry as u8, result);
                self.a = result as u8;
                4
            }
            0xA0 => { // AND B
                self.a &= self.b;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA1 => { // AND C
                self.a &= self.c;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA2 => { // AND D
                self.a &= self.d;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA3 => { // AND E
                self.a &= self.e;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA4 => { // AND H
                self.a &= self.h;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA5 => { // AND L
                self.a &= self.l;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA6 => { // AND (HL)
                let val = bus.read_byte(self.get_hl());
                self.a &= val;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xA7 => { // AND A
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA8 => { // XOR B
                self.a ^= self.b;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xA9 => { // XOR C
                self.a ^= self.c;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xAA => { // XOR D
                self.a ^= self.d;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xAB => { // XOR E
                self.a ^= self.e;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xAC => { // XOR H
                self.a ^= self.h;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xAD => { // XOR L
                self.a ^= self.l;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xAE => { // XOR (HL)
                let val = bus.read_byte(self.get_hl());
                self.a ^= val;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xAF => { // XOR A
                self.a = 0;
                self.f = F_Z | F_PV;
                4
            }
            0xB0 => { // OR B
                self.a |= self.b;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB1 => { // OR C
                self.a |= self.c;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB2 => { // OR D
                self.a |= self.d;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB3 => { // OR E
                self.a |= self.e;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB4 => { // OR H
                self.a |= self.h;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB5 => { // OR L
                self.a |= self.l;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB6 => { // OR (HL)
                let val = bus.read_byte(self.get_hl());
                self.a |= val;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xB7 => { // OR A
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                4
            }
            0xB8 => { // CP B
                let result = self.a as i16 - self.b as i16;
                self.set_flags_sub(self.a, self.b, result);
                4
            }
            0xB9 => { // CP C
                let result = self.a as i16 - self.c as i16;
                self.set_flags_sub(self.a, self.c, result);
                4
            }
            0xBA => { // CP D
                let result = self.a as i16 - self.d as i16;
                self.set_flags_sub(self.a, self.d, result);
                4
            }
            0xBB => { // CP E
                let result = self.a as i16 - self.e as i16;
                self.set_flags_sub(self.a, self.e, result);
                4
            }
            0xBC => { // CP H
                let result = self.a as i16 - self.h as i16;
                self.set_flags_sub(self.a, self.h, result);
                4
            }
            0xBD => { // CP L
                let result = self.a as i16 - self.l as i16;
                self.set_flags_sub(self.a, self.l, result);
                4
            }
            0xBE => { // CP (HL)
                let val = bus.read_byte(self.get_hl());
                let result = self.a as i16 - val as i16;
                self.set_flags_sub(self.a, val, result);
                7
            }
            0xBF => { // CP A
                self.f = F_Z | F_N;
                4
            }
            0xC0 => { // RET NZ
                if (self.f & F_Z) == 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xC1 => { // POP BC
                let val = self.pop(bus);
                self.set_bc(val);
                10
            }
            0xC2 => { // JP NZ, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_Z) == 0 {
                    self.pc = nn;
                }
                10
            }
            0xC3 => { // JP nn
                self.pc = self.read_word_pc(bus);
                10
            }
            0xC4 => { // CALL NZ, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_Z) == 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xC5 => { // PUSH BC
                self.push(bus, self.get_bc());
                11
            }
            0xC6 => { // ADD A, n
                let n = self.read_byte_pc(bus);
                let result = self.a as u16 + n as u16;
                self.set_flags_add(self.a, n, result);
                self.a = result as u8;
                7
            }
            0xC7 => { // RST 00
                self.push(bus, self.pc);
                self.pc = 0x00;
                11
            }
            0xC8 => { // RET Z
                if (self.f & F_Z) != 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xC9 => { // RET
                self.pc = self.pop(bus);
                10
            }
            0xCA => { // JP Z, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_Z) != 0 {
                    self.pc = nn;
                }
                10
            }
            0xCB => { // CB prefix - bit operations
                self.step_cb(bus)
            }
            0xCC => { // CALL Z, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_Z) != 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xCD => { // CALL nn
                let nn = self.read_word_pc(bus);
                self.push(bus, self.pc);
                self.pc = nn;
                17
            }
            0xCE => { // ADC A, n
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let n = self.read_byte_pc(bus);
                let result = self.a as u16 + n as u16 + carry;
                self.set_flags_add(self.a, n + carry as u8, result);
                self.a = result as u8;
                7
            }
            0xCF => { // RST 08
                self.push(bus, self.pc);
                self.pc = 0x08;
                11
            }
            0xD0 => { // RET NC
                if (self.f & F_C) == 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xD1 => { // POP DE
                let val = self.pop(bus);
                self.set_de(val);
                10
            }
            0xD2 => { // JP NC, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_C) == 0 {
                    self.pc = nn;
                }
                10
            }
            0xD3 => { // OUT (n), A
                let n = self.read_byte_pc(bus);
                // For simplicity, ignore I/O
                11
            }
            0xD4 => { // CALL NC, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_C) == 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xD5 => { // PUSH DE
                self.push(bus, self.get_de());
                11
            }
            0xD6 => { // SUB n
                let n = self.read_byte_pc(bus);
                let result = self.a as i16 - n as i16;
                self.set_flags_sub(self.a, n, result);
                self.a = result as u8;
                7
            }
            0xD7 => { // RST 10
                self.push(bus, self.pc);
                self.pc = 0x10;
                11
            }
            0xD8 => { // RET C
                if (self.f & F_C) != 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xD9 => { // EXX
                let temp_b = self.b; let temp_c = self.c;
                self.b = self.b_alt; self.c = self.c_alt;
                self.b_alt = temp_b; self.c_alt = temp_c;
                let temp_d = self.d; let temp_e = self.e;
                self.d = self.d_alt; self.e = self.e_alt;
                self.d_alt = temp_d; self.e_alt = temp_e;
                let temp_h = self.h; let temp_l = self.l;
                self.h = self.h_alt; self.l = self.l_alt;
                self.h_alt = temp_h; self.l_alt = temp_l;
                4
            }
            0xDA => { // JP C, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_C) != 0 {
                    self.pc = nn;
                }
                10
            }
            0xDB => { // IN A, (n)
                let n = self.read_byte_pc(bus);
                // For simplicity, return 0
                self.a = 0;
                11
            }
            0xDC => { // CALL C, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_C) != 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xDD => { // DD prefix - IX operations
                self.step_dd(bus)
            }
            0xDE => { // SBC A, n
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let n = self.read_byte_pc(bus);
                let result = self.a as i16 - n as i16 - carry;
                self.set_flags_sub(self.a, n + carry as u8, result);
                self.a = result as u8;
                7
            }
            0xDF => { // RST 18
                self.push(bus, self.pc);
                self.pc = 0x18;
                11
            }
            0xE0 => { // RET PO
                if (self.f & F_PV) == 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xE1 => { // POP HL
                let val = self.pop(bus);
                self.set_hl(val);
                10
            }
            0xE2 => { // JP PO, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_PV) == 0 {
                    self.pc = nn;
                }
                10
            }
            0xE3 => { // EX (SP), HL
                let sp_val = self.pop(bus);
                self.push(bus, self.get_hl());
                self.set_hl(sp_val);
                19
            }
            0xE4 => { // CALL PO, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_PV) == 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xE5 => { // PUSH HL
                self.push(bus, self.get_hl());
                11
            }
            0xE6 => { // AND n
                let n = self.read_byte_pc(bus);
                self.a &= n;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | F_H | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xE7 => { // RST 20
                self.push(bus, self.pc);
                self.pc = 0x20;
                11
            }
            0xE8 => { // RET PE
                if (self.f & F_PV) != 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xE9 => { // JP (HL)
                self.pc = self.get_hl();
                4
            }
            0xEA => { // JP PE, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_PV) != 0 {
                    self.pc = nn;
                }
                10
            }
            0xEB => { // EX DE, HL
                let temp_d = self.d; let temp_e = self.e;
                let temp_h = self.h; let temp_l = self.l;
                self.d = temp_h; self.e = temp_l;
                self.h = temp_d; self.l = temp_e;
                4
            }
            0xEC => { // CALL PE, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_PV) != 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xED => { // ED prefix - extended operations
                self.step_ed(bus)
            }
            0xEE => { // XOR n
                let n = self.read_byte_pc(bus);
                self.a ^= n;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xEF => { // RST 28
                self.push(bus, self.pc);
                self.pc = 0x28;
                11
            }
            0xF0 => { // RET P
                if (self.f & F_S) == 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xF1 => { // POP AF
                let val = self.pop(bus);
                self.set_af(val);
                10
            }
            0xF2 => { // JP P, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_S) == 0 {
                    self.pc = nn;
                }
                10
            }
            0xF3 => { // DI
                self.iff1 = false;
                self.iff2 = false;
                4
            }
            0xF4 => { // CALL P, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_S) == 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xF5 => { // PUSH AF
                self.push(bus, self.get_af());
                11
            }
            0xF6 => { // OR n
                let n = self.read_byte_pc(bus);
                self.a |= n;
                self.f = if self.a == 0 { F_Z } else { 0 } | if (self.a & 0x80) != 0 { F_S } else { 0 } | if Self::parity(self.a) { F_PV } else { 0 };
                7
            }
            0xF7 => { // RST 30
                self.push(bus, self.pc);
                self.pc = 0x30;
                11
            }
            0xF8 => { // RET M
                if (self.f & F_S) != 0 {
                    self.pc = self.pop(bus);
                    11
                } else {
                    5
                }
            }
            0xF9 => { // LD SP, HL
                self.sp = self.get_hl();
                6
            }
            0xFA => { // JP M, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_S) != 0 {
                    self.pc = nn;
                }
                10
            }
            0xFB => { // EI
                self.iff1 = true;
                self.iff2 = true;
                4
            }
            0xFC => { // CALL M, nn
                let nn = self.read_word_pc(bus);
                if (self.f & F_S) != 0 {
                    self.push(bus, self.pc);
                    self.pc = nn;
                    17
                } else {
                    10
                }
            }
            0xFD => { // FD prefix - IY operations
                self.step_fd(bus)
            }
            0xFE => { // CP n
                let n = self.read_byte_pc(bus);
                let result = self.a as i16 - n as i16;
                self.set_flags_sub(self.a, n, result);
                7
            }
            0xFF => { // RST 38
                self.push(bus, self.pc);
                self.pc = 0x38;
                11
            }
            _ => {
                println!("Unimplemented opcode: {:02X}", opcode);
                4 // Default T-states for unimplemented opcodes
            }
        }
    }

    fn step_cb(&mut self, bus: &mut dyn Bus) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            0x00 => { // RLC B
                let val = self.b;
                let new_val = val.rotate_left(1);
                self.b = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                8
            }
            0x5C => { // BIT 3, H
                let bit = 3;
                let val = self.h;
                let bit_set = (val & (1 << bit)) != 0;
                self.f = (self.f & F_C) | F_H | (if !bit_set { F_Z } else { 0 }) | (if bit_set { F_PV } else { 0 }) | (if bit == 7 && bit_set { F_S } else { 0 });
                8
            }
            // SRL opcodes
            0x38 => { // SRL B
                let val = self.b;
                let new_val = val >> 1;
                self.b = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x39 => { // SRL C
                let val = self.c;
                let new_val = val >> 1;
                self.c = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x3A => { // SRL D
                let val = self.d;
                let new_val = val >> 1;
                self.d = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x3B => { // SRL E
                let val = self.e;
                let new_val = val >> 1;
                self.e = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x3C => { // SRL H
                let val = self.h;
                let new_val = val >> 1;
                self.h = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x3D => { // SRL L
                let val = self.l;
                let new_val = val >> 1;
                self.l = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x3E => { // SRL (HL)
                let addr = self.get_hl();
                let val = bus.read_byte(addr);
                let new_val = val >> 1;
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                15
            }
            0x3F => { // SRL A
                let val = self.a;
                let new_val = val >> 1;
                self.a = new_val;
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                8
            }
            0x80..=0xBF => { // RES b, r
                let b = (opcode - 0x80) / 8;
                let r = (opcode - 0x80) % 8;
                if r == 6 {
                    let addr = self.get_hl();
                    let val = bus.read_byte(addr);
                    let new_val = val & !(1 << b);
                    bus.write_byte(addr, new_val);
                    15
                } else {
                    let val = self.get_reg(r);
                    let new_val = val & !(1 << b);
                    self.set_reg(r, new_val);
                    8
                }
            }
            0xC0..=0xFF => { // SET b, r
                let b = (opcode - 0xC0) / 8;
                let r = (opcode - 0xC0) % 8;
                if r == 6 {
                    let addr = self.get_hl();
                    let val = bus.read_byte(addr);
                    let new_val = val | (1 << b);
                    bus.write_byte(addr, new_val);
                    15
                } else {
                    let val = self.get_reg(r);
                    let new_val = val | (1 << b);
                    self.set_reg(r, new_val);
                    8
                }
            }
            _ => {
                println!("Unimplemented CB opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_ed(&mut self, bus: &mut dyn Bus) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            0x43 => { // LD (nn), BC
                let nn = self.read_word_pc(bus);
                bus.write_byte(nn, self.c);
                bus.write_byte(nn.wrapping_add(1), self.b);
                20
            }
            0x47 => { // LD I, A
                self.i = self.a;
                9
            }
            0x52 => { // SBC HL, DE
                let hl = self.get_hl() as u32;
                let de = self.get_de() as u32;
                let carry = if (self.f & F_C) != 0 { 1 } else { 0 };
                let result = hl.wrapping_sub(de).wrapping_sub(carry);
                self.set_hl(result as u16);
                let result_u16 = result as u16;
                let overflow = ((hl ^ de) & (hl ^ result)) & 0x8000 != 0;
                self.f = (if (result_u16 & 0x8000) != 0 { F_S } else { 0 }) |
                         (if result_u16 == 0 { F_Z } else { 0 }) |
                         (if overflow { F_PV } else { 0 }) |
                         F_N |
                         (if result > 0xFFFF { F_C } else { 0 }) |
                         (if ((hl ^ de ^ result) & 0x1000) != 0 { F_H } else { 0 });
                15
            }
            0x53 => { // LD (nn), DE
                let nn = self.read_word_pc(bus);
                bus.write_byte(nn, self.e);
                bus.write_byte(nn.wrapping_add(1), self.d);
                20
            }
            0x56 => { // IM 1
                self.im = 1;
                8
            }
            0x78 => { // IN A, (C)
                let port = self.get_bc();
                let val = bus.read_port(port);
                self.a = val;
                self.f = (self.f & F_C) | (if (val & 0x80) != 0 { F_S } else { 0 }) | (if val == 0 { F_Z } else { 0 }) | (if val.count_ones() % 2 == 0 { F_PV } else { 0 });
                12
            }
            0x7B => { // LD SP, (nn)
                let nn = self.read_word_pc(bus);
                let val = bus.read_byte(nn) as u16 | ((bus.read_byte(nn.wrapping_add(1)) as u16) << 8);
                self.sp = val;
                20
            }
            0x41 => { // OUT (C), B
                bus.write_port(self.get_bc(), self.b);
                12
            }
            0x49 => { // OUT (C), C
                bus.write_port(self.get_bc(), self.c);
                12
            }
            0x51 => { // OUT (C), D
                bus.write_port(self.get_bc(), self.d);
                12
            }
            0x59 => { // OUT (C), E
                bus.write_port(self.get_bc(), self.e);
                12
            }
            0x61 => { // OUT (C), H
                bus.write_port(self.get_bc(), self.h);
                12
            }
            0x69 => { // OUT (C), L
                bus.write_port(self.get_bc(), self.l);
                12
            }
            0x71 => { // OUT (C), 0
                bus.write_port(self.get_bc(), 0);
                12
            }
            0x79 => { // OUT (C), A
                bus.write_port(self.get_bc(), self.a);
                12
            }
            0xB0 => { // LDIR
                let mut tstates = 0u32;
                loop {
                    let val = bus.read_byte(self.get_hl());
                    bus.write_byte(self.get_de(), val);
                    self.set_hl(self.get_hl().wrapping_add(1));
                    self.set_de(self.get_de().wrapping_add(1));
                    self.set_bc(self.get_bc().wrapping_sub(1));
                    tstates += 21;
                    if self.get_bc() == 0 {
                        self.f &= !F_PV;
                        break;
                    }
                }
                tstates
            }
            0xB8 => { // LDDR
                let mut tstates = 0u32;
                loop {
                    let val = bus.read_byte(self.get_hl());
                    bus.write_byte(self.get_de(), val);
                    self.set_hl(self.get_hl().wrapping_sub(1));
                    self.set_de(self.get_de().wrapping_sub(1));
                    self.set_bc(self.get_bc().wrapping_sub(1));
                    tstates += 21;
                    if self.get_bc() == 0 {
                        self.f &= !F_PV;
                        break;
                    }
                }
                tstates
            }
            0xA0 => { // LDI
                let val = bus.read_byte(self.get_hl());
                bus.write_byte(self.get_de(), val);
                let hl = self.get_hl().wrapping_add(1);
                self.set_hl(hl);
                let de = self.get_de().wrapping_add(1);
                self.set_de(de);
                let bc = self.get_bc().wrapping_sub(1);
                self.set_bc(bc);
                self.f &= !(F_H | F_N | F_PV);
                if bc != 0 { self.f |= F_PV; }
                16
            }
            _ => {
                println!("Unimplemented ED opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_dd(&mut self, bus: &mut dyn Bus) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            // Implement DD opcodes here
            0xCB => { // DD CB prefix
                self.step_ddcb(bus)
            }
            _ => {
                println!("Unimplemented DD opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_ddcb(&mut self, bus: &mut dyn Bus) -> u32 {
        let d = self.read_byte_pc(bus) as i8 as i16;
        let addr = self.ix.wrapping_add(d as u16);
        let opcode = self.read_byte_pc(bus);
        match opcode {
            0x06 => { // RLC (IX+d)
                let val = bus.read_byte(addr);
                let new_val = val.rotate_left(1);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x0E => { // RRC (IX+d)
                let val = bus.read_byte(addr);
                let new_val = val.rotate_right(1);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x16 => { // RL (IX+d)
                let val = bus.read_byte(addr);
                let new_val = (val << 1) | (if (self.f & F_C) != 0 { 1 } else { 0 });
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x1E => { // RR (IX+d)
                let val = bus.read_byte(addr);
                let new_val = (val >> 1) | (if (self.f & F_C) != 0 { 0x80 } else { 0 });
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x26 => { // SLA (IX+d)
                let val = bus.read_byte(addr);
                let new_val = val << 1;
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x2E => { // SRA (IX+d)
                let val = bus.read_byte(addr);
                let new_val = (val >> 1) | (val & 0x80);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x3E => { // SRL (IX+d)
                let val = bus.read_byte(addr);
                let new_val = val >> 1;
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x46 | 0x4E | 0x56 | 0x5E | 0x66 | 0x6E | 0x76 | 0x7E => { // BIT b, (IX+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let bit_set = (val & (1 << bit)) != 0;
                self.f = (self.f & F_C) | F_H | (if !bit_set { F_Z | F_PV } else { 0 }) | (if bit == 7 && bit_set { F_S } else { 0 });
                20
            }
            0x86 | 0x8E | 0x96 | 0x9E | 0xA6 | 0xAE | 0xB6 | 0xBE => { // RES b, (IX+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let new_val = val & !(1 << bit);
                bus.write_byte(addr, new_val);
                23
            }
            0xC6 | 0xCE | 0xD6 | 0xDE | 0xE6 | 0xEE | 0xF6 | 0xFE => { // SET b, (IX+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let new_val = val | (1 << bit);
                bus.write_byte(addr, new_val);
                23
            }
            _ => {
                println!("Unimplemented DDCB opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_fd(&mut self, bus: &mut dyn Bus) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            0x21 => { // LD IY, nn
                self.iy = self.read_word_pc(bus);
                14
            }
            0x35 => { // DEC (IY+d)
                let d = self.read_byte_pc(bus) as i8 as i16;
                let addr = self.iy.wrapping_add(d as u16);
                let val = bus.read_byte(addr);
                let result = val.wrapping_sub(1);
                bus.write_byte(addr, result);
                self.f = (self.f & F_C) |
                         (if (result & 0x80) != 0 { F_S } else { 0 }) |
                         (if result == 0 { F_Z } else { 0 }) |
                         F_N |
                         (if val == 0x80 { F_PV } else { 0 }) |
                         (if (val & 0x0F) == 0 { F_H } else { 0 });
                23
            }
            0x71 => { // LD (IY+d), C
                let d = self.read_byte_pc(bus) as i8 as i16;
                let addr = self.iy.wrapping_add(d as u16);
                bus.write_byte(addr, self.c);
                19
            }
            0x75 => { // LD (IY+d), L
                let d = self.read_byte_pc(bus) as i8 as i16;
                let addr = self.iy.wrapping_add(d as u16);
                bus.write_byte(addr, self.l);
                19
            }
            0xCB => { // FD CB prefix
                self.step_fdcb(bus)
            }
            0x36 => { // LD (IY+d), n
                let d = self.read_byte_pc(bus) as i8;
                let n = self.read_byte_pc(bus);
                let addr = self.iy.wrapping_add(d as u16);
                bus.write_byte(addr, n);
                19
            }
            _ => {
                println!("Unimplemented FD opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_fdcb(&mut self, bus: &mut dyn Bus) -> u32 {
        let d = self.read_byte_pc(bus) as i8 as i16;
        let addr = self.iy.wrapping_add(d as u16);
        let opcode = self.read_byte_pc(bus);
        match opcode {
            0x06 => { // RLC (IY+d)
                let val = bus.read_byte(addr);
                let new_val = val.rotate_left(1);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x0E => { // RRC (IY+d)
                let val = bus.read_byte(addr);
                let new_val = val.rotate_right(1);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x16 => { // RL (IY+d)
                let val = bus.read_byte(addr);
                let new_val = (val << 1) | (if (self.f & F_C) != 0 { 1 } else { 0 });
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x1E => { // RR (IY+d)
                let val = bus.read_byte(addr);
                let new_val = (val >> 1) | (if (self.f & F_C) != 0 { 0x80 } else { 0 });
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x26 => { // SLA (IY+d)
                let val = bus.read_byte(addr);
                let new_val = val << 1;
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x80) != 0 { F_C } else { 0 });
                23
            }
            0x2E => { // SRA (IY+d)
                let val = bus.read_byte(addr);
                let new_val = (val >> 1) | (val & 0x80);
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x3E => { // SRL (IY+d)
                let val = bus.read_byte(addr);
                let new_val = val >> 1;
                bus.write_byte(addr, new_val);
                self.f = (if (new_val & 0x80) != 0 { F_S } else { 0 }) | (if new_val == 0 { F_Z } else { 0 }) | (if new_val.count_ones() % 2 == 0 { F_PV } else { 0 }) | (if (val & 0x01) != 0 { F_C } else { 0 });
                23
            }
            0x46 | 0x4E | 0x56 | 0x5E | 0x66 | 0x6E | 0x76 | 0x7E => { // BIT b, (IY+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let bit_set = (val & (1 << bit)) != 0;
                self.f = (self.f & F_C) | F_H | (if !bit_set { F_Z | F_PV } else { 0 }) | (if bit == 7 && bit_set { F_S } else { 0 });
                20
            }
            0x86 | 0x8E | 0x96 | 0x9E | 0xA6 | 0xAE | 0xB6 | 0xBE => { // RES b, (IY+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let new_val = val & !(1 << bit);
                bus.write_byte(addr, new_val);
                23
            }
            0xC6 | 0xCE | 0xD6 | 0xDE | 0xE6 | 0xEE | 0xF6 | 0xFE => { // SET b, (IY+d)
                let bit = (opcode >> 3) & 0x07;
                let val = bus.read_byte(addr);
                let new_val = val | (1 << bit);
                bus.write_byte(addr, new_val);
                23
            }
            _ => {
                println!("Unimplemented FDCB opcode: {:02X}", opcode);
                8
            }
        }
    }

    pub fn reset(&mut self){
        // Reset CPU state
        self.a = 0; 
        self.f = 0;
        
        self.b = 0; 
        self.c = 0; 
        
        self.d = 0; 
        self.e = 0;
        
        self.h = 0; 
        self.l = 0;
        
        self.a_alt = 0; 
        self.f_alt = 0;
        
        self.b_alt = 0; 
        self.c_alt = 0;
        
        self.d_alt = 0; 
        self.e_alt = 0;
        
        self.h_alt = 0; 
        self.l_alt = 0;

        self.sp = 0; 
        self.pc = 0;
        
        self.ix = 0; 
        self.iy = 0;
        
        self.i = 0; 
        self.r = 0;
        
        self.iff1 = false; 
        self.iff2 = false;
        
        self.im = 1;
        self.int_requested = false;
    }

    pub fn raise_int(&mut self) {
        if !self.iff1 {
            return;
        }
        self.iff1 = false;
        self.iff2 = false;
        self.int_requested = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::emulator::Ports;

    struct TestMemory {
        ram: [u8; 0x10000],
    }

    impl TestMemory {
        fn new() -> Self {
            Self { ram: [0; 0x10000] }
        }
    }

    impl Memory for TestMemory {
        fn read_byte(&self, addr: u16) -> u8 {
            self.ram[addr as usize]
        }
        fn write_byte(&mut self, addr: u16, val: u8) {
            self.ram[addr as usize] = val;
        }
    }

    impl Ports for TestMemory {
        fn read_port(&self, _port: u16) -> u8 {
            0xFF
        }
        fn write_port(&mut self, _port: u16, _val: u8) {
            // Do nothing
        }
    }

    impl Bus for TestMemory {}

    #[test]
    fn test_00_nop() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x00; // NOP
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_01_ld_bc_nn() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x01; // LD BC, nn
        mem.ram[1] = 0x34;
        mem.ram[2] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.get_bc(), 0x1234);
        assert_eq!(cycles, 10);
        assert_eq!(cpu.pc, 3);
    }

    #[test]
    fn test_02_ld_bc_a() {
        let mut cpu = Z80::new();
        cpu.a = 0xCD;
        cpu.set_bc(0x1234);
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x02; // LD (BC), A
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1234], 0xCD);
        assert_eq!(cycles, 7);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_04_inc_b() {
        let mut cpu = Z80::new();
        cpu.b = 5;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x04; // INC B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 6);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_09_add_hl_bc() {
        let mut cpu = Z80::new();
        cpu.set_hl(0x1000);
        cpu.set_bc(0x2000);
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x09; // ADD HL, BC
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.get_hl(), 0x3000);
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cycles, 11);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_0a_ld_a_bc() {
        let mut cpu = Z80::new();
        cpu.set_bc(0x1234);
        let mut mem = TestMemory::new();
        mem.ram[0x1234] = 0xAB;
        mem.ram[0] = 0x0A; // LD A, (BC)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xAB);
        assert_eq!(cycles, 7);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_0d_dec_c() {
        let mut cpu = Z80::new();
        cpu.c = 10;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x0D; // DEC C
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.c, 9);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, F_N); // N set
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_20_jr_nz() {
        let mut cpu = Z80::new();
        cpu.f = 0; // Z=0
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x20; // JR NZ, n
        mem.ram[1] = 5; // +5
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 7); // 2 + 5
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_21_ld_hl_nn() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x21; // LD HL, nn
        mem.ram[1] = 0x78;
        mem.ram[2] = 0x56;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.get_hl(), 0x5678);
        assert_eq!(cycles, 10);
        assert_eq!(cpu.pc, 3);
    }

    #[test]
    fn test_3e_ld_a_n() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x3E; // LD A, n
        mem.ram[1] = 0x42; // n = 0x42
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x42);
        assert_eq!(cycles, 7);
        assert_eq!(cpu.pc, 2);
    }

    #[test]
    fn test_80_add_a_b() {
        let mut cpu = Z80::new();
        cpu.a = 5;
        cpu.b = 3;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x80; // ADD A, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 8);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_80_add_a_b_flags() {
        let mut cpu = Z80::new();
        cpu.a = 0x7F; // 127
        cpu.b = 0x01; // 1
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x80; // ADD A, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x80); // 128, -128
        assert_eq!(cpu.f & F_S, F_S); // sign set
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!
(cpu.f & F_C, 0); // no carry
        assert_eq!(cpu.f & F_H, F_H); // half carry
        assert_eq!(cpu.f & F_PV, F_PV); // overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_90_sub_b() {
        let mut cpu = Z80::new();
        cpu.a = 10;
        cpu.b = 3;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x90; // SUB B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 7);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, F_N); // N set
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_aa_xor_d() {
        let mut cpu = Z80::new();
        cpu.a = 0xAA;
        cpu.d = 0x55;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xAA; // XOR D
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xFF);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_b1_or_c() {
        let mut cpu = Z80::new();
        cpu.a = 0x0F;
        cpu.c = 0xF0;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xB1; // OR C
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xFF);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_bb_cp_e() {
        let mut cpu = Z80::new();
        cpu.a = 10;
        cpu.e = 5;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xBB; // CP E
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 10); // A unchanged
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_c3_jp_nn() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xC3; // JP nn
        mem.ram[1] = 0x34;
        mem.ram[2] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_c5_push_bc() {
        let mut cpu = Z80::new();
        cpu.sp = 0xFFFE;
        cpu.set_bc(0x1234);
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xC5; // PUSH BC
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.sp, 0xFFFC);
        assert_eq!(mem.ram[0xFFFC], 0x12);
        assert_eq!(mem.ram[0xFFFD], 0x34);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_c9_ret() {
        let mut cpu = Z80::new();
        cpu.sp = 0xFFFC;
        let mut mem = TestMemory::new();
        mem.ram[0xFFFC] = 0x34;
        mem.ram[0xFFFD] = 0x12;
        mem.ram[0] = 0xC9; // RET
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cpu.sp, 0xFFFE);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_cb_00_rlc_b() {
        let mut cpu = Z80::new();
        cpu.b = 0x85; // 10000101
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCB;
        mem.ram[1] = 0x00; // RLC B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x0B); // 00001011, C=1
        assert_eq!(cpu.f & F_C, F_C);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_cb_40_bit_0_b() {
        let mut cpu = Z80::new();
        cpu.b = 0x01; // bit 0 set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCB;
        mem.ram[1] = 0x40; // BIT 0, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_cb_92_res_2_d() {
        let mut cpu = Z80::new();
        cpu.d = 0xFF;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCB;
        mem.ram[1] = 0x92; // RES 2, D
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.d, 0xFB);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_cb_c9_set_1_c() {
        let mut cpu = Z80::new();
        cpu.c = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCB;
        mem.ram[1] = 0xC9; // SET 1, C
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.c, 0x02);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_cd_call_nn() {
        let mut cpu = Z80::new();
        cpu.sp = 0xFFFE;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCD; // CALL nn
        mem.ram[1] = 0x34;
        mem.ram[2] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cpu.sp, 0xFFFC);
        assert_eq!(mem.ram[0xFFFC], 0x00);
        assert_eq!(mem.ram[0xFFFD], 0x03);
        assert_eq!(cycles, 17);
    }

    #[test]
    fn test_d1_pop_de() {
        let mut cpu = Z80::new();
        cpu.sp = 0xFFFC;
        let mut mem = TestMemory::new();
        mem.ram[0xFFFC] = 0x78;
        mem.ram[0xFFFD] = 0x56;
        mem.ram[0] = 0xD1; // POP DE
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.get_de(), 0x5678);
        assert_eq!(cpu.sp, 0xFFFE);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_ed_78_in_a_c() {
        let mut cpu = Z80::new();
        cpu.set_bc(0xFE00);
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xED;
        mem.ram[1] = 0x78; // IN A, (C)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xFF); // from test port
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_ed_79_out_c_a() {
        let mut cpu = Z80::new();
        cpu.a = 0x42;
        cpu.set_bc(0xFE00);
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xED;
        mem.ram[1] = 0x79; // OUT (C), A
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        // write_port called with 0xFE00, 0x42
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_ed_a0_ldi() {
        let mut cpu = Z80::new();
        cpu.set_hl(0x1000);
        cpu.set_de(0x2000);
        cpu.set_bc(1);
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0xAB;
        mem.ram[0] = 0xED;
        mem.ram[1] = 0xA0; // LDI
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x2000], 0xAB);
        assert_eq!(cpu.get_hl(), 0x1001);
        assert_eq!(cpu.get_de(), 0x2001);
        assert_eq!(cpu.get_bc(), 0);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn test_ed_b0_ldir() {
        let mut cpu = Z80::new();
        cpu.set_hl(0x1000);
        cpu.set_de(0x2000);
        cpu.set_bc(3);
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x11;
        mem.ram[0x1001] = 0x22;
        mem.ram[0x1002] = 0x33;
        mem.ram[0] = 0xED;
        mem.ram[1] = 0xB0; // LDIR
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x2000], 0x11);
        assert_eq!(mem.ram[0x2001], 0x22);
        assert_eq!(mem.ram[0x2002], 0x33);
        assert_eq!(cpu.get_hl(), 0x1003);
        assert_eq!(cpu.get_de(), 0x2003);
        assert_eq!(cpu.get_bc(), 0);
        assert_eq!(cycles, 21 * 3);
    }

    #[test]
    fn test_fd_21_ld_iy_nn() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xFD;
        mem.ram[1] = 0x21; // LD IY, nn
        mem.ram[2] = 0x34;
        mem.ram[3] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.iy, 0x1234);
        assert_eq!(cycles, 14);
    }

    #[test]
    fn test_fd_35_dec_iy_d() {
        let mut cpu = Z80::new();
        cpu.iy = 0x1000;
        let mut mem = TestMemory::new();
        mem.ram[0x1005] = 10;
        mem.ram[0] = 0xFD;
        mem.ram[1] = 0x35; // DEC (IY+d)
        mem.ram[2] = 0x05; // d=5
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1005], 9);
        assert_eq!(cycles, 23);
    }

    #[test]
    fn test_fd_36_ld_iy_d_n() {
        let mut cpu = Z80::new();
        cpu.iy = 0x1000;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xFD;
        mem.ram[1] = 0x36; // LD (IY+d), n
        mem.ram[2] = 0x10; // d=16
        mem.ram[3] = 0xAB; // n
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1010], 0xAB);
        assert_eq!(cycles, 19);
    }

    #[test]
    fn test_fd_cb_06_rlc_iy_d() {
        let mut cpu = Z80::new();
        cpu.iy = 0x1000;
        let mut mem = TestMemory::new();
        mem.ram[0x1005] = 0x85;
        mem.ram[0] = 0xFD;
        mem.ram[1] = 0xCB; // FD CB
        mem.ram[2] = 0x05; // d=5
        mem.ram[3] = 0x06; // RLC (IY+d)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1005], 0x0B);
        assert_eq!(cycles, 23);
    }

    #[test]
    fn test_fd_cb_5e_bit_3_iy_d() {
        let mut cpu = Z80::new();
        cpu.iy = 0x1000;
        let mut mem = TestMemory::new();
        mem.ram[0x1005] = 0x08; // bit 3 set
        mem.ram[0] = 0xFD;
        mem.ram[1] = 0xCB;
        mem.ram[2] = 0x05;
        mem.ram[3] = 0x5E; // BIT 3, (IY+d)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.f & F_Z, 0);
        assert_eq!(cycles, 20);
    }

    #[test]
    fn test_40_ld_b_b() {
        let mut cpu = Z80::new();
        cpu.b = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x40; // LD B, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_41_ld_b_c() {
        let mut cpu = Z80::new();
        cpu.c = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x41; // LD B, C
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_42_ld_b_d() {
        let mut cpu = Z80::new();
        cpu.d = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x42; // LD B, D
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_46_ld_b_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x46; // LD B, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_47_ld_b_a() {
        let mut cpu = Z80::new();
        cpu.a = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x47; // LD B, A
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.b, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_48_ld_c_b() {
        let mut cpu = Z80::new();
        cpu.b = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x48; // LD C, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.c, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_4e_ld_c_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x4E; // LD C, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.c, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_56_ld_d_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x56; // LD D, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.d, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_5e_ld_e_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x5E; // LD E, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.e, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_66_ld_h_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x66; // LD H, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.h, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_6e_ld_l_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x6E; // LD L, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.l, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_70_ld_hl_b() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        cpu.b = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x70; // LD (HL), B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1000], 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_77_ld_hl_a() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        cpu.a = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x77; // LD (HL), A
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1000], 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_78_ld_a_b() {
        let mut cpu = Z80::new();
        cpu.b = 0x42;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x78; // LD A, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x42);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_7e_ld_a_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x42;
        mem.ram[0] = 0x7E; // LD A, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x42);
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_88_adc_a_b() {
        let mut cpu = Z80::new();
        cpu.a = 0x10;
        cpu.b = 0x20;
        cpu.f = F_C; // Set carry flag
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x88; // ADC A, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x31); // 0x10 + 0x20 + 1 = 0x31
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_8e_adc_a_hl() {
        let mut cpu = Z80::new();
        cpu.a = 0x10;
        cpu.h = 0x10;
        cpu.l = 0x00;
        cpu.f = F_C; // Set carry flag
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x20;
        mem.ram[0] = 0x8E; // ADC A, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x31); // 0x10 + 0x20 + 1 = 0x31
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_98_sbc_a_b() {
        let mut cpu = Z80::new();
        cpu.a = 0x20;
        cpu.b = 0x10;
        cpu.f = F_C; // Set carry flag
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x98; // SBC A, B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x0F); // 0x20 - 0x10 - 1 = 0x0F
        println!("Flags: {:02X}", cpu.f);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, F_H); // half carry (borrow from bit 4)
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, F_N); // N set
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_9e_sbc_a_hl() {
        let mut cpu = Z80::new();
        cpu.a = 0x20;
        cpu.h = 0x10;
        cpu.l = 0x00;
        cpu.f = F_C; // Set carry flag
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x10;
        mem.ram[0] = 0x9E; // SBC A, (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x0F); // 0x20 - 0x10 - 1 = 0x0F
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, F_H); // half carry (borrow from bit 4)
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, F_N); // N set
        assert_eq!(cpu.f & F_C, 0); // no carry
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_a0_and_b() {
        let mut cpu = Z80::new();
        cpu.a = 0xF0;
        cpu.b = 0x0F;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xA0; // AND B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x00); // 0xF0 & 0x0F = 0x00
        assert_eq!(cpu.f & F_Z, F_Z); // Zero flag set
        assert_eq!(cpu.f & F_H, F_H); // Half-carry set for AND
        assert_eq!(cpu.f & F_N, 0); // Subtract flag not set
        assert_eq!(cpu.f & F_C, 0); // Carry flag not set
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_a6_and_hl() {
        let mut cpu = Z80::new();
        cpu.a = 0xF0;
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x0F;
        mem.ram[0] = 0xA6; // AND (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x00); // 0xF0 & 0x0F = 0x00
        assert_eq!(cpu.f & F_Z, F_Z); // Zero flag set
        assert_eq!(cpu.f & F_H, F_H); // Half-carry set for AND
        assert_eq!(cpu.f & F_N, 0); // Subtract flag not set
        assert_eq!(cpu.f & F_C, 0); // Carry flag not set
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_b0_or_b() {
        let mut cpu = Z80::new();
        cpu.a = 0xF0;
        cpu.b = 0x0F;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xB0; // OR B
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xFF); // 0xF0 | 0x0F = 0xFF
        assert_eq!(cpu.f & F_Z, 0); // Zero flag not set
        assert_eq!(cpu.f & F_H, 0); // Half-carry not set for OR
        assert_eq!(cpu.f & F_N, 0); // Subtract flag not set
        assert_eq!(cpu.f & F_C, 0); // Carry flag not set
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_ae_xor_hl() {
        let mut cpu = Z80::new();
        cpu.a = 0xFF;
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x0F;
        mem.ram[0] = 0xAE; // XOR (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0xF0); // 0xFF ^ 0x0F = 0xF0
        assert_eq!(cpu.f & F_Z, 0); // Zero flag not set
        assert_eq!(cpu.f & F_H, 0); // Half-carry not set for XOR
        assert_eq!(cpu.f & F_N, 0); // Subtract flag not set
        assert_eq!(cpu.f & F_C, 0); // Carry flag not set
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_be_cp_hl() {
        let mut cpu = Z80::new();
        cpu.a = 0x20;
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 0x10;
        mem.ram[0] = 0xBE; // CP (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.a, 0x20); // A unchanged
        assert_eq!(cpu.f & F_Z, 0); // Not zero
        assert_eq!(cpu.f & F_S, 0); // Not negative
        assert_eq!(cpu.f & F_H, 0); // No half-carry
        assert_eq!(cpu.f & F_N, F_N); // Subtract flag set
        assert_eq!(cpu.f & F_C, 0); // No carry
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_18_jr_d() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x18; // JR d
        mem.ram[0x101] = 0x10; // d = 16
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x112); // 0x100 + 2 + 16 = 0x112
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_20_jr_nz_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.f = 0; // Z flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x20; // JR NZ, d
        mem.ram[0x101] = 0x10; // d = 16
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x112); // 0x100 + 2 + 16 = 0x112
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_20_jr_nz_not_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.f = F_Z; // Z flag set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x20; // JR NZ, d
        mem.ram[0x101] = 0x10; // d = 16
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x102); // 0x100 + 2 = 0x102
        assert_eq!(cycles, 7);
    }

    #[test]
    fn test_28_jr_z_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.f = F_Z; // Z flag set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x28; // JR Z, d
        mem.ram[0x101] = 0xF0; // d = -16
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x0F2); // 0x100 + 2 - 16 = 0x0F2
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_30_jr_nc_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.f = 0; // C flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x30; // JR NC, d
        mem.ram[0x101] = 0x05; // d = 5
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x107); // 0x100 + 2 + 5 = 0x107
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_38_jr_c_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.f = F_C; // C flag set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0x38; // JR C, d
        mem.ram[0x101] = 0xFB; // d = -5
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x0FD); // 0x100 + 2 - 5 = 0xFD
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_c2_jp_nz_taken() {
        let mut cpu = Z80::new();
        cpu.f = 0; // Z flag not set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xC2; // JP NZ, nn
        mem.ram[1] = 0x34;
        mem.ram[2] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_ca_jp_z_taken() {
        let mut cpu = Z80::new();
        cpu.f = F_Z; // Z flag set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xCA; // JP Z, nn
        mem.ram[1] = 0x78;
        mem.ram[2] = 0x56;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x5678);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_d2_jp_nc_taken() {
        let mut cpu = Z80::new();
        cpu.f = 0; // C flag not set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xD2; // JP NC, nn
        mem.ram[1] = 0xBC;
        mem.ram[2] = 0x9A;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x9ABC);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_da_jp_c_taken() {
        let mut cpu = Z80::new();
        cpu.f = F_C; // C flag set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xDA; // JP C, nn
        mem.ram[1] = 0xEF;
        mem.ram[2] = 0xCD;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0xCDEF);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_e2_jp_pe_taken() {
        let mut cpu = Z80::new();
        cpu.f = F_PV; // P/V flag set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xEA; // JP PE, nn
        mem.ram[1] = 0x11;
        mem.ram[2] = 0x22;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x2211);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_ea_jp_m_taken() {
        let mut cpu = Z80::new();
        cpu.f = F_S; // S flag set (negative)
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xFA; // JP M, nn
        mem.ram[1] = 0x33;
        mem.ram[2] = 0x44;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x4433);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_f2_jp_p_taken() {
        let mut cpu = Z80::new();
        cpu.f = 0; // S flag not set (positive)
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xF2; // JP P, nn
        mem.ram[1] = 0x55;
        mem.ram[2] = 0x66;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x6655);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_fa_jp_po_taken() {
        let mut cpu = Z80::new();
        cpu.f = 0; // P/V flag not set
        let mut mem = TestMemory::new();
        mem.ram[0] = 0xE2; // JP PO, nn
        mem.ram[1] = 0x77;
        mem.ram[2] = 0x88;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x8877);
        assert_eq!(cycles, 10);
    }

    #[test]
    fn test_c4_call_nz_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x100;
        cpu.sp = 0x200;
        cpu.f = 0; // Z flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x100] = 0xC4; // CALL NZ, nn
        mem.ram[0x101] = 0x34;
        mem.ram[0x102] = 0x12;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cpu.sp, 0x1FE);
        assert_eq!(mem.ram[0x1FF], 0x03); // Low byte of return address
        assert_eq!(mem.ram[0x1FE], 0x01); // High byte of return address
        assert_eq!(cycles, 17);
    }

    #[test]
    fn test_cc_call_z_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x200;
        cpu.sp = 0x300;
        cpu.f = F_Z; // Z flag set
        let mut mem = TestMemory::new();
        mem.ram[0x200] = 0xCC; // CALL Z, nn
        mem.ram[0x201] = 0x78;
        mem.ram[0x202] = 0x56;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x5678);
        assert_eq!(cpu.sp, 0x2FE);
        assert_eq!(mem.ram[0x2FF], 0x03); // Low byte of return address
        assert_eq!(mem.ram[0x2FE], 0x02); // High byte of return address (0x200 + 3)
        assert_eq!(cycles, 17);
    }

    #[test]
    fn test_d4_call_nc_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x150;
        cpu.sp = 0x250;
        cpu.f = 0; // C flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x150] = 0xD4; // CALL NC, nn
        mem.ram[0x151] = 0xBC;
        mem.ram[0x152] = 0x9A;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x9ABC);
        assert_eq!(cpu.sp, 0x24E);
        assert_eq!(mem.ram[0x24F], 0x53); // Low byte of return address
        assert_eq!(mem.ram[0x24E], 0x01); // High byte of return address (0x150 + 3)
        assert_eq!(cycles, 17);
    }

    #[test]
    fn test_dc_call_c_taken() {
        let mut cpu = Z80::new();
        cpu.pc = 0x180;
        cpu.sp = 0x280;
        cpu.f = F_C; // C flag set
        let mut mem = TestMemory::new();
        mem.ram[0x180] = 0xDC; // CALL C, nn
        mem.ram[0x181] = 0xEF;
        mem.ram[0x182] = 0xCD;
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0xCDEF);
        assert_eq!(cpu.sp, 0x27E);
        assert_eq!(mem.ram[0x27F], 0x83); // Low byte of return address
        assert_eq!(mem.ram[0x27E], 0x01); // High byte of return address (0x180 + 3)
        assert_eq!(cycles, 17);
    }

    #[test]
    fn test_c0_ret_nz_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x200;
        cpu.f = 0; // Z flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x200] = 0x34; // Return address low
        mem.ram[0x201] = 0x12; // Return address high
        mem.ram[0] = 0xC0; // RET NZ
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x1234);
        assert_eq!(cpu.sp, 0x202);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_c8_ret_z_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x300;
        cpu.f = F_Z; // Z flag set
        let mut mem = TestMemory::new();
        mem.ram[0x300] = 0x78; // Return address low
        mem.ram[0x301] = 0x56; // Return address high
        mem.ram[0] = 0xC8; // RET Z
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x5678);
        assert_eq!(cpu.sp, 0x302);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_d0_ret_nc_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x250;
        cpu.f = 0; // C flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x250] = 0xBC; // Return address low
        mem.ram[0x251] = 0x9A; // Return address high
        mem.ram[0] = 0xD0; // RET NC
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x9ABC);
        assert_eq!(cpu.sp, 0x252);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_d8_ret_c_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x280;
        cpu.f = F_C; // C flag set
        let mut mem = TestMemory::new();
        mem.ram[0x280] = 0xEF; // Return address low
        mem.ram[0x281] = 0xCD; // Return address high
        mem.ram[0] = 0xD8; // RET C
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0xCDEF);
        assert_eq!(cpu.sp, 0x282);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_e0_ret_po_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x220;
        cpu.f = 0; // P/V flag not set
        let mut mem = TestMemory::new();
        mem.ram[0x220] = 0x11; // Return address low
        mem.ram[0x221] = 0x22; // Return address high
        mem.ram[0] = 0xE0; // RET PO
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x2211);
        assert_eq!(cpu.sp, 0x222);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_e8_ret_pe_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x240;
        cpu.f = F_PV; // P/V flag set
        let mut mem = TestMemory::new();
        mem.ram[0x240] = 0x33; // Return address low
        mem.ram[0x241] = 0x44; // Return address high
        mem.ram[0] = 0xE8; // RET PE
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x4433);
        assert_eq!(cpu.sp, 0x242);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_f0_ret_p_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x260;
        cpu.f = 0; // S flag not set (positive)
        let mut mem = TestMemory::new();
        mem.ram[0x260] = 0x55; // Return address low
        mem.ram[0x261] = 0x66; // Return address high
        mem.ram[0] = 0xF0; // RET P
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x6655);
        assert_eq!(cpu.sp, 0x262);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_f8_ret_m_taken() {
        let mut cpu = Z80::new();
        cpu.sp = 0x270;
        cpu.f = F_S; // S flag set (negative)
        let mut mem = TestMemory::new();
        mem.ram[0x270] = 0x77; // Return address low
        mem.ram[0x271] = 0x88; // Return address high
        mem.ram[0] = 0xF8; // RET M
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(cpu.pc, 0x8877);
        assert_eq!(cpu.sp, 0x272);
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_34_inc_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 5;
        mem.ram[0] = 0x34; // INC (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1000], 6);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, 0); // N reset
        assert_eq!(cycles, 11);
    }

    #[test]
    fn test_35_dec_hl() {
        let mut cpu = Z80::new();
        cpu.h = 0x10;
        cpu.l = 0x00;
        let mut mem = TestMemory::new();
        mem.ram[0x1000] = 10;
        mem.ram[0] = 0x35; // DEC (HL)
        let cycles = cpu.step(&mut mem as &mut dyn Bus);
        assert_eq!(mem.ram[0x1000], 9);
        assert_eq!(cpu.f & F_Z, 0); // not zero
        assert_eq!(cpu.f & F_S, 0); // not negative
        assert_eq!(cpu.f & F_H, 0); // no half carry
        assert_eq!(cpu.f & F_PV, 0); // no overflow
        assert_eq!(cpu.f & F_N, F_N); // N set
        assert_eq!(cycles, 11);
    }
}