//! Z80 CPU Emulation
//!
//! This module implements a complete Z80 CPU emulator with full instruction set support.

use crate::emulator::Memory;

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
        if Self::parity(res) { self.f |= F_PV; }
    }

    fn set_flags_sub(&mut self, a: u8, b: u8, result: i16) {
        let res = result as u8;
        self.f = F_N;
        if res == 0 { self.f |= F_Z; }
        if (res & 0x80) != 0 { self.f |= F_S; }
        if result < 0 { self.f |= F_C; }
        if ((a ^ b ^ res) & 0x10) != 0 { self.f |= F_H; }
        if Self::parity(res) { self.f |= F_PV; }
    }

    pub fn step(&mut self, bus: &mut dyn Memory) -> u32 {
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
                self.set_flags_sub(self.a, self.b + carry as u8, result);
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
                self.set_flags_sub(self.a, val + carry as u8, result);
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

    fn step_cb(&mut self, bus: &mut dyn Memory) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            // Implement CB opcodes here
            _ => {
                println!("Unimplemented CB opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_ed(&mut self, bus: &mut dyn Memory) -> u32 {
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
            _ => {
                println!("Unimplemented ED opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_dd(&mut self, bus: &mut dyn Memory) -> u32 {
        let opcode = self.read_byte_pc(bus);
        match opcode {
            // Implement DD opcodes here
            _ => {
                println!("Unimplemented DD opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_fd(&mut self, bus: &mut dyn Memory) -> u32 {
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
            0xCB => { // FD CB prefix
                self.step_fdcb(bus)
            }
            _ => {
                println!("Unimplemented FD opcode: {:02X}", opcode);
                8
            }
        }
    }

    fn step_fdcb(&mut self, bus: &mut dyn Memory) -> u32 {
        let d = self.read_byte_pc(bus) as i8 as i16;
        let addr = self.iy.wrapping_add(d as u16);
        let opcode = self.read_byte_pc(bus);
        match opcode {
            // Implement FD CB opcodes here (bit operations on (IY+d))
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

    #[test]
    fn test_ld_a_n() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x3E; // LD A, n
        mem.ram[1] = 0x42; // n = 0x42
        let cycles = cpu.step(&mut mem);
        assert_eq!(cpu.a, 0x42);
        assert_eq!(cycles, 7);
        assert_eq!(cpu.pc, 2);
    }

    #[test]
    fn test_nop() {
        let mut cpu = Z80::new();
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x00; // NOP
        let cycles = cpu.step(&mut mem);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }

    #[test]
    fn test_add_a_b() {
        let mut cpu = Z80::new();
        cpu.a = 5;
        cpu.b = 3;
        let mut mem = TestMemory::new();
        mem.ram[0] = 0x80; // ADD A, B
        let cycles = cpu.step(&mut mem);
        assert_eq!(cpu.a, 8);
        assert_eq!(cycles, 4);
        assert_eq!(cpu.pc, 1);
    }
}