//! Z80 Debugging and Disassembly Module
//!
//! This module provides disassembly functionality for Z80 instructions.

use crate::emulator::{Bus};

pub struct Debugger {
    pub enable_disassembler: bool,
    pub enable_trace_interrupts: bool,
    pub enable_zexall_test: bool,
}

impl Debugger {
    pub fn new(enable_disassembler: bool, enable_trace_interrupts: bool, enable_zexall_test: bool) -> Self {
        Self {
            enable_disassembler,
            enable_trace_interrupts,
            enable_zexall_test,
        }
    }

    // Helper for CB-prefixed instructions
    fn disassemble_cb_instruction(cb_opcode: u8) -> String {
        let reg_names_8 = ["B", "C", "D", "E", "H", "L", "(HL)", "A"];
        let bit_op_types = ["RLC", "RRC", "RL", "RR", "SLA", "SRA", "SLL", "SRL"];
        let bit = (cb_opcode >> 3) & 0x07;
        let reg_idx = cb_opcode & 0x07;

        match cb_opcode {
            0x00..=0x3F => format!("{} {}", bit_op_types[((cb_opcode >> 3) & 0x07) as usize], reg_names_8[reg_idx as usize]),
            0x40..=0x7F => format!("BIT {}, {}", bit, reg_names_8[reg_idx as usize]),
            0x80..=0xBF => format!("RES {}, {}", bit, reg_names_8[reg_idx as usize]),
            0xC0..=0xFF => format!("SET {}, {}", bit, reg_names_8[reg_idx as usize]),
        }
    }

    // Helper for ED-prefixed instructions
    fn disassemble_ed_instruction(ed_opcode: u8, bus: &dyn Bus, pc: u16) -> (String, u16) {
        let mut instruction_len = 2; // ED prefix + opcode
        let mnemonic = match ed_opcode {
            0x40 => "IN B, (C)".to_string(),
            0x41 => "OUT (C), B".to_string(),
            0x42 => "SBC HL, BC".to_string(),
            0x43 => { instruction_len = 4; format!("LD ({:04X}), BC", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x44 => "NEG".to_string(),
            0x45 => "RETN".to_string(),
            0x46 => "IM 0".to_string(),
            0x47 => "LD I, A".to_string(),
            0x48 => "IN C, (C)".to_string(),
            0x49 => "OUT (C), C".to_string(),
            0x4A => "ADC HL, BC".to_string(),
            0x4B => { instruction_len = 4; format!("LD BC, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x4D => "RETI".to_string(),
            0x4F => "LD R, A".to_string(),
            0x50 => "IN D, (C)".to_string(),
            0x51 => "OUT (C), D".to_string(),
            0x52 => "SBC HL, DE".to_string(),
            0x53 => { instruction_len = 4; format!("LD ({:04X}), DE", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x56 => "IM 1".to_string(),
            0x57 => "LD A, I".to_string(),
            0x58 => "IN E, (C)".to_string(),
            0x59 => "OUT (C), E".to_string(),
            0x5A => "ADC HL, DE".to_string(),
            0x5B => { instruction_len = 4; format!("LD DE, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x5E => "IM 2".to_string(),
            0x5F => "LD A, R".to_string(),
            0x60 => "IN H, (C)".to_string(),
            0x61 => "OUT (C), H".to_string(),
            0x62 => "SBC HL, HL".to_string(),
            0x63 => { instruction_len = 4; format!("LD ({:04X}), HL", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x67 => "RRD".to_string(),
            0x68 => "IN L, (C)".to_string(),
            0x69 => "OUT (C), L".to_string(),
            0x6A => "ADC HL, HL".to_string(), // Added ADC HL, HL
            0x6B => { instruction_len = 4; format!("LD HL, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x6F => "RLD".to_string(),
            0x72 => "SBC HL, SP".to_string(),
            0x73 => { instruction_len = 4; format!("LD ({:04X}), SP", Self::read_word_at_pc_offset(bus, pc, 2)) },
            0x78 => "IN A, (C)".to_string(),
            0x79 => "OUT (C), A".to_string(),
            0x7A => "ADC HL, SP".to_string(),
            0x7B => { instruction_len = 4; format!("LD SP, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)) },

            // Block Transfer and Search Instructions
            0xA0 => "LDI".to_string(),
            0xA1 => "CPI".to_string(),
            0xA2 => "INI".to_string(),
            0xA3 => "OUTI".to_string(),
            0xA8 => "LDD".to_string(),
            0xA9 => "CPD".to_string(),
            0xAA => "IND".to_string(),
            0xAB => "OUTD".to_string(),
            0xB0 => "LDIR".to_string(),
            0xB1 => "CPIR".to_string(),
            0xB2 => "INIR".to_string(),
            0xB3 => "OTIR".to_string(),
            0xB8 => "LDDR".to_string(),
            0xB9 => "CPDR".to_string(),
            0xBA => "INDR".to_string(),
            0xBB => "OTDR".to_string(),

            _ => format!("UNKNOWN ED {:02X}", ed_opcode),
        };
        (mnemonic, instruction_len)
    }

    fn read_byte_at_pc_offset(bus: &dyn Bus, pc: u16, offset: u16) -> u8 {
        bus.read_byte(pc.wrapping_add(offset))
    }

    fn read_word_at_pc_offset(bus: &dyn Bus, pc: u16, offset: u16) -> u16 {
        let low = bus.read_byte(pc.wrapping_add(offset));
        let high = bus.read_byte(pc.wrapping_add(offset.wrapping_add(1)));
        (high as u16) << 8 | low as u16
    }

    pub fn disassemble_instruction(&self, bus: &dyn Bus, pc: u16) -> (String, u16) {
        let opcode = bus.read_byte(pc);
        let mut instruction_len = 1;
        let mnemonic;

        match opcode {
            // 8-bit loads
            0x06 => { mnemonic = format!("LD B, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x0E => { mnemonic = format!("LD C, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x16 => { mnemonic = format!("LD D, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x1E => { mnemonic = format!("LD E, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x26 => { mnemonic = format!("LD H, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x2E => { mnemonic = format!("LD L, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x36 => { mnemonic = format!("LD (HL), #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0x3E => { mnemonic = format!("LD A, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }

            // 16-bit loads
            0x01 => { mnemonic = format!("LD BC, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x11 => { mnemonic = format!("LD DE, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x21 => { mnemonic = format!("LD HL, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x31 => { mnemonic = format!("LD SP, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }

            0x2A => { mnemonic = format!("LD HL, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x3A => { mnemonic = format!("LD A, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x32 => { mnemonic = format!("LD ({:04X}), A", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0x22 => { mnemonic = format!("LD ({:04X}), HL", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }

            // Jumps and calls
            0xC3 => { mnemonic = format!("JP #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xCD => { mnemonic = format!("CALL #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xC2 => { mnemonic = format!("JP NZ, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xCA => { mnemonic = format!("JP Z, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xD2 => { mnemonic = format!("JP NC, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xDA => { mnemonic = format!("JP C, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xE2 => { mnemonic = format!("JP PO, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xEA => { mnemonic = format!("JP PE, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xF2 => { mnemonic = format!("JP P, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }
            0xFA => { mnemonic = format!("JP M, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 1)); instruction_len = 3; }

            0x18 => { mnemonic = format!("JR #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }
            0x20 => { mnemonic = format!("JR NZ, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }
            0x28 => { mnemonic = format!("JR Z, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }
            0x30 => { mnemonic = format!("JR NC, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }
            0x38 => { mnemonic = format!("JR C, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }
            0x10 => { mnemonic = format!("DJNZ #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1) as i8); instruction_len = 2; }

            // ALU ops
            0xC6 => { mnemonic = format!("ADD A, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xCE => { mnemonic = format!("ADC A, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xD6 => { mnemonic = format!("SUB #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xDE => { mnemonic = format!("SBC A, #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xE6 => { mnemonic = format!("AND #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xEE => { mnemonic = format!("XOR #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xF6 => { mnemonic = format!("OR #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }
            0xFE => { mnemonic = format!("CP #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; }

            // Prefixes
            0xCB => {
                let cb_opcode = bus.read_byte(pc.wrapping_add(1));
                instruction_len = 2;
                mnemonic = Self::disassemble_cb_instruction(cb_opcode);
            }
            0xDD => {
                let dd_opcode = bus.read_byte(pc.wrapping_add(1));
                instruction_len = 2;
                match dd_opcode {
                    0x21 => { mnemonic = format!("LD IX, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x22 => { mnemonic = format!("LD ({:04X}), IX", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x2A => { mnemonic = format!("LD IX, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x36 => { mnemonic = format!("LD (IX+{}), #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 2) as i8, Self::read_byte_at_pc_offset(bus, pc, 3)); instruction_len = 4; }
                    0xCB => { // DDCB prefix
                        let d = Self::read_byte_at_pc_offset(bus, pc, 2) as i8;
                        let ddcbc_opcode = Self::read_byte_at_pc_offset(bus, pc, 3);
                        instruction_len = 4;
                        let bit_op_types = ["RLC", "RRC", "RL", "RR", "SLA", "SRA", "SLL", "SRL"];

                        match ddcbc_opcode {
                            0x00..=0x3F => { // Rotates and shifts
                                let op_type_idx = (ddcbc_opcode >> 3) & 0x07;
                                mnemonic = format!("{} (IX+{})", bit_op_types[op_type_idx as usize], d);
                            }
                            0x40..=0x7F => { // BIT
                                let bit = (ddcbc_opcode >> 3) & 0x07;
                                mnemonic = format!("BIT {}, (IX+{})", bit, d);
                            }
                            0x80..=0xBF => { // RES
                                let bit = (ddcbc_opcode >> 3) & 0x07;
                                mnemonic = format!("RES {}, (IX+{})", bit, d);
                            }
                            0xC0..=0xFF => { // SET
                                let bit = (ddcbc_opcode >> 3) & 0x07;
                                mnemonic = format!("SET {}, (IX+{})", bit, d);
                            }
                        }
                    }
                    _ => mnemonic = format!("DD {:02X}", dd_opcode) // Fallback for other DD instructions
                }
            }
            0xED => {
                let ed_opcode = bus.read_byte(pc.wrapping_add(1));
                (mnemonic, instruction_len) = Self::disassemble_ed_instruction(ed_opcode, bus, pc);
            }
            0xFD => {
                let fd_opcode = bus.read_byte(pc.wrapping_add(1));
                instruction_len = 2;
                match fd_opcode {
                    0x21 => { mnemonic = format!("LD IY, #{:04X}", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x22 => { mnemonic = format!("LD ({:04X}), IY", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x2A => { mnemonic = format!("LD IY, ({:04X})", Self::read_word_at_pc_offset(bus, pc, 2)); instruction_len = 4; }
                    0x36 => { mnemonic = format!("LD (IY+{}), #{:02X}", Self::read_byte_at_pc_offset(bus, pc, 2) as i8, Self::read_byte_at_pc_offset(bus, pc, 3)); instruction_len = 4; }
                    0xCB => { // FDCB prefix
                        let d = Self::read_byte_at_pc_offset(bus, pc, 2) as i8;
                        let fdcb_opcode = Self::read_byte_at_pc_offset(bus, pc, 3);
                        instruction_len = 4;
                        let bit_op_types = ["RLC", "RRC", "RL", "RR", "SLA", "SRA", "SLL", "SRL"];

                        match fdcb_opcode {
                            0x00..=0x3F => { // Rotates and shifts
                                let op_type_idx = (fdcb_opcode >> 3) & 0x07;
                                mnemonic = format!("{} (IY+{})", bit_op_types[op_type_idx as usize], d);
                            }
                            0x40..=0x7F => { // BIT
                                let bit = (fdcb_opcode >> 3) & 0x07;
                                mnemonic = format!("BIT {}, (IY+{})", bit, d);
                            }
                            0x80..=0xBF => { // RES
                                let bit = (fdcb_opcode >> 3) & 0x07;
                                mnemonic = format!("RES {}, (IY+{})", bit, d);
                            }
                            0xC0..=0xFF => { // SET
                                let bit = (fdcb_opcode >> 3) & 0x07;
                                mnemonic = format!("SET {}, (IY+{})", bit, d);
                            }
                        }
                    }
                    _ => mnemonic = format!("FD {:02X}", fd_opcode) // Fallback for other FD instructions
                }
            }
            // General 8-bit register loads (LD r, r')
            0x40..=0x7F if opcode != 0x76 => { // Exclude HALT
                let r_dest = (opcode >> 3) & 0x07;
                let r_src = opcode & 0x07;
                let reg_names = ["B", "C", "D", "E", "H", "L", "(HL)", "A"];
                mnemonic = format!("LD {}, {}", reg_names[r_dest as usize], reg_names[r_src as usize]);
            }
            // Other single byte opcodes
            0x00 => mnemonic = "NOP".to_string(),
            0x02 => mnemonic = "LD (BC), A".to_string(),
            0x03 => mnemonic = "INC BC".to_string(),
            0x04 => mnemonic = "INC B".to_string(),
            0x05 => mnemonic = "DEC B".to_string(),
            0x07 => mnemonic = "RLCA".to_string(),
            0x08 => mnemonic = "EX AF, AF'".to_string(),
            0x09 => mnemonic = "ADD HL, BC".to_string(),
            0x0A => mnemonic = "LD A, (BC)".to_string(),
            0x0B => mnemonic = "DEC BC".to_string(),
            0x0C => mnemonic = "INC C".to_string(),
            0x0D => mnemonic = "DEC C".to_string(),
            0x0F => mnemonic = "RRCA".to_string(),
            0x12 => mnemonic = "LD (DE), A".to_string(),
            0x13 => mnemonic = "INC DE".to_string(),
            0x14 => mnemonic = "INC D".to_string(),
            0x15 => mnemonic = "DEC D".to_string(),
            0x17 => mnemonic = "RLA".to_string(),
            0x19 => mnemonic = "ADD HL, DE".to_string(),
            0x1A => mnemonic = "LD A, (DE)".to_string(),
            0x1B => mnemonic = "DEC DE".to_string(),
            0x1C => mnemonic = "INC E".to_string(),
            0x1D => mnemonic = "DEC E".to_string(),
            0x1F => mnemonic = "RRA".to_string(),
            0x23 => mnemonic = "INC HL".to_string(),
            0x24 => mnemonic = "INC H".to_string(),
            0x25 => mnemonic = "DEC H".to_string(),
            0x27 => mnemonic = "DAA".to_string(),
            0x29 => mnemonic = "ADD HL, HL".to_string(),
            0x2B => mnemonic = "DEC HL".to_string(),
            0x2C => mnemonic = "INC L".to_string(),
            0x2D => mnemonic = "DEC L".to_string(),
            0x2F => mnemonic = "CPL".to_string(),
            0x33 => mnemonic = "INC SP".to_string(),
            0x34 => mnemonic = "INC (HL)".to_string(),
            0x35 => mnemonic = "DEC (HL)".to_string(),
            0x37 => mnemonic = "SCF".to_string(),
            0x39 => mnemonic = "ADD HL, SP".to_string(),
            0x3B => mnemonic = "DEC SP".to_string(),
            0x3C => mnemonic = "INC A".to_string(),
            0x3D => mnemonic = "DEC A".to_string(),
            0x3F => mnemonic = "CCF".to_string(),
            0x76 => mnemonic = "HALT".to_string(),
            0x80 => mnemonic = "ADD A, B".to_string(),
            0x81 => mnemonic = "ADD A, C".to_string(),
            0x82 => mnemonic = "ADD A, D".to_string(),
            0x83 => mnemonic = "ADD A, E".to_string(),
            0x84 => mnemonic = "ADD A, H".to_string(),
            0x85 => mnemonic = "ADD A, L".to_string(),
            0x86 => mnemonic = "ADD A, (HL)".to_string(),
            0x87 => mnemonic = "ADD A, A".to_string(),
            0x88 => mnemonic = "ADC A, B".to_string(),
            0x89 => mnemonic = "ADC A, C".to_string(),
            0x8A => mnemonic = "ADC A, D".to_string(),
            0x8B => mnemonic = "ADC A, E".to_string(),
            0x8C => mnemonic = "ADC A, H".to_string(),
            0x8D => mnemonic = "ADC A, L".to_string(),
            0x8E => mnemonic = "ADC A, (HL)".to_string(),
            0x8F => mnemonic = "ADC A, A".to_string(),
            0x90 => mnemonic = "SUB B".to_string(),
            0x91 => mnemonic = "SUB C".to_string(),
            0x92 => mnemonic = "SUB D".to_string(),
            0x93 => mnemonic = "SUB E".to_string(),
            0x94 => mnemonic = "SUB H".to_string(),
            0x95 => mnemonic = "SUB L".to_string(),
            0x96 => mnemonic = "SUB (HL)".to_string(),
            0x97 => mnemonic = "SUB A".to_string(),
            0x98 => mnemonic = "SBC A, B".to_string(),
            0x99 => mnemonic = "SBC A, C".to_string(),
            0x9A => mnemonic = "SBC A, D".to_string(),
            0x9B => mnemonic = "SBC A, E".to_string(),
            0x9C => mnemonic = "SBC A, H".to_string(),
            0x9D => mnemonic = "SBC A, L".to_string(),
            0x9E => mnemonic = "SBC A, (HL)".to_string(),
            0x9F => mnemonic = "SBC A, A".to_string(),
            0xA0 => mnemonic = "AND B".to_string(),
            0xA1 => mnemonic = "AND C".to_string(),
            0xA2 => mnemonic = "AND D".to_string(),
            0xA3 => mnemonic = "AND E".to_string(),
            0xA4 => mnemonic = "AND H".to_string(),
            0xA5 => mnemonic = "AND L".to_string(),
            0xA6 => mnemonic = "AND (HL)".to_string(),
            0xA7 => mnemonic = "AND A".to_string(),
            0xA8 => mnemonic = "XOR B".to_string(),
            0xA9 => mnemonic = "XOR C".to_string(),
            0xAA => mnemonic = "XOR D".to_string(),
            0xAB => mnemonic = "XOR E".to_string(),
            0xAC => mnemonic = "XOR H".to_string(),
            0xAD => mnemonic = "XOR L".to_string(),
            0xAE => mnemonic = "XOR (HL)".to_string(),
            0xAF => mnemonic = "XOR A".to_string(),
            0xB0 => mnemonic = "OR B".to_string(),
            0xB1 => mnemonic = "OR C".to_string(),
            0xB2 => mnemonic = "OR D".to_string(),
            0xB3 => mnemonic = "OR E".to_string(),
            0xB4 => mnemonic = "OR H".to_string(),
            0xB5 => mnemonic = "OR L".to_string(),
            0xB6 => mnemonic = "OR (HL)".to_string(),
            0xB7 => mnemonic = "OR A".to_string(),
            0xB8 => mnemonic = "CP B".to_string(),
            0xB9 => mnemonic = "CP C".to_string(),
            0xBA => mnemonic = "CP D".to_string(),
            0xBB => mnemonic = "CP E".to_string(),
            0xBC => mnemonic = "CP H".to_string(),
            0xBD => mnemonic = "CP L".to_string(),
            0xBE => mnemonic = "CP (HL)".to_string(),
            0xBF => mnemonic = "CP A".to_string(),
            0xC0 => mnemonic = "RET NZ".to_string(),
            0xC1 => mnemonic = "POP BC".to_string(),
            0xC4 => mnemonic = "CALL NZ, nn".to_string(),
            0xC5 => mnemonic = "PUSH BC".to_string(),
            0xC7 => mnemonic = "RST 00h".to_string(),
            0xC8 => mnemonic = "RET Z".to_string(),
            0xC9 => mnemonic = "RET".to_string(),
            0xCC => mnemonic = "CALL Z, nn".to_string(),
            0xCF => mnemonic = "RST 08h".to_string(),
            0xD0 => mnemonic = "RET NC".to_string(),
            0xD1 => mnemonic = "POP DE".to_string(),
            0xD3 => {mnemonic = format!("OUT ({:02X}), A", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; },
            0xD4 => mnemonic = "CALL NC, nn".to_string(),
            0xD5 => mnemonic = "PUSH DE".to_string(),
            0xD7 => mnemonic = "RST 10h".to_string(),
            0xD8 => mnemonic = "RET C".to_string(),
            0xD9 => mnemonic = "EXX".to_string(),
            0xDB => {mnemonic = format!("IN A, ({:02X})", Self::read_byte_at_pc_offset(bus, pc, 1)); instruction_len = 2; },
            0xDC => mnemonic = "CALL C, nn".to_string(),
            0xDF => mnemonic = "RST 18h".to_string(),
            0xE0 => mnemonic = "RET PO".to_string(),
            0xE1 => mnemonic = "POP HL".to_string(),
            0xE3 => mnemonic = "EX (SP), HL".to_string(),
            0xE4 => mnemonic = "CALL PO, nn".to_string(),
            0xE5 => mnemonic = "PUSH HL".to_string(),
            0xE7 => mnemonic = "RST 20h".to_string(),
            0xE8 => mnemonic = "RET PE".to_string(),
            0xE9 => mnemonic = "JP (HL)".to_string(),
            0xEB => mnemonic = "EX DE, HL".to_string(),
            0xEC => mnemonic = "CALL PE, nn".to_string(),
            0xEF => mnemonic = "RST 28h".to_string(),
            0xF0 => mnemonic = "RET P".to_string(),
            0xF1 => mnemonic = "POP AF".to_string(),
            0xF3 => mnemonic = "DI".to_string(),
            0xF4 => mnemonic = "CALL P, nn".to_string(),
            0xF5 => mnemonic = "PUSH AF".to_string(),
            0xF7 => mnemonic = "RST 30h".to_string(),
            0xF8 => mnemonic = "RET M".to_string(),
            0xF9 => mnemonic = "LD SP, HL".to_string(),
            0xFB => mnemonic = "EI".to_string(),
            0xFC => mnemonic = "CALL M, nn".to_string(),
            0xFF => mnemonic = "RST 38h".to_string(),
            _ => mnemonic = format!("UNKNOWN {:02X}", opcode)
        }
        (mnemonic, instruction_len)
    }

    pub fn trace_instruction(&self, bus: &dyn Bus, pc: u16, a: u8, f: u8, b: u8, c: u8, d: u8, e: u8, h: u8, l: u8, sp: u16, ix: u16, iy: u16, i: u8, r: u8) {
        let (disassembly, instruction_len) = self.disassemble_instruction(bus, pc);
        let mut instruction_bytes = Vec::new();
        for i in 0..instruction_len {
            instruction_bytes.push(bus.read_byte(pc.wrapping_add(i)));
        }
        println!("{:04X}: {:<10} | A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} IX:{:04X} IY:{:04X} I:{:02X} R:{:02X} | Bytes: {}",
            pc,
            disassembly,
            a, f, b, c, d, e, h, l, sp, ix, iy, i, r,
            instruction_bytes.iter().map(|b| format!("{:02X}", b)).collect::<Vec<String>>().join(" "),
        );
    }
}