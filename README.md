Draft of ZX Spectrum emulator on Rust.
======================================

TODO:

* implement missing commands
* add tests
* check flags correctness

Testing CPU status (ZEXALL):
============================
<pre>
Loaded ZEXALL test ROM from roms/zexall-0x0100.rom
Z80 instruction exerciser
<adc,sbc> hl,<bc,de,hl,sp>....  ERROR **** crc expected:d48ad519 found:f39089a0
add hl,<bc,de,hl,sp>..........  OK
add ix,<bc,de,ix,sp>..........  ERROR **** crc expected:b1df8ec0 found:e4ae3aef
add iy,<bc,de,iy,sp>..........  ERROR **** crc expected:39c8589b found:b0363496
aluop a,nn....................  ERROR **** crc expected:51c19c2e found:ff09e96c
aluop a,<b,c,d,e,h,l,(hl),a>..  ERROR **** crc expected:06c7aa8e found:dbd06e78
aluop a,<ixh,ixl,iyh,iyl>.....  ERROR **** crc expected:a886cc44 found:eab157e2
aluop a,(<ix,iy>+1)...........  ERROR **** crc expected:d3f2d74a found:722c9987
bit n,(<ix,iy>+1).............  ERROR **** crc expected:83534ee1 found:2b526613
bit n,<b,c,d,e,h,l,(hl),a>....  ERROR **** crc expected:5e020e98 found:fe7d580c
cpd<r>........................  OK
cpi<r>........................  OK
<daa,cpl,scf,ccf>.............  ERROR **** crc expected:6d2dd213 found:f10482f9
<inc,dec> a...................  OK
<inc,dec> b...................  OK
<inc,dec> bc..................  OK
<inc,dec> c...................  OK
<inc,dec> d...................  OK
<inc,dec> de..................  OK
<inc,dec> e...................  OK
<inc,dec> h...................  OK
<inc,dec> hl..................  OK
<inc,dec> ix..................  OK
<inc,dec> iy..................  OK
<inc,dec> l...................  OK
<inc,dec> (hl)................  OK
<inc,dec> sp..................  OK
<inc,dec> (<ix,iy>+1).........  OK
<inc,dec> ixh.................  ERROR **** crc expected:6f463662 found:04c71dc4
<inc,dec> ixl.................  ERROR **** crc expected:027bef2c found:705b9016
<inc,dec> iyh.................  ERROR **** crc expected:2d966cf3 found:5fb613c9
<inc,dec> iyl.................  ERROR **** crc expected:36c11e75 found:44e1614f
ld <bc,de>,(nnnn).............  ERROR **** crc expected:4d45a9ac found:b21e8b4d
ld hl,(nnnn)..................  OK
ld sp,(nnnn)..................  OK
ld <ix,iy>,(nnnn).............  OK
ld (nnnn),<bc,de>.............  OK
ld (nnnn),hl..................  OK
ld (nnnn),sp..................  OK
ld (nnnn),<ix,iy>.............  OK
ld <bc,de,hl,sp>,nnnn.........  OK
ld <ix,iy>,nnnn...............  OK
ld a,<(bc),(de)>..............  OK
ld <b,c,d,e,h,l,(hl),a>,nn....  OK
ld (<ix,iy>+1),nn.............  OK
ld <b,c,d,e>,(<ix,iy>+1)......  OK
ld <h,l>,(<ix,iy>+1)..........  OK
ld a,(<ix,iy>+1)..............  OK
ld <ixh,ixl,iyh,iyl>,nn.......  OK
ld <bcdehla>,<bcdehla>........  OK
ld <bcdexya>,<bcdexya>........
thread 'main' panicked at src\cpu.rs:3606:17:
Unimplemented DD opcode: 40
</pre>