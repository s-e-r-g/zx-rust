Draft of ZX Spectrum emulator on Rust.
======================================

TODO:

* implement missing commands
* add tests
* check flags correctness


Testing CPU status (ZEXALL):
============================
```
$ cargo run --release -- --run-zexall --no-ui
```

```
Loaded ZEXALL/ZEXCOM test ROM from roms/zexall-0x0100.rom
Z80 instruction exerciser
<adc,sbc> hl,<bc,de,hl,sp>....  OK
add hl,<bc,de,hl,sp>..........  OK
add ix,<bc,de,ix,sp>..........  OK
add iy,<bc,de,iy,sp>..........  OK
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
<inc,dec> ixh.................  OK
<inc,dec> ixl.................  OK
<inc,dec> iyh.................  OK
<inc,dec> iyl.................  OK
ld <bc,de>,(nnnn).............  OK
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
ld <bcdexya>,<bcdexya>........  OK
ld a,(nnnn) / ld (nnnn),a.....  OK
ldd<r> (1)....................  OK
ldd<r> (2)....................  ERROR **** crc expected:39dd3de1 found:58232761
ldi<r> (1)....................  ERROR **** crc expected:f782b0d1 found:980eaf00
ldi<r> (2)....................  ERROR **** crc expected:e9ead0ae found:e798d5ff
neg...........................  ERROR **** crc expected:d638dd6a found:f7dc0ecd
<rrd,rld>.....................  OK
<rlca,rrca,rla,rra>...........  OK
shf/rot (<ix,iy>+1)...........  ERROR **** crc expected:710034cb found:0e312ba1
shf/rot <b,c,d,e,h,l,(hl),a>..  OK
<set,res> n,<bcdehl(hl)a>.....  ERROR **** crc expected:8b57f008 found:a965275e
<set,res> n,(<ix,iy>+1).......  OK
ld (<ix,iy>+1),<b,c,d,e>......  OK
ld (<ix,iy>+1),<h,l>..........  OK
ld (<ix,iy>+1),a..............  OK
ld (<bc,de>),a................  OK
Tests complete
```
