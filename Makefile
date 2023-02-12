
CFLAGS = -O3 -Wall -pedantic -g3 -Werror -Wno-stringop-truncation

TARGETS=rc2014 rcbus-1802 rcbus-6303 rcbus-6502 rcbus-65c816-mini \
	rcbus-65c816 rcbus-6800 rcbus-68008 rcbus-6809 rcbus-68hc11 \
	rcbus-80c188 rcbus-8085 rcbus-z8 rcbus-z180 rbcv2 searle linc80 \
	makedisk markiv mbc2 smallz80 sbc2g z80mc simple80 flexbox tiny68k \
	s100-z80 scelbi rb-mbc rcbus-tms9995 rhyophyre pz1 68knano \
	littleboard mini68k kiss68030

all:	$(TARGETS)

sdl2:	rc2014_sdl2 nc100 nc200 n8_sdl2 scelbi_sdl2 nascom uk101 z180-mini-itx_sdl2 vz300

libz80/libz80.o:
	$(MAKE) --directory libz80

libz180/libz180.o:
	$(MAKE) --directory libz180

lib765/lib/lib765.a:
	$(MAKE) --directory lib765/lib

am9511/libam9511.a:
	$(MAKE) --directory am9511

rc2014:	rc2014.o rc2014_noui.o acia.o amd9511.o ef9345.o ef9345_norender.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_norender.o w5100.o z80dma.o z80copro.o zxkey_none.o z80dis.o libz80/libz80.o lib765/lib/lib765.a am9511/libam9511.a
	cc -g3 rc2014.o rc2014_noui.o zxkey_none.o acia.o amd9511.o ef9345.o ef9345_norender.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_norender.o w5100.o z80dma.o z80copro.o z80dis.o libz80/libz80.o lib765/lib/lib765.a am9511/libam9511.a -lm -o rc2014

rc2014_sdl2: rc2014.o rc2014_sdlui.o acia.o amd9511.o ef9345.o ef9345_sdl2.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_sdl2.o w5100.o z80dma.o z80copro.o zxkey_sdl2.o keymatrix.o z80dis.o libz80/libz80.o lib765/lib/lib765.a am9511/libam9511.a
	cc -g3 rc2014.o rc2014_sdlui.o acia.o amd9511.o ef9345.o ef9345_sdl2.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_sdl2.o w5100.o z80dma.o z80copro.o zxkey_sdl2.o keymatrix.o z80dis.o libz80/libz80.o lib765/lib/lib765.a am9511/libam9511.a -lm -o rc2014_sdl2 -lSDL2

rb-mbc:	rb-mbc.o 16x50.o ide.o ppide.o rtc_bitbang.o z80dis.o libz80/libz80.o
	cc -g3 rb-mbc.o 16x50.o ide.o ppide.o rtc_bitbang.o z80dis.o libz80/libz80.o -o rb-mbc

rbcv2:	rbcv2.o 16x50.o ide.o ppide.o propio.o ramf.o rtc_bitbang.o w5100.o libz80/libz80.o
	cc -g3 rbcv2.o 16x50.o ide.o ppide.o propio.o ramf.o rtc_bitbang.o w5100.o libz80/libz80.o -o rbcv2

searle:	searle.o ide.o libz80/libz80.o
	cc -g3 searle.o ide.o libz80/libz80.o -o searle

linc80:	linc80.o ide.o sdcard.o libz80/libz80.o
	cc -g3 linc80.o ide.o sdcard.o libz80/libz80.o -o linc80

littleboard:	littleboard.o ncr5380.o sasi.o wd17xx.o z80dis.o libz80/libz80.o
	cc -g3 littleboard.o ncr5380.o sasi.o wd17xx.o z80dis.o libz80/libz80.o -o littleboard

mbc2:	mbc2.o ide.o libz80/libz80.o
	cc -g3 mbc2.o libz80/libz80.o -o mbc2

rcbus-1802: rcbus-1802.o 1802.o ide.o acia.o w5100.o ppide.o rtc_bitbang.o 16x50.o
	cc -g3 rcbus-1802.o acia.o ide.o ppide.o rtc_bitbang.o 16x50.o w5100.o 1802.o -o rcbus-1802

rcbus-6303: rcbus-6303.o 6800.o ide.o w5100.o ppide.o rtc_bitbang.o
	cc -g3 rcbus-6303.o ide.o ppide.o rtc_bitbang.o w5100.o 6800.o -o rcbus-6303

rcbus-6502: rcbus-6502.o 6502.o 6502dis.o ide.o 6522.o acia.o 16x50.o rtc_bitbang.o w5100.o
	cc -g3 rcbus-6502.o ide.o 6522.o acia.o 16x50.o rtc_bitbang.o w5100.o 6502.o 6502dis.o -o rcbus-6502

rcbus-65c816: rcbus-65c816.o sram_mmu8.o ide.o 6522.o rtc_bitbang.o acia.o 16x50.o w5100.o lib65c816/src/lib65816.a
	cc -g3 rcbus-65c816.o sram_mmu8.o ide.o 6522.o rtc_bitbang.o acia.o 16x50.o w5100.o lib65c816/src/lib65816.a -o rcbus-65c816

rcbus-65c816-mini: rcbus-65c816-mini.o ide.o 6522.o rtc_bitbang.o acia.o 16x50.o w5100.o lib65c816/src/lib65816.a
	cc -g3 rcbus-65c816-mini.o ide.o 6522.o rtc_bitbang.o acia.o 16x50.o w5100.o lib65c816/src/lib65816.a -o rcbus-65c816-mini

lib65c816/src/lib65816.a:
	$(MAKE) --directory lib65c816 -j 1

rcbus-65c816.o: rcbus-65c816-mini.c lib65c816/src/lib65816.a
	$(CC) $(CFLAGS) -Ilib65c816 -c rcbus-65c816.c

rcbus-65c816-mini.o: rcbus-65c816-mini.c lib65c816/src/lib65816.a
	$(CC) $(CFLAGS) -Ilib65c816 -c rcbus-65c816-mini.c

rcbus-6800: rcbus-6800.o 6800.o ide.o acia.o 16x50.o
	cc -g3 rcbus-6800.o ide.o acia.o 6800.o 16x50.o -o rcbus-6800

rcbus-6809: rcbus-6809.o d6809.o e6809.o ide.o ppide.o w5100.o rtc_bitbang.o 6840.o 16x50.o
	cc -g3 rcbus-6809.o ide.o ppide.o w5100.o rtc_bitbang.o 6840.o 16x50.o d6809.o e6809.o -o rcbus-6809

rcbus-68hc11: rcbus-68hc11.o 68hc11.o ide.o w5100.o ppide.o rtc_bitbang.o sdcard.o
	cc -g3 rcbus-68hc11.o ide.o ppide.o rtc_bitbang.o sdcard.o w5100.o 68hc11.o -o rcbus-68hc11

rcbus-68008: rcbus-68008.o sram_mmu8.o ide.o w5100.o 16x50.o acia.o rtc_bitbang.o m68k/lib68k.a
	cc -g3 rcbus-68008.o sram_mmu8.o ide.o w5100.o ppide.o 16x50.o acia.o rtc_bitbang.o m68k/lib68k.a -o rcbus-68008

m68k/lib68k.a:
	$(MAKE) --directory m68k

rcbus-68008.o: rcbus-68008.c m68k/lib68k.a
	$(CC) $(CFLAGS) -Im68k -c rcbus-68008.c

rcbus-8085: rcbus-8085.o intel_8085_emulator.o ide.o acia.o w5100.o ppide.o rtc_bitbang.o 16x50.o
	cc -g3 rcbus-8085.o acia.o ide.o ppide.o rtc_bitbang.o 16x50.o w5100.o intel_8085_emulator.o -o rcbus-8085

rcbus-80c188: rcbus-80c188.o ide.o w5100.o ppide.o rtc_bitbang.o
	$(MAKE) --directory 80x86 && \
	cc -g3 rcbus-80c188.o ide.o ppide.o rtc_bitbang.o w5100.o 80x86/*.o -o rcbus-80c188

rcbus-ns32k: rcbus-ns32k.o ide.o ppide.o 16x50.o w5100.o rtc_bitbang.o ns32k/32016.o
	$(MAKE) --directory ns32k
	cc -g3 rcbus-ns32k.o ide.o ppide.o 16x50.o w5100.o rtc_bitbang.o ns32k/32016.c -o rcbus-ns32k

rcbus-tms9995: rcbus-tms9995.o tms9995.o ide.o ppide.o w5100.o rtc_bitbang.o 16x50.o tms9902.o
	cc -g3 rcbus-tms9995.o ide.o ppide.o w5100.o rtc_bitbang.o 16x50.o tms9902.o tms9995.o -o rcbus-tms9995

rcbus-z280: rcbus-z280.o ide.o libz280/libz80.o
	cc -g3 rcbus-z280.o ide.o libz280/libz80.o -o rcbus-z280

rcbus-z8: rcbus-z8.o z8.o ide.o acia.o w5100.o ppide.o rtc_bitbang.o
	cc -g3 rcbus-z8.o acia.o ide.o ppide.o rtc_bitbang.o w5100.o z8.o -o rcbus-z8

rcbus-z180:	rcbus-z180.o rc2014_noui.o z180_io.o 16x50.o acia.o ide.o ppide.o piratespi.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_norender.o w5100.o zxkey_none.o z80dis.o libz180/libz180.o lib765/lib/lib765.a
	cc -g3 rcbus-z180.o rc2014_noui.o z180_io.o zxkey_none.o 16x50.o acia.o ide.o piratespi.o ppide.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_norender.o w5100.o z80dis.o libz180/libz180.o lib765/lib/lib765.a -o rcbus-z180

smallz80: smallz80.o ide.o libz80/libz80.o
	cc -g3 smallz80.o ide.o libz80/libz80.o -o smallz80

sbc2g:	sbc2g.o ide.o libz80/libz80.o
	cc -g3 sbc2g.o ide.o libz80/libz80.o -o sbc2g

tiny68k: tiny68k.o ide.o duart.o m68k/lib68k.a
	cc -g3 tiny68k.o ide.o duart.o m68k/lib68k.a -o tiny68k

tiny68k.o: tiny68k.c m68k/lib68k.a
	$(CC) $(CFLAGS) -Im68k -c tiny68k.c

68knano: 68knano.o ide.o 16x50.o ds3234.o m68k/lib68k.a
	cc -g3 68knano.o ide.o 16x50.o ds3234.o m68k/lib68k.a -o 68knano

68knano.o: 68knano.c m68k/lib68k.a
	$(CC) $(CFLAGS) -Im68k -c 68knano.c

mini68k: mini68k.o ide.o ppide.o 16x50.o ns202.o rtc_bitbang.o sdcard.o m68k/lib68k.a lib765/lib/lib765.a
	cc -g3 mini68k.o ide.o ppide.o 16x50.o ns202.o rtc_bitbang.o sdcard.o m68k/lib68k.a lib765/lib/lib765.a -o mini68k

kiss68030: kiss68030.o ide.o ppide.o 16x50.o ns202.o rtc_bitbang.o sdcard.o m68k/lib68k.a lib765/lib/lib765.a
	cc -g3 kiss68030.o ide.o ppide.o 16x50.o ns202.o rtc_bitbang.o sdcard.o m68k/lib68k.a lib765/lib/lib765.a -o kiss68030

mini68k.o: mini68k.c m68k/lib68k.a
	$(CC) $(CFLAGS) -Im68k -c mini68k.c

kiss68030.o: kiss68030.c m68k/lib68k.a
	$(CC) $(CFLAGS) -O3 -Im68k -c kiss68030.c

z80mc:	z80mc.o sdcard.o libz80/libz80.o
	cc -g3 z80mc.o sdcard.o libz80/libz80.o -o z80mc

z180-mini-itx: z180-mini-itx.o rc2014_noui.o z180_io.o i82c55a.o ide.o sdcard.o z80dis.o libz180/libz180.o lib765/lib/lib765.a
	cc -g3 z180-mini-itx.o rc2014_noui.o z180_io.o i82c55a.o ide.o sdcard.o z80dis.o libz180/libz180.o lib765/lib/lib765.a -o z180-mini-itx

z180-mini-itx_sdl2: z180-mini-itx.o rc2014_sdlui.o z180_io.o i82c55a.o ide.o keymatrix.o ps2.o sdcard.o z80dis.o zxkey_sdl2.o libz180/libz180.o lib765/lib/lib765.a
	cc -g3 z180-mini-itx.o rc2014_sdlui.o z180_io.o i82c55a.o ide.o keymatrix.o ps2.o sdcard.o z80dis.o zxkey_sdl2.o libz180/libz180.o lib765/lib/lib765.a -lSDL2  -o z180-mini-itx_sdl2

flexbox: flexbox.o 6800.o acia.o ide.o
	cc -g3 flexbox.o 6800.o acia.o ide.o -o flexbox

simple80: simple80.o ide.o rtc_bitbang.o libz80/libz80.o z80dis.o
	cc -g3 simple80.o ide.o rtc_bitbang.o libz80/libz80.o z80dis.o -o simple80

zsc: zsc.o ide.o acia.o libz80/libz80.o
	cc -g3 zsc.o acia.o ide.o libz80/libz80.o -o zsc

nc100: nc100.o keymatrix.o libz80/libz80.o z80dis.o
	cc -g3 nc100.o keymatrix.o libz80/libz80.o z80dis.o -o nc100 -lSDL2

nc200: nc200.o keymatrix.o libz80/libz80.o z80dis.o lib765/lib/lib765.a
	cc -g3 nc200.o keymatrix.o libz80/libz80.o z80dis.o lib765/lib/lib765.a -o nc200 -lSDL2

markiv:	markiv.o z180_io.o ide.o rtc_bitbang.o propio.o sdcard.o z80dis.o libz180/libz180.o
	cc -g3 markiv.o z180_io.o ide.o rtc_bitbang.o propio.o sdcard.o z80dis.o libz180/libz180.o -o markiv

n8_sdl2: n8.o n8_sdlui.o z180_io.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_sdl2.o z80dis.o libz180/libz180.o lib765/lib/lib765.a
	cc -g3 n8.o n8_sdlui.o z180_io.o ide.o ppide.o ps2.o rtc_bitbang.o sdcard.o tms9918a.o tms9918a_sdl2.o z80dis.o libz180/libz180.o lib765/lib/lib765.a  -o n8_sdl2 -lSDL2

s100-z80:	s100-z80.o acia.o ppide.o ide.o libz80/libz80.o
	cc -g3 s100-z80.o acia.o ppide.o ide.o libz80/libz80.o -o s100-z80

mini11: mini11.o 68hc11.o sdcard.o
	cc -g3 mini11.o sdcard.o 68hc11.o -o mini11

scelbi: scelbi.o i8008.o dgvideo.o dgvideo_norender.o scopewriter.o scopewriter_norender.o asciikbd_none.o
	cc -g3 scelbi.o i8008.o dgvideo.o dgvideo_norender.o scopewriter.o scopewriter_norender.o asciikbd_none.o -o scelbi

scelbi_sdl2: scelbi.o i8008.o dgvideo.o dgvideo_sdl2.o scopewriter.o scopewriter_sdl2.o asciikbd_sdl2.o
	cc -g3 scelbi.o i8008.o dgvideo.o dgvideo_sdl2.o scopewriter.o scopewriter_sdl2.o asciikbd_sdl2.o -o scelbi_sdl2 -lSDL2

nascom: nascom.o keymatrix.o 58174.o libz80/libz80.o z80dis.o wd17xx.o sasi.o
	cc -g3 nascom.o keymatrix.o 58174.o sasi.o wd17xx.o libz80/libz80.o z80dis.o -lSDL2 -o nascom

uk101: uk101.o keymatrix.o acia.o 6502.o 6502dis.o
	cc -g3 uk101.o keymatrix.o acia.o 6502.o 6502dis.o -lSDL2 -o uk101

vz300: vz300.o 6847.o 6847_sdl2.o keymatrix.o sdcard.o libz80/libz80.o z80dis.o
	cc -g3 vz300.o 6847.o 6847_sdl2.o keymatrix.o sdcard.o libz80/libz80.o z80dis.o -lSDL2 -o vz300

rhyophyre:rhyophyre.o z180_io.o ppide.o ide.o rtc_bitbang.o z80dis.o libz180/libz180.o
	cc -g3 rhyophyre.o z180_io.o ppide.o ide.o rtc_bitbang.o z80dis.o libz180/libz180.o -o rhyophyre

pz1: pz1.o 6502.o 6502dis.o
	cc -g3 pz1.o 6502.o 6502dis.o -o pz1

nabupc: nabupc.o nabupc_noui.o ide.o tms9918a.o tms9918a_norender.o z80dis.o libz80/libz80.o
	cc -g3 nabupc.o nabupc_noui.o z80dis.o ide.o tms9918a.o tms9918a_norender.o libz80/libz80.o -o nabupc

nabupc_sdl2: nabupc.o nabupc_sdlui.o ide.o tms9918a.o tms9918a_sdl2.o z80dis.o libz80/libz80.o
	cc -g3 nabupc.o nabupc_sdlui.o z80dis.o ide.o tms9918a.o tms9918a_sdl2.o libz80/libz80.o -o nabupc_sdl2 -lSDL2

68hc11.o: 6800.c

makedisk: makedisk.o ide.o
	cc -O3 -o makedisk makedisk.o ide.o

clean:
	$(MAKE) --directory libz80 clean && \
	$(MAKE) --directory libz180 clean && \
	$(MAKE) --directory lib765/lib clean && \
	$(MAKE) --directory 80x86 clean && \
	$(MAKE) --directory lib65c816 clean && \
	$(MAKE) --directory m68k clean && \
	$(MAKE) --directory am9511 clean && \
	$(MAKE) --directory ns32k clean && \
	rm -f *.o *~ $(TARGETS)

SRCS := $(subst ./,,$(shell find . -name '*.c'))
DEPDIR := .deps
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.d

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c

%.o : %.c
%.o : %.c $(DEPDIR)/%.d | $(DEPDIR)
	$(COMPILE.c) $(OUTPUT_OPTION) $<

$(DEPDIR): ; @mkdir -p $@

DEPFILES := $(SRCS:%.c=$(DEPDIR)/%.d)
$(DEPFILES):

lib765/lib/lib765.a: lib765/lib/765drive.c lib765/lib/765dsk.c \
		     lib765/lib/765fdc.c lib765/lib/765i.h \
		     lib765/lib/765ldsk.c lib765/lib/error.c
libz80/libz80.o: libz80/z80.c libz80/z80.h
libz280/libz80.o: libz280/z80.c libz280/z80.h
cpu.c: lib65816/config.h

include $(wildcard $(DEPFILES))
