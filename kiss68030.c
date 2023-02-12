/*
 * 2023-02-04 Will Sowerbutts <will@sowerbutts.com>
 *
 * Based on Alan Cox's mini68k.c
 *
 *	John Coffman's
 *	RBC KISS-68030 + MF/PIC board with PPIDE
 *
 *	68030 CPU @32MHz 0 or 1 ws I/O 1 or 2 ws
 *	NS32202 interrupt controller
 *	up to 256MB RAM
 *	512KB flash ROM
 *	Autvectored interrupts off the MF/PIC
 *
 *	Mapping
 *	0000 0000 - 1000 0000   256MB DRAM
 *      FFF0 0000 - FFF7 FFFF   512KB ROM
 *      FFF8 0000 - FFFB FFFF   256KB ECB memory (not implemented by the emulator)
 *      FFFE 0000 - FFFE FFFF   32KB SRAM mapped twice to 64KB
 *      FFFF 0000 - FFFF FFFF   64KB ECB I/O space (16 bit addressing)
 *
 *	I/O space on the ECB
 *	0x40	MF/PIC board base
 *		0x40	32202
 *		0x42	cfg
 *		0x43	rtc
 *		0x44	PPI     PPIDE
 *		0x48	sio	16x50 UART
 *	0x2x	DiskIO PPIDE
 *	0x3x	DiskIO Floppy
 *	0x00	4Mem
 *	0x08	DualSD
 *
 *	TODO
 *	- make take_a_nap smarter (measure wall clock time elapsed)
 *	- share ns202 and other mf/pic code with mini68k?
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <m68k.h>
#include <m68kcpu.h>
#include <endian.h>
#include "16x50.h"
#include "ppide.h"
#include "rtc_bitbang.h"
#include "sdcard.h"
#include "ns202.h"
#include "lib765/include/765.h"

#define NVRAM_FILENAME "kiss68030.nvram"

/* CPU */
extern m68ki_cpu_core m68ki_cpu;

/* IDE controller */
static struct ppide *ppide;	/* MFPIC */
static struct ppide *ppide2;	/* DiskIO */

/* SD */
static struct sdcard *sd[2];	/* Dual SD */

/* Serial */
static struct uart16x50 *uart;  /* console UART */

/* RTC */
static struct rtc *rtc;
static unsigned rtc_loaded;

/* FDC on DiskIO */
static FDC_PTR fdc;
static FDRV_PTR drive_a, drive_b;

/* up to 256MB DRAM */
#define MAXDRAMMB 256
static uint8_t ram[MAXDRAMMB << 20];
static uint32_t memsize;
static uint32_t fast_memsize = 0; /* this is 0 until u304 is 0xFF, then it is equal to memsize */

/* 32KB SRAM */
static uint8_t sram[32 << 10];

/* fixed 512K ROM */
#define ROMSIZE (512 << 10)
static uint8_t rom[ROMSIZE];

/* Config register on the MFPIC */
static uint8_t mfpic_cfg;

static int trace = 0, trace_alt = 0;

#define TRACE_MEM	1
#define TRACE_CPU	2
#define TRACE_UART	4
#define TRACE_PPIDE	8
#define TRACE_RTC	16
#define TRACE_FDC	32
#define TRACE_NS202	64
#define TRACE_SD	128

uint8_t fc;

/* Force ROM into low space for the first 8 reads */
static uint8_t u304 = 0;

/* U304 on the KISS-68030 is an 8-bit shift register that starts
 * as all 0s and fills with 1s from the LSB. All memory access is
 * directed to ROM while the MSB is 0. */
void u304_reset(void)
{
    u304 = 0;
    fast_memsize = 0;
}

void u304_clk(void)
{
    if(u304 == 0xFF)
        return;
    u304 = (u304 << 1) | 1;
    if(u304 == 0xFF){ /* finished: safe to enable fast 32-bit RAM access */
        fast_memsize = memsize;
    }
}

#if 0
/* dumps memory as viewed by CPU -- MMU translation, S-bit in SR all taken into account */
static void hexdump_cpu_memory(uint32_t base, uint32_t count)
{
    m68ki_cpu_core *state = &m68ki_cpu;
    int i,j;
    unsigned char c;
    for (i = 0; i < count; i+= 16) {
        fprintf(stderr, "VIRT %08X | ", base+i);
        for(j = 0; j < 16; j++)
            fprintf(stderr, "%02X ", m68ki_read_8(state, base+i+j));
        fprintf(stderr, "|");
        for(j = 0; j < 16; j++){
            c = m68ki_read_8(state, base+i+j);
            fprintf(stderr, "%c", (c > 32 && c < 128) ? c : ' ');
        }
        fprintf(stderr, "\n");
    }
}

static void hexdump_phys_memory(uint32_t base, uint32_t count)
{
    int i,j;
    unsigned char c;

    for (i = 0; i < count; i+= 16) {
        fprintf(stderr, "PHYS %08X | ", base+i);
        for(j = 0; j < 16; j++)
            fprintf(stderr, "%02X ", ram[base+i+j]);
        fprintf(stderr, "|");
        for(j = 0; j < 16; j++){
            c = ram[base+i+j];
            fprintf(stderr, "%c", (c > 32 && c < 128) ? c : ' ');
        }
        fprintf(stderr, "\n");
    }
}
#endif

unsigned int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, &o, NULL, &tv) == -1) {
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

void cpu_dump_regs(void)
{
    unsigned int d0   = m68k_get_reg(NULL, M68K_REG_D0);
    unsigned int d1   = m68k_get_reg(NULL, M68K_REG_D1);
    unsigned int d2   = m68k_get_reg(NULL, M68K_REG_D2);
    unsigned int d3   = m68k_get_reg(NULL, M68K_REG_D3);
    unsigned int d4   = m68k_get_reg(NULL, M68K_REG_D4);
    unsigned int d5   = m68k_get_reg(NULL, M68K_REG_D5);
    unsigned int d6   = m68k_get_reg(NULL, M68K_REG_D6);
    unsigned int d7   = m68k_get_reg(NULL, M68K_REG_D7);
    unsigned int a0   = m68k_get_reg(NULL, M68K_REG_A0);
    unsigned int a1   = m68k_get_reg(NULL, M68K_REG_A1);
    unsigned int a2   = m68k_get_reg(NULL, M68K_REG_A2);
    unsigned int a3   = m68k_get_reg(NULL, M68K_REG_A3);
    unsigned int a4   = m68k_get_reg(NULL, M68K_REG_A4);
    unsigned int a5   = m68k_get_reg(NULL, M68K_REG_A5);
    unsigned int a6   = m68k_get_reg(NULL, M68K_REG_A6);
    unsigned int a7   = m68k_get_reg(NULL, M68K_REG_A7);
    unsigned int pc   = m68k_get_reg(NULL, M68K_REG_PC);
    unsigned int sr   = m68k_get_reg(NULL, M68K_REG_SR);
    unsigned int sp   = m68k_get_reg(NULL, M68K_REG_SP);
    unsigned int usp  = m68k_get_reg(NULL, M68K_REG_USP);
    unsigned int isp  = m68k_get_reg(NULL, M68K_REG_ISP);
    unsigned int msp  = m68k_get_reg(NULL, M68K_REG_MSP);
    unsigned int sfc  = m68k_get_reg(NULL, M68K_REG_SFC);
    unsigned int dfc  = m68k_get_reg(NULL, M68K_REG_DFC);
    unsigned int vbr  = m68k_get_reg(NULL, M68K_REG_VBR);
    unsigned int cacr = m68k_get_reg(NULL, M68K_REG_CACR);
    unsigned int caar = m68k_get_reg(NULL, M68K_REG_CAAR);

    fprintf(stderr, "D0=%08x\tD4=%08x\tA0=%08x\tA4=%08x\n", d0, d4, a0, a4);
    fprintf(stderr, "D1=%08x\tD5=%08x\tA1=%08x\tA5=%08x\n", d1, d5, a1, a5);
    fprintf(stderr, "D2=%08x\tD6=%08x\tA2=%08x\tA6=%08x\n", d2, d6, a2, a6);
    fprintf(stderr, "D3=%08x\tD7=%08x\tA3=%08x\tA7=%08x\n", d3, d7, a3, a7);
    fprintf(stderr, "PC=%08x\tSR=%08x\tSP=%08x\tUSP=%08x\n", pc, sr, sp, usp);
    fprintf(stderr, "ISP=%08x\tMSP=%08x\tSFC=%08x\tDFC=%08x\n", isp, msp, sfc, dfc);
    fprintf(stderr, "VBR=%08x\tCACR=%08x\tCAAR=%08x\n\n", vbr, cacr, caar);
}

#undef WARN_ON_READ_BEFORE_READY
unsigned int next_char(void)
{
	char c;

        if(!(check_chario() & 1)){ /* not ready */
#ifdef WARN_ON_READ_BEFORE_READY
            unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
            fprintf(stderr, "(tty read before ready, ++PC=%08x)\n", pc);
            cpu_dump_regs();
#endif
            return 0xFF;
        }

	if (read(0, &c, 1) != 1) {
		fprintf(stderr, "(tty read error)\n");
		return 0xFF;
	}
	return c;
}
	
static unsigned int cfg_read(void)
{
	return mfpic_cfg;
}

static void cfg_write(unsigned int value)
{
	/* 7-3 user */
	/* Bit 2 masks upper 8 interrupts */
	/* 1:0 shift value for interrupt vector */
	mfpic_cfg = value;
}

/* Remap the bits as the MF/PIC doesn't follow the usual RBC/RC2014 mapping */

static unsigned rtc_remap_w(unsigned v)
{
	unsigned r = 0;
	if (v & 1)		/* Data / Data */
		r |= 0x80;
	if (!(v & 2))		/* Write / /Write */
		r |= 0x20;
	if (v & 4)		/* Clock / Clock */
		r |= 0x40;
	if (!(v & 8))		/* Reset / /Reset */
		r |= 0x10;
	return r;
}

static unsigned rtc_remap_r(unsigned v)
{
	unsigned r = 0;
	if (v & 0x01)		/* Data in */
		r |= 0x01;
	return r;
}

/*
 *	Dual SD. Simple bitbang SD card. Nothing fancy
 *	not even a write/read driven clock
 *
 *	Interrupt not currently emulated. Currently we hardcode
 *	a card in slot 0 and none in slot 1
 */

static uint8_t dsd_op;
static uint8_t dsd_sel = 0x10;
static uint8_t dsd_sel_w;
static uint8_t dsd_rx, dsd_tx;
static uint8_t dsd_bitcnt;
static uint8_t dsd_bit;

/* Rising edge: Value sampling */
static void dualsd_clock_high(void)
{
	dsd_rx <<= 1;
	dsd_rx |= dsd_op & 1;
	dsd_bitcnt++;
	if (dsd_bitcnt == 8) {
		dsd_tx = sd_spi_in(sd[0], dsd_rx);
		if (trace & TRACE_SD)
			fprintf(stderr, "sd: sent %02X got %02x\n", dsd_rx, dsd_tx);
		dsd_bitcnt = 0;
	}
}

/* Falling edge: Values change */
static void dualsd_clock_low(void)
{
	dsd_bit = (dsd_tx & 0x80) ? 1: 0;
	dsd_tx <<= 1;
}

static void dualsd_write(unsigned addr, unsigned val)
{
	static unsigned delta;
	if (sd[0] == NULL)
		return;
	if (trace & TRACE_SD)
		fprintf(stderr, "dsd_write %d %02X\n", addr & 1, val);
	if (addr & 1) {
		dsd_sel_w = val;
		dsd_sel &= 0xFE;
		dsd_sel |= val & 0x01;
		/* IRQ logic not yet emulated TODO */
	} else {
		delta = dsd_op ^ val;
		dsd_op = val;
		/* Only doing card 0 for now */
		if ((dsd_sel & 1) == 0) {
			if (delta & 0x04) {
				if (val & 0x04) {
					sd_spi_lower_cs(sd[0]);
					dsd_bitcnt = 0;
				} else
					sd_spi_raise_cs(sd[0]);
			}
			if (delta & 0x02) {
				if (val & 0x02)
					dualsd_clock_high();
				else
					dualsd_clock_low();
			}
		}
	}
}

static unsigned do_dualsd_read(unsigned addr)
{
	if (sd[0] == NULL)
		return 0xFF;
	if (addr & 1)
		return dsd_sel;
	else {
		/* For now we just fake card 1 absent, 0 rw present */
		if (dsd_sel & 1)
			return (dsd_op & 0x06);
		else
			return (dsd_op & 0x06) | 0x20 | dsd_bit;
	}
}

static unsigned dualsd_read(unsigned addr)
{
	unsigned val = do_dualsd_read(addr);
	if (trace & TRACE_SD)
		fprintf(stderr, "dsd_read %d %02X\n", addr & 1, val);
	return val;
}


/* FDC: TC not connected ? */
static void fdc_log(int debuglevel, char *fmt, va_list ap)
{
	if ((trace & TRACE_FDC) || debuglevel == 0)
		vfprintf(stderr, "fdc: ", ap);
}

static void fdc_write(uint8_t addr, uint8_t val)
{
	if (addr & 0x08)
		fdc_write_dor(fdc, val);
	else if (addr & 1)
		fdc_write_data(fdc, val);
}

static uint8_t fdc_read(uint8_t addr)
{
	if (addr & 0x08)
		return fdc_read_dir(fdc);
	else if (addr & 1)
		return fdc_read_data(fdc);
	else
		return fdc_read_ctrl(fdc);
}

void uart16x50_signal_change(struct uart16x50 *uart, uint8_t mcr)
{
	/* Modem lines changed - don't care */
}

/* Read data from RAM, ROM, or a device */
unsigned int do_cpu_read_byte(unsigned int address, unsigned debug)
{
    if (!(u304 & 0x80)) {
        if (debug == 0) {
            u304_clk();
        }
        return rom[address & (ROMSIZE-1)];
    }
    if (debug == 0) {
        u304_clk();
    }

    if (address < memsize)
        return ram[address];
    if (address < 0xFFF00000){ /* unmapped */
        if((memsize <= (64<<20)  && address >= (64<<20)) ||
           (memsize <= (256<<20) && address >= (256<<20))){
            unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
            fprintf(stderr, "(bus error: read at %08x, memsize=%08x, ++PC=%08x)\n", address, memsize, pc);
            //cpu_dump_regs();
            m68ki_exception_trap(&m68ki_cpu, EXCEPTION_BUS_ERROR);
        }
        return 0xFF;
    }
    if (address < 0xFFF80000) /* ROM! Everyone loves ROM */
        return rom[address & (ROMSIZE-1)];
    if (address < 0xFFFE0000) /* ECB memory: not implemented */
        return 0xFF;
    if (address < 0xFFFF0000) /* SRAM - 32KB mapped twice */
        return sram[address & 0x7FFF];

    /* I/O space */
    /* Disassembler doesn't trigger I/O side effects */
    if (debug)
        return 0xFF;

    address &= 0xFFFF;

    if ((address & 0xF0) == 0x20)
        return ppide_read(ppide2, address & 0x03);
    if ((address & 0xF0) == 0x30)
        return fdc_read(address);
    switch(address & 0xFF) {
        case 0x08:
        case 0x09:
            return dualsd_read(address);
        case 0x40:
            return ns202_read(address);
        case 0x42:
            return cfg_read();
        case 0x43:
            return rtc_remap_r(rtc_read(rtc));
        case 0x44:
        case 0x45:
        case 0x46:
        case 0x47:
            return ppide_read(ppide, address & 0x03);
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        case 0x4E:
        case 0x4F:
            return uart16x50_read(uart, address & 0x07);
    }
    return 0xFF;
}

unsigned int cpu_read_byte(unsigned int address)
{
    unsigned int v;
    if(address < fast_memsize)
        v = ram[address];
    else
        v = do_cpu_read_byte(address, 0);
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X:%02X}", address, v);
    return v;
}

unsigned int do_cpu_read_word(unsigned int address, unsigned int debug)
{
    return (do_cpu_read_byte(address, debug) << 8) | do_cpu_read_byte(address + 1, debug);
}

unsigned int cpu_read_word(unsigned int address)
{
    unsigned int v;
    if(address < fast_memsize)
        v = be16toh(*((uint16_t*)&ram[address]));
    else
        v = do_cpu_read_word(address, 0);
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X:%04X}", address, v);
    return v;
}

unsigned int cpu_read_word_dasm(unsigned int address)
{
    return do_cpu_read_word(address, 1);
}

unsigned int cpu_read_long(unsigned int address)
{
    unsigned int v;
    if(address < fast_memsize)
        v = be32toh(*((uint32_t*)&ram[address]));
    else
        v = (cpu_read_word(address) << 16) | cpu_read_word(address + 2);
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X:%08X}", address, v);
    return v;
}

unsigned int cpu_read_long_dasm(unsigned int address)
{
    return (cpu_read_word_dasm(address) << 16) | cpu_read_word_dasm(address + 2);
}

extern int trace_opcode_fetch;

void cpu_write_byte(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%02X}", address, value);
    if(address < fast_memsize) {
        ram[address] = value;
        return;
    }
    if (!(u304 & 0x80)) {
        u304_clk();
        return;
    }
    u304_clk();

    if (address < memsize) {
        ram[address] = value;
        return;
    }
    if(address == 0xFFFFEEEE){
        fprintf(stderr, "magic trace write: trace=0x%02x\n", value);
        trace = TRACE_CPU | TRACE_MEM;
        trace_opcode_fetch = 1;
        return;
    }
    if(address == 0xFFFFEEEF){
        fprintf(stderr, "magic trace write: signal=0x%02x\n", value);
        trace = value;
        return;
    }
    if (address < 0xFFF00000){ /* unmapped */
        if (trace & TRACE_MEM)
            fprintf(stderr,  "%08x: write to invalid space.\n", address);
        if((memsize <= (64<<20)  && address >= (64<<20)) ||
           (memsize <= (256<<20) && address >= (256<<20))){
            unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
            fprintf(stderr, "(bus error: write at %08x, memsize=%08x, ++PC=%08x)\n", address, memsize, pc);
            //cpu_dump_regs();
            m68ki_exception_trap(&m68ki_cpu, EXCEPTION_BUS_ERROR);
        }
        return;
    }
    if (address < 0xFFF80000){ /* ROM */
        if (trace & TRACE_MEM)
            fprintf(stderr,  "%08x: write to ROM.\n", address);
        return;
    }
    if (address < 0xFFFE0000){ /* ECB memory (unimplemented) */
        if (trace & TRACE_MEM)
            fprintf(stderr,  "%08x: write to ECB RAM.\n", address);
        return;
    }
    if (address < 0xFFFF0000){ /* SRAM - 32KB mapped twice */
        sram[address & 0x7FFF] = value;
        return;
    }

    if ((address & 0xF0) == 0x20) {
        ppide_write(ppide2, address & 0x03, value);
        return;
    }
    if ((address & 0xF0) == 0x30) {
        fdc_write(address, value);
        return;
    }
    switch(address & 0xFF) {
        /* DualSD */
        case 0x08:
        case 0x09:
            dualsd_write(address, value);
            return;
            /* MFPIC */
        case 0x40:
            ns202_write(address & 0xFFFF, value);
            return;
        case 0x42:
            cfg_write(value);
            return;
        case 0x43:
            rtc_write(rtc, rtc_remap_w(value));
            return;
        case 0x44:
        case 0x45:
        case 0x46:
        case 0x47:
            ppide_write(ppide, address & 0x03, value);
            return;
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0x4B:
        case 0x4C:
        case 0x4D:
        case 0x4E:
        case 0x4F:
            uart16x50_write(uart, address & 0x07, value);
            return;
    }
}

void cpu_write_word(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%04X}", address, value);
    if(address < fast_memsize) {
        *((uint16_t*)&ram[address]) = htobe16(value);
        return;
    } else {
        cpu_write_byte(address, value >> 8);
        cpu_write_byte(address + 1, value & 0xFF);
    }
}

void cpu_write_long(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%08X}", address, value);
    if(address < fast_memsize) {
        *((uint32_t*)&ram[address]) = htobe32(value);
    } else {
	cpu_write_word(address, value >> 16);
	cpu_write_word(address + 2, value & 0xFFFF);
    }
}

void cpu_write_pd(unsigned int address, unsigned int value)
{
    cpu_write_word(address + 2, value & 0xFFFF);
    cpu_write_word(address, value >> 16);
}

int cpu_illegal_instuction(int opcode)
{
    unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
    fprintf(stderr, "KISS SAYS ILLEGAL OPCODE %x, PC=%x\n", opcode, pc);
    cpu_dump_regs();

    raise(SIGTRAP);
    // sleep(1);

    return 0; /* return nonzero to skip normal exception processing */
}

void cpu_instr_callback(void)
{
    if (trace & TRACE_CPU) {
        int len, i;
        char buf[128];
        unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);

        m68k_disassemble_il(buf, pc, M68K_CPU_TYPE_68030, &len);
        fprintf(stderr, "\n>%08X| ", pc);
        for(i=0; i<len; i++)
            fprintf(stderr, "%04x ", cpu_read_word_dasm(pc+i));
        for(; i<6; i++)
            fprintf(stderr, "     ");
        fprintf(stderr, "%-40s", buf);
    }
}

static void device_init(void)
{
    u304_reset();
    ns202_reset();
    ppide_reset(ppide);
    uart16x50_reset(uart);
    uart16x50_set_input(uart, 1);
    uart16x50_signal_event(uart, 0x10); /* mini68K ROM wants the CTS bit asserted */
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
    tcsetattr(0, 0, &saved_term);
    if (rtc_loaded)
        rtc_save(rtc, NVRAM_FILENAME);
    exit(1);
}

static void exit_cleanup(void)
{
    if (rtc_loaded)
        rtc_save(rtc, NVRAM_FILENAME);
    tcsetattr(0, 0, &saved_term);
}

int need_dump_state = 0;
static void sig_dump_state(int sig)
{
    /* minimal signal handler */
    need_dump_state = 1;
}

static void do_dump_state(void)
{
    need_dump_state = 0;
    
    fprintf(stderr, "\n");
    ns202_dump_state();
    uart16x50_dump_state(uart);
    cpu_dump_regs();
}

int need_toggle_trace = 0;
static void sig_toggle_trace(int sig)
{
    /* minimal signal handler */
    need_toggle_trace = 1;
}

static void do_toggle_trace(void)
{
    int s;

    need_toggle_trace = 0;
    s = trace;
    trace = trace_alt;
    trace_alt = s;
    fprintf(stderr, "[trace %d]", trace);
    uart16x50_trace(uart, trace & TRACE_UART ? 1 : 0);
    ns202_trace(trace & TRACE_NS202 ? 1 : 0);
}

static void take_a_nap(void)
{
    /* WRS: could be smarter here - measure time since we last slept? */
    struct timespec t;
    t.tv_sec = 0;
    t.tv_nsec = 100000;
    if (nanosleep(&t, NULL))
        perror("nanosleep");
}

int cpu_irq_ack(int level)
{
    uint8_t v=0, i;

    i = ns202_int_ack();
    if(!ns202_irq_asserted())
        m68k_set_irq(M68K_IRQ_NONE); /* clear IRQ */

    /*
    ** computing the vector:
    **            bit7   bit6   bit5   bit4   bit3   bit2   bit1   bit0
    ** mfpic_cfg  VEC7   VEC6   VEC5   VEC4   VEC3   OPT168 S0     S1
    ** S1=0,S0=0| VEC7   VEC6   VEC5   VEC4   IID3V3 IID2   IID1   IID0
    ** S1=0,S0=1| VEC7   VEC6   VEC5   IID3V4 IID2   IID1   IID0   VEC3
    ** S1=1,S0=0| VEC7   VEC6   VEC5   IID2   IID1   IID0   VEC4   VEC3
    ** S1=1,S0=1| VEC7   VEC6   IID2   IID1   IID0   VEC5   VEC4   VEC3
    **
    ** IID3V4 = OPT168 ? IID3 : VEC4
    ** IID3V3 = OPT168 ? IID3 : VEC3
    */
    switch(mfpic_cfg & 3){ /* shift bits S0, S1 */
        case 0:
            if(mfpic_cfg & 4){ /* OPT168 = 1: 16 interrupts */
                i &= 0x0F;
                v = (mfpic_cfg & 0xF0) | i;
            }else{ /* OPT168 = 0: 8 interrupts */
                i &= 0x07;
                v = (mfpic_cfg & 0xF8) | i;
            }
            break;
        case 1:
            if(mfpic_cfg & 4){ /* OPT168 = 1: 16 interrupts */
                i &= 0x0F;
                v = (mfpic_cfg & 0xE0) | (i << 1) | ((mfpic_cfg & 0x08) >> 3);
            }else{ /* OPT168 = 0: 8 interrupts */
                i &= 0x07;
                v = (mfpic_cfg & 0xF0) | (i << 1) | ((mfpic_cfg & 0x08) >> 3);
            }
            break;
        case 2:
            i &= 0x08;
            v = (mfpic_cfg & 0xE0) | (i << 2) | ((mfpic_cfg & 0x18) >> 3);
            break;
        case 3:
            i &= 0x08;
            v = (mfpic_cfg & 0xC0) | (i << 3) | ((mfpic_cfg & 0x38) >> 3);
            break;
    }

    /*
    if (trace & TRACE_NS202)
        fprintf(stderr, "68K interrupt ACK: vector %02X\n", v);
    */

    return v;
}

void recalc_interrupts(void)
{
    /* OPT16/8=0: UART on IRQ 4, OPT16/8=1: UART on IRQ 12 */
    if(uart16x50_irq_pending(uart))
        ns202_raise(mfpic_cfg & 4 ? 12 : 4);

    if(ppide->ide->drive[0].intrq || ppide->ide->drive[1].intrq)
        ns202_raise(9);

    /* NS202 signals CPU interrupt with IPL0=0, IPL1=1, IPL2=0 */
    if(ns202_irq_asserted())
        m68k_set_irq(M68K_IRQ_2);
}

/* called by the 68K emulator when the CPU encounters a RESET instruction */
void cpu_pulse_reset(void)
{
    device_init();
}

void cpu_set_fc(int fc)
{
}

void usage(const char *name)
{
    fprintf(stderr, "%s: [-m memsize][-r rompath][-i idepath][-I idepath][-s sdpath][-S sdpath][-d debug][-D debug].\n", name);
    fprintf(stderr, "memsize is in megabytes\n");
    fprintf(stderr, "debug is a bitwise OR combination of the following:\n");
    fprintf(stderr, "%5d   MEM\n",   TRACE_MEM);
    fprintf(stderr, "%5d   CPU\n",   TRACE_CPU);
    fprintf(stderr, "%5d   UART\n",  TRACE_UART);
    fprintf(stderr, "%5d   PPIDE\n", TRACE_PPIDE);
    fprintf(stderr, "%5d   RTC\n",   TRACE_RTC);
    fprintf(stderr, "%5d   FDC\n",   TRACE_FDC);
    fprintf(stderr, "%5d   NS202\n", TRACE_NS202);
    fprintf(stderr, "%5d   SD\n",    TRACE_SD);
    exit(1);
}

int main(int argc, char *argv[])
{
    int fd;
    int fast = 0;
    int opt;
    int memsize_mb = MAXDRAMMB;
    const char *romname = "kiss68030.rom";
    const char *diskname = NULL;
    const char *diskname2 = NULL;
    const char *patha = NULL;
    const char *pathb = NULL;
    const char *sdname = NULL;
    const char *sdname2 = NULL;

    while((opt = getopt(argc, argv, "d:fi:m:r:s:A:B:I:S:D:")) != -1) {
        switch(opt) {
            case 'd':
                trace = atoi(optarg);
                break;
            case 'D':
                trace_alt = atoi(optarg);
                break;
            case 'f':
                fast = 1;
                break;
            case 'i':
                diskname = optarg;
                break;
            case 'm':
                memsize_mb = atoi(optarg);
                break;
            case 'r':
                romname = optarg;
                break;
            case 's':
                sdname = optarg;
                break;
            case 'S':
                sdname2 = optarg;
                break;
            case 'A':
                patha = optarg;
                break;
            case 'B':
                pathb = optarg;
                break;
            case 'I':
                diskname2 = optarg;
                break;
            default:
                usage(argv[0]);
        }
    }

    if (tcgetattr(0, &term) == 0) {
        saved_term = term;
        atexit(exit_cleanup);
        signal(SIGINT, SIG_IGN);
        signal(SIGQUIT, cleanup);
        signal(SIGTSTP, SIG_IGN);
        signal(SIGUSR1, sig_toggle_trace);
        signal(SIGUSR2, sig_dump_state);
        term.c_lflag &= ~ICANON;
        term.c_iflag &= ~(ICRNL | IGNCR);
        term.c_cc[VMIN] = 1;
        term.c_cc[VTIME] = 0;
        term.c_cc[VINTR] = 0;
        term.c_cc[VSUSP] = 0;
        term.c_cc[VEOF] = 0;
        term.c_lflag &= ~(ECHO | ECHOE | ECHOK);
        tcsetattr(0, 0, &term);
    }

    if (optind < argc)
        usage(argv[0]);

    if(memsize_mb < 1){
        fprintf(stderr, "%s: RAM size must be at least 1MB\n", argv[0]);
        exit(1);
    }

    memsize = memsize_mb << 20; /* convert MB to bytes */

    if(memsize > sizeof(ram)) {
        fprintf(stderr, "%s: RAM size must be no more than %ldMB\n",
                argv[0], (long)(sizeof(ram) >> 20));
        exit(1);
    }
    /* WRS: could use sizeof(ram) here but that causes us to touch memory that
     * we then never touch again, causing the emulator to hog more RAM than it 
     * actually needs? */
    memset(ram, 0xA7, memsize);
    memset(sram, 0xA7, sizeof(sram));

    fd = open(romname, O_RDONLY);
    if (fd == -1) {
        perror(romname);
        exit(1);
    }
    if (read(fd, rom, sizeof(rom)) != sizeof(rom)) {
        fprintf(stderr, "%s: too short. Must be %ldKiB.\n", romname, (long)sizeof(rom) >> 10);
        exit(1);
    }
    close(fd);

    ppide = ppide_create("hd0");
    ppide_reset(ppide);
    if (diskname) {
        fd = open(diskname, O_RDWR);
        if (fd == -1) {
            perror(diskname);
            exit(1);
        }
        if (ppide == NULL)
            exit(1);
        if (ppide_attach(ppide, 0, fd))
            exit(1);
    }
    ppide_trace(ppide, trace & TRACE_PPIDE);

    ppide2 = ppide_create("hd1");
    ppide_reset(ppide2);
    if (diskname2) {
        fd = open(diskname2, O_RDWR);
        if (fd == -1) {
            perror(diskname2);
            exit(1);
        }
        if (ppide2 == NULL)
            exit(1);
        if (ppide_attach(ppide2, 0, fd))
            exit(1);
    }
    ppide_trace(ppide2, trace & TRACE_PPIDE);

    sd[0] = sd_create("sd0");
    sd[1] = sd_create("sd1");
    sd_reset(sd[0]);
    sd_reset(sd[1]);
    if (sdname) {
        fd = open(sdname, O_RDWR);
        if (fd == -1) {
            perror(sdname);
            exit(1);
        }
        sd_attach(sd[0], fd);
    }
    if (sdname2) {
        fd = open(sdname2, O_RDWR);
        if (fd == -1) {
            perror(sdname2);
            exit(1);
        }
        sd_attach(sd[1], fd);
    }
    sd_trace(sd[0], trace & TRACE_SD);
    sd_trace(sd[1], trace & TRACE_SD);

    uart = uart16x50_create();
    if (trace & TRACE_UART)
        uart16x50_trace(uart, 1);

    rtc = rtc_create();
    rtc_reset(rtc);
    rtc_trace(rtc, trace & TRACE_RTC);
    rtc_load(rtc, NVRAM_FILENAME);
    rtc_loaded = 1;

    fdc = fdc_new();

    lib765_register_error_function(fdc_log);

    if (patha) {
        drive_a = fd_newdsk();
        fd_settype(drive_a, FD_35);
        fd_setheads(drive_a, 2);
        fd_setcyls(drive_a, 80);
        fdd_setfilename(drive_a, patha);
    } else
        drive_a = fd_new();

    if (pathb) {
        drive_b = fd_newdsk();
        fd_settype(drive_a, FD_35);
        fd_setheads(drive_a, 2);
        fd_setcyls(drive_a, 80);
        fdd_setfilename(drive_a, pathb);
    } else
        drive_b = fd_new();

    fdc_reset(fdc);
    fdc_setisr(fdc, NULL);

    fdc_setdrive(fdc, 0, drive_a);
    fdc_setdrive(fdc, 1, drive_b);

    ns202_trace(trace & TRACE_NS202 ? 1 : 0);

    m68k_init();
    m68k_set_cpu_type(&m68ki_cpu, M68K_CPU_TYPE_68030);
    m68k_pulse_reset(&m68ki_cpu);

    /* Init devices */
    device_init();

    fprintf(stderr, "interrupts disabled during CPU trace - bodge\n");

    while (1) {
        if(need_toggle_trace)
            do_toggle_trace();
        if(need_dump_state)
            do_dump_state();
        m68k_execute(&m68ki_cpu, 3200); /* 32MHz target */
        uart16x50_event(uart);
        if(!(trace & TRACE_CPU))
            recalc_interrupts();
        /* NS202 is run off the MF-PIC UART clock */
        ns202_tick(184);
        if (!fast)
            take_a_nap();
    }
}
