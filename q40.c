/*
 * 2023-02-04 Will Sowerbutts <will@sowerbutts.com>
 *
 * Based on Alan Cox's mini68k.c
 *
 *	Q40
 *
 *	68040 CPU @40MHz
 *	up to 32MB main RAM
 *	1MB video RAM
 *	256KB flash ROM
 *	"MASTER" CPLD logic for interrupt and display control
 *
 *	Mapping
 *	0000 0000 - 0001 7FFF   96KB ROM (lower 96KB only)
 *      0001 8000 - 0001 FFFF   32KB RAM
 *      0002 0000 - 0002 7FFF   32KB video RAM (repeated at FE82 0000)
 *      0002 8000 - 01FF FFFF   ~32MB RAM
 *      FE00 0000 - FE03 FFFF   256KB ROM (full ROM)
 *      FE80 0000 - FE8F FFFF   1MB video RAM
 *      FF00 0000 - FF00 0034   Master chip (keyboard, interrupts, video, LED, ISA reset)
 *      FF00 8000 - FF00 8004   Audio DACs
 *      FF01 0000 - FF01 8000   Address decoder (ROM mode select)
 *      FF02 0000 - FF02 1FFC   RTC including 2KB NVRAM
 *      FF40 0000 - FF7F FFFF   ISA I/O
 *      FF80 0000 - FFBF FFFF   ISA memory
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
#include "ne2000.h"
#include "ide.h"
#include "16x50.h"

#define NVRAM_FILENAME "q40.nvram"

/* CPU */
extern m68ki_cpu_core m68ki_cpu;

/* Serial */
static struct uart16x50 *uart;  /* console UART */

/* DRAM */
#define MAXDRAMMB 32
static uint8_t ram[MAXDRAMMB << 20];
static uint8_t vram[1 << 20];
static uint32_t memsize;

/* ROM */
#define ROMSIZE (256 << 10)
static uint8_t rom[ROMSIZE];
int softrom = 0;

/* NVRAM */
#define NVRAMSIZE 2040
unsigned char nvram[NVRAMSIZE];

/* IDE controller */
static struct ide_controller *ide;

/* Q40 CPLD */
int q40_interrupts_enabled = 0;
int frame_interrupt_asserted = 0;
int frame_interrupt_rate_bit = 0;
int frame_interrupt_counter = 0;

/* NE2000 ISA card */
ne2000_t *ethdev;

static int trace = 0, trace_alt = 0;

#define TRACE_MEM	1
#define TRACE_CPU	2
#define TRACE_UART	4
#define TRACE_PPIDE	8
#define TRACE_RTC	16
#define TRACE_FDC	32
#define TRACE_NS202	64
#define TRACE_SD	128

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
#endif

#if 0
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

int need_dump_state = 0;
static void do_dump_state(void)
{
    need_dump_state = 0;
    
    fprintf(stderr, "\n");
    //ns202_dump_state();
    uart16x50_dump_state(uart);
    cpu_dump_regs();
}

void isa_mem_write_word(unsigned int address, unsigned int value)
{
    fprintf(stderr, "Unexpected ISA word write at 0x%08x\n", address);
}

unsigned int isa_mem_read_word(unsigned int address)
{
    fprintf(stderr, "Unexpected ISA word read at 0x%08x\n", address);
    return 0xffff;
}

void isa_mem_write_byte(unsigned int address, unsigned int value)
{
    fprintf(stderr, "Unexpected ISA byte write at 0x%08x\n", address);
}

unsigned int isa_mem_read_byte(unsigned int address)
{
    fprintf(stderr, "Unexpected ISA byte read at 0x%08x\n", address);
    return 0xff;
}

void isa_io_write_word(unsigned int address, unsigned int value)
{
    unsigned int isa_addr;
    if((address & 3) != 0){
        fprintf(stderr, "Bad ISA word write at 0x%08x (bad low address bits)\n", address);
    }

    isa_addr = (address - 0xff400000) / 4;

    switch(isa_addr){
        case 0x300: case 0x301: case 0x302: case 0x303: case 0x304: case 0x305: case 0x306: case 0x307:
        case 0x308: case 0x309: case 0x30a: case 0x30b: case 0x30c: case 0x30d: case 0x30e: case 0x30f:
            if(ethdev)
                ne2000_write(isa_addr, value, ethdev);
            break;
        case 0x310: case 0x311: case 0x312: case 0x313: case 0x314: case 0x315: case 0x316: case 0x317:
        case 0x318: case 0x319: case 0x31a: case 0x31b: case 0x31c: case 0x31d: case 0x31e: case 0x31f:
            if(ethdev)
                ne2000_asic_write_w(isa_addr, value, ethdev);
            break;
        case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3: case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
            ide_write16(ide, isa_addr & 0x07, value);
            break;
        case 0x3f6:
            ide_write16(ide, ide_altst_r, value);
            break;
        default:
            fprintf(stderr, "Unhandled ISA I/O word write port 0x%x\n", address);
    }
}

unsigned int isa_io_read_word(unsigned int address)
{
    unsigned int isa_addr;
    if((address & 3) != 0){
        fprintf(stderr, "Bad ISA word read at 0x%08x (bad low address bits)\n", address);
    }

    isa_addr = (address - 0xff400000) / 4;

    switch(isa_addr){
        case 0x300: case 0x301: case 0x302: case 0x303: case 0x304: case 0x305: case 0x306: case 0x307:
        case 0x308: case 0x309: case 0x30a: case 0x30b: case 0x30c: case 0x30d: case 0x30e: case 0x30f:
            if(ethdev)
                return ne2000_read(isa_addr, ethdev);
            else
                return 0xffff;
        case 0x310: case 0x311: case 0x312: case 0x313: case 0x314: case 0x315: case 0x316: case 0x317:
        case 0x318: case 0x319: case 0x31a: case 0x31b: case 0x31c: case 0x31d: case 0x31e: case 0x31f:
            if(ethdev)
                return ne2000_asic_read_w(isa_addr, ethdev);
            else
                return 0xffff;
        case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3: case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
            return ide_read16(ide, isa_addr & 0x07);
        case 0x3f6:
            return ide_read16(ide, ide_altst_r);
        default:
            fprintf(stderr, "Unhandled ISA I/O word read port 0x%x\n", address);
            return 0xffff;
    }
}

void isa_io_write_byte(unsigned int address, unsigned int value)
{
    unsigned int isa_addr;
    if((address & 3) != 1){
        fprintf(stderr, "Bad ISA byte write at 0x%08x (bad low address bits)\n", address);
    }

    isa_addr = (address - 0xff400001) / 4;


    switch(isa_addr){
        case 0x300: case 0x301: case 0x302: case 0x303: case 0x304: case 0x305: case 0x306: case 0x307:
        case 0x308: case 0x309: case 0x30a: case 0x30b: case 0x30c: case 0x30d: case 0x30e: case 0x30f:
            if(ethdev)
                ne2000_write(isa_addr, value, ethdev);
            break;
        case 0x310: case 0x311: case 0x312: case 0x313: case 0x314: case 0x315: case 0x316: case 0x317:
        case 0x318: case 0x319: case 0x31a: case 0x31b: case 0x31c: case 0x31d: case 0x31e: case 0x31f:
            if(ethdev)
                ne2000_asic_write_b(isa_addr, value, ethdev);
            break;
        case 0x3f8: case 0x3f9: case 0x3fa: case 0x3fb: case 0x3fc: case 0x3fd: case 0x3fe: case 0x3ff:
            uart16x50_write(uart, isa_addr & 0x07, value);
            return;
        case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3: case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
            ide_write8(ide, isa_addr & 0x07, value);
            return;
        case 0x3f6:
            ide_write8(ide, ide_altst_r, value);
            return;
        default:
            break;
    }
    /* NE2000 range -- avoid noise during probing! */
    if(isa_addr >= 0x280 && isa_addr <= 0x3a0)
        return;

    fprintf(stderr, "Unhandled ISA I/O byte write port 0x%x\n", address);
}

unsigned int isa_io_read_byte(unsigned int address)
{
    unsigned int isa_addr;
    if((address & 3) != 1){
        fprintf(stderr, "Bad ISA byte read at 0x%08x (wrong address?)\n", address);
    }

    isa_addr = (address - 0xff400001) / 4;

    switch(isa_addr){
        case 0x300: case 0x301: case 0x302: case 0x303: case 0x304: case 0x305: case 0x306: case 0x307:
        case 0x308: case 0x309: case 0x30a: case 0x30b: case 0x30c: case 0x30d: case 0x30e: case 0x30f:
            if(ethdev)
                return ne2000_read(isa_addr, ethdev);
            break;
        case 0x310: case 0x311: case 0x312: case 0x313: case 0x314: case 0x315: case 0x316: case 0x317:
        case 0x318: case 0x319: case 0x31a: case 0x31b: case 0x31c: case 0x31d: case 0x31e: case 0x31f:
            if(ethdev)
                return ne2000_asic_read_b(isa_addr, ethdev);
            break;
        case 0x3f8: case 0x3f9: case 0x3fa: case 0x3fb: case 0x3fc: case 0x3fd: case 0x3fe: case 0x3ff:
            return uart16x50_read(uart, isa_addr & 0x07);
        case 0x1f0: case 0x1f1: case 0x1f2: case 0x1f3: case 0x1f4: case 0x1f5: case 0x1f6: case 0x1f7:
            return ide_read8(ide, isa_addr & 0x07);
            break;
        case 0x3f6:
            return ide_read8(ide, ide_altst_r);
            break;
        default:
            break;
    }

    /* NE2000 range - avoid noise during probing! */
    if(isa_addr >= 0x280 && isa_addr <= 0x3a0)
        return 0xFF;

    fprintf(stderr, "Unhandled ISA I/O byte read port 0x%x\n", address);
    return 0xFF;
}

static unsigned char to_bcd(int n)
{
    return ((((n / 10)%10) << 4) | (n % 10));
}

unsigned int rtc_read_byte(unsigned int address)
{
    address = (address >> 2) & 0x7ff;

    if(address < NVRAMSIZE){
        return nvram[address];
    }else{
        time_t now_time;
        time(&now_time);
        struct tm *now = localtime(&now_time);
        switch(address & 0x7){
            case 1: return to_bcd(now->tm_sec);
            case 2: return to_bcd(now->tm_min);
            case 3: return to_bcd(now->tm_hour);
            case 4: return to_bcd((now->tm_wday - 2 % 7)+1);
            case 5: return to_bcd(now->tm_mday);
            case 6: return to_bcd(now->tm_mon + 1);
            case 7: return to_bcd(now->tm_year % 100);
            default:
            case 0: 
                    return 0; // control
        }
    }
}

void rtc_write_byte(unsigned int address, unsigned int value)
{
    address = (address >> 2) & 0x7ff;
    if(address < NVRAMSIZE){
        nvram[address] = value;
    }else{
        fprintf(stderr, "rtc write: %d = %02x\n", address & 8, value);
    }
}

unsigned int master_io_read(unsigned int address)
{
    address = (address - 0xff000000) / 4;

    switch(address){
        case 0: // interrupt status
            return ((frame_interrupt_asserted    ? 0x08 : 0x00) |
                    (uart16x50_irq_pending(uart) ? 0x04 : 0x00));
        case 1: // external interrupt register
            return ((uart16x50_irq_pending(uart) ? 0x02 : 0x00));
        default:
            return 0;
    }
}

void master_io_write(unsigned int address, unsigned int value)
{
    address = (address - 0xff000000) / 4;

    switch(address){
        case 2: // keyboard interrupt enable
            fprintf(stderr, "master_io_write: keyboard interrupt enable = 0x%02x\n", value & 0xff);
            break;
        case 4: // isa interrupt enable
            fprintf(stderr, "master_io_write: isa interrupt enable = 0x%02x\n", value & 0xff);
            break;
        case 5: // sample interrupt enable
            fprintf(stderr, "master_io_write: sample interrupt enable = 0x%02x\n", value & 0xff);
            break;
        case 8: // keyboard interrupt ack/clear
            fprintf(stderr, "master_io_write: keyboard interrupt ack = 0x%02x\n", value & 0xff);
            break;
        case 10: // sample interrupt ack/clear
            fprintf(stderr, "master_io_write: sample interrupt ack = 0x%02x\n", value & 0xff);
            break;
        case 9: // frame interrupt ack/clear
            // SHUT UP
            // fprintf(stderr, "master_io_write: frame interrupt ack = 0x%02x\n", value & 0xff);
            frame_interrupt_asserted = 0;
            break;
        case 6:
            fprintf(stderr, "master_io_write: display control = 0x%02x\n", value & 0xff);
            break;
        case 11:
            fprintf(stderr, "master_io_write: audio sample rate = 0x%02x\n", value & 0xff);
            break;
        case 12:
            fprintf(stderr, "master_io_write: led control = 0x%02x\n", value & 0xff);
            break;
        case 13:
            fprintf(stderr, "master_io_write: isa bus reset = 0x%02x\n", value & 0xff);
            break;
        case 14:
            fprintf(stderr, "master_io_write: frame interrupt rate = 0x%02x\n", value & 0xff);
            frame_interrupt_rate_bit = value & 1;
            break;
        default:
            fprintf(stderr, "master_io_write: register %d = 0x%02x\n", address, value & 0xff);
    }
}

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

#define WARN_ON_READ_BEFORE_READY
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

void uart16x50_signal_change(struct uart16x50 *uart, uint8_t mcr)
{
	/* Modem lines changed - don't care */
}

/* Read data from RAM, ROM, or a device */
unsigned int do_cpu_read_byte(unsigned int address, unsigned debug)
{
    /* First up -- ROM! Everyone loves ROM */
    if(address < 0x18000){ /* low 96KB view of ROM */
        if(softrom)
            return ram[address];
        else
            return rom[address];
    }
    if(address >= 0xfe000000 && address < 0xfe040000){ /* full 256KB view of ROM */
        return rom[address - 0xfe000000];
    }

    /* Now try video RAM */
    if(address >= 0x20000 && address < 0x28000){
        return vram[address];
    }
    if(address >= 0xfe800000 && address < 0xfe900000){
        return vram[address-0xfe800000];
    }

    /* Main RAM */
    if(address < memsize)
        return ram[address];

    /* OK, some sort of I/O space */
    if(address >= 0xff000000 && address < 0xff008000){
        return master_io_read(address);
    }
    if(address >= 0xff400000 && address < 0xff800000){
        return isa_io_read_byte(address);
    }
    if(address >= 0xff800000 && address < 0xffC00000){
        return isa_mem_read_byte(address);
    }
    if(address >= 0xff020000 && address < 0xff030000){
        return rtc_read_byte(address);
    }

    unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
    fprintf(stderr, "unmapped read at 0x%08x PC=0x%x\n", address, pc);
    return 0xff;
}

unsigned int cpu_read_byte(unsigned int address)
{
    unsigned int v;
    v = do_cpu_read_byte(address, 0);
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X:%02X}", address, v);
    return v;
}

unsigned int do_cpu_read_word(unsigned int address, unsigned int debug)
{
    if(address >= 0xff400000 && address < 0xff800000){
        return isa_io_read_word(address);
    }
    if(address >= 0xff800000 && address < 0xffC00000){
        return isa_mem_read_word(address);
    }
    return (do_cpu_read_byte(address, debug) << 8) | do_cpu_read_byte(address + 1, debug);
}

unsigned int cpu_read_word(unsigned int address)
{
    unsigned int v;
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
    v = (cpu_read_word(address) << 16) | cpu_read_word(address + 2);
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X:%08X}", address, v);
    return v;
}

unsigned int cpu_read_long_dasm(unsigned int address)
{
    return (cpu_read_word_dasm(address) << 16) | cpu_read_word_dasm(address + 2);
}

void cpu_write_byte(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%02X}", address, value);

    /* First up -- ROM! Everyone loves ROM */
    if(address < 0x18000){ /* low 96KB view of ROM */
        /* write to RAM below */
        if(!softrom)
            ram[address] = value;
        else{
            fprintf(stderr, "Write to 0x%08x in softrom mode\n", address);
        }
        return;
    }

    /* Now try video RAM */
    if(address >= 0x20000 && address < 0x28000){
        vram[address] = value;
        if(value != 0xaa){
            fprintf(stderr, "BAD VRAM WRITE 0x%x=0x%02x\n", address, value);
            do_dump_state();
        }
        return;
    }
    if(address >= 0xfe800000 && address < 0xfe900000){
        vram[address-0xfe800000] = value;
        if(value != 0xaa){
            fprintf(stderr, "BAD VRAM WRITE 0x%x=0x%02x\n", address, value);
            do_dump_state();
        }
        return;
    }

    /* Main RAM */
    if(address < memsize){
        ram[address] = value;
        return;
    }

    /* OK, some sort of I/O space */
    if(address >= 0xff000000 && address < 0xff008000){
        master_io_write(address, value);
        return;
    }
    if(address >= 0xff400000 && address < 0xff800000){
        isa_io_write_byte(address, value);
        return;
    }
    if(address >= 0xff800000 && address < 0xffC00000){
        isa_mem_write_byte(address, value);
        return;
    }
    if(address >= 0xff020000 && address < 0xff030000){
        rtc_write_byte(address, value);
        return;
    }

    unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
    fprintf(stderr, "unmapped write at 0x%08x, PC=0x%x\n", address, pc);
}

void cpu_write_word(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%04X}", address, value);

    if(address >= 0xff400000 && address < 0xff800000){
        isa_io_write_word(address, value);
        return;
    }
    if(address >= 0xff800000 && address < 0xffC00000){
        isa_mem_write_word(address, value);
        return;
    }

    cpu_write_byte(address, value >> 8);
    cpu_write_byte(address + 1, value & 0xFF);
}

void cpu_write_long(unsigned int address, unsigned int value)
{
    if (trace & TRACE_MEM)
        fprintf(stderr, "{%08X<%08X}", address, value);
    cpu_write_word(address, value >> 16);
    cpu_write_word(address + 2, value & 0xFFFF);
}

void cpu_write_pd(unsigned int address, unsigned int value)
{
    cpu_write_word(address + 2, value & 0xFFFF);
    cpu_write_word(address, value >> 16);
}

void cpu_instr_callback(void)
{
    if (trace & TRACE_CPU) {
        int len, i;
        char buf[128];
        unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);

        m68k_disassemble_il(buf, pc, M68K_CPU_TYPE_68030, &len);
        fprintf(stderr, "\n>%08X| ", pc);
        for(i=0; i<len; i+=2)
            fprintf(stderr, "%04x ", cpu_read_word_dasm(pc+i));
        for(; i<10; i+=2)
            fprintf(stderr, "     ");
        fprintf(stderr, "%-40s", buf);
    }
}

static void device_init(void)
{
    //u304_reset();
    //ns202_reset();
    //ppide_reset(ppide);
    //ecb_usb_fifo_reset(&usb_fifo);
    fprintf(stderr, "device_init\n");
    softrom = 0;
    uart16x50_reset(uart);
    uart16x50_set_input(uart, 1);
    uart16x50_signal_event(uart, 0x10); /* mini68K ROM wants the CTS bit asserted */
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
    tcsetattr(0, 0, &saved_term);
    exit(1);
}

static void exit_cleanup(void)
{
    tcsetattr(0, 0, &saved_term);
}

static void sig_dump_state(int sig)
{
    /* minimal signal handler */
    need_dump_state = 1;
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
    //ns202_trace(trace & TRACE_NS202 ? 1 : 0);
}

void recalc_interrupts(void)
{
    if(frame_interrupt_asserted)
        m68k_set_irq(M68K_IRQ_2);
    else
        m68k_set_irq(M68K_IRQ_NONE);


    // /* OPT16/8=0: UART on IRQ 4, OPT16/8=1: UART on IRQ 12 */
    // if(uart16x50_irq_pending(uart))
    //     ns202_raise(mfpic_cfg & 4 ? 12 : 4);

    // if(ppide->ide->drive[0].intrq || ppide->ide->drive[1].intrq)
    //     ns202_raise(9);

    // if(ecb_usb_fifo_get_irq(&usb_fifo))
    //     ns202_raise(7);

    // /* NS202 signals CPU interrupt with IPL0=0, IPL1=1, IPL2=0 */
    // if(ns202_irq_asserted())
    //     m68k_set_irq(M68K_IRQ_2);
}

int cpu_irq_ack(int level)
{
    recalc_interrupts();
    return M68K_INT_ACK_AUTOVECTOR;
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
    fprintf(stderr, "%s: [-m memsize][-r rompath][-i idepath][-I idepath][-d debug][-D debug].\n", name);
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

#define NS_BETWEEN_NAPS 100000
#define NS_PER_SEC 1000000000
#define TARGET_MHZ 40
#define FUDGE_FACTOR 2
#define CYCLES_PER_INTERVAL ((TARGET_MHZ*1000)/(1000000/NS_BETWEEN_NAPS)/FUDGE_FACTOR)

static struct timespec last_wakeup;
static const struct timespec interval = {
    .tv_sec  = NS_BETWEEN_NAPS / NS_PER_SEC,
    .tv_nsec = NS_BETWEEN_NAPS % NS_PER_SEC,
};

static void timespec_add(const struct timespec *a, const struct timespec *b, struct timespec *r)
{
    r->tv_sec = a->tv_sec + b->tv_sec;
    r->tv_nsec = a->tv_nsec + b->tv_nsec;
    if(r->tv_nsec >= NS_PER_SEC){
        r->tv_sec++;
        r->tv_nsec -= NS_PER_SEC;
    }
}

static void timespec_sub(const struct timespec *a, const struct timespec *b, struct timespec *r)
{
    r->tv_sec = a->tv_sec - b->tv_sec;
    r->tv_nsec = a->tv_nsec - b->tv_nsec;
    if(r->tv_nsec < 0){
        r->tv_sec--;
        r->tv_nsec += NS_PER_SEC;
    }
}

static void take_a_nap(void)
{
    struct timespec now;
    struct timespec delay;

    clock_gettime(CLOCK_MONOTONIC_RAW, &now);

    /* compute how long to sleep: delay = last_wakeup - now + interval */
    timespec_add(&last_wakeup, &interval, &delay);
    timespec_sub(&delay, &now, &delay);

    if(delay.tv_sec > 0 || delay.tv_nsec > 0)
        nanosleep(&delay, NULL);

    clock_gettime(CLOCK_MONOTONIC_RAW, &last_wakeup);
}

int main(int argc, char *argv[])
{
    int fd;
    int fast = 0;
    int opt;
    int memsize_mb = MAXDRAMMB;
    int rom_used;
    const char *romname = "q40boot.rom";
    const char *diskname = NULL;
    const char *diskname2 = NULL;
    const char *ethdevname = NULL;

    while((opt = getopt(argc, argv, "e:d:fi:m:r:s:A:B:I:S:D:u:U:")) != -1) {
        switch(opt) {
            case 'e':
                ethdevname = optarg;
                break;
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
            case 'I':
                diskname2 = optarg;
                break;
            case 'm':
                memsize_mb = atoi(optarg);
                break;
            case 'r':
                romname = optarg;
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

    if(optind < argc)
        usage(argv[0]);

    if(memsize_mb < 1){
        fprintf(stderr, "%s: RAM size must be at least 1MB\n", argv[0]);
        exit(1);
    }

    if(ethdevname){
        ethdev = ne2000_init(ethdevname);
    }else{
        ethdev = NULL;
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
    memset(vram, 0xA7, sizeof(vram));

    fd = open(romname, O_RDONLY);
    if (fd == -1) {
        perror(romname);
        exit(1);
    }
    rom_used = read(fd, rom, sizeof(rom));
    if(rom_used < sizeof(rom)){
        fprintf(stderr, "%s: too short. Padding to %ldKiB.\n", romname, (long)sizeof(rom) >> 10);
        memset(&rom[rom_used], 0xff, sizeof(rom)-rom_used);
    }
    close(fd);

    ide = ide_allocate("hd0");
    if(!ide)
        exit(1);

    if(diskname){
        fd = open(diskname, O_RDWR);
        if(fd < 0){
            perror(diskname);
            exit(1);
        }else{
            if(ide_attach(ide, 0, fd))
                exit(1);
        }
    }

    if(diskname2){
        fd = open(diskname2, O_RDWR);
        if(fd < 0){
            perror(diskname2);
            exit(1);
        }else{
            if(ide_attach(ide, 1, fd))
                exit(1);
        }
    }

    uart = uart16x50_create();
    if (trace & TRACE_UART)
        uart16x50_trace(uart, 1);

    m68k_init();
    m68k_set_cpu_type(&m68ki_cpu, M68K_CPU_TYPE_68040);
    m68k_pulse_reset(&m68ki_cpu);

    /* Init devices */
    device_init();

    clock_gettime(CLOCK_MONOTONIC_RAW, &last_wakeup);

    while (1) {
        if(need_toggle_trace)
            do_toggle_trace();
        if(need_dump_state)
            do_dump_state();
        m68k_execute(&m68ki_cpu, CYCLES_PER_INTERVAL);
        uart16x50_event(uart);
        if(ethdev)
            ne2000_poller(ethdev);
        recalc_interrupts();
        if (!fast)
            take_a_nap();
        if(++frame_interrupt_counter >= (frame_interrupt_rate_bit ? 
                    (NS_PER_SEC/NS_BETWEEN_NAPS/200) : (NS_PER_SEC/NS_BETWEEN_NAPS/50))){
            frame_interrupt_counter = 0;
            frame_interrupt_asserted = 1;
        }
    }
}
