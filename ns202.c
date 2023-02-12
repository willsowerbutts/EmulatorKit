/*
 *	NS32202 interrupt controller
 */

#include <stdio.h>
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
#include "ns202.h"

#define R_HVCT	0
#define R_SVCT	1
#define R_ELTG	2
#define R_TPR	4
#define R_IPND	6
#define R_ISRV	8
#define R_IMSK	10
#define R_CSRC	12
#define R_FPRT	14
#define R_MCTL	16
#define R_OCASN	17
#define R_CIPTR	18
#define R_PDAT	19
#define R_IPS	20
#define	R_PDIR	21
#define	R_CCTL	22
#define R_CICTL	23
#define R_CSV	24
#define R_LCSV  24
#define R_HCSV  26
#define R_CCV	28
#define R_LCCV  28
#define R_HCCV  30

struct ns32202 {
    uint8_t reg[32];
    uint16_t ct_l, ct_h;
    unsigned live;
    unsigned irq;
    int trace;
};

struct ns32202 ns202;

void ns202_trace(int on)
{
    ns202.trace = on;
}

void ns202_dump_state(void)
{
    fprintf(stderr, "ns202: ");
    fprintf(stderr, "HVCT=%02x    ",                             ns202.reg[R_HVCT]);
    fprintf(stderr, "SVCT=%02x    ",                             ns202.reg[R_SVCT]);
    fprintf(stderr, "ELTG=%04x  ",  (ns202.reg[1+R_ELTG] << 8) | ns202.reg[R_ELTG]);
    fprintf(stderr, "TPL=%04x   ",  (ns202.reg[1+R_TPR] << 8)  | ns202.reg[R_TPR]);
    fprintf(stderr, "IPND=%04x  ",  (ns202.reg[1+R_IPND] << 8) | ns202.reg[R_IPND]);
    fprintf(stderr, "ISRV=%04x  ",  (ns202.reg[1+R_ISRV] << 8) | ns202.reg[R_ISRV]);
    fprintf(stderr, "\nns202: ");
    fprintf(stderr, "IMSK=%04x  ",  (ns202.reg[1+R_IMSK] << 8) | ns202.reg[R_IMSK]);
    fprintf(stderr, "CSRC=%04x  ",  (ns202.reg[1+R_CSRC] << 8) | ns202.reg[R_CSRC]);
    fprintf(stderr, "FPRT=%04x  ",  (ns202.reg[1+R_FPRT] << 8) | ns202.reg[R_FPRT]);
    fprintf(stderr, "MCTL=%02x    ",                             ns202.reg[R_MCTL]);
    fprintf(stderr, "OCASN=%02x   ",                             ns202.reg[R_OCASN]);
    fprintf(stderr, "CIPTR=%04x ",  (ns202.reg[1+R_CIPTR] << 8)| ns202.reg[R_CIPTR]);
    fprintf(stderr, "\nns202: ");
    fprintf(stderr, "PDAT=%02x    ",                             ns202.reg[R_PDAT]);
    fprintf(stderr, "IPS=%02x     ",                             ns202.reg[R_IPS]);
    fprintf(stderr, "PDIR=%02x    ",                             ns202.reg[R_PDIR]);
    fprintf(stderr, "CCTL=%02x    ",                             ns202.reg[R_CCTL]);
    fprintf(stderr, "CICTL=%02x   ",                             ns202.reg[R_CICTL]);
    fprintf(stderr, "\nns202: ");
    fprintf(stderr, "LCSV=%04x  ",  (ns202.reg[1+R_LCSV] << 8) | ns202.reg[R_LCSV]);
    fprintf(stderr, "HCSV=%04x  ",  (ns202.reg[1+R_HCSV] << 8) | ns202.reg[R_HCSV]);
    fprintf(stderr, "LCCV=%04x  ",  (ns202.reg[1+R_LCCV] << 8) | ns202.reg[R_LCCV]);
    fprintf(stderr, "HCCV=%04x  ",  (ns202.reg[1+R_HCCV] << 8) | ns202.reg[R_HCCV]);
    fprintf(stderr, "\nns202: live=%x irq=%x\n", ns202.live, ns202.irq);
}

/* Bitop helpers for reg pairs */
static int ns202_top16(unsigned r)
{
    unsigned v = ns202.reg[r] | (ns202.reg[r+1] << 8);
    int n = 15;
    while (n >= 0) {
        if (v & (1 << n))
            return n;
        n--;
    }
    return -1;
}

static unsigned ns202_test16(unsigned r, unsigned n)
{
    if (n >= 8)
        return !!(ns202.reg[r + 1] & (1 << (n - 8)));
    return !!(ns202.reg[r] & (1 << n));
}

static unsigned prev_ipnd=0, prev_isrv=0, prev_imsk;
void trace_ipnd_isrv(void)
{
    if(ns202.trace){
        unsigned int ipnd, isrv, imsk;
        ipnd = (ns202.reg[R_IPND+1] << 8) | ns202.reg[R_IPND];
        isrv = (ns202.reg[R_ISRV+1] << 8) | ns202.reg[R_ISRV];
        imsk = (ns202.reg[R_IMSK+1] << 8) | ns202.reg[R_IMSK];
        if(ipnd != prev_ipnd){
            fprintf(stderr, " IPND=%04x", ipnd);
            prev_ipnd = ipnd;
        }
        if(isrv != prev_isrv){
            fprintf(stderr, " ISRV=%04x", isrv);
            prev_isrv = isrv;
        }
        if(imsk != prev_imsk){
            fprintf(stderr, " IMSK=%04x", imsk);
            prev_imsk = imsk;
        }
    }
}

static void ns202_set16(unsigned r, unsigned n)
{
    if (n >= 8)
        ns202.reg[r + 1] |= 1 << (n - 8);
    else
        ns202.reg[r] |= 1 << n;
    trace_ipnd_isrv();
}

static void ns202_clear16(unsigned r, unsigned n)
{
    if (n >= 8)
        ns202.reg[r + 1] &= ~(1 << (n - 8));
    else
        ns202.reg[r] &= ~(1 << n);
    trace_ipnd_isrv();
}

/* TODO - rotating priority */
/* FIXME: is prio low - hi or hi - low */
static int ns202_hipri(void)
{
    int n = 15;
    while(n >= 0) {
        if (ns202_test16(R_IPND, n) && !ns202_test16(R_IMSK, n))
            return n;
        n--;
    }
    return -1;
}

/* Find the highest interrupt priority and if it is higher than the
   current highest priority then set irq and remember it
TODO: set HSRV ?? */
static void ns202_compute_int(void)
{
    /* Find the highest interrupt that isn't masked */
    int n = ns202_hipri();
    int t = ns202_top16(R_ISRV);
    if (n == -1)
        return;
    /* TODO: rotating mode */
    if (ns202.live == NS202_NO_INT || (n > ns202.live && n > t)) {
        /* We have a new winner for topmost interrupt */
        if(ns202.trace)
            fprintf(stderr, " IRQ=1");
            /* fprintf(stderr, "<RAISE%d top=%d live=%d> ", n, t, ns202.live); */
        ns202.irq = 1;
        ns202.live = n;
        ns202.reg[R_HVCT] &= 0xF0;
        ns202.reg[R_HVCT] |= n;
    }
}

/* We had int raised (hopefully) and the CPU acked it */
unsigned int ns202_int_ack(void)
{
    unsigned live = ns202.live;
    if (ns202.trace)
        fprintf(stderr, "<ACK%d", live);
    /* The IPND for the active interrupt is cleared, the corresponding
       bit in the ISRV is set. We don't model cascaded ICU */
    ns202_clear16(R_IPND, live);
    ns202_set16(R_ISRV, live);
    /* Clear the timer interrupt requests
     * Guesswork - seems the ACK clears the counter flag, but what about error bit ? */
    if (live == (ns202.reg[R_CIPTR] & 0x0F))
        ns202.reg[R_CICTL] &= 0xFB;
    if (live == ns202.reg[R_CIPTR] >> 4)
        ns202.reg[R_CICTL] &= 0xBF;
    /* And the interrupt is dropped */
    ns202.irq = 0;
    if(ns202.trace)
        fprintf(stderr, " IRQ=0");
    /* Check if there isn't now a higher priority into to interrupt the
       interrupt */
    ns202_compute_int();
    live |= ns202.reg[R_HVCT] & 0xF0;
    /* if (ns202.trace)
           fprintf(stderr, "vector 0x%02X\n", live); */
    if (ns202.trace)
        fprintf(stderr, ">");
    return live;
}

unsigned int ns202_irq_asserted(void)
{
    return ns202.irq;
}

/* RETI or equivalent occurred. */
static void ns202_clear_int(void)
{
    unsigned live = ns202.live;
    if (live == NS202_NO_INT)
        return;
    if(ns202.trace)
        fprintf(stderr, "<CLR%d", live);
    ns202.reg[R_HVCT] |= 0x0F;
    /* Clear the live interrupt in ISRV */
    ns202_clear16(R_ISRV, live);
    /* if (ns202.trace)
        fprintf(stderr, "ns202: int clear %d\n", live); */
    ns202.live = NS202_NO_INT;
    /* Check if there is anything pending to cause a next interrupt */
    ns202_compute_int();
    if(ns202.trace)
        fprintf(stderr, ">\n");
}

/* TODO: emulate mis-setting 8 v 16bit mode */
static unsigned int do_ns202_read(unsigned int address)
{
    unsigned ns32_reg = (address >> 8) & 0x1F;
    unsigned ns32_sti = (address >> 8) & 0x20;

    switch(ns32_reg) {
        case R_HVCT:
            /* INTA or RETI cycle */
            if (ns32_sti)	/* RETI */
                ns202_clear_int();
            else		/* INTA */
                ns202_int_ack();
            if (ns32_sti)
                return ns202.reg[R_HVCT]|0x0F;
            return ns202.reg[R_HVCT];
        case R_SVCT:
            //		ns32_hvct_recalc();
            return ns202.reg[R_HVCT];
        case R_FPRT:
            if (ns202.reg[R_FPRT] < 8)
                return 1 << ns202.reg[R_FPRT];
            return 0;
        case R_FPRT + 1:
            if (ns202.reg[R_FPRT] >= 8)
                return 1 << (ns202.reg[R_FPRT] - 8);
            return 0;
        case R_CCV:
        case R_CCV + 1:
        case R_CCV + 2:
        case R_CCV + 3:
            /* The CCV can only be read when counter readings are
               frozen, but the documentation says nothing about what
               happens otherwise, so ignore this TODO */
        case R_TPR:
        case R_TPR + 1:
        case R_ELTG:
        case R_ELTG + 1:
        case R_IPND:
        case R_IPND + 1:
        case R_CSRC:
        case R_CSRC + 1:
        case R_ISRV:
        case R_ISRV + 1:
        case R_IMSK:
        case R_IMSK + 1:
        case R_MCTL:
        case R_OCASN:
        case R_CIPTR:
        case R_PDAT:
            /* We assume no input GPIO */
        case R_IPS:
        case R_PDIR:
        case R_CCTL:
        case R_CICTL:
        case R_CSV:
        case R_CSV + 1:
        case R_CSV + 2:
        case R_CSV + 3:
        default:
            return ns202.reg[ns32_reg];
    }
}

unsigned int ns202_read(unsigned int address)
{
    unsigned r = do_ns202_read(address);
    if (ns202.trace > 1)
        fprintf(stderr, "ns202_read [%02d] = %02X\n", (address >> 8) & 0x1F, r);
    return r;
}

void ns202_write(unsigned int address, unsigned int value)
{
    unsigned ns32_reg = (address >> 8) & 0x1F;

    if (ns202.trace > 1)
        fprintf(stderr, "ns202_write [%02d] = %02X\n", ns32_reg, value);
    switch(ns32_reg) {
        case R_HVCT:
            break;
        case R_SVCT:
            ns202.reg[R_HVCT] &= 0x0F;
            ns202.reg[R_HVCT] |= (value & 0xF0);
            break;
            /* TODO: IPND write is special forms */
        case R_IPND:
        case R_IPND + 1:
            break;
        case R_FPRT:
            ns202.reg[R_FPRT] = value & 0x0F;
            break;
        case R_FPRT + 1:
            /* Not writeable */
            break;
        case R_CCTL:
            /* Never see CDCRL or CDCRH 1 */
            ns202.reg[ns32_reg] &= 0xFC;
            ns202.reg[ns32_reg] = value;
            /* Need to process single cycle decrementer here TODO */
            break;
        case R_CICTL:
            if (value & 0x01) {
                ns202.reg[ns32_reg] &= 0xF0;
                ns202.reg[ns32_reg] |= value & 0x0F;
            }
            if (value & 0x10) {
                ns202.reg[ns32_reg] &= 0x0F;
                ns202.reg[ns32_reg] |= value & 0xF0;
            }
            break;
        case R_CIPTR:
            /* clear the interrupt if it is pending */
            ns202.reg[ns32_reg] = value;
            if(ns202_test16(R_IPND, ns202.reg[R_CIPTR] >> 4))
                ns202_clear16(R_IPND, ns202.reg[R_CIPTR] >> 4);
            if(ns202_test16(R_IPND, ns202.reg[R_CIPTR] & 0x0F))
                ns202_clear16(R_IPND, ns202.reg[R_CIPTR] & 0x0F);
            ns202_compute_int();
            break;
        case R_ISRV + 1:
        case R_ISRV:
            ns202.reg[ns32_reg] = value;
            if(ns202.reg[R_ISRV+1] == 0 && ns202.reg[R_ISRV] == 0){
                ns202_clear_int();
                trace_ipnd_isrv();
            }
            break;
        case R_CCV:
        case R_CCV + 1:
        case R_CCV + 2:
        case R_CCV + 3:
            /* Just adjust the register */
        case R_TPR:
        case R_TPR + 1:
        case R_ELTG:
        case R_ELTG + 1:
        case R_IMSK:
        case R_IMSK + 1:
        case R_CSRC:
        case R_CSRC + 1:
        case R_MCTL:
        case R_OCASN:
        case R_PDAT:
            /* We assume no output GPIO activity */
        case R_IPS:
        case R_PDIR:
        case R_CSV:
        case R_CSV + 1:
        case R_CSV + 2:
        case R_CSV + 3:
        default:
            ns202.reg[ns32_reg] = value;
    }
    ns202_compute_int();
}

void ns202_raise(unsigned irq)
{
    int close=0;
    const char *intname[17] = {
        "0/uart1",
        "1/unused",
        "2/unused",
        "3/bios_timer", /* timer in BIOS */
        "4/unused",
        "5/unused",
        "6/unused",
        "7/ide1",
        "8/unused",
        "9/ide0",
        "10/timerH",
        "11/timerL",
        "12/uart0",
        "13/unused",
        "14/unused",
        "15/unused",
        "<unexpected>" };

    if (ns202.reg[R_MCTL] & 0x08)	/* FRZ (freeze) - prevents interrupts being raised */
        return;
    if(ns202.trace){
        if(!ns202_test16(R_IPND, irq)){
            fprintf(stderr, "<IRQ%s", intname[irq < 16 ? irq : 16]);
            close=1;
        }
    }
    ns202_set16(R_IPND, irq);
    ns202_compute_int();
    if(close)
        fprintf(stderr, ">");
}

uint32_t ticks=0, last_low_tick=0, last_high_tick=0;

void ns202_counter(void)
{
    uint8_t cctl = ns202.reg[R_CCTL];
    uint8_t cictl = ns202.reg[R_CICTL];

    ticks++;

    /* Split clocks, low enabled */
    if ((cctl & 0x84) == 0x04) {
        /* Low counter decrement */
        ns202.ct_l--;
        if (ns202.ct_l == 0) {
            if(ns202.trace){
                fprintf(stderr, "ns202: low, interval=%d ", ticks - last_low_tick);
                last_low_tick = ticks;
            }
            if (cictl & 0x04)
                cictl |= 0x08;
            else
                cictl |= 0x04;
            ns202.ct_l = ns202.reg[R_CSV] | (ns202.reg[R_CSV + 1] << 8);
            //			if (ns202.trace)
            //				fprintf(stderr, "ns202: low counter 0 reload %d.\n", ns202.ct_l);
        }
    }

    /* Split clocks, high enabled */
    if ((cctl & 0x88) == 0x08) {
        /* High counter decrement */
        ns202.ct_h--;
        if (ns202.ct_h == 0) {
            if(ns202.trace){
                fprintf(stderr, "ns202: high, interval=%d ", ticks - last_high_tick);
                last_high_tick = ticks;
            }
            if (cictl & 0x40)
                cictl |= 0x80;
            else
                cictl |= 0x40;
            ns202.ct_h = ns202.reg[R_CSV + 2] | (ns202.reg[R_CSV + 3] << 8);
            //			if (ns202.trace)
            //				fprintf(stderr, "ns202: high counter 0 reload %d.\n", ns202.ct_h);
        }
    }

    if ((cctl & 0x88) == 0x88) {
        /* 32bit decrement */
        if (ns202.ct_l == 0) {
            ns202.ct_h--;
            if (ns202.ct_h == 0) {
                if(ns202.trace){
                    fprintf(stderr, "ns202: high, interval=%d ", ticks - last_high_tick);
                    last_high_tick = ticks;
                }
                if (cictl & 0x40)
                    cictl |= 0x80;
                else
                    cictl |= 0x40;
                ns202.ct_l = ns202.reg[R_CSV] | (ns202.reg[R_CSV + 1] << 8);
                ns202.ct_h = ns202.reg[R_CSV + 2] | (ns202.reg[R_CSV + 3] << 8);
                //				if (ns202.trace)
                //					fprintf(stderr, "ns202: dual counter 0.\n");
            }
        }
        ns202.ct_l--;
    }

    ns202.reg[R_CICTL] = cictl;
}

void ns202_tick(unsigned clocks)
{
    static unsigned dclock;
    unsigned scale = (ns202.reg[R_CCTL] & 0x40) ? 4 : 1;

    /* TODO there must be a more efficient way to do this! 
     * surely we can deal with multiple ticks in one go */
    dclock += clocks;
    while (dclock >= scale) {
        dclock -= scale;
        ns202_counter();
    }

    /* Update LCCV/HCCV if we should do so */
    if (!(ns202.reg[R_MCTL] & 0x80)) {	/* CFRZ */
        ns202.reg[28] = ns202.ct_l;
        ns202.reg[29] = ns202.ct_l >> 8;
        ns202.reg[30] = ns202.ct_h;
        ns202.reg[31] = ns202.ct_h >> 8;
    }

    /* Raise interrupts as needed */
    if ((ns202.reg[R_CICTL] & 0x60) == 0x60) {
        ns202_raise(ns202.reg[R_CIPTR] >> 4);
    }
    if ((ns202.reg[R_CICTL] & 0x06) == 0x06) {
        ns202_raise(ns202.reg[R_CIPTR] & 0x0F);
    }
}

void ns202_reset(void)
{
    ns202.reg[R_IMSK] = 0xFF;
    ns202.reg[R_IMSK + 1] = 0xFF;
    ns202.reg[R_CIPTR] = 0xFF;
    ns202.live = NS202_NO_INT;
}
