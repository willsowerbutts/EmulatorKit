// WRS: based on GPL code borrowed from PCem
/////////////////////////////////////////////////////////////////////////
// $Id: ne2k.cc,v 1.56.2.1 2004/02/02 22:37:22 cbothamy Exp $
/////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2002  MandrakeSoft S.A.
//
//    MandrakeSoft S.A.
//    43, rue d'Aboukir
//    75002 Paris - France
//    http://www.linux-mandrake.com/
//    http://www.mandrakesoft.com/
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

// Peter Grehan (grehan@iprg.nokia.com) coded all of this
// NE2000/ether stuff.
//#include "vl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pcap.h>
#include <asm/byteorder.h>
#include "ne2000.h"

#undef  NE2000_DEBUG

#define le16_to_cpu __le16_to_cpu
#define cpu_to_le16 __cpu_to_le16

// #include "ibm.h"
// #include "device.h"
// 
// #include "nethandler.h"
// 
// #include "config.h"
// #include "io.h"
// #include "ne2000.h"
// #include "pic.h"
// #include "pci.h"
// #include "timer.h"

/*MAC address*/
uint8_t maclocal[6];

#define NE2000_PCI_IRQ -1

#define NETBLOCKING 0 // we won't block our pcap

pcap_t *net_pcap;

#define pclog(...) fprintf(stderr, __VA_ARGS__)
#define fatal(...) do { fprintf(stderr, __VA_ARGS__); exit(1); } while(0)

static void ne2000_tx_event(int val, void *p);

// static void ne2000_setirq(ne2000_t *ne2000, int irq) { ne2000->base_irq = irq; }

static void ne2000_raise_irq(ne2000_t *ne2000) {
    // WRS: TODO!
        //if (ne2000->is_pci)
        //        pci_set_irq(ne2000->card, PCI_INTA);
        //else
        //        picint(1 << ne2000->base_irq);
}

static void ne2000_lower_irq(ne2000_t *ne2000) {
    // WRS: TODO!
        //if (ne2000->is_pci)
        //        pci_clear_irq(ne2000->card, PCI_INTA);
        //else
        //        picintc(1 << ne2000->base_irq);
}

//
// reset - restore state to power-up, cancelling all i/o
//
static void ne2000_reset(int type, void *p) {
        ne2000_t *ne2000 = (ne2000_t *)p;
        int i;

        pclog("ne2000 reset\n");

        // Initialise the mac address area by doubling the physical address
        ne2000->macaddr[0] = ne2000->physaddr[0];
        ne2000->macaddr[1] = ne2000->physaddr[0];
        ne2000->macaddr[2] = ne2000->physaddr[1];
        ne2000->macaddr[3] = ne2000->physaddr[1];
        ne2000->macaddr[4] = ne2000->physaddr[2];
        ne2000->macaddr[5] = ne2000->physaddr[2];
        ne2000->macaddr[6] = ne2000->physaddr[3];
        ne2000->macaddr[7] = ne2000->physaddr[3];
        ne2000->macaddr[8] = ne2000->physaddr[4];
        ne2000->macaddr[9] = ne2000->physaddr[4];
        ne2000->macaddr[10] = ne2000->physaddr[5];
        ne2000->macaddr[11] = ne2000->physaddr[5];

        // ne2k signature
        for (i = 12; i < 32; i++)
                ne2000->macaddr[i] = 0x57;

        // Zero out registers and memory
        memset(&ne2000->CR, 0, sizeof(ne2000->CR));
        memset(&ne2000->ISR, 0, sizeof(ne2000->ISR));
        memset(&ne2000->IMR, 0, sizeof(ne2000->IMR));
        memset(&ne2000->DCR, 0, sizeof(ne2000->DCR));
        memset(&ne2000->TCR, 0, sizeof(ne2000->TCR));
        memset(&ne2000->TSR, 0, sizeof(ne2000->TSR));
        // memset( & ne2000->RCR, 0, sizeof(ne2000->RCR));
        memset(&ne2000->RSR, 0, sizeof(ne2000->RSR));
        ne2000->tx_timer_active = 0;
        ne2000->local_dma = 0;
        ne2000->page_start = 0;
        ne2000->page_stop = 0;
        ne2000->bound_ptr = 0;
        ne2000->tx_page_start = 0;
        ne2000->num_coll = 0;
        ne2000->tx_bytes = 0;
        ne2000->fifo = 0;
        ne2000->remote_dma = 0;
        ne2000->remote_start = 0;
        ne2000->remote_bytes = 0;
        ne2000->tallycnt_0 = 0;
        ne2000->tallycnt_1 = 0;
        ne2000->tallycnt_2 = 0;

        // memset( & ne2000->physaddr, 0, sizeof(ne2000->physaddr));
        // memset( & ne2000->mchash, 0, sizeof(ne2000->mchash));
        ne2000->curr_page = 0;

        ne2000->rempkt_ptr = 0;
        ne2000->localpkt_ptr = 0;
        ne2000->address_cnt = 0;

        memset(&ne2000->mem, 0, sizeof(ne2000->mem));

        // Set power-up conditions
        ne2000->CR.stop = 1;
        ne2000->CR.rdma_cmd = 4;
        ne2000->ISR.reset = 1;
        ne2000->DCR.longaddr = 1;
        ne2000_raise_irq(ne2000);
        ne2000_lower_irq(ne2000);

        // DEV_pic_lower_irq(ne2000->base_irq);
}

// #include "bswap.h"

//
// chipmem_read/chipmem_write - access the 64K private RAM.
// The ne2000 memory is accessed through the data port of
// the asic (offset 0) after setting up a remote-DMA transfer.
// Both byte and word accesses are allowed.
// The first 16 bytes contains the MAC address at even locations,
// and there is 16K of buffer memory starting at 16K
//
static inline uint8_t ne2000_chipmem_read_b(uint32_t address, ne2000_t *ne2000) {
        // ROM'd MAC address
        if ((address >= 0) && (address <= 31))
                return ne2000->macaddr[address];

        if ((address >= BX_NE2K_MEMSTART) && (address < BX_NE2K_MEMEND))
                return ne2000->mem[address - BX_NE2K_MEMSTART];
        else
                return 0xff;
}

static inline uint16_t ne2000_chipmem_read_w(uint32_t address, ne2000_t *ne2000) {
        // ROM'd MAC address
        if ((address >= 0) && (address <= 31))
                return le16_to_cpu(*(uint16_t *)(ne2000->macaddr + address));

        if ((address >= BX_NE2K_MEMSTART) && (address < BX_NE2K_MEMEND))
                return le16_to_cpu(*(uint16_t *)(ne2000->mem + (address - BX_NE2K_MEMSTART)));
        else
                return 0xffff;
}

static inline void ne2000_chipmem_write_b(uint32_t address, uint8_t value, ne2000_t *ne2000) {
        if ((address >= BX_NE2K_MEMSTART) && (address < BX_NE2K_MEMEND))
                ne2000->mem[address - BX_NE2K_MEMSTART] = value & 0xff;
}

static inline void ne2000_chipmem_write_w(uint32_t address, uint16_t value, ne2000_t *ne2000) {
        if ((address >= BX_NE2K_MEMSTART) && (address < BX_NE2K_MEMEND))
                *(uint16_t *)(ne2000->mem + (address - BX_NE2K_MEMSTART)) = cpu_to_le16(value);
}

//
// asic_read/asic_write - This is the high 16 bytes of i/o space
// (the lower 16 bytes is for the DS8390). Only two locations
// are used: offset 0, which is used for data transfer, and
// offset 0xf, which is used to reset the device.
// The data transfer port is used to as 'external' DMA to the
// DS8390. The chip has to have the DMA registers set up, and
// after that, insw/outsw instructions can be used to move
// the appropriate number of bytes to/from the device.
//
uint16_t ne2000_dma_read(int io_len, ne2000_t *ne2000)
{

        //
        // The 8390 bumps the address and decreases the byte count
        // by the selected word size after every access, not by
        // the amount of data requested by the host (io_len).
        //
        ne2000->remote_dma += io_len;
        if (ne2000->remote_dma == ne2000->page_stop << 8)
                ne2000->remote_dma = ne2000->page_start << 8;

        // keep s.remote_bytes from underflowing
        if (ne2000->remote_bytes > 1)
                ne2000->remote_bytes -= io_len;
        else
                ne2000->remote_bytes = 0;

        // If all bytes have been written, signal remote-DMA complete
        if (ne2000->remote_bytes == 0) {
                ne2000->ISR.rdma_done = 1;
                if (ne2000->IMR.rdma_inte) {
                        ne2000_raise_irq(ne2000);
                }
        }

        return 0;
}

uint16_t ne2000_asic_read_w(uint16_t offset, ne2000_t *ne2000)
{
        int retval;

        if (ne2000->DCR.wdsize & 0x01) {
                /* 16 bit access */
                retval = ne2000_chipmem_read_w(ne2000->remote_dma, ne2000);
                ne2000_dma_read(2, ne2000);
        } else {
                /* 8 bit access */
                retval = ne2000_chipmem_read_b(ne2000->remote_dma, ne2000);
                ne2000_dma_read(1, ne2000);
        }
#ifdef NE2000_DEBUG
        pclog("asic read val=0x%04x\n", retval);
#endif
        return retval;
}

void ne2000_dma_write(int io_len, ne2000_t *ne2000)
{

        // is this right ??? asic_read uses DCR.wordsize
        ne2000->remote_dma += io_len;
        if (ne2000->remote_dma == ne2000->page_stop << 8)
                ne2000->remote_dma = ne2000->page_start << 8;

        ne2000->remote_bytes -= io_len;
        if (ne2000->remote_bytes > BX_NE2K_MEMSIZ)
                ne2000->remote_bytes = 0;

        // If all bytes have been written, signal remote-DMA complete
        if (ne2000->remote_bytes == 0) {
                ne2000->ISR.rdma_done = 1;
                if (ne2000->IMR.rdma_inte) {
                        ne2000_raise_irq(ne2000);
                }
        }
}

void ne2000_asic_write_w(uint16_t offset, uint16_t value, ne2000_t *ne2000)
{
#ifdef NE2000_DEBUG
        pclog("asic write val=0x%04x\n", value);
#endif
        if (ne2000->remote_bytes == 0)
                return;
        if (ne2000->DCR.wdsize & 0x01) {
                /* 16 bit access */
                ne2000_chipmem_write_w(ne2000->remote_dma, value, ne2000);
                ne2000_dma_write(2, ne2000);
        } else {
                /* 8 bit access */
                ne2000_chipmem_write_b(ne2000->remote_dma, value, ne2000);
                ne2000_dma_write(1, ne2000);
        }
}

uint8_t ne2000_asic_read_b(uint16_t offset, ne2000_t *ne2000)
{
        if (offset & 1)
                return ne2000_asic_read_w(offset & ~1, ne2000) >> 1;
        return ne2000_asic_read_w(offset, ne2000) & 0xff;
}

void ne2000_asic_write_b(uint16_t offset, uint8_t value, ne2000_t *ne2000)
{
        if (offset & 1)
                ne2000_asic_write_w(offset & ~1, value << 8, ne2000);
        else
                ne2000_asic_write_w(offset, value, ne2000);
}

uint8_t ne2000_reset_read(uint16_t offset, ne2000_t *ne2000)
{
        ne2000_reset(BX_RESET_SOFTWARE, ne2000);
        return 0;
}

void ne2000_reset_write(uint16_t offset, uint8_t value, ne2000_t *ne2000)
{
    // end reset cycle
}

//
// read_handler/read - i/o 'catcher' function called from BOCHS
// mainline when the CPU attempts a read in the i/o space registered
// by this ne2000 instance
//
uint8_t ne2000_read(uint16_t address, ne2000_t *ne2000) 
{
        int ret = 0;

#ifdef NE2000_DEBUG
        pclog("read addr %x\n", address);
#endif
        address &= 0xf;

        if (address == 0x00) {
                ret = (((ne2000->CR.pgsel & 0x03) << 6) | ((ne2000->CR.rdma_cmd & 0x07) << 3) | (ne2000->CR.tx_packet << 2) |
                       (ne2000->CR.start << 1) | (ne2000->CR.stop));
#ifdef NE2000_DEBUG
                pclog("read CR returns 0x%08x\n", ret);
#endif
        } else {
                switch (ne2000->CR.pgsel) {
                case 0x00:
#ifdef NE2000_DEBUG
                        pclog("page 0 read from port %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: // CLDA0
                                return ne2000->local_dma & 0xff;

                        case 0x2: // CLDA1
                                return ne2000->local_dma >> 8;

                        case 0x3: // BNRY
                                return ne2000->bound_ptr;

                        case 0x4: // TSR
                                return ((ne2000->TSR.ow_coll << 7) | (ne2000->TSR.cd_hbeat << 6) | (ne2000->TSR.fifo_ur << 5) |
                                        (ne2000->TSR.no_carrier << 4) | (ne2000->TSR.aborted << 3) | (ne2000->TSR.collided << 2) |
                                        (ne2000->TSR.tx_ok));

                        case 0x5: // NCR
                                return ne2000->num_coll;

                        case 0x6: // FIFO
                                  // reading FIFO is only valid in loopback mode
#ifdef NE2000_DEBUG
                                pclog("reading FIFO not supported yet\n");
#endif
                                return ne2000->fifo;

                        case 0x7: // ISR
                                return ((ne2000->ISR.reset << 7) | (ne2000->ISR.rdma_done << 6) | (ne2000->ISR.cnt_oflow << 5) |
                                        (ne2000->ISR.overwrite << 4) | (ne2000->ISR.tx_err << 3) | (ne2000->ISR.rx_err << 2) |
                                        (ne2000->ISR.pkt_tx << 1) | (ne2000->ISR.pkt_rx));

                        case 0x8: // CRDA0
                                return ne2000->remote_dma & 0xff;

                        case 0x9: // CRDA1
                                return ne2000->remote_dma >> 8;

                        case 0xa: // reserved
#ifdef NE2000_DEBUG
                                pclog("reserved read - page 0, 0xa\n");
#endif
                                return 0xff;

                        case 0xb: // reserved
#ifdef NE2000_DEBUG
                                pclog("reserved read - page 0, 0xb\n");
#endif
                                return 0xff;

                        case 0xc: // RSR
                                return ((ne2000->RSR.deferred << 7) | (ne2000->RSR.rx_disabled << 6) |
                                        (ne2000->RSR.rx_mbit << 5) | (ne2000->RSR.rx_missed << 4) | (ne2000->RSR.fifo_or << 3) |
                                        (ne2000->RSR.bad_falign << 2) | (ne2000->RSR.bad_crc << 1) | (ne2000->RSR.rx_ok));

                        case 0xd: // CNTR0
                                return ne2000->tallycnt_0;

                        case 0xe: // CNTR1
                                return ne2000->tallycnt_1;

                        case 0xf: // CNTR2
                                return ne2000->tallycnt_2;
                        }

                        return 0;

                case 0x01:
#ifdef NE2000_DEBUG
                        pclog("page 1 read from port %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: // PAR0-5
                        case 0x2:
                        case 0x3:
                        case 0x4:
                        case 0x5:
                        case 0x6:
                                return ne2000->physaddr[address - 1];

                        case 0x7: // CURR
#ifdef NE2000_DEBUG
                                pclog("returning current page: %02x\n", (ne2000->curr_page));
#endif
                                return ne2000->curr_page;

                        case 0x8: // MAR0-7
                        case 0x9:
                        case 0xa:
                        case 0xb:
                        case 0xc:
                        case 0xd:
                        case 0xe:
                        case 0xf:
                                return ne2000->mchash[address - 8];
                        }

                        return 0;

                case 0x02:
#ifdef NE2000_DEBUG
                        pclog("page 2 read from port %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: // PSTART
                                return (ne2000->page_start);

                        case 0x2: // PSTOP
                                return ne2000->page_stop;

                        case 0x3: // Remote Next-packet pointer
                                return ne2000->rempkt_ptr;

                        case 0x4: // TPSR
                                return ne2000->tx_page_start;

                        case 0x5: // Local Next-packet pointer
                                return ne2000->localpkt_ptr;

                        case 0x6: // Address counter (upper)
                                return ne2000->address_cnt >> 8;

                        case 0x7: // Address counter (lower)
                                return ne2000->address_cnt & 0xff;

                        case 0x8: // Reserved
                        case 0x9:
                        case 0xa:
                        case 0xb:
#ifdef NE2000_DEBUG
                                pclog("reserved read - page 2, 0x%02x\n", address);
#endif
                                break;

                        case 0xc: // RCR
                                return ((ne2000->RCR.monitor << 5) | (ne2000->RCR.promisc << 4) | (ne2000->RCR.multicast << 3) |
                                        (ne2000->RCR.broadcast << 2) | (ne2000->RCR.runts_ok << 1) | (ne2000->RCR.errors_ok));

                        case 0xd: // TCR
                                return ((ne2000->TCR.coll_prio << 4) | (ne2000->TCR.ext_stoptx << 3) |
                                        ((ne2000->TCR.loop_cntl & 0x3) << 1) | (ne2000->TCR.crc_disable));

                        case 0xe: // DCR
                                return (((ne2000->DCR.fifo_size & 0x3) << 5) | (ne2000->DCR.auto_rx << 4) |
                                        (ne2000->DCR.loop << 3) | (ne2000->DCR.longaddr << 2) | (ne2000->DCR.endian << 1) |
                                        (ne2000->DCR.wdsize));

                        case 0xf: // IMR
                                return ((ne2000->IMR.rdma_inte << 6) | (ne2000->IMR.cofl_inte << 5) |
                                        (ne2000->IMR.overw_inte << 4) | (ne2000->IMR.txerr_inte << 3) |
                                        (ne2000->IMR.rxerr_inte << 2) | (ne2000->IMR.tx_inte << 1) | (ne2000->IMR.rx_inte));
                        }
                        break;

                case 0x03:
                        if (ne2000->type != NE2000_RTL8029AS)
                                fatal("ne2000 unknown value of pgsel in read - %d\n", ne2000->CR.pgsel);
#ifdef NE2000_DEBUG
                        pclog("page 3 read from port %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: /*9346CR*/
                                return ne2000->_9346cr;

                        case 0x3:            /*CONFIG0*/
                                return 0x00; /*Cable not BNC*/

                        case 0x5:                              /*CONFIG2*/
                                return ne2000->config2 & 0xe0; /*No boot ROM*/

                        case 0x6: /*CONFIG3*/
                                return ne2000->config3 & 0x46;

                        case 0xe: /*8029ASID0*/
                                return 0x29;

                        case 0xf: /*8029ASID1*/
                                return 0x08;
                        }

                        pclog("reserved read - page 3, 0x%02x\n", address);
                        break;
                }
        }

        return ret;
}

//
// write_handler/write - i/o 'catcher' function called from BOCHS
// mainline when the CPU attempts a write in the i/o space registered
// by this ne2000 instance
//
void ne2000_write(uint16_t address, uint8_t value, ne2000_t *ne2000)
{
#ifdef NE2000_DEBUG
        pclog("write address %x, val=%x\n", address, value);
#endif
        address &= 0xf;

        //
        // The high 16 bytes of i/o space are for the ne2000 asic -
        //  the low 16 bytes are for the DS8390, with the current
        //  page being selected by the PS0,PS1 registers in the
        //  command register
        //
        if (address == 0x00) {
#ifdef NE2000_DEBUG
                pclog("wrote 0x%02x to CR\n", value);
#endif
                // Validate remote-DMA
                if ((value & 0x38) == 0x00) {
#ifdef NE2000_DEBUG
                        pclog("CR write - invalid rDMA value 0\n");
#endif
                        value |= 0x20; /* dma_cmd == 4 is a safe default */
                                       // value = 0x22; /* dma_cmd == 4 is a safe default */
                }

                // Check for s/w reset
                if (value & 0x01) {
                        ne2000->ISR.reset = 1;
                        ne2000->CR.stop = 1;
                } else
                        ne2000->CR.stop = 0;

                ne2000->CR.rdma_cmd = (value & 0x38) >> 3;

                // If start command issued, the RST bit in the ISR
                // must be cleared
                if ((value & 0x02) && !ne2000->CR.start)
                        ne2000->ISR.reset = 0;

                ne2000->CR.start = ((value & 0x02) == 0x02);
                ne2000->CR.pgsel = (value & 0xc0) >> 6;

                // Check for send-packet command
                if (ne2000->CR.rdma_cmd == 3) {
                        // Set up DMA read from receive ring
                        ne2000->remote_start = ne2000->remote_dma = ne2000->bound_ptr * 256;
                        ne2000->remote_bytes = *((uint16_t *)&ne2000->mem[ne2000->bound_ptr * 256 + 2 - BX_NE2K_MEMSTART]);
#ifdef NE2000_DEBUG
                        pclog("Sending buffer #x%x length %d\n", ne2000->remote_start, ne2000->remote_bytes);
#endif
                }

                // Check for start-tx
                if ((value & 0x04) && ne2000->TCR.loop_cntl) {
                        // loopback mode
                        if (ne2000->TCR.loop_cntl != 1) {
#ifdef NE2000_DEBUG
                                pclog("Loop mode %d not supported.\n", ne2000->TCR.loop_cntl);
#endif
                        } else {
                                ne2000_rx_frame(ne2000, &ne2000->mem[ne2000->tx_page_start * 256 - BX_NE2K_MEMSTART],
                                                ne2000->tx_bytes);

                                // do a TX interrupt
                                // Generate an interrupt if not masked and not one in progress
                                if (ne2000->IMR.tx_inte && !ne2000->ISR.pkt_tx) {
                                        // LOG_MSG("tx complete interrupt");
                                        ne2000_raise_irq(ne2000);
                                }
                                ne2000->ISR.pkt_tx = 1;
                        }
                } else if (value & 0x04) {
                        // start-tx and no loopback
#ifdef NE2000_DEBUG
                        if (ne2000->CR.stop || !ne2000->CR.start)
                                pclog("CR write - tx start, dev in reset\n");

                        if (ne2000->tx_bytes == 0)
                                pclog("CR write - tx start, tx bytes == 0\n");
#endif
                        // Send the packet to the system driver
                        /* TODO: Transmit packet */
                        // BX_NE2K_THIS ethdev->sendpkt(& ne2000->mem[ne2000->tx_page_start*256 - BX_NE2K_MEMSTART],
                        // ne2000->tx_bytes); pcap_sendpacket(adhandle,&ne2000->mem[ne2000->tx_page_start*256 - BX_NE2K_MEMSTART],
                        // ne2000->tx_bytes);
                        if (net_pcap != NULL) {
                                pcap_sendpacket(net_pcap, &ne2000->mem[ne2000->tx_page_start * 256 - BX_NE2K_MEMSTART],
                                                ne2000->tx_bytes);
#ifdef NE2000_DEBUG
                                pclog("ne2000 pcap sending packet\n");
#endif
                        }
                        ne2000_tx_event(value, ne2000);
                        // Schedule a timer to trigger a tx-complete interrupt
                        // The number of microseconds is the bit-time / 10.
                        // The bit-time is the preamble+sfd (64 bits), the
                        // inter-frame gap (96 bits), the CRC (4 bytes), and the
                        // the number of bits in the frame (s.tx_bytes * 8).
                        //

                        /* TODO: Code transmit timer */
                        /*
                        bx_pc_system.activate_timer(ne2000->tx_timer_index,
                                (64 + 96 + 4*8 + ne2000->tx_bytes*8)/10,
                        0); // not continuous
                        */
                } // end transmit-start branch

                // Linux probes for an interrupt by setting up a remote-DMA read
                // of 0 bytes with remote-DMA completion interrupts enabled.
                // Detect this here
                if (ne2000->CR.rdma_cmd == 0x01 && ne2000->CR.start && ne2000->remote_bytes == 0) {
                        ne2000->ISR.rdma_done = 1;
                        if (ne2000->IMR.rdma_inte) {
                                ne2000_raise_irq(ne2000);
                                // DEV_pic_raise_irq(ne2000->base_irq);
                        }
                }
        } else {
                switch (ne2000->CR.pgsel) {
                case 0x00:
#ifdef NE2000_DEBUG
                        pclog("page 0 write to port %04x\n", address);
#endif
                        // It appears to be a common practice to use outw on page0 regs...

                        switch (address) {
                        case 0x1: // PSTART
                                ne2000->page_start = value;
                                break;

                        case 0x2: // PSTOP
                                // BX_INFO(("Writing to PSTOP: %02x", value));
                                ne2000->page_stop = value;
                                break;

                        case 0x3: // BNRY
                                ne2000->bound_ptr = value;
                                break;

                        case 0x4: // TPSR
                                ne2000->tx_page_start = value;
                                break;

                        case 0x5: // TBCR0
                                // Clear out low byte and re-insert
                                ne2000->tx_bytes &= 0xff00;
                                ne2000->tx_bytes |= (value & 0xff);
                                break;

                        case 0x6: // TBCR1
                                // Clear out high byte and re-insert
                                ne2000->tx_bytes &= 0x00ff;
                                ne2000->tx_bytes |= ((value & 0xff) << 8);
                                break;

                        case 0x7:              // ISR
                                value &= 0x7f; // clear RST bit - status-only bit
                                // All other values are cleared iff the ISR bit is 1
                                ne2000->ISR.pkt_rx &= ~((int)((value & 0x01) == 0x01) ? ~0:0);
                                ne2000->ISR.pkt_tx &= ~((int)((value & 0x02) == 0x02) ? ~0:0);
                                ne2000->ISR.rx_err &= ~((int)((value & 0x04) == 0x04) ? ~0:0);
                                ne2000->ISR.tx_err &= ~((int)((value & 0x08) == 0x08) ? ~0:0);
                                ne2000->ISR.overwrite &= ~((int)((value & 0x10) == 0x10) ? ~0:0);
                                ne2000->ISR.cnt_oflow &= ~((int)((value & 0x20) == 0x20) ? ~0:0);
                                ne2000->ISR.rdma_done &= ~((int)((value & 0x40) == 0x40) ? ~0:0);
                                value = ((ne2000->ISR.rdma_done << 6) | (ne2000->ISR.cnt_oflow << 5) |
                                         (ne2000->ISR.overwrite << 4) | (ne2000->ISR.tx_err << 3) | (ne2000->ISR.rx_err << 2) |
                                         (ne2000->ISR.pkt_tx << 1) | (ne2000->ISR.pkt_rx));
                                value &= ((ne2000->IMR.rdma_inte << 6) | (ne2000->IMR.cofl_inte << 5) |
                                          (ne2000->IMR.overw_inte << 4) | (ne2000->IMR.txerr_inte << 3) |
                                          (ne2000->IMR.rxerr_inte << 2) | (ne2000->IMR.tx_inte << 1) | (ne2000->IMR.rx_inte));
                                if (value == 0)
                                        ne2000_lower_irq(ne2000);
                                // DEV_pic_lower_irq(ne2000->base_irq);
                                break;

                        case 0x8: // RSAR0
                                // Clear out low byte and re-insert
                                ne2000->remote_start &= 0xff00;
                                ne2000->remote_start |= (value & 0xff);
                                ne2000->remote_dma = ne2000->remote_start;
                                break;

                        case 0x9: // RSAR1
                                // Clear out high byte and re-insert
                                ne2000->remote_start &= 0x00ff;
                                ne2000->remote_start |= ((value & 0xff) << 8);
                                ne2000->remote_dma = ne2000->remote_start;
                                break;

                        case 0xa: // RBCR0
                                // Clear out low byte and re-insert
                                ne2000->remote_bytes &= 0xff00;
                                ne2000->remote_bytes |= (value & 0xff);
                                break;

                        case 0xb: // RBCR1
                                // Clear out high byte and re-insert
                                ne2000->remote_bytes &= 0x00ff;
                                ne2000->remote_bytes |= ((value & 0xff) << 8);
                                break;

                        case 0xc: // RCR
                                  // Check if the reserved bits are set
#ifdef NE2000_DEBUG
                                if (value & 0xc0)
                                        pclog("RCR write, reserved bits set\n");
#endif
                                // Set all other bit-fields
                                ne2000->RCR.errors_ok = ((value & 0x01) == 0x01);
                                ne2000->RCR.runts_ok = ((value & 0x02) == 0x02);
                                ne2000->RCR.broadcast = ((value & 0x04) == 0x04);
                                ne2000->RCR.multicast = ((value & 0x08) == 0x08);
                                ne2000->RCR.promisc = ((value & 0x10) == 0x10);
                                ne2000->RCR.monitor = ((value & 0x20) == 0x20);

                                // Monitor bit is a little suspicious...
#ifdef NE2000_DEBUG
                                if (value & 0x20)
                                        pclog("RCR write, monitor bit set!\n");
#endif
                                break;

                        case 0xd: // TCR
                                  // Check reserved bits
#ifdef NE2000_DEBUG
                                if (value & 0xe0)
                                        pclog("TCR write, reserved bits set\n");
#endif
                                // Test loop mode (not supported)
                                if (value & 0x06) {
                                        ne2000->TCR.loop_cntl = (value & 0x6) >> 1;
#ifdef NE2000_DEBUG
                                        pclog("TCR write, loop mode %d not supported\n", ne2000->TCR.loop_cntl);
#endif
                                } else
                                        ne2000->TCR.loop_cntl = 0;

                                // Inhibit-CRC not supported.
                                if (value & 0x01) {
                                        // fatal("ne2000 TCR write, inhibit-CRC not supported\n");
#ifdef NE2000_DEBUG
                                        pclog("ne2000 TCR write, inhibit-CRC not supported\n");
#endif
                                        return;
                                }

                                // Auto-transmit disable very suspicious
                                if (value & 0x08) {
                                        // fatal("ne2000 TCR write, auto transmit disable not supported\n");
#ifdef NE2000_DEBUG
                                        pclog("ne2000 TCR write, auto transmit disable not supported\n");
#endif
                                }

                                // Allow collision-offset to be set, although not used
                                ne2000->TCR.coll_prio = ((value & 0x08) == 0x08);
                                break;

                        case 0xe: // DCR
                                  // the loopback mode is not suppported yet
#ifdef NE2000_DEBUG
                                if (!(value & 0x08))
                                        pclog("DCR write, loopback mode selected\n");

                                // It is questionable to set longaddr and auto_rx, since they
                                // aren't supported on the ne2000. Print a warning and continue
                                if (value & 0x04)
                                        pclog("DCR write - LAS set ???\n");
                                if (value & 0x10)
                                        pclog("DCR write - AR set ???\n");
#endif
                                // Set other values.
                                ne2000->DCR.wdsize = ((value & 0x01) == 0x01);
                                ne2000->DCR.endian = ((value & 0x02) == 0x02);
                                ne2000->DCR.longaddr = ((value & 0x04) == 0x04); // illegal ?
                                ne2000->DCR.loop = ((value & 0x08) == 0x08);
                                ne2000->DCR.auto_rx = ((value & 0x10) == 0x10); // also illegal ?
                                ne2000->DCR.fifo_size = (value & 0x50) >> 5;
                                break;

                        case 0xf: // IMR
                                  // Check for reserved bit
#ifdef NE2000_DEBUG
                                if (value & 0x80)
                                        pclog("IMR write, reserved bit set\n");
#endif
                                // Set other values
                                ne2000->IMR.rx_inte = ((value & 0x01) == 0x01);
                                ne2000->IMR.tx_inte = ((value & 0x02) == 0x02);
                                ne2000->IMR.rxerr_inte = ((value & 0x04) == 0x04);
                                ne2000->IMR.txerr_inte = ((value & 0x08) == 0x08);
                                ne2000->IMR.overw_inte = ((value & 0x10) == 0x10);
                                ne2000->IMR.cofl_inte = ((value & 0x20) == 0x20);
                                ne2000->IMR.rdma_inte = ((value & 0x40) == 0x40);
                                value = ((ne2000->ISR.rdma_done << 6) | (ne2000->ISR.cnt_oflow << 5) |
                                         (ne2000->ISR.overwrite << 4) | (ne2000->ISR.tx_err << 3) | (ne2000->ISR.rx_err << 2) |
                                         (ne2000->ISR.pkt_tx << 1) | (ne2000->ISR.pkt_rx));
                                value &= ((ne2000->IMR.rdma_inte << 6) | (ne2000->IMR.cofl_inte << 5) |
                                          (ne2000->IMR.overw_inte << 4) | (ne2000->IMR.txerr_inte << 3) |
                                          (ne2000->IMR.rxerr_inte << 2) | (ne2000->IMR.tx_inte << 1) | (ne2000->IMR.rx_inte));
                                if (value)
                                        ne2000_raise_irq(ne2000);
                                else
                                        ne2000_lower_irq(ne2000);
                                break;
                        }
                        break;

                case 0x01:
#ifdef NE2000_DEBUG
                        pclog("page 1 w offset %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: // PAR0-5
                        case 0x2:
                        case 0x3:
                        case 0x4:
                        case 0x5:
                        case 0x6:
                                ne2000->physaddr[address - 1] = value;
                                break;

                        case 0x7: // CURR
                                ne2000->curr_page = value;
                                break;

                        case 0x8: // MAR0-7
                        case 0x9:
                        case 0xa:
                        case 0xb:
                        case 0xc:
                        case 0xd:
                        case 0xe:
                        case 0xf:
                                ne2000->mchash[address - 8] = value;
                                break;
                        }
                        break;

                case 0x02:
#ifdef NE2000_DEBUG
                        if (address != 0)
                                pclog("page 2 write ?\n");
#endif
                        switch (address) {
                        case 0x1: // CLDA0
                                // Clear out low byte and re-insert
                                ne2000->local_dma &= 0xff00;
                                ne2000->local_dma |= (value & 0xff);
                                break;

                        case 0x2: // CLDA1
                                // Clear out high byte and re-insert
                                ne2000->local_dma &= 0x00ff;
                                ne2000->local_dma |= ((value & 0xff) << 8);
                                break;

                        case 0x3: // Remote Next-pkt pointer
                                ne2000->rempkt_ptr = value;
                                break;

                        case 0x4:
                                // fatal("page 2 write to reserved offset 4\n");
                                // OS/2 Warp can cause this to freak out.
#ifdef NE2000_DEBUG
                                pclog("ne2000 page 2 write to reserved offset 4\n");
#endif
                                break;

                        case 0x5: // Local Next-packet pointer
                                ne2000->localpkt_ptr = value;
                                break;

                        case 0x6: // Address counter (upper)
                                // Clear out high byte and re-insert
                                ne2000->address_cnt &= 0x00ff;
                                ne2000->address_cnt |= ((value & 0xff) << 8);
                                break;

                        case 0x7: // Address counter (lower)
                                // Clear out low byte and re-insert
                                ne2000->address_cnt &= 0xff00;
                                ne2000->address_cnt |= (value & 0xff);
                                break;

                        case 0x8:
                        case 0x9:
                        case 0xa:
                        case 0xb:
                        case 0xc:
                        case 0xd:
                        case 0xe:
                        case 0xf:
                                // fatal("page 2 write to reserved offset %0x\n", address);
#ifdef NE2000_DEBUG
                                pclog("ne2000 page 2 write to reserved offset %0x\n", address);
#endif
                        default:
                                break;
                        }
                        break;

                case 0x03:
                        if (ne2000->type != NE2000_RTL8029AS) {
                                pclog("ne2000 unknown value of pgsel in write - %d\n", ne2000->CR.pgsel);
                                break;
                        }
#ifdef NE2000_DEBUG
                        pclog("page 3 write to port %04x\n", address);
#endif
                        switch (address) {
                        case 0x1: /*9346CR*/
                                ne2000->_9346cr = value & 0xfe;
                                break;

                        case 0x5: /*CONFIG2*/
                                ne2000->config2 = value & 0xe0;
                                break;

                        case 0x6: /*CONFIG3*/
                                ne2000->config3 = value & 0x46;
                                break;

                        case 0x9: /*HLTCLK*/
                                break;

                        default:
                                pclog("reserved write - page 3, 0x%02x\n", address);
                                break;
                        }
                        break;
                }
        }
}

/*
 * mcast_index() - return the 6-bit index into the multicast
 * table. Stolen unashamedly from FreeBSD's if_ed.c
 */
static int mcast_index(const void *dst) {
#define POLYNOMIAL 0x04c11db6
        unsigned long crc = 0xffffffffL;
        int carry, i, j;
        unsigned char b;
        unsigned char *ep = (unsigned char *)dst;

        for (i = 6; --i >= 0;) {
                b = *ep++;
                for (j = 8; --j >= 0;) {
                        carry = ((crc & 0x80000000L) ? 1 : 0) ^ (b & 0x01);
                        crc <<= 1;
                        b >>= 1;
                        if (carry)
                                crc = ((crc ^ POLYNOMIAL) | carry);
                }
        }
        return crc >> 26;
#undef POLYNOMIAL
}

/*
 * rx_frame() - called by the platform-specific code when an
 * ethernet frame has been received. The destination address
 * is tested to see if it should be accepted, and if the
 * rx ring has enough room, it is copied into it and
 * the receive process is updated
 */
void ne2000_rx_frame(ne2000_t *ne2000, const void *buf, int io_len) 
{
        int pages;
        int avail;
        int idx;
        int nextpage;
        uint8_t pkthdr[4];
        uint8_t *pktbuf = (uint8_t *)buf;
        uint8_t *startptr;
        static uint8_t bcast_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

        // pclog("ne2000_rx_frame len=%d\n", io_len);
        // for(int q=0; q<io_len; q++){
        //     pclog("%02x ", pktbuf[q]);
        //     if(q % 16 == 15)
        //         pclog("\n");
        // }

#ifdef NE2000_DEBUG
        pclog("rx_frame with length %d\n", io_len);
#endif
        // LOG_MSG("stop=%d, pagestart=%x, dcr_loop=%x, tcr_loopcntl=%x",
        //	ne2000->CR.stop, ne2000->page_start,
        //	ne2000->DCR.loop, ne2000->TCR.loop_cntl);
        if ((ne2000->CR.stop != 0) ||
		(ne2000->page_start == 0) /*||
                ((ne2000->DCR.loop == 0) &&
                (ne2000->TCR.loop_cntl != 0))*/)
                return;

        // Add the pkt header + CRC to the length, and work
        // out how many 256-byte pages the frame would occupy
        pages = (io_len + 4 + 4 + 255) / 256;

        if (ne2000->curr_page < ne2000->bound_ptr)
                avail = ne2000->bound_ptr - ne2000->curr_page;
        else
                avail = (ne2000->page_stop - ne2000->page_start) - (ne2000->curr_page - ne2000->bound_ptr);

        // Avoid getting into a buffer overflow condition by not attempting
        // to do partial receives. The emulation to handle this condition
        // seems particularly painful.
        if ((avail < pages)
#if BX_NE2K_NEVER_FULL_RING
            || (avail == pages)
#endif
        ) {
#ifdef NE2000_DEBUG
                pclog("no space\n");
#endif
                return;
        }

        if ((io_len < 40 /*60*/) && !ne2000->RCR.runts_ok) {
#ifdef NE2000_DEBUG
                pclog("rejected small packet, length %d\n", io_len);
#endif
                return;
        }
        // some computers don't care...
        if (io_len < 60)
                io_len = 60;

        // Do address filtering if not in promiscuous mode
        if (!ne2000->RCR.promisc) {
                if (!memcmp(buf, bcast_addr, 6)) {
                        if (!ne2000->RCR.broadcast)
                                return;
                } else if (pktbuf[0] & 0x01) {
                        if (!ne2000->RCR.multicast)
                                return;
                        idx = mcast_index(buf);
                        if (!(ne2000->mchash[idx >> 3] & (1 << (idx & 0x7))))
                                return;
                } else if (0 != memcmp(buf, ne2000->physaddr, 6))
                        return;
        }
#ifdef NE2000_DEBUG
        else
                pclog("rx_frame promiscuous receive\n");

        pclog("rx_frame %d to %x:%x:%x:%x:%x:%x from %x:%x:%x:%x:%x:%x\n", io_len, pktbuf[0], pktbuf[1], pktbuf[2], pktbuf[3],
              pktbuf[4], pktbuf[5], pktbuf[6], pktbuf[7], pktbuf[8], pktbuf[9], pktbuf[10], pktbuf[11]);
#endif
        nextpage = ne2000->curr_page + pages;
        if (nextpage >= ne2000->page_stop)
                nextpage -= ne2000->page_stop - ne2000->page_start;

        // Setup packet header
        pkthdr[0] = 0; // rx status - old behavior
        pkthdr[0] = 1; // Probably better to set it all the time
        // rather than set it to 0, which is clearly wrong.
        if (pktbuf[0] & 0x01)
                pkthdr[0] |= 0x20; // rx status += multicast packet

        pkthdr[1] = nextpage;            // ptr to next packet
        pkthdr[2] = (io_len + 4) & 0xff; // length-low
        pkthdr[3] = (io_len + 4) >> 8;   // length-hi
#ifdef NE2000_DEBUG
        pclog("ne2000: header %02x %02x %02x %02x\n", pkthdr[0], pkthdr[1], pkthdr[2], pkthdr[3]);
#endif

        // copy into buffer, update curpage, and signal interrupt if config'd
        startptr = &ne2000->mem[ne2000->curr_page * 256 - BX_NE2K_MEMSTART];
        if ((nextpage > ne2000->curr_page) || ((ne2000->curr_page + pages) == ne2000->page_stop)) {
                memcpy(startptr, pkthdr, 4);
                memcpy(startptr + 4, buf, io_len);
                ne2000->curr_page = nextpage;
        } else {
                int endbytes = (ne2000->page_stop - ne2000->curr_page) * 256;
                memcpy(startptr, pkthdr, 4);
                memcpy(startptr + 4, buf, endbytes - 4);
                startptr = &ne2000->mem[ne2000->page_start * 256 - BX_NE2K_MEMSTART];
                memcpy(startptr, (void *)(pktbuf + endbytes - 4), io_len - endbytes + 8);
                ne2000->curr_page = nextpage;
        }

        ne2000->RSR.rx_ok = 1;
        if (pktbuf[0] & 0x80)
                ne2000->RSR.rx_mbit = 1;

        ne2000->ISR.pkt_rx = 1;

        if (ne2000->IMR.rx_inte) {
                // LOG_MSG("packet rx interrupt");
                ne2000_raise_irq(ne2000);
                // DEV_pic_raise_irq(ne2000->base_irq);
        }
        // else LOG_MSG("no packet rx interrupt");
}

void ne2000_tx_timer(ne2000_t *ne2000)
{
#ifdef NE2000_DEBUG
        pclog("tx_timer\n");
#endif
        ne2000->TSR.tx_ok = 1;
        // Generate an interrupt if not masked and not one in progress
        if (ne2000->IMR.tx_inte && !ne2000->ISR.pkt_tx) {
                // LOG_MSG("tx complete interrupt");
                ne2000_raise_irq(ne2000);
                // DEV_pic_raise_irq(ne2000->base_irq);
        }
        // else 	  LOG_MSG("no tx complete interrupt");
        ne2000->ISR.pkt_tx = 1;
        ne2000->tx_timer_active = 0;
}

static void ne2000_tx_event(int val, void *p) {
        ne2000_t *ne2000 = (ne2000_t *)p;
        ne2000_tx_timer(ne2000);
}

// static void ne2000_pci_add(ne2000_t *ne2000) {
//         io_sethandler(ne2000->base_addr, 0x0010, ne2000_read, NULL, NULL, ne2000_write, NULL, NULL, ne2000);
//         io_sethandler(ne2000->base_addr + 0x10, 0x000f, ne2000_asic_read_b, ne2000_asic_read_w, NULL, ne2000_asic_write_b,
//                       ne2000_asic_write_w, NULL, ne2000);
//         io_sethandler(ne2000->base_addr + 0x1f, 0x0001, ne2000_reset_read, NULL, NULL, ne2000_reset_write, NULL, NULL, ne2000);
// }
// 
// static void ne2000_pci_remove(ne2000_t *ne2000) {
//         io_removehandler(ne2000->base_addr, 0x0010, ne2000_read, NULL, NULL, ne2000_write, NULL, NULL, ne2000);
//         io_removehandler(ne2000->base_addr + 0x10, 0x000f, ne2000_asic_read_b, ne2000_asic_read_w, NULL, ne2000_asic_write_b,
//                          ne2000_asic_write_w, NULL, ne2000);
//         io_removehandler(ne2000->base_addr + 0x1f, 0x0001, ne2000_reset_read, NULL, NULL, ne2000_reset_write, NULL, NULL, ne2000);
// }

// static uint8_t rtl8029_pci_read(int func, int addr, void *p) {
//         ne2000_t *ne2000 = (ne2000_t *)p;
// 
//         if (func)
//                 return 0;
// 
//         //        pclog("RTL8029 PCI read %08X PC=%08x\n", addr, cpu_state.pc);
// 
//         switch (addr) {
//         case 0x00:
//                 return 0xec; /*Realtek*/
//         case 0x01:
//                 return 0x10;
// 
//         case 0x02:
//                 return 0x29; /*RTL8029AS*/
//         case 0x03:
//                 return 0x80;
// 
//         case 0x04:
//                 return ne2000->pci_command;
// 
//         case 0x07:
//                 return 0x02;
// 
//         case 0x08:
//                 return 0; /*Revision ID*/
//         case 0x09:
//                 return 0; /*Programming interface*/
//         case 0x0a:
//                 return 0; /*Ethernet controller*/
//         case 0x0b:
//                 return 0x02; /*Network controller*/
// 
//         case 0x10:
//                 return 0x01 | (ne2000->base_addr & 0xe0); /*memBaseAddr*/
//         case 0x11:
//                 return ne2000->base_addr >> 8;
//         case 0x12:
//                 return ne2000->base_addr >> 16;
//         case 0x13:
//                 return ne2000->base_addr >> 24;
// 
//         case 0x2c:
//                 return 0xec; /*Subsystem vendor - Realtek*/
//         case 0x2d:
//                 return 0x10;
// 
//         case 0x2e:
//                 return 0x29; /*Subsystem ID - RTL8029AS*/
//         case 0x2f:
//                 return 0x80;
// 
//         case 0x3c:
//                 return ne2000->int_line;
//         case 0x3d:
//                 return 0x01; /*INTA*/
//         }
//         return 0;
// }
// 
// static void rtl8029_pci_write(int func, int addr, uint8_t val, void *p) {
//         ne2000_t *ne2000 = (ne2000_t *)p;
// 
//         if (func)
//                 return;
// 
//         //        pclog("RTL8029 PCI write %04X %02X PC=%08x\n", addr, val, cpu_state.pc);
// 
//         switch (addr) {
//         case 0x04:
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_remove(ne2000);
//                 ne2000->pci_command = val & 3;
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_add(ne2000);
//                 break;
// 
//         case 0x10:
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_remove(ne2000);
//                 ne2000->base_addr = (ne2000->base_addr & 0xffffff00) | (val & 0xe0);
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_add(ne2000);
//                 break;
//         case 0x11:
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_remove(ne2000);
//                 ne2000->base_addr = (ne2000->base_addr & 0xffff00e0) | (val << 8);
//                 if (ne2000->pci_command & PCI_COMMAND_IO)
//                         ne2000_pci_add(ne2000);
//                 break;
//         case 0x12:
//                 ne2000->base_addr = (ne2000->base_addr & 0xff00ffe0) | (val << 16);
//                 break;
//         case 0x13:
//                 ne2000->base_addr = (ne2000->base_addr & 0x00ffffe0) | (val << 24);
//                 break;
// 
//         case 0x3c:
//                 ne2000->int_line = val;
//                 break;
//         }
// }

void ne2000_poller(ne2000_t *ne2000)
{
        const unsigned char *data;
        struct pcap_pkthdr h;
        if (net_pcap != NULL) {
                data = pcap_next(net_pcap, &h);
                if (data == 0x0)
                        return;
                if ((memcmp(data + 6, maclocal, 6)) == 0)
                        pclog("ne2000 we just saw ourselves\n");
                else {
                        if ((ne2000->DCR.loop == 0) || (ne2000->TCR.loop_cntl != 0))
                                return;

#ifdef NE2000_DEBUG
                        pclog("ne2000 pcap received a frame %d bytes\n", h.caplen);
#endif
                        ne2000_rx_frame(ne2000, data, h.caplen);
                }
        }
}

ne2000_t *ne2000_common_init(const char *devname) 
{
        int rc;

        ne2000_t *ne2000 = malloc(sizeof(ne2000_t));
        memset(ne2000, 0, sizeof(ne2000_t));

        // maclocal[0] = 0x10;
        // maclocal[1] = 0xf6;
        // maclocal[2] = 0x0a;
        // maclocal[3] = 0xdc;
        // maclocal[4] = 0x4d;
        // maclocal[5] = 0x8f;
        maclocal[0] = 0xac;
        maclocal[1] = 0xde;
        maclocal[2] = 0x48;
        maclocal[3] = 0x88;
        maclocal[4] = 0xbb;
        maclocal[5] = 0xaa;
        pclog("ne2000: Using default MAC address (AC:DE:48:88:BB:AA)\n");

        memcpy(ne2000->physaddr, maclocal, 6);

        ne2000_reset(BX_RESET_HARDWARE, ne2000);
        //vlan_handler(ne2000_poller, ne2000);

        // need a switch statment for more network types.

        if (1) { // pcap
                char errbuf[32768];

                pclog("ne2000 initalizing libpcap\n");

                pclog("ne2000 Pcap version [%s]\n", pcap_lib_version());

                if ((net_pcap = pcap_open_live(devname, 1518, 1, 15, errbuf)) == 0) {
                    pclog("ne2000 pcap_open_live error on %s!\n", devname);
                    free(ne2000);
                    return NULL;
                }

                // Time to check that we are in non-blocking mode.
                rc = pcap_getnonblock(net_pcap, errbuf);
                pclog("ne2000 pcap is currently in %s mode\n", rc ? "non-blocking" : "blocking");
                switch (rc) {
                case 0:
                        pclog("ne2000 Setting interface to non-blocking mode..");
                        rc = pcap_setnonblock(net_pcap, 1, errbuf);
                        if (rc == 0) {
                                pclog("..");
                                rc = pcap_getnonblock(net_pcap, errbuf);
                                if (rc == 1) {
                                        pclog("..! rc=%d", rc);
                                        //net_is_pcap = 1;
                                } else {
                                        pclog("\tunable to set pcap into non-blocking mode!\nContinuining without pcap.\n");
                                        //net_is_pcap = 0;
                                }
                        } else {
                                pclog("There was an unexpected error of [%s]\n\nexiting.\n", errbuf);
                                //net_is_pcap = 0;
                        }
                        pclog("\n");
                        break;
                case 1:
                        pclog("non blocking\n");
                        break;
                default:
                        pclog("this isn't right!!!\n");
                        //net_is_pcap = 0;
                        break;
                }
                if (1) {
                    struct bpf_program fp;
                    char filter_exp[255];

                    pclog("ne2000 Building packet filter...");
                    sprintf(filter_exp,
                            "( ((ether dst ff:ff:ff:ff:ff:ff) or (ether dst %02x:%02x:%02x:%02x:%02x:%02x)) and not "
                            "(ether src %02x:%02x:%02x:%02x:%02x:%02x) )",
                            maclocal[0], maclocal[1], maclocal[2], maclocal[3], maclocal[4], maclocal[5], maclocal[0],
                            maclocal[1], maclocal[2], maclocal[3], maclocal[4], maclocal[5]);

                    // I'm doing a MAC level filter so TCP/IP doesn't matter.
                    if (pcap_compile(net_pcap, &fp, filter_exp, 0, 0xffffffff) == -1) {
                        pclog("\nne2000 Couldn't compile filter\n");
                    } else {
                        pclog("...");
                        if (pcap_setfilter(net_pcap, &fp) == -1) {
                            pclog("\nError installing pcap filter.\n");
                        } else {
                            pclog("...!\n");
                        }
                    }
                }
        } // end pcap setup

        return ne2000;
}

ne2000_t *ne2000_init(const char *devname) 
{
        ne2000_t *ne2000 = ne2000_common_init(devname);

        if(!ne2000)
            return NULL;

        ne2000->type = NE2000_NE2000;

        // ne2000_setirq(ne2000, 5);

        // addr = 0x300;
        // io_sethandler(addr, 0x0010, ne2000_read, NULL, NULL, ne2000_write, NULL, NULL, ne2000);
        // io_sethandler(addr + 0x10, 0x0010, ne2000_asic_read_b, ne2000_asic_read_w, NULL, ne2000_asic_write_b, ne2000_asic_write_w,
        //               NULL, ne2000);
        // io_sethandler(addr + 0x1f, 0x0001, ne2000_reset_read, NULL, NULL, ne2000_reset_write, NULL, NULL, ne2000);

        return ne2000;
}

// void *rtl8029_init() {
//         ne2000_t *ne2000 = ne2000_common_init();
// 
//         ne2000->is_pci = 1;
//         ne2000->type = NE2000_RTL8029AS;
// 
//         ne2000->card = pci_add(rtl8029_pci_read, rtl8029_pci_write, ne2000);
// 
//         ne2000_setirq(ne2000, NE2000_PCI_IRQ);
// 
//         return ne2000;
// }

void ne2000_close(ne2000_t *ne2000) 
{
        free(ne2000);
        if (net_pcap != NULL) {
                pcap_close(net_pcap);
                pclog("ne2000 closing pcap\n");
        }
        pclog("ne2000 close\n");
}

// static device_config_t ne2000_config[] = {{.name = "addr",
//                                            .description = "Address",
//                                            .type = CONFIG_BINARY,
//                                            .type = CONFIG_SELECTION,
//                                            .selection = {{.description = "0x280", .value = 0x280},
//                                                          {.description = "0x300", .value = 0x300},
//                                                          {.description = "0x320", .value = 0x320},
//                                                          {.description = "0x340", .value = 0x340},
//                                                          {.description = "0x360", .value = 0x360},
//                                                          {.description = "0x380", .value = 0x380},
//                                                          {.description = ""}},
//                                            .default_int = 0x300},
//                                           {.name = "irq",
//                                            .description = "IRQ",
//                                            .type = CONFIG_SELECTION,
//                                            .selection = {{.description = "IRQ 3", .value = 3},
//                                                          {.description = "IRQ 5", .value = 5},
//                                                          {.description = "IRQ 7", .value = 7},
//                                                          {.description = "IRQ 10", .value = 10},
//                                                          {.description = "IRQ 11", .value = 11},
//                                                          {.description = "IRQ 12", .value = 12},
//                                                          {.description = ""}},
//                                            .default_int = 10},
//                                           {.type = -1}};
// 
// device_t ne2000_device = {"Novell NE2000", 0, ne2000_init, ne2000_close, NULL, NULL, NULL, NULL, ne2000_config};
// 
// device_t rtl8029as_device = {"Realtek RTL8029AS", DEVICE_PCI, rtl8029_init, ne2000_close, NULL, NULL, NULL, NULL, NULL};
