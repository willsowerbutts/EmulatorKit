#ifndef __NS202_HEADER__
#define __NS202_HEADER__

#include <stdint.h>

#define NS202_NO_INT	0xFF

void ns202_trace(int on);
void ns202_reset(void);
void ns202_tick(unsigned clocks);
void ns202_raise(unsigned irq);
void ns202_write(unsigned int address, unsigned int value);
unsigned int ns202_read(unsigned int address);
unsigned int ns202_int_ack(void);
unsigned int ns202_irq_asserted(void);

#endif
