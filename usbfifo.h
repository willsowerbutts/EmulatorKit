#ifndef __ECB_USB_FIFO_HEADER__
#define __ECB_USB_FIFO_HEADER__

typedef struct {
    int connection;         /* fd for live connection */
    int irq;                /* output to CPU */
    int status;             /* status register contents */
} ecb_usb_fifo_state;

void ecb_usb_fifo_reset(ecb_usb_fifo_state *fifo);
void ecb_usb_fifo_create(ecb_usb_fifo_state *fifo);
int ecb_usb_fifo_connect(ecb_usb_fifo_state *fifo, const char *host, const char *port);
void ecb_usb_fifo_destroy(ecb_usb_fifo_state *fifo);
unsigned char ecb_usb_fifo_read_data(ecb_usb_fifo_state *fifo);
unsigned char ecb_usb_fifo_read_status(ecb_usb_fifo_state *fifo);
void ecb_usb_fifo_write_data(ecb_usb_fifo_state *fifo, unsigned char value);
void ecb_usb_fifo_write_status(ecb_usb_fifo_state *fifo, unsigned char value);
int ecb_usb_fifo_get_irq(ecb_usb_fifo_state *fifo);

/* bits in USB_FIFO_STATUS register */
#define USB_FIFO_STATUS_TXF             0x01 /* 0 = transmit FIFO space available, 1 = transmit FIFO full */
#define USB_FIFO_STATUS_INT_ENABLE      0x02 /* 1 = interrupt request enabled */
#define USB_FIFO_STATUS_INT_TXE         0x04 /* 1 = request interrupt on transmit FIFO space available */
#define USB_FIFO_STATUS_INT_RXF         0x08 /* 1 = request interrupt on receive FIFO data waiting */
#define USB_FIFO_STATUS_GPIO_IN         0x10 /* GPIO bit: data from ECB to USB (read/write) */
#define USB_FIFO_STATUS_GPIO_OUT        0x20 /* GPIO bit: data from USB to ECB (read only) */
#define USB_FIFO_STATUS_IRQ             0x40 /* 1 when interrupt would be requested (regardless of INT_ENABLE bit) */
#define USB_FIFO_STATUS_RXE             0x80 /* 0 = receive FIFO data waiting, 1 = receive FIFO empty */

#endif
