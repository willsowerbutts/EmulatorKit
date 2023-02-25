/*
 *	ECB-USB-FIFO
 *
 *	To allow connection to other applications, the emulated FIFO
 *	is presented to the outside world as a TCP/IP connection.
 *
 */

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include "usbfifo.h"

#define NODATA 0xFF

void ecb_usb_fifo_reset(ecb_usb_fifo_state *fifo)
{
    fifo->status = 0;
    fifo->irq = 0;
    /* do not reset next_read here */
}

void ecb_usb_fifo_create(ecb_usb_fifo_state *fifo)
{
    memset(fifo, 0, sizeof(ecb_usb_fifo_state));
    fifo->connection = -1;
    ecb_usb_fifo_reset(fifo);
}

int ecb_usb_fifo_connect(ecb_usb_fifo_state *fifo, const char *host, const char *port)
{
    struct addrinfo hints, *services, *serv;
    int r;

    if(fifo->connection >= 0){
        fprintf(stderr, "ecb_usb_fifo_connect: already connected!\n");
        return -1;
    }

    /* now we mess with the sockets */
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    r = getaddrinfo(host, port, &hints, &services);
    if(r){
        fprintf(stderr, "ecb_usb_fifo_connect: getaddrinfo failed: %s\n", gai_strerror(r));
        return -1;
    }

    /* try each possible address returned */
    for(serv=services; serv; serv=serv->ai_next){
        fifo->connection = socket(serv->ai_family, serv->ai_socktype, serv->ai_protocol);
        if(fifo->connection == -1)
            continue;
        if(connect(fifo->connection, serv->ai_addr, serv->ai_addrlen) == -1){
            close(fifo->connection);
            fifo->connection = -1;
            continue;
        }
        /* if we get here, we're connected! */
        break;
    }

    /* none of the addresses worked? */
    if(!serv){
        fprintf(stderr, "ecb_usb_fifo_connect: connection failed\n");
        return -2;
    }

    /* done with the list returned by getaddrinfo() */
    freeaddrinfo(services);

    /* mark non-blocking */
    if(fcntl(fifo->connection, F_SETFL, fcntl(fifo->connection, F_GETFL, 0) | O_NONBLOCK) < 0){
        fprintf(stderr, "ecb_usb_fifo_connect: cannot set O_NONBLOCK\n");
        close(fifo->connection);
        return -3;
    }

    fprintf(stderr, "ecb_usb_fifo_connect: connected to %s:%s\n", host, port);

    return 0;
}

void ecb_usb_fifo_destroy(ecb_usb_fifo_state *fifo)
{
    if(fifo->connection >= 0)
        close(fifo->connection);
    fifo->connection = -1;
}

unsigned char ecb_usb_fifo_read_data(ecb_usb_fifo_state *fifo)
{
    unsigned char b = NODATA;
    int r;

    if(fifo->connection >= 0){
        r = read(fifo->connection, &b, 1);
        if((r < 0) || (r == 0 && errno != EAGAIN && errno != EWOULDBLOCK)){
            /* most likely the far end disconnected */
            close(fifo->connection);
            fifo->connection = -1;
            fprintf(stderr, "ecb_usb_fifo_read_data: disconnected\n");
        }
    }

    return b;
}

void ecb_usb_fifo_write_data(ecb_usb_fifo_state *fifo, unsigned char value)
{
    int r;

    if(fifo->connection >= 0){
        r = write(fifo->connection, &value, 1);
        if((r < 0) || (r == 0 && errno != EAGAIN && errno != EWOULDBLOCK)){
            /* most likely the far end disconnected */
            close(fifo->connection);
            fifo->connection = -1;
            fprintf(stderr, "ecb_usb_fifo_write_data: disconnected\n");
        }
    }
}

static void ecb_usb_fifo_update_status(ecb_usb_fifo_state *fifo)
{
    fd_set rs, ws;
    struct timeval tv;

    fifo->status |= USB_FIFO_STATUS_TXF | USB_FIFO_STATUS_RXE;

    if(fifo->connection >= 0){
        FD_ZERO(&rs);
        FD_SET(fifo->connection, &rs);
        FD_ZERO(&ws);
        FD_SET(fifo->connection, &ws);
        tv.tv_sec = tv.tv_usec = 0;

        if(select(fifo->connection + 1, &rs, &ws, NULL, &tv) == -1){
            perror("ecb_usb_fifo_read_status: select");
        }else{
            if(FD_ISSET(fifo->connection, &rs))
                fifo->status &= ~USB_FIFO_STATUS_RXE; /* signal rx data waiting */
            if(FD_ISSET(fifo->connection, &ws))
                fifo->status &= ~USB_FIFO_STATUS_TXF; /* signal tx space available */
        }
    }

    if( ((fifo->status & USB_FIFO_STATUS_INT_TXE) && !(fifo->status & USB_FIFO_STATUS_TXF)) ||
        ((fifo->status & USB_FIFO_STATUS_INT_RXF) && !(fifo->status & USB_FIFO_STATUS_RXE))){
        fifo->status |= USB_FIFO_STATUS_IRQ;
        fifo->irq = (fifo->status & USB_FIFO_STATUS_INT_ENABLE) ? 1 : 0;
    }else{
        fifo->status &= ~USB_FIFO_STATUS_IRQ;
        fifo->irq = 0;
    }
}

unsigned char ecb_usb_fifo_read_status(ecb_usb_fifo_state *fifo)
{
    ecb_usb_fifo_update_status(fifo);
    return fifo->status;
}

void ecb_usb_fifo_write_status(ecb_usb_fifo_state *fifo, unsigned char value)
{
    fifo->status = value & (USB_FIFO_STATUS_INT_ENABLE | 
                            USB_FIFO_STATUS_INT_TXE | 
                            USB_FIFO_STATUS_INT_RXF |
                            USB_FIFO_STATUS_GPIO_IN);
}

int ecb_usb_fifo_get_irq(ecb_usb_fifo_state *fifo)
{
    ecb_usb_fifo_update_status(fifo);
    return fifo->irq;
}
