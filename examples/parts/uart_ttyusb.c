/*
	uart_ttyusb.c
	$Id: uart_ttyusb.c,v 1.8 2020/02/25 10:45:08 ewan Exp $

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>
	Copyright (C) 2020 Ewan Parker.

 	This file originated from simavr.
	The TTYUSB UART part was written by Ewan Parker.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pthread.h>
#include <sys/prctl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "uart_ttyusb.h"
#include "avr_uart.h"
#include "sim_time.h"
#include "sim_hex.h"

DEFINE_FIFO(uint8_t,uart_ttyusb_fifo);

/*
 * called when a byte is send via the uart on the AVR
 */
static void uart_ttyusb_in_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	uart_ttyusb_t * p = (uart_ttyusb_t*)param;
	//printf("uart_ttyusb_in_hook %02x\n", value);
	uart_ttyusb_fifo_write(&p->in, value);
}

// Try to empty our fifo, the uart_pty_xoff_hook() will be called when other
// side is full.
static void
uart_pty_flush_incoming(
		uart_ttyusb_t * p)
{
	while (p->xon && !uart_ttyusb_fifo_isempty(&p->out)) {
		uint8_t byte = uart_ttyusb_fifo_read(&p->out);
		//printf("uart_ttyusb_xon_hook send %02x\r\n", byte);
		avr_raise_irq(p->irq + IRQ_UART_TTYUSB_BYTE_OUT, byte);
	}
}

avr_cycle_count_t
uart_ttyusb_flush_timer(
	struct avr_t * avr,
	avr_cycle_count_t when,
		void * param)
{
	uart_ttyusb_t * p = (uart_ttyusb_t*)param;

	uart_pty_flush_incoming(p);
	/* always return a cycle NUMBER not a cycle count */
	return p->xon ? when + avr_hz_to_cycles(p->avr, 125) : 0;
}

/*
 * Called when the uart has room in its input buffer.  This is called
 * repeateadly if necessary, while the xoff is called only when the uart fifo
 * is FULL
 */
static void uart_ttyusb_xon_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	uart_ttyusb_t * p = (uart_ttyusb_t*)param;
	//if (!p->xon)
	//	printf("uart_ttyusb_xon_hook\r\n");
	p->xon = 1;

	uart_pty_flush_incoming(p);

	if (p->xon)
		avr_cycle_timer_register(p->avr, avr_hz_to_cycles(p->avr, 125),
					uart_ttyusb_flush_timer, param);
}

/*
 * Called when the uart ran out of room in its input buffer.
 */
static void uart_ttyusb_xoff_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	uart_ttyusb_t * p = (uart_ttyusb_t*)param;
	//if (p->xon)
	//	printf("uart_ttyusb_xoff_hook\r\n");
	p->xon = 0;
	avr_cycle_timer_cancel(p->avr, uart_ttyusb_flush_timer, param);
}

static void * uart_ttyusb_thread(void * param)
{
	prctl(PR_SET_NAME, "clock [ttyusb]",0 ,0 ,0);
	uart_ttyusb_t * p = (uart_ttyusb_t*)param;

	while (1) {
		fd_set read_set, write_set;
		int max = p->f + 1;
		FD_ZERO(&read_set);
		FD_ZERO(&write_set);

		FD_SET(p->f, &read_set);
		if (!uart_ttyusb_fifo_isempty(&p->in))
			FD_SET(p->f, &write_set);

		struct timeval timo = { 0, 500 };	// short, but not too short interval
		int ret = select(max, &read_set, &write_set, NULL, &timo);

		if (!ret)
			continue;

		if (FD_ISSET(p->f, &read_set)) {
			uint8_t buffer[UART_TTYUSB_FIFO_SIZE];

			ssize_t r = read(p->f, buffer, sizeof(buffer)-1);

			//hdump("uart recv", buffer, r);

			// write them in fifo
			uint8_t * src = buffer;
			while (!uart_ttyusb_fifo_isfull(&p->out) && r--)
				uart_ttyusb_fifo_write(&p->out, *src++);
			if (r > 0)
			{
				//printf("UART dropped %zu bytes\r\n", r);
				++ p->oflow;
				#if UART_TTYUSB_FIFO_CLEAR_ON_OFLOW
				// Discard buffer contents.
				p->out.read = p->out.write = 0;
				#endif
			}
		}
		if (FD_ISSET(p->f, &write_set)) {
			uint8_t buffer[UART_TTYUSB_FIFO_SIZE];
			// write them in fifo
			uint8_t * dst = buffer;
			while (!uart_ttyusb_fifo_isempty(&p->in) && dst < (buffer+sizeof(buffer)))
				*dst++ = uart_ttyusb_fifo_read(&p->in);
			size_t len = dst - buffer;
			/*size_t r = */write(p->f, buffer, len);
			//hdump("uart send", buffer, r);
			//printf("uart send %zu bytes\n", r);
		}
	}
	return NULL;
}

static const char * irq_names[IRQ_UART_TTYUSB_COUNT] = {
	[IRQ_UART_TTYUSB_BYTE_IN] = "8<uart_ttyusb.in",
	[IRQ_UART_TTYUSB_BYTE_OUT] = "8>uart_ttyusb.out",
};

void uart_ttyusb_init(struct avr_t * avr, uart_ttyusb_t * p)
{
	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, IRQ_UART_TTYUSB_COUNT, irq_names);
	avr_irq_register_notify(p->irq + IRQ_UART_TTYUSB_BYTE_IN, uart_ttyusb_in_hook, p);

	p->speed = 9600;
	if (!strnlen(p->dev, 20))
		strncpy(p->dev, "/dev/ttyUSB0", 20);
	if ((p->f = open(p->dev, O_RDWR)) < 0) {
		fprintf(stderr, "%s: Can't open file %s: %s\n", __FUNCTION__, p->dev, strerror(errno));
		p->speed = 0;
		strncpy(p->dev, "", 20);
		return ;
	}

	if ((ioctl(p->f, TIOCEXCL, NULL)) < 0) {
		fprintf(stderr, "%s: Can't lock file %s: %s\n", __FUNCTION__, p->dev, strerror(errno));
		return ;
	}

	struct termios ios;
	cfmakeraw(&ios);
	cfsetspeed(&ios, B9600);
	if ((tcsetattr(p->f, TCSANOW, &ios)) < 0) {
		fprintf(stderr, "%s: Can't ioctl file %s: %s\n", __FUNCTION__, p->dev, strerror(errno));
		return ;
	}

	p->oflow = 0;

	printf("uart_ttyusb_init bridge on %s at %d bps\n", p->dev, p->speed);

	pthread_create(&p->thread, NULL, uart_ttyusb_thread, p);

}

void uart_ttyusb_connect(uart_ttyusb_t * p, char uart)
{
	// disable the stdio dump, as we are sending binary there
	uint32_t f = 0;
	avr_ioctl(p->avr, AVR_IOCTL_UART_GET_FLAGS(uart), &f);
	f &= ~AVR_UART_FLAG_STDIO;
	avr_ioctl(p->avr, AVR_IOCTL_UART_SET_FLAGS(uart), &f);

	avr_irq_t * src = avr_io_getirq(p->avr, AVR_IOCTL_UART_GETIRQ(uart), UART_IRQ_OUTPUT);
	avr_irq_t * dst = avr_io_getirq(p->avr, AVR_IOCTL_UART_GETIRQ(uart), UART_IRQ_INPUT);
	avr_irq_t * xon = avr_io_getirq(p->avr, AVR_IOCTL_UART_GETIRQ(uart), UART_IRQ_OUT_XON);
	avr_irq_t * xoff = avr_io_getirq(p->avr, AVR_IOCTL_UART_GETIRQ(uart), UART_IRQ_OUT_XOFF);
	if (src && dst) {
		avr_connect_irq(src, p->irq + IRQ_UART_TTYUSB_BYTE_IN);
		avr_connect_irq(p->irq + IRQ_UART_TTYUSB_BYTE_OUT, dst);
	}
	if (xon)
		avr_irq_register_notify(xon, uart_ttyusb_xon_hook, p);
	if (xoff)
		avr_irq_register_notify(xoff, uart_ttyusb_xoff_hook, p);
}

int uart_ttyusb_fifo_read_size(uart_ttyusb_fifo_t * fifo)
{
  return uart_ttyusb_fifo_get_read_size(fifo);
}
