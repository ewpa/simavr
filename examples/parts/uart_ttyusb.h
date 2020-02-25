/*
	uart_ttyusb.h
	$Id: uart_ttyusb.h,v 1.6 2020/02/22 18:35:59 ewan Exp $

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


#ifndef __UART_TTYUSB_H___
#define __UART_TTYUSB_H___

#include "sim_irq.h"
#include "fifo_declare.h"

// FIFO size must be a power of 2.
#define UART_TTYUSB_FIFO_SIZE 256
#define UART_TTYUSB_FIFO_CLEAR_ON_OFLOW 1

enum {
	IRQ_UART_TTYUSB_BYTE_IN = 0,
	IRQ_UART_TTYUSB_BYTE_OUT,
	IRQ_UART_TTYUSB_COUNT
};

DECLARE_FIFO(uint8_t,uart_ttyusb_fifo, UART_TTYUSB_FIFO_SIZE);

typedef struct uart_ttyusb_t {
	avr_irq_t *	irq;		// irq list
	struct avr_t *avr;		// keep it around so we can pause it

	pthread_t	thread;
	int		f;		// ttyusb file we chat on
	int		speed;
	char		dev[20];

	int			xon;
	uart_ttyusb_fifo_t in;
	uart_ttyusb_fifo_t out;
	int oflow;			// writes affected by buffer overflows
} uart_ttyusb_t;

void uart_ttyusb_init(struct avr_t * avr, uart_ttyusb_t * b);

void uart_ttyusb_connect(uart_ttyusb_t * p, char uart);

int uart_ttyusb_fifo_read_size(uart_ttyusb_fifo_t * fifo);

#endif /* __UART_TTYUSB_H___ */
