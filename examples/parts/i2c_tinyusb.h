/*
 i2c_tinyusb.h
 $Id: i2c_tinyusb.h,v 1.4 2020/02/22 18:35:59 ewan Exp $

 (c) 2006 by Till Harbaum
 Copyright 2011 Michel Pollet <buserror@gmail.com>
 Copyright 2014 Doug Szumski <d.s.szumski@gmail.com>
 Copyright (C) 2020 Ewan Parker.

 http://www.harbaum.org/till/i2c_tiny_usb
 This file originated from simavr.
 The i2c_tiny_usb example was ported to simavr by Ewan Parker.

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

/*
 * This "Part" accesses I2C devices by bridging them on an I2C Tiny USB device.
 * Connect pin P0 on the interface to SDA on the device and P2 to SCL.
 *
 * Using a bridge enables simulation of AVR code without the need to create
 * complex peripheral simulations.
 */

#ifndef __I2C_TINYUSB_H__
#define __I2C_TINYUSB_H__

// Buffer sized to permit a complete buffered transfer of a black and white
// 128x64 OLED display.
#define I2C_TINYUSB_BUFFER_SIZE 1536

#include <stdbool.h>
#include "sim_irq.h"

enum
{
	//IRQ_I2C_TINYUSB_ALL = 0,
	IRQ_I2C_TINYUSB_TWI_IN,
	IRQ_I2C_TINYUSB_TWI_OUT,
	IRQ_I2C_TINYUSB_COUNT
	//TODO: Add IRQs for VCD: Internal state etc.
};

typedef struct i2c_tinyusb_t
{
	avr_irq_t * irq;
	struct avr_t * avr;
	char addr[0x78];
        int addr_len;
	int usb_vid, usb_pid;
	bool start_pending, stop_pending, write_pending;
	char twi_data[I2C_TINYUSB_BUFFER_SIZE];
	int twi_data_len;
} i2c_tinyusb_t;

void
i2c_tinyusb_init (struct avr_t *avr, struct i2c_tinyusb_t * b);

void
i2c_tinyusb_connect (i2c_tinyusb_t * part);

void
i2c_tinyusb_add_listen_addr (i2c_tinyusb_t * part, char addr);

#endif
