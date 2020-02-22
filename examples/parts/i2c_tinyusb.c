/*
 i2c_tinyusb.c
 $Id: i2c_tinyusb.c,v 1.5 2020/02/22 18:35:59 ewan Exp $
 
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

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "i2c_tinyusb.h"
#include "avr_twi.h"

#include <usb.h>

#define USB_CTRL_IN    (USB_TYPE_CLASS | USB_ENDPOINT_IN)
#define USB_CTRL_OUT   (USB_TYPE_CLASS)

/* the vendor and product id was donated by ftdi ... many thanks!*/
#define I2C_TINY_USB_VID  0x0403
#define I2C_TINY_USB_PID  0xc631

#define I2C_M_RD                0x01

/* commands via USB, must e.g. match command ids firmware */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3
#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag to I2C_IO
#define CMD_I2C_END    2  // flag to I2C_IO

#define STATUS_IDLE          0
#define STATUS_ADDRESS_ACK   1
#define STATUS_ADDRESS_NAK   2

usb_dev_handle      *handle = NULL;

/* write a set of bytes to the i2c_tiny_usb device */
/*
int i2c_tiny_usb_write(int request, int value, int index) {
  if(usb_control_msg(handle, USB_CTRL_OUT, request,
                      value, index, NULL, 0, 1000) < 0) {
    fprintf(stderr, "USB error #1: %s\n", usb_strerror());
    return -1;
  }
  return 1;
}
*/

/* read a set of bytes from the i2c_tiny_usb device */
int i2c_tiny_usb_read(unsigned char cmd, void *data, int len) {
  int                 nBytes;

  /* send control request and accept return value */
  nBytes = usb_control_msg(handle,
           USB_CTRL_IN,
           cmd, 0, 0, data, len, 1000);

  if(nBytes < 0) {
    fprintf(stderr, "USB error #2: %s\n", usb_strerror());
    return nBytes;
  }

  return 0;
}

/* set a value in the I2C_USB interface */
void i2c_tiny_usb_set(unsigned char cmd, int value) {
  if(usb_control_msg(handle,
             USB_TYPE_VENDOR, cmd, value, 0,
             NULL, 0, 1000) < 0) {
    fprintf(stderr, "USB error #3: %s\n", usb_strerror());
  }
}

/* get the current transaction status from the i2c_tiny_usb interface */
int i2c_tiny_usb_get_status(void) {
  int i;
  unsigned char status;

  if((i=i2c_tiny_usb_read(CMD_GET_STATUS, &status, sizeof(status))) < 0) {
    fprintf(stderr, "Error reading status\n");
    return i;
  }

  return status;
}

/*
 * Called when AVR sends us something.
 */
static void
i2c_tinyusb_twi_hook (struct avr_irq_t * irq, uint32_t value, void * param)
{
	i2c_tinyusb_t * p = (i2c_tinyusb_t*) param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	bool twiStart = (v.u.twi.msg & TWI_COND_START) != 0;
	bool twiStop = (v.u.twi.msg & TWI_COND_STOP) != 0;
	bool twiAck = (v.u.twi.msg & TWI_COND_ACK) != 0;
	bool twiWrite = (v.u.twi.msg & TWI_COND_WRITE) != 0;
	bool twiRead = (v.u.twi.msg & TWI_COND_READ) != 0;
	uint8_t twiAddr = v.u.twi.addr >> 1;

	bool addrMatch = false;
	if (!p->addr_len) addrMatch = true;
	else
	{
		int a;
		for (a = 0; a < p->addr_len; a++)
			if (p->addr[a] == twiAddr)
				addrMatch = true;
	}

	if (addrMatch)
	{
		//printf("TWI:O addr=0x%02X data=0x%02X ", twiAddr, v.u.twi.data);
		//printf("PART: pending start=%d stop=%d write=%d len=%d ", p->start_pending, p->stop_pending, p->write_pending, p->twi_data_len);
		//printf("FLAGS: start=%d stop=%d addr=%d ack=%d write=%d read=%d\r\n", twiStart, twiStop, v.u.twi.msg&TWI_COND_ADDR?1:0, twiAck, twiWrite, twiRead);

		if (twiStop)
		{
			p->stop_pending = true;
		}

		if (p->write_pending && p->stop_pending)
		{
			//printf("TWI:W addr=0x%02X data=0x%02X ", twiAddr, v.u.twi.data);
			//printf("PART: pending start=%d stop=%d len=%d\r\n", p->start_pending, p->stop_pending, p->twi_data_len);

			// Write pending data with or without start bit.
			if(usb_control_msg(handle, USB_CTRL_OUT,
			CMD_I2C_IO + (p->start_pending?CMD_I2C_BEGIN:0)
			+ (p->stop_pending?CMD_I2C_END:0),
			0, twiAddr, p->twi_data, p->twi_data_len,
			1000) < 1)
			{
				fprintf(stderr, "USB error #9: %s\n", usb_strerror());
			}

			if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
				fprintf(stderr, "write command status failed\n");
			}
			p->start_pending = p->stop_pending = p->write_pending = false;
			p->twi_data_len = 0;
		}

		if (twiRead)
		{
			// Read byte with or without start and stop bits.
			char result;
			if(usb_control_msg(handle,
			USB_CTRL_IN,
			CMD_I2C_IO + (p->start_pending?CMD_I2C_BEGIN:0)
			+ (!twiAck?CMD_I2C_END:0),
			I2C_M_RD, twiAddr, &result, 1,
			1000) < 1)
			{
				fprintf(stderr, "USB error #10: %s\n", usb_strerror());
			}

			if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
				fprintf(stderr, "read data status failed\n");
			}

			//printf("TWI:R PART: pending start=%d stop=%d data=0x%02X\r\n", p->start_pending, !twiAck, result);

			avr_raise_irq(p->irq + IRQ_I2C_TINYUSB_TWI_IN,
					avr_twi_irq_msg(TWI_COND_READ, v.u.twi.addr, result));
			p->start_pending = p->stop_pending = false;
		}

		if (twiStart)
		{
			p->start_pending = true;
			p->stop_pending = false;
			avr_raise_irq(p->irq + IRQ_I2C_TINYUSB_TWI_IN,
					avr_twi_irq_msg(TWI_COND_ACK, v.u.twi.addr, 1));
		}

		if (twiWrite)
		{
			//printf("TWI:S PART: data=0x%02X pos=%d\r\n", v.u.twi.data, p->twi_data_len);

			// Save the next byte.
			if (p->twi_data_len < I2C_TINYUSB_BUFFER_SIZE)
				p->twi_data[p->twi_data_len++] = v.u.twi.data;
			else
				printf("I2C lost write\r\n");
			p->write_pending = true;
			// Ack here, but transfer later with stop bit.
			avr_raise_irq(p->irq + IRQ_I2C_TINYUSB_TWI_IN,
					avr_twi_irq_msg(TWI_COND_ACK, v.u.twi.addr, 1));
		}
	}
}

static const char * irq_names[IRQ_I2C_TINYUSB_COUNT] =
{
	[IRQ_I2C_TINYUSB_TWI_OUT] = "32<tinyusb.twi.out",
	[IRQ_I2C_TINYUSB_TWI_IN] = "8>tinyusb.twi.in",
};

void
i2c_tinyusb_connect (i2c_tinyusb_t * part)
{
	if (part->usb_pid)
	{
		avr_connect_irq (
			avr_io_getirq (part->avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT),
			part->irq + IRQ_I2C_TINYUSB_TWI_OUT);

		avr_connect_irq (
			part->irq + IRQ_I2C_TINYUSB_TWI_IN,
			avr_io_getirq (part->avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT));
	}
}

void
i2c_tinyusb_add_listen_addr (i2c_tinyusb_t * part, char addr)
{
	// Respond to just a subset of addresses.
	if (part->addr_len < 0x78)
		part->addr[part->addr_len++] = addr;
}

void
i2c_tinyusb_init (struct avr_t *avr, struct i2c_tinyusb_t * part)
{
  if (!avr || !part)
    return;

  memset (part, 0, sizeof(*part));
  part->avr = avr;

  struct usb_bus      *bus;
  struct usb_device   *dev;
  int ret;

  usb_init();

  usb_find_busses();
  usb_find_devices();

  for(bus = usb_get_busses(); bus; bus = bus->next) {
    for(dev = bus->devices; dev; dev = dev->next) {
      if((dev->descriptor.idVendor == I2C_TINY_USB_VID) &&
         (dev->descriptor.idProduct == I2C_TINY_USB_PID)) {

        printf("Found i2c_tiny_usb device %04x:%04x on bus %s device %s\n",
               I2C_TINY_USB_VID, I2C_TINY_USB_PID, bus->dirname, dev->filename);

        /* open device */
        if(!(handle = usb_open(dev)))
          fprintf(stderr, "Error: Cannot open the device: %s\n",
                  usb_strerror());

        break;
      }
    }
  }

  if(!handle) {
    fprintf(stderr, "Error: Could not find i2c_tiny_usb device\n");
    return;
  }

  /* Get exclusive access to interface 0. */
  ret = usb_claim_interface(handle, 0);
  if (ret != 0) {
    fprintf(stderr, "USB error #11: %s, maybe run 'modprobe -r i2c_tiny_usb'\n", usb_strerror());
    return;
  }

  /* try to set i2c clock to 400kHz (2.5us), will actually result in ~50kHz */
  /* since the software generated i2c clock isn't too exact. */
  i2c_tiny_usb_set(CMD_SET_DELAY, 3);

  part->usb_vid = I2C_TINY_USB_VID;
  part->usb_pid = I2C_TINY_USB_PID;

  /*
   * Register callbacks on all our IRQs
   */
  part->irq = avr_alloc_irq (&avr->irq_pool, 0, IRQ_I2C_TINYUSB_COUNT,
    irq_names);

  avr_irq_register_notify (part->irq + IRQ_I2C_TINYUSB_TWI_OUT,
     i2c_tinyusb_twi_hook, part);
}
