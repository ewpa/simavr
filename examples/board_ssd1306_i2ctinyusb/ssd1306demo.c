/*
 charlcd.c

 Copyright Luki <humbell@ethz.ch>
 Copyright 2011 Michel Pollet <buserror@gmail.com>
 Copyright 2014 Doug Szumski <d.s.szumski@gmail.com>
 Copyright (C) 2020 Ewan Parker.

 This file is part of simavr.

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
#include <libgen.h>

#include "sim_avr.h"
#include "avr_ioport.h"
#include "sim_elf.h"
#include "sim_gdb.h"

#include <pthread.h>

#include "i2c_tinyusb.h"

avr_t * avr = NULL;
i2c_tinyusb_t i2c_tinyusb;

static void *
avr_run_thread (void * ignore)
{
	while (1)
	{
		avr_run (avr);
	}
	return NULL;
}

int
main (int argc, char *argv[])
{
	elf_firmware_t f;
	const char * fname = "atmega32_ssd1306.axf";
	char path[256];
	sprintf (path, "%s/%s", dirname (argv[0]), fname);
	printf ("Firmware pathname is %s\n", path);
	elf_read_firmware (fname, &f);

	printf ("firmware %s f=%d mmcu=%s\n", fname, (int) f.frequency, f.mmcu);

	avr = avr_make_mcu_by_name (f.mmcu);
	if (!avr)
	{
		fprintf (stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
		exit (1);
	}

	avr_init (avr);
	avr_load_firmware (avr, &f);

        i2c_tinyusb_init (avr, &i2c_tinyusb);
        i2c_tinyusb_connect (&i2c_tinyusb);
	// Filter the external I2C bus to only address 0x3C, the SSD1306.
        i2c_tinyusb_add_listen_addr (&i2c_tinyusb, 0x3C);

	printf ("SSD1306 display demo\n   Press 'q + ENTER' to quit\n");

	pthread_t run;
	pthread_create (&run, NULL, avr_run_thread, NULL);

	while (getchar() != 'q') {}
}
