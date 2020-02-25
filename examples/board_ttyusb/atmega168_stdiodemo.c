/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * Stdio demo
 *
 * $Id: stdiodemo.c,v 1.1.2.1 2005/12/28 22:35:08 joerg_wunsch Exp $
 */

// START simavr
#include "avr_mcu_section.h"
AVR_MCU(F_CPU, "atmega168");
// FINISH simavr

#include "at_defines.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

#include "at_lcd.h"
#include "at_uart.h"

int lcd_putchar(char c, FILE *f)
{
  (void)f;
  lcd_putc(c);
  return 1;
}

/*
 * Do all the startup-time peripheral initializations.
 */
static void
ioinit(void)
{
  uart_init();
  lcd_init();
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
FILE lcd_str = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);

static void
delay_1s(void)
{
  uint8_t i;

  for (i = 0; i < 100; i++)
    _delay_ms(10);
}

int
main(void)
{
  uint8_t i;
  char buf[20], s[20];

    CLKPR = _BV(CLKPCE); // enable clock scalar update
    CLKPR = 0x00;		// set to 8Mhz

  ioinit();

  stdout = stdin = &uart_str;
  stderr = &lcd_str;

  fprintf(stderr, "Hello world!\n");

  for (;;)
    {
      printf_P(PSTR("Enter command: "));
      if (fgets(buf, sizeof buf - 1, stdin) == NULL)
	break;
      if (tolower(buf[0]) == 'q')
	break;

      switch (tolower(buf[0]))
	{
	default:
	  printf("Unknown command: %s\n", buf);
	  break;

	case 'l':
	  if (sscanf(buf, "%*s %s", s) > 0)
	    {
	      fprintf(&lcd_str, "Got %s\n", s);
	      printf("OK\n");
	    }
	  else
	    {
	      printf("sscanf() failed\n");
	    }
	  break;

	case 'u':
	  if (sscanf(buf, "%*s %s", s) > 0)
	    {
	      fprintf(&uart_str, "Got %s\n", s);
	      printf("OK\n");
	    }
	  else
	    {
	      printf("sscanf() failed\n");
	    }
	  break;
	}
    }
  fprintf(stderr, "Bye-bye");
  delay_1s();
  for (i = 0; i < 3; i++)
    {
      putc('.', stderr);
      delay_1s();
    }
  fprintf(stderr, "\n ");

  return 0;
}
