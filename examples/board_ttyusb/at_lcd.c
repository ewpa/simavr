/*-
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <tobias.rehbein@web.de> wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me a beer in return.
 *                                                             Tobias Rehbein
 */

/*
 * Upper layer LCD driver.
 */

#include <stdbool.h>

#include "at_lcd.h"
#include "at_hd44780.h"

void 
lcd_clr(void)
{
	hd44780_wait_ready(false);
	hd44780_outcmd(HD44780_CLR);
	hd44780_wait_ready(false);
	hd44780_outcmd(HD44780_HOME);
	hd44780_wait_ready(true);
	hd44780_outcmd(HD44780_DDADDR(0));
}

void
lcd_putc(char c)
{
	hd44780_wait_ready(false);
	hd44780_outdata(c);
}

void
lcd_puts(char *s)
{
	while (*s != '\0')
		lcd_putc(*(s++));
}

void
lcd_select_line(uint8_t l)
{
	uint8_t addr = (0x40) * (l - 1);

	hd44780_wait_ready(false);
	hd44780_outcmd(HD44780_DDADDR(addr));
}

void
lcd_init(void)
{
	hd44780_init();
	
	/*  
	 * Clear the display.
	 */
	hd44780_outcmd(HD44780_CLR);
	hd44780_wait_ready(true);
	
	/*  
	 * Entry mode: auto-increment address counter, no display shift in
	 * effect.
	 */
	hd44780_outcmd(HD44780_ENTMODE(1, 0));
	hd44780_wait_ready(false);

	/*  
	 * Function set: arrange for twoline display.
	 */
	hd44780_outcmd(HD44780_FNSET(0, 1, 0));
	hd44780_wait_ready(false);
	
	/*  
	 * Enable display, activate non-blinking cursor.
	 */
	hd44780_outcmd(HD44780_DISPCTL(1, 0, 0));
	hd44780_wait_ready(false);
}

/*
 * This file is partly based on a file under the following license:
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.  Joerg Wunsch
 * ----------------------------------------------------------------------------
 */
