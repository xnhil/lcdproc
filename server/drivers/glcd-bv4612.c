/** \file server/drivers/glcd-t6963.c
 * Connection type to drive displays based on the Toshiba T6963 controller
 * connected to the parallel port. The display is driven in graphic mode. See
 * \c t6963_low.c for details of the parallel port connection.
 *
 * \note  The display must be wired to use 8x8 font. Check with your datasheet.
 */

/*-
 * Copyright (c) 2011 Markus Dolze, based on the t6963 driver by
 *               2001 Manuel Stahl <mythos@xmythos.de>
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "lcd.h"
#include "shared/report.h"
#include "glcd-low.h"

#include "bv4612.h"

#define BV4612_WIDTH		128
#define BV4612_HEIGHT		64
#define DEFAULT_ADDRESS		0x35

static void bv4612_graphic_clear(PrivateData *p);
void glcd_bv4612_blit(PrivateData *p);
void glcd_bv4612_close(PrivateData *p);
unsigned char glcd_bv4612_poll_keys(PrivateData *p);
void glcd_bv4612_set_backlight(PrivateData *p, int state);
void glcd_bv4612_set_contrast(PrivateData *p, int value);

/** Data local to the bv4612 connection type */
typedef struct glcd_bv4612_data {
	int address;
	int fd;
	uint8_t *framestore;
} CT_bv4612_data;

/**
 * API: Initialize the connection type driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval <0      Error.
 */
int
glcd_bv4612_init(Driver *drvthis)
{
	PrivateData *p = (PrivateData *)drvthis->private_data;
	CT_bv4612_data *ct_data;

	report(RPT_INFO, "GLCD/BV4612: intializing");

	/* Check the size before doing anything else! */
	if ((p->framebuf.px_width != BV4612_WIDTH) || (p->framebuf.px_height != BV4612_HEIGHT)) {
		report(RPT_ERR, "GLCD/BV4612: Size %dx%d not supported by connection type",
		       p->framebuf.px_width, p->framebuf.px_height);
		return -1;
	}

	/* Set up connection type low-level functions */
	p->glcd_functions->blit = glcd_bv4612_blit;
	p->glcd_functions->close = glcd_bv4612_close;
	p->glcd_functions->poll_keys = glcd_bv4612_poll_keys;
	p->glcd_functions->set_backlight = glcd_bv4612_set_backlight;
	p->glcd_functions->set_contrast = glcd_bv4612_set_contrast;

	/* Allocate memory structures */
	ct_data = (CT_bv4612_data *) calloc(1, sizeof(CT_bv4612_data));
	if (ct_data == NULL) {
		report(RPT_ERR, "GLCD/BV4612: error allocating connection data");
		return -1;
	}
	p->ct_data = ct_data;
	
	ct_data->framestore = (uint8_t *) calloc(1, ((p->framebuf.px_width * p->framebuf.px_height) / 8) + 2);
	if (ct_data->framestore == NULL) {
		report(RPT_ERR, "GLCD/BV4612: error allocating framestore");
		return -1;
	}
	ct_data->framestore[0] = p->framebuf.px_height / 8;
	ct_data->framestore[1] = p->framebuf.px_width;
	
	ct_data->address = drvthis->config_get_int(drvthis->name, "Address", 0, DEFAULT_ADDRESS);
	if (ct_data->address < 0x03 || ct_data->address > 0x77) {
		report(RPT_ERR, "GLCD/BV4612: i2c address out of range (0x03-0x77)");
		return -1;
	}
	
	ct_data->fd = bv4612_join(ct_data->address);
	if(ct_data->fd == -1) {
		report(RPT_ERR, "GLCD/BV4612: failed to open i2c device: %s", strerror(errno));
		return -1;
	}
	
	char buf[30];
	
	Version(ct_data->fd, buf);
	
	report(RPT_INFO, "GLCD/BV4612: connected to BV4612 at 0x%2x, version: %s", ct_data->address, buf);
	
	lcd_reset(ct_data->fd);
	
	lcd_clear(ct_data->fd);
	
	debug(RPT_DEBUG, "GLCD/BV4612: init() done");

	return 0;
}


/**
 * API: Write the framebuffer to the display
 * \param p  Pointer to glcd driver's private date structure.
 */
void
glcd_bv4612_blit(PrivateData *p)
{
	CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
/*	uint8_t *s = ct_data->framestore+2, bit;
	int x, y;
	for(y = 0; y < p->framebuf.px_height; y++) {
        	for(x = 0; x < p->framebuf.px_width; x++) {
        	    bit = p->framebuf.data[((y * p->framebuf.px_width) / 8) + (x  / 8)] & (1 << (x % 8)) ? 1 : 0;
        	    ct_data->framestore[2 + (y / 8)]
        	}
	}
			temp = p->framebuf.data[tmp++];*/
        int x, y;
	uint8_t *s = ct_data->framestore+2, *buf = p->framebuf.data;
	memset(ct_data->framestore + 2, 0, (p->framebuf.px_width * p->framebuf.px_height) / 8);
        for(y = 0; y < 64/8; y++) {
            for(x = 0; x < 128; x++) {
            *s++ = 0x00
                | (buf[(((y*8)+0) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 0
                | (buf[(((y*8)+1) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 1
                | (buf[(((y*8)+2) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 2
                | (buf[(((y*8)+3) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 3
                | (buf[(((y*8)+4) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 4
                | (buf[(((y*8)+5) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 5
                | (buf[(((y*8)+6) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 6
                | (buf[(((y*8)+7) * (128/8)) + (x/8)] >> (7-(x%8)) & 0x01) << 7
                ;
            }
        }
        
        lcd_image(ct_data->fd, ct_data->framestore, 0, 0);
}


/**
 * API: Release low-level resources.
 * \param p  Pointer to glcd driver's private date structure.
 */
void
glcd_bv4612_close(PrivateData *p)
{
	if (p->ct_data != NULL) {
		CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
		
        	if (ct_data->framestore != NULL) {
        		free(ct_data->framestore);
        	}
        	
		free(p->ct_data);
		p->ct_data = NULL;
	}
}


/**
 * Clears graphic memory.
 * \param p  Pointer to glcd driver's private date structure.
 */
static void
bv4612_graphic_clear(PrivateData *p)
{
	CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
	int num = p->framebuf.size;

	p->glcd_functions->drv_debug(RPT_DEBUG, "GLCD/BV4612: Clearing graphic: %d bytes", num);

	lcd_clear(ct_data->fd);
	memset(ct_data->framestore + 2, 0, (p->framebuf.px_width * p->framebuf.px_height) / 8);
}

/**
 * API: Poll for any pressed keys. Converts the bitmap of keys pressed into a
 * scancode (1-6) for each pressed key.
 */
unsigned char
glcd_bv4612_poll_keys(PrivateData *p)
{
    CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
    
    return key(ct_data->fd);
}

/**
 * API: Set the backlight brightness.
 */
void
glcd_bv4612_set_backlight(PrivateData *p, int state)
{
    CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
    
    p->glcd_functions->drv_debug(RPT_DEBUG, "GLCD/BV4612: setting backlight state: %d", state);
    
    lcd_bl(ct_data->fd, state ? 1 : 0);
}

/**
 * API: Change LCD contrast.
 */
void
glcd_bv4612_set_contrast(PrivateData *p, int value)
{
    CT_bv4612_data *ct_data = (CT_bv4612_data *) p->ct_data;
    int v = 0;
    
    if(value >= 1000) {
        v = 63;
    } else if(value < 0) {
        v = 0;
    } else {
        v = value / (1000 / 64);
    }
    
    p->glcd_functions->drv_report(RPT_INFO, "GLCD/BV4612: setting contrast: %d (%d)", v, value);
    
    lcd_contrast(ct_data->fd, v);
}
	
// BV4242 is a touch keypad with LCD front panel
// This helps to tune the EERPOM settings
//
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "bv4612.h"

// just part of this file
#include "sv3_i2c.c"

// keypad commands
#define PAD_CLEARBUF    1
#define PAD_KEYSINBUF   2
#define PAD_GETKEY      3
#define PAD_ISINBUF     4
#define PAD_SCANCODE    5
#define PAD_LCDRESET    7
#define PAD_GETAVG      10
#define PAD_GETDELTA    11
#define PAD_RESETEEPROM 20
#define PAD_SLEEP       21
#define LCD_RESET       30
#define LCD_CMD         31
#define LCD_DATA        32
#define LCD_SIGN        35
#define LCD_BACKLIGHT   36
#define LCD_CONTRAST    37
#define LCD_SENDBYTES   38
#define LCD_SETFONT     40
#define LCD_HOMECLR     41
#define LCD_COLUMN      42
#define LCD_PAGE        43
#define LCD_SCROLL      44
#define LCD_WRITE       45

// EEPROM constants
#define EE_TRIGGERH 18
#define EE_TRIGGERL EE_TRIGGERH+1
#define EE_HYST 20
#define EE_KEYPTR 21
#define EE_KEYSIZE EE_KEYPTR+1
#define EE_DEBOUNCE 23
#define EE_REPEATH 24
#define EE_REPEATL EE_REPEATH+1
#define EE_TIMEBASE 26
#define EE_BL 27

// *****************************************************************************
// gets a 16 bit value
// *****************************************************************************
void get16(int bus, uint16_t *values, uint8_t cmd)
{
uint16_t v, j;
    wiringPiI2CWrite(bus,cmd); // send command
    for(j=0; j<8; j++) {
        v = wiringPiI2CRead(bus) << 8;
        v += wiringPiI2CRead(bus);
        *(values+j) = v;
    }
}

// =============================================================================
// =============================================================================
int bv4612_join(uint8_t i2caddress)
{
    return(wiringPiI2CSetup(i2caddress));
}

// =============================================================================
// =============================================================================
// =============================================================================
// Keypad section
// =============================================================================
// *****************************************************************************
// Clear keypad buffer
// *****************************************************************************
void clrBuf(int bus)
{
    wiringPiI2CWrite(bus,PAD_CLEARBUF);
    delayMicroseconds(250);
}

// *****************************************************************************
// Gets number of keys in buffer
// *****************************************************************************
uint8_t keysBuf(int bus)
{
    wiringPiI2CWrite(bus,PAD_KEYSINBUF);
    return wiringPiI2CRead(bus);
}

// *****************************************************************************
// Gets a key returns 0 if no keys in buffer
// *****************************************************************************
uint8_t key(int bus)
{
    wiringPiI2CWrite(bus,PAD_GETKEY);
    return wiringPiI2CRead(bus);
//    return wiringPiI2CReadReg8(bus,PAD_GETKEY);
}

// *****************************************************************************
// See if a partiular key is in the buffer, returns 0 if not and it it is
// the position of the key within the buffer
// *****************************************************************************
uint8_t keyIn(int bus, uint8_t k)
{
    wiringPiI2CWriteReg8(bus,PAD_ISINBUF,k);
    return wiringPiI2CRead(bus);
}

// *****************************************************************************
// returns a key scan code 0 of no key pressed
// *****************************************************************************
uint8_t scan(int bus)
{
    wiringPiI2CWrite(bus,PAD_SCANCODE);
    return wiringPiI2CRead(bus);
}

// *****************************************************************************
// sets back light brightness 0 - 130 but setting higher does not matter
// *****************************************************************************
void bl(int bus, uint8_t v)
{
    wiringPiI2CWriteReg8(bus,PAD_ISINBUF,v);
}

// *****************************************************************************
// returns 8 average values in the buffer provided, values are 16 bits
// *****************************************************************************
void average(int bus, uint16_t *b)
{
    get16(bus , b, PAD_GETAVG);
}

// *****************************************************************************
// returns 8 delta values in the buffer provided, values are 16 bits
// *****************************************************************************
void delta(int bus, uint16_t *b)
{
    get16(bus , b, PAD_GETDELTA);
}

// *****************************************************************************
// Resets eeprom values
// *****************************************************************************
void EEreset(int bus)
{
    wiringPiI2CWrite(bus,PAD_RESETEEPROM); // command
}

// *****************************************************************************
// Sleep mode, the keypad will not function in this mode but the device can 
// be awakend vie a read or write to I2C
// *****************************************************************************
void sleep(int bus)
{
    wiringPiI2CWrite(bus,PAD_SLEEP); // command
}

// =============================================================================
// SV3 system section
// =============================================================================
// *****************************************************************************
// Writes a byte to the eeprom at a given address
// *****************************************************************************
void EEWrite(int bus, uint8_t adr, uint8_t value)
{
    sv3_eewrite(bus,adr,value);
}

// *****************************************************************************
// Reads a byte from eeprom at given address
// *****************************************************************************
uint8_t EEread(int bus, uint8_t adr)
{
    return sv3_eeread(bus,adr);
}

// *****************************************************************************
// Get device ID as a 16 bit number
// *****************************************************************************
uint16_t ID(int bus)
{
    return sv3_deviceID(bus);
}

// *****************************************************************************
// Get firmware version as a string, 4 bytes needed
// *****************************************************************************
void Version(int bus, char *s)
{
    sv3_Version(bus,s);
}

// =============================================================================
// EEPROM settings
// =============================================================================
// *****************************************************************************
// sets new trigger value
// *****************************************************************************
void trigger(int bus, uint16_t value)
{
    EEWrite(bus,EE_TRIGGERH,(value >> 8) & 0xff);
    EEWrite(bus,EE_TRIGGERL,value & 0xff);
}

// *****************************************************************************
// sets new hysteresis value
// *****************************************************************************
void hyst(int bus, uint8_t value)
{
    EEWrite(bus, EE_HYST,value);
}

// *****************************************************************************
// sets new keytable pointer
// *****************************************************************************
void keyPtr(int bus, uint8_t value)
{
    EEWrite(bus, EE_KEYPTR,value);
}

// *****************************************************************************
// sets new key table size
// *****************************************************************************
void keySize(int bus, uint8_t value)
{
    EEWrite(bus, EE_KEYSIZE,value);
}

// *****************************************************************************
// sets new debounce
// *****************************************************************************
void debounce(int bus, uint8_t value)
{
    EEWrite(bus, EE_DEBOUNCE,value);
}

// *****************************************************************************
// sets new repeat value
// *****************************************************************************
void repeat(int bus, uint16_t value)
{
    EEWrite(bus,EE_REPEATH,(value >> 8) & 0xff);
    EEWrite(bus,EE_REPEATL,value & 0xff);
}

// *****************************************************************************
// sets new timebase
// *****************************************************************************
void timebase(int bus, uint8_t value)
{
    EEWrite(bus,EE_TIMEBASE,value);
}

// *****************************************************************************
// sets new defailt bl
// *****************************************************************************
void defaultBL(int bus,uint8_t value)
{
    EEWrite(bus,EE_BL,value);
}

// =============================================================================
// =============================================================================
// =============================================================================
// LCD Section
// This is for the ST7032i controller and follows the same commands used 
// for LiquidCrystal; to this end column is specified first i.e 16x2
// =============================================================================
// 
// *****************************************************************************
// sends a command to the display, bus open at this point
// *****************************************************************************
void lcd_reset(int bus)
{
    wiringPiI2CWrite(bus,LCD_RESET);
    delay(350); // commands takes 300mS
}

// *****************************************************************************
// sends a command to the display, bus open at this point
// *****************************************************************************
void lcd_cmd(int bus, uint8_t cmd)
{
    wiringPiI2CWriteReg8(bus,LCD_CMD,cmd);
}

// *****************************************************************************
// sends data to the display, bus open at this point
// *****************************************************************************
void lcd_write(int bus, uint8_t data)
{
    wiringPiI2CWriteReg8(bus,LCD_WRITE,data);
}

// *****************************************************************************
// *****************************************************************************
void lcd_string(int bus, int row, int col, char *s)
{
int del=strlen(s)*5; // delay per char
    lcd_setCursor(bus,col,row);
    while(*s) {
        lcd_write(bus,*(s++));
    }
    delay(del);
}


// *****************************************************************************
//  
// *****************************************************************************
void lcd_contrast(int bus,uint8_t v)
{
    wiringPiI2CWriteReg8(bus,LCD_CONTRAST,v); 
}

// *****************************************************************************
//  
// *****************************************************************************
void lcd_bl(int bus,uint8_t v)
{
    wiringPiI2CWriteReg8(bus,LCD_BACKLIGHT,v); 
}

// *****************************************************************************
// Sets column address
// *****************************************************************************
void lcd_column(int bus,uint8_t v)
{
    wiringPiI2CWriteReg8(bus,LCD_COLUMN,v);
}

// *****************************************************************************
// Sets row address
// *****************************************************************************
void lcd_page(int bus,uint8_t v)
{
    wiringPiI2CWriteReg8(bus,LCD_PAGE,v);
}

// *****************************************************************************
//
// *****************************************************************************
void lcd_clear(int bus)
{
    wiringPiI2CWrite(bus,LCD_HOMECLR);
    delay(350); // commands takes 300mS
}

// *****************************************************************************
// *****************************************************************************
void lcd_home(int bus)
{
    lcd_column(bus,0);
    lcd_page(bus,0);
}

// *****************************************************************************
// *****************************************************************************
void lcd_setCursor(int bus,uint8_t col, uint8_t row)
{
uint8_t adr;
    if(row > 7) row = 7;
    if(col > 127) col = 127;
    lcd_column(bus,col);
    lcd_page(bus,row);
}

// *****************************************************************************
// *****************************************************************************
void lcd_dataline(int bus,uint8_t *s, uint8_t row, uint8_t col, uint8_t nBytes)
{
uint8_t j;

    wiringPiI2CWrite(bus,LCD_SENDBYTES);
    wiringPiI2CWrite(bus,row);
    wiringPiI2CWrite(bus,col);
    wiringPiI2CWrite(bus,nBytes);
    
    for(j=0; j < nBytes; j++) {
        wiringPiI2CWrite(bus,s[j]);
    }
}

void lcd_image(int bus,uint8_t *s, uint8_t row, uint8_t col)
{
        uint8_t pages = s[0], bpp = s[1];
        int i;
        s+=2;
        for(i = 0; i < pages; i++) {
        	lcd_dataline(bus, s, i+row, col, bpp);
        	s += bpp;
        }
}

// int main(int argc, char *argv[])
// {
//   int pad=bv4612_join(0x35); // attache to bus
//   lcd_clear(pad);
//   lcd_string(pad,3,3,"hello fred");
// }
