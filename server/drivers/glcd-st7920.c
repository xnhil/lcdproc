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
#include "timing.h"
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <stdint.h>

#define DEFAULT_USEC_DELAY	80
#define ST7920_WIDTH		128
#define ST7920_HEIGHT		64
#define DEFAULT_DEVICE		"/dev/spidev0.0"

/** KS0073 5-bit sync lead-in on SPI interface */
#define SYNC	0xF8u
/** KS0073 Read Write bit: 1 = read selected data/register, 0 = write to selected data/register */
#define RW	0x04u
/** KS0073 Register Select bit: 0 = instruction register follows, 1 = data register follows */
#define RS	0x02u
/* commands for senddata */
#define RS_DATA		0x00
#define RS_INSTR	0x01

static void st7920_graphic_clear(PrivateData *p);
void glcd_st7920_blit(PrivateData *p);
void glcd_st7920_close(PrivateData *p);

/** Data local to the st7920 connection type */
typedef struct glcd_st7920_data {
	int fd;
} CT_st7920_data;

/**
 * Do a SPI transfer by sending \c length bytes of \c outbuf and read the same
 * number of bytes into \c inbuf.
 * \param p       Pointer to driver's private data structure.
 * \param outbuf  Buffer holding data to send.
 * \param inbuf   Buffer to store incoming data (or NULL).
 * \param length  Length of buffers (both).
 * \return  Status from ioctl.
 */
static int
spi_transfer(PrivateData *p, const unsigned char *outbuf, unsigned char *inbuf, unsigned length)
{
	CT_st7920_data *ct_data = (CT_st7920_data *) p->ct_data;
	struct spi_ioc_transfer xfer;
	int status;

	static unsigned char no_more_errormsgs = 0;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (unsigned long) outbuf;
	xfer.rx_buf = (unsigned long) inbuf;
	xfer.len = length;
	// !!!!! delay
	xfer.delay_usecs = DEFAULT_USEC_DELAY;
	xfer.speed_hz = 1800000;
	xfer.bits_per_word = 8;
	xfer.cs_change = 1;
	
	p->glcd_functions->drv_debug(RPT_DEBUG, "SPI sending %02x %02x %02x",
		outbuf[0], outbuf[1], length > 2 ? outbuf[2] : 0xFFu);
	status = ioctl(ct_data->fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		p->glcd_functions->drv_report(no_more_errormsgs ? RPT_DEBUG : RPT_ERR,
			 "GLCD/ST7920: SPI: spidev write data %u failed: %s",
			 status, strerror(errno));
		no_more_errormsgs = 1;
	}
#ifdef DEBUG
	if (inbuf)
		p->glcd_functions->drv_debug(RPT_DEBUG, "SPI returned data %02x %02x %02x",
			inbuf[0], inbuf[1], length > 2 ? inbuf[2] : 0xFFu);
#endif

	return status;
}

/**
 * Send data or commands to the display.
 * \param p          Pointer to driver's private data structure.
 * \param flags      Defines whether to end a command or data.
 * \param ch         The value to send.
 */
void
spi_st7920_senddata(PrivateData *p, unsigned char flags, unsigned char ch)
{
	unsigned char buf[3];

	p->glcd_functions->drv_debug(RPT_DEBUG, "GLCD/ST7920: SPI: sending %s %02x",
		 RS_INSTR == flags ? "CMD" : "DATA", ch);

	if (flags == RS_INSTR)
		buf[0] = SYNC;
	else
		buf[0] = SYNC | RS;

	buf[1] = ch & 0xF0;
	buf[2] = (ch & 0x0F) << 4;

	spi_transfer(p, buf, NULL, sizeof(buf));
}

/**
 * API: Initialize the connection type driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval <0      Error.
 */
int
glcd_st7920_init(Driver *drvthis)
{
	PrivateData *p = (PrivateData *)drvthis->private_data;
	CT_st7920_data *ct_data;

	report(RPT_INFO, "GLCD/ST7920: intializing");

	/* Check the size before doing anything else! */
	if ((p->framebuf.px_width != ST7920_WIDTH) || (p->framebuf.px_height != ST7920_HEIGHT)) {
		report(RPT_ERR, "GLCD/ST7920: Size %dx%d not supported by connection type",
		       p->framebuf.px_width, p->framebuf.px_height);
		return -1;
	}

	/* Set up connection type low-level functions */
	p->glcd_functions->blit = glcd_st7920_blit;
	p->glcd_functions->close = glcd_st7920_close;

	/* Allocate memory structures */
	ct_data = (CT_st7920_data *) calloc(1, sizeof(CT_st7920_data));
	if (ct_data == NULL) {
		report(RPT_ERR, "GLCD/ST7920: error allocating connection data");
		return -1;
	}
	p->ct_data = ct_data;
	
	char device[256] = DEFAULT_DEVICE;

	/* READ CONFIG FILE */

	/* Get and open SPI device to use */
	strncpy(device, drvthis->config_get_string(drvthis->name, "Device", 0, DEFAULT_DEVICE),
		sizeof(device));
	device[sizeof(device) - 1] = '\0';
	report(RPT_INFO, "GLCD/ST7920: SPI: Using device '%s'", device);

	ct_data->fd = open(device, O_RDWR);
	if (ct_data->fd < 0) {
		report(RPT_ERR, "GLCD/ST7920: SPI: open spidev device '%s' failed: %s", device,
		       strerror(errno));
		return -1;
	}

 // set SPI mode !!!!!
  uint32_t mode = SPI_CS_HIGH;
   int status = ioctl(ct_data->fd, SPI_IOC_WR_MODE, &mode);
    if (status < 0) {
      report(RPT_ERR,
         "GLCD/ST7920: SPI: spidev set mode %u failed: %s", mode,
            status, strerror(errno));
              return -1;
               }
                 
                 mode = 0;
   status = ioctl(ct_data->fd, SPI_IOC_RD_MODE, &mode);
    if (status < 0) {
      report(RPT_ERR,
         "GLCD/ST7920: SPI: spidev read mode %u failed: %s", mode,
            status, strerror(errno));
              return -1;
               }
                 
  
	report(RPT_DEBUG, "HD44780: SPI: mode %02x", mode);
	
	debug(RPT_INFO, "GLCD/ST7920: Sending init to display...");

/*	spi_st7920_senddata(p, RS_INSTR, 0x30);
	spi_st7920_senddata(p, RS_INSTR, 0x0c);
	spi_st7920_senddata(p, RS_INSTR, 0x01);
	spi_st7920_senddata(p, RS_INSTR, 0x06);
	
	char str[] = "Test";
	
	spi_st7920_senddata(p, RS_INSTR, 0x80);
	for(int i = 0; i < sizeof(str) - 1; i++) {
		spi_st7920_senddata(p, RS_DATA, str[i]);
	}*/
	spi_st7920_senddata(p, RS_INSTR, 0x30);
	spi_st7920_senddata(p, RS_INSTR, 0x30);
	spi_st7920_senddata(p, RS_INSTR, 0x0c);
	spi_st7920_senddata(p, RS_INSTR, 0x34);
	spi_st7920_senddata(p, RS_INSTR, 0x34);
	spi_st7920_senddata(p, RS_INSTR, 0x36);

	debug(RPT_DEBUG, "GLCD/ST7920: init() done");

	return 0;
}


/**
 * API: Write the framebuffer to the display
 * \param p  Pointer to glcd driver's private date structure.
 */
void
glcd_st7920_blit(PrivateData *p)
{
	CT_st7920_data *ct_data = (CT_st7920_data *) p->ct_data;
	int ygroup, x, y, i;
	int temp;
	int tmp;
	
	int checksum = 0;
	
	for(ygroup = 0; ygroup < 64; ygroup++) {
		if(ygroup < 32) {
			x = 0x80;
			y = ygroup + 0x80;
		} else {
			x = 0x88;
			y = ygroup - 32 + 0x80;
		}
		
//		spi_st7920_senddata(p, RS_INSTR, 0x34);
		spi_st7920_senddata(p, RS_INSTR, y);
		spi_st7920_senddata(p, RS_INSTR, x);
//		spi_st7920_senddata(p, RS_INSTR, 0x30);
		
		tmp = ygroup * 16;
		
		for(i = 0; i < 16; i++) {
			temp = p->framebuf.data[tmp++];
			checksum += temp;
			spi_st7920_senddata(p, RS_DATA, temp);
		}
	}
	
//	spi_st7920_senddata(p, RS_INSTR, 0x34);
//	spi_st7920_senddata(p, RS_INSTR, 0x36);
	p->glcd_functions->drv_report(RPT_INFO, "GLCD/ST7920: blit - checksum: %i", checksum);
}


/**
 * API: Release low-level resources.
 * \param p  Pointer to glcd driver's private date structure.
 */
void
glcd_st7920_close(PrivateData *p)
{
	if (p->ct_data != NULL) {
		CT_st7920_data *ct_data = (CT_st7920_data *) p->ct_data;
		
		free(p->ct_data);
		p->ct_data = NULL;
	}
}


/**
 * Clears graphic memory.
 * \param p  Pointer to glcd driver's private date structure.
 */
static void
st7920_graphic_clear(PrivateData *p)
{
	CT_st7920_data *ct_data = (CT_st7920_data *) p->ct_data;
	int num = p->framebuf.size;

	p->glcd_functions->drv_debug(RPT_DEBUG, "GLCD/ST7920: Clearing graphic: %d bytes", num);
	p->glcd_functions->drv_report(RPT_INFO, "GLCD/ST7920: clear");
	
	spi_st7920_senddata(p, RS_INSTR, 0x30);
	spi_st7920_senddata(p, RS_INSTR, 0x01);
}
