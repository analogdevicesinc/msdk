/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/*-------------------------------------------------------------------------------------------------
 *  SHARP memory in pixel monochrome display series
 *      LS012B7DD01 (184x38  pixels.)
 *      LS013B7DH03 (128x128 pixels.)
 *      LS013B7DH05 (144x168 pixels.)
 *      LS027B7DH01 (400x240 pixels.) (tested)
 *      LS032B7DD02 (336x536 pixels.)
 *      LS044Q7DH01 (320x240 pixels.)
 *
 *  These displays need periodic com inversion, there are two ways :
 *    - software com inversion :
 *      define SHARP_MIP_SOFT_COM_INVERSION 1 and set EXTMODE display pin LOW,
 *      call sharp_mip_com_inversion() periodically
 *    - hardware com inversion with EXTCOMIN display pin :
 *      define SHARP_MIP_SOFT_COM_INVERSION 0,
 *      set EXTMODE display pin HIGH and handle
 *      EXTCOMIN waveform (for example with mcu pwm output),
 *      see datasheet pages 8-12 for details
 *
 *  draw_buf size : (LV_VER_RES / X) * (2 + LV_HOR_RES / 8) + 2 bytes, structure :
 *      [FRAME_HEADER (1 byte)] [GATE_ADDR (1 byte )] [LINE_DATA (LV_HOR_RES / 8 bytes)]  1st  line
 *      [DUMMY        (1 byte)] [GATE_ADDR (1 byte )] [LINE_DATA (LV_HOR_RES / 8 bytes)]  2nd  line
 *      ...........................................................................................
 *      [DUMMY        (1 byte)] [GATE_ADDR (1 byte )] [LINE_DATA (LV_HOR_RES / 8 bytes)]  last line
 *      [DUMMY                             (2 bytes)]
 *
 *  Since extra bytes (dummy, addresses, header) are stored in draw_buf, we need to use
 *  an "oversized" draw_buf. Buffer declaration in "lv_port_disp.c" becomes for example :
 *      static lv_disp_buf_t disp_buf;
 *      static uint8_t buf[(LV_VER_RES_MAX / X) * (2 + (LV_HOR_RES_MAX / 8)) + 2];
 *      lv_disp_buf_init(&disp_buf, buf, NULL, LV_VER_RES_MAX * LV_HOR_RES_MAX / X);
 *-----------------------------------------------------------------------------------------------*/

#include "sharp_mip.h"
#include <stdbool.h>

/************************************ DEFINES ********************************/
#define SHARP_MIP_REV_BYTE(x)         ( \
  ((x & 0x01) << 7) | ((x & 0x02) << 5) | ((x & 0x04) << 3) | ((x & 0x08) << 1) | \
  ((x & 0x80) >> 7) | ((x & 0x40) >> 5) | ((x & 0x20) >> 3) | ((x & 0x10) >> 1))  /*Architecture / compiler dependent byte bits order reverse*/

#define SHARP_MIP_HEADER              0
#define SHARP_MIP_CLEAR_SCREEN_FLAG   (1 << 5)  /* (M2) All clear flag : H -> clear all pixels               */
#define SHARP_MIP_COM_INVERSION_FLAG  (1 << 6)  /* (M1) Frame inversion flag : relevant when EXTMODE = L,    */
                                                /*      H -> outputs VCOM = H, L -> outputs VCOM = L         */
#define SHARP_MIP_UPDATE_RAM_FLAG     (1 << 7)  /* (M0) Mode flag : H -> update memory, L -> maintain memory */

#define BUFIDX(x, y)  (((x) >> 3) + ((y) * (2 + (dev->init_param.col >> 3))) + 2)
#define PIXIDX(x)     SHARP_MIP_REV_BYTE(1 << ((x) & 7))

#define BIT_SET(a,b) (a |= b)
#define BIT_CLEAR(a,b) (a &= ~b)

/***** Static Functions *****/
static void sharp_mip_send_payload(sharp_mip_dev* dev, uint8_t* payload, uint32_t payloadlen)
{
	dev->comm_api.write(payload, payloadlen);
}

/***** Functions *****/
int sharp_mip_configure(sharp_mip_dev* dev, sharp_mip_init_param_t* init_param, display_comm_api* comm_api)
{
    int err = DISP_E_SUCCESS;

    if (init_param == NULL || comm_api == NULL || dev == NULL) {
        return DISP_E_BAD_PARAM;
    }
	dev->init_param = *init_param;
	dev->comm_api = *comm_api;
	return err;
}

int sharp_mip_init(sharp_mip_dev* dev)
{
    if (dev == NULL) {
        return DISP_E_NOT_CONFIGURED;
    }
	dev->comm_api.init();
	return DISP_E_SUCCESS;
}

void sharp_mip_flush_area(sharp_mip_dev *dev, const display_area_t* area, const uint8_t* data)
{
	/*Return if the area is out the screen*/
	if(area->y2 < 0) return;
	if(area->y1 > dev->init_param.row - 1) return;

	/*Truncate the area to the screen*/
	uint16_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	uint16_t act_y2 = area->y2 > dev->init_param.row - 1 ? dev->init_param.row - 1 : area->y2;

	uint8_t * buf      = (uint8_t*) data;                     			/*Get the buffer address*/
	uint16_t  buf_h    = (act_y2 - act_y1 + 1);                			/*Number of buffer lines*/
	uint16_t  buf_size = buf_h * (2 + dev->init_param.col / 8) + 2;		/*Buffer size in bytes  */

	/* Set lines to flush dummy byte & gate address in draw_buf*/
	for(uint16_t act_y = 0 ; act_y < buf_h ; act_y++) {
		buf[BUFIDX(0, act_y) - 1] = SHARP_MIP_REV_BYTE((act_y1 + act_y + 1));
		buf[BUFIDX(0, act_y) - 2] = 0;
	}

	/* Set last dummy two bytes in draw_buf */
	buf[BUFIDX(0, buf_h) - 1] = 0;
	buf[BUFIDX(0, buf_h) - 2] = 0;

	/* Set frame header in draw_buf */
	buf[0] = SHARP_MIP_HEADER | SHARP_MIP_UPDATE_RAM_FLAG;

	/* Write the frame on display memory */
	sharp_mip_send_payload(dev, buf, buf_size);
}

void sharp_mip_set_buffer_pixel_util(sharp_mip_dev *dev, uint8_t* buf, uint16_t buf_w, uint16_t x, uint16_t y, uint8_t color, uint8_t is_opaque)
{
	//TODO:: check buf_w with dev->init_param.col!
	if ( color == 0 && is_opaque) {
		BIT_SET( buf[BUFIDX(x, y)] , PIXIDX(x) );		/*Set draw_buf pixel bit to 1 for other colors than BLACK*/
	} else {
		BIT_CLEAR( buf[BUFIDX(x, y)] , PIXIDX(x) );	/*Set draw_buf pixel bit to 0 for BLACK color*/
	}
}

void sharp_mip_com_inversion(sharp_mip_dev *dev, int inversion_on) {
	uint8_t inversion_header[2] = {0};

	/* Set inversion header */
	if (inversion_on) {
		inversion_header[0] |= SHARP_MIP_COM_INVERSION_FLAG;
	}

	/* Write inversion header on display memory */
	sharp_mip_send_payload(dev, inversion_header, 2);
}
