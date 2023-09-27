/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "camera.h"
#include "sccb.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gc0308_regs.h"

#define cambus_write(addr, x)     sccb_write_byt(g_slv_addr, addr, x)
#define cambus_read(addr, x)      sccb_read_byt(g_slv_addr, addr, x)

#define ORGN

static const uint8_t default_regs[][2] = {
		{BANKSEL_RESET,			0x80},		// Reset Chip
		{BANKSEL_RESET,			0x00},		// Release from reset and select Page-0

		{AEC_MODE3,				0x10},   	// Turn-off AEC
		{AAAA_ENABLE,			0x55},   	// Turn-off AWB

		{EXPOSURE_MSB,			0x01},		// Set exposure time
		{EXPOSURE_LSB,			0x2c},

		{AWB_R_GAIN,			0x56},		// White balancing red channel gain
		{AWB_G_GAIN,			0x40},		// White balancing green channel gain
		{AWB_G_GAIN,			0x4a},		// White balancing blue channel gain
		///{0x22,0x57},   // Open AWB
		{HORIZONTAL_BLANKING,	0x00}, //0x6a},		// Horizontal blanking in pixel clock units
		{VERTICAL_BLANKING,		0x0b}, //0x70	// Vertical blankin
		{0x0f,0x00},
		{0xe2,0x00},   //anti-flicker step [11:8]
		{0xe3,0x96},   //anti-flicker step [7:0]
		{0xe4,0x01},   //exp level 1  16.67fps
		{0xe5,0xfe},
		{0xe6,0x02},   //exp level 2  12.5fps
		{0xe7,0x58},
		{0xe8,0x02},   //exp level 3  8.33fps
		{0xe9,0x58},
		{0xea,0x02},   //exp level 4  4.00fps
		{0xeb,0x58},
		{0xec,0x00},

		{ROW_START_MSB,				0x00},   //window
		{ROW_START_LSB,				0x10},
		{COLUMN_START_MSB,			0x00},
		{COLUMN_START_LSB,			0x00},
		{WINDOW_HEIGHT_MSB,			0x01},
		{WINDOW_HEIGHT_LSB,			0xe8},
		{WINDOW_WIDTH_MSB,			0x02},
		{WINDOW_WIDTH_LSB,			0x88},

		{CROP_WIN_MODE,				0x80},		// Disable Crop Mode
		{CROP_WIN_Y1,				0x40},
		{CROP_WIN_X1,				0x40},
		{CROP_WIN_HEIGHT_MSB,		0x00},		// 0x120 = 288		0x090 = 144
		{CROP_WIN_HEIGHT_LSB,		0x90},
		{CROP_WIN_WIDTH_MSB,		0x00},		// 0x160 = 352		0x0b0 = 176
		{CROP_WIN_WIDTH_LSB,		0xb0},


		{0x0d,0x02},
		{0x0e,0x02},
		{0x10,0x22},
		{0x11,0xfd},
		{0x12,0x2a},


		{0x13,0x00},
		{0x14,0x10},
		{0x15,0x0a},
		{0x16,0x05},

		{0x17,0x01},
		{0x18,0x44},
		{0x19,0x44},
		{0x1a,0x1e},
		{0x1b,0x00},
		{0x1c,0xc1},


		{0x1d,0x08},
		{0x1e,0x60},
		{0x1f,0x16},
		{0x20,0xff},
		{0x21,0xf8},
		{0x22,0x00},
		{OUTPUT_FORMAT,		0xa6},		// {0x24,0xa2},
		{OUTPUT_EN,			0x0f},
		{SYNC_MODE,			0x36},		// {0x26,0x06},
		{0x2f,0x01},
		{0x30,0xf7},
		{0x31,0x50},
		{0x32,0x00},
		{0x39,0x04},
		{0x3a,0x18},
		{0x3b,0x20},
		{0x3c,0x00},
		{0x3d,0x00},
		{0x3e,0x00},
		{0x3f,0x00},
		{0x50,0x10},
		{0x53,0x82},
		{0x54,0x80},
		{0x55,0x80},
		{0x56,0x82},
		{0x8b,0x40},
		{0x8c,0x40},
		{0x8d,0x40},
		{0x8e,0x2e},
		{0x8f,0x2e},
		{0x90,0x2e},
		{0x91,0x3c},
		{0x92,0x50},
		{0x5d,0x12},
		{0x5e,0x1a},
		{0x5f,0x24},
		{0x60,0x07},
		{0x61,0x15},
		{0x62,0x08},
		{0x64,0x03},
		{0x66,0xe8},
		{0x67,0x86},
		{0x68,0xa2},
		{0x69,0x18},
		{0x6a,0x0f},
		{0x6b,0x00},
		{0x6c,0x5f},
		{0x6d,0x8f},
		{0x6e,0x55},
		{0x6f,0x38},
		{0x70,0x15},
		{0x71,0x33},
		{0x72,0xdc},
		{0x73,0x80},
		{0x74,0x02},
		{0x75,0x3f},
		{0x76,0x02},
		{0x77,0x36},
		{0x78,0x88},
		{0x79,0x81},
		{0x7a,0x81},
		{0x7b,0x22},
		{0x7c,0xff},
		{0x93,0x48},
		{0x94,0x00},
		{0x95,0x05},
		{0x96,0xe8},
		{0x97,0x40},
		{0x98,0xf0},
		{0xb1,0x38},
		{0xb2,0x38},
		{0xbd,0x38},
		{0xbe,0x36},
		{0xd0,0xc9},
		{0xd1,0x10},
		{0xd3,0x80},
		{0xd5,0xf2},
		{0xd6,0x16},
		{0xdb,0x92},
		{0xdc,0xa5},
		{0xdf,0x23},
		{0xd9,0x00},
		{0xda,0x00},
		{0xe0,0x09},
		{0xed,0x04},
		{0xee,0xa0},
		{0xef,0x40},


		{0x80,0x03},
		{0x80,0x03},
		{0x9F,0x10},
		{0xA0,0x20},
		{0xA1,0x38},
		{0xA2,0x4E},
		{0xA3,0x63},
		{0xA4,0x76},
		{0xA5,0x87},
		{0xA6,0xA2},
		{0xA7,0xB8},
		{0xA8,0xCA},
		{0xA9,0xD8},
		{0xAA,0xE3},
		{0xAB,0xEB},
		{0xAC,0xF0},
		{0xAD,0xF8},
		{0xAE,0xFD},
		{0xAF,0xFF},
		{0xc0,0x00},
		{0xc1,0x10},
		{0xc2,0x1C},
		{0xc3,0x30},
		{0xc4,0x43},
		{0xc5,0x54},
		{0xc6,0x65},
		{0xc7,0x75},
		{0xc8,0x93},
		{0xc9,0xB0},
		{0xca,0xCB},
		{0xcb,0xE6},
		{0xcc,0xFF},
		{0xf0,0x02},
		{0xf1,0x01},
		{0xf2,0x01},
		{0xf3,0x30},
		{0xf9,0x9f},
		{0xfa,0x78},


		{BANKSEL_RESET,			0x01},	// Switch to Page-1
		{0x00,0xf5},
		{0x02,0x1a},
		{0x0a,0xa0},
		{0x0b,0x60},
		{0x0c,0x08},
		{0x0e,0x4c},
		{0x0f,0x39},
		{0x11,0x3f},
		{0x12,0x72},
		{0x13,0x13},
		{0x14,0x42},
		{0x15,0x43},
		{0x16,0xc2},
		{0x17,0xa8},
		{0x18,0x18},
		{0x19,0x40},


		{0x1a,0xd0},
		{0x1b,0xf5},
		{0x70,0x40},
		{0x71,0x58},
		{0x72,0x30},
		{0x73,0x48},
		{0x74,0x20},
		{0x75,0x60},
		{0x77,0x20},
		{0x78,0x32},
		{0x30,0x03},
		{0x31,0x40},
		{0x32,0xe0},
		{0x33,0xe0},
		{0x34,0xe0},
		{0x35,0xb0},
		{0x36,0xc0},
		{0x37,0xc0},
		{0x38,0x04},
		{0x39,0x09},
		{0x3a,0x12},
		{0x3b,0x1C},
		{0x3c,0x28},
		{0x3d,0x31},
		{0x3e,0x44},
		{0x3f,0x57},
		{0x40,0x6C},
		{0x41,0x81},
		{0x42,0x94},
		{0x43,0xA7},
		{0x44,0xB8},
		{0x45,0xD6},
		{0x46,0xEE},
		{0x47,0x0d},
		{0xfe,0x00}, // set page0
		{0xd2,0x90},
		{0xfe,0x00},// set page0
		{0x10,0x26},
		{0x11,0x0d},  // fd,modified by mormo 2010/07/06
		{0x1a,0x2a}, // 1e,modified by mormo 2010/07/06
		{0x1c,0x49},// c1,modified by mormo 2010/07/06
		{0x1d,0x9a},// 08,modified by mormo 2010/07/06
		{0x1e,0x61},// 60,modified by mormo 2010/07/06
		{0x3a,0x20},
		{0x50,0x14}, // 10,modified by mormo 2010/07/06
		{0x53,0x80},
		{0x56,0x80},
		{0x8b,0x20},//LSC
		{0x8c,0x20},
		{0x8d,0x20},
		{0x8e,0x14},
		{0x8f,0x10},
		{0x90,0x14},
		{0x94,0x02},
		{0x95,0x07},
		{0x96,0xe0},
		{0xb1,0x40},// YCPT
		{0xb2,0x40},
		{0xb3,0x40},
		{0xb6,0xe0},
		{0xd0,0xcb},// AECT  c9,modifed by mormo 2010/07/06
		{0xd3,0x48},// 80,modified by mormor 2010/07/06
		{0xf2,0x02},
		{0xf7,0x12},
		{0xf8,0x0a},
		//Registers of Page1
		{0xfe,0x01},// set page1
		{0x02,0x20},
		{0x04,0x10},
		{0x05,0x08},
		{0x06,0x20},
		{0x08,0x0a},
		{0x0e,0x44},
		{0x0f,0x32},
		{0x10,0x41},
		{0x11,0x37},
		{0x12,0x22},
		{0x13,0x19},
		{0x14,0x44},
		{0x15,0x44},
		{0x19,0x50},
		{0x1a,0xd8},
		{0x32,0x10},
		{0x35,0x00},
		{0x36,0x80},
		{0x37,0x00},



		//-----------Update the registers end---------//
		{0xfe,0x00},// set page0
		{0x9F,0x0E},
		{0xA0,0x1C},
		{0xA1,0x34},
		{0xA2,0x48},
		{0xA3,0x5A},
		{0xA4,0x6B},
		{0xA5,0x7B},
		{0xA6,0x95},
		{0xA7,0xAB},
		{0xA8,0xBF},
		{0xA9,0xCE},
		{0xAA,0xD9},
		{0xAB,0xE4},
		{0xAC,0xEC},
		{0xAD,0xF7},
		{0xAE,0xFD},
		{0xAF,0xFF},
		{0x14,0x10},

		{BANKSEL_RESET,			0x01},	// Switch to Page-1
	    //{SUBSAMPLE,				0x22},  // Sub-sample Horizontal:1/2  Vertical:1/2
		{SUBSAMPLE,				0x22},  // Sub-sample Horizontal:1/2  Vertical:1/2
		{SUB_MODE,				0x00},
		{BANKSEL_RESET,			0x00},

//#endif
		//-----------Update the registers end---------//
		{BANKSEL_RESET,		0x00},
		{BANKSEL_RESET,		0x00},
		{0xff,0xff},


};


static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_BAYER;

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;
//#if 0
    printf("\nScanning Camera On SCCB\n");
 ///   g_slv_addr = sccb_scan();

    if (g_slv_addr == -1) {
    	printf("Couldn't get ack from the camera\n");
    	while(1);
        return -1;
    }
    printf("The camera address : %0.2x \n", g_slv_addr);



//#else
    g_slv_addr = GC0308_I2C_SLAVE_ADDR;
   // uint8_t rev;
   // while(1) {



   //     cambus_read(REVISION, &rev);
    //    for (int i = 0 ; i < 100000 ; i++);


   // }
//#endif
    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int* id)
{
    int ret = 0;
    uint8_t rev;

    ret |= cambus_read(REVISION, &rev);
    *id = (int)rev;
    return ret;
}

static int get_manufacture_id(int* id)
{
    int ret = 0;
    uint8_t cam_id;

    ret = 0;
    ret |= cambus_read(MODEL, &cam_id);

    *id = (int) cam_id;
    return ret;
}

static int dump_registers(void)
{
    int ret = 0;
    uint8_t byt = 0;
    uint32_t i;
    uint8_t buf[64] = {0};
    uint8_t* ptr = buf;

    for (i = 0; ; i++) {

        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }

        if (i == 0x35B3) {
            break;
        }

        ret = cambus_read(i, &byt);

        if (ret == 0) {
            ret = sprintf((char*)ptr, " %02X", byt);

            if (ret < 0) {
                return ret;
            }

            ptr += 3;// XX + space
        }
        else {
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
        }
    }

    return ret;
}

static int reset(void)
{
    int ret = 0;

/*
    ret |= cambus_write(SW_RESET, 0x80);
    MXC_Delay(10000);
    ret |= cambus_write(SW_RESET, 0x00);
    MXC_Delay(100);
    int a = SW_RESET;
    printf("sw reset %0.2x \n", a);
*/


#if 1
    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xFF); i++) {
        ret |= cambus_write(default_regs[i][0], (uint8_t)default_regs[i][1]);
        printf("reg: 0x%04x , val: 0x%02x\r\n",default_regs[i][0], default_regs[i][1]);
        //if (ret)
        //    printf("fail");
    }
#endif

    printf("fail");
    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
    uint8_t reg;

  ///  ret = cambus_read(MODE_SELECT, &reg);

    if (ret == 0) {
        if (enable) {
          //  reg = STANDBY_MODE;
        }
        else {
          //  reg = SW_STEAMING_MODE;
        }

        // Write back register
  ///      ret |= cambus_write(MODE_SELECT, reg);
    }

    return ret;
}

static int read_reg(uint16_t reg_addr, uint8_t* reg_data)
{
    *reg_data = 0xff;

    if (cambus_read(reg_addr, reg_data) != 0) {
        return -1;
    }

    return 0;
}

static int write_reg(uint16_t reg_addr, uint8_t reg_data)
{
    return cambus_write(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;
    uint8_t buffer;

    g_pixelformat = pixformat;

    // Read the current register value and mask out the field we want to set
    ret |= cambus_read(OUTPUT_FORMAT, &buffer);
    buffer &= ~(0b11111);

    switch (pixformat) {
    case PIXFORMAT_RGB888:
    case PIXFORMAT_RGB565:
        ret |= cambus_write(OUTPUT_FORMAT, buffer | 0x06);
        break;
    case PIXFORMAT_RGB444:
        ret |= cambus_write(OUTPUT_FORMAT, buffer | 0x09);
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int get_pixformat(pixformat_t* pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}

static int set_framesize(int width, int height)
{
    int ret = 0;
#if 0
    // Image typically outputs one line short, add a line to account.
    //height = height + 1;
    // Apply passed in resolution as output resolution.

    ret |= cambus_write(WINDOW_WIDTH_MSB, (width >> 8) & 0xff);
    ret |= cambus_write(WINDOW_WIDTH_LSB, (width >> 0) & 0xff);
    ret |= cambus_write(WINDOW_HEIGHT_MSB, (height >> 8) & 0xff);
    ret |= cambus_write(WINDOW_HEIGHT_LSB, (height >> 0) & 0xff);

#endif


    return ret;
}

static int set_windowing(int width, int height, int hsize, int vsize)
{
    /* Note: width and height is used to control scaling size of the image
       width: horizontal input size
       height: vertical input size
       hsize: horizontal size of cropped image
       vsize: vertical size of cropped image
    */
    int ret = 0;

    if (width < hsize || height < vsize) {
        ret = -1;
    }

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;

    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;

    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;

    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;

    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;

    if (enable) {
		ret |= cambus_write(TEST_PATTERN_MODE, 0x11);
    }
    else {
        ret |= cambus_write(TEST_PATTERN_MODE, 0x0);
    }
    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;
/*
    ret = cambus_read(IMAGE_ORIENTATION, &reg);

    if (enable) {
        reg |= H_MIRROR;
    }
    else {
        reg &= ~H_MIRROR;
    }

    ret |= cambus_write(IMAGE_ORIENTATION, reg);
*/
    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;

    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;
/*
    ret = cambus_read(IMAGE_ORIENTATION, &reg);

    if (enable) {
        reg |= V_FLIP;
    }
    else {
        reg &= ~V_FLIP;
    }

    ret |= cambus_write(IMAGE_ORIENTATION, reg);
*/
    return ret;
}

static  int get_luminance(int* lum)
{
    int ret = 0;

    *lum = 0xFFFFFF;
    return ret;
}

/******************************** Public Functions ***************************/
int sensor_register(camera_t* camera)
{
    // Initialize sensor structure.
    camera->init                = init;
    camera->get_slave_address   = get_slave_address;
    camera->get_product_id      = get_product_id;
    camera->get_manufacture_id  = get_manufacture_id;
    camera->dump_registers      = dump_registers;
    camera->reset               = reset;
    camera->sleep               = sleep;
    camera->read_reg            = read_reg;
    camera->write_reg           = write_reg;
    camera->set_pixformat       = set_pixformat;
    camera->get_pixformat       = get_pixformat;
    camera->set_framesize       = set_framesize;
    camera->set_windowing       = set_windowing;
    camera->set_contrast        = set_contrast;
    camera->set_brightness      = set_brightness;
    camera->set_saturation      = set_saturation;
    camera->set_gainceiling     = set_gainceiling;
    camera->set_colorbar        = set_colorbar;
    camera->set_hmirror         = set_hmirror;
    camera->set_vflip           = set_vflip;
    camera->set_negateimage     = set_negateimage;
    camera->get_luminance       = get_luminance;
    return 0;
}


int context_switch(int enable, int context)
{
    int ret = 0;
    uint8_t reg;

    reg = 0;
/*
    if ( enable ) {
    	reg = reg & 0xf7;		// clear CXT_disable bit

    	if ( context == 1 )
    		reg = reg | 1;		// If Context B is selected, set bit 0

    	printf("Context Selected %d\n\n", reg);
    }
    else {
    	reg = reg | 0x08;		// set CXT disable bit
    	printf("Context Disabled %d\n\n", reg);

    }
*/
    ret |= cambus_write(PMU_CFG_3, reg);
    return ret;

}

int set_subsampling(int context, int h_sub, int v_sub, int h_binning, int v_binning)
{
    int ret = 0;


    uint8_t binreg = 0;

    if ( h_binning ==  1 ) {

    	binreg = binreg | 0x02;
    }
    if ( v_binning ==  1 ) {

    	binreg = binreg | 0x01;
    }

/*
    switch ( context ) {

    	case 0 :   ret |= cambus_write(H_SUB, h_sub);
    			   ret |= cambus_write(V_SUB, v_sub);
    			   ret |= cambus_write(BINNING_MODE, binreg);
    			   printf("Subsampling Binning Mode %d %d %d\n", h_sub, v_sub,  binreg);
    			   break;
    	case 1 :   ret |= cambus_write(H_SUB_CTXA, h_sub);
    			   ret |= cambus_write(V_SUB_CTXA, v_sub);
    			   ret |= cambus_write(BINNING_MODE_CTXA, binreg);
    			   break;
    	case 2 :   ret |= cambus_write(H_SUB_CTXB, h_sub);
    			   ret |= cambus_write(V_SUB_CTXB, v_sub);
    			   ret |= cambus_write(BINNING_MODE_CTXB, binreg);
    			   break;
    	default:
    			   break;
    }
*/

    return ret;

}


