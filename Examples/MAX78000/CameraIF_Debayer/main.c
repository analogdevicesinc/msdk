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

/**
 * @file    main.c
 * @brief   Parallel camera example with the OV7692/OV5642/HM01B0/HM0360 camera sensors as defined in the makefile.
 *
 * @details This example uses the UART to stream out the image captured from the camera.
 *          Alternatively, it can display the captured image on TFT is it is enabled in the make file.
 *          The image is prepended with a header that is interpreted by the grab_image.py
 *          python script.  The data from this example through the UART is in a binary format.
 *          Instructions: 1) Load and execute this example. The example will initialize the camera
 *                        and start the repeating binary output of camera frame data.
 *                        2) Run 'sudo grab_image.py /dev/ttyUSB0 115200'
 *                           Substitute the /dev/ttyUSB0 string for the serial port on your system.
 *                           The python program will read in the binary data from this example and
 *                           output a png image.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "uart.h"
#include "led.h"
#include "board.h"

#include "camera.h"
#include "utils.h"
#include "dma.h"


#ifdef ENABLE_TFT

#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#endif

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

#endif


#define CAMERA_FREQ (10 * 1000 * 1000)

#define IMAGE_XRES  160
#define IMAGE_YRES  120


#define CON_BAUD    115200*8   //UART baudrate used for sending data to PC, use max 921600 for serial stream
#define X_START     0
#define Y_START     0

//#define GRADPAT

static uint8_t* bayer_data;

// // Receives Raw image, performs bilinear bayer interpolation
// void bayer_bilinear_demosaicing(uint8_t* srcimg, uint32_t w,  uint32_t h, uint16_t* dstimg ) {

// 	uint8_t *srcLinePrev;
// 	uint8_t *srcLineCurr;
// 	uint8_t *srcLineNext;

// 	unsigned short r,g,b;
// 	unsigned short rgb565;

// 	uint16_t* dstimgO;
// 	dstimgO=dstimg;

// 	srcLinePrev = srcimg - ( w );
// 	srcLineCurr = srcimg;
// 	srcLineNext = srcimg + ( w );

// 	int pixelcount = 0;
// 	int linecount = 0;

// 	unsigned short lsb, msb;

// 	int y;
// 	for (y=0; y < IMAGE_YRES; y++) {

// 		if ( (y & 0x01 )  ) {
// 				// Odd numbered line
// 		    	//	G R G R G R Line
// 				pixelcount = 0;
// 				for (int x = 0; x < IMAGE_XRES >> 1; x++) {
// 	    				r = ( *(srcLineCurr - 1 ) + *(srcLineCurr + 1) ) >> 1;
// 	    				g =   *(srcLineCurr);									// we have green
// 	    				b = ( *(srcLineNext     ) + *(srcLinePrev    ) ) >> 1;
// 						srcLinePrev++;
// 						srcLineCurr++;
// 						srcLineNext++;
// #ifdef GRADPAT
// 						r = (y>>1)+(x>>1);
// 						g = (y>>1)+(x>>1);
// 						b = (y>>1)+(x>>1);
// #endif
// 						*dstimg = ((b&0xf8)<<5) | ( (g&0x1c) << 11) | (r&0xf8) | ( (g&0xe0) >> 5);
// 			    		dstimg++;

// 						r = ( *(srcLineCurr  )); //// we have red
// 		    			g = ( *(srcLineCurr-1) + *(srcLineCurr+1) + *(srcLineNext  ) + *(srcLinePrev  ) ) >> 2;
// 		    			b = ( *(srcLineNext-1) + *(srcLineNext+1) + *(srcLinePrev-1) + *(srcLinePrev+1) ) >> 2;
// 						srcLinePrev++;
// 						srcLineCurr++;
// 						srcLineNext++;
// #ifdef GRADPAT
// 						r = (y>>1)+(x>>1);
// 						g = (y>>1)+(x>>1);
// 						b = (y>>1)+(x>>1);
// #endif
// 						*dstimg = ((b&0xf8)<<5) | ( (g&0x1c) << 11) | (r&0xf8) | ( (g&0xe0) >> 5);
// 			    		dstimg++;
// 			    		pixelcount++;
// 				}

// 			}
// 			else {
// 				// Even numbered line
// 			    // B G B G B G Line
// 				pixelcount = 0;
// 				for (int x = 0; x < 80; x++) {
// 						r = ( *(srcLinePrev-1) + *(srcLinePrev+1) + *(srcLineNext-1) + *(srcLineNext+1)) >> 2;
// 						g = ( *(srcLineCurr-1) + *(srcLineCurr+1) + *(srcLineNext  ) + *(srcLinePrev  )) >> 2;
// 						b =   *(srcLineCurr  );		// we have blue

// 						srcLinePrev++;
// 						srcLineCurr++;
// 						srcLineNext++;
// #ifdef GRADPAT
// 						r = (y>>1)+(x>>1);
// 						g = (y>>1)+(x>>1);
// 						b = (y>>1)+(x>>1);
// #endif
// 						*dstimg = ((b&0xf8)<<5) | ( (g&0x1c) << 11) | (r&0xf8) | ( (g&0xe0) >> 5);
// 			    		dstimg++;
// 			    		pixelcount++;

// 						r = ( *(srcLineNext  ) + *(srcLinePrev  ) ) >> 1;
// 						g =   *(srcLineCurr  );			// we have green
// 						b = ( *(srcLineCurr-1) + *(srcLineCurr+1) ) >> 1;
// 						srcLinePrev++;
// 						srcLineCurr++;
// 						srcLineNext++;
// #ifdef GRADPAT
// 						r = (y>>1)+(x>>1);
// 						g = (y>>1)+(x>>1);
// 						b = (y>>1)+(x>>1);
// #endif
// 						*dstimg = ((b&0xf8)<<5) | ( (g&0x1c) << 11) | (r&0xf8) | ( (g&0xe0) >> 5);
// 			    		dstimg++;
// 			    		pixelcount++;
// 				}

// 		}
// 		 linecount++;
// 	}

// }



void process_img(void)
{
    uint8_t*   raw;
    uint32_t  imgLen;
    uint32_t  w, h;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    if ( bayer_data ) {
    	// bayer_bilinear_demosaicing(raw, w, h, bayer_data );
    	bayer_malvarcutler_demosaicing(raw, w, h, (uint16_t *)bayer_data);
        MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, bayer_data, h, w);
    }

	// initialize the communication by providing image format and size
	utils_stream_img_to_pc_init(bayer_data, imgLen*2, w, h, (uint8_t *)"RGB565");

    // Get image line by line
    for (int i = 0; i < h; i++) {

		// Send one line to PC
		utils_stream_image_row_to_pc(bayer_data + (i*w*2), w*2);

	}
}

void UART_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(CONSOLE_UART));
    printf("TTY\n");
}

// *****************************************************************************
int main(void)
{
	int ret = 0;
    int slaveAddress;
    int id;
    int dma_channel;

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    mxc_uart_regs_t* ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);

    if ((ret = MXC_UART_Init(ConsoleUart, CON_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return ret;
    }

    // Reset Console UART
    MXC_UART_ClearRXFIFO(MXC_UART_GET_UART(CONSOLE_UART));
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(CONSOLE_UART), UART_Handler);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);
    printf("\n\nCamera Example\n");

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    // Obtain the manufacturer ID of the camera.
    ret = camera_get_manufacture_id(&id);

    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    printf("Camera ID detected: %04x\n", id);

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || defined(CAMERA_OV5642)
    camera_set_hmirror(0);
    camera_set_vflip(0);
#endif

#ifdef ENABLE_TFT
    printf("Init TFT\n");
    /* Initialize TFT display */
#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
#endif
#ifdef BOARD_FTHR_REVA
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
#endif
    MXC_TFT_SetBackGroundColor(4);
#endif
    // Setup the camera image dimensions, pixel format and data acquiring details.
//#ifndef STREAM_ENABLE


    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_BAYER, FIFO_FOUR_BYTE, USE_DMA, dma_channel); // RGB565


#ifdef ENABLE_TFT
    // Set the screen rotation
    //MXC_TFT_SetRotation(SCREEN_NORMAL);
    MXC_TFT_SetRotation(SCREEN_ROTATE);
    // Change entry mode settings
    MXC_TFT_WriteReg(0x0011, 0x6858);
#endif


    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    MXC_Delay(SEC(1));

#if defined(CAMERA_OV7692) && defined(STREAM_ENABLE)
    camera_write_reg(0x11, 0x6); // set camera clock prescaller to prevent streaming overflow for QVGA
#endif

    // Start capturing a first camera image frame.
    printf("Starting\n");
    camera_start_capture_image();

    bayer_data = (uint8_t*)malloc(2 * 160 * 120);

    while (1) {
        // Check if image is acquired.
#ifndef STREAM_ENABLE
        if (camera_is_image_rcv())
#endif
        {
            // Process the image, send it through the UART console.
            process_img();

            // Prepare for another frame capture.
            LED_Toggle(LED1);
            camera_start_capture_image();
        }
    }

    return ret;
}
