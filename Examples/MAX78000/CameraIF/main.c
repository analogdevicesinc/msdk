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

/**
 * @file    main.c
 * @brief   Parallel camera example with the OV7692/OV5642/HM01B0/HM0360/PAG7920 camera sensors as defined in the makefile.
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
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "uart.h"
#include "led.h"
#include "board.h"

#include "camera.h"
#include "utils.h"
#include "dma.h"

// Configuration options
// ------------------------
#define ENABLE_TFT // Comment out to disable TFT and send image to serial port instead.
#define STREAM_ENABLE
/* If enabled, camera is setup in streaming mode to send the image
line by line to TFT, or serial port as they are captured. Otherwise, it buffers the entire
image first and then sends to TFT or serial port.
With serial port set at 900kbps, it can stream for up to 80x80 with OV5642 camera in 
stream mode, or 176x144 when stream mode is disabled.  It can display on TFT up to 176x144 
if stream mode is disabled, or 320x240 if enabled
*/
// #define BUTTON
/*
If BUTTON is defined, you'll need to push PB1 to capture an image frame.  Otherwise, images
will be captured continuously.
*/

// ------------------------

/*
Compiler definitions...  These configure TFT and camera settings based on the options above
*/
#ifdef ENABLE_TFT

#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#endif

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

#endif

#define CAMERA_FREQ (10 * 1000 * 1000)

// Match image dimensions to the selected camera and capture mode.
// These definitions, including "CAMERA_MONO", come from board.mk

#if defined(CAMERA_HM01B0)

#ifdef STREAM_ENABLE
#define IMAGE_XRES 324 / 2
#define IMAGE_YRES 244 / 2

#else
#define IMAGE_XRES 80
#define IMAGE_YRES 80

#endif
#endif

#if defined(CAMERA_HM0360_MONO) || defined(CAMERA_HM0360_COLOR) || defined(CAMERA_PAG7920)

#ifdef STREAM_ENABLE
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#else
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#endif
#endif

#if defined(CAMERA_OV7692) || defined(CAMERA_OV5642)

#ifdef ENABLE_TFT
#ifdef STREAM_ENABLE
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#else
#define IMAGE_XRES 176
#define IMAGE_YRES 144
#endif

#else
#ifdef STREAM_ENABLE
#define IMAGE_XRES 80
#define IMAGE_YRES 80
#else
#define IMAGE_XRES 176
#define IMAGE_YRES 144
#endif

#endif
#endif

#define CON_BAUD \
    115200 * 8 //UART baudrate used for sending data to PC, use max 921600 for serial stream
#define X_START 0
#define Y_START 0

void process_img(void)
{
    uint8_t* raw;
    uint32_t imgLen;
    uint32_t w, h;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

#ifndef STREAM_ENABLE
    // buffers the entire image and send to serial or TFT
#ifndef ENABLE_TFT
    // Send the image through the UART to the console if no TFT
    // A python program will read from the console and write
    // to an image file
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
#else
#ifndef CAMERA_MONO
    // Send the image to TFT
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, h, w);
#else
    MXC_TFT_ShowImageCameraMono(X_START, Y_START, raw, h, w);
#endif // #ifndef CAMERA_MONO
#endif // ##ifndef ENABLE_TFT

#else // #ifndef STREAM_ENABLE
    // STREAM mode
    uint8_t* data = NULL;
    // send image line by line to PC, or TFT
#ifndef ENABLE_TFT
    // initialize the communication by providing image format and size
    utils_stream_img_to_pc_init(raw, imgLen, w, h, camera_get_pixel_format());
#endif

    // Get image line by line
    for (int i = 0; i < h; i++) {
        // Wait until camera streaming buffer is full
        while ((data = get_camera_stream_buffer()) == NULL) {
            if (camera_is_image_rcv()) {
                break;
            }
        };

#ifndef ENABLE_TFT
        // Send one line to PC
        utils_stream_image_row_to_pc(data, w * 2);

#else
#ifndef CAMERA_MONO
        // Send one line to TFT
        MXC_TFT_ShowImageCameraRGB565(X_START, Y_START + i, data, w, 1);

#else
        MXC_TFT_ShowImageCameraMono(X_START, Y_START + i, data, w, 1);

#endif
#endif
        // Release stream buffer
        release_camera_stream_buffer();
    }

    stream_stat_t* stat = get_camera_stream_statistic();

    //printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    //printf("OVERFLOW = %d\n", stat->overflow_count);
    if (stat->overflow_count > 0) {
        LED_On(LED_RED); // Turn on red LED if overflow detected

        while (1)
            ;
    }

#endif //#ifndef STREAM_ENABLE
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
#ifndef STREAM_ENABLE
#ifndef CAMERA_MONO
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, USE_DMA,
                       dma_channel); // RGB565
#else
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_BAYER, FIFO_FOUR_BYTE, USE_DMA,
                       dma_channel); // Mono
#endif

#ifdef ENABLE_TFT
    /* Set the screen rotation */
    MXC_TFT_SetRotation(SCREEN_ROTATE);
    /* Change entry mode settings */
    MXC_TFT_WriteReg(0x0011, 0x6858);
#endif
#else
#ifndef CAMERA_MONO
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, STREAMING_DMA,
                       dma_channel); // RGB565 stream
#else
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_BAYER, FIFO_FOUR_BYTE, STREAMING_DMA,
                       dma_channel); // Mono stream
#endif

#ifdef ENABLE_TFT
    /* Set the screen rotation */
#ifdef BOARD_EVKIT_V1
    MXC_TFT_SetRotation(SCREEN_NORMAL);
#endif
#ifdef BOARD_FTHR_REVA
    MXC_TFT_SetRotation(ROTATE_270);
#endif

#endif
#endif //#ifndef STREAM_ENABLE

    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    MXC_Delay(SEC(1));

#if defined(CAMERA_OV7692) && defined(STREAM_ENABLE)
    // set camera clock prescaller to prevent streaming overflow for QVGA
#ifdef BOARD_EVKIT_V1
    camera_write_reg(0x11, 0x8); // can be set to 0x6 in release mode ( -o2 )
#endif
#ifdef BOARD_FTHR_REVA
    camera_write_reg(0x11, 0xE); // can be set to 0xB in release mode ( -o2 )
#endif
#endif

    // Start capturing a first camera image frame.
    printf("Starting\n");
#ifdef BUTTON
    while (!PB_Get(0))
        ;
#endif
    camera_start_capture_image();

    while (1) {
        // Check if image is acquired.
#ifndef STREAM_ENABLE
        if (camera_is_image_rcv())
#endif
        {
            // Process the image, send it through the UART console.
            process_img();

            // Prepare for another frame capture.
            LED_Toggle(LED_GREEN);
#ifdef BUTTON
            while (!PB_Get(0))
                ;
#endif
            camera_start_capture_image();
        }
    }

    return ret;
}
