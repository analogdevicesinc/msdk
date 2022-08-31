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
 * @brief   Parallel camera example with the OV7692/OV5642/HM01B0/HM0360 camera sensors as defined
 * in the makefile.
 *
 * @details This example uses the UART to stream out the image captured from the camera.
 *          Alternatively, it can display the captured image on TFT is it is enabled in the make
 * file. The image is prepended with a header that is interpreted by the grab_image.py python
 * script.  The data from this example through the UART is in a binary format. Instructions: 1) Load
 * and execute this example. The example will initialize the camera and start the repeating binary
 * output of camera frame data. 2) Run 'sudo grab_image.py /dev/ttyUSB0 921600' Substitute the
 * /dev/ttyUSB0 string for the serial port on your system. The python program will read in the
 * binary data from this example and output a png image.
 */

/***** Includes *****/
#include "board.h"
#include "camera.h"
#include "dma.h"
#include "led.h"
#include "mxc.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "uart.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>

#define CAMERA_FREQ (10 * 1000 * 1000)

// Display captured image on TFT. Disable if image is sent to serial interface
#define ENABLE_TFT

#if defined(CAMERA_HM01B0)
#define IMAGE_XRES 324 / 2
#define IMAGE_YRES 244 / 2
#define CAMERA_MONO
#endif

#if defined(CAMERA_HM0360)
#define IMAGE_XRES 320
#define IMAGE_YRES 240
#define CAMERA_MONO
#endif

#if defined(CAMERA_OV7692) || defined(CAMERA_OV5642)
#define IMAGE_XRES 320
#define IMAGE_YRES 240

#endif

#define CON_BAUD                                                                                   \
    115200 * 8 // UART baudrate used for sending data to PC, use max 921600 for serial stream
#define X_START 0
#define Y_START 0

void process_img(void)
{
    uint8_t* raw;
    uint32_t imgLen;
    uint32_t w, h;

    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);

    // buffers the entire image and send to serial or TFT
#ifndef ENABLE_TFT
    // Send the image through the UART to the console if no TFT
    // A python program will read from the console and write
    // to an image file
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
#else
#ifndef CAMERA_MONO
    // Send the image to TFT
    MXC_TFT_ShowImageCameraRGB565(X_START, Y_START, raw, w, h);
#else
    MXC_TFT_ShowImageCameraMono(X_START, Y_START, raw, h, w);
#endif // #ifndef CAMERA_MONO
#endif // ##ifndef ENABLE_TFT
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
    printf("\n\nCameraIF Example\n");

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    // Obtain product ID of the camera.
    ret = camera_get_product_id(&id);

    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    printf("Camera ID detected: %04x\n", id);

#if defined(CAMERA_HM01B0) || defined(CAMERA_HM0360) || defined(CAMERA_OV5642)
    camera_set_hmirror(0);
    camera_set_vflip(0);
#endif

#if defined(CAMERA_OV7692)
    camera_set_hmirror(0);
#endif

#ifdef ENABLE_TFT
    printf("Init TFT\n");
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetBackGroundColor(4);
#endif

    // Setup the camera image dimensions, pixel format and data acquiring details.
#ifndef CAMERA_MONO
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, USE_DMA,
        dma_channel); // RGB565
#else
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_BAYER, FIFO_FOUR_BYTE, USE_DMA,
        dma_channel); // Mono
#endif

    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    // Start capturing a first camera image frame.
    printf("Capture image\n");
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
            LED_Toggle(LED1);
            camera_start_capture_image();
        }
    }

    return ret;
}
