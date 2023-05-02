/**
 * @file    main.c
 * @brief   MIPI CSI-2 Demo.
 * @details This example captures an image using a MIPI Camera and an CSI-2 Receiver.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "csi2.h"
#include "csi2_regs.h"
#include "dma.h"
#include "icc.h"
#include "pb.h"
#include "led.h"
#include "board.h"
#include "nvic_table.h"
#include "mipi_camera.h"
#include "utils.h"
#include "gcr_regs.h"
#include "mcr_regs.h"
#include "console.h"
#include "aps6404.h"
#include "tft_st7789v.h"
#include "fastspi.h"

/***** Definitions *****/

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 256
// #define RAW

#ifndef RAW
#define PIXEL_FORMAT PIXEL_FORMAT_RGB565
#define PIXEL_ORDER PIXEL_ORDER_RGB565_RGB
#define BYTES_PER_PIXEL 2
#else
#define PIXEL_FORMAT PIXEL_FORMAT_RAW8
#define PIXEL_ORDER PIXEL_ORDER_RAW_BGGR
#define BYTES_PER_PIXEL 1
#endif

// Size of Image Buffer
// This buffer will hold the full output image after the CSI2 has converted it
// to color.
// #define IMAGE_SIZE ((BITS_PER_PIXEL_ODD + BITS_PER_PIXEL_EVEN) * IMAGE_WIDTH * IMAGE_HEIGHT) >> 4
#define IMAGE_SIZE \
    (IMAGE_WIDTH * IMAGE_HEIGHT * BYTES_PER_PIXEL_EVEN) + (IMAGE_WIDTH * BYTES_PER_PIXEL_EVEN)
// ^ The addition of the extra row above is a workaround to another CSI2 hardware and/or driver issue.
// When writing to the output image buffer the CSI2 block will overflow beyond the bounds of the array
// by a variable amount and cause a hard fault.  At 160x120 it overflows by exactly 5 bytes.  At 320x240
// It overflows by somewhere between 64 and 96 bytes (Exact # TBD by sheer trial and error).
// Extending the output array by this extra row seems to be a reliable workaround to this issue.

// Update for future cameras
#if defined(CAMERA_OV5640)
#define CAMERA_ID 0x5640
#else // Default
#define CAMERA_ID 0x5640
#endif

/***** Globals *****/

// RAW Line Buffers
// These buffers are used by the CSI2 hardware for debayering.
// The size of these is hard-coded to 2048 because there appears
// to be some instability in the CSI2 hardware if they are scaled
// based on the image dimensions
// __attribute__((section(".csi2_buff_raw0"))) uint32_t RAW_ADDR0[2048];
// __attribute__((section(".csi2_buff_raw1"))) uint32_t RAW_ADDR1[2048];

// Buffer for processed image
// __attribute__((section(".csi2_img_buff"))) uint8_t IMAGE[IMAGE_SIZE] = { 0 };
unsigned int g_index = 0;

// uint8_t img_buffer[IMAGE_WIDTH * IMAGE_HEIGHT * BYTES_PER_PIXEL];

/***** Functions *****/

int CSI2_line_handler(uint8_t* data, unsigned int len)
{
    ram_write_quad(g_index, data, len);
    g_index += len;
    return E_NO_ERROR;
}

void process_img(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    printf("Capturing image...\n");
    spi_init();
    ram_enter_quadmode();

    g_index = 0;
    MXC_TMR_SW_Start(MXC_TMR0);
    int error = mipi_camera_capture();
    unsigned int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    if(error) {
        printf("Failed!\n");
        mxc_csi2_capture_stats_t stats = MXC_CSI2_GetCaptureStats();
        printf("CTRL Error flags: 0x%x\tPPI Error flags: 0x%x\tVFIFO Error flags: 0x%x\n", stats.ctrl_err, stats.ppi_err, stats.vfifo_err);
        return;
    }
    printf("Done! (took %i us)\n", elapsed);

    // Get the details of the image from the camera driver.
    MXC_CSI2_GetImageDetails(&raw, &imgLen, &w, &h);

    MXC_TMR_SW_Start(MXC_TMR0);
    // Send image header
    clear_serial_buffer();
    send_msg(mipi_camera_get_image_header());

    // Send image data
    for (int i = 0; i < imgLen; i += SERIAL_BUFFER_SIZE) {
        // cnn_addr = read_bytes_from_cnn_sram((uint8_t *)g_serial_buffer, transfer_len, cnn_addr);
        ram_read_quad(i, (uint8_t*)g_serial_buffer, SERIAL_BUFFER_SIZE);
        MXC_UART_WriteBytes(Con_Uart, (uint8_t *)g_serial_buffer, SERIAL_BUFFER_SIZE);
    }
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (serial transmission took %i us)\n", elapsed);

    printf("Writing to TFT Display...\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    TFT_SPI_Init();

    uint8_t tft_buffer[IMAGE_WIDTH*BYTES_PER_PIXEL];
    unsigned int address = 0;
    for (int y = 0; y < 240; y++) {
        spi_init();
        ram_enter_quadmode();
        ram_read_quad(address, tft_buffer, IMAGE_WIDTH*2);
        TFT_SPI_Init();
        MXC_TFT_WriteBufferRGB565(0, y, tft_buffer, IMAGE_WIDTH, 1);
        address += IMAGE_WIDTH*2;
    }
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (took %i us)\n", elapsed);
}

void service_console(cmd_t cmd)
{
    // Process the received command...
    if (cmd == CMD_UNKNOWN) {
        printf("Uknown command '%s'\n", g_serial_buffer);
    } else if (cmd == CMD_HELP) {
        print_help();
    } else if (cmd == CMD_RESET) {
        // Issue a soft reset
        MXC_GCR->rst0 |= MXC_F_GCR_RST0_SYS;
    } else if (cmd == CMD_CAPTURE) {
        process_img();
    } else if (cmd == CMD_SETREG) {
        // Set a camera register
        unsigned int reg;
        unsigned int val;
        // ^ Declaring these as unsigned ints instead of uint8_t
        // avoids some issues caused by type-casting inside of sscanf.

        sscanf(g_serial_buffer, "%s %u %u", cmd_table[cmd], &reg, &val);
        printf("Writing 0x%x to camera reg 0x%x\n", val, reg);
        mipi_camera_write_reg((uint16_t)reg, (uint16_t)val);
    } else if (cmd == CMD_GETREG) {
        // Read a camera register
        unsigned int reg;
        uint8_t val;
        sscanf(g_serial_buffer, "%s %u", cmd_table[cmd], &reg);
        mipi_camera_read_reg((uint16_t)reg, &val);
        snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE, "Camera reg 0x%x=0x%x", reg, val);
        send_msg(g_serial_buffer);
    }
}

volatile int buttonPressed = 0;
void buttonHandler()
{
    buttonPressed = 1;
}

int main(void)
{
    int error;
    int id;

    console_init();

    // Enable cache
    MXC_ICC_Enable(MXC_ICC0);

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("\n\n**** MIPI CSI-2 Example ****\n");
    printf("This example streams the image data through the COM port\n");
    printf("and a script running on the host pc converts the data into\n");
    printf("a .png image.\n");
    printf("\nGo into the pc_utility folder and run the script:\n");
    printf("python console.py [COM#]\n");
    printf("\nPress PB1 (SW4) or send the 'capture' command to trigger a frame capture.\n\n");

    printf("Initializing camera...\n");

    MXC_TMR_SW_Start(MXC_TMR0);
    TFT_SPI_Init();
    int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("TFT SPI Initialization took %i us\n", elapsed);

    // Initialize camera
    mipi_camera_settings_t settings = {
        .width = IMAGE_WIDTH,
        .height = IMAGE_HEIGHT,
        .camera_format = {
            .pixel_format = PIXEL_FORMAT,
            .pixel_order = PIXEL_ORDER
        },
        .line_handler = CSI2_line_handler,
    };

    mipi_camera_init(settings);

    // Confirm correct camera is connected
    mipi_camera_get_product_id(&id);
    printf("Camera ID = %x\n", id);
    if (id != CAMERA_ID) {
        printf("Incorrect camera.\n");
        LED_On(1);
        while (1) {}
    }

    printf("Initializing SRAM...\n");
    error = ram_init();
    if (error) {
        printf("Failed to initialize SRAM with error %i\n", error);
        return error;
    }

    MXC_TMR_SW_Start(MXC_TMR0);
    spi_init();
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("RAM SPI Initialization took %i us\n", elapsed);

    ram_id_t ram_id;
    error = ram_read_id(&ram_id);
    if (error) {
        printf("Failed to read expected SRAM ID!\n");
        return error;
    }
    printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", ram_id.MFID, ram_id.KGD, ram_id.density, ram_id.EID);

    MXC_TFT_SetRotation(ROTATE_270);

    PB_RegisterCallback(0, (pb_callback)buttonHandler);
    buttonPressed = 0;

    while (1) {
        LED_On(0);

        MXC_Delay(MXC_DELAY_MSEC(100));
        // ^ Slow down main processing loop to work around some timing issues
        // with the console at extreme speeds.  1000 checks/sec is plenty
        // service_console();

        if (buttonPressed) {
            process_img();
            LED_Off(0);

            buttonPressed = 0;
        }
    }
}
