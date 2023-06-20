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

/**
 * @file    main.c
 * @brief   MAX78000 Feather Facial Recognition Demo
 *
 * @details
 *
 */

#define S_MODULE_NAME "main"

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "mxc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "camera.h"
#include "icc.h"
#include "rtc.h"
#include "cnn_1.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#include "MAXCAM_Debug.h"
#include "facedetection.h"
#include "post_process.h"
#include "faceID.h"
#include "embedding_process.h"

#define CONSOLE_BAUD 115200

extern void SD_Init(void);
extern volatile uint8_t face_detected;

mxc_uart_regs_t *CommUart;
#ifdef TFT_ENABLE
area_t area = { 50, 290, 180, 30 };
#endif
// *****************************************************************************
int main(void)
{
    int ret = 0;
    int slaveAddress;
    int id;
    int dma_channel;
    int undetect_count = 0;
    mxc_uart_regs_t *ConsoleUart;

#ifdef BOARD_FTHR_REVA
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    /* Enable camera power */
    Camera_Power(POWER_ON);
#endif
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);

    if ((ret = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        PR_ERR("UART1 Init Error: %d\n", ret);
        return ret;
    }

    PR_DEBUG("\n\nMAX78000 Feather Facial Recognition Demo\n");

    // Initialize FaceID embeddings database
    if (init_database() < 0) {
        PR_ERR("Could not initialize the database");
        return -1;
    }

    /* Initialize RTC */
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);

    // Obtain the I2C slave address of the camera.
    slaveAddress = camera_get_slave_address();
    PR_DEBUG("Camera I2C slave address is %02x\n", slaveAddress);

    // Obtain the product ID of the camera.
    ret = camera_get_product_id(&id);

    if (ret != STATUS_OK) {
        PR_ERR("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    PR_DEBUG("Camera Product ID is %04x\n", id);

    // Obtain the manufacture ID of the camera.
    ret = camera_get_manufacture_id(&id);

    if (ret != STATUS_OK) {
        PR_ERR("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    PR_DEBUG("Camera Manufacture ID is %04x\n", id);

    // Setup the camera image dimensions, pixel format and data acquiring details.
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, USE_DMA,
                       dma_channel);

    if (ret != STATUS_OK) {
        PR_ERR("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

#ifdef ROTATE_FEATHER_BOARD
    camera_set_hmirror(0);
#else
    camera_set_vflip(0);
#endif

#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
#ifdef ROTATE_FEATHER_BOARD
    MXC_TFT_SetRotation(ROTATE_0);
#else
    MXC_TFT_SetRotation(ROTATE_180);
#endif
    MXC_TFT_SetBackGroundColor(4);
    MXC_TFT_SetForeGroundColor(WHITE); // set font color to white
#endif
#endif

    /* Initilize SD card */
    SD_Init();

    while (1) {
        face_detection();

        if (face_detected) {
            face_id();
            face_detected = 0;
            undetect_count = 0;
        } else
            undetect_count++;

#ifdef TFT_ENABLE
        if (undetect_count > 5) {
            MXC_TFT_ClearArea(&area, 4);
            undetect_count = 0;
        }
#endif
    }

    return 0;
}
