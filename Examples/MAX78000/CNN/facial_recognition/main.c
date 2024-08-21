/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include "embeddings.h"
#include "faceID.h"
#include "utils.h"
#define CONSOLE_BAUD 115200

extern void SD_Init(void);
extern volatile uint8_t face_detected;
volatile char names[1024][7];
mxc_uart_regs_t *CommUart;

void init_names()
{
    char default_names[DEFAULT_EMBS_NUM][7] = DEFAULT_NAMES;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    for (int i = 0; i < DEFAULT_EMBS_NUM; i++) {
        strncpy((char *)names[i], default_names[i], 7);
    }
#pragma GCC diagnostic pop
}
#ifdef TFT_ENABLE
area_t area = { 50, 290, 180, 30 };
//area_t area = { 290, 50, 30, 180 };
#endif
// *****************************************************************************
int main(void)
{
    int error = 0;
    int slaveAddress;
    int id;
    int dma_channel;
    mxc_uart_regs_t *ConsoleUart;

    MXC_Delay(MXC_DELAY_SEC(2)); // Provide window for debugger to connect

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

    if ((error = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        PR_ERR("UART1 Init Error: %d\n", error);
        return error;
    }

    PR_INFO("\n\nMAX78000 Feather Facial Recognition Demo");

    PR_INFO("Initializing...\n");
    init_names();
    /* Initialize RTC */
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    error = camera_init(CAMERA_FREQ);
    if (error) {
        PR_ERR("Camera initialization error (%i)", error);
        return error;
    }

    // Obtain the I2C slave address of the camera.
    slaveAddress = camera_get_slave_address();
    PR_DEBUG("Camera I2C slave address is %02x", slaveAddress);

    // Obtain the product ID of the camera.
    error = camera_get_product_id(&id);

    if (error) {
        PR_ERR("Error returned from reading camera id. Error %d", error);
        return error;
    }

    PR_DEBUG("Camera Product ID is %04x", id);

    // Obtain the manufacture ID of the camera.
    error = camera_get_manufacture_id(&id);

    if (error) {
        PR_ERR("Error returned from reading camera id. Error %d", error);
        return error;
    }

    PR_DEBUG("Camera Manufacture ID is %04x", id);

    // Setup the camera image dimensions, pixel format and data acquiring details.
    error = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB565, FIFO_FOUR_BYTE, USE_DMA,
                         dma_channel);

    if (error) {
        PR_ERR("Error returned from setting up camera. Error %d", error);
        return error;
    }

    // Double PCLK
    camera_write_reg(0x11, 0x80);

#ifdef ROTATE_FEATHER_BOARD
    //camera_set_hmirror(0);
#else
    camera_set_vflip(0);
#endif

#ifdef TFT_ENABLE
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
#ifdef ROTATE_FEATHER_BOARD
    MXC_TFT_SetRotation(ROTATE_270);
#else
    MXC_TFT_SetRotation(ROTATE_180);
#endif
    MXC_TFT_SetBackGroundColor(4);
    MXC_TFT_SetForeGroundColor(WHITE); // set font color to white
#endif
#endif

    /* Initilize SD card */
    PR_INFO("Initializing SD Card...");
    SD_Init();

    PR_INFO("Launching face detection loop...");
    uint32_t loop_time;
    while (1) {
        PR_INFO("-----");
        loop_time = utils_get_time_ms();
        LED_On(0);
        face_detection();
        LED_Off(0);

        if (face_detected) {
            PR_INFO("Face detected!");
            LED_On(1);
            face_id();
            face_detected = 0;
        } else {
            PR_INFO("No face detected.");
            LED_Off(1);
        }

        loop_time = utils_get_time_ms() - loop_time;
        PR_INFO("----- (Total loop time: %dms)\n", loop_time);

#ifdef TFT_ENABLE
        MXC_TFT_SetRotation(ROTATE_180);
        MXC_TFT_ClearArea(&area, 4);
        MXC_TFT_SetRotation(ROTATE_270);
#endif
    }

    return 0;
}
