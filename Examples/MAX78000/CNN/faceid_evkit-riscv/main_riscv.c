/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 * Analog Devices, Inc.),
 * 2023 Analog Devices, Inc. All Rights Reserved.
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
 * @file    main_riscv.c
 * @brief   FaceID EvKit Demo
 *
 * @details
 *
 */

#define S_MODULE_NAME "MAIN-RISCV"

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "board.h"
#include "fcr_regs.h"
#include "camera.h"
#include "faceID.h"
#include "icc.h"
#include "dma.h"
#include "cnn.h"
#include "MAXCAM_Debug.h"
#include "weights.h"
#include "embedding_process.h"
#include "sema_regs.h"

__attribute__((section(
    ".shared__at__mailbox"))) volatile uint32_t mail_box[ARM_MAILBOX_SIZE + RISCV_MAILBOX_SIZE];
volatile uint32_t *arm_mail_box = &mail_box[0];
volatile uint32_t *riscv_mail_box = &mail_box[ARM_MAILBOX_SIZE];

extern int start_faceid(void);

// *****************************************************************************
void __attribute__((interrupt("machine"))) WUT_IRQHandler(void)
{
    MXC_WUT_IntClear();
    NVIC_ClearPendingIRQ(WUT_IRQn);
    NVIC_ClearPendingEVENT(WUT_IRQn);
}

int main(void)
{
    int ret = 0;
    int slaveAddress;
    int id;
    int dma_channel;

    Debug_Init(); // Set up RISCV JTAG
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC1);

    PR_DEBUG("Start RISC-V\n");
    /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
    /* CNN clock: 50 MHz div 1 */
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    /* Configure P2.5, turn on the CNN Boost */
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);
    /* Bring state machine into consistent state */
    cnn_init();
    /* Load kernels */
    cnn_load_weights();
    /* Configure state machine */
    cnn_configure();
    PR_DEBUG("Init CNN");
    /* Enable CNN interrupt */
    NVIC_EnableIRQ(CNN_IRQn);
    /* Enable CNN wakeup event */
    NVIC_EnableEVENT(CNN_IRQn);

    if (init_database() < 0) {
        PR_ERR("Could not initialize the database");
        return -1;
    }

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

#ifdef BOARD_FTHR_REVA
    /* Enable camera power */
    Camera_Power(POWER_ON);
    MXC_Delay(300000);
    PR_DEBUG("FaceID Feather Demo\n");
#else
    PR_DEBUG("FaceID Evkit Demo\n");
#endif

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);

    PR_DEBUG("Init Camera");

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

#ifdef BOARD_FTHR_REVA
    camera_write_reg(0x0c, 0x56); //camera vertical flip=0
#endif

    /* Enable PCIF wakeup event */
    NVIC_EnableEVENT(PCIF_IRQn);
    /* Enable wakeup timer interrupt */
    NVIC_EnableIRQ(WUT_IRQn);
    /* Enable wakeup timer event */
    NVIC_EnableEVENT(WUT_IRQn);

    PR_DEBUG("ARM mailbox: %x", &arm_mail_box[0]);
    PR_DEBUG("RISC-V mailbox: %x\n", &riscv_mail_box[0]);

    while (1) {
        if (arm_mail_box[0]) {
            PR_DEBUG("Start FaceId");
            /* clear ARM mailbox */
            arm_mail_box[0] = 0;
            start_faceid();
            asm volatile("wfi"); // RISC-V sleeps and waits for command from ARM
        }
    }

    return 0;
}
