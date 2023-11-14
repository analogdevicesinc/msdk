/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
 * @brief   SPI_ControllerTarget Demo
 * @details Shows Controller and Target transactions between two SPI instances.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "led.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "pb.h"
#include "spi.h"

/***** Preprocessors *****/

#define TARGET_DMA 0

// Target Select Control Scheme
#define TSCONTROL_HW_AUTO 1 // Hardware asserts/deasserts TSn pins.
#define TSCONTROL_SW_APP 0 // Application asserts/deasserts custom TS pins.

// Preprocessor Error Checking
#if (!(TSCONTROL_HW_AUTO || TSCONTROL_SW_APP))
#error "You must set either TSCONTROL_HW_AUTO or TSCONTROL_SW_APP."
#endif
#if (TSCONTROL_HW_AUTO && TSCONTROL_SW_APP)
#error "You must select either TSCONTROL_HW_AUTO or TSCONTROL_SW_APP."
#endif

/***** Definitions *****/

#define DATA_LEN 1024 // Words
#define DATA_SIZE 8
#define VALUE 0xFF
#define SPI_SPEED 100000 // Bit Rate (Max.: 1,850,000)

#define SPI_CONTROLLER MXC_SPI1
#define SPI_CONTROLLER_TSIDX 0
#define SPI_TARGET MXC_SPI0
#define SPI_TARGET_TSIDX 0
#define SPI_TARGET_IRQ SPI0_IRQn

/***** Globals *****/
uint8_t controller_rx[DATA_LEN];
uint8_t controller_tx[DATA_LEN];
uint8_t target_rx[DATA_LEN];
uint8_t target_tx[DATA_LEN];
uint8_t TX_DMA_CH, RX_DMA_CH;

/***** Functions *****/
void SPI_Target_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI_TARGET);
}

#if TARGET_DMA
void DMA_TX_IRQHandler(void)
{
    MXC_SPI_DMA_TX_Handler(SPI_TARGET);
}

void DMA_RX_IRQHandler(void)
{
    MXC_SPI_DMA_RX_Handler(SPI_TARGET);
}
#endif

int main(void)
{
    int error;
    uint8_t ts_active_pol_mask = 0;

    // Controller (L. Master)
    mxc_spi_pins_t controller_pins;
    mxc_spi_cfg_t controller_cfg;
    mxc_spi_req_t controller_req;

    // Target (L. Slave)
    mxc_spi_pins_t target_pins;
    mxc_spi_cfg_t target_cfg;
    mxc_spi_req_t target_req;

    printf("\n************************ SPI Controller-Target Example ************************\n");
    printf("This example sends data between two SPI peripherals in the MAX78002.\n");
    printf("SPI%d is configured as the ts (L. Slave) and SPI%d is configured\n",
           MXC_SPI_GET_IDX(SPI_TARGET), MXC_SPI_GET_IDX(SPI_CONTROLLER));
    printf("as the controller (L. Master). Each SPI peripheral sends %d bytes\n", DATA_LEN);
    printf("on the SPI bus. If the data received by each SPI instance matches the\n");
    printf("the data sent by the other instance, then the green LED will illuminate,\n");
    printf("otherwise the red LED will illuminate.\n\n");

#if TSCONTROL_SW_APP
    printf("A custom Target Select pin was set up for the Controller (SPI%d).\n",
           MXC_SPI_GET_IDX(SPI_CONTROLLER));
    printf("Please connect the custom TS pins: P0.4 to P0.12.\n\n");
#else // TSCONTROL_HW_AUTO
    printf("Please connect the HW TS pins: P0.4 to P0.20.\n\n");
#endif

    /***** Initialize data buffers *****/
    for (int i = 0; i < DATA_LEN; i++) {
        controller_tx[i] = i;
        target_tx[i] = i;
    }

    memset(controller_rx, 0x0, DATA_LEN * sizeof(uint8_t));
    memset(target_rx, 0x0, DATA_LEN * sizeof(uint8_t));

    /***** Configure Controller (L. Master) *****/

    controller_cfg.spi = SPI_CONTROLLER;
    controller_cfg.clk_mode = MXC_SPI_CLKMODE_0; // CPOL: 0, CPHA: 0
    controller_cfg.frame_size = DATA_SIZE;
    controller_cfg.use_dma_tx = false;
    controller_cfg.use_dma_rx = false;

    // Target Select Settings
#if TSCONTROL_HW_AUTO
    // Initialize HW TS0 pin.
    controller_pins.ss0 = true;
    controller_pins.ss1 = false;
    controller_pins.ss2 = false;
    controller_pins.vddioh = false;
    controller_pins.drvstr = MXC_GPIO_DRVSTR_0;

    // This demonstrates how to set the Active Polarity for each TSn pin.
    // This setting is passed into MXC_SPI_Init(...) and should match between
    // the Controller and Target.
    //  ts_active_pol_mask[0] = 1 -> Active HIGH (1)
    //  ts_active_pol_mask[1] = 0 -> Active LOW (0)
    //  ts_active_pol_mask[2] = 0 -> Active LOW (0)
    ts_active_pol_mask = 0b0001;

#else // TSCONTROL_SW_DRV or TSCONTROL_SW_APP
    // Example to set up a custom TS pin with Active HIGH polarity.
    mxc_gpio_cfg_t ts_pins;
    ts_pins.port = MXC_GPIO0;
    ts_pins.mask = MXC_GPIO_PIN_12;
    ts_pins.func = MXC_GPIO_FUNC_OUT;
    ts_pins.pad = MXC_GPIO_PAD_NONE;
    ts_pins.vssel = MXC_GPIO_VSSEL_VDDIO; // Set custom ts pin to VDDIO.

    // Don't initialize HW TS pins in this scheme.
    controller_pins.ss0 = false;
    controller_pins.ss1 = false;
    controller_pins.ss2 = false;
    controller_pins.vddioh = false;
    controller_pins.drvstr = MXC_GPIO_DRVSTR_0;

    // No HW TS pins are used in this scheme.
    ts_active_pol_mask = 0;

    // Configure the custom TS pin.
    MXC_GPIO_Config(&ts_pins);
    // Active HIGH, Idle LOW.
    MXC_GPIO_OutClr(ts_pins.port, ts_pins.mask);
#endif

    error = MXC_SPI_Init(SPI_CONTROLLER, MXC_SPI_TYPE_CONTROLLER, MXC_SPI_INTERFACE_STANDARD, 0,
                         ts_active_pol_mask, SPI_SPEED, controller_pins);
    if (error != E_NO_ERROR) {
        printf("\nSPI CONTROLLER INITIALIZATION ERROR\n");
        while (1) {}
    }

    error = MXC_SPI_Config(&controller_cfg);
    if (error != E_NO_ERROR) {
        printf("\nSPI CONTROLLER CONFIGURATION ERROR\n");
        while (1) {}
    }

    // Setup Controller Request.
    controller_req.spi = SPI_CONTROLLER;
    controller_req.txData = (uint8_t *)controller_tx;
    controller_req.txLen = DATA_LEN;
    controller_req.rxData = (uint8_t *)controller_rx;
    controller_req.rxLen = DATA_LEN;
    controller_req.ssDeassert = 1;
    controller_req.ssIdx = SPI_CONTROLLER_TSIDX;
    controller_req.completeCB = NULL;

    /***** Configure Target (L. Slave) *****/
    // Initialize HW TS0 pin.
    target_pins.ss0 = true;
    target_pins.ss1 = false;
    target_pins.ss2 = false;
    target_pins.vddioh = false;
    target_pins.drvstr = MXC_GPIO_DRVSTR_0;

    // This demonstrates how to set the Active Polarity for each TSn pin.
    // This setting is passed into MXC_SPI_Init(...) and should match between
    // the Controller and Target.
    //  ts_active_pol_mask[0] = 1 -> Active HIGH (1)
    //  ts_active_pol_mask[1] = 0 -> Active LOW (0)
    //  ts_active_pol_mask[2] = 0 -> Active LOW (0)
    ts_active_pol_mask = 0b0001;

    target_cfg.spi = SPI_TARGET;
    target_cfg.clk_mode = MXC_SPI_CLKMODE_0; // CPOL: 0, CPHA: 0
    target_cfg.frame_size = DATA_SIZE;

#if TARGET_DMA
    target_cfg.use_dma_tx = true;
    target_cfg.use_dma_rx = true;
    target_cfg.dma = MXC_DMA;
#else
    target_cfg.use_dma_tx = false;
    target_cfg.use_dma_rx = false;
#endif

    error = MXC_SPI_Init(SPI_TARGET, MXC_SPI_TYPE_TARGET, MXC_SPI_INTERFACE_STANDARD, 0,
                         ts_active_pol_mask, SPI_SPEED, target_pins);
    if (error != E_NO_ERROR) {
        printf("\nSPI TARGET INITIALIZATION ERROR\n");
        while (1) {}
    }

    error = MXC_SPI_Config(&target_cfg);
    if (error != E_NO_ERROR) {
        printf("\nSPI TARGET CONFIGURATION ERROR\n");
        while (1) {}
    }

    // Setup Target Request.
    target_req.spi = SPI_TARGET;
    target_req.txData = (uint8_t *)target_tx;
    target_req.txLen = DATA_LEN;
    target_req.rxData = (uint8_t *)target_rx;
    target_req.rxLen = DATA_LEN;
    target_req.ssDeassert = 1;
    target_req.completeCB = NULL;

#if TARGET_DMA
    TX_DMA_CH = MXC_SPI_DMA_GetTXChannel(SPI_TARGET);
    RX_DMA_CH = MXC_SPI_DMA_GetRXChannel(SPI_TARGET);

    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(TX_DMA_CH));
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(RX_DMA_CH));

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(TX_DMA_CH), DMA_TX_IRQHandler);
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(RX_DMA_CH), DMA_RX_IRQHandler);
#else
    MXC_NVIC_SetVector(SPI_TARGET_IRQ, SPI_Target_IRQHandler);
    NVIC_EnableIRQ(SPI_TARGET_IRQ);
#endif

    printf("Press PB1 to begin transaction.\n");
    while (!PB_Get(0)) {}

    /***** Perform Transaction *****/
#if TARGET_DMA
    MXC_SPI_TargetTransactionDMA(&target_req);
#else
    MXC_SPI_TargetTransactionAsync(&target_req);
#endif

#if TSCONTROL_SW_APP
    // Assert custom TS pin.
    MXC_GPIO_OutToggle(ts_pins.port, ts_pins.mask);
#endif

    MXC_SPI_ControllerTransaction(&controller_req);

#if TSCONTROL_SW_APP
    MXC_GPIO_OutToggle(ts_pins.port, ts_pins.mask);
#endif

    /***** Verify Results *****/
    if (memcmp(target_rx, controller_tx, sizeof(controller_tx)) != 0) { // Controller->Target
        printf("\nTarget failed to receive data.\n");
        LED_On(1);
        return E_COMM_ERR;
    } else if (memcmp(controller_rx, target_tx, sizeof(target_tx)) != 0) { // Target->Controller
        printf("\nController failed to receive data.\n");
        LED_On(1);
        return E_COMM_ERR;
    }

    MXC_SPI_Shutdown(SPI_CONTROLLER);
    MXC_SPI_Shutdown(SPI_TARGET);

    LED_On(0); // Indicates SUCCESS
    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
