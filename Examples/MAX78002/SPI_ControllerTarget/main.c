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

/***** Definitions *****/

#define DMA 0

#define CUSTOM_TARGET 1

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

#if DMA
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
    mxc_spi_init_t controller_init; // L. Master
    mxc_spi_init_t target_init; // L. Slave
    mxc_spi_target_t target;

    printf("\n************************ SPI Controller-Target Example ************************\n");
    printf("This example sends data between two SPI peripherals in the MAX78002.\n");
    printf("SPI%d is configured as the target (L. Slave) and SPI%d is configured\n",
           MXC_SPI_GET_IDX(SPI_TARGET), MXC_SPI_GET_IDX(SPI_CONTROLLER));
    printf("as the controller (L. Master). Each SPI peripheral sends %d bytes\n", DATA_LEN);
    printf("on the SPI bus. If the data received by each SPI instance matches the\n");
    printf("the data sent by the other instance, then the green LED will illuminate,\n");
    printf("otherwise the red LED will illuminate.\n\n");

#if CUSTOM_TARGET
    printf("A custom Target Select pin for the Controller (SPI%d) was selected.\n", MXC_SPI_GET_IDX(SPI_CONTROLLER));
    printf("Please connect the custom TS pin P0.9 to P0.20.\n\n");
#endif

    printf("Press PB1 to begin transaction.\n");
    while (!PB_Get(0)) {}

    /***** Initialize data buffers *****/
    for (int i = 0; i < DATA_LEN; i++) {
        controller_tx[i] = i;
        target_tx[i] = i;
    }

    memset(controller_rx, 0x0, DATA_LEN * sizeof(uint8_t));
    memset(target_rx, 0x0, DATA_LEN * sizeof(uint8_t));

    /***** Configure Controller (L. Master) *****/
    controller_init.spi = SPI_CONTROLLER;
    controller_init.freq = SPI_SPEED;
    controller_init.spi_pins = NULL; // Use default, predefined pins
    controller_init.mode = MXC_SPI_INTERFACE_STANDARD;
    controller_init.type = MXC_SPI_TYPE_CONTROLLER; // L. Master
    controller_init.clk_mode = MXC_SPI_CLKMODE_0; // CPOL: 0, CPHA: 0
    controller_init.frame_size = DATA_SIZE;
    controller_init.callback = NULL;
    controller_init.use_dma = false;

    // Target Select Settings
#if CUSTOM_TARGET
    // Example to select a custom target.
    mxc_gpio_cfg_t target_pins;
    target_pins.port = MXC_GPIO0;
    target_pins.mask = MXC_GPIO_PIN_12;
    target_pins.func = MXC_GPIO_FUNC_OUT;
    target_pins.pad = MXC_GPIO_PAD_PULL_UP;
    target_pins.vssel = MXC_GPIO_VSSEL_VDDIO; // Set custom target pin to VDDIOH (3.3V).
    target_pins.ds = MXC_GPIO_DS_3; // Set custom target pin to VDDIOH (3.3V).

    controller_init.ts_control =
        MXC_SPI_TSCONTROL_SW_DRV; // SPI Driver will handle deassertion for TS pins.
    controller_init.target.pins = target_pins;
    controller_init.target.active_polarity = 0;
    controller_init.vssel = MXC_GPIO_VSSEL_VDDIO; // Set SPI pins to VDDIOH (3.3V).

    // Select target for transaction.
    target.pins = target_pins; // Custom pins
#else
    controller_init.ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // HW will deassert/assert TS pins.
    controller_init.target.active_polarity = 0;
    controller_init.target.init_mask = 1 << SPI_CONTROLLER_TSIDX; // Initialize Target Select 0 pin.
    controller_init.vssel = MXC_GPIO_VSSEL_VDDIO;

    // Select target for transaction.
    target.index = SPI_CONTROLLER_TSIDX;
#endif

    error = MXC_SPI_Init_v2(&controller_init);
    if (error != E_NO_ERROR) {
        printf("\nSPI CONTROLLER INITIALIZATION ERROR\n");
        while (1) {}
    }

    /***** Configure Target (L. Slave) *****/
    target_init.spi = SPI_TARGET;
    target_init.freq = SPI_SPEED;
    target_init.spi_pins = NULL; // Use default, predefined pins
    target_init.mode = MXC_SPI_INTERFACE_STANDARD;
    target_init.type = MXC_SPI_TYPE_TARGET; // L. Slave
    target_init.clk_mode = MXC_SPI_CLKMODE_0; // CPOL: 0, CPHA: 0
    target_init.frame_size = DATA_SIZE;
    target_init.callback = NULL;
    target_init.ts_control =
        MXC_SPI_TSCONTROL_HW_AUTO; // Target transactions only supports HW_AUTO mode
    target_init.target.active_polarity = 0;
    target_init.target.init_mask = 1 << SPI_TARGET_TSIDX; // Initialize Target Select 0 pin.
    target_init.vssel = MXC_GPIO_VSSEL_VDDIO;

#if DMA
    target_init.use_dma = true;
    target_init.dma = MXC_DMA;
#else
    target_init.use_dma = false;
#endif

    error = MXC_SPI_Init_v2(&target_init);
    if (error != E_NO_ERROR) {
        printf("\nSPI TARGET INITIALIZATION ERROR\n");
        while (1) {}
    }

#if DMA
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

    /***** Perform Transaction *****/
#if DMA
    MXC_SPI_TargetTransactionDMA(SPI_TARGET, target_tx, DATA_LEN, target_rx, DATA_LEN, 1);
#else
    MXC_SPI_TargetTransaction(SPI_TARGET, target_tx, DATA_LEN, target_rx, DATA_LEN, 1);
#endif

    MXC_SPI_ControllerTransactionB(SPI_CONTROLLER, controller_tx, DATA_LEN, controller_rx, DATA_LEN,
                                   1, &target);

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
