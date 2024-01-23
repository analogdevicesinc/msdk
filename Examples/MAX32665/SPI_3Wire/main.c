/**
 * @file    main.c
 * @brief   3 Wire SPI Demo
 * @details Shows data communication btw SPI peripherals operating in 3-wire SPI mode
 *          Read the printf() for instructions
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "dma.h"
#include "led.h"

/***** Definitions *****/

#define DATA_LEN 250 // Words
#define DATA_SIZE 8
#define SPI_SPEED 100000 // Bit Rate (Max.: 1,850,000)

#define SPI_MASTER MXC_SPI2
#define SPI_MASTER_SSIDX 0
#define SPI_SLAVE MXC_SPI1
#define SPI_SLAVE_SSIDX 0
#define SPI_SLAVE_IRQ SPI1_IRQn

/***** Globals *****/
uint8_t master_rx[DATA_LEN];
uint8_t master_tx[DATA_LEN];
uint8_t slave_rx[DATA_LEN];
uint8_t slave_tx[DATA_LEN];

const mxc_gpio_cfg_t gpio_p0_7 = { MXC_GPIO0,         (MXC_GPIO_PIN_7),     MXC_GPIO_FUNC_OUT,
                                   MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_1 };

/***** Functions *****/
void SPI_Slave_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI_SLAVE);
}

int main(void)
{
    int ret = E_NO_ERROR;
    mxc_spi_req_t slave_req;
    mxc_spi_req_t master_req;

    printf("\n******************* 3 Wire SPI Example for MAX32665/MAX32666 *******************\n");
    printf("This example sends data between two SPI peripherals operating in 3-wire SPI mode.\n");
    printf("SPI%d is configured as master and SPI%d is configured as slave.\n\n",
           MXC_SPI_GET_IDX(SPI_MASTER), MXC_SPI_GET_IDX(SPI_SLAVE));
    printf("Connections:\n");
    printf("SPI Clock Connection: Connect P0.27 - P0.19\n");
    printf("SPI SS Connection   : Connect P0.7(GPIO) - P0.16\n");
    printf("SPI Data Connection : Connect P0.25 - P0.17\n\n");

    /***** Configure master *****/
    ret = MXC_GPIO_Config(&gpio_p0_7);
    if (ret != E_NO_ERROR) {
        printf("\nGPIO INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    MXC_GPIO_OutSet(gpio_p0_7.port, gpio_p0_7.mask);

    ret = MXC_SPI_Init(SPI_MASTER, 1, 0, 1, 0, SPI_SPEED, MAP_A);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetDataSize(SPI_MASTER, DATA_SIZE);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetWidth(SPI_MASTER, SPI_WIDTH_3WIRE);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetMode(SPI_MASTER, SPI_MODE_0);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    /***** Configure slave *****/
    ret = MXC_GPIO_Config(&gpio_cfg_spi1_ss0);

    if (ret != E_NO_ERROR) {
        printf("\nGPIO INITIALIZATION ERROR\n");
        return E_FAIL;
    }
    ret = MXC_SPI_Init(SPI_SLAVE, 0, 0, 1, 0, SPI_SPEED, MAP_A);
    if (ret != E_NO_ERROR) {
        printf("\nSPI SLAVE INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetDataSize(SPI_SLAVE, DATA_SIZE);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetWidth(SPI_SLAVE, SPI_WIDTH_3WIRE);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    ret = MXC_SPI_SetMode(SPI_SLAVE, SPI_MODE_0);
    if (ret != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        return E_FAIL;
    }

    MXC_NVIC_SetVector(SPI_SLAVE_IRQ, SPI_Slave_IRQHandler);
    NVIC_EnableIRQ(SPI_SLAVE_IRQ);

    /***** Initialize Transaction Parameters *****/
    master_req.spi = SPI_MASTER;
    master_req.txData = (uint8_t *)master_tx;
    master_req.rxData = (uint8_t *)master_rx;
    master_req.txLen = 0;
    master_req.rxLen = 0;
    master_req.ssIdx = SPI_MASTER_SSIDX;
    master_req.ssDeassert = 1;
    master_req.txCnt = 0;
    master_req.rxCnt = 0;
    master_req.completeCB = NULL;

    slave_req.spi = SPI_SLAVE;
    slave_req.txData = (uint8_t *)slave_tx;
    slave_req.rxData = (uint8_t *)slave_rx;
    slave_req.txLen = 0;
    slave_req.rxLen = 0;
    slave_req.ssIdx = SPI_SLAVE_SSIDX;
    slave_req.ssDeassert = 1;
    slave_req.txCnt = 0;
    slave_req.rxCnt = 0;
    slave_req.completeCB = NULL;

    while (1) {
        for (int i = 0; i < DATA_LEN; i++) {
            master_tx[i] = i;
            slave_tx[i] = i;
        }
        memset(master_rx, 0x0, DATA_LEN * sizeof(uint8_t));
        memset(slave_rx, 0x0, DATA_LEN * sizeof(uint8_t));

        // Master sends - Slave receives
        master_req.txLen = DATA_LEN;
        master_req.rxLen = 0;
        slave_req.txLen = 0;
        slave_req.rxLen = DATA_LEN;

        /***** Perform Transaction *****/
        MXC_GPIO_OutClr(gpio_p0_7.port, gpio_p0_7.mask);

        MXC_SPI_SlaveTransactionAsync(&slave_req);
        MXC_SPI_MasterTransaction(&master_req);

        MXC_GPIO_OutSet(gpio_p0_7.port, gpio_p0_7.mask);

        /***** Verify Results *****/
        if (memcmp(slave_rx, master_tx, DATA_LEN) != 0) { // Master->Slave
            printf("Slave failed to receive data.\n");
            return E_FAIL;
        } else {
            printf("Master send / slave receive operation successful\n");
        }

        // Slave sends - Master receives
        master_req.txLen = 0;
        master_req.rxLen = DATA_LEN;
        slave_req.txLen = DATA_LEN;
        slave_req.rxLen = 0;

        /***** Perform Transaction *****/
        MXC_GPIO_OutClr(gpio_p0_7.port, gpio_p0_7.mask);

        MXC_SPI_SlaveTransactionAsync(&slave_req);
        MXC_SPI_MasterTransaction(&master_req);

        MXC_GPIO_OutSet(gpio_p0_7.port, gpio_p0_7.mask);

        /***** Verify Results *****/
        if (memcmp(slave_tx, master_rx, DATA_LEN) != 0) { // Master->Slave
            printf("Master failed to receive data\n\n.");
            return E_FAIL;
        } else {
            printf("Master receive / slave send operation successful\n\n");
        }
        MXC_Delay(MXC_DELAY_SEC(1));
    }
    return E_NO_ERROR;
}
