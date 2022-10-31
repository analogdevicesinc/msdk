/**
 * @file    main.c
 * @brief   SPI_MasterSlave Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
 */

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
#define DATA_LEN 1024 // Words
#define DATA_SIZE 8
#define VALUE 0xFF
#define SPI_SPEED 100000 // Bit Rate (Max.: 1,850,000)

#define SPI_MASTER MXC_SPI1
#define SPI_MASTER_SSIDX 1
#define SPI_SLAVE MXC_SPI0
#define SPI_SLAVE_SSIDX 0
#define SPI_SLAVE_IRQ SPI0_IRQn

/***** Globals *****/
uint8_t master_rx[DATA_LEN];
uint8_t master_tx[DATA_LEN];
uint8_t slave_rx[DATA_LEN];
uint8_t slave_tx[DATA_LEN];

/***** Functions *****/
void SPI_Slave_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI_SLAVE);
}

int main(void)
{
    mxc_spi_req_t slave_req;
    mxc_spi_req_t master_req;

    printf("\n************************ SPI Master-Slave Example ************************\n");
    printf("This example sends data between two SPI peripherals in the MAX32520.\n");
    printf("SPI%d is configured as the slave and SPI%d is configured as the master.\n",
           MXC_SPI_GET_IDX(SPI_SLAVE), MXC_SPI_GET_IDX(SPI_MASTER));
    printf("Each SPI peripheral sends 1024 bytes on the SPI bus. If the data received\n", DATA_LEN);
    printf("by each SPI instance matches the data sent by the other instance, the\n");
    printf("green LED will illuminate, otherwise the red LED will illuminate.\n\n");

    printf("Press SW2 to begin transaction.\n\n");
    while (!PB_Get(0)) {}

    /***** Initialize data buffers *****/
    for (int i = 0; i < DATA_LEN; i++) {
        master_tx[i] = i;
        slave_tx[i] = i;
    }
    memset(master_rx, 0x0, DATA_LEN * sizeof(uint8_t));
    memset(slave_rx, 0x0, DATA_LEN * sizeof(uint8_t));

    /***** Configure master *****/
    MXC_GPIO_Config(&gpio_cfg_spi1_ss1);
    if (MXC_SPI_Init(SPI_MASTER, 1, 0, 1, (1 << SPI_MASTER_SSIDX), SPI_SPEED) != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        while (1) {}
    }

    MXC_SPI_SetDataSize(SPI_MASTER, DATA_SIZE);
    MXC_SPI_SetWidth(SPI_MASTER, SPI_WIDTH_STANDARD);

    /***** Configure slave *****/
    MXC_GPIO_Config(&gpio_cfg_spi0_ss0);
    if (MXC_SPI_Init(SPI_SLAVE, 0, 0, 1, (1 << SPI_SLAVE_SSIDX), SPI_SPEED) != E_NO_ERROR) {
        printf("\nSPI SLAVE INITIALIZATION ERROR\n");
        while (1) {}
    }

    MXC_SPI_SetDataSize(SPI_SLAVE, DATA_SIZE);
    MXC_SPI_SetWidth(SPI_SLAVE, SPI_WIDTH_STANDARD);

    MXC_NVIC_SetVector(SPI_SLAVE_IRQ, SPI_Slave_IRQHandler);
    NVIC_EnableIRQ(SPI_SLAVE_IRQ);

    /***** Initialize Transaction Parameters *****/
    master_req.spi = SPI_MASTER;
    master_req.txData = (uint8_t *)master_tx;
    master_req.rxData = (uint8_t *)master_rx;
    master_req.txLen = DATA_LEN;
    master_req.rxLen = DATA_LEN;
    master_req.ssIdx = SPI_MASTER_SSIDX;
    master_req.ssDeassert = 1;
    master_req.txCnt = 0;
    master_req.rxCnt = 0;
    master_req.completeCB = NULL;

    slave_req.spi = SPI_SLAVE;
    slave_req.txData = (uint8_t *)slave_tx;
    slave_req.rxData = (uint8_t *)slave_rx;
    slave_req.txLen = DATA_LEN;
    slave_req.rxLen = DATA_LEN;
    slave_req.ssIdx = SPI_SLAVE_SSIDX;
    slave_req.ssDeassert = 1;
    slave_req.txCnt = 0;
    slave_req.rxCnt = 0;
    slave_req.completeCB = NULL;

    /***** Perform Transaction *****/
    MXC_SPI_SlaveTransactionAsync(&slave_req);
    MXC_SPI_MasterTransaction(&master_req);

    /***** Verify Results *****/
    if (memcmp(slave_rx, master_tx, sizeof(master_tx)) != 0) { // Master->Slave
        printf("\nSlave failed to receive data.\n");
        LED_On(0);
        return E_COMM_ERR;
    } else if (memcmp(master_rx, slave_tx, sizeof(slave_tx)) != 0) { // Slave->Master
        printf("\nMaster failed to receive data.\n");
        LED_On(0);
        return E_COMM_ERR;
    } else {
        printf("EXAMPLE SUCCEEDED!\n");
        LED_On(1);
    }

    MXC_SPI_Shutdown(SPI_MASTER);
    MXC_SPI_Shutdown(SPI_SLAVE);

    return E_NO_ERROR;
}
