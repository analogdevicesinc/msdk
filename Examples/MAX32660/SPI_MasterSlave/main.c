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
#include "spimss.h"
#include "uart.h"

/***** Definitions *****/
#define DATA_LEN 1024 // Words
#define DATA_SIZE 8
#define VALUE 0xFF
#define SPI_SPEED 100000 // Bit Rate (Max.: 1,850,000)

#define SPI_MASTER MXC_SPIMSS
#define SPI_MASTER_SSIDX 0
#define SPI_SLAVE MXC_SPI0
#define SPI_SLAVE_SSIDX 0
#define SPI_SLAVE_IRQ SPI0_IRQn

/***** Globals *****/
uint8_t master_rx[DATA_LEN];
uint8_t master_tx[DATA_LEN];
uint8_t slave_rx[DATA_LEN];
uint8_t slave_tx[DATA_LEN];

/***** Functions *****/
void unblk_spi(void)
{
    /*
     * To initialize SPI0, it's CS must be de-asserted. However,
     * SPIMSS does not take control of the state of the CS pin
     * until the MXC_SPIMSS_MasterTrans function and by default
     * the CS pin idles in the asserted state. To ensure proper
     * SPI0 initialization in this example, we manually de-assert
     * the CS line here. This does not need to be done for normal
     * SPI applications.
     */
    mxc_gpio_cfg_t spimss_cs;
    spimss_cs.port = MXC_GPIO0;
    spimss_cs.mask = MXC_GPIO_PIN_13;
    spimss_cs.func = MXC_GPIO_FUNC_OUT;
    spimss_cs.pad = MXC_GPIO_PAD_NONE;

    MXC_GPIO_Config(&spimss_cs);
    MXC_GPIO_OutSet(spimss_cs.port, spimss_cs.mask);
}

void SPI_Slave_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI_SLAVE);
}

int main(void)
{
    mxc_spi_req_t slave_req;
    mxc_spimss_req_t master_req;

    printf("\n************************ SPI Master-Slave Example ************************\n");
    printf("This example sends data between two SPI peripherals in the MAX32660.\n");
    printf("SPI0 is configured as the slave and SPIMSS (SPI1) is configured as the master.\n");
    printf("Each SPI peripheral sends 1024 bytes on the SPI bus. If the data received\n", DATA_LEN);
    printf("by each SPI instance matches the data sent by the other instance, the\n");
    printf("LED will illuminate.\n\n");

    printf("Press SW2 to begin transaction.\n\n");
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}
    Console_Shutdown(); // Console UART shares pins with SPIMSS so de-initialize it here.
    while (!PB_Get(0)) {}

    /***** Initialize data buffers *****/
    for (int i = 0; i < DATA_LEN; i++) {
        master_tx[i] = i;
        slave_tx[i] = i;
    }
    memset(master_rx, 0x0, DATA_LEN * sizeof(uint8_t));
    memset(slave_rx, 0x0, DATA_LEN * sizeof(uint8_t));

    /***** Configure slave (SPI0) *****/
    if (MXC_SPI_Init(SPI_SLAVE, 0, 0, 1, 0, SPI_SPEED) != E_NO_ERROR) {
        printf("\nSPI SLAVE INITIALIZATION ERROR\n");
        while (1) {}
    }

    unblk_spi();
    MXC_SPI_SetDataSize(SPI_SLAVE, DATA_SIZE);
    MXC_SPI_SetWidth(SPI_SLAVE, SPI_WIDTH_STANDARD);

    MXC_NVIC_SetVector(SPI_SLAVE_IRQ, SPI_Slave_IRQHandler);
    NVIC_EnableIRQ(SPI_SLAVE_IRQ);

    /***** Configure master (SPIMSS) *****/
    if (MXC_SPIMSS_Init(SPI_MASTER, 0, SPI_SPEED, MAP_A) != E_NO_ERROR) {
        printf("\nSPI MASTER INITIALIZATION ERROR\n");
        while (1) {}
    }

    /***** Initialize Transaction Parameters *****/
    master_req.tx_data = (void *)master_tx;
    master_req.rx_data = (void *)master_rx;
    master_req.len = DATA_LEN;
    master_req.bits = DATA_SIZE;
    master_req.tx_num = 0;
    master_req.rx_num = 0;
    master_req.callback = NULL;

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
    MXC_SPIMSS_MasterTrans(SPI_MASTER, &master_req);

    /***** Verify Results *****/
    Console_Init(); // Re-initialize Console UART
    if (memcmp(slave_rx, master_tx, sizeof(master_tx)) != 0) { // Master->Slave
        printf("Slave failed to receive data.\n");
        return E_COMM_ERR;
    } else if (memcmp(master_rx, slave_tx, sizeof(slave_tx)) != 0) { // Slave->Master
        printf("Master failed to receive data.\n");
        return E_COMM_ERR;
    } else {
        printf("EXAMPLE SUCCEEDED!\n");
        LED_On(0);
    }

    MXC_SPIMSS_Shutdown(SPI_MASTER);
    MXC_SPI_Shutdown(SPI_SLAVE);

    return E_NO_ERROR;
}
