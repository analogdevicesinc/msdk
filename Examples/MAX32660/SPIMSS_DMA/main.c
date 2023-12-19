/**
 * @file    main.c
 * @brief   SPI Master DMA Demo
 * @details Shows Master DMA loopback demo for SPI1 (AKA: SPIMSS)
 *          Read the printf() for instructions
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "led.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "spi.h"
#include "spimss.h"
#include "uart.h"
#include "dma.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spimss_regs.h"
#include "max32660.h"

#define SPIMSS MXC_SPIMSS
#define SPI_SPEED 10000000 
#define NUMBER_OF_UNIT_SIZE 512
#define TRANSFER_UNIT_SIZE  2           /*
                                        * Transfer unit size is defined according to the bit
                                        * count.(BitCount/8) is defined as transfer unit size.
                                        * tx_data and rx_data is define as uint16_t. So, unit
                                        * size is 16/8 = 2.
                                        */

#define SPIMSS_ENABLE_SS ((uint32_t)(0x1UL << 0)) 
#define SPIMSS_ENABLE_TRANSACTION ((uint32_t)(0x1UL << 0))
#define DATA_VALUE 0x0102
#define TEST_COUNT 100

/**
 * Transfer unit size is 2. So, we define tx_data and rx_data as uint16_t. 
*/
uint16_t tx_data[NUMBER_OF_UNIT_SIZE] = {0};
uint16_t rx_data[NUMBER_OF_UNIT_SIZE] = {0};

static uint8_t dma_tx_completed = 0;
static uint8_t dma_rx_completed = 0;

void spimss_callback(mxc_spimss_req_t *req, int error_code)
{
    if( req->deass ){
        SPIMSS->mode |= (SPIMSS_ENABLE_SS);           // Set Slave Select to HIGH.
    }
    SPIMSS->ctrl &= ~SPIMSS_ENABLE_TRANSACTION;       // Disable SPIMSS transaction.

    SPIMSS->dma &= ~(1 << 31); // Disable SPIMSS RX DMA requests
    SPIMSS->dma &= ~(1 << 15); // Disable SPIMSS TX DMA requests

    // Setting the dma flags to 0 for the next transaction.
    dma_tx_completed = 0;
    dma_rx_completed = 0;
}

void DMA0_IRQHandler(void){
    //Disable channel 0 interrupts.
    mxc_dma_ch_regs_t *tx_channel_reg = MXC_DMA_GetCHRegs(0);
    MXC_DMA_ChannelClearFlags(0, tx_channel_reg->stat);
    MXC_DMA_SetChannelInterruptEn(0, false, false);
    MXC_DMA_ReleaseChannel(0);
    dma_tx_completed = 1;
}

void DMA1_IRQHandler(void){
    //Disable channel 1 interrupts.
    mxc_dma_ch_regs_t *tx_channel_reg = MXC_DMA_GetCHRegs(1);
    MXC_DMA_ChannelClearFlags(1, tx_channel_reg->stat);
    MXC_DMA_SetChannelInterruptEn(1, false, false);
    MXC_DMA_ReleaseChannel(1);
    dma_rx_completed = 1;
}

int main()
{
    int i = 0;
    mxc_spimss_req_t spimss_dma_req;

    printf("\n\n************** SPIMSS-DMA Master Demo ****************\n");
    printf("This example configures the SPIMSS to send data between the MISO (P0.10) and\n");
    printf("MOSI (P0.11) pins over dma channel 0 and dma channel 1.  Connect these two pins\n");
    printf("together. This demo shows 1024 byte data transfer for 100 times using dma.\n");
    printf("During this demo you may see junk data printed to the serial port because the\n");
    printf("console UART shares the same pins as the SPIMSS. Dma channel 0 is used as tx from\n");
    printf("memory to SPIMSS TX FIFO and dma channel 1 is used for reading data from SPIMSS\n");
    printf("RX FIFO to memory.\n\n");

    // Preparation of the transmission data and receive data buffer.
    for (i = 0; i < NUMBER_OF_UNIT_SIZE; i++){
        tx_data[i] = DATA_VALUE + i;
        rx_data[i] = 0;
    }

    // Initialize SPIMSS with MAP_A. 
    // For MAP_A P0.10:MISO, P0.11:MOSI, P0.12:SCK, P0.13:SSEL
    if ( 0 != MXC_SPIMSS_Init(SPIMSS, 0, SPI_SPEED, MAP_A)){
        return -1;
    }

    // Prepare the SPIMSS data transaction request.
    spimss_dma_req.tx_data = tx_data;
    spimss_dma_req.rx_data = rx_data;
    spimss_dma_req.len     = NUMBER_OF_UNIT_SIZE;
    spimss_dma_req.bits    = 16;                       
    spimss_dma_req.deass   = 1; 
    spimss_dma_req.tx_num  = 0;
    spimss_dma_req.rx_num  = 0;
    spimss_dma_req.callback = spimss_callback;

    /**
     * DMA channel configuration. There is only one instance of DMA in MAX32660.
     * This instance has 4 channels. We want to use channel 0 and channel 1 for
     * data tx and rx respectively.
    */

    // Releasing channel 0 an 1 to use as tx and rx channels respectively.
    MXC_DMA_ReleaseChannel(0);
    MXC_DMA_ReleaseChannel(1);

    // Calling MXC_DMA_Init is compulsory to be able to use MXC_SPIMSS_MasterTransDMA API.
    MXC_DMA_Init();

    for( i = 0; i < TEST_COUNT; i++){

        // Calling SPIM transaction function for communication over DMA.
        if(E_NO_ERROR != MXC_SPIMSS_MasterTransDMA(SPIMSS, &spimss_dma_req)){
            return -1;
        }

        // Wait until we complete the transaction.
        while(dma_rx_completed != 1 || dma_tx_completed != 1){}

        // Callback function to update SS and Transaction control bit.
        spimss_dma_req.callback(&spimss_dma_req, E_NO_ERROR);

        MXC_Delay(MXC_DELAY_MSEC(10));
    }

    MXC_SPIMSS_Shutdown(SPIMSS);

    if(0 == memcmp((void*)rx_data, (void*)tx_data, TRANSFER_UNIT_SIZE * NUMBER_OF_UNIT_SIZE)){
        printf("\nTest succesful!\n");
    }else{
        printf("\nTest failed!\n");
    }
    while(1){}
    return 0;
}