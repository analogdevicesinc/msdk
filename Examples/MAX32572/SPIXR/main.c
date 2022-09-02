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

/**
 * @file    main.c
 * @brief   SPIXR example writing to SRAM Microchip 23LC1024
 * @details Setup, Initialize, Write, and verifies the data written to SRAM.
 *          This example is showing how to configure the SRAM 23LC1024 and
 *          uses the SPIXR library to write and read data from it in Quad mode.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/
// RAM Vendor Specific Commands
#define A1024_READ  0x03
#define A1024_WRITE 0x02
#define A1024_EQIO  0x38

// RAM Vendor Specific Values
#define BUFFER_SIZE   16
#define A1024_ADDRESS 0x80000000

/***** Globals *****/
mxc_spixr_cfg_t init_cfg = {
    0x08,                /* Number of bits per character     */
    MXC_SPIXR_QUAD_SDIO, /* SPI Data Width                   */
    0x04,                /* num of system clocks between SS active & first serial clock edge     */
    0x08,                /* num of system clocks between last serial clock edge and ss inactive  */
    0x10,                /* num of system clocks between transactions (read / write)             */
    5000000              /* Baud freq                        */
};

/***** Functions *****/
/******************************************************************************/
void setup(void)
{
    uint8_t quad_cmd = A1024_EQIO; /* pre-defined command to use quad mode         */

    // // Initialize the desired configuration
    if (MXC_SPIXR_Init(&init_cfg) != E_NO_ERROR) {
        printf("\nSPIXR was not initialized properly.\n");
        printf("\nExample Failed\n");

        while (1)
            ;
    }

    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_SRCC_DIS;

    MXC_SPIXR->dma &= ~MXC_F_SPIXR_DMA_RX_DMA_EN;
    MXC_SPIXR->dma |= MXC_F_SPIXR_DMA_TX_FIFO_EN;
    MXC_SPIXR->ctrl3 &= ~MXC_F_SPIXR_CTRL3_DATA_WIDTH;

    // Setup to communicate in quad mode
    MXC_SPIXR_SendCommand(&quad_cmd, 1, 1);

    // Wait until quad cmd is sent
    while (MXC_SPIXR_Busy())
        ;

    MXC_SPIXR_SetWidth(MXC_SPIXR_QUAD_SDIO);
    MXC_SPIXR_ThreeWireModeDisable();
    MXC_SPIXR_DmaTXFIFODisable();
    MXC_SPIXR_DmaRXFIFODisable();
    MXC_SPIXR_TXFIFODisable();
    MXC_SPIXR_RXFIFODisable();

    MXC_SPIXR_ExMemUseDummy(0x01);
    MXC_SPIXR_ExMemSetReadCommand(A1024_READ);
    MXC_SPIXR_ExMemSetWriteCommand(A1024_WRITE);
    MXC_SPIXR_ExMemEnable();
}

// *****************************************************************************

int main(void)
{
    // Defining Variable(s) to write & store data to RAM
    uint8_t write_buffer[BUFFER_SIZE], read_buffer[BUFFER_SIZE];
    uint8_t* address = (uint8_t*)A1024_ADDRESS;
    ; /* Variable to store address of RAM */
    int temp, i;
    int fail = 0;

    printf("\n****************** SPIXR Example ******************\n\n");
    printf("This example communicates with an MX25 SPI RAM on the\n");
    printf("EvKit using Quad SPI mode And the SPIXR peripheral\n");

    // Configure the SPIXR
    printf("\nSetting up the SPIXR\n");

    setup();

    // Initialize & write pseudo-random data to be written to the RAM
    // printf("Initializing & Writing pseudo-random data to RAM \n");
    srand(0);
    printf("\nTX BUFFER:\t ");

    for (i = 0; i < BUFFER_SIZE; i++) {
        read_buffer[i]  = 0;
        temp            = rand();
        write_buffer[i] = temp;
        // Write the data to the RAM
        *(address + i) = write_buffer[i];
        printf("%x  ", write_buffer[i]);
    }

    // Read data from RAM
    printf("\n\nRX BUFFER:\t ");

    for (i = 0; i < BUFFER_SIZE; i++) {
        read_buffer[i] = *(address + i);
        printf("%x  ", read_buffer[i]);
    }

    // Disable the SPIXR
    MXC_SPIXR_Disable();

    // Verify data being read from RAM
    if (memcmp(write_buffer, read_buffer, BUFFER_SIZE)) {
        printf("\n\nDATA IS NOT VERIFIED.\n\n");
        fail++;
    } else {
        printf("\n\nDATA IS VERIFIED.\n\n");
    }

    if (fail == 0) {
        printf("EXAMPLE SUCCEEDED\n");
    } else {
        printf("EXAMPLE FAILED\n");
    }

    while (1)
        ;
}
