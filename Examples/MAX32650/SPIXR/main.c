/**
 * @file
 * @brief   SPIXR example writing to External SRAM
 * @details Setup, Initialize, Write, and verifies the data written to SRAM.
 *          This example shows how to configure the External SRAM and
 *          uses the SPIXR library to write and read data from it in Quad mode.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "spixr.h"

/* **** Definitions **** */
// RAM Vendor Specific Commands
#define A1024_READ 0x03
#define A1024_WRITE 0x02
#define A1024_EQIO 0x38

// RAM Vendor Specific Values
#define BUFFER_SIZE 5000
#define A1024_ADDRESS 0x80000000

/* **** Globals **** */
mxc_spixr_cfg_t init_cfg = {
    0x08, /* Number of bits per character     */
    MXC_SPIXR_QUAD_SDIO, /* SPI Data Width                   */
    0x04, /* num of system clocks between SS active & first serial clock edge     */
    0x08, /* num of system clocks between last serial clock edge and ss inactive  */
    0x10, /* num of system clocks between transactions (read / write)             */
    0x1, /* Baud freq                        */
};

/* **** Functions **** */

/* ************************************************************************** */
void setup(void)
{
    uint8_t quad_cmd = A1024_EQIO; /* pre-defined command to use quad mode         */

    // Initialize the desired configuration
    if (MXC_SPIXR_Init(&init_cfg) == E_NO_ERROR) {
        printf("SPIXR was initialized properly.\n");
    } else {
        printf("SPIXR was not initialized properly.\n");
    }

    /* Hide this with function in SPIXR.C later */
    MXC_SPIXR->dma &= ~MXC_S_SPIXR_DMA_RX_DMA_EN_EN;
    MXC_SPIXR->dma |= MXC_F_SPIXR_DMA_TX_FIFO_EN;
    MXC_SPIXR->ctrl3 &= ~MXC_F_SPIXR_CTRL3_DATA_WIDTH;

    // Setup to communicate in quad mode
    MXC_SPIXR_SendCommand(&quad_cmd, 1, 1);
    // Wait until quad cmd is sent
    while (MXC_SPIXR_Busy()) {}

    MXC_SPIXR->ctrl3 &= ~MXC_F_SPIXR_CTRL3_DATA_WIDTH;
    MXC_SPIXR->ctrl3 |= MXC_S_SPIXR_CTRL3_DATA_WIDTH_QUAD;
    MXC_SPIXR->ctrl3 &= ~MXC_F_SPIXR_CTRL3_THREE_WIRE;

    MXC_SPIXR->dma = 0x00; /* Disable the FIFOs for transparent operation  */
    MXC_SPIXR->xmem_ctrl = (0x01 << MXC_F_SPIXR_XMEM_CTRL_XMEM_DCLKS_POS) |
                           (A1024_READ << MXC_F_SPIXR_XMEM_CTRL_XMEM_RD_CMD_POS) |
                           (A1024_WRITE << MXC_F_SPIXR_XMEM_CTRL_XMEM_WR_CMD_POS) |
                           MXC_F_SPIXR_XMEM_CTRL_XMEM_EN;

    // Do some sort of checking mechanism to see whether it is being initialized properly or no
    // Return 1 if success and 0 if not
}

// *****************************************************************************
int main(void)
{
    // Defining Variable(s) to write & store data to RAM
    uint8_t write_buffer[BUFFER_SIZE], read_buffer[BUFFER_SIZE];
    uint8_t *address = (uint8_t *)A1024_ADDRESS;

    /* Variable to store address of RAM */
    int temp, i;
    unsigned int seed = 0;

    printf("\n***** SPIXR Example communicating with RAM in SPI Quad Mode *****\n");

    // Configure the SPIXR
    printf("Setting up the SPIXR to communicate with RAM in Quad Mode \n");
    setup();

    // Initialize & write pseudo-random data to be written to the RAM
    printf("Initializing & Writing pseudo-random data to be written to RAM \n");
    for (i = 0; i < BUFFER_SIZE; i++) {
        temp = rand_r(&seed);
        write_buffer[i] = temp;
        // Write the data to the RAM
        *(address + i) = temp;
    }

    // Read data from RAM
    printf("Reading data from RAM and store it inside the read_buffer \n");
    for (i = 0; i < BUFFER_SIZE; i++) { read_buffer[i] = *(address + i); }

    // Verify data being read from RAM
    if (memcmp(write_buffer, read_buffer, BUFFER_SIZE)) {
        printf("Data is not verified.\n\n");
    } else {
        printf("Data is verified.\n\n");
    }

    // Disable the SPIXR
    MXC_SPIXR_Disable();

    return 0;
}
