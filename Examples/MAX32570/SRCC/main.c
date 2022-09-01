/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 * @file        main.c
 * @brief       External Memory Cache Controller (SRCC) using SPIXR, writing to external SRAM
 * @details     Writing to External SRAM With Cache Enabled and Cache Disabled.
 */

/***** Includes *****/
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/
// RAM Vendor Specific Commands
#define A1024_READ 0x03
#define A1024_WRITE 0x02
#define A1024_EQIO 0x38

// RAM Vendor Specific Values
#define BUFFER_SIZE 512
#define A1024_ADDRESS 0x80000000
#define ITERATIONS 5000

/***** Globals *****/
int s, ss;

mxc_spixr_cfg_t init_cfg = {
    0x08, /* Number of bits per character     */
    MXC_SPIXR_QUAD_SDIO, /* SPI Data Width                   */
    0x04, /* num of system clocks between SS active & first serial clock edge     */
    0x08, /* num of system clocks between last serial clock edge and ss inactive  */
    0x10, /* num of system clocks between transactions (read / write)             */
    0x800000, /* Baud freq                        */
};

/***** Functions *****/

void setup(void)
{
    uint8_t quad_cmd = A1024_EQIO; /* pre-defined command to use quad mode         */

    // Initialize the desired configuration
    if (MXC_SPIXR_Init(&init_cfg) != E_NO_ERROR) {
        printf("FAILED: SPIXR was not initialized properly.\n");
        return;
    }

    /* Hide this with function in SPIXR.C later */
    MXC_SPIXR->dma &= ~MXC_F_SPIXR_DMA_RX_DMA_EN;
    MXC_SPIXR->dma |= MXC_F_SPIXR_DMA_TX_FIFO_EN;
    MXC_SPIXR->ctrl3 &= ~MXC_F_SPIXR_CTRL3_DATA_WIDTH;

    // Setup to communicate in quad mode
    MXC_SPIXR_SendCommand(&quad_cmd, 1, 1);

    // Wait until quad cmd is sent
    while (MXC_SPIXR_Busy()) { }

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

void start_timer(void)
{
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed setup_timer.\n");
        return;
    }

    MXC_RTC_Start();
}

void stop_timer(void)
{
    printf("Time elapsed: %d.%03d \n", MXC_RTC->sec, MXC_RTC->ssec);
    MXC_RTC_Stop();
}

void test_function(void)
{
    // Defining Variable(s) to write & store data to RAM
    uint8_t write_buffer[BUFFER_SIZE], read_buffer[BUFFER_SIZE];
    uint8_t* address = (uint8_t*)A1024_ADDRESS;
    {
    }
    /* Variable to store address of RAM */
    int temp, i;

    // Configure the SPIXR
    setup();

    // Initialize & write pseudo-random data to be written to the RAM
    srand(0);

    for (i = 0; i < BUFFER_SIZE; i++) {
        temp = rand();
        write_buffer[i] = temp;
        // Write the data to the RAM
        *(address + i) = temp;
    }

    start_timer();

    for (temp = 0; temp < ITERATIONS; temp++) {
        // Read data from RAM
        for (i = 0; i < BUFFER_SIZE; i++) {
            read_buffer[i] = *(address + i);
        }

        // Verify data being read from RAM
        if (memcmp(write_buffer, read_buffer, BUFFER_SIZE)) {
            printf("FAILED: Data was not read properly.\n\n");
            break;
        }
    }

    stop_timer();

    // Disable the SPIXR
    MXC_SPIXR_Disable();
}

// *****************************************************************************
int main(void)
{
    printf("***** SRCC Example *****\n\n");

    // Instruction cache enabled
    printf("Running test reads with data cache enabled.   ");
    MXC_SRCC_Enable();
    test_function();

    // Instruction cache disabled
    printf("Running test reads with data cache disabled.  ");
    MXC_SRCC_Disable();
    test_function();

    printf("Example complete.\n");

    while (1) { }
}
