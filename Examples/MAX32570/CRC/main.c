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
 * @brief       Example showing how to use the CRC module. Covers 16 and 32-bit CRC.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/
#define POLY 0xEDB88320
#define CHECK 0xDEBB20E3
/***** Globals *****/
volatile int wait;
volatile int callback_result;
volatile int counter;
/***** Functions *****/

void CRYPTO_IRQHandler(void)
{
    MXC_CTB_Handler();
}

void Test_Callback(void* req, int result)
{
    wait = 0;
    callback_result = result;
}

void Test_Result(int result)
{
    if (result) {
        printf(" * Failed *\n");
    } else {
        printf("   Passed  \n");
    }
}

void Test_CRC(int asynchronous)
{
    uint32_t array[101];
    int i;

    printf(asynchronous ? "Test CRC Async\n" : "Test CRC Sync\n");

    for (i = 0; i < 100; i++) { array[i] = i; }

    MXC_CTB_Init(MXC_CTB_FEATURE_CRC | MXC_CTB_FEATURE_DMA);

    // Load CRC polynomial into crc polynomial register
    MXC_CTB_CRC_SetPoly(POLY);

    mxc_ctb_crc_req_t crc_req = { (uint8_t*)&array, 400, 0, &Test_Callback };

    MXC_CTB_EnableInt();

    if (asynchronous) {
        wait = 1;
        MXC_CTB_CRC_ComputeAsync(&crc_req);

        while (wait) { }
    } else {
        MXC_CTB_CRC_Compute(&crc_req);
    }

    array[100] = ~(crc_req.resultCRC);

    crc_req.dataLen += sizeof(array[100]);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_CRC_ComputeAsync(&crc_req);

        while (wait) { }
    } else {
        MXC_CTB_CRC_Compute(&crc_req);
    }

    Test_Result(CHECK != crc_req.resultCRC);
    MXC_CTB_Shutdown(MXC_CTB_FEATURE_CRC | MXC_CTB_FEATURE_DMA);
}

// *****************************************************************************
int main(void)
{
    Test_CRC(0);
    Test_CRC(1);

    while (1) { }
}
