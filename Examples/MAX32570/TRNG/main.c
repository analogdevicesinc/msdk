/**
 * @file        main.c
 * @brief       TRNG Example
 * @details
 */

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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <MAX32xxx.h>

volatile int wait;
volatile int callback_result;

/***** Globals *****/
uint8_t var_rnd_no[16] = { 0 };

void TRNG_IRQHandler(void)
{
    MXC_CTB_Handler();
}

void Test_Callback(void *req, int result)
{
    wait = 0;
    callback_result = result;
}

void print(char *stuff)
{
    int i, j, size = 4;

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            printf("0x%02x ", stuff[i * size + j]);
        }

        printf("\n");
    }

    return;
}

void Test_TRNG(int asynchronous)
{
    printf(asynchronous ? "\nTest TRNG Async\n" : "\nTest TRNG Sync\n");

    int num_bytes = 16;

    memset(var_rnd_no, 0, sizeof(var_rnd_no));

    MXC_CTB_Init(MXC_CTB_FEATURE_TRNG);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_TRNG_RandomAsync(var_rnd_no, num_bytes, &Test_Callback);

        while (wait) {}
    } else {
        MXC_CTB_TRNG_Random(var_rnd_no, num_bytes);
    }

    print((char *)var_rnd_no);

    MXC_CTB_Shutdown(MXC_CTB_FEATURE_TRNG);
}

int main(void)
{
    printf("\n\n********** TRNG Example **********\n");

    Test_TRNG(0);
    Test_TRNG(1);

    return 0;
}
