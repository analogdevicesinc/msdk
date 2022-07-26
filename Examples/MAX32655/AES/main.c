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

/**
 * @file        main.c
 * @brief       AES Example
 * @details     Encryption and decryption of AES on different modes (ECB and OFB) with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "max32655.h"
#include "board.h"
#include "dma.h"
#include "aes.h"
#include "aes_regs.h"

/***** Definitions *****/
#define MXC_AES_DATA_LENGTH 8 //4 words

#define MXC_AES_ENC_DATA_LENGTH 8 //Always multiple of 4
//(equal to or greater than MXC_AES_DATA_LENGTH)

/***** Globals *****/
uint32_t inputData[MXC_AES_DATA_LENGTH]         = {0x873AC125, 0x2F45A7C8, 0x3EB7190,  0x486FA931,
                                           0x94AE56F2, 0x89B4D0C1, 0x2F45A7C8, 0x3EB7190};
uint32_t encryptedData[MXC_AES_ENC_DATA_LENGTH] = {0};
uint32_t decryptedData[MXC_AES_DATA_LENGTH]     = {0};

//AES request
mxc_aes_req_t req;

volatile int dma_flag = 0;
/***** Functions *****/
void DMA0_IRQHandler()
{
    MXC_DMA_Handler();
    dma_flag++;
}

int AES_encrypt(int asynchronous, mxc_aes_keys_t key)
{
    req.length     = MXC_AES_DATA_LENGTH;
    req.inputData  = inputData;
    req.resultData = encryptedData;
    req.keySize    = key;
    req.encryption = MXC_AES_ENCRYPT_EXT_KEY;

    MXC_AES_Init();

    if (asynchronous) {
        MXC_AES_EncryptAsync(&req);

        while (dma_flag == 0)
            ;

        dma_flag = 0;
    } else {
        MXC_AES_Encrypt(&req);
    }

    return E_NO_ERROR;
}

int AES_decrypt(int asynchronous, mxc_aes_keys_t key)
{
    req.length     = MXC_AES_DATA_LENGTH;
    req.inputData  = encryptedData;
    req.resultData = decryptedData;
    req.keySize    = key;
    req.encryption = MXC_AES_DECRYPT_INT_KEY;

    if (asynchronous) {
        MXC_AES_DecryptAsync(&req);

        while (dma_flag == 0)
            ;

        dma_flag = 0;
    } else {
        MXC_AES_Decrypt(&req);
    }

    MXC_AES_Shutdown();

    if (memcmp(inputData, decryptedData, MXC_AES_DATA_LENGTH) == 0) {
        printf("\nData Verified");
        return E_NO_ERROR;
    }

    printf("\nData Mismatch");

    return 1;
}

// *****************************************************************************
int main(void)
{
    printf("\n***** AES Example *****\n");

    int fail = 0;

    MXC_DMA_ReleaseChannel(0);
    NVIC_EnableIRQ(DMA0_IRQn);

    //ECB
    printf("\nAES 128 bits Key Test");
    AES_encrypt(0, MXC_AES_128BITS);
    fail += AES_decrypt(0, MXC_AES_128BITS);
    printf("\n\nAES 192 bits Key Test");
    AES_encrypt(0, MXC_AES_192BITS);
    fail += AES_decrypt(0, MXC_AES_192BITS);
    printf("\n\nAES 256 bits Key Test");
    AES_encrypt(0, MXC_AES_256BITS);
    fail += AES_decrypt(0, MXC_AES_256BITS);

    printf("\n");

    if (fail == 0) {
        printf("\nExample Succeeded\n");
    } else {
        printf("Example Failed\n");
    }

    while (1) {
    }

    return 0;
}
