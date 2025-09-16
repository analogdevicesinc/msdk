/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

/**
 * @file        main.c
 * @brief       AES Example
 * @details     Encryption and decryption of AES on different modes (ECB and OFB) with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc.h"

/***** Definitions *****/
#define MXC_AES_DATA_LENGTH 8 //4 words

#define MXC_AES_ENC_DATA_LENGTH 8 //Always multiple of 4
//(equal to or greater than MXC_AES_DATA_LENGTH)

/***** Globals *****/
uint32_t inputData[MXC_AES_DATA_LENGTH] = { 0x873AC125, 0x2F45A7C8, 0x3EB7190,  0x486FA931,
                                            0x94AE56F2, 0x89B4D0C1, 0x2F45A7C8, 0x3EB7190 };
uint32_t encryptedData[MXC_AES_ENC_DATA_LENGTH] = { 0 };
uint32_t decryptedData[MXC_AES_DATA_LENGTH] = { 0 };

//AES request
mxc_aes_req_t req;

volatile int dma_flag = 0;
/***** Functions *****/
void DMA1_CH0_IRQHandler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
    dma_flag++;
}

int AES_encrypt(int asynchronous, mxc_aes_keys_t key)
{
    req.length = MXC_AES_DATA_LENGTH;
    req.inputData = inputData;
    req.resultData = encryptedData;
    req.keySize = key;
    req.encryption = MXC_AES_ENCRYPT_EXT_KEY;

    MXC_AES_Init(MXC_DMA1);

    if (asynchronous) {
        MXC_AES_EncryptAsync(&req);

        while (dma_flag == 0) {}

        dma_flag = 0;
    } else {
        MXC_AES_Encrypt(&req);
    }

    return E_NO_ERROR;
}

int AES_decrypt(int asynchronous, mxc_aes_keys_t key)
{
    req.length = MXC_AES_DATA_LENGTH;
    req.inputData = encryptedData;
    req.resultData = decryptedData;
    req.keySize = key;
    req.encryption = MXC_AES_DECRYPT_INT_KEY;

    if (asynchronous) {
        MXC_AES_DecryptAsync(&req);

        while (dma_flag == 0) {}

        dma_flag = 0;
    } else {
        MXC_AES_Decrypt(&req);
    }

    MXC_AES_Shutdown();

    if (memcmp(inputData, decryptedData, MXC_AES_DATA_LENGTH) != 0) {
        printf("\nData Mismatch");
        return 1;
    }

    printf("\nData Verified");
    return E_NO_ERROR;
}

// *****************************************************************************
int main(void)
{
    printf("\n***** AES Example *****\n");

    int fail = 0;

    MXC_DMA_ReleaseChannel(0);
    NVIC_EnableIRQ(DMA1_CH0_IRQn);

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

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    LED_On(0);
    return E_NO_ERROR;
}
