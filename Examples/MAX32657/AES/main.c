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
 * @details     Encryption and decryption of AES with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "mxc.h"

/***** Definitions *****/
#define MXC_AES_DATA_LENGTH 4 //4 words

#define MXC_AES_ENC_DATA_LENGTH 4 //Always multiple of 4
//(equal to or greater than MXC_AES_DATA_LENGTH)

/***** Globals *****/
uint32_t inputData[MXC_AES_DATA_LENGTH] = { 0x873AC125, 0x2F45A7C8, 0x3EB7190,  0x486FA931,
                                            0x94AE56F2, 0x89B4D0C1, 0x2F45A7C8, 0x3EB7190 };
uint32_t encryptedData[MXC_AES_ENC_DATA_LENGTH] = { 0 };
uint32_t decryptedData[MXC_AES_DATA_LENGTH] = { 0 };

//AES request
mxc_aes_req_t req;

volatile int aes_done = 0;

/***** Functions *****/
void AES_IRQHandler(void)
{
    MXC_AES_Handler();
}

void DMA1_AESTX_IRQHandler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
}

void DMA1_AESRX_IRQHandler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
}

void aes_callback(void *req, int result)
{
    aes_done = 1;
}

/**
 *  For this example, AES_encrypt(...) initializes the AES peripheral
 *  and must be called before AES_decrypt(...)
 */
int AES_encrypt(bool asynchronous, bool dma, mxc_aes_keys_t key)
{
    int8_t rx_channel, tx_channel;

    // Must only select one or the other
    if (asynchronous == true && dma == true) {
        return 1;
    }

    req.length = MXC_AES_DATA_LENGTH;
    req.inputData = inputData;
    req.resultData = encryptedData;
    req.keySize = key;
    req.encryption = MXC_AES_ENCRYPT_EXT_KEY;
    req.callback = aes_callback;

    if (MXC_AES_Init(MXC_DMA1) != E_NO_ERROR) {
        // Fail
        return 1;
    }

    if (dma) {
        if (MXC_AES_PreInitDMA(&rx_channel, &tx_channel) != E_NO_ERROR) {
            // Fail
            return 1;
        }

        NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(MXC_DMA1, tx_channel));
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(MXC_DMA1, tx_channel), DMA1_AESTX_IRQHandler);

        NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(MXC_DMA1, rx_channel));
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(MXC_DMA1, rx_channel), DMA1_AESRX_IRQHandler);

        if (MXC_AES_EncryptDMA(&req) != E_NO_ERROR) {
            // Fail
            return 1;
        }

        while (aes_done == 0) {}

        aes_done = 0;

    } else if (asynchronous) {
        NVIC_EnableIRQ(AES_IRQn);
        MXC_AES_EnableInt(MXC_F_AES_INTEN_DONE);

        if (MXC_AES_EncryptAsync(&req) != E_NO_ERROR) {
            // Fail
            return 1;
        }

        while (aes_done == 0) {}

        aes_done = 0;

    } else {
        // When not using external keys, an internal decryption key is generated
        //  during an encryption operation. It may be necessary to complete
        //  a dummy encryption before doing the first decryption to ensure
        //  that it was generated properly.
        // MXC_AES_Encrypt(&req);

        MXC_AES_Encrypt(&req);
    }

    return E_NO_ERROR; // 0
}

/**
 *  For this example, AES_decrypt(...) must be called after AES_encrypt(...)
 *  and then it shuts down or de-initializes the AES peripheral to allow for
 *  the next AES_encrypt(...) operation to start.
 */
int AES_decrypt(bool asynchronous, bool dma, mxc_aes_keys_t key)
{
    int8_t rx_channel, tx_channel;

    // Must only select one or the other
    if (asynchronous == true && dma == true) {
        return 1;
    }

    req.length = MXC_AES_DATA_LENGTH;
    req.inputData = encryptedData;
    req.resultData = decryptedData;
    req.keySize = key;
    req.encryption = MXC_AES_DECRYPT_INT_KEY;
    req.callback = aes_callback;

    // AES already initialized in 'AES_encrypt(...)'

    if (dma) {
        // DMA for AES async operations already initializes in 'AES_encrypt(...)'

        MXC_AES_DecryptDMA(&req);

        while (aes_done == 0) {}

        aes_done = 0;

        MXC_AES_GetTXDMAChannel(&tx_channel);
        MXC_AES_GetRXDMAChannel(&rx_channel);

        NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(MXC_DMA1, tx_channel));
        NVIC_DisableIRQ(MXC_DMA_CH_GET_IRQ(MXC_DMA1, rx_channel));

    } else if (asynchronous) {

        MXC_AES_DecryptAsync(&req);

        while (aes_done == 0) {}

        aes_done = 0;

        NVIC_DisableIRQ(AES_IRQn);
        MXC_AES_DisableInt(MXC_F_AES_INTEN_DONE);

    } else {
        MXC_AES_Decrypt(&req);
    }

    MXC_AES_Shutdown();

    printf("\nINPUT         |       RESULT\n");
    for (int i = 0; i < MXC_AES_DATA_LENGTH; i++) {
        printf("0x%08x    |   0x%08x\n", req.inputData[i], req.resultData[i]);
    }

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
    // Clear screen (ANSI escape code supported terminals only)
    printf("\033[H\033[J");

    printf("\n***** AES Example *****\n");

    int fail = 0;

    printf("\nAES 128 bits Key Test");
    fail += AES_encrypt(0, 0, MXC_AES_128BITS);
    fail += AES_decrypt(0, 0, MXC_AES_128BITS);

    printf("\n\nAES 192 bits Key Test");
    fail += AES_encrypt(0, 0, MXC_AES_192BITS);
    fail += AES_decrypt(0, 0, MXC_AES_192BITS);

    printf("\n\nAES 256 bits Key Test");
    fail += AES_encrypt(0, 0, MXC_AES_256BITS);
    fail += AES_decrypt(0, 0, MXC_AES_256BITS);

    printf("\n\nAES 128 bits Key Test (async)");
    fail += AES_encrypt(1, 0, MXC_AES_128BITS);
    fail += AES_decrypt(1, 0, MXC_AES_128BITS);

    printf("\n\nAES 192 bits Key Test (async)");
    fail += AES_encrypt(1, 0, MXC_AES_192BITS);
    fail += AES_decrypt(1, 0, MXC_AES_192BITS);

    printf("\n\nAES 256 bits Key Test (async)");
    fail += AES_encrypt(1, 0, MXC_AES_256BITS);
    fail += AES_decrypt(1, 0, MXC_AES_256BITS);

    printf("\n\nAES 128 bits Key Test (dma)");
    fail += AES_encrypt(0, 1, MXC_AES_128BITS);
    fail += AES_decrypt(0, 1, MXC_AES_128BITS);

    printf("\n\nAES 192 bits Key Test (dma)");
    fail += AES_encrypt(0, 1, MXC_AES_192BITS);
    fail += AES_decrypt(0, 1, MXC_AES_192BITS);

    printf("\n\nAES 256 bits Key Test (dma)");
    fail += AES_encrypt(0, 1, MXC_AES_256BITS);
    fail += AES_decrypt(0, 1, MXC_AES_256BITS);

    if (fail != 0) {
        printf("\n\nExample Failed\n");
        return E_FAIL;
    }

    printf("\n\nExample Succeeded\n");
    LED_On(0);
    return E_NO_ERROR;
}
