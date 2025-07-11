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
 * @file    main.c
 * @brief       PUF Example
 * @details     Generation and use of PUF keys.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"

#include "puf.h"
#include "ctb.h"

/***** Definitions *****/

/***** Globals *****/
uint8_t NULL_key[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t inputData0[16] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Expected ciphertext using zero key
// Input: 80000000000000000000000000000000
// ddc6bf790c15760d8d9aeb6f9a75fd4e


/***** Functions *****/

// *****************************************************************************
static void hexdump(uint8_t *data, int length)
{
    for (int i = 0;i < length;i++)
    {
        if (((i & 0xF) == 0) && (i != 0))
        {
            printf("\n");
        }
        printf("%02X ",data[i]);
    }
    printf("\n");
}

// *****************************************************************************
int main(void)
{
    int count = 0;
    uint8_t ciphertext[16];
    mxc_ctb_cipher_req_t aesReq;
    int result;

    printf("\n***** PUF Example *****\n");

    // Initialize CTB
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);

    printf("\n***** AES256 ECB Encryption with NULL Key *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);
    MXC_CTB_Cipher_SetKey(NULL_key, 32);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    printf("\nPUF KEY0 Generation: ");
    if (MXC_PUF_Generate_Key(MXC_PUF_KEY0) == E_NO_ERROR)
    {
        printf("Success\n");
    }
    else
    {
        printf("FAIL\n");
    }

    printf("\n***** AES ECB Encryption with PUF Key0 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY0);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    printf("\nPUF KEY1 Generation: ");
    if (MXC_PUF_Generate_Key(MXC_PUF_KEY1) == E_NO_ERROR)
    {
        printf("Success\n");
    }
    else
    {
        printf("FAIL\n");
    }

    printf("\n***** AES ECB Encryption with PUF Key1 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY1);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    printf("\nPUF KEY0 and KEY1 Generation: ");
    if (MXC_PUF_Generate_Key(MXC_PUF_KEY_BOTH) == E_NO_ERROR)
    {
        printf("Success\n");
    }
    else
    {
        printf("FAIL\n");
    }

    printf("\n***** AES ECB Encryption with PUF Key0 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY0);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    printf("\n***** AES ECB Encryption with PUF Key1 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY1);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    // Clear the PUF keys
    MXC_PUF_Clear_Keys();

    printf("\n***** AES ECB Encryption with Cleared PUF Key0 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY0);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }

    printf("\n***** AES ECB Encryption with Cleared PUF Key1 *****\n");
    // Set to AES256 in ECB mode.
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_AES_PUFKEY1);

    // Prepare and execute encryption
    aesReq.plaintext = inputData0;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = (uint8_t *)ciphertext;
    result = MXC_CTB_Cipher_Encrypt(&aesReq);
    if (result)
    {
        printf("Encrypt call failed with result: %d\n",result);
    }
    else
    {
        printf("Ciphertext:\n");
        hexdump(ciphertext,16);
    }


    while (1) {
        LED_On(LED_RED);
        MXC_Delay(500000);
        LED_Off(LED_RED);
        MXC_Delay(500000);
        printf("count = %d\n", count++);
    }
}


