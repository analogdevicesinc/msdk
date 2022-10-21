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
 * @file        main.c
 * @brief       AES Example
 * @details     Encryption and decryption of AES on different modes (ECB and OFB) with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/

volatile int wait;
volatile int callback_result;
volatile int counter;

#define MXC_AES_DATA_LEN (128 / 8)
#define MXC_AES_KEY_128_LEN (128 / 8)
#define MXC_AES_KEY_192_LEN (192 / 8)
#define MXC_AES_KEY_256_LEN (256 / 8)

/***** Globals *****/
unsigned int rnd_no[4] = { 0 };
uint8_t var_rnd_no[16] = { 0 };

char temp[] = { 0x00, 0x00, 0x00 };

/***** Globals *****/
char result[512];

/***** Functions *****/
void CRYPTO_IRQHandler(void)
{
    MXC_CTB_Handler();
}

void Test_Callback(void *req, int result)
{
    wait = 0;
    callback_result = result;
}

//Convert ascii to byte
void ascii_to_byte(const char *src, char *dst, int len)
{
    int i;

    for (i = 0; i < len; ++i) {
        int val;
        temp[0] = *src;
        src++;
        temp[1] = *src;
        src++;
        sscanf(temp, "%0x", &val);
        dst[i] = val;
    }

    return;
}

//Verify by comparing calculated to expected
int AES_check(char *calculated, char *expected, int len)
{
    int i, fail = 0;

    for (i = 0; i < len; ++i) {
        if (calculated[i] != expected[i]) {
            ++fail;
        }
    }

    if (fail > 0) {
        printf("Fail.\n");
    } else {
        printf("Pass.\n");
        return 0;
    }

    return -1;
}

int AES128_ECB_enc(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef36";
    char key[MXC_AES_KEY_128_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_128_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "00000000000000000000000000000000";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES128);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_128_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_EncryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Encrypt(&cipher_req);
    }

    const char *_expected = "322FD6E503395CDB89A77AC53D2B954F";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES128_ECB_dec(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef36";
    char key[MXC_AES_KEY_128_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_128_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "322FD6E503395CDB89A77AC53D2B954F";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES128);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_128_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_DecryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Decrypt(&cipher_req);
    }

    const char *_expected = "00000000000000000000000000000000";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES192_ECB_enc(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef367a16f8635f626473";
    char key[MXC_AES_KEY_192_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_192_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "00000000000000000000000000000000";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES192);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_192_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_EncryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Encrypt(&cipher_req);
    }

    const char *_expected = "91D29E37E9B5B39CB2BF1AC8FD0FCFD2";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES192_ECB_dec(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef367a16f8635f626473";
    char key[MXC_AES_KEY_192_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_192_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "91D29E37E9B5B39CB2BF1AC8FD0FCFD2";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES192);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_192_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_DecryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Decrypt(&cipher_req);
    }

    const char *_expected = "00000000000000000000000000000000";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES256_ECB_enc(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef367a16f8635f6264737591c5c07bf57a3e";
    char key[MXC_AES_KEY_256_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_256_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "00000000000000000000000000000000";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_256_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_EncryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Encrypt(&cipher_req);
    }

    const char *_expected = "a74289fe73a4c123ca189ea1e1b49ad5";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES256_ECB_dec(int asynchronous)
{
    printf(asynchronous ? "Test Cipher Async\n" : "Test Cipher Sync\n");

    char *_key = "797f8b3d176dac5b7e34a2d539c4ef367a16f8635f6264737591c5c07bf57a3e";
    char key[MXC_AES_KEY_256_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_256_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    char *_pt = "a74289fe73a4c123ca189ea1e1b49ad5";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)pt, MXC_AES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_AES_KEY_256_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_DecryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Decrypt(&cipher_req);
    }

    const char *_expected = "00000000000000000000000000000000";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

// *****************************************************************************
int main(void)
{
    printf("\n***** AES Example *****\n");

    int fail = 0;

    //ECB
    fail += AES128_ECB_enc(0);
    fail += AES128_ECB_enc(1);
    fail += AES128_ECB_dec(0);
    fail += AES128_ECB_dec(1);
    fail += AES192_ECB_enc(0);
    fail += AES192_ECB_enc(1);
    fail += AES192_ECB_dec(0);
    fail += AES192_ECB_dec(1);
    fail += AES256_ECB_enc(0);
    fail += AES256_ECB_enc(1);
    fail += AES256_ECB_dec(0);
    fail += AES256_ECB_dec(1);

    printf("\n");

    if (fail == 0) {
        printf("Example Succeeded\n");
        return 0;
    } else {
        printf("Example Failed\n");
        return -1;
    }

    return 0;
}
