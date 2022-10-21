/**
 * @file        main.c
 * @brief       DES Example
 * @details     Encryption and decryption of DES on different modes (ECB) with different bit sizes (128, 192, and 256)
 */

/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-09-05 16:46:11 -0500 (Wed, 05 Sep 2018) $
 * $Revision: 37695 $
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/

volatile int wait;
volatile int callback_result;

#define MXC_DES_DATA_LEN \
    (64 /                \
     8) /**< Number of bytes in an DES plaintext or cyphertext block, which are always 64-bits long. */
#define MXC_DES_KEY_LEN (64 / 8) /**< Number of bytes in a TDES key. */
#define MXC_TDES_KEY_LEN (192 / 8) /**< Number of bytes in a TDES key. */

/***** Globals *****/
unsigned int rnd_no[4] = { 0 };
uint8_t var_rnd_no[16] = { 0 };

char temp[] = { 0x00, 0x00, 0x00 };

/***** Globals *****/
char result[512];

/***** Functions *****/
void CTB_IRQHandler(void)
{
    MXC_CTB_Handler();
}
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
int DES_check(char *calculated, char *expected, int len)
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

int DES_ECB_enc(int asynchronous)
{
    char *xkey = "2f5d4b8c12a4a9c1";
    char key[MXC_DES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xmsg = "0000000000000000";
    char msg[MXC_DES_DATA_LEN];
    char *xexpected = "20597b6decaf7166";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_DES_KEY_LEN);
    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);
    ascii_to_byte(xmsg, msg, MXC_DES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)msg, MXC_DES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_DES);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_DES_KEY_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_EncryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Encrypt(&cipher_req);
    }

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    return DES_check(result, expected, MXC_DES_DATA_LEN);
}

int DES_ECB_dec(int asynchronous)
{
    char *xkey = "00c3de5446614d35";
    char key[MXC_DES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xct = "d940635dcb8148ae";
    char ct[MXC_DES_DATA_LEN];
    char *xexpected = "0000000000000000";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_DES_KEY_LEN);
    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);
    ascii_to_byte(xct, ct, MXC_DES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)ct, MXC_DES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_DES);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_DES_KEY_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_DecryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Decrypt(&cipher_req);
    }

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    return DES_check(result, expected, MXC_DES_DATA_LEN);
}

int TDES_ECB_enc(int asynchronous)
{
    char *xkey = "2f5d4b8c12a4a9c1";
    char key[MXC_DES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xmsg = "0000000000000000";
    char msg[MXC_DES_DATA_LEN];
    char *xexpected = "20597b6decaf7166";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_DES_KEY_LEN);
    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);
    ascii_to_byte(xmsg, msg, MXC_DES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)msg, MXC_DES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_TDES);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_DES_KEY_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_EncryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Encrypt(&cipher_req);
    }

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    return DES_check(result, expected, MXC_DES_DATA_LEN);
}

int TDES_ECB_dec(int asynchronous)
{
    char *xkey = "00c3de5446614d35";
    char key[MXC_DES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xct = "d940635dcb8148ae";
    char ct[MXC_DES_DATA_LEN];
    char *xexpected = "0000000000000000";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_DES_KEY_LEN);
    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);
    ascii_to_byte(xct, ct, MXC_DES_DATA_LEN);

    mxc_ctb_cipher_req_t cipher_req = { (uint8_t *)ct, MXC_DES_DATA_LEN, (uint8_t *)iv_src,
                                        (uint8_t *)result, &Test_Callback };

    // Reset crypto block
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER | MXC_CTB_FEATURE_DMA);
    MXC_CTB_EnableInt();

    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_TDES);
    MXC_CTB_Cipher_SetKeySource(MXC_CTB_CIPHER_KEY_SOFTWARE);

    // Load key into cipher key register
    MXC_CTB_Cipher_SetKey((uint8_t *)key, MXC_DES_DATA_LEN);

    if (asynchronous) {
        wait = 1;
        MXC_CTB_Cipher_DecryptAsync(&cipher_req);

        while (wait) {}
    } else {
        MXC_CTB_Cipher_Decrypt(&cipher_req);
    }

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    return DES_check(result, expected, MXC_DES_DATA_LEN);
}

// *****************************************************************************
int main(void)
{
    printf("\n***** DES Example *****\n");

    int fail = 0;

    printf("DES ECB Encryption ... ");
    fail += DES_ECB_enc(0);
    fail += DES_ECB_enc(1);
    printf("DES ECB Decryption ... ");
    fail += DES_ECB_dec(0);
    fail += DES_ECB_dec(1);
    printf("Triple DES ECB Encryption ... ");
    fail += TDES_ECB_enc(0);
    fail += TDES_ECB_enc(1);
    printf("Triple DES ECB Decryption ... ");
    fail += TDES_ECB_dec(0);
    fail += TDES_ECB_dec(1);

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
