/**
 * @file        main.c
 * @brief       DES Example
 * @details     Encryption and decryption of DES on different modes (ECB) with
 *              different bit sizes (128, 192, and 256)
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
#include <string.h>
#include "mxc_errors.h"
#include "tpu.h"
#include "tpu_regs.h"

/* **** Definitions **** */
#define MXC_DES_DATA_LEN \
    (64 /                \
     8) /**< Number of bytes in an DES plaintext or cyphertext block, which are always 64-bits long. */
#define MXC_DES_KEY_LEN (64 / 8) /**< Number of bytes in a TDES key. */
#define MXC_TDES_KEY_LEN (192 / 8) /**< Number of bytes in a TDES key. */

/* **** Globals **** */
char result[512];

/* **** Functions **** */
/* Convert ascii to byte */
void ascii_to_byte(const char *src, char *dst, int len)
{
    int i;
    int val;
    char temp[3];

    temp[2] = 0x00;
    for (i = 0; i < len; ++i) {
        temp[0] = *src;
        src++;
        temp[1] = *src;
        src++;
        sscanf(temp, "%0x", &val);
        dst[i] = val;
    }

    return;
}

/* Verify by comparing calculated to expected */
void DES_check(char *calculated, char *expected, int len)
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
    }

    return;
}

void DES_ECB_enc(void)
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

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_DES);
    MXC_TPU_Cipher_DES_Encrypt(msg, iv_dst, key, MXC_TPU_MODE_ECB, MXC_DES_DATA_LEN, result);

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    DES_check(result, expected, MXC_DES_DATA_LEN);
    return;
}

void DES_ECB_dec(void)
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

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_DES);
    MXC_TPU_Cipher_DES_Decrypt(ct, iv_dst, key, MXC_TPU_MODE_ECB, MXC_DES_DATA_LEN, result);

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    DES_check(result, expected, MXC_DES_DATA_LEN);
    return;
}

void TDES_ECB_enc(void)
{
    char *xkey = "0fb5b906471296bc1ab269585e1c99dcf10dd7b047cdee29";
    char key[MXC_TDES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xpt = "0000000000000000";
    char pt[MXC_DES_DATA_LEN];
    char *xexpected = "d05ef547adf0db98";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_TDES_KEY_LEN);

    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);

    ascii_to_byte(xpt, pt, MXC_DES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_TDEA);
    MXC_TPU_Cipher_TDES_Encrypt(pt, iv_dst, key, MXC_TPU_MODE_ECB, MXC_DES_DATA_LEN, result);

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    DES_check(result, expected, MXC_DES_DATA_LEN);
    return;
}

void TDES_ECB_dec(void)
{
    char *xkey = "2e0a67fe76bc3d3c1081c45a48784f49c876033acc85f69c";
    char key[MXC_TDES_KEY_LEN];
    char *iv_src = "";
    char iv_dst[MXC_DES_DATA_LEN];
    char *xct = "2a78627595b42376";
    char ct[MXC_DES_DATA_LEN];
    char *xexpected = "0000000000000000";
    char expected[MXC_DES_DATA_LEN];

    ascii_to_byte(xkey, key, MXC_TDES_KEY_LEN);
    ascii_to_byte(iv_src, iv_dst, MXC_DES_DATA_LEN);
    ascii_to_byte(xct, ct, MXC_DES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_TDEA);
    MXC_TPU_Cipher_TDES_Decrypt(ct, iv_dst, key, MXC_TPU_MODE_ECB, MXC_DES_DATA_LEN, result);

    ascii_to_byte(xexpected, expected, MXC_DES_DATA_LEN);

    DES_check(result, expected, MXC_DES_DATA_LEN);
    return;
}

// *****************************************************************************
int main(void)
{
    printf("\n***** DES Example *****\n");

    printf("DES ECB Encryption ... ");
    DES_ECB_enc();
    printf("DES ECB Decryption ... ");
    DES_ECB_dec();
    printf("Triple DES ECB Encryption ... ");
    TDES_ECB_enc();
    printf("Triple DES ECB Decryption ... ");
    TDES_ECB_dec();

    printf("\nExample complete.\n");
    while (1) {}
}
