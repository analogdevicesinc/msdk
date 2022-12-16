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
 * @file    	main.c
 * @brief   	AES Example
 * @details 	Encryption and decryption of AES on different modes (ECB and OFB) with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "board.h"
#include "tpu.h"

/***** Definitions *****/

/***** Globals *****/
char temp[] = { 0x00, 0x00, 0x00 }; //for ascii_to_byte()
char result[512];

/***** Functions *****/
//Print human-friendly readable output of array
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
        sscanf(temp, "%x", &val);
        //val = hexconf(temp[0])*16 + hexconf(temp[1]);
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
            printf("calc:%x  Expect : %x\n", calculated[i], expected[i]);
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

int AES128_ECB_enc(void)
{
    printf("128-bit AES ECB Encryption ... ");
    const char *_key = "10a58869d74be5a374cf867cfb473859";
    char key[MXC_AES_KEY_128_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_128_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_msg = "00000000000000000000000000000000";
    char msg[MXC_AES_DATA_LEN];
    ascii_to_byte(_msg, msg, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES128);
    MXC_TPU_Cipher_AES_Encrypt(msg, iv_dst, key, MXC_TPU_CIPHER_AES128, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

    const char *_expected = "6d251e6944b051e04eaa6fb4dbf78465";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES128_ECB_dec(void)
{
    printf("128-bit AES ECB Decryption ... ");
    const char *_key = "caea65cdbb75e9169ecd22ebe6e54675";
    char key[MXC_AES_KEY_128_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_128_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_ct = "6e29201190152df4ee058139def610bb";
    char ct[MXC_AES_DATA_LEN];
    ascii_to_byte(_ct, ct, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES128);
    MXC_TPU_Cipher_AES_Decrypt(ct, iv_dst, key, MXC_TPU_CIPHER_AES128, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

    const char *_expected = "00000000000000000000000000000000";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES192_ECB_enc(void)
{
    printf("192-bit AES ECB Encryption ... ");
    const char *_key = "ffffffffffffffffffffffffffffffffffffffffffffc000";
    char key[MXC_AES_KEY_192_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_192_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_pt = "00000000000000000000000000000000";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES192);
    MXC_TPU_Cipher_AES_Encrypt(pt, iv_dst, key, MXC_TPU_CIPHER_AES192, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

    const char *_expected = "ace4b91c9c669e77e7acacd19859ed49";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES192_ECB_dec(void)
{
    printf("192-bit AES ECB Decryption ... ");
    const char *_key = "ffffffffffffffffffffffffffffffffffffffffe0000000";
    char key[MXC_AES_KEY_192_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_192_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_ct = "fb4bc78b225070773f04c40466d4e90c";
    char ct[MXC_AES_DATA_LEN];
    ascii_to_byte(_ct, ct, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES192);
    MXC_TPU_Cipher_AES_Decrypt(ct, iv_dst, key, MXC_TPU_CIPHER_AES192, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

    const char *_expected = "00000000000000000000000000000000";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES256_ECB_enc(void)
{
    printf("256-bit AES ECB Encryption ... ");
    const char *_key = "797f8b3d176dac5b7e34a2d539c4ef367a16f8635f6264737591c5c07bf57a3e";
    char key[MXC_AES_KEY_256_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_256_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_pt = "00000000000000000000000000000000";
    char pt[MXC_AES_DATA_LEN];
    ascii_to_byte(_pt, pt, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES256);
    MXC_TPU_Cipher_AES_Encrypt(pt, iv_dst, key, MXC_TPU_CIPHER_AES256, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

    const char *_expected = "a74289fe73a4c123ca189ea1e1b49ad5";
    char expected[MXC_AES_DATA_LEN];
    ascii_to_byte(_expected, expected, MXC_AES_DATA_LEN);

    return AES_check(result, expected, MXC_AES_DATA_LEN);
}

int AES256_ECB_dec(void)
{
    printf("256-bit AES ECB Decryption ... ");
    const char *_key = "13428b5e4c005e0636dd338405d173ab135dec2a25c22c5df0722d69dcc43887";
    char key[MXC_AES_KEY_256_LEN];
    ascii_to_byte(_key, key, MXC_AES_KEY_256_LEN);

    const char *iv_src = "";
    char iv_dst[16];
    ascii_to_byte(iv_src, iv_dst, 16);

    const char *_ct = "649a71545378c783e368c9ade7114f6c";
    char ct[MXC_AES_DATA_LEN];
    ascii_to_byte(_ct, ct, MXC_AES_DATA_LEN);

    MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_AES256);
    MXC_TPU_Cipher_AES_Decrypt(ct, iv_dst, key, MXC_TPU_CIPHER_AES256, MXC_TPU_MODE_ECB,
                               MXC_AES_DATA_LEN, result);

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
    fail += AES128_ECB_enc();
    fail += AES128_ECB_dec();
    fail += AES192_ECB_enc();
    fail += AES192_ECB_dec();
    fail += AES256_ECB_enc();
    fail += AES256_ECB_dec();

    printf("\n");
    if (fail == 0) {
        printf("Example Succeeded\n");
    } else {
        printf("Example Failed\n");
        return E_FAIL;
    }

    return E_NO_ERROR;
}
