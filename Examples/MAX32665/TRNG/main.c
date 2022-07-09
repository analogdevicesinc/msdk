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
 * @brief   	TRNG Example
 * @note        Generates random 32-bit number for first part
 *              followed by an application (AES) with TRNG
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "board.h"
#include "max32665.h"
#include "tpu.h"

/***** Definitions *****/
#define TRNG_32BIT_RND_NO	4
#define LEN			        16 //User specified length for random number

#define MXC_AES_DATA_LEN    (128 / 8)
#define MXC_AES_KEY_128_LEN (128 / 8)

/***** Globals *****/
unsigned int rnd_no[TRNG_32BIT_RND_NO] = {0};
uint8_t var_rnd_no[LEN] = {0};

char aes_result[512];
char temp[] = {0x00, 0x00, 0x00};


/***** Functions *****/
void ascii_to_byte(const char *src, char *dst, int len)
{
	int i;
	for(i=0; i < len; ++i){
		int val;
		temp[0] = *src; src++;
		temp[1] = *src; src++;
		sscanf(temp, "%0x", &val);
		dst[i] = val;
	}
}

void print(char *stuff)
{
	int i, j, size = 4;
	for(i = 0; i < 4; ++i){
		for(j = 0; j < 4; ++j){
			printf("0x%x ", stuff[i*size+j]);
		}
		printf("\n");
	}
	return;
}

// *****************************************************************************
int main(void)
{
    int i;
	printf("***** TRNG Example *****\n");

	MXC_TPU_Init(MXC_SYS_PERIPH_CLOCK_TRNG);

    //Reading and printing rnd 32-bit number
	for(i = 0; i < TRNG_32BIT_RND_NO; ++i){
		rnd_no[i] = MXC_TPU_TRNG_Read32BIT(MXC_TRNG);
		printf("%0x\n", rnd_no[i]);
	}

	printf("\n");

    //Reading a user-specified length of random
	MXC_TPU_TRNG_Read(MXC_TRNG, var_rnd_no, LEN);

	//AES Application using random IV from TRNG peripheral
	printf("AES Application\n");
	const char *_key = "10a58869d74be5a374cf867cfb473859";
	char key[MXC_AES_KEY_128_LEN];
	ascii_to_byte(_key, key, MXC_AES_KEY_128_LEN);

	char iv[MXC_AES_KEY_128_LEN];
	for(i = 0; i < LEN; ++i){
		iv[i] = var_rnd_no[i];
	}

	const char *_msg = "00000000000000000000000000000000";
	char msg[MXC_AES_DATA_LEN];
	ascii_to_byte(_msg, msg, MXC_AES_DATA_LEN);

	char aes_result[512];

	printf("Key:\n");
	print(key);

	printf("\nInitial Value:\n");
	print(iv);

	printf("\nPlain Text:\n");
	print(msg);

	printf("\nEncrypting using AES and CFB Mode\n");
	MXC_TPU_Cipher_Config(MXC_TPU_MODE_CFB, MXC_TPU_CIPHER_AES128);
	MXC_TPU_Cipher_AES_Encrypt(msg, iv, key, MXC_TPU_CIPHER_AES128, MXC_TPU_MODE_CFB, MXC_AES_DATA_LEN, aes_result);

	printf("\nCipher Text:\n");
	print(aes_result);

	printf("\nExample complete.\n");
    
    return 0;
}
