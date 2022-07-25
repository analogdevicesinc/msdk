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
 * @brief   	DES Example
 * @details 	Encryption and decryption of AES on different modes (ECB and OFB) with different bit sizes (128, 192, and 256)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "board.h"
#include "tpu.h"

void printArray(char* arr, int len)
{
    for (int i = 0; i < len; i++) {
        printf("%02X ", arr[i]);
        if ((i % 8) == 7) {
            printf("\n");
        }
    }
    if ((len % 8) != 0) {
        printf("\n");
    }
    printf("\n");
}

// Compares the contents of two arrays and return 0 if they match, 1 if they do not.
int compareArrays(char* a, char* b, int len)
{
    if (memcmp(a, b, len) == 0) {
        printf("Data verified.\n\n");
        return 0;
    } else {
        printf("Data MISMATCH!\n\n");
        printf("result:\n");
        printArray(a, 48);
        printf("expected:\n");
        printArray(b, 48);
        printf("\n");
        return 1;
    }
}

// Adds trailing 0's to the array to pad the message to a multiple of the DES block size of 16 bytes.
int padMessage(char* arr, int len)
{
    int i;
    for (i = len; i < ((len + 15) & 0xFFFFFFF0); i++) { arr[i] = 0; }
    return i;
}

// Performs a DES encryption using electronic code book and compares the result with the expected value.
// Returns 1 for any failure, 0 for success.
int DES_ECB_Encryption_Example(void)
{
    int err;
    int datalen;

    char plaintext[48] = "This is an example of DES encryption with ECB.";
    char iv[]          = {0, 0, 0, 0, 0, 0, 0, 0}; // Not used for ECB.
    char key[]         = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9};
    char ciphertext[48];
    char expected[48] = {0x2d, 0x72, 0xe6, 0x07, 0xad, 0x2d, 0xdd, 0x84, 0x2a, 0x63, 0xe9, 0x5b,
                         0x83, 0x56, 0x9b, 0x54, 0x10, 0xae, 0xa0, 0x28, 0x49, 0xa6, 0x72, 0x36,
                         0x9f, 0x6a, 0x5a, 0xf9, 0xbe, 0x11, 0x81, 0x41, 0x20, 0xc7, 0x1a, 0x45,
                         0x81, 0xd1, 0xa8, 0xa3, 0x50, 0x09, 0xff, 0x0c, 0x30, 0x39, 0x93, 0x08};

    printf("Demonstrating DES ECB (electronic code book) encryption...\n");

    // Pad the input message to a multiple of the DES block size.
    datalen = padMessage(plaintext, strlen(plaintext));

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_DES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_ECB, datalen, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a DES encryption using cipher block chaining and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int DES_CBC_Encryption_Example(void)
{
    int err;
    int datalen;

    char plaintext[48] = "This is an example of DES encryption with CBC.";
    char iv[]          = {0x6d, 0x8a, 0x3b, 0xee, 0x5f, 0x8c, 0x9a, 0xfd};
    char key[]         = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9};
    char ciphertext[48];
    char expected[48] = {0x59, 0x59, 0x5b, 0x29, 0x9e, 0x00, 0x7e, 0xf8, 0xa9, 0x73, 0xd4, 0xbb,
                         0x72, 0x35, 0x22, 0x5e, 0xa3, 0xb8, 0x5f, 0x27, 0xd7, 0x2e, 0xb6, 0xbf,
                         0xc3, 0x7c, 0x12, 0x80, 0xe5, 0xef, 0x68, 0xdd, 0xab, 0xef, 0x29, 0xf0,
                         0x5a, 0x26, 0x61, 0x31, 0x1a, 0x25, 0x51, 0x9c, 0xe0, 0xc9, 0xad, 0x17};

    printf("Demonstrating DES CBC (cipher block chaining) encryption...\n");

    // Pad the input message to a multiple of the DES block size.
    datalen = padMessage(plaintext, strlen(plaintext));

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CBC, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_DES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_CBC, datalen, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a DES encryption using cipher feedback mode and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int DES_CFB_Encryption_Example(void)
{
    int err;

    char plaintext[48] = {0xb3, 0x48, 0xf0, 0x70, 0x78, 0xfa, 0x15, 0x8e, 0xb4, 0xc3, 0x7e, 0x47,
                          0xee, 0x20, 0x84, 0x1d, 0xf6, 0xb2, 0x16, 0xd4, 0xc8, 0x91, 0xb9, 0x6e,
                          0xb3, 0xe2, 0xcc, 0x5a, 0xbb, 0xac, 0x11, 0xe9, 0x3b, 0xe7, 0x69, 0x01,
                          0xcf, 0xd6, 0xdf, 0x15, 0x70, 0x97, 0xe8, 0x54, 0xe1, 0xf9, 0xe9, 0x21};
    char iv[]          = {0x43, 0xc8, 0x41, 0xef, 0x07, 0x04, 0x30, 0x73};
    char key[]         = {0x86, 0xd0, 0xcd, 0x04, 0xc4, 0x76, 0xf7, 0x5d};
    char ciphertext[48];
    char expected[48] = {0x9d, 0x6e, 0x21, 0x39, 0x52, 0x91, 0x65, 0x67, 0x00, 0xc8, 0x6f, 0x0b,
                         0x0a, 0x58, 0xca, 0xaf, 0x0f, 0xc1, 0xdc, 0xc0, 0x3f, 0x02, 0x55, 0x2a,
                         0x8f, 0x0f, 0x90, 0x48, 0x2a, 0x99, 0x42, 0x30, 0x15, 0x68, 0x05, 0x38,
                         0xf8, 0xe7, 0x7e, 0x18, 0xba, 0x75, 0xc9, 0xfd, 0x0f, 0xfb, 0x91, 0xd6};

    printf("Demonstrating DES CFB (cipher feedback) encryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CFB, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_DES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_CFB, 48, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a DES decryption using electronic code book and compares the result with the expected value.
// Returns 1 for any failure, 0 for success.
int DES_ECB_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0x2d, 0x72, 0xe6, 0x07, 0xad, 0x2d, 0xdd, 0x84, 0x2a, 0x63, 0xe9, 0x5b,
                           0x83, 0x56, 0x9b, 0x54, 0x10, 0xae, 0xa0, 0x28, 0x49, 0xa6, 0x72, 0x36,
                           0xfb, 0xaf, 0x7d, 0xd8, 0xf4, 0x96, 0x06, 0xac, 0x20, 0xc7, 0x1a, 0x45,
                           0x81, 0xd1, 0xa8, 0xa3, 0x50, 0x09, 0xff, 0x0c, 0x30, 0x39, 0x93, 0x08};
    char iv[]           = {0, 0, 0, 0, 0, 0, 0, 0}; // Not used for ECB.
    char key[]          = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9};
    char plaintext[48];
    char expected[48] = "This is an example of DES decryption with ECB.";

    printf("Demonstrating DES ECB (electronic code book) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the Decryption.
    err = MXC_TPU_Cipher_DES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_ECB, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

// Performs a DES decryption using cipher block chaining and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int DES_CBC_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0x59, 0x59, 0x5b, 0x29, 0x9e, 0x00, 0x7e, 0xf8, 0xa9, 0x73, 0xd4, 0xbb,
                           0x72, 0x35, 0x22, 0x5e, 0xa3, 0xb8, 0x5f, 0x27, 0xd7, 0x2e, 0xb6, 0xbf,
                           0x23, 0x50, 0x06, 0xce, 0x72, 0x42, 0xe2, 0x0d, 0xc8, 0x5c, 0x4d, 0x0f,
                           0x83, 0x35, 0xc3, 0xe1, 0xb1, 0x5a, 0xff, 0x5b, 0x79, 0x50, 0x5e, 0x3b};
    char iv[]           = {0x6d, 0x8a, 0x3b, 0xee, 0x5f, 0x8c, 0x9a, 0xfd};
    char key[]          = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9};
    char plaintext[48];
    char expected[48] = "This is an example of DES decryption with CBC.";

    printf("Demonstrating DES CBC (cipher block chaining) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CBC, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the decryption.
    err = MXC_TPU_Cipher_DES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_CBC, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

// Performs a DES decryption using cipher feedback and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int DES_CFB_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0xe5, 0xda, 0x7b, 0xcb, 0x32, 0x25, 0x47, 0x9b, 0x65, 0xe8, 0x26, 0xc7,
                           0xc0, 0x72, 0xfa, 0x3b, 0xd8, 0x11, 0xa4, 0xe1, 0x3c, 0x2b, 0xca, 0x59,
                           0x57, 0x8a, 0xd7, 0xf0, 0x98, 0xf8, 0x43, 0xd0, 0xd0, 0x0c, 0x14, 0x74,
                           0xdb, 0x43, 0xa5, 0x5f, 0x82, 0x52, 0xa6, 0x0e, 0xa0, 0xbb, 0x64, 0x2b};
    char iv[]           = {0x39, 0x34, 0x82, 0x47, 0x9b, 0xac, 0x81, 0x58};
    char key[]          = {0xd3, 0x01, 0x38, 0x3b, 0x5d, 0xe9, 0x26, 0xb0};
    char plaintext[48];
    char expected[48] = {0x51, 0xab, 0x20, 0x5a, 0x26, 0x53, 0xbf, 0xbd, 0x0c, 0x55, 0x91, 0xcb,
                         0x07, 0x0a, 0x86, 0x4e, 0xa9, 0x9e, 0x8c, 0x65, 0x0a, 0xac, 0x1b, 0xe8,
                         0xdf, 0x2f, 0x91, 0xbc, 0xb4, 0x3a, 0xee, 0x4c, 0xa1, 0x89, 0x8c, 0x6a,
                         0xc3, 0x76, 0x5d, 0xe8, 0xe8, 0x33, 0x1b, 0x5b, 0x61, 0xe3, 0xef, 0x17};

    printf("Demonstrating DES CFB (cipher feedback) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CFB, MXC_TPU_CIPHER_DES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the decryption.
    err = MXC_TPU_Cipher_DES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_CFB, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

// Performs a Triple DES encryption using electronic code book and compares the result with the expected value.
// Returns 1 for any failure, 0 for success.
int TDES_ECB_Encryption_Example(void)
{
    int err;
    int datalen;

    char plaintext[48] = "This is an example of 3DES encryption with ECB.";
    char iv[]          = {0, 0, 0, 0, 0, 0, 0, 0}; // Not used for ECB.
    char key[]         = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9, 0xcf, 0xd0, 0x5b, 0x2c,
                  0x45, 0x85, 0x7f, 0xdb, 0x8d, 0x37, 0x23, 0x3c, 0x15, 0x05, 0xef, 0x53};
    char ciphertext[48];
    char expected[48] = {0xc6, 0xff, 0xca, 0xe0, 0x0a, 0xfc, 0x7c, 0xcf, 0xde, 0x41, 0x80, 0x13,
                         0xca, 0xe9, 0x60, 0x12, 0xe4, 0xf1, 0xf0, 0x98, 0xd1, 0x02, 0x7a, 0x8c,
                         0x93, 0x50, 0x0a, 0x7e, 0x58, 0xbc, 0x74, 0x0c, 0xfa, 0xf3, 0x76, 0x88,
                         0xc9, 0xf8, 0xb0, 0x15, 0x11, 0xde, 0x87, 0x93, 0x04, 0x56, 0x43, 0x82};

    printf("Demonstrating 3DES ECB (electronic code book) encryption...\n");

    // Pad the input message to a multiple of the DES block size.
    datalen = padMessage(plaintext, strlen(plaintext));

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_TDES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_ECB, datalen, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a Triple DES encryption using cipher block chaining and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int TDES_CBC_Encryption_Example(void)
{
    int err;
    int datalen;

    char plaintext[48] = "This is an example of 3DES encryption with CBC.";
    char iv[]          = {0x6d, 0x8a, 0x3b, 0xee, 0x5f, 0x8c, 0x9a, 0xfd};
    char key[]         = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9, 0xcf, 0xd0, 0x5b, 0x2c,
                  0x45, 0x85, 0x7f, 0xdb, 0x8d, 0x37, 0x23, 0x3c, 0x15, 0x05, 0xef, 0x53};
    char ciphertext[48];
    char expected[48] = {0xd8, 0xc9, 0x1e, 0x29, 0xda, 0xa6, 0xa7, 0x14, 0x20, 0x89, 0x7b, 0x20,
                         0x0b, 0xaf, 0x50, 0x60, 0x84, 0x6a, 0x61, 0x0d, 0xc5, 0xff, 0x47, 0x2c,
                         0x6e, 0x63, 0x06, 0x67, 0x45, 0x1e, 0xa7, 0x97, 0x5a, 0x2b, 0x47, 0xe7,
                         0x42, 0xb4, 0xb2, 0x07, 0x40, 0xf9, 0xf9, 0xda, 0x9f, 0x7e, 0x72, 0xd7};

    printf("Demonstrating 3DES CBC (cipher block chaining) encryption...\n");

    // Pad the input message to a multiple of the DES block size.
    datalen = padMessage(plaintext, strlen(plaintext));

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CBC, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_TDES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_CBC, datalen, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a Triple DES encryption using cipher feedback and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int TDES_CFB_Encryption_Example(void)
{
    int err;

    char plaintext[48] = {0xe0, 0x3a, 0xab, 0x86, 0x5e, 0xcd, 0x0a, 0x63, 0x19, 0xe5, 0x22, 0x3b,
                          0x74, 0x6f, 0x00, 0x76, 0x87, 0x28, 0x35, 0xf0, 0x0c, 0xfb, 0xd9, 0xf2,
                          0x7c, 0x8a, 0xcf, 0x62, 0x75, 0x0b, 0x4b, 0x36, 0x5f, 0x55, 0x92, 0x2e,
                          0x4f, 0x3a, 0x01, 0xc3, 0x2b, 0x18, 0x51, 0x5b, 0x10, 0xab, 0x19, 0xbe};
    char iv[]          = {0x18, 0xe7, 0xa2, 0xac, 0xcc, 0xe2, 0xaf, 0xc6};
    char key[]         = {0xea, 0xd6, 0xfe, 0xf8, 0x29, 0xec, 0x94, 0xec, 0x58, 0xc2, 0x15, 0x9e,
                  0xf8, 0xad, 0x37, 0xd9, 0xa4, 0x0b, 0x89, 0xea, 0x61, 0xa1, 0xb5, 0xec};
    char ciphertext[48];
    char expected[48] = {0x45, 0xe4, 0xdd, 0xe2, 0xcb, 0x0f, 0x96, 0x31, 0x26, 0x6c, 0x72, 0xf8,
                         0xa6, 0x35, 0x0e, 0x2b, 0x20, 0xd0, 0x62, 0xa4, 0xf0, 0x9a, 0x3d, 0x90,
                         0xc3, 0xb8, 0x92, 0xf4, 0x32, 0xdc, 0xbe, 0x36, 0x6a, 0x22, 0xa8, 0x3d,
                         0xb8, 0x48, 0x89, 0x66, 0x20, 0xcf, 0x1b, 0x84, 0x64, 0x37, 0xce, 0x87};

    printf("Demonstrating 3DES CFB (cipher feedback) encryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CFB, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the encryption.
    err = MXC_TPU_Cipher_TDES_Encrypt(plaintext, iv, key, MXC_TPU_MODE_CFB, 48, ciphertext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete encryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(ciphertext, expected, 48);
}

// Performs a Triple DES decryption using electronic code book and compares the result with the expected value.
// Returns 1 for any failure, 0 for success.
int TDES_ECB_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0xc6, 0xff, 0xca, 0xe0, 0x0a, 0xfc, 0x7c, 0xcf, 0xde, 0x41, 0x80, 0x13,
                           0xca, 0xe9, 0x60, 0x12, 0xe4, 0xf1, 0xf0, 0x98, 0xd1, 0x02, 0x7a, 0x8c,
                           0x44, 0x91, 0x08, 0x70, 0xb2, 0xd7, 0x6d, 0xf0, 0xfa, 0xf3, 0x76, 0x88,
                           0xc9, 0xf8, 0xb0, 0x15, 0x11, 0xde, 0x87, 0x93, 0x04, 0x56, 0x43, 0x82};
    char iv[]           = {0, 0, 0, 0, 0, 0, 0, 0}; // Not used for ECB.
    char key[]          = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9, 0xcf, 0xd0, 0x5b, 0x2c,
                  0x45, 0x85, 0x7f, 0xdb, 0x8d, 0x37, 0x23, 0x3c, 0x15, 0x05, 0xef, 0x53};
    char plaintext[48];
    char expected[48] = "This is an example of 3DES decryption with ECB.";

    printf("Demonstrating 3DES ECB (electronic code book) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_ECB, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the Decryption.
    err = MXC_TPU_Cipher_TDES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_ECB, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

// Performs a Triple DES decryption using cipher block chaining and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int TDES_CBC_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0xd8, 0xc9, 0x1e, 0x29, 0xda, 0xa6, 0xa7, 0x14, 0x20, 0x89, 0x7b, 0x20,
                           0x0b, 0xaf, 0x50, 0x60, 0x84, 0x6a, 0x61, 0x0d, 0xc5, 0xff, 0x47, 0x2c,
                           0x0a, 0xaf, 0x9a, 0x2e, 0x5b, 0x69, 0x07, 0x4e, 0x1f, 0x54, 0x8e, 0x55,
                           0x74, 0xd3, 0xd8, 0x48, 0x61, 0x06, 0x61, 0xa3, 0x6c, 0x4b, 0x5e, 0xbf};
    char iv[]           = {0x6d, 0x8a, 0x3b, 0xee, 0x5f, 0x8c, 0x9a, 0xfd};
    char key[]          = {0x3c, 0xc3, 0x09, 0x9d, 0xd1, 0x46, 0xdb, 0xe9, 0xcf, 0xd0, 0x5b, 0x2c,
                  0x45, 0x85, 0x7f, 0xdb, 0x8d, 0x37, 0x23, 0x3c, 0x15, 0x05, 0xef, 0x53};
    char plaintext[48];
    char expected[48] = "This is an example of 3DES decryption with CBC.";

    printf("Demonstrating 3DES CBC (cipher block chaining) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CBC, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the decryption.
    err = MXC_TPU_Cipher_TDES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_CBC, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

// Performs a Triple DES decryption using cipher feedback and compares the result with the expected value.
// Returns 0 for any failure, non-zero for success.
int TDES_CFB_Decryption_Example(void)
{
    int err;

    char ciphertext[48] = {0xf5, 0xee, 0x26, 0x02, 0x16, 0x50, 0xef, 0x21, 0x8f, 0xa4, 0xf4, 0x4a,
                           0xa5, 0xeb, 0xc5, 0x22, 0x7d, 0x86, 0xd3, 0xa5, 0x30, 0xf5, 0x24, 0x0d,
                           0x21, 0xe5, 0x4b, 0xf6, 0x8f, 0x6a, 0x98, 0xbf, 0x66, 0x89, 0x8c, 0x33,
                           0x5b, 0xf9, 0x8f, 0x69, 0x37, 0x2a, 0xe5, 0xe8, 0x73, 0x87, 0xdf, 0x0f};
    char iv[]           = {0x96, 0x89, 0xbc, 0x68, 0x4a, 0xe8, 0x6c, 0x32};
    char key[]          = {0x07, 0xa4, 0xe5, 0x45, 0x6e, 0x07, 0x2c, 0x61, 0x68, 0x26, 0xf8, 0x91,
                  0x07, 0x9d, 0xda, 0x4c, 0x0e, 0x57, 0xd0, 0x58, 0x13, 0x62, 0x9d, 0x61};
    char plaintext[48];
    char expected[48] = {0x1a, 0x10, 0x60, 0x58, 0xcb, 0xf5, 0x10, 0xba, 0xd9, 0x99, 0xcb, 0xbb,
                         0x34, 0x0b, 0x6d, 0x3a, 0x53, 0x59, 0x80, 0x9f, 0x80, 0x12, 0xe8, 0x92,
                         0xb3, 0x84, 0x11, 0xa4, 0x16, 0x26, 0x21, 0xbc, 0x31, 0x71, 0xc4, 0xc3,
                         0x6a, 0xf6, 0xf4, 0x3c, 0x5a, 0x55, 0x96, 0xa9, 0x22, 0x7b, 0xc1, 0xa6};

    printf("Demonstrating 3DES CBC (cipher feedback) decryption...\n");

    // Prepare Crypto Engine for a DES operation.
    err = MXC_TPU_Cipher_Config(MXC_TPU_MODE_CFB, MXC_TPU_CIPHER_TDES);
    if (err != E_NO_ERROR) {
        printf("Failed to configure crypto module (code = %d).\n\n", err);
        return 1;
    }

    // Do the decryption.
    err = MXC_TPU_Cipher_TDES_Decrypt(ciphertext, iv, key, MXC_TPU_MODE_CFB, 48, plaintext);
    if (err != E_NO_ERROR) {
        printf("Failed to complete decryption (code = %d).\n\n", err);
        return 1;
    }

    // Verify the cipher text matches what we expect.
    return compareArrays(plaintext, expected, 48);
}

int main(void)
{
    int errCnt = 0;

    printf("\n***** DES Example *****\n\n");

    errCnt += DES_ECB_Encryption_Example();
    errCnt += DES_CBC_Encryption_Example();
    errCnt += DES_CFB_Encryption_Example();

    errCnt += DES_ECB_Decryption_Example();
    errCnt += DES_CBC_Decryption_Example();
    errCnt += DES_CFB_Decryption_Example();

    errCnt += TDES_ECB_Encryption_Example();
    errCnt += TDES_CBC_Encryption_Example();
    errCnt += TDES_CFB_Encryption_Example();

    errCnt += TDES_ECB_Decryption_Example();
    errCnt += TDES_CBC_Decryption_Example();
    errCnt += TDES_CFB_Decryption_Example();

    if (errCnt == 0) {
        printf("Example Complete.\n");
    } else {
        printf("%d Operations FAILED!\n", errCnt);
    }
}
