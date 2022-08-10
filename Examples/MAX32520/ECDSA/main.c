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
 * @file    main.c
 * @brief   ECDSA using Free UCL
 * @details This examples uses Free UCL in a few different ways and times how long it takes.
 */

// #define DO_KEYGEN_TEST
// Define SEPARATE_HASH to calculate SHA hash outside of ECDSA sign/verify function.
#define SEPARATE_HASH
// Use Timer 0 for timed operations
#define UCL_TEST_TIMER MXC_TMR0

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "tmr.h"
#include "icc.h"
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_rng.h>
#include <ucl/ucl_aes.h>
#include <ucl/ucl_aes_cbc.h>
#include <ucl/ucl_aes_ecb.h>

// Hash related
#include <ucl/ucl_sha256.h>

// ECDSA related
// #define WORD32
#include <ucl/ecdsa_generic_api.h>
#ifdef DO_KEYGEN_TEST
#include <ucl/ucl_ecc_keygen.h>
#endif // DO_KEYGEN_TEST

/***** Definitions *****/

/***** Globals *****/
mxc_tmr_cfg_t tmr;

/***** Functions *****/

__attribute__((section(".bss"), aligned(4))) uint32_t ucl_work_buffer[1024];
//__attribute__((section(".bss"),aligned(4))) ucl_sha256_ctx_t sha256_context;
extern ucl_type_curve secp256r1;

void hexdump(uint8_t* data, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        if (((i & 0x0F) == 0) && (i != 0)) {
            printf("\n");
        }

        printf("%02X ", data[i]);
    }

    printf("\n");
}

void c_hexdump(uint8_t* data, int length)
{
    int i;

    printf("uint8_t arr[] = {\n");

    for (i = 0; i < length; i++) {
        if (((i & 0x0F) == 0) && (i != 0)) {
            printf("\n");
        }

        printf("0x%02X, ", data[i]);
    }

    printf("};\n");
}

uint8_t demo_d[] = {
    0xD5, 0x7D, 0x1B, 0xD2, 0xDF, 0x0F, 0x18, 0xA7, 0x74, 0x24, 0xE6, 0x6F, 0x58, 0xC4, 0x2B, 0x7B,
    0x58, 0xC7, 0x66, 0xA6, 0xC6, 0x65, 0x7B, 0xCD, 0x2B, 0xC6, 0x32, 0xE4, 0x09, 0x7A, 0x90, 0x85,
};

uint8_t demo_x[] = {
    0xBC, 0x57, 0x10, 0xAA, 0xB4, 0xE6, 0x15, 0x5B, 0x8B, 0x31, 0xF1, 0xE5, 0x20, 0x60, 0x4F, 0x4A,
    0x93, 0x1A, 0x16, 0xD0, 0xE8, 0x51, 0x21, 0xAC, 0xDB, 0x08, 0xA1, 0x37, 0xDF, 0xA5, 0x56, 0xF9,
};

uint8_t demo_y[] = {
    0x78, 0x22, 0x97, 0x83, 0x37, 0x2A, 0x95, 0xFE, 0x62, 0x7E, 0x55, 0x6E, 0x99, 0x86, 0x2E, 0x1E,
    0x2C, 0x0A, 0x90, 0x25, 0xB5, 0x08, 0xE9, 0x57, 0x08, 0x9F, 0x6E, 0x3A, 0x56, 0x68, 0xDF, 0x73,
};

// Storage for demo key
ucl_type_ecc_u8_affine_point demo_q = {.x = demo_x, .y = demo_y};

int ecdsa_test(void)
{
    int i;
    int retval;
    uint32_t start, stop;

    // Set ECC Curve
    ucl_type_curve* curve_params = &secp256r1;

    // Setup signature storage
    u8 r[SECP256R1_BYTESIZE];
    u8 s[SECP256R1_BYTESIZE];
    ucl_type_ecdsa_signature signature = {.r = r, .s = s};

#ifdef SEPARATE_HASH
    // Storage for SHA context and output.
    ucl_sha256_ctx_t sha256_context;
    uint8_t hash256[32];
#endif // SEPARATE_HASH

    // Set hash function
    int (*hash_function_ptr)(u8*, u8*, u32) = &ucl_sha256;

    // ECDSA function configuration flags
    int configuration;

    // Set up message to sign/verify.
    uint8_t message[]      = {0, 1, 2, 3, 4, 5, 6};
    uint8_t message_length = sizeof(message);

    printf("ECDSA Sign\n");

    // Sign message
#ifdef SEPARATE_HASH
    // Initialize SHA256 working area.
    retval = ucl_sha256_init(&sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    // Slow hash one byte at a time.
    for (i = 0; i < message_length; i++) {
        retval = ucl_sha256_core(&sha256_context, message + i, 1);

        if (UCL_OK != retval) {
            while (1)
                ;
        }
    }

    // Finish hash.
    memset(hash256, 0, sizeof(hash256));
    retval = ucl_sha256_finish(hash256, &sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    hexdump(hash256, sizeof(hash256));

    MXC_TMR_Stop(UCL_TEST_TIMER);
    MXC_TMR_Init(UCL_TEST_TIMER, &tmr);
    MXC_TMR_Start(UCL_TEST_TIMER);
    start = MXC_TMR_GetCount(UCL_TEST_TIMER);

    configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_HASH_INPUT << UCL_INPUT_SHIFT) ^
                    (UCL_SHA256 << UCL_HASH_SHIFT) ^ (UCL_NO_PRECOMP << UCL_PRECOMP_TRICK_SHIFT);
    retval = ucl_ecdsa_signature(signature, demo_d, hash_function_ptr, hash256, sizeof(hash256),
                                 curve_params, configuration);
#else  // SEPARATE_HASH
    configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_MSG_INPUT << UCL_INPUT_SHIFT) ^
                    (UCL_SHA256 << UCL_HASH_SHIFT) ^ (UCL_NO_PRECOMP << UCL_PRECOMP_TRICK_SHIFT);
    retval = ucl_ecdsa_signature(signature, demo_d, hash_function_ptr, message, message_length,
                                 curve_params, configuration);
#endif // SEPARATE_HASH

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    stop = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("Elapsed: %d\n", stop - start);

    printf("r:\n");
    hexdump(signature.r, sizeof(r));
    printf("s:\n");
    hexdump(signature.s, sizeof(s));

    printf("ECDSA Verify\n");

    // Verify signature
#ifdef SEPARATE_HASH
    // Initialize SHA256 working area.
    retval = ucl_sha256_init(&sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    retval = ucl_sha256_core(&sha256_context, message, sizeof(message));

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    // Finish hash.
    memset(hash256, 0, sizeof(hash256));
    retval = ucl_sha256_finish(hash256, &sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    hexdump(hash256, sizeof(hash256));

    MXC_TMR_Stop(UCL_TEST_TIMER);
    MXC_TMR_Init(UCL_TEST_TIMER, &tmr);
    MXC_TMR_Start(UCL_TEST_TIMER);
    start = MXC_TMR_GetCount(UCL_TEST_TIMER);

    configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_HASH_INPUT << UCL_INPUT_SHIFT) ^
                    (UCL_SHA256 << UCL_HASH_SHIFT) ^ (UCL_NO_PRECOMP << UCL_PRECOMP_TRICK_SHIFT);
    retval = ucl_ecdsa_verification(demo_q, signature, hash_function_ptr, hash256, sizeof(hash256),
                                    curve_params, configuration);
#else  // SEPARATE_HASH
    configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_MSG_INPUT << UCL_INPUT_SHIFT) ^
                    (UCL_SHA256 << UCL_HASH_SHIFT) ^ (UCL_NO_PRECOMP << UCL_PRECOMP_TRICK_SHIFT);
    retval = ucl_ecdsa_verification(demo_q, signature, hash_function_ptr, message, message_length,
                                    curve_params, configuration);
#endif // SEPARATE_HASH

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    stop = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("Elapsed: %d\n", stop - start);

    return 0;
}

int aes_test(void)
{
    int i;
    int retval;
    uint8_t aes_key[32];
    uint8_t aes_plaintext[1024];
    uint8_t aes_ciphertext[1024];

    /** AES-128 KAT for self tests */
    u8 ct_ecb[4 * 16] = {0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca,
                         0xf3, 0x24, 0x66, 0xef, 0x97, 0xf5, 0xd3, 0xd5, 0x85, 0x03, 0xb9,
                         0x69, 0x9d, 0xe7, 0x85, 0x89, 0x5a, 0x96, 0xfd, 0xba, 0xaf, 0x43,
                         0xb1, 0xcd, 0x7f, 0x59, 0x8e, 0xce, 0x23, 0x88, 0x1b, 0x00, 0xe3,
                         0xed, 0x03, 0x06, 0x88, 0x7b, 0x0c, 0x78, 0x5e, 0x27, 0xe8, 0xad,
                         0x3f, 0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5d, 0xd4};

    u8 pt[4 * 16] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73,
                     0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7,
                     0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4,
                     0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45,
                     0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

    u8 key0[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                   0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
    u8 iv[16]   = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};

    uint8_t testmessage[256];

    for (i = 0; i < sizeof(testmessage); i++) {
        testmessage[i] = (uint8_t)i;
    }

    printf("AES Test\n");
    printf("UCL_AES_BLOCKSIZE %d\n", UCL_AES_BLOCKSIZE);

    // Do one AES encryption
    memcpy(aes_plaintext, pt, 64);
    memcpy(aes_key, key0, 16);
    retval = ucl_aes_ecb(aes_ciphertext, aes_plaintext, 64, aes_key, UCL_AES_KEYLEN_128,
                         UCL_CIPHER_ENCRYPT);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    if (memcmp(aes_ciphertext, ct_ecb, 64) != 0) {
        printf("AES encrypt failed\n");
        printf("Expected:\n");
        hexdump(aes_ciphertext, 64);
        printf("Calculated:\n");
        hexdump(ct_ecb, 64);

        while (1)
            ;
    }

    // Do one AES decryption
    memset(aes_plaintext, 0, sizeof(aes_plaintext));
    retval = ucl_aes_ecb(aes_plaintext, aes_ciphertext, 64, aes_key, UCL_AES_KEYLEN_128,
                         UCL_CIPHER_DECRYPT);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    if (memcmp(aes_plaintext, pt, 64) != 0) {
        printf("AES decrypt failed\n");
        printf("Expected:\n");
        hexdump(pt, 64);
        printf("Calculated:\n");
        hexdump(aes_plaintext, 64);

        while (1)
            ;
    }

    // Do AES CBC encryption
    memcpy(aes_plaintext, testmessage, sizeof(testmessage));
    memcpy(aes_key, key0, 16);
    retval = ucl_aes_cbc(aes_ciphertext, aes_plaintext, sizeof(testmessage), aes_key,
                         UCL_AES_KEYLEN_128, iv, UCL_CIPHER_ENCRYPT);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    printf("AES CBC Encrypt Output:\n");
    hexdump(aes_ciphertext, sizeof(testmessage));

    // Do AES CBC decryption
    memset(aes_plaintext, 0, sizeof(aes_plaintext));
    retval = ucl_aes_cbc(aes_plaintext, aes_ciphertext, sizeof(testmessage), aes_key,
                         UCL_AES_KEYLEN_128, iv, UCL_CIPHER_DECRYPT);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    if (memcmp(aes_plaintext, testmessage, sizeof(testmessage)) != 0) {
        printf("AES CBC decrypt failed\n");
        printf("Expected:\n");
        hexdump(testmessage, sizeof(testmessage));
        printf("Calculated:\n");
        hexdump(aes_plaintext, sizeof(testmessage));

        while (1)
            ;
    }

    printf("AES Decrypt Output:\n");
    hexdump(aes_plaintext, sizeof(testmessage));

    return 0;
}

// "abc" hash result.
uint8_t sha256_hash_result[] = {
    0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA, 0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
    0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C, 0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD,
};

int sha_test(void)
{
    int i;
    int retval;
    ucl_sha256_ctx_t sha256_context;
    uint8_t abc[] = {'a', 'b', 'c'};
    uint8_t hash256[32];

    // Initialize SHA256 working area.
    retval = ucl_sha256_init(&sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    // Slow hash one byte at a time.
    for (i = 0; i < 3; i++) {
        retval = ucl_sha256_core(&sha256_context, abc + i, 1);

        if (UCL_OK != retval) {
            while (1)
                ;
        }
    }

    // Finish hash.
    memset(hash256, 0, sizeof(hash256));
    retval = ucl_sha256_finish(hash256, &sha256_context);

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    if (memcmp(hash256, sha256_hash_result, sizeof(sha256_hash_result)) != 0) {
        printf("SHA Hash failed\n");
        printf("Expected:\n");
        hexdump(sha256_hash_result, sizeof(sha256_hash_result));
        printf("Calculated:\n");
        hexdump(hash256, sizeof(hash256));

        while (1)
            ;
    }

    printf("Hash:\n");
    c_hexdump(hash256, sizeof(hash256));

    return 0;
}

// *****************************************************************************
int main(void)
{
    int retval;
    int count;

    printf("Using Free UCL, software crypto\n");

    tmr.mode    = TMR_MODE_ONESHOT;
    tmr.cmp_cnt = 0xFFFFFFFF;
    tmr.pol     = 0;

    MXC_TMR_Stop(UCL_TEST_TIMER);
    MXC_TMR_Init(UCL_TEST_TIMER, &tmr);
    MXC_TMR_Start(UCL_TEST_TIMER);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    MXC_TMR_Stop(UCL_TEST_TIMER);
    MXC_TMR_Init(UCL_TEST_TIMER, &tmr);
    MXC_TMR_Start(UCL_TEST_TIMER);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    count = MXC_TMR_GetCount(UCL_TEST_TIMER);
    printf("count: %d\n", count);
    MXC_TMR_Stop(UCL_TEST_TIMER);

    printf("UCL Init\n");
    retval = ucl_init();

    if (UCL_OK != retval) {
        while (1)
            ;
    }

    sha_test();

    aes_test();

    printf("ICC_ctrl 0x%08X\n", MXC_ICC->cache_ctrl);
    ecdsa_test();
    MXC_ICC_Enable();
    printf("ICC_ctrl 0x%08X\n", MXC_ICC->cache_ctrl);
    ecdsa_test();

    printf("Program exit.\n");

    while (1) {
    }
}
