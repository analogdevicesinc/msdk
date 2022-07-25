/*******************************************************************************
* Copyright (C) 2009-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
*******************************************************************************
*
* @author: Yann Loisel <yann.loisel@maximintegrated.com>
* @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>

#include <ucl/ucl_config.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_defs.h>
//#include <ucl/ucl_aes.h>
#include <ucl/ucl_sha256.h>
#include <ucl/ecdsa_generic_api.h>

#include "session_build.h"
#include "scp_definitions.h"
#include "read_file.h"
#include "ecdsa.h"
#include <maxim_c_utils.h>
#include <log.h>

int ecdsa_sign(const unsigned char* input, unsigned int input_size, unsigned char* signature,
               ecdsa_key_t key)
{
    /* message */
    uint8_t msg3[] = {'a', 'b', 'c'};
    /* public key */
    uint8_t xq3[] = {0x24, 0x42, 0xA5, 0xCC, 0x0E, 0xCD, 0x01, 0x5F, 0xA3, 0xCA, 0x31,
                     0xDC, 0x8E, 0x2B, 0xBC, 0x70, 0xBF, 0x42, 0xD6, 0x0C, 0xBC, 0xA2,
                     0x00, 0x85, 0xE0, 0x82, 0x2C, 0xB0, 0x42, 0x35, 0xE9, 0x70};
    uint8_t yq3[] = {0x6F, 0xC9, 0x8B, 0xD7, 0xE5, 0x02, 0x11, 0xA4, 0xA2, 0x71, 0x02,
                     0xFA, 0x35, 0x49, 0xDF, 0x79, 0xEB, 0xCB, 0x4B, 0xF2, 0x46, 0xB8,
                     0x09, 0x45, 0xCD, 0xDF, 0xE7, 0xD5, 0x09, 0xBB, 0xFD, 0x7D};
    /* signature for the message above */
    uint8_t r3[] = {0xCB, 0x28, 0xE0, 0x99, 0x9B, 0x9C, 0x77, 0x15, 0xFD, 0x0A, 0x80,
                    0xD8, 0xE4, 0x7A, 0x77, 0x07, 0x97, 0x16, 0xCB, 0xBF, 0x91, 0x7D,
                    0xD7, 0x2E, 0x97, 0x56, 0x6E, 0xA1, 0xC0, 0x66, 0x95, 0x7C};
    uint8_t s3[] = {0x86, 0xFA, 0x3B, 0xB4, 0xE2, 0x6C, 0xAD, 0x5B, 0xF9, 0x0B, 0x7F,
                    0x81, 0x89, 0x92, 0x56, 0xCE, 0x75, 0x94, 0xBB, 0x1E, 0xA0, 0xC8,
                    0x92, 0x12, 0x74, 0x8B, 0xFF, 0x3B, 0x3D, 0x5B, 0x03, 0x15};

    unsigned int i;
    int resu;

    ucl_type_ecdsa_signature ucl_signature;
    int configuration = (SECP256R1 << UCL_CURVE_SHIFT) ^ (UCL_MSG_INPUT << UCL_INPUT_SHIFT) ^
                        (UCL_SHA256 << UCL_HASH_SHIFT) ^
                        (UCL_NO_PRECOMP << UCL_PRECOMP_TRICK_SHIFT);

    ucl_type_ecc_u8_affine_point Q3 = {.x = xq3, .y = yq3};
    ucl_type_ecdsa_signature RS3    = {.r = r3, .s = s3};

#ifdef _MAXIM_HSM
    long unsigned int l_iSignatureLength = 128;
    u8 hash[UCL_SHA256_HASHSIZE];
#endif

    /* Allocate memory for Signature key (UCL constraint) */
    ucl_signature.r = malloc(ECDSA_MODULUS_LEN);
    ucl_signature.s = malloc(ECDSA_MODULUS_LEN);

    if (ucl_signature.r == NULL || ucl_signature.s == NULL) {
        /* In case only one of the two is not allocated */
        free(ucl_signature.r);
        free(ucl_signature.s);

        print_error("Unable to allocate memory");
        return ERR_MEMORY_ERROR;
    }

    /* Known Answer Test Check */

    resu =
        ucl_ecdsa_verification(Q3, RS3, &ucl_sha256, msg3, sizeof(msg3), &secp256r1, configuration);

    if (resu != UCL_OK) {
        free(ucl_signature.r);
        free(ucl_signature.s);

        print_error("ECDSA Known Answer Test Failed (%d)\n", resu);
        return resu;
    }

    /* Compute Signature */

#ifdef _MAXIM_HSM
    if (key.in_hsm) {
        resu = ucl_sha256(hash, (unsigned char*)input, input_size);
        resu = HSM_SignECDSA(session, hash, UCL_SHA256_HASHSIZE, signature, &l_iSignatureLength,
                             key.HSM_Objkey);

        if (resu != UCL_OK) {
            HSM_pError(resu);
            print_error("HSM ECDSA sha256 sign failed \n");
            return resu;
        }

        for (i = 0; i < 64; i++) {
            if (i < 32) {
                ucl_signature.r[i] = signature[i];
            } else {
                ucl_signature.s[i - 32] = signature[i];
            }
        }
    } else
#endif /* _MAXIM_HSM */
    {
        resu = ucl_ecdsa_signature(ucl_signature, key.ecdsa_privkey, &ucl_sha256,
                                   (unsigned char*)input, input_size, &secp256r1, configuration);

        if (resu != UCL_OK) {
            print_error("ECDSA-P256r1-SHA256 SIGNATURE COMPUTATION TEST-1 NOK %d \n", resu);
            return resu;
        }
    }

    /* Verify Signature */

#ifdef _MAXIM_HSM
    if (key.in_hsm) {
        resu = HSM_VerifyECDSA(session, hash, UCL_SHA256_HASHSIZE, signature, l_iSignatureLength,
                               key.HSM_Objpubkey);
    } else
#endif /* _MAXIM_HSM */
    {
        resu = ucl_ecdsa_verification(key.ecdsa_pubkey, ucl_signature, &ucl_sha256,
                                      (unsigned char*)input, input_size, &secp256r1, configuration);

        if (resu != UCL_OK) {
            print_error("ECDSA-P256r1-SHA256 Signature Verification Failed (%d) \n", resu);
            return resu;
        }
    }

    print_debug("Payload(%d):", input_size);

    for (i = 0; i < input_size; i++) { print_d("%02x", input[i]); }
    print_d("\n");

    print_debug("Signature:\n");
    print_d("\tr:");

    for (i = 0; i < 32; i++) { print_d("%02x", ucl_signature.r[i]); }
    print_d("\n");

    print_d("\ts:");
    for (i = 0; i < 32; i++) { print_d("%02x", ucl_signature.s[i]); }
    print_d("\n");

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) { signature[i] = ucl_signature.r[i]; }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        signature[ECDSA_MODULUS_LEN + i] = ucl_signature.s[i];
    }

    return ERR_OK;
}

int read_file_ecdsa_keypair(ecdsa_key_t* key, char* filename)
{
    FILE* pFile;
    int result = 0;
    unsigned char tmp_buffer[ECDSA_MODULUS_LEN];
    size_t buffer_length = ECDSA_MODULUS_LEN;

    print_debug("Read ECDSA Key File : %s\n", filename);

#ifdef _MAXIM_HSM
    key->in_hsm = FALSE;
#endif

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid ECDSA key filename\n");

        return ERR_FILE_ERROR;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /* Allocate memory for public key (UCL constraint) */
    key->ecdsa_pubkey.x = malloc(ECDSA_MODULUS_LEN);
    key->ecdsa_pubkey.y = malloc(ECDSA_MODULUS_LEN);

    if (key->ecdsa_pubkey.x == NULL || key->ecdsa_pubkey.y == NULL) {
        print_error("Unable to allocate memory");
        return ERR_MEMORY_ERROR;
    }

    /* Read Secret D */
    memset(key->ecdsa_privkey, 0, ECDSA_MODULUS_LEN);

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA secret D from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    if (buffer_length != ECDSA_MODULUS_LEN) {
        print_error("Bad format ECDSA key file %s (wrong size for D)\n", filename);
        fclose(pFile);
        return ERR_BAD_FORMAT;
    }

    memcpy(key->ecdsa_privkey, tmp_buffer, buffer_length);
    key->ecdsa_len = buffer_length;

    /* Read Public Point Q */
    memset(key->ecdsa_pubkey.x, 0, ECDSA_MODULUS_LEN);
    memset(key->ecdsa_pubkey.y, 0, ECDSA_MODULUS_LEN);
    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA public point X from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    if (buffer_length != ECDSA_MODULUS_LEN) {
        print_error("Bad format ECDSA key file %s (wrong size for X)\n", filename);
        fclose(pFile);
        return ERR_BAD_FORMAT;
    }

    memcpy(key->ecdsa_pubkey.x, tmp_buffer, buffer_length);

    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA public point Y from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    if (buffer_length != ECDSA_MODULUS_LEN) {
        print_error("Bad format ECDSA key file %s (wrong size for Y)\n", filename);
        fclose(pFile);
        return ERR_BAD_FORMAT;
    }

    memcpy(key->ecdsa_pubkey.y, tmp_buffer, buffer_length);

    print_ecdsaKey(*key);

    fclose(pFile);

    return ERR_OK;
}

int read_file_signed_ecdsa_publickey(u8* x, u8* y, u8* r, u8* s, size_t size, const char* filename)
{
    FILE* pFile;
    unsigned int i;
    int result;
    unsigned char tmp_buffer[MAX_RSA_LENGTH];
    size_t buffer_length = MAX_RSA_LENGTH;

    print_debug("Read file signed rsa puk <%s>\n", filename);

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid ECDSA signed key filename\n");

        return ERR_FILE_ERROR;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return ERR_FILE_ERROR;
    }

    /* Read Q.X */
    memset(x, 0, ECDSA_MODULUS_LEN);
    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA public point X from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(x, tmp_buffer, buffer_length);

    /* Read Q.Y */
    memset(y, 0, ECDSA_MODULUS_LEN);
    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA public point Y from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(y, tmp_buffer, buffer_length);

    /* Read Signature.R */
    memset(r, 0, ECDSA_MODULUS_LEN);
    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA Signature R from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(r, tmp_buffer, buffer_length);

    /* Read Signature.S */
    memset(r, 0, ECDSA_MODULUS_LEN);
    buffer_length = ECDSA_MODULUS_LEN;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read ECDSA Signature S from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(s, tmp_buffer, buffer_length);

    print_debug("Public Key:\n");
    print_d("\tX:\t");
    for (i = 0; i < size; i++) { print_d("%02x", x[i]); }
    print_d("\n");

    print_d("\tY:\t");
    for (i = 0; i < size; i++) { print_d("%02x", y[i]); }
    print_d("\n");

    print_debug("Public Key Signature:\n");
    print_d("\tr :\t");
    for (i = 0; i < size; i++) { print_d("%02x", r[i]); }
    print_d("\n");

    print_d("\ts :\t");
    for (i = 0; i < size; i++) { print_d("%02x", s[i]); }
    print_d("\n");

    fclose(pFile);

    return ERR_OK;
}

void print_ecdsaKey(ecdsa_key_t key)
{
    int i = 0;

    print_debug("ECDSA key: \n");
#ifdef _MAXIM_HSM
    if (key.in_hsm) {
        print_d("\tHSM key name:\t%s", key.HSM_KeyLabel);
    } else
#endif /* _MAXIM_HSM */
    {
        print_d("\tX:\t");
        for (i = 0; i < ECDSA_BLOCK_SIZE; i++) { print_d("%02x", key.ecdsa_pubkey.x[i]); }
        print_d("\n");

        print_d("\tY:\t");
        for (i = 0; i < ECDSA_BLOCK_SIZE; i++) { print_d("%02x", key.ecdsa_pubkey.y[i]); }
        print_d("\n");

        print_d("\tD:\t");
        for (i = 0; i < ECDSA_BLOCK_SIZE; i++) { print_d("%02x", key.ecdsa_privkey[i]); }
        print_d("\n");
    }
}

#ifdef _MAXIM_HSM

int load_HSM_ecdsa_key(ecdsa_key_t* key, char* keyname)
{
    CK_RV rv = ERR_OK;

    print_debug("Load HSM ecdsa key <%s>\n", keyname);

    rv = HSM_FindKey(session, keyname, TRUE, &(key->HSM_Objkey));
    HSM_pError(rv);
    if (rv != 0) {
        print_error("HSM ECDSA Private key not Found\n");
        return rv;
    }

    rv = HSM_FindKey(session, keyname, FALSE, &(key->HSM_Objpubkey));
    HSM_pError(rv);
    if (rv != 0) {
        print_error("HSM ECDSA Public key not Found\n");
        return rv;
    }

    key->in_hsm = TRUE;

    return rv;
}
#endif /* _MAXIM_HSM */
