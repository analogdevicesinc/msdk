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
* @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
*
*/

#ifndef __ECDSA_H__
#define __ECDSA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <maxim_c_utils.h>
#include <config.h>

#include <ucl/ecdsa_generic_api.h>

#ifdef _MAXIM_HSM
#include <libhsm/HSM.h>
#endif /* _MAXIM_HSM */

/**
 * ECDSA Key Structure
 */
typedef struct {
    ucl_type_ecc_u8_affine_point ecdsa_pubkey;
    uint8_t ecdsa_privkey[ECDSA_MODULUS_LEN];
    size_t ecdsa_len;
#ifdef _MAXIM_HSM
    CK_OBJECT_HANDLE HSM_Objkey;
    CK_OBJECT_HANDLE HSM_Objpubkey;
    char HSM_KeyLabel[MAX_LINE];
    int in_hsm;
#endif
} ecdsa_key_t;

/**
 * Compute the ECDSA signature of data provided in the input buffer
 * @param input data to be signed
 * @param input_size Length of the data to be signed
 * @param signature output buffer of the computed ECDSA signature
 * @param key ECDSA key to sign the data
 * @return ERR_OK if success otherwise error code
 */
int ecdsa_sign(const unsigned char* input, unsigned int input_size, unsigned char* signature,
               ecdsa_key_t key);

/**
 * Read an ECDSA Key pair in an ASCII encoded UCL format file
 * @param key ECDSA key pair structure to store the data
 * @param filename name of the file to read the data
 * @return ERR_OK if success otherwise error code
 */
int read_file_ecdsa_keypair(ecdsa_key_t* key, char* filename);

/**
 * Read an signed ECDSA public key
 * @param x X coordinate of public key
 * @param y Y coordinate of public key
 * @param r R coordinate of signature
 * @param s S coordinate of signature
 * @param size
 * @param filename name of the file to read the data
 * @return ERR_OK if success otherwise error code
 */
int read_file_signed_ecdsa_publickey(u8* x, u8* y, u8* r, u8* s, size_t size, const char* filename);

/**
 * Print ECDSA key component
 * @param key Key to display
 */
void print_ecdsaKey(ecdsa_key_t key);

#ifdef _MAXIM_HSM
/**
 * Load a reference of the key in the HSM specified by keyname
 * @param key pointer to structure to store key reference
 * @param keyname Name of the key to load
 * @return ERR_OK if success otherwise error code
 */
int load_HSM_ecdsa_key(ecdsa_key_t* key, char* keyname);
#endif /* _MAXIM_HSM */

#ifdef __cplusplus
}
#endif

#endif /* __ECDSA_H__ */
