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

#ifndef __RSA_H__
#define __RSA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <config.h>

#include <ucl/ucl.h>
#include <ucl/ucl_data_conv.h>
#include <ucl/ucl_rsa.h>

#ifdef _MAXIM_HSM
#include <libhsm/HSM.h>
#endif /* _MAXIM_HSM */

typedef struct {
    ucl_rsa_public_key_t keyPu;
    ucl_rsa_private_key_t keyPr;
#ifdef _MAXIM_HSM
    CK_OBJECT_HANDLE HSM_Objkey;
    CK_OBJECT_HANDLE HSM_Objpubkey;
    char HSM_KeyLabel[MAX_LINE];
    int in_hsm;
#endif
} rsa_key_t;

/**
 * Perform a RSA Signature
 * @param input data to sign
 * @param input_size data length
 * @param signature output signature
 * @param key RSA key used to sign
 * @return ERR_OK if success otherwise error code
 */
int rsa_sign(const uint8_t* input, unsigned int input_size, uint8_t* signature, rsa_key_t key);

/**
 * Read a Signed RSA public key
 * @param modulus public key modulus
 * @param modulus_len public key modulus length
 * @param public_exponent public key exponent
 * @param public_exponent_len public key exponent length
 * @param signature Signature
 * @param signature_len signature length
 * @param filename file top read
 * @return ERR_OK if success otherwise error code
 */
int read_file_signed_rsa_publickey(uint8_t* modulus, size_t* modulus_len, uint8_t* public_exponent,
                                   size_t public_exponent_len, uint8_t* signature,
                                   size_t* signature_len, const char* filename);

/**
 * Read a RSA Key pair in an ASCII encoded UCL format file
 * @param rsaKey rsa key pair structure to store the data
 * @param filename name of the file to read the data
 * @return ERR_OK if success otherwise error code
 */
int read_file_rsa_keypair(rsa_key_t* rsaKey, char* filename);

/**
 * Print RSA key component
 * @param key Key to display
 */
void print_rsakey(rsa_key_t key);

#ifdef _MAXIM_HSM
/**
 * Load a reference of the key in the HSM specified by keyname
 * @param rsaKey pointer to structure to store key reference
 * @param keyname Name of the key to load
 * @return ERR_OK if success otherwise error code
 */
int load_HSM_rsa_key(rsa_key_t* rsaKey, char* keyname);
#endif /* _MAXIM_HSM */

#ifdef __cplusplus
}
#endif

#endif /* __RSA_H__ */
