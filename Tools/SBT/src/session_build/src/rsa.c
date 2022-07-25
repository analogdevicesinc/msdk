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
#include <string.h>
#include <errno.h>

#include <ucl/ucl_config.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_aes.h>
#include <ucl/ucl_data_conv.h>
#include <ucl/ucl_rsa.h>
#include <ucl/ucl_pkcs1_ssa_pss_sha256.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_defs.h>

#include "session_build.h"
#include "scp_definitions.h"
#include "read_file.h"
#include "rsa.h"
#include <log.h>

int ucl_pkcs1_emsa_pss_sha256_encode(u8* EM, u8* M, u32 M_length, u32 salt_length, u32 k,
                                     int offset);

int rsa_sign(const uint8_t* input, unsigned int input_size, uint8_t* signature, rsa_key_t key)
{
    unsigned int i = 0;
    int err        = 0;
    int sl         = 0; /* salt length is 0 */

#ifdef _MAXIM_HSM
    long unsigned int l_iSignatureLength = key.keyPr.modulus_length;
    uint8_t EncodedMessage[MAX_RSA_LENGTH];
    uint8_t lastbyte;
    int offset;
    uint32_t tmp;
#endif /* _MAXIM_HSM */

    print_debug("RSA-%d Signature\n", key.keyPr.modulus_length * 8);

#ifdef _MAXIM_HSM
    if (key.in_hsm) {
        print_debug("Using HSM\n");

        ucl_data_os2int((uint32_t*)EncodedMessage, key.keyPr.modulus_length / 4, key.keyPu.modulus,
                        key.keyPr.modulus_length);

        tmp      = ((u32*)(EncodedMessage))[(key.keyPr.modulus_length / 4) - 1];
        lastbyte = (u8)(tmp >> 24);
        offset   = 9;
        while (0 != lastbyte) {
            lastbyte >>= 1;
            offset--;
        }

        err = ucl_pkcs1_emsa_pss_sha256_encode(EncodedMessage, (uint8_t*)input, input_size, sl,
                                               key.keyPr.modulus_length, offset);
        if (err != UCL_OK) {
            print_error("on ucl_pkcs1_emsa_pss_sha256_encode (%d)\n", err);
            return err;
        }

        err = HSM_DecryptRSA(session, EncodedMessage, key.keyPr.modulus_length, signature,
                             &l_iSignatureLength, key.HSM_Objkey);

        if (err != UCL_OK) {
            HSM_pError(err);
            print_error("on HSM_DecryptRSA (0x%x)\n", err);
            return err;
        }
    } else
#endif /* _MAXIM_HSM */
    {
        err = ucl_pkcs1_ssa_pss_sha256_sign(signature, (uint8_t*)input, input_size, &key.keyPr, sl);

        if (err != UCL_OK) {
            print_error("rsa pkcs1 sha256 sign (%d) %d %d\n", err, input_size, sl);
            return err;
        }
    }

    err = ucl_pkcs1_ssa_pss_sha256_verify(signature, (uint8_t*)input, input_size, &key.keyPu, sl);

    if (err != UCL_OK) {
        print_error("in verify signature (%d)\n", err);
        return err;
    }

    print_debug("Signature verification OK\n");

    print_debug("Signature :\n\t");
    for (i = 0; i < key.keyPr.modulus_length; i++) { print_d("%02x", signature[i]); }
    print_d("\n");

    return ERR_OK;
}

int read_file_signed_rsa_publickey(uint8_t* modulus, size_t* modulus_len, uint8_t* public_exponent,
                                   size_t public_exponent_len, uint8_t* signature,
                                   size_t* signature_len, const char* filename)

{
    FILE* pFile;
    unsigned int i;
    int result;
    unsigned char tmp_buffer[MAX_RSA_LENGTH];
    size_t buffer_length = MAX_RSA_LENGTH;

    print_debug("Read file signed rsa puk <%s>\n", filename);

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid RSA key filename\n");

        return ERR_FILE_ERROR;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return ERR_FILE_ERROR;
    }

    /* Read Modulus */
    memset(modulus, 0, MAX_RSA_LENGTH);
    buffer_length = MAX_RSA_LENGTH;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read RSA modulus from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(modulus, tmp_buffer, buffer_length);
    *modulus_len = buffer_length;

    /* Read public exponent */
    memset(public_exponent, 0, UCL_RSA_PUBLIC_EXPONENT_MAXSIZE);
    buffer_length = UCL_RSA_PUBLIC_EXPONENT_MAXSIZE;

    result = read_line_ascii_data(pFile, &buffer_length, public_exponent);
    if (result != ERR_OK) {
        print_error("Unable to read RSA private Exponent from file %s \n", filename);
        fclose(pFile);
        return result;
    }
    public_exponent_len = buffer_length;

    /* Read Signature */
    memset(signature, 0, MAX_RSA_LENGTH);
    buffer_length = MAX_RSA_LENGTH;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read RSA modulus from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(signature, tmp_buffer, buffer_length);
    *signature_len = buffer_length;

    print_debug("\tModulus Length: " SSIZET_FMT " bits\n", *modulus_len * 8);
    print_debug("\tModulus :\n\t");
    for (i = 0; i < *modulus_len; i++) { print_d("%02x", modulus[i]); }
    print_d("\n");

    print_debug("\tPublic Exponent :\n\t");
    for (i = 0; i < public_exponent_len; i++) { print_d("%02x", public_exponent[i]); }
    print_d("\n");

    print_debug("\tSignature Length: " SSIZET_FMT " bits\n", *signature_len * 8);
    print_debug("\tSignature :\n\t");
    for (i = 0; i < *signature_len; i++) { print_d("%02x", signature[i]); }
    print_d("\n");

    fclose(pFile);

    return ERR_OK;
}

#ifdef _MAXIM_HSM

int load_HSM_rsa_key(rsa_key_t* rsaKey, char* keyname)
{
    CK_RV rv = ERR_OK;

    print_debug("Load HSM RSA key <%s>\n", keyname);

    rv = HSM_FindKey(session, keyname, TRUE, &(rsaKey->HSM_Objkey));
    if (rv != 0) {
        print_error("Unable to found RSA private key in HSM\n");
        HSM_pError(rv);
        return rv;
    }

    rv = HSM_GetRSAPublicKey(session, rsaKey->HSM_Objkey, rsaKey->keyPu.modulus,
                             (long unsigned int*)&(rsaKey->keyPu.modulus_length),
                             rsaKey->keyPu.public_exponent,
                             (long unsigned int*)&(rsaKey->keyPu.public_exponent_length));
    if (rv != 0) {
        HSM_pError(rv);
        print_error("Unable to found RSA public key in HSM\n");
        return rv;
    }
    rsaKey->in_hsm               = TRUE;
    rsaKey->keyPr.modulus_length = rsaKey->keyPu.modulus_length;
    print_rsakey(*rsaKey);

    return rv;
}
#endif /* _MAXIM_HSM */

int read_file_rsa_keypair(rsa_key_t* rsaKey, char* filename)
{
    FILE* pFile = NULL;
    int result  = 0;
    unsigned char tmp_buffer[MAX_RSA_LENGTH];
    size_t buffer_length = MAX_RSA_LENGTH;

    print_debug("Read RSA Key File : %s\n", filename);

#ifdef _MAXIM_HSM
    rsaKey->in_hsm = FALSE;
#endif

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid RSA key filename\n");

        return ERR_FILE_ERROR;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return ERR_FILE_ERROR;
    }

    /* Allocate memory for private key (UCL constraint) */
    rsaKey->keyPr.modulus          = malloc(MAX_RSA_LENGTH);
    rsaKey->keyPr.private_exponent = malloc(MAX_RSA_LENGTH);

    if (rsaKey->keyPr.modulus == NULL || rsaKey->keyPr.private_exponent == NULL) {
        print_error("Unable to allocate memory");
        return ERR_MEMORY_ERROR;
    }

    /* Read Modulus */
    memset(rsaKey->keyPu.modulus, 0, MAX_RSA_LENGTH);
    memset(rsaKey->keyPr.modulus, 0, MAX_RSA_LENGTH);

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read RSA modulus from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(rsaKey->keyPu.modulus, tmp_buffer, buffer_length);
    memcpy(rsaKey->keyPr.modulus, tmp_buffer, buffer_length);
    rsaKey->keyPu.modulus_length = buffer_length;
    rsaKey->keyPr.modulus_length = buffer_length;

    /* Read private exponent */
    memset(rsaKey->keyPr.private_exponent, 0, MAX_RSA_LENGTH);
    buffer_length = MAX_RSA_LENGTH;

    result = read_line_ascii_data(pFile, &buffer_length, tmp_buffer);
    if (result != ERR_OK) {
        print_error("Unable to read RSA private Exponent from file %s \n", filename);
        fclose(pFile);
        return result;
    }

    memcpy(rsaKey->keyPr.private_exponent, tmp_buffer, buffer_length);

    if (rsaKey->keyPu.modulus_length != buffer_length) {
        print_error("lengths of modulus and private exponent do not match: %dB vs " SSIZET_FMT
                    "B\n",
                    rsaKey->keyPu.modulus_length, buffer_length);
        fclose(pFile);
        return ERR_INCONSISTENT_KEY;
    }

    /* Read public exponent */
    memset(rsaKey->keyPu.public_exponent, 0, UCL_RSA_PUBLIC_EXPONENT_MAXSIZE);
    buffer_length = UCL_RSA_PUBLIC_EXPONENT_MAXSIZE;

    result = read_line_ascii_data(pFile, &buffer_length, rsaKey->keyPu.public_exponent);
    if (result != ERR_OK) {
        print_error("Unable to read RSA private Exponent from file %s \n", filename);
        fclose(pFile);
        return result;
    }
    rsaKey->keyPu.public_exponent_length = buffer_length;

    print_rsakey(*rsaKey);

    fclose(pFile);

    return ERR_OK;
}

void print_rsakey(rsa_key_t key)
{
    unsigned int i = 0;

    print_debug("RSA key: \n");
#ifdef _MAXIM_HSM
    if (key.in_hsm) {
        print_d("\tHSM key name:\t%s", key.HSM_KeyLabel);
    } else
#endif /* _MAXIM_HSM */
    {
        print_debug("\tModulus Length: %d bits\n", key.keyPu.modulus_length * 8);
        print_debug("\tModulus :\n\t");
        for (i = 0; i < key.keyPu.modulus_length; i++) { print_d("%02x", key.keyPu.modulus[i]); }
        print_d("\n");

        print_debug("\tPrivate Exponent :\n\t");
        for (i = 0; i < key.keyPr.modulus_length; i++) {
            print_d("%02x", key.keyPr.private_exponent[i]);
        }
        print_d("\n");

        print_debug("\tPublic Exponent :\n\t");
        for (i = 0; i < key.keyPu.public_exponent_length; i++) {
            print_d("%02x", key.keyPu.public_exponent[i]);
        }
        print_d("\n");
    }
}
