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

#ifndef __SESSION_BUILD_H__
#define __SESSION_BUILD_H__

#include <stdint.h>
#include <stdio.h>

#include <ucl/ucl_config.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_aes.h>

#include <ecdsa.h>
#include <rsa.h>
#include <process.h>

#ifdef _MAXIM_HSM
#include <libhsm/HSM.h>
#endif /* _MAXIM_HSM */

#define MAJV 4
#define MINV 0
#define ZVER 0

#define INIFILE "session_build.ini"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define USE_COLOR   1
#define MAX_FRAME   102400
#define MAX_SEGMENT 102400
#define MAX_DATA    102400

#define MAX_PARAMS              4
#define MAX_EXTRA_PARAMS        10
#define MAX_SCP_SCRIPT_LINE     20
#define MAX_TAB                 100
#define MAX_FLASH_MB            1023
#define MAX_CHUNK_SIZE          102400
#define SCP_LITE_MAX_CHUNK_SIZE 40960
#define ANGELA_MAX_CHUNK_SIZE   15354
#define PAOLA_MAX_CHUNK_SIZE    MAX_CHUNK_SIZE
#define MAXQ1852_MAX_CHUNK_SIZE 1856
#define FLORA_MAX_CHUNK_SIZE    4094

/**
 *		Cryptography information
 */

#define MAX_RSA_LENGTH 512

/* byte length of the CRK public exponent */
#define RSA_2048_MODULUS_LEN    256
#define RSA_4096_MODULUS_LEN    512
#define RSA_PUBLIC_EXPONENT_LEN 4
#define RSA_2048_SIGNATURE_LEN  RSA_2048_MODULUS_LEN
#define RSA_4096_SIGNATURE_LEN  RSA_4096_MODULUS_LEN

/* ECDSA256 so signature is 2*32=64 bytes */
#define MAX_ECDSA 64

/* byte length of the CRK public exponent */
#define ECDSA_MODULUS_LEN   32
#define ECDSA_SIGNATURE_LEN (2 * ECDSA_MODULUS_LEN)

#define SECTOR_SIZE 4096

#define USN_LEN 13

typedef struct _type_config {
    u8 usn[USN_LEN];

    char output_file[MAX_STRING];
    char output_dir[MAX_STRING];
    char script_file[MAX_STRING];

    unsigned int rsa_len;
    unsigned int rsa_explen;
    unsigned int rsa_privexplen;

    rsa_key_t rsaKey;
    ecdsa_key_t ecdsaKey;

    u8 pp;

    /* this parameter represents the size, in MB, of the flash */
    /* targeted for the file programming (write-file) */
    int flash_mb;
    int address_offset;
    size_t chunk_size;

    char keyfile[MAX_STRING];
    /* RSA - ECDSA */
    u8 session_mode;
    char fullpath[500];
    char extra_param[MAX_PARAMS][MAX_STRING];

    /* SPC 1852 */
    unsigned int msp_1852_tr_id;

#ifdef _MAXIM_HSM
    int hsm;
    char HSM_KeyLabel[MAX_STRING];
    char hsm_thales_dll[MAX_STRING];
    int32_t hsm_slot_nb;
#endif /* _MAXIM_HSM */
} type_config_struct;

typedef enum {
    ERR_OK,
    ERR_INTERNAL_ERROR,
    ERR_NULL_POINTER,
    ERR_FILE_ERROR,
    ERR_CMD_UNKNOWN,
    ERR_FILE_NOT_FOUND,
    ERR_FILE_TOO_LONG,
    ERR_TOO_MANY_ARGS,
    ERR_MISSING_ARGS,
    ERR_MEMORY_ERROR,
    ERR_BAD_PARAMETER,
    ERR_BAD_FORMAT,
    ERR_NO_DEFAULT_DEVICE,
    ERR_NO_DEFAULT_CONFIG_DIR,
    ERR_PARSE_INI,
    ERR_REGEXP_ERROR,
    ERR_INVALID_OPTION_FORMAT,
    ERR_UNSUPPORTED_EXT,
    ERR_INCONSISTENT_KEY,
    ERR_BAD_MODE,
    ERR_UCL_INIT
} error_t;

typedef struct {
    char* name;
    option_type_t type;
    void* ptr;
    int min;
    int max;
} config_option_t;

type_config_struct config_g;
u8 verbose;

extern const char* mode_name[];
extern const char* pp_name[];

#ifdef _MAXIM_HSM
CK_SESSION_HANDLE session;
#endif /* _MAXIM_HSM */

size_t* addr_g;
FILE* fp_g;

#endif /* __SESSION_BUILD_H__ */
