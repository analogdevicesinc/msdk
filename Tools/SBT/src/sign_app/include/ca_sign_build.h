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

#ifndef __CA_SIGN_BUILD__
#define __CA_SIGN_BUILD__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include <ucl/ecdsa_generic_api.h>
#include <rsa.h>
#include <ecdsa.h>
#include <process.h>
#include <log.h>

#ifdef _MAXIM_HSM
#include <libhsm/HSM.h>
#endif /* _MAXIM_HSM */

#define MAJV 2
#define MINV 0
#define ZVER 1

#define INIFILE "ca_sign_build.ini"

#include <config.h>

#define MAX_RSA_LENGTH 512

#define RSA_2048_MODULUS_LEN 256
#define RSA_4096_MODULUS_LEN 512

#define HEADER_SYNC_LEN 8

#define HEADER_VERSION_LEN 4
#define HEADER_APPLICATION_VERSION_LEN 4
#define HEADER_LOAD_ADDRESS_LEN 4
#define HEADER_JUMP_ADDRESS_LEN 4
#define HEADER_BINARY_LEN 4
#define HEADER_ARGV_LEN 4
#define HEADER_SR_PAPD_LEN 1
#define HEADER_SR_PRFSH_LEN 4
#define HEADER_SR_PCFG_LEN 4
#define HEADER_SR_PEXT_LEN 1
#define HEADER_DMC_GCFG_LEN 4
#define HEADER_DMC_CLK_LEN 1
#define HEADER_UCI_KSRC_CONFIGENCINT_LEN 1
#define HEADER_UCI0_AC1R_START_OFFSET_LEN 4
#define HEADER_UCI0_AC1R_END_OFFSET_LEN 4
#define HEADER_UCI0_DDR_R0_LEN 4

#define FLORA_HEADER_LEN                                                                        \
    HEADER_SYNC_LEN + HEADER_VERSION_LEN + HEADER_LOAD_ADDRESS_LEN + HEADER_JUMP_ADDRESS_LEN +  \
        HEADER_BINARY_LEN + HEADER_ARGV_LEN + /*HEADER_GENERAL_INFO_LEN*/ +HEADER_SR_PAPD_LEN + \
        HEADER_SR_PRFSH_LEN + HEADER_SR_PCFG_LEN + HEADER_SR_PEXT_LEN + HEADER_DMC_GCFG_LEN +   \
        HEADER_DMC_CLK_LEN + HEADER_UCI_KSRC_CONFIGENCINT_LEN +                                 \
        HEADER_UCI0_AC1R_START_OFFSET_LEN + HEADER_UCI0_AC1R_END_OFFSET_LEN +                   \
        HEADER_UCI0_DDR_R0_LEN
#define ANGELA_HEADER_LEN                                                                      \
    HEADER_SYNC_LEN + HEADER_VERSION_LEN + HEADER_LOAD_ADDRESS_LEN + HEADER_JUMP_ADDRESS_LEN + \
        HEADER_BINARY_LEN + HEADER_ARGV_LEN + HEADER_APPLICATION_VERSION_LEN
#define MAX_HEADER_LEN FLORA_HEADER_LEN

typedef enum algo_e { a_rsa, a_rsa_paola, a_sha256, a_crc32, a_none, a_ecdsa } algo_t;

typedef struct {
    char device[32];
    ecdsa_key_t ecdsaKey;
    rsa_key_t rsaKey;

    uint32_t load_address;
    uint32_t jump_address;

    char arguments[MAX_STRING];
    u8 sr_papd;
    u32 sr_prfsh;
    u32 sr_pcfg;
    u32 dmc_gcfg;
    u8 dmc_clk;
    u32 uci2_ctrl_reg;
    u8 uci0_ksrc_configencint;
    u32 uci0_ac1r_so;
    u32 uci0_ac1r_eo;
    u32 uci0_ddr_r0;
    u8 sr_pext;
    u32 version;
    u32 application_version;
    algo_t algo;
    int verbose;
    bootmethod_t boot_method;
    char cafile[MAX_STRING];
    char scafile[MAX_STRING];
    char sigfile[MAX_STRING];
    char keyfile[MAX_STRING];

    int headergenerated;
    int signonly;
    char fullpath[MAX_STRING];

#ifdef _MAXIM_HSM
    int hsm;
    CK_OBJECT_HANDLE HSM_Objkey;
    CK_OBJECT_HANDLE HSM_Objpubkey;
    char HSM_KeyLabel[MAX_STRING];
    char hsm_thales_dll[MAX_STRING];
    int32_t hsm_slot_nb;
#endif

} type_config_struct;

type_config_struct config_g;
uint8_t verbose;

#ifdef _MAXIM_HSM
CK_SESSION_HANDLE session;
#endif /* _MAXIM_HSM */

typedef struct {
    char *name;
    option_type_t type;
    void *ptr;
    int min;
    int max;
} config_option_t;

typedef enum {
    ERR_OK,
    ERR_INTERNAL_ERROR,
    ERR_NULL_POINTER,
    ERR_FILE_ERROR,
    ERR_CMD_UNKNOWN,
    ERR_UNKNOWN_ALGO,
    ERR_FILE_NOT_FOUND,
    ERR_FILE_TOO_LONG,
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

extern const char *algo_name[];

#endif /* __CA_SIGN_BUILD__ */
