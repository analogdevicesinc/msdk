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

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ecdsa.h"
#include "maxim_c_utils.h"
#include "read_file.h"
#include "scp_definitions.h"
#include "scp_utils.h"
#include "session_build.h"
#include <log.h>

static int seq;

int maxq1852_signed_cmd(const uint8_t* payload, size_t payload_length, char* name)
{
    int result;
    unsigned int i = 0;
    uint8_t data_frame[MAX_FRAME];
    size_t iframe = 0;
    size_t length;

    /* Signed Command Designator */
    data_frame[iframe++] = MAXQ1852_SCDESIGNATOR;
    uint8_t signature[ECDSA_SIGNATURE_LEN];

    /* Length : payload + TRID + SEQ + Signature */
    length = payload_length + 4 + 2 + ECDSA_SIGNATURE_LEN;
    data_frame[iframe++] = length & 255;
    data_frame[iframe++] = length >> 8;

    /* Payload */
    memcpy(&data_frame[iframe], payload, length);
    iframe += length;

    /* Transation ID */
    data_frame[iframe++] = config_g.msp_1852_tr_id >> 24;
    data_frame[iframe++] = (config_g.msp_1852_tr_id >> 16) & 255;
    data_frame[iframe++] = (config_g.msp_1852_tr_id >> 8) & 255;
    data_frame[iframe++] = config_g.msp_1852_tr_id & 255;

    /* Sequence Number */
    data_frame[iframe++] = seq & 255;
    data_frame[iframe++] = seq >> 8;
    seq = (seq + 1) & 0xffff;

    /* Sign Command */
    ASSERT_OK(ecdsa_sign(data_frame, iframe, signature, config_g.ecdsaKey));

    /* Add Signature */
    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        data_frame[iframe++] = signature[ECDSA_MODULUS_LEN - 1 - i];
    }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        data_frame[iframe++] = signature[2 * ECDSA_MODULUS_LEN - 1 - i];
    }

    // sprintf (message, "%s", idf_scp_cmd[COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS]);
    return packet_send(data_frame, iframe, "", name);
}

int maxq1852_generic_response(char* name)
{
    uint8_t data_frame[MAX_FRAME];
    size_t iframe = 0;

    data_frame[iframe++] = 'A';
    data_frame[iframe++] = 'G';
    data_frame[iframe++] = 'P';
    data_frame[iframe++] = 0x00;
    data_frame[iframe++] = 0x00;
    data_frame[iframe++] = 0x00;
    data_frame[iframe++] = 0x00;
    data_frame[iframe++] = MAXQ1852_SCPROMPT;

    // sprintf (message, "%s", idf_scp_cmd[COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS]);
    return packet_send(data_frame, iframe, "", name);
}

int erase_all_flash_areas_response(void)
{
    return maxq1852_generic_response("erase_all_flash_areas_response");
}

int erase_all_flash_areas(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_ERASE_ALL_FLASH_AREAS;

    return maxq1852_signed_cmd(payload, ipayload, "erase_all_flash_areas");
}

int load_customer_key(char* pub_x, char* pub_y)
{
    unsigned int i;
    unsigned int one_byte;
    u8 xq[ECDSA_MODULUS_LEN];
    u8 yq[ECDSA_MODULUS_LEN];

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_LOAD_CUSTOMER_KEY;

    for (i = 0; i < config_g.ecdsaKey.ecdsa_len; i++) {
        if (0 == sscanf(&(pub_x[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: pub_x parameter <%s> incorrectly formatted as a number\n", pub_x);
            return (EXIT_FAILURE);
        }
        xq[i] = one_byte;
    }

    for (i = 0; i < config_g.ecdsaKey.ecdsa_len; i++) {
        if (0 == sscanf(&(pub_y[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: pub_y parameter <%s> incorrectly formatted as a number\n", pub_y);
            return (EXIT_FAILURE);
        }
        yq[i] = one_byte;
    }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        payload[ipayload++] = xq[ECDSA_MODULUS_LEN - 1 - i];
    }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        payload[ipayload++] = yq[ECDSA_MODULUS_LEN - 1 - i];
    }

    return maxq1852_signed_cmd(payload, ipayload, "load_customer_key");
}

int verify_customer_key_response(void)
{
    return maxq1852_generic_response("verify_customer_key_response");
}

int verify_customer_key(char* pub_x, char* pub_y)
{
    unsigned int i;
    unsigned int one_byte;
    u8 xq[ECDSA_MODULUS_LEN];
    u8 yq[ECDSA_MODULUS_LEN];

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_VERIFY_CUSTOMER_KEY;

    for (i = 0; i < config_g.ecdsaKey.ecdsa_len; i++) {
        if (0 == sscanf(&(pub_x[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: pub_x parameter <%s> incorrectly formatted as a number\n", pub_x);
            return (EXIT_FAILURE);
        }

        xq[i] = one_byte;
    }

    for (i = 0; i < config_g.ecdsaKey.ecdsa_len; i++) {
        if (0 == sscanf(&(pub_y[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: pub_y parameter <%s> incorrectly formatted as a number\n", pub_y);
            return (EXIT_FAILURE);
        }
        yq[i] = one_byte;
    }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        payload[ipayload++] = xq[ECDSA_MODULUS_LEN - 1 - i];
    }

    for (i = 0; i < ECDSA_MODULUS_LEN; i++) {
        payload[ipayload++] = yq[ECDSA_MODULUS_LEN - 1 - i];
    }

    return maxq1852_signed_cmd(payload, ipayload, "verify_customer_key");
}

int activate_customer_key_response(void)
{
    return maxq1852_generic_response("activate_customer_key_response");
}

int activate_customer_key(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_ACTIVATE_CUSTOMER_KEY;

    return maxq1852_signed_cmd(payload, ipayload, "activate_customer_key");
}

int generate_application_startup_signature_response(void)
{
    return maxq1852_generic_response("generate_application_startup_signature_response");
}

int generate_application_startup_signature(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE;

    return maxq1852_signed_cmd(payload, ipayload, "generate_application_startup_signature");
}

int verify_application_startup_signature_response(void)
{
    return maxq1852_generic_response("verify_application_startup_signature_response");
}

int verify_application_startup_signature(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE;

    return maxq1852_signed_cmd(payload, ipayload, "verify_application_startup_signature");
}

int write_register_response(void)
{
    return maxq1852_generic_response("write_register_response");
}

int write_register(char* reg, char* value)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_WRITE_REGISTER;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(reg[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: reg parameter <%s> incorrectly formatted as a number\n", reg);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(value[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: value parameter <%s> incorrectly formatted as a number\n", value);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "write_register");
}

int read_register_response(void)
{
    return maxq1852_generic_response("read_register_response");
}

int read_register(char* reg)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_READ_REGISTER;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(reg[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: reg parameter <%s> incorrectly formatted as a number\n", reg);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "read_register");
}

int load_code_response(void)
{
    return maxq1852_generic_response("load_code_response");
}

int load_code(char* addr, char* code)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_LOAD_CODE;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(addr[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: addr parameter <%s> incorrectly formatted as a number\n", addr);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    for (i = 0; i < (int)strlen(code); i += 2) {
        if (0 == sscanf(&(code[i]), "%02x", &one_byte)) {
            printf("ERROR: code parameter <%s> incorrectly formatted as a number\n", code);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "load_code");
}

int load_file(char* hexfilename)
{
    unsigned int i, k;
    unsigned int last_index;
    u8* dataloc;
    char schunk_addr[10];
    char schunk[20000];
    char schunk_tmp[20000];
    int allff;
    u8 ad1, ad2, ad3, ad4;
    u32 ad;
    int result;

    size_t data_len = sizeof(u8) * 1024 * 1024 * config_g.flash_mb;
    uint8_t* data = malloc(data_len);
    if (NULL == data) {
        print_error(
            "Unable to allocate memory for binary data (%dMB requested)\n", config_g.flash_mb);
        return ERR_MEMORY_ERROR;
    }

    dataloc = (u8*)malloc(sizeof(u8) * 1024 * 1024);
    if (NULL == dataloc) {
        printf("ERROR: <data> allocation is not possible (%dMB requested)\n", config_g.flash_mb);
        return (EXIT_FAILURE);
    }
    if (extension("hex", hexfilename)) {
        ASSERT_OK(read_hex_file(hexfilename, data, &data_len, addr_g));
    } else {
        printf("ERROR: <%s> file extension not supported (only .hex)\n", hexfilename);
        return (EXIT_FAILURE);
    }

    for (i = 0; i < 1024 * 1024; i++) {
        dataloc[i] = 0xff;
    }

    for (i = 0; i < data_len; i++) {
        if (addr_g[i] > 1024 * 1024) {
            printf("ERROR: addr[%d]=" SSIZET_XFMT " is too large\n", i, addr_g[i]);
            exit(1);
        }
        if (dataloc[addr_g[i]] != 0xff) {
            printf("ERROR: data already allocated in " SSIZET_XFMT ": %02x-%02x\n", addr_g[i],
                dataloc[addr_g[i]], data[i]);
        } else {
            dataloc[addr_g[i]] = data[i];
        }
    }
    /* #4393 */
    for (last_index = 0, i = 0; i < data_len; i++) {
        if (last_index < addr_g[i]) {
            last_index = addr_g[i];
        }
    }

    /*  last_index=addr[data_len-1]; */
    for (i = 0; i < last_index; i += config_g.chunk_size) {
        /* 3.7.14 */
        /* extended-address is now already included in the address */
        /*      ad=(hex_extended_address<<16)^(i); */
        ad = i;
        ad1 = ad >> 24;
        ad2 = (ad >> 16) & 255;
        ad3 = (ad >> 8) & 255;
        ad4 = (ad & 255);
        sprintf(schunk_addr, "%08x", (ad4 << 24) ^ (ad3 << 16) ^ (ad2 << 8) ^ (ad1));

        /* sprintf(schunk_tmp,""); */
        schunk_tmp[0] = '\0';
        for (allff = 1, k = 0; k < config_g.chunk_size; k++) {
            if (0xff != dataloc[i + k]) {
                allff = 0;
            }
            sprintf(schunk, "%s%02x", schunk_tmp, dataloc[i + k]);
            sprintf(schunk_tmp, "%s", schunk);
        }
        /* load-code only if not ff    */
        if (0 == allff) {
            result = load_code(schunk_addr, schunk);
            if (EXIT_SUCCESS != result) {
                printf("ERROR in write_mem\n");
                free(dataloc);
                return (EXIT_FAILURE);
            }
            target();
            load_code_response();
            host();
        }
    }
    free(dataloc);
    return (EXIT_SUCCESS);
}

int load_customer_key_response(void)
{
    return maxq1852_generic_response("load_customer_key_response");
}

int verify_code_response(void)
{
    return maxq1852_generic_response("verify_code_response");
}

int verify_code(char* addr, char* code)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_VERIFY_CODE;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(addr[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: addr parameter <%s> incorrectly formatted as a number\n", addr);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    for (i = 0; i < (int)strlen(code); i += 2) {
        if (0 == sscanf(&(code[i]), "%02x", &one_byte)) {
            printf("ERROR: code parameter <%s> incorrectly formatted as a number\n", code);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "verify_code");
}

int verify_code_file(char* hexfilename)
{
    unsigned int i, k;
    unsigned int last_index;
    u8* dataloc;
    char schunk_addr[10];
    char schunk[20000];
    char schunk_tmp[20000];
    int allff;
    u8 ad1, ad2, ad3, ad4;
    u32 ad;
    int result;

    size_t data_len = sizeof(u8) * 1024 * 1024 * config_g.flash_mb;
    uint8_t* data = malloc(data_len);
    if (NULL == data) {
        print_error(
            "Unable to allocate memory for binary data (%dMB requested)\n", config_g.flash_mb);
        return ERR_MEMORY_ERROR;
    }

    dataloc = (u8*)malloc(sizeof(u8) * 1024 * 1024);
    if (NULL == dataloc) {
        printf("ERROR: <data> allocation is not possible (%dMB requested)\n", config_g.flash_mb);
        return (EXIT_FAILURE);
    }

    if (extension("hex", hexfilename)) {
        ASSERT_OK(read_hex_file(hexfilename, data, &data_len, addr_g));
    } else {
        free(data);
        free(dataloc);
        printf("ERROR: <%s> file extension not supported (only .hex)\n", hexfilename);
        return (EXIT_FAILURE);
    }

    for (i = 0; i < 1024 * 1024; i++) {
        dataloc[i] = 0xff;
    }

    for (i = 0; i < data_len; i++) {
        dataloc[addr_g[i]] = data[i];
    }

    last_index = addr_g[data_len - 1];
    for (i = 0; i < last_index; i += config_g.chunk_size) {
        /*3.7.14
           extended-address is now already included in the address
           ad=(hex_extended_address<<16)^(i);
         */
        ad = i;
        ad1 = ad >> 24;
        ad2 = (ad >> 16) & 255;
        ad3 = (ad >> 8) & 255;
        ad4 = (ad & 255);
        sprintf(schunk_addr, "%08x", (ad4 << 24) ^ (ad3 << 16) ^ (ad2 << 8) ^ (ad1));

        /* sprintf(schunk_tmp,""); */
        schunk_tmp[0] = '\0';
        for (allff = 1, k = 0; k < config_g.chunk_size; k++) {
            if (0xff != dataloc[i + k]) {
                allff = 0;
            }
            sprintf(schunk, "%s%02x", schunk_tmp, dataloc[i + k]);
            sprintf(schunk_tmp, "%s", schunk);
        }
        /* load-code only if not ff   */
        if (0 == allff) {
            result = verify_code(schunk_addr, schunk);
            if (EXIT_SUCCESS != result) {
                printf("ERROR in write_mem\n");
                free(dataloc);
                return (EXIT_FAILURE);
            }
            target();
            verify_code_response();
            host();
        }
    }
    free(dataloc);
    return (EXIT_SUCCESS);
}

int load_data_response(void)
{
    return maxq1852_generic_response("load_data_response");
}

int load_data(char* addr, char* data)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_LOAD_DATA;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(addr[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: addr parameter <%s> incorrectly formatted as a number\n", addr);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    for (i = 0; i < (int)strlen(data); i += 2) {
        if (0 == sscanf(&(data[i]), "%02x", &one_byte)) {
            printf("ERROR: data parameter <%s> incorrectly formatted as a number\n", data);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "load_data");
}

int verify_maxq1852_data_response(void)
{
    return maxq1852_generic_response("verify_data_response");
}

int verify_maxq1852_data(char* addr, char* data)
{
    int i;
    unsigned int one_byte;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_VERIFY_DATA;

    for (i = 0; i < 4; i++) {
        if (0 == sscanf(&(addr[i * 2]), "%02x", &one_byte)) {
            printf("ERROR: addr parameter <%s> incorrectly formatted as a number\n", addr);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }
    for (i = 0; i < (int)strlen(data); i += 2) {
        if (0 == sscanf(&(data[i]), "%02x", &one_byte)) {
            printf("ERROR: data parameter <%s> incorrectly formatted as a number\n", data);
            return (EXIT_FAILURE);
        }
        payload[ipayload++] = one_byte;
    }

    return maxq1852_signed_cmd(payload, ipayload, "verify_data");
}

int erase_code_flash_area_response(void)
{
    return maxq1852_generic_response("erase_code_flash_area_response");
}

int erase_code_flash_area(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_ERASE_CODE_FLASH_AREA;

    return maxq1852_signed_cmd(payload, ipayload, "erase_code_flash_area");
}

int engage_pllo_response(void)
{
    return maxq1852_generic_response("engage_pllo_response");
}

int engage_pllo(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = MAXQ1852_ENGAGE_PLLO;

    payload[ipayload++] = 'P';
    payload[ipayload++] = 'L';
    payload[ipayload++] = 'L';
    payload[ipayload++] = 'O';

    return maxq1852_signed_cmd(payload, ipayload, "engage_pllo");
}

static char params[MAX_PARAMS][MAX_STRING];
static int nb_params;

int process_script_maxq1852_ecdsa(const char* filename)
{
    char line[MAX_STRING];
    int command;
    FILE* fpscript;
    fpscript = fopen(filename, "r");
    if (NULL == fpscript) {
        printf("ERROR: impossible to open <%s>\n", filename);
        return (EXIT_FAILURE);
    }
    /* initialize seq & ch id */
    seq = 1;
    host();
    while (fgets(line, MAX_STRING, fpscript) != NULL) {
        if (TRUE == verbose) {
            printf("<%s>", line);
        }
        /* if 1st char is a #, then considered as a comment and skip to next line */
        if ('#' == line[0]) {
            continue;
        }
        /* look for the command */
        command = process_command(line);
        if (TRUE == verbose) {
            printf("command=%s\n", idf_scp_cmd[command].name);
        }

        if (COMMAND_UNKNOWN == command) {
            printf(
                "ERROR: the command <%s> is unknown or not supported; check the script file <%s>\n",
                line, filename);
            return (EXIT_FAILURE);
        }
        switch (command) {
            /*	case COMMAND_HELP:
        if (0 == nb_params)
        {
            if (EXIT_SUCCESS != help ())
            {
                printf ("ERROR: help\n");
                return (EXIT_FAILURE);
            }
        }
        else
        {
            printf ("ERROR for help command\n");
            return (EXIT_FAILURE);
        }
        break;*/
        case COMMAND_MAXQ1852_ENGAGE_PLLO:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != engage_pllo()) {
                    printf("ERROR: engage-pllo\n");
                    return (EXIT_FAILURE);
                }
                target();
                engage_pllo_response();
                host();
            } else {
                printf("ERROR: incorrect format for engage-pllo command: engage-pllo\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_ERASE_CODE_FLASH_AREA:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != erase_code_flash_area()) {
                    printf("ERROR: erase-code-flash-area\n");
                    return (EXIT_FAILURE);
                }
                target();
                erase_code_flash_area_response();
                host();
            } else {
                printf("ERROR: incorrect format for erase-code-flash-area command: "
                       "erase-code-flash-area\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != erase_all_flash_areas()) {
                    printf("ERROR: erase-all-flash-areas\n");
                    return (EXIT_FAILURE);
                }
                target();
                erase_all_flash_areas_response();
                host();
            } else {
                printf("ERROR: incorrect format for erase-all-flash-areas command: "
                       "erase-all-flash-areas\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_ACTIVATE_CUSTOMER_KEY:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != activate_customer_key()) {
                    printf("ERROR: activate-customer-key\n");
                    return (EXIT_FAILURE);
                }
                target();
                activate_customer_key_response();
                host();
            } else {
                printf("ERROR: incorrect format for activate-customer-key command: "
                       "activate-customer-key\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != generate_application_startup_signature()) {
                    printf("ERROR: generate-application-startup-signature\n");
                    return (EXIT_FAILURE);
                }
                target();
                generate_application_startup_signature_response();
                host();
            } else {
                printf("ERROR: incorrect format for generate-application-startup-signature "
                       "command: generate-application-startup-signature\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE:
            if (0 == nb_params) {
                if (EXIT_SUCCESS != verify_application_startup_signature()) {
                    printf("ERROR: verify-application-startup-signature\n");
                    return (EXIT_FAILURE);
                }
                target();
                verify_application_startup_signature_response();
                host();
            } else {
                printf("ERROR: incorrect format for verify-application-startup-signature "
                       "command: verify-application-startup-signature\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_LOAD_CUSTOMER_KEY:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != load_customer_key(params[0], params[1])) {
                    printf("ERROR: load-customer-key\n");
                    return (EXIT_FAILURE);
                }
                target();
                load_customer_key_response();
                host();
            } else {
                printf("ERROR: incorrect format for load-customer-key command: "
                       "load-customer-key <pub-x> <pub-y>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_VERIFY_CUSTOMER_KEY:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != verify_customer_key(params[0], params[1])) {
                    printf("ERROR: verify-customer-key\n");
                    return (EXIT_FAILURE);
                }
                target();
                verify_customer_key_response();
                host();
            } else {
                printf("ERROR: incorrect format for verify-customer-key command: "
                       "verify-customer-key <pub-x> <pub-y>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_WRITE_REGISTER:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != write_register(params[0], params[1])) {
                    printf("ERROR: write-register\n");
                    return (EXIT_FAILURE);
                }
                target();
                write_register_response();
                host();
            } else {
                printf("ERROR: incorrect format for write-register: write-register <register> "
                       "<value>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_READ_REGISTER:
            if (1 == nb_params) {
                if (EXIT_SUCCESS != read_register(params[0])) {
                    printf("ERROR: read-register\n");
                    return (EXIT_FAILURE);
                }
                target();
                read_register_response();
                host();
            } else {
                printf("ERROR: incorrect format for write-register: read-register <register>\n");
                return (EXIT_FAILURE);
            }
            break;
            /* write-only-file is an abstraction for write-data, used in scp and scp-flora */
        case COMMAND_MAXQ1852_LOAD_FILE:
            if (1 == nb_params) {
                if (EXIT_SUCCESS != load_file(params[0])) {
                    printf("ERROR: load-file\n");
                    return (EXIT_FAILURE);
                }
            } else if (2 == nb_params) {
                if (EXIT_SUCCESS != load_file(params[0])) {
                    printf("ERROR: load-file\n");
                    return (EXIT_FAILURE);
                }
            } else {
                printf("ERROR: incorrect format for WRITE-ONLY command: write-file-only "
                       "<s19file> <address-offset:optional>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_LOAD_CODE:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != load_code(params[0], params[1])) {
                    printf("ERROR: load-code\n");
                    return (EXIT_FAILURE);
                }
                target();
                load_code_response();
                host();
            } else {
                printf("ERROR: incorrect format for load-code: load-code <address> <code>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_VERIFY_FILE:
            if (1 == nb_params) {
                if (EXIT_SUCCESS != verify_code_file(params[0])) {
                    printf("ERROR: verify-file\n");
                    return (EXIT_FAILURE);
                }
            } else if (2 == nb_params) {
                if (EXIT_SUCCESS != verify_code_file(params[0])) {
                    printf("ERROR: verify-file\n");
                    return (EXIT_FAILURE);
                }
            } else {
                printf("ERROR: incorrect format for WRITE-ONLY command: verify-code-file-only "
                       "<s19file> <address-offset:optional>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_VERIFY_CODE:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != verify_code(params[0], params[1])) {
                    printf("ERROR: verify-code\n");
                    return (EXIT_FAILURE);
                }
                target();
                verify_code_response();
                host();
            } else {
                printf("ERROR: incorrect format for verify-code: verify-code <address> <code>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_LOAD_DATA:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != load_data(params[0], params[1])) {
                    printf("ERROR: load-data\n");
                    return (EXIT_FAILURE);
                }
                target();
                load_data_response();
                host();
            } else {
                printf("ERROR: incorrect format for load-data: load-data <address> <data>\n");
                return (EXIT_FAILURE);
            }
            break;
        case COMMAND_MAXQ1852_VERIFY_DATA:
            if (2 == nb_params) {
                if (EXIT_SUCCESS != verify_maxq1852_data(params[0], params[1])) {
                    printf("ERROR: verify-data\n");
                    return (EXIT_FAILURE);
                }
                target();
                verify_maxq1852_data_response();
                host();
            } else {
                printf("ERROR: incorrect format for verify-data: verify-data <address> <code>\n");
                return (EXIT_FAILURE);
            }
            break;
        default:
            printf("ERROR: the command <%s> is not supported\n", line);
            return (EXIT_FAILURE);
        }
    }
    (void)fclose(fpscript);
    return (EXIT_SUCCESS);
}
