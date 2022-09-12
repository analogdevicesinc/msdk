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
#include <ctype.h>
#include <errno.h>
#include <regex.h>

#include <ucl/ucl_config.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_aes.h>

#include "session_build.h"
#include "scp_utils.h"

#include <log.h>

#include <maxim_c_utils.h>

int hex_extended_address;

/* -- ASCII files - format parameters -- */

#define S19_ADDRESS_LEN 4
#define S19_CRC_LEN 1

#define HEX_START_CHAR ':'
#define HEX_RECORD_TYPE_POS1 7
#define HEX_RECORD_TYPE_POS2 8
#define HEX_LINE_LEN_POS1 1
#define HEX_LINE_LEN_POS2 2
#define HEX_ADDRESS_START 3
#define HEX_ADDRESS_END 6
#define HEX_EXTENDED_LINEAR_ADDRESS_START 9
#define HEX_EXTENDED_LINEAR_ADDRESS_END 10
#define HEX_DATA_START 9

int read_hex_file(const char *filename, uint8_t *data, size_t *data_len, size_t *addr)
{
    FILE *pFile;
    char line[MAXLINE];
    unsigned int i;
    regex_t regex;
    regex_t regex_discard;
    regmatch_t rm[6];
    int ret;
    char record_len_str[10];
    char record_addr_str[10];
    char record_type_str[10];
    char record_data_str[MAXLINE];
    char record_crc_str[10];
    unsigned int tmp;

    unsigned int record_type = 0;
    size_t current_len = 0;
    size_t record_len = 0;
    size_t record_addr = 0;
    size_t extended_addr = 0;

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid HEX filename\n");
        return ERR_BAD_PARAMETER;
    }

    print_debug("Read HEX file %s\n", filename);

    if ((data == NULL) || (data_len == NULL) || (addr == NULL) || (*data_len == 0)) {
        print_error("Null pointer\n");
        return ERR_NULL_POINTER;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /* Parse hex record line : :LLAAAATTDDDD..DDDCC */
    ret = regcomp(&regex,
                  "^:([0-9A-Fa-f]{2})([0-9A-Fa-f]{4})([0-9A-Fa-f]{2})([0-9A-Fa-f]*)([0-9A-Fa-f]{2})"
                  "[[:cntrl:]]*$",
                  REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    while (fgets(line, MAXLINE, pFile) != NULL) {
        if (!regexec(&regex, line, 6, rm, 0)) {
            memcpy(record_len_str, &line[rm[1].rm_so], rm[1].rm_eo - rm[1].rm_so);
            record_len_str[rm[1].rm_eo - rm[1].rm_so] = '\0';
            memcpy(record_addr_str, &line[rm[2].rm_so], rm[2].rm_eo - rm[2].rm_so);
            record_addr_str[rm[2].rm_eo - rm[2].rm_so] = '\0';
            memcpy(record_type_str, &line[rm[3].rm_so], rm[3].rm_eo - rm[3].rm_so);
            record_type_str[rm[3].rm_eo - rm[3].rm_so] = '\0';
            memcpy(record_data_str, &line[rm[4].rm_so], rm[4].rm_eo - rm[4].rm_so);
            record_data_str[rm[4].rm_eo - rm[4].rm_so] = '\0';
            memcpy(record_crc_str, &line[rm[5].rm_so], rm[5].rm_eo - rm[5].rm_so);
            record_crc_str[rm[5].rm_eo - rm[5].rm_so] = '\0';
        } else {
            print_error("HEX file not correctly formated\n");
            return ERR_BAD_FORMAT;
        }

        record_len = strtoul(record_len_str, NULL, 16);
        record_addr = strtoul(record_addr_str, NULL, 16);
        record_type = strtoul(record_type_str, NULL, 16);

        switch (record_type) {
        case 0: /* Data */
            if (record_len + current_len >= *data_len) {
                print_error("HEX file is too large\n");
                return ERR_FILE_TOO_LONG;
            }

            i = 0;
            while (EOF != sscanf(&(record_data_str[2 * i]), "%02x", &tmp)) {
                data[current_len + i] = (unsigned char)tmp;
                addr[current_len + i] = record_addr + i + extended_addr;
                i++;
            }
            current_len += i;

            if (i != record_len) {
                print_error("HEX Reading error, Length field and data length mismatch (%d "
                            "!= " SSIZET_FMT ") \n",
                            i, record_len);
                return ERR_BAD_FORMAT;
            }

            break;
        case 1: /* End Of File : do nothing */
            break;
        case 2: /* Extended Segment Address : do nothing */
            break;
        case 3: /* Start Segment Address : do nothing */
            break;
        case 4: /* Extended Linear Address  */
            extended_addr = strtoul(record_data_str, NULL, 16) * 65536;
            break;
        case 5: /* Start Linear Address : do nothing */
            break;
        default:
            break;
        }
    }

    if (ferror(pFile) || (feof(pFile) && current_len == 0)) {
        print_error("Unable to read %s\n", filename);
        return ERR_FILE_ERROR;
    }

    *data_len = current_len;

    print_debug("HEX file %s read :\n", filename);
    print_d("\tData Size :" SSIZET_FMT "\n", *data_len);

    fclose(pFile);
    regfree(&regex);
    regfree(&regex_discard);

    fprintf(fp_g, "\tData Size :" SSIZET_FMT "\n", *data_len);

    return ERR_OK;
}

int read_s19_file(const char *filename, size_t address_offset, uint8_t *data, size_t *data_len,
                  size_t *addr)
{
    FILE *pFile;
    char line[MAXLINE];
    unsigned int i;
    regex_t regex;
    regex_t regex_discard;
    regmatch_t rm[5];
    int ret;
    char record_len_str[10];
    char record_addr_str[10];
    char record_data_str[MAXLINE];
    char record_crc_str[10];
    unsigned int tmp;

    size_t current_len = 0;
    size_t record_len = 0;
    size_t record_addr = 0;

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid S19 filename\n");
        return ERR_BAD_PARAMETER;
    }

    print_debug("Read S19 file %s\n", filename);

    if ((data == NULL) || (data_len == NULL) || (addr == NULL) || (*data_len == 0)) {
        print_error("Null pointer\n");
        return ERR_NULL_POINTER;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /* Parse S3 record line : S3LLAAAAAAAADDDD..DDDCC */
    ret = regcomp(&regex,
                  "^S3([0-9A-Fa-f]{2})([0-9A-Fa-f]{8})([0-9A-Fa-f]*)([0-9A-Fa-f]{2})[[:cntrl:]]*$",
                  REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    /* Parse other S record line */
    ret = regcomp(&regex_discard, "^S([0-2]|[4-9])([0-9A-Fa-f]*)", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    while (fgets(line, MAXLINE, pFile) != NULL) {
        /* This function only read S3 records */
        if (!regexec(&regex_discard, line, 0, NULL, 0)) {
            continue;
        }

        if (!regexec(&regex, line, 5, rm, 0)) {
            memcpy(record_len_str, &line[rm[1].rm_so], rm[1].rm_eo - rm[1].rm_so);
            record_len_str[rm[1].rm_eo - rm[1].rm_so] = '\0';
            memcpy(record_addr_str, &line[rm[2].rm_so], rm[2].rm_eo - rm[2].rm_so);
            record_addr_str[rm[2].rm_eo - rm[2].rm_so] = '\0';
            memcpy(record_data_str, &line[rm[3].rm_so], rm[3].rm_eo - rm[3].rm_so);
            record_data_str[rm[3].rm_eo - rm[3].rm_so] = '\0';
            memcpy(record_crc_str, &line[rm[4].rm_so], rm[4].rm_eo - rm[4].rm_so);
            record_crc_str[rm[4].rm_eo - rm[4].rm_so] = '\0';
        } else {
            print_error("S19 file not correctly formated\n");
            return ERR_BAD_FORMAT;
        }

        record_len = strtoul(record_len_str, NULL, 16);
        record_addr = strtoul(record_addr_str, NULL, 16);

        record_addr += address_offset;

        if (record_len + current_len >= *data_len) {
            print_error("S19 file is too large\n");
            return ERR_FILE_TOO_LONG;
        }

        i = 0;
        while (EOF != sscanf(&(record_data_str[2 * i]), "%02x", &tmp)) {
            data[current_len + i] = (unsigned char)tmp;
            addr[current_len + i] = record_addr + i;
            i++;
        }
        current_len += i;

        if (i != record_len - S19_ADDRESS_LEN - S19_CRC_LEN) {
            print_error(
                "S19 Reading error, Length field and data length mismatch (%d != " SSIZET_FMT
                ") \n",
                i, record_len - S19_ADDRESS_LEN - S19_CRC_LEN);
            return ERR_BAD_FORMAT;
        }
    }

    if (ferror(pFile) || (feof(pFile) && current_len == 0)) {
        print_error("Unable to read %s\n", filename);
        return ERR_FILE_ERROR;
    }

    *data_len = current_len;

    print_debug("S19 file %s read :\n", filename);
    print_d("\tData Size :" SSIZET_FMT "\n", *data_len);

    fclose(pFile);
    regfree(&regex);
    regfree(&regex_discard);

    fprintf(fp_g, "\tData Size :" SSIZET_FMT "\n", *data_len);

    return ERR_OK;
}

int get_start_addr_and_length_s19(const char *filename, size_t *start_addr, size_t *end_addr)
{
    FILE *pFile;
    char line[MAXLINE];
    regex_t regex;
    regex_t regex_discard;
    regmatch_t rm[5];
    int ret;
    char record_len_str[10];
    char record_addr_str[10];

    size_t record_len = 0;
    size_t record_addr = 0;

    if ((filename == NULL) || (strlen(filename) == 0)) {
        print_error("Invalid S19 filename\n");
        return ERR_BAD_PARAMETER;
    }

    print_debug("Get Start address and Length of S19 file %s\n", filename);

    if ((start_addr == NULL) || (end_addr == NULL)) {
        print_error("Null pointer\n");
        return ERR_NULL_POINTER;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /* Parse S3 record line : S3LLAAAADDDD..DDDCC */
    ret = regcomp(&regex,
                  "^S3([0-9A-Fa-f]{2})([0-9A-Fa-f]{8})([0-9A-Fa-f]*)([0-9A-Fa-f]{2})[[:cntrl:]]*$",
                  REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    /* Parse other S record line */
    ret = regcomp(&regex_discard, "^S([0-2]|[4-9])([0-9A-Fa-f]*)", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    *start_addr = SIZE_MAX;
    *end_addr = 0;

    while (fgets(line, MAXLINE, pFile) != NULL) {
        /* This function only read S3 records */
        if (!regexec(&regex_discard, line, 0, NULL, 0)) {
            continue;
        }

        if (!regexec(&regex, line, 5, rm, 0)) {
            memcpy(record_len_str, &line[rm[1].rm_so], rm[1].rm_eo - rm[1].rm_so);
            record_len_str[rm[1].rm_eo - rm[1].rm_so] = '\0';
            memcpy(record_addr_str, &line[rm[2].rm_so], rm[2].rm_eo - rm[2].rm_so);
            record_addr_str[rm[2].rm_eo - rm[2].rm_so] = '\0';
        } else {
            print_error("S19 file not correctly formated\n");
            return ERR_BAD_FORMAT;
        }

        record_len = strtoul(record_len_str, NULL, 16) - S19_ADDRESS_LEN - S19_CRC_LEN;
        record_addr = strtoul(record_addr_str, NULL, 16);

        if (record_addr < *start_addr) {
            *start_addr = record_addr;
        }

        if ((record_addr + record_len) > *end_addr) {
            *end_addr = (record_addr + record_len);
        }
    }

    if (ferror(pFile) || (feof(pFile) && *end_addr == 0)) {
        print_error("Unable to read %s\n", filename);
        return ERR_FILE_ERROR;
    }

    print_debug("S19 file %s Start Address 0x" SSIZET_XFMT " :\n", filename, *start_addr);
    print_d("\tEnd Address : 0x" SSIZET_FMT "\n", *end_addr);

    fclose(pFile);
    regfree(&regex);
    regfree(&regex_discard);

    return ERR_OK;
}

int read_line_ascii_data(FILE *file_ptr, size_t *data_length, unsigned char *data_buffer)
{
    char *result = NULL;
    char *ascii_data;
    unsigned int i = 0;
    unsigned int tmp = 0;

    /* Null pointer check */
    if ((file_ptr == NULL) || data_length == NULL || data_buffer == NULL) {
        return ERR_NULL_POINTER;
    }

    if (*data_length == 0) {
        return ERR_BAD_PARAMETER;
    }

    ascii_data = (char *)malloc(*data_length * sizeof(char) * 2 + 10);
    if (ascii_data == NULL) {
        print_error("Unable to allocate memory");
        return ERR_MEMORY_ERROR;
    }

    /* Read an entire line in the file */
    result = fgets(ascii_data, *data_length * 2 + 10, file_ptr);
    if (result == NULL) {
        print_error("Read line from file error : %s\n", strerror(errno));
        free(ascii_data);
        return ERR_FILE_ERROR;
    }

    while (EOF != sscanf(&(ascii_data[2 * i]), "%02x", &tmp)) {
        data_buffer[i] = (unsigned char)tmp;
        i++;
    }

    *data_length = i;
    free(ascii_data);

    return ERR_OK;
}

int extension(const char *ext, const char *name)
{
    regex_t regex;
    int ret;
    char regex_str[32];

    sprintf(regex_str, "^(.*)\\.%s\\s*$", ext);

    ret = regcomp(&regex, regex_str, REG_EXTENDED | REG_ICASE);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, name, 0, NULL, 0);
    regfree(&regex);

    return ((!ret) ? TRUE : FALSE);
}

int read_file_size(u32 *size, char *filename)
{
    FILE *pFile = NULL;

    print_debug("Read size of file %s\n", filename);

    pFile = fopen(filename, "rb");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /** obtain file size */
    fseek(pFile, 0, SEEK_END);
    *size = ftell(pFile);
    rewind(pFile);

    print_debug("Size of file %s : %d Bytes\n", filename, *size);
    fclose(pFile);

    return ERR_OK;
}

int read_binary_file(char *filename, u8 *p_pucData, size_t *file_size)
{
    int i = 0;
    unsigned int result;
    FILE *pFile = NULL;

    print_debug("Read binary file : %s\n", filename);

    pFile = fopen(filename, "rb");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    /*  obtain file size: */
    fseek(pFile, 0, SEEK_END);
    *file_size = ftell(pFile);
    rewind(pFile);
    /*  copy the file into the buffer: */
    result = fread(p_pucData, 1, *file_size, pFile);
    if (result != *file_size) {
        print_error("when reading %s : %s\n", filename, strerror(errno));
        return errno;
    }

    print_debug(SSIZET_FMT " bytes read\n", *file_size);
    for (i = 0; i < (int)(*file_size); i++) {
        addr_g[i] = i;
        print_d("%02x", p_pucData[i]);
    }
    print_d("\n");

    fclose(pFile);
    return ERR_OK;
}
