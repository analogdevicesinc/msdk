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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <regex.h>
#include <log.h>

#include <ca_sign_build.h>
#include <maxim_c_utils.h>

int parse_datahex(unsigned char *ptr, const char *str, int length)
{
    regex_t regex;
    regmatch_t rm[2];

    int ret;
    int i;
    char value_str[4];
    char regex_str[32];
    (void)ptr;

    sprintf(regex_str, "^0?x?([0-9a-fA-F]{%d})\\s*$", length * 2);

    ret = regcomp(&regex, regex_str, REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 2, rm, 0);
    if (!ret) {
        for (i = 0; i < length * 2; i++) {
            memcpy(value_str, &str[rm[1].rm_so + i * 2], 2);
            value_str[3] = '\0';
            ptr[i] = strtoul(value_str, NULL, 16);
        }

    } else {
        regfree(&regex);
        print_warn("Bad formated data hex parameter \n");
        return ERR_INVALID_OPTION_FORMAT;
    }

    regfree(&regex);
    return ERR_OK;
}

int parse_longhex(unsigned int *ptr, const char *str)
{
    regex_t regex;
    regmatch_t rm[2];

    int ret;
    char value_str[10];
    (void)ptr;

    ret = regcomp(&regex, "^0?x?([0-9a-fA-F]{8})\\s*$", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 2, rm, 0);
    if (!ret) {
        memcpy(value_str, &str[rm[1].rm_so], 8);
        value_str[8] = '\0';
        *ptr = strtoul(value_str, NULL, 16);
    } else {
        regfree(&regex);
        print_warn("Bad formated 4 bytes hex parameter \n");
        return ERR_INVALID_OPTION_FORMAT;
    }

    regfree(&regex);
    return ERR_OK;
}

int parse_hex(unsigned int *ptr, const char *str)
{
    regex_t regex;
    regmatch_t rm[2];

    int ret;
    char value_str[4];
    (void)ptr;

    ret = regcomp(&regex, "^0?x?([0-9a-fA-F]{2})\\s*", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 2, rm, 0);
    regfree(&regex);
    if (!ret) {
        memcpy(value_str, &str[rm[1].rm_so], 2);
        value_str[3] = '\0';
        *ptr = strtoul(value_str, NULL, 16);
    } else {
        print_warn("Bad formated 1 byte hex parameter \n");
        return ERR_INVALID_OPTION_FORMAT;
    }

    return ERR_OK;
}

int parse_yesno(unsigned char *ptr, const char *str)
{
    regex_t regex;
    int ret;

    ret = regcomp(&regex, "^\\s*(y|o|yes|oui)\\s*$", REG_EXTENDED | REG_ICASE);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 0, NULL, 0);
    regfree(&regex);
    if (!ret) {
        *ptr = TRUE;
    } else {
        ret = regcomp(&regex, "^\\s*(n|no|non)\\s*$", REG_EXTENDED | REG_ICASE);
        if (ret) {
            return ERR_REGEXP_ERROR;
        }
        ret = regexec(&regex, str, 0, NULL, 0);
        regfree(&regex);
        if (!ret) {
            *ptr = FALSE;
        } else {
            print_warn("Bad formated yes/no parameter \n");
            return ERR_INVALID_OPTION_FORMAT;
        }
    }

    return ERR_OK;
}

int parse_int(unsigned int *ptr, const char *str)
{
    regex_t regex;
    regmatch_t rm[2];

    int ret;
    char value_str[10];

    ret = regcomp(&regex, "^ *([0-9]+) *", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 2, rm, 0);
    regfree(&regex);
    if (!ret) {
        memcpy(value_str, &str[rm[1].rm_so], rm[1].rm_eo - rm[1].rm_so);
        value_str[rm[1].rm_eo - rm[1].rm_so] = '\0';
        *ptr = strtoul(value_str, NULL, 10);
    } else {
        print_warn("Bad formated int parameter \n");
        return ERR_INVALID_OPTION_FORMAT;
    }

    return ERR_OK;
}

int parse_string(char *ptr, const char *str)
{
    regex_t regex;
    regmatch_t rm[2];

    int ret;

    ret = regcomp(&regex, "^\\s*(.*)\\s*$", REG_EXTENDED);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 2, rm, 0);
    regfree(&regex);
    if (!ret) {
        memcpy(ptr, &str[rm[1].rm_so], rm[1].rm_eo - rm[1].rm_so);
        ptr[rm[1].rm_eo - rm[1].rm_so] = '\0';
    } else {
        print_warn("Bad formated string parameter \n");
        return ERR_INVALID_OPTION_FORMAT;
    }

    return ERR_OK;
}

int parse_algo(unsigned char *ptr, const char *str)
{
    char strptr[MAX_STRING];
    unsigned int i = 0;
    parse_string(strptr, str);

    while (algo_name[i] != NULL) {
        if (strcmp(strptr, algo_name[i]) == 0) {
            *ptr = i;

            return ERR_OK;
        }
        i++;
    }

    print_warn("Bad Algo : %s\n", strptr);

    return ERR_INVALID_OPTION_FORMAT;
}

int parse_bootmethod(bootmethod_t *ptr, const char *str)
{
    regex_t regex;
    int ret;

    ret = regcomp(&regex, "^\\s*(direct|d)\\s*$", REG_EXTENDED | REG_ICASE);
    if (ret) {
        return ERR_REGEXP_ERROR;
    }

    ret = regexec(&regex, str, 0, NULL, 0);
    regfree(&regex);
    if (!ret) {
        *ptr = bm_direct;
    } else {
        ret = regcomp(&regex, "^\\s*(cmsis|c)\\s*$", REG_EXTENDED | REG_ICASE);
        if (ret) {
            return ERR_REGEXP_ERROR;
        }
        ret = regexec(&regex, str, 0, NULL, 0);
        regfree(&regex);
        if (!ret) {
            *ptr = bm_cmsis;
        } else {
            return ERR_INVALID_OPTION_FORMAT;
        }
    }

    return ERR_OK;
}

int parse_store(option_type_t type, void *ptr, const char *value, int min)
{
    int result;

    switch (type) {
    case OT_FILE:
        ASSERT_OK(parse_string(ptr, value));
        break;
    case OT_HEX:
        ASSERT_OK(parse_hex(ptr, value));
        break;
    case OT_INT:
        ASSERT_OK(parse_int(ptr, value));
        break;
    case OT_LONGHEX:
        ASSERT_OK(parse_longhex(ptr, value));
        break;
    case OT_DATAHEX:
        ASSERT_OK(parse_datahex(ptr, value, min));
        break;
    case OT_STRING:
        ASSERT_OK(parse_string(ptr, value));
        break;
    case OT_YESNO:
        ASSERT_OK(parse_yesno(ptr, value));
        break;
    case OT_ALGO:
        ASSERT_OK(parse_algo(ptr, value));
        break;
    case OT_BOOTMETHOD:
        ASSERT_OK(parse_bootmethod(ptr, value));
        break;
    default:
        print_error("Unknown parse type\n");
        return ERR_INTERNAL_ERROR;
    }

    return ERR_OK;
}
