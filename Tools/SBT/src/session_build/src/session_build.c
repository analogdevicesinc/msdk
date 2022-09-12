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
#include <ctype.h>
#include <string.h>
#include <ini.h>
#include <regex.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>

#include <ucl/ucl.h>
#include <ucl/ucl_info.h>
#include <ucl/ucl_sys.h>

#ifdef _MAXIM_HSM
#include <libhsm/HSM.h>
#include "hsm.h"
#endif /* _MAXIM_HSM */

#include "session_build.h"
#include "scp_definitions.h"
#include <maxim_c_utils.h>
#include "process.h"
#include "scp.h"
#include "scp_lite.h"
#include "scp_maxq1852.h"
#include "ecdsa.h"
#include "rsa.h"
#include "log.h"
#include "utils.h"

#ifndef MAX_ARG_LEN
#define MAX_ARG_LEN 4096
#endif

config_option_t config_option[] = {
    { "flash_size_mb", OT_INT, &config_g.flash_mb, 0, 0 },
    { "usn", OT_USN, &config_g.usn, USN_LEN, 0 },
    { "key_file", OT_FILE, &config_g.keyfile, 0, 0 },
    /* Legacy */
    { "ecdsa_file", OT_FILE, &config_g.keyfile, 0, 0 },
    { "rsa_file", OT_FILE, &config_g.keyfile, 0, 0 },

#ifdef _MAXIM_HSM
    { "hsm", OT_YESNO, &config_g.hsm, 0, 0 },
    { "hsm_key_name", OT_STRING, &config_g.HSM_KeyLabel, 0, 0 },
    { "hsm_thales_dll", OT_FILE, &config_g.hsm_thales_dll, 0, 0 },
    { "hsm_slot_nb", OT_INT, &config_g.hsm_slot_nb, 0, 0 },
#endif

    { "verbose", OT_INT, &verbose, 0, 0 },

    { "session_mode", OT_SESSION_MODE, &config_g.session_mode, 0, 0 },
    { "pp", OT_PP, &config_g.pp, 0, 0 },
    { "script_file", OT_FILE, config_g.script_file, 0, 0 },
    { "output_file", OT_FILE, config_g.output_file, 0, 0 },
    { "output_dir", OT_FILE, config_g.output_dir, 0, 0 },

    { "chunk_size", OT_INT, &config_g.chunk_size, 0, 0 },
    { "transaction_id", OT_INT, &config_g.msp_1852_tr_id, 0, 0 },
    { "addr_offset", OT_LONGHEX, &config_g.address_offset, 0, 0 },
    { 0, 0, 0, 0, 0 },
};

static const int mode[MAX_SCP_COMMAND] = {
    [COMMAND_WRITE_FILE] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_RSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_WRITE_ONLY] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_RSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_ERASE_DATA] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_RSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_VERIFY_FILE] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_RSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_WRITE_CRK] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_REWRITE_CRK] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_WRITE_OTP] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_WRITE_TIMEOUT] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_KILL_CHIP] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_KILL_CHIP2] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_EXECUTE_CODE] = SCP_PAOLA_MSK + SCP_ANGELA_ECDSA_MSK + SCP_FLORA_RSA_MSK,
    [COMMAND_MAXQ1852_LOAD_CUSTOMER_KEY] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_VERIFY_CUSTOMER_KEY] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_ACTIVATE_CUSTOMER_KEY] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_ERASE_CODE_FLASH_AREA] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_LOAD_CODE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_LOAD_FILE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_LOAD_DATA] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_VERIFY_FILE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_VERIFY_CODE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_VERIFY_DATA] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_WRITE_REGISTER] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_READ_REGISTER] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_ENGAGE_PLLO] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE] = MSP_MAXQ1852_ECDSA_MSK,
    [COMMAND_SCP_LITE_LOAD_RAM] = SCP_LITE_ECDSA
};

uint32_t ucl_buffer_g[4096];

int init_crypto(void)
{
    int err = 0;

    err = ucl_init(ucl_buffer_g, 4096);

    if (err != UCL_OK) {
        print_error("Unable to Initialize UCL (ucl_init %d)\n", err);
        return err;
    }

#ifdef _MAXIM_HSM
    if (config_g.hsm) {
        /* Initialise DLL, giving the path of Thales cknfast DLL  */
        err = HSM_InitDLL(config_g.hsm_thales_dll);
        if (err != CKR_OK) {
            print_error("Unable to Initialize Thales HSM DLL \n");
            HSM_pError(err);
            return err;
        }
    }
#endif /* _MAXIM_HSM */

    return ERR_OK;
}

int parse_store_option(const char *name, const char *value)
{
    unsigned int i = 0;
    int result = 0;

    while (config_option[i].name != 0) {
        if (strcmp(config_option[i].name, name) == 0) {
            ASSERT_OK(parse_store(config_option[i].type, config_option[i].ptr, value,
                                  config_option[i].min));
            break;
        }
        i++;
    }

    return ERR_OK;
}

int load_args(int argc, char **argv, int start_index)
{
    int k;
    int extra_arg_idx = 0;

    for (k = start_index; k < argc; k++) {
        regex_t regex;
        regmatch_t rm[3];
        int ret;
        char name[MAX_ARG_LEN];
        char value[MAX_ARG_LEN];
        size_t memcpyLen;

        ret = regcomp(&regex, "^\\s*([0-9a-zA-Z_-]*)=(.*)\\s*", REG_EXTENDED);
        if (ret) {
            return ERR_INTERNAL_ERROR;
        }

        ret = regexec(&regex, argv[k], 3, rm, 0);
        if (!ret) {
            memcpyLen = rm[1].rm_eo - rm[1].rm_so;
            if (memcpyLen > MAX_ARG_LEN) {
                print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                return ERR_MEMORY_ERROR;
            }
            memcpy(name, &(argv[k][rm[1].rm_so]), memcpyLen);
            name[rm[1].rm_eo - rm[1].rm_so] = '\0';

            memcpyLen = rm[2].rm_eo - rm[2].rm_so;
            if (memcpyLen > MAX_ARG_LEN) {
                print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                return ERR_MEMORY_ERROR;
            }
            memcpy(value, &argv[k][rm[2].rm_so], memcpyLen);
            value[rm[2].rm_eo - rm[2].rm_so] = '\0';
            parse_store_option(name, value);
        } else {
            if (extra_arg_idx == 0) {
                parse_store_option("output_dir", argv[k]);
                extra_arg_idx++;
            } else if (extra_arg_idx < MAX_EXTRA_PARAMS) {
                strcpy(config_g.extra_param[extra_arg_idx - 1], argv[k]);
                extra_arg_idx++;
            } else {
                print_warn("bad parameter %s \n", argv[k]);
            }
        }

        regfree(&regex);
    }

    return ERR_OK;
}

static int load_keys(void)
{
    int resu;
    size_t max_chunk_size = 1024;

    if (MSP_MAXQ1852_ECDSA == config_g.session_mode || SCP_ANGELA_ECDSA == config_g.session_mode ||
        SCP_LITE_ECDSA == config_g.session_mode) {
#ifdef _MAXIM_HSM
        if (config_g.hsm) {
            resu = load_HSM_ecdsa_key(&config_g.ecdsaKey, config_g.HSM_KeyLabel);
        } else
#endif /* _MAXIM_HSM */
        {
            resu = read_file_ecdsa_keypair(&config_g.ecdsaKey, config_g.keyfile);
        }

        if (ERR_OK != resu) {
            print_error("Unable to retrieve ECDSA key\n");
            return resu;
        }

        if (config_g.pp != SCP_PP_ECDSA) {
            print_warn("SCP session mode is defined with this value: \"%s\".\n",
                       mode_name[config_g.session_mode]);
            print_warn("But the protection profile (\"pp\" value) is different from \"ECDSA\".\n");
        }
    }

    if (SCP_RSA == config_g.session_mode || SCP_FLORA_RSA == config_g.session_mode ||
        SCP_PAOLA == config_g.session_mode) {
#ifdef _MAXIM_HSM
        if (config_g.hsm) {
            resu = load_HSM_rsa_key(&config_g.rsaKey, config_g.HSM_KeyLabel);
        } else
#endif /* _MAXIM_HSM */
        {
            resu = read_file_rsa_keypair(&config_g.rsaKey, config_g.keyfile);
        }

        if (ERR_OK != resu) {
            print_error("Unable to retrieve RSA key\n");
            return resu;
        }

        if (SCP_PP_PAOLA_4096 == config_g.pp &&
            config_g.rsaKey.keyPr.modulus_length != RSA_4096_MODULUS_LEN) {
            print_error("Protection Profile selected RSA-4096 but key length is %d bits.\n",
                        config_g.rsaKey.keyPr.modulus_length * 8);
            return ERR_INCONSISTENT_KEY;
        }

        if (SCP_PP_PAOLA_2048 == config_g.pp &&
            config_g.rsaKey.keyPr.modulus_length != RSA_2048_MODULUS_LEN) {
            print_error("Protection Profile selected RSA-2048 but key length is %d bits.\n",
                        config_g.rsaKey.keyPr.modulus_length * 8);
            return ERR_INCONSISTENT_KEY;
        }

        if (config_g.pp != SCP_PP_RSA && config_g.pp != SCP_PP_PAOLA_4096 &&
            config_g.pp != SCP_PP_PAOLA_2048) {
            print_warn("SCP session mode is defined with this value: \"%s\".\n",
                       mode_name[config_g.session_mode]);
            print_warn("But the protection profile (\"pp\" value) is different from \"RSA_2048\" "
                       "or  \"RSA_4096\".\n");
        }
    }

    if (SCP_FLORA_RSA == config_g.session_mode) {
        max_chunk_size = FLORA_MAX_CHUNK_SIZE;
    }
    if (SCP_ANGELA_ECDSA == config_g.session_mode) {
        max_chunk_size = ANGELA_MAX_CHUNK_SIZE;
    }
    if (SCP_PAOLA == config_g.session_mode) {
        max_chunk_size = PAOLA_MAX_CHUNK_SIZE;
    }
    if (SCP_LITE_ECDSA == config_g.session_mode) {
        max_chunk_size = SCP_LITE_MAX_CHUNK_SIZE;
    }
    if (MSP_MAXQ1852_ECDSA == config_g.session_mode) {
        max_chunk_size = MAXQ1852_MAX_CHUNK_SIZE;
    }
    if (config_g.chunk_size > max_chunk_size) {
        print_warn("Provided chunk_size (" SSIZET_FMT
                   " bytes) is larger than the supported max chunk_size (" SSIZET_FMT " bytes)\n",
                   config_g.chunk_size, max_chunk_size);
        print_warn("Using Maximum chunk size (" SSIZET_FMT " bytes).\n", max_chunk_size);
        config_g.chunk_size = max_chunk_size;
    }

    return ERR_OK;
}

int load_default_config(void)
{
    int i;
    u8 usn_default[] = { 0x04, 0x00, 0x43, 0x47, 0x1f, 0xd2, 0x03, 0x08,
                         0x0c, 0x07, 0x00, 0x00, 0x7f, 0x24, 0xea, 0x2f };
    config_g.session_mode = SCP_RSA;
    /* default config */
    config_g.pp = SCP_PP_RSA;
    /* flash size is 32MB */
    config_g.flash_mb = 32;
    config_g.address_offset = 0;
    config_g.chunk_size = 1024;

    for (i = 0; i < USN_LEN; i++) { config_g.usn[i] = usn_default[i]; }

    sprintf(config_g.script_file, "script.txt");
    sprintf(config_g.output_file, "session.txt");

    return ERR_OK;
}

static int handler_configfile(void *user, const char *section, const char *name, const char *value)
{
    UNUSED_PARAMETER(user);
    UNUSED_PARAMETER(section);

    print_debug("\tName %s Value %s \n", name, value);
    parse_store_option(name, value);

    return 1;
}

int load_ini_config(void)
{
    if (ini_parse(INIFILE, handler_configfile, NULL) < 0) {
        return ERR_OK;
    }

    print_info("Loading configuration from %s \n", INIFILE);

    return ERR_OK;
}

void display_config(void)
{
    unsigned int i = 0;
    print_debug("Config:\n");
    print_d("\tSession mode : %s\n", mode_name[config_g.session_mode]);
    print_d("\tProtection Profile mode : %s\n", pp_name[config_g.pp]);
    print_d("\tOutput file: %s\n", config_g.output_file);
    print_d("\tTransaction ID: %08x\n", config_g.msp_1852_tr_id);
    print_d("\tFlash maximal size: %dMBytes\n", config_g.flash_mb);
    print_d("\tScript file:<%s>\n", config_g.script_file);
    print_d("\tChunk size: " SSIZET_FMT "\n", config_g.chunk_size);
    print_d("\tAddress offset: %08x\n", config_g.address_offset);

    print_d("\tUSN:");
    for (i = 0; i < 16; i++) { print_d("%02x", config_g.usn[i]); }
    print_d("\n");

#ifdef _MAXIM_HSM
    print_d("\tHSM slot: %d\n", config_g.hsm_slot_nb);
    print_d("\tHSM Thales DLL: %s\n", config_g.hsm_thales_dll);
    print_d("\tHSM key name: %s\n", config_g.HSM_KeyLabel);
#endif /* _MAXIM_HSM */
}

int check_parameters(void)
{
    if (!file_exist(config_g.keyfile)) {
        print_error("Key file : %s not found.\n", config_g.keyfile);
        return ERR_FILE_NOT_FOUND;
    }

    if (!file_exist(config_g.script_file)) {
        print_error("Script file %s not found.\n", config_g.script_file);
        return ERR_FILE_NOT_FOUND;
    }

#ifdef _MAXIM_HSM
    if (config_g.hsm) {
        if (!file_exist(config_g.hsm_thales_dll)) {
            print_error("Thales HSM DLL File %s not found.\n", config_g.hsm_thales_dll);
            return ERR_FILE_NOT_FOUND;
        }

        if (strcmp(config_g.HSM_KeyLabel, "") == 0) {
            print_error("HSM Keyname not set.\n");
            return ERR_BAD_PARAMETER;
        }
    }
#endif /*_MAXIM_HSM */

    /* check if requested size is not too large compared to what supported (i.e. MAX_FLASH_MB), because of int coding) */
    if (config_g.flash_mb > MAX_FLASH_MB) {
        print_error("Requested flash size is too large (%dMB) while limited to %dMB\n",
                    config_g.flash_mb, MAX_FLASH_MB);
        return ERR_BAD_PARAMETER;
    }

    return ERR_OK;
}

static int handler(void *user, const char *section, const char *name, const char *value)
{
    char *chip = (char *)user;
    char *new_str;
    if (strcmp(section, chip) == 0) {
        new_str = str_replace(value, "%MAXIM_SBT_DIR%", config_g.fullpath);
        parse_store_option(name, new_str);
        free(new_str);
    }
    if (strcmp(section, "HSM") == 0) {
        parse_store_option(name, value);
    }

    return 1;
}

int load_device_config(char *device_name)
{
    char device_ini_path[500];

    print_debug("Retrieve Config for default device \n");
    if (strcmp(device_name, "") == 0) {
        char *default_device = getenv("MAXIM_SBT_DEVICE");
        if (default_device == NULL) {
            return ERR_OK;
        }
        strcpy(device_name, default_device);
    }
    print_debug("Default device : %s \n", device_name);

    char *envpath = getenv("MAXIM_SBT_DIR");
    if (envpath == NULL) {
        return ERR_NO_DEFAULT_CONFIG_DIR;
    }
    strcpy(config_g.fullpath, envpath);
    strcpy(device_ini_path, envpath);
    strcat(device_ini_path, "\\device.ini");

    print_info("Maxim SBT folder %s \n", config_g.fullpath);

    if (ini_parse(device_ini_path, handler, device_name) < 0) {
        print_warn("Unable to load default config from 'device.ini'\n");
        return ERR_PARSE_INI;
    }

    print_info("Loading device configuration for %s \n", device_name);

    return ERR_OK;
}

int parse_script(script_cmd_list_t **script, size_t *list_length)
{
    if (MSP_MAXQ1852_ECDSA == config_g.session_mode) {
        return ERR_OK;
    }

    if (SCP_LITE_ECDSA == config_g.session_mode) {
        return ERR_OK;
    }

    return parse_scp_script(config_g.script_file, script, list_length);
}

int process(script_cmd_list_t *script, size_t list_length)
{
    /* dynamic allocation of *addr, which contains the addresses of meaningful binary file bytes */
    addr_g = malloc(sizeof(size_t) * 1024 * 1024 * config_g.flash_mb);
    if (NULL == addr_g) {
        print_error("Unable to allocate memory for Address array (%dMB requested)\n",
                    config_g.flash_mb);
        return ERR_MEMORY_ERROR;
    }

    if (strcmp(config_g.output_dir, "") != 0) {
        make_dir(config_g.output_dir);
    }

    print_debug("Session mode : %s\n", mode_name[config_g.session_mode]);
    fprintf(fp_g, "Session mode : %s\n", mode_name[config_g.session_mode]);

    if (MSP_MAXQ1852_ECDSA == config_g.session_mode) {
        return process_script_maxq1852_ecdsa();
    }

    if (SCP_LITE_ECDSA == config_g.session_mode) {
        return process_script_scp_lite_ecdsa();
    }

    return process_script(script, list_length);
}

void print_help(void)
{
    int i;

    print_i("Usage: build_scp_session [OPTION] [PARAMETERS]... [ FOLDER [EXTRA_PARAM]... ]\n");
    print_i("Generate an SCP session according to the script provided or the default one.\n");

    print_i("\nOPTIONS:\n");
    print_i("  -h				Print this help and quit.\n");
    print_i("  -v				output software and libraries versions and quit.\n");
    print_i("  -d				Activate debug output.\n");
    print_i("  -c CHIP_NAME		Use default configuration of CHIP_NAME.\n");

    print_i("\nPARAMETERS:\n");
    print_i("\tParameters are used by priority in the following order :\n");
    print_i("\t\t1) Command line\n");
    print_i("\t\t2) Configuration file " INIFILE " in the current folder\n");
    print_i("\t\t3) Chip default parameters selected by the -c option or the MAXIM_SBT_DEVICE env "
            "variable.\n");
    print_i("\t\t4) Software default values.\n");
    print_i("\n");
    print_i("\t\n");

    print_i("\tkey_file\tfile\tUCL format private key file for SCP packet signing  \n");
    print_i("\tverbose\tlevel\tverbose level (0-5)\n");
    print_i("\tsession_mode\tmode\tSCP session mode : ");

    i = 0;
    while (mode_name[i] != NULL) {
        if (mode_name[i][0] != 0) {
            print_i("%s, ", mode_name[i]);
        }
        i++;
    }
    print_i("\n");

    print_i("\tpp\t\tPP\tSCP Protection profile to use : ");
    i = 0;
    while (pp_name[i] != NULL) {
        if (pp_name[i][0] != 0) {
            print_i("%s, ", pp_name[i]);
        }
        i++;
    }
    print_i("\n");

    print_i("\tscript_file\tfile\ttext file containing SCP operation to perform\n");
    print_i("\toutput_file\tname\tfilename prefix of SCP file generated\n");
    print_i("\toutput_dir\tdir\tfolder wher SCP files will be saved\n");

#ifdef _MAXIM_HSM
    print_i("\nHSM:\n");
    print_i(
        "\thsm\t\tyes,no\tUse or not an HSM to manage key and perform cryptographics operations\n");
    print_i("\thsm_key_name\tname\tname of the key in the HSM to use\n");
    print_i("\thsm_thales_dll\tdll_path\tpath to the Thales cknfast DLL\n");
    print_i("\thsm_slot_nb\tnb\tnumber of the HSM slot to use (usually : 1)\n");
#endif

    print_i("\tchunk_size\tsize\tmaximum data size for one SCP packet (in bytes)\n");
    print_i("\tflash_size_mb\tsize\tmaximum flash size in Mo\n");
    print_i("\tusn\t\tUSN\tUnique Serial Number of the device\n");
    print_i(
        "\ttransaction_id\ttr_id\tUser Selected transaction ID when using MSP_MAXQ1852_ECDSA\n");
    print_i("\taddr_offset\taddr\taddress offset when reading S19 or S20 files\n");

    print_i("\nSUPPORTED COMMAND :\n");

    for (i = 0; i < MAX_SCP_COMMAND; i++) {
        int pm;

        print_i("\t%-40s", idf_scp_cmd[i].name);

        pm = (mode[i] & SCP_RSA_MSK) == SCP_RSA_MSK;
        print_cr(" %-7s ", pm ? "SCP-RSA" : "");

        pm = (mode[i] & SCP_FLORA_RSA_MSK) == SCP_FLORA_RSA_MSK;
        print_cy(" %-9s ", pm ? "SCP-FLORA" : "");

        pm = (mode[i] & SCP_ANGELA_ECDSA_MSK) == SCP_ANGELA_ECDSA_MSK;
        print_cb(" %-9s ", pm ? "SCP-ECDSA" : "");

        pm = (mode[i] & SCP_PAOLA_MSK) == SCP_PAOLA_MSK;
        print_cw(" %-9s ", pm ? "SCP-PAOLA" : "");

        pm = (mode[i] & MSP_MAXQ1852_ECDSA_MSK) == MSP_MAXQ1852_ECDSA_MSK;
        print_cw(" %-18s ", pm ? "MSP_MAXQ1852_ECDSA" : "");

        print_i("\n");
    }
}

void print_version(void)
{
    print_info("Libs version :\n");
#ifdef _MAXIM_HSM
    print_i("\t - LibHSM Version: %s (%s)\n", HSM_GetVersion(), HSM_GetBuildDate());
#endif
    print_i("\t - UCL Version: %s (%s)\n", ucl_get_version(), ucl_get_build_date());
}

int main(int argc, char **argv)
{
    int result;
    char c;
    char default_device[MAX_STRING] = "";
    char output_name[MAX_STRING + 5];
    verbose = 3;
    script_cmd_list_t *script = NULL;
    size_t list_length;

#ifdef __WIN
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO info;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &info);
    bg_color_g = info.wAttributes & 0xF0;
#endif

    while ((c = getopt(argc, argv, "dhvc:")) != -1) {
        switch (c) {
        case 'c':
            strcpy(default_device, optarg);
            break;
        case 'h':
            print_help();
            return ERR_OK;
            break;
        case 'v':
            print_version();
            return ERR_OK;
            break;
        case 'd':
            verbose = 5;
            break;
        case '?':
            if (optopt == 'c') {
                fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            } else if (isprint(optopt)) {
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            } else {
                fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
            }
            return 1;
        default:
            abort();
        }
    }
    print_info("SBL/SCP packets builder v%d.%d.%d (%s %s) (c)Maxim Integrated 2006-2018\n", MAJV,
               MINV, ZVER, __DATE__, __TIME__);

    load_default_config();

    ASSERT_OK(load_device_config(default_device));

    ASSERT_OK(load_ini_config());

    ASSERT_OK(load_args(argc, argv, optind));

    ASSERT_OK(check_parameters());

    display_config();

    ASSERT_OK(init_crypto());

#ifdef _MAXIM_HSM
    if (config_g.hsm) {
        print_info("This tool does handle keys with Thales nCipher Edge HSM \n");
    } else
#endif /*_MAXIM_HSM */
    {
        print_warn("This tool does not handle keys in a PCI-PTS compliant way, only for test \n");
    }

    ASSERT_OK(parse_script(&script, &list_length));

#ifdef _MAXIM_HSM
    if (config_g.hsm) {
        ASSERT_OK(hsm_login());
    }
#endif /* _MAXIM_HSM */

    print_info("Loadings Keys \n");

    ASSERT_OK(load_keys());

    snprintf(output_name, MAX_STRING + 5, "%s.log", config_g.output_file);
    fp_g = fopen(output_name, "w");
    if (NULL == fp_g) {
        print_error("on opening log file %s : %s\n", output_name, strerror(errno));
        return ERR_FILE_ERROR;
    }

    fprintf(fp_g, "session generation v%d.%d.%d (%s %s) (c)Maxim Integrated 2006-2018\n", MAJV,
            MINV, ZVER, __DATE__, __TIME__);

    print_info("Processing SCP script %s \n", config_g.script_file);
    result = process(script, list_length);

    (void)fclose(fp_g);

#ifdef _MAXIM_HSM
    if (config_g.hsm)
        hsm_close();
#endif /* _MAXIM_HSM */

    if (ERR_OK != result) {
        print_error("SCP session process failed exiting...\n");
        return result;
    }

    print_debug("Log file <%s> created\n", output_name);
    print_success("SCP session process finished with success\n");

    return (ERR_OK);
}
