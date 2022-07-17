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
* @contributor: Emmanuel Puerto <emmanuel.puerto@maximintegrated.com>
*
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <ctype.h>
#include <errno.h>
#include <regex.h>

#include <ucl/ucl.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_info.h>
#include <ucl/ucl_sha256.h>

#include <ca_sign_build.h>
#include <hsm.h>
#include <ini.h>
#include <utils.h>
#include <read_file.h>

#include <maxim_c_utils.h>

#ifndef MAX_ARG_LEN
#define MAX_ARG_LEN			4096
#endif

u32 ucl_buffer_g[4096];

const uint8_t flora_magic_c[] = {0x59, 0x45, 0x53, 0x57, 0x45, 0x43, 0x41, 0x4E};
const uint8_t magic_c[] = { 0x48, 0x49, 0x53, 0x57, 0x45, 0x44, 0x47, 0x44};

const char * algo_name[]={"rsa", "rsa_paola", "sha256", "crc32", "none", "ecdsa", NULL};


config_option_t config_option[] = {
		{"ca", OT_FILE,  &config_g.cafile, 0, 0},
		{"sca", OT_FILE,  &config_g.scafile, 0, 0},

		{"algo", OT_ALGO,  &config_g.algo, 0, 0},
		{"key_file", OT_FILE,  &config_g.keyfile, 0, 0},
		/* Legacy */
		{"ecdsa_file", OT_FILE,  &config_g.keyfile, 0, 0},
		{"rsa_file", OT_FILE,  &config_g.keyfile, 0, 0},

		{"signonly", OT_YESNO,  &config_g.signonly, 0, 0},
		{"header", OT_YESNO,  &config_g.headergenerated, 0, 0},

		{"verbose", OT_INT,  &config_g.verbose, 0, 0},

		{"application_version", OT_LONGHEX,  &config_g.application_version, 0, 0},
		{"load_address", OT_LONGHEX,  &config_g.load_address, 0, 0},
		{"jump_address", OT_LONGHEX,  &config_g.jump_address, 0, 0},
		{"rom_version", OT_LONGHEX,  &config_g.version, 0, 0},
		{"arguments", OT_STRING,  &config_g.arguments, 0, 0},
		{"boot_method", OT_BOOTMETHOD,  &config_g.boot_method, 0, 0},
#ifdef _MAXIM_HSM
		{"hsm", OT_YESNO,  &config_g.hsm, 0, 0},
		{"hsm_key_name", OT_STRING,  &config_g.HSM_KeyLabel, 0, 0},
		{"hsm_thales_dll", OT_FILE,  &config_g.hsm_thales_dll, 0, 0},
		{"hsm_slot_nb", OT_INT,  &config_g.hsm_slot_nb, 0, 0},
#endif
		{"sr_papd", OT_HEX,  &config_g.sr_papd, 0, 0},
		{"sr_pext", OT_HEX,  &config_g.sr_pext, 0, 0},
		{"sr_prfsh", OT_LONGHEX,  &config_g.sr_prfsh, 0, 0},

		{"sr_pcfg", OT_LONGHEX,  &config_g.sr_pcfg, 0, 0},
		{"dmc_gcfg", OT_LONGHEX,  &config_g.dmc_gcfg, 0, 0},
		{"dmc_clk", OT_HEX,  &config_g.dmc_clk, 0, 0},
		{"uci0_ksrc_configencint", OT_HEX,  &config_g.uci0_ksrc_configencint, 0, 0},
		{"uci0_ac1r_so", OT_LONGHEX,  &config_g.uci0_ac1r_so, 0, 0},
		{"uci0_ac1r_eo", OT_LONGHEX,  &config_g.uci0_ac1r_eo, 0, 0},
		{"uci0_ddr_r0", OT_LONGHEX,  &config_g.uci0_ddr_r0, 0, 0},
		{0, 0, 0, 0, 0},
};


int parse_store_option (const char * name, const char * value)
{
	unsigned int i = 0;
	int result = 0;

	while(config_option[i].name != 0){
		if(strcmp(config_option[i].name, name) == 0){
			ASSERT_OK(parse_store (config_option[i].type, config_option[i].ptr,value, config_option[i].min));
		}
		i++;
	}

	print_debug("Bad Parameter %s=%s \n", name, value);

	return ERR_OK;
}


int hash_sign_payload(uint8_t * hash, const uint8_t * payload, size_t payload_len)
{
	unsigned int i;
	int result;

	print_debug("SHA-256 Hash computation\n");

	/** Set up data to be signed */
	result = ucl_sha256 (hash, (uint8_t *) payload, payload_len);
	if (UCL_OK != result)
	{
		print_error("Hash sha256 failed\n");
		return result;
	}

	print_debug("hash: ");
	for (i = 0; i < UCL_SHA256_HASHSIZE; i++)
	{
		print_d ("%02x", hash[i]);
	}
	print_d ("\n");

	return ERR_OK;
}


int crc32_sign_payload (uint32_t * p_crc, const uint8_t * payload, size_t payload_len)
{
	const unsigned int Polynomial = 0xEDB88320;
	volatile unsigned char *p_current;
	unsigned int i;


	/* Check inputs */
	if (p_crc == NULL || payload == NULL)
	{
		print_error("Null Pointer\n");
		return ERR_NULL_POINTER;
	}

	/* Assign input pointer to work pointer */
	p_current = (volatile unsigned char *) payload;
	*p_crc = 0;

	/* Compute CRC */
	while (payload_len--)
	{
		*p_crc ^= *p_current++;
		for (i = 0; i < 8; i++)
		{
			if (*p_crc & 1)
			{
				*p_crc = (*p_crc >> 1) ^ Polynomial;
			}
			else
			{
				*p_crc = *p_crc >> 1;
			}
		}
	}

	print_debug ("\ncrc32: 0x%08x\n", *p_crc);

	return ERR_OK;
}


int check_parameters(void){
	char *ptr;


	if(!file_exist(config_g.keyfile)){
		print_error("File %s not found.\n", config_g.keyfile);
		return ERR_FILE_NOT_FOUND;
	}


	if(!file_exist(config_g.cafile)){
		print_error("File %s not found.\n", config_g.cafile);
		return ERR_FILE_NOT_FOUND;
	}


#ifdef _MAXIM_HSM
	if(config_g.hsm){
		if(!file_exist(config_g.hsm_thales_dll)){
			print_error("Thales HSM DLL File %s not found.\n", config_g.hsm_thales_dll);
			return ERR_FILE_NOT_FOUND;
		}


		if(strcmp(config_g.HSM_KeyLabel,"") == 0){
			print_error("HSM Keyname not set.\n");
			return ERR_BAD_PARAMETER;
		}
	}
#endif	/*_MAXIM_HSM */

	if(strcmp(config_g.scafile,"") ==0 ){
		strlcpy(config_g.scafile, config_g.cafile, MAX_STRING);
			ptr = strrchr(config_g.scafile,'.');
			if (NULL != ptr) {
				strlcpy(ptr, ".sbin", MAX_STRING);
			} else {
				strcat(config_g.scafile, ".sbin");
			}
	}


	strlcpy(config_g.sigfile, config_g.scafile, MAX_STRING);
	ptr = strrchr(config_g.sigfile,'.');
	if (NULL != ptr) {
		strlcpy(ptr, ".sig", MAX_STRING);
	} else {
		strcat(config_g.sigfile, ".sig");
	}

	return ERR_OK;
}


int write_file (const char * filename, const uint8_t * payload, size_t payload_len)
{
	FILE *pFile;
	size_t resu;

	if((filename == NULL) || (strlen(filename) == 0))
	{
		print_error("Invalid Filename\n");
		return ERR_BAD_PARAMETER;
	}

	if((payload == NULL) || (payload_len == 0))
	{
		print_error("Null pointer\n");
		return ERR_NULL_POINTER;
	}

	pFile = fopen (filename, "wb");
	if (pFile == NULL)
	{
		print_error("on opening %s : %s\n", filename, strerror(errno));
		return errno;
	}


	resu = fwrite (payload, sizeof (u8), payload_len, pFile);
	if (payload_len != resu)
	{
		print_error ("on writing payload in file %s\n", filename);
		return ERR_FILE_ERROR;
	}


	fclose(pFile);
	
	return ERR_OK;
}


int generate_header(u8 * payload, size_t payload_len, size_t * payload_offset)
{
	unsigned int i;
	size_t header_len;
	int offset;
	size_t arguments_len;

	/* Start with magic word */
	switch(config_g.algo){
		case a_rsa: /* flora */
			header_len = FLORA_HEADER_LEN;
			memcpy(payload, flora_magic_c, 8);

			break;
	
		case a_rsa_paola:
			header_len = ANGELA_HEADER_LEN;
			memcpy(payload, magic_c, 8);

			if ( config_g.rsaKey.keyPr.modulus_length == RSA_4096_MODULUS_LEN )
			{
				payload[0] = 0x47;
			}
			else if (config_g.rsaKey.keyPr.modulus_length == RSA_2048_MODULUS_LEN )
			{
				payload[0] = 0x46;
			}
			
			if (config_g.boot_method == bm_cmsis)
			{
				payload[0] |= 0xb0;	/*  0xf6 to indicate that ROM Code shall boot CMSIS way */
			}

			break;
		case a_sha256:	/* paola sha256 */
			header_len = ANGELA_HEADER_LEN;
			memcpy(payload, magic_c, 8);

			payload[0] = 0x49;

			if (config_g.boot_method == bm_cmsis)
			{
				payload[0] |= 0xb0; /*  0xf9 to indicate that ROM Code shall boot CMSIS way */
			}

			break;
	
		case a_crc32 :	/* paola crc */
			header_len = ANGELA_HEADER_LEN;
			memcpy(payload, magic_c, 8);

			payload[0] = 0x44;

			if (config_g.boot_method == bm_cmsis)
			{
				payload[0] |= 0xb0;			/*  0xf4 to indicate that ROM Code shall boot CMSIS way */
			}

			break;

		case a_none: /* paola none */
			header_len = ANGELA_HEADER_LEN;
			memcpy(payload, magic_c, 8);

			payload[0] = 0xff;

			break;
			
		case a_ecdsa:	/* angela/lhassa/clara/maria */
			header_len = ANGELA_HEADER_LEN;
			memcpy(payload, magic_c, 8);

			/*
			 *  There is one exception for MAX32570, sync byte is swapped
			 *  i do not know why it has been done, found it in ELSA ROM Code
			 *  0x44 means K_RCE_SECTION_H_DIGITAL_KEY_ECDSA256 for MAX32570 not CRC32
			 */
			if ( strcmp(config_g.device, "MAX32570") == 0 ) {
				payload[0] = 0x44; //
			}

			if (config_g.boot_method == bm_cmsis)
			{
				payload[0] |= 0xb0;			/*  0xf8 to indicate that ROM Code shall boot CMSIS way */
			}

			break;

		default:
			print_error ("Bad Algorithm %d\n",config_g.algo);
			return ERR_UNKNOWN_ALGO;
	}
	
	offset = 8;

	/* Version */
	for (i = 0; i < HEADER_VERSION_LEN; i++){
		payload[offset++] = config_g.version >> ((HEADER_VERSION_LEN - i - 1) * 8) & 0xff;
	}

	/* Load Address */
	for (i = 0; i < HEADER_LOAD_ADDRESS_LEN; i++){
		payload[offset++] = config_g.load_address >> ((HEADER_LOAD_ADDRESS_LEN - i - 1) * 8) & 0xff;
	}

	/* Payload Length */
	for (i = 0; i < HEADER_BINARY_LEN; i++){
		payload[offset++] = (payload_len >> ((HEADER_BINARY_LEN - i - 1) * 8)) & 0xff;
	}

	/* Jump Address */
	for (i = 0; i < HEADER_JUMP_ADDRESS_LEN; i++){
		payload[offset++] = config_g.jump_address >> ((HEADER_JUMP_ADDRESS_LEN - i - 1) * 8) & 0xff;
	}

	/* Arguments */
	arguments_len = strlen(config_g.arguments);

	for (i = 0; i < HEADER_ARGV_LEN; i++){
		payload[offset++] = arguments_len >> ((HEADER_ARGV_LEN - i - 1) * 8) & 0xff;
	}

	/* Specific parameters */
	switch(config_g.algo){
		case a_rsa: /* flora */

			payload[offset++] = config_g.sr_papd;

			for (i = 0; i < HEADER_SR_PRFSH_LEN; i++, offset++){
				payload[offset] = config_g.sr_prfsh >> ((HEADER_SR_PRFSH_LEN - i - 1) * 8) & 0xff;
			}

			for (i = 0; i < HEADER_SR_PCFG_LEN; i++){
				payload[offset++] = config_g.sr_pcfg >> ((HEADER_SR_PCFG_LEN - i - 1) * 8) & 0xff;
			}

			payload[offset++] = config_g.sr_pext;

			for (i = 0; i < HEADER_DMC_GCFG_LEN; i++){
				payload[offset++] = config_g.dmc_gcfg >> ((HEADER_DMC_GCFG_LEN - i - 1) * 8) & 0xff;
			}

			payload[offset++] = config_g.dmc_clk;
			payload[offset++] = config_g.uci0_ksrc_configencint;

			for (i = 0; i < HEADER_UCI0_AC1R_START_OFFSET_LEN; i++){
				payload[offset++] = config_g.uci0_ac1r_so >> ((HEADER_UCI0_AC1R_START_OFFSET_LEN - i - 1) * 8) & 0xff;
			}

			for (i = 0; i < HEADER_UCI0_AC1R_END_OFFSET_LEN; i++){
				payload[offset++] = config_g.uci0_ac1r_eo >> ((HEADER_UCI0_AC1R_END_OFFSET_LEN - i - 1) * 8) & 0xff;
			}

			for (i = 0; i < HEADER_UCI0_DDR_R0_LEN; i++){
				payload[offset++] = config_g.uci0_ddr_r0 >> ((HEADER_UCI0_DDR_R0_LEN - i - 1) * 8) & 0xff;
			}

			break;
			
		case a_ecdsa:
		case a_sha256:
		case a_crc32:
		case a_none:
		case a_rsa_paola:
		
			for (i = 0; i < HEADER_APPLICATION_VERSION_LEN; i++)
			{
				payload[offset++] = config_g.application_version >> ((HEADER_APPLICATION_VERSION_LEN - i - 1) * 8) & 0xff;
			}
			break;

		default:
	
			print_error ("Bad Algorithm %d\n",config_g.algo);
			return ERR_UNKNOWN_ALGO;
			break;
	}
	
	print_debug ("Header (Length "SSIZET_FMT") :", header_len);
	for (i = 0; i < header_len; i++){
		print_d ("%02x", payload[i]);
	}
	print_d ("\n");
	

	/* put the arguments in the empty space */
	for (i = 0; i < arguments_len; i++){
		payload[i] = config_g.arguments[i];
	}
	
	*payload_offset = offset + arguments_len;

	return ERR_OK;
}



int load_keys(void){

	int resu = 0;
	
	switch(config_g.algo){
		case a_rsa: 
		case a_rsa_paola:
	
		#ifdef _MAXIM_HSM
			if(config_g.hsm){
				resu = load_HSM_rsa_key (&config_g.rsaKey, config_g.HSM_KeyLabel);
			}else
		#endif /* _MAXIM_HSM */
			{
				resu = read_file_rsa_keypair(&config_g.rsaKey, config_g.keyfile);
			}

			if(ERR_OK != resu)
			{
				print_error("Unable to retrieve RSA key\n");
				return resu;
			}
			break;
			
		case a_ecdsa:

		#ifdef _MAXIM_HSM
			if(config_g.hsm){
				resu = load_HSM_ecdsa_key(&config_g.ecdsaKey, config_g.HSM_KeyLabel);
			}else
		#endif /* _MAXIM_HSM */
			{
				resu = read_file_ecdsa_keypair(&config_g.ecdsaKey, config_g.keyfile);
			}

			if (ERR_OK != resu)
			{
				print_error("Unable to retrieve ECDSA key\n");
				return resu;
			}

			break;
			
		case a_sha256:
			/* Hard-coded Signature length when signing */
			break;
	
		case a_crc32:
			/* Hard-coded Signature length when signing */
			break;
	
		case a_none:
			/* Hard-coded Signature length when signing */
			break;

		default:
			print_error("Unknown signature algorithm\n");
			return ERR_UNKNOWN_ALGO;
			break;
	}

	return ERR_OK;
}



int sign_payload(const uint8_t * payload, size_t payload_len, uint8_t * signature, size_t * signature_len){

	int result;

	print_info("Algorithm : %s \n",algo_name[config_g.algo]);

	switch(config_g.algo){
		case a_rsa:
		case a_rsa_paola:

			ASSERT_OK(rsa_sign(payload, payload_len, signature, config_g.rsaKey));
			*signature_len = config_g.rsaKey.keyPu.modulus_length;
			break;

		case a_ecdsa:

			ASSERT_OK(ecdsa_sign (payload, payload_len, signature, config_g.ecdsaKey));
			*signature_len = config_g.ecdsaKey.ecdsa_len * 2;
			break;

		case a_sha256:

			ASSERT_OK(hash_sign_payload((unsigned char *) signature, payload, payload_len));
			*signature_len = UCL_SHA256_HASHSIZE;
			break;

		case a_crc32:

			ASSERT_OK(crc32_sign_payload((unsigned int *) signature, payload, payload_len));
			*signature_len = 4;
			break;

		case a_none:

			print_info("No signature therefore no length.\n");
			*signature_len = 0;
			break;

		default:
			print_error("Unknown Signing algorithm ID %d\n", config_g.algo);
			return ERR_UNKNOWN_ALGO;
			break;
	}

	return ERR_OK;
}




int process (void)
{
	int result = ERR_OK;
	size_t signature_len = 0;
	size_t payload_len;
	size_t offset = 0;
	size_t signing_length = 0;
	unsigned char *payload;
	unsigned char signature[MAX_RSA_LENGTH];

	ASSERT_OK(load_keys());

	/* Read file size */
	ASSERT_OK(read_file_size(&payload_len, config_g.cafile));

	/* Memory space allocation */
	payload = (u8 *) malloc ((MAX_HEADER_LEN + payload_len + strlen (config_g.arguments)) * sizeof (u8) + MAX_RSA_LENGTH);
	if (NULL == payload)
	{
		print_error ("Unable to allocate memory for payload ("SSIZET_FMT" bytes required)\n", payload_len);
		return ERR_MEMORY_ERROR;
	}
	
	/* If asked generate header */
	if (config_g.headergenerated)
	{
		print_info("Generating Header\n");
		ASSERT_OK(generate_header(payload, payload_len, &offset));
	}

	/* Read binary File */
	ASSERT_OK(read_binary_file(config_g.cafile, &payload[offset], &payload_len));

	signing_length = payload_len + offset;

	/* Signing Payload */
	print_info("Signing Payload\n");
	ASSERT_OK(sign_payload(payload, signing_length, signature, &signature_len));

	
	
	print_info("Writing Signature file\n");

	ASSERT_OK(write_file(config_g.sigfile, signature, signature_len));


	if (!config_g.signonly)
	{
		memcpy(&payload[signing_length], signature, signature_len);

		print_info("Writing Complete Binary file ( header + payload + Signature )\n");
		ASSERT_OK(write_file (config_g.scafile, payload, signing_length + signature_len));
	}
	
	/* Free memory */
	free (payload);

	return ERR_OK;
}

int load_args (int argc, char **argv, int start_index)
{
	int k;
	int result;
	int extra_arg_idx = 0;

	for (k = start_index; k < argc; k++)
	{
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
			if(memcpyLen > MAX_ARG_LEN) {
				print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
				return ERR_MEMORY_ERROR;
			}
			memcpy(name, &(argv[k][rm[1].rm_so]), memcpyLen);
			name[rm[1].rm_eo - rm[1].rm_so] = '\0';

			memcpyLen = rm[2].rm_eo - rm[2].rm_so;
			if(memcpyLen > MAX_ARG_LEN) {
				print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
				return ERR_MEMORY_ERROR;
			}
			memcpy(value, &argv[k][rm[2].rm_so], memcpyLen);
			value[rm[2].rm_eo - rm[2].rm_so] = '\0';
			ASSERT_OK(parse_store_option (name, value));
		}else {
			if(extra_arg_idx == 0){
				parse_store_option ("ca", argv[k]);
				extra_arg_idx++;
			}else if(extra_arg_idx == 1){
				parse_store_option ("key_file", argv[k]);
				extra_arg_idx++;
			}else {
				print_warn("bad parameter %s \n", argv[k]);
			}
		}

	    regfree(&regex);
	}


	return ERR_OK;
}


void load_default_config (void)
{
	// no default device
	config_g.device[0] = '\0';
	sprintf (config_g.cafile, "file.bin");
	config_g.scafile[0] = '\0';
	sprintf (config_g.keyfile, "keyfile.key");
	config_g.boot_method = bm_direct;
	config_g.headergenerated = TRUE;
	config_g.signonly = FALSE;
	config_g.load_address = 0x01020304;
	config_g.jump_address = 0x02030405;
	config_g.arguments[0] = '\0';
	config_g.algo = a_rsa;
	config_g.version = 0x010203ff;
	config_g.sr_papd = 0xac;
	config_g.sr_prfsh = 0xabcdef01;
	config_g.sr_pcfg = 0x01efcdab;
	config_g.sr_pext = 0xa3;
	config_g.dmc_gcfg = 0x02030405;
	config_g.dmc_clk = 0x04;
	config_g.uci2_ctrl_reg = 0x040506cd;
}


static int handler_configfile(void* user, const char* section, const char* name, const char* value)
{
	UNUSED_PARAMETER(user);

	print_debug("Section %s Name %s Value %s \n", section, name, value);
   	parse_store_option(name, value);

    return 1;
}


/* this function reads the .ini and configures the parameters */
int load_ini_config (void)
{
	if (ini_parse(INIFILE, handler_configfile, NULL) < 0) {
		        return ERR_OK;
	}

	print_info ("Loading configuration from %s \n", INIFILE);

	return ERR_OK;
}


void display_config (void)
{
	print_debug ("Config :\n");

	if (config_g.signonly){
		print_d ("\tOnly signature file is generated (signonly=yes)\n");
	}else{
		print_d ("\tSigned application file is generated (signonly=no)\n");
	}
	
	if (config_g.headergenerated){
		print_d ("\tThe secure header is generated\n");
	}else{
		print_d ("\tThe secure header is not generated\n");
	}
	
	print_d ("\tBootLoader version:\t %08x\n", config_g.version);
	
#ifdef _MAXIM_HSM
	print_d ("\t%s\n", config_g.hsm ? "HSM cryptography selected" : "Built-in cryptography selected");
	if(config_g.hsm){
		print_d ("\tHSM key name :\t %s\n", config_g.HSM_KeyLabel);
		print_d ("\tHSM Thales DLL path :\t %s\n", config_g.hsm_thales_dll);
		print_d ("\tHSM Slot number :\t %d\n", config_g.hsm_slot_nb);
	}else

#endif /* _MAXIM_HSM */
	{
		print_d ("\tKey file:\t %s\n", config_g.keyfile);
	}

	print_d ("\tCustomer application (input) file:\t %s\n", config_g.cafile);
	print_d ("\tSigned customer application (output) file:\t %s\n", config_g.scafile);
	print_d ("\tBinary load address:\t %08x\n", config_g.load_address);
	print_d ("\tBinary jump address:\t %08x\n", config_g.jump_address);
	print_d ("\tApplication arguments:\t <%s> \n", config_g.arguments);
	print_d ("\tAlgorithm : %s \n",algo_name[config_g.algo]);
	print_d ("\tApplication version: %08x\n", config_g.application_version);
	print_d ("\tBoot Method: %s\n", config_g.boot_method == bm_cmsis ? "cmsis" : "direct");

	print_d("\tMAX32590 Configuration :\n");
	print_d("\t\tSR_PAPD:\t %02x\n", config_g.sr_papd);
	print_d("\t\tSR_PRFSH:\t %08x\n", config_g.sr_prfsh);
	print_d("\t\tSR_PCFG:\t %08x\n", config_g.sr_pcfg);
	print_d("\t\tSR_PEXT:\t %02x\n", config_g.sr_pext);
	print_d("\t\tMEM_GCFG:\t %08x\n", config_g.dmc_gcfg);
	print_d("\t\tDMC_CLK:\t %02x\n", config_g.dmc_clk);
	print_d("\t\tKSRC-Config ENC-INT:\t %02x\n", config_g.uci0_ksrc_configencint);
	print_d("\t\tuci0-AC1R-start-offset:\t %08x\n", config_g.uci0_ac1r_so);
	print_d("\t\tuci0-AC1R-end-offset:\t %08x\n", config_g.uci0_ac1r_eo);
	print_d("\t\tuci0-DDR-r0:\t %08x\n", config_g.uci0_ddr_r0);
}


int init_crypto (void)
{
	int err = 0;

	err = ucl_init (ucl_buffer_g, 4096);

	if (err != UCL_OK)
	{
		print_error("Unable to Initialize UCL (ucl_init %d)\n", err);
		return err;
	}

#ifdef _MAXIM_HSM
	if(config_g.hsm){
		/* Initialise DLL, giving the path of Thales cknfast DLL  */
		err = HSM_InitDLL (config_g.hsm_thales_dll);
		if (err != CKR_OK)
		{
			print_error("Unable to Initialize Thales HSM DLL \n");
			HSM_pError (err);
			return err;
		}
	}
#endif /* _MAXIM_HSM */

	return ERR_OK;
}


static int handler(void* user, const char* section, const char* name, const char* value)
{

	char * chip = (char *) user;
	char * new_str;
    if(strcmp(section, chip) == 0){
    	new_str = str_replace(value, "%MAXIM_SBT_DIR%", config_g.fullpath);
    	parse_store_option(name, new_str);
    	free(new_str);
    }
    if(strcmp(section, "HSM") == 0){
        	parse_store_option(name, value);
    }

    return 1;
}


int load_device_config(void){

	char device_ini_path[500];

	print_debug("Retrieve Config for default device \n");
	if (config_g.device[0] == '\0') { // means no device defined
		char * default_device = getenv ("MAXIM_SBT_DEVICE");
		if(default_device == NULL){
			return ERR_OK;
		}
		strlcpy(config_g.device, default_device, sizeof(config_g.device));
	}
	print_debug("Default device : %s \n", config_g.device);

	char * envpath = getenv ("MAXIM_SBT_DIR");
	if(envpath == NULL){
		return ERR_NO_DEFAULT_CONFIG_DIR;
	}
	strlcpy(config_g.fullpath, envpath, MAX_STRING);
	strlcpy(device_ini_path, envpath, MAX_STRING);
	strcat(device_ini_path,"\\device.ini");

	if (ini_parse(device_ini_path, handler, config_g.device) < 0) {
		print_warn("Unable to load default config from 'device.ini'\n");
	    return ERR_PARSE_INI;
	}

	print_info ("Loading device configuration for %s \n", config_g.device);

	return ERR_OK;
}

void print_help(void){

	int i;

	print_i("Usage: sign_app [OPTION] [PARAMETERS] [APP [KEYFILE]]\n");
	print_i("sign_app build binary file including signature and SLA header from customer application binary file.\n");

	print_i("\nOPTIONS:\n");
	print_i("  -h				Print this help and quit.\n");
	print_i("  -v				output software and libraries versions and quit.\n");
	print_i("  -d				Activate debug output.\n");
	print_i("  -c CHIP_NAME		Use default configuration of CHIP_NAME.\n");

	print_i("\nPARAMETERS:\n");
	print_i("\tParameters are used by priority in the following order :\n");
	print_i("\t\t1) Command line\n");
	print_i("\t\t2) Configuration file " INIFILE " in the current folder\n");
	print_i("\t\t3) Chip default parameters selected by the -c option or the MAXIM_SBT_DEVICE env variable.\n");
	print_i("\t\t4) Software default values.\n");
	print_i("\n");
	print_i("\t\n");


	print_i("\tca\t\tfile\t\tapplication binary file to be signed\n");
	print_i("\tsca\t\tfile\t\toutput signed binary file\n");

	print_i("\talgo\t\talgorithm\talgorithm used to sign the file : ");
	i = 0;
	while(algo_name[i] != NULL){
		if(algo_name[i][0] != 0){
			print_i("%s, ",algo_name[i]);
		}
		i++;
	}
	print_i("\n");
	print_i("\tkey_file\tfile\t\tUCL format private key file for signing application\n");

	print_i("\tsignonly\tyes,no\t\tif yes do not generate binary with appended signature Only .sig file is produce\n");
	print_i("\theader\t\tyes,no\t\tgenerate or not secure header\n");

	print_i("\tverbose\t\tlevel\t\tverbose level (0-5)\n");
	print_i("\n\tHeader field:\n");
	print_i("\tapplication_version\tversion\tapplication version field\n");
	print_i("\tload_address\t\taddress\tbinary load address\n");
	print_i("\tjump_address\t\taddress\tbinary load address\n");
	print_i("\tversion\t\t\tversion\ttargeted Secure boot-loader version\n");
	print_i("\targuments\t\targs\tapplication argument\n");
	print_i("\tboot_method\t\tmethod\tdirect or cmsis boot methode\n");

	print_i("\n\tMAX3259x specific header field:\n");
	print_i("\tsr_papd\t\t\treg\tDMC Primary SDRAM Power down register value\n");
	print_i("\tsr_pext\t\t\treg\tDMC Primary LPDDR Mode register value\n");
	print_i("\tsr_prfsh\t\treg\tDMC Primary SDRAM Refresh register value\n");
	print_i("\tsr_pcfg\t\t\treg\tDMC Primary SDRAM Configuration register value\n");
	print_i("\tdmc_gcfg\t\treg\tDMC Global config register value\n");
	print_i("\tdmc_clk\t\t\treg\tDMC Clock config register value\n");
	print_i("\tuci0_ksrc_configencint\treg\tUCI AES Encryption key 0 register value\n");
	print_i("\tuci0_ac1r_so\t\treg\tUCI Area Config 0 Start Offset register value\n");
	print_i("\tuci0_ac1r_eo\t\treg\tUCI Area Config 0 End Offset register value\n");
	print_i("\tuci0_ddr_r0\t\treg\tUCI DDR Region 0 Config register value\n");
}


int print_version(void){

	print_info("Libs version :\n");

	print_info ("\tUCL Version: %s (%s)\n", (char *) ucl_get_version (), (char *) ucl_get_build_date ());
	#ifdef _MAXIM_HSM
	print_info ("\tLibHSM Version: %s (%s)\n", HSM_GetVersion (), HSM_GetBuildDate ());
	#endif /* _MAXIM_HSM */

	return ERR_OK;
}


int main(int argc, char **argv)
{
	int result;
	char c;
	verbose = 3;

	load_default_config();

#ifdef __WIN	
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO info;
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &info);
	bg_color_g = info.wAttributes & 0xF0;
#endif 
	
	while((c = getopt(argc, argv, "dhvc:")) != -1){
		switch(c){
			case 'c':
				strlcpy(config_g.device, optarg, sizeof(config_g.device));
				break;
			case 'd':
				verbose = 5;
				break;
			case 'h':
				print_help();
				return ERR_OK;
				break;
			case 'v':
				print_version();
				return ERR_OK;
				break;
			case '?':
	        if (optopt == 'c'){
	          fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	        }
	        else if (isprint (optopt)){
	          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	        }else{
	          fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
	        }
	        return 1;
	      default:
	        abort ();
	      }
	}

	print_info("Sign App v%d.%d.%d (%s %s) (c)Maxim Integrated 2006-2018\n", MAJV, MINV, ZVER, __DATE__, __TIME__);

	load_device_config();

	ASSERT_OK(load_ini_config());


	ASSERT_OK(load_args(argc, argv, optind));

	ASSERT_OK(check_parameters());

	display_config();

	ASSERT_OK(init_crypto());


#ifdef _MAXIM_HSM
	if(config_g.hsm){
		print_info ("This tool does handle keys with Thales nCipher Edge HSM \n");
		hsm_login();
	}else
#endif	/*_MAXIM_HSM */
	{
		print_warn ("This tool does not handle keys in a PCI-PTS compliant way, only for test \n");
	}

	result = process();

#ifdef _MAXIM_HSM
	if(config_g.hsm) hsm_close();
#endif /* _MAXIM_HSM */

	if (ERR_OK != result)
	{
		print_error("Signing process failed ! Exiting ...\n");
		return result;
	}

	print_success("Application successfully signed\n");

	return (ERR_OK);
}
