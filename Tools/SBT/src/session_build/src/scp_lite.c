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
#include <errno.h>

#include "scp_definitions.h"
#include "session_build.h"
#include "scp_utils.h"
#include "read_file.h"
#include "ecdsa.h"
#include <log.h>


int scp_lite_cmd (const uint8_t * data, size_t data_len, const uint8_t * address)
{
	unsigned int i;
	int result;
	uint8_t signature[MAX_FRAME];
	uint8_t payload[MAX_FRAME];
	size_t ipayload = 0;

	payload[ipayload++] = SCP_LITE_SYNC1;
	payload[ipayload++] = SCP_LITE_SYNC2;

	/* Destination Address */
	for (i = 0; i < 4; i++){
		payload[ipayload++] = address[i];
	}

	/* Payload Length */
	for (i = 4; i != 0; i--){
		payload[ipayload++] = (data_len >> (8 * (i - 1))) & 255;
	}

	/* Payload */
	for (i = 0; i < data_len; i++){
			payload[ipayload++] = data[i];
	}

	ASSERT_OK(ecdsa_sign (payload, ipayload, signature, config_g.ecdsaKey));

	for (i = 0; i < ECDSA_MODULUS_LEN * 2; i++){
		payload[ipayload++] = signature[i];
	}

	return packet_send(payload, ipayload, "scp-lite-load-ram", "scp_lite_load_mem");
}


int scp_lite_response (void)
{
	uint8_t payload[MAX_FRAME];
	size_t ipayload = 0;

	payload[ipayload++] = SCP_LITE_SYNC1;
	payload[ipayload++] = SCP_LITE_SYNC2;

	payload[ipayload++] = 0x00;
	payload[ipayload++] = 0x00;
	payload[ipayload++] = 0x00;
	payload[ipayload++] = 0x00;

	return packet_send(payload, ipayload, "scp-lite-load-ram-response","scp_lite_load_ram_response");
}


int scp_lite_load_ram (char *ptr_address_offset, const char * sfilename)
{
	int result;
	int i;
	unsigned int tmp;
	uint8_t address[4];

	size_t data_len = sizeof (u8) * 1024 * 1024 * config_g.flash_mb;
	uint8_t * data = malloc (data_len);
	if (NULL == data)
	{
		print_error("Unable to allocate memory for binary data (%dMB requested)\n", config_g.flash_mb);
		return ERR_MEMORY_ERROR;
	}

	if (extension("s19", sfilename))
	{
		ASSERT_OK(read_s19_file (sfilename, config_g.address_offset, data, &data_len, addr_g));
	}
	else
	{
		free(data);
		print_error("File extension not supported %s (only .s19)\n", sfilename);
		return ERR_UNSUPPORTED_EXT;
	}

	for (i = 0; i < 4; i++)
	{
		if (0 == sscanf(&(ptr_address_offset[i * 2]), "%02x", &tmp))
		{
			print_error("Register parameter <%s> incorrectly formatted as a number\n", address);
			return ERR_BAD_FORMAT;
		}
		address[i] = (uint8_t) tmp;
	}

	ASSERT_OK(scp_lite_cmd (data, data_len, address));

	target ();
	ASSERT_OK(scp_lite_response());
	host ();

	return ERR_OK;
}

static char params[MAX_PARAMS][MAX_STRING];
static int nb_params;

int process_script_scp_lite_ecdsa (const char * filename)
{
	char line[MAX_STRING];
	int command;
	int result;
	FILE *fpscript;

	fpscript = fopen (filename, "r");
	if (fpscript == NULL)
	{
		print_error("on opening %s : %s\n", filename, strerror(errno));
		return ERR_FILE_ERROR;
	}

	host ();
	while (fgets (line, MAX_STRING, fpscript) != NULL)
	{
		print_debug("<%s>", line);

		/* if 1st char is a #, then considered as a comment and skip to next line */
		if ('#' == line[0]){
			continue;
		}

		/* look for the command */
		command = process_command (line);
		print_debug("command=%s\n", idf_scp_cmd[command].name);

		switch (command) {
			case COMMAND_SCP_LITE_LOAD_RAM:

				if (2 == nb_params)
				{
					ASSERT_OK(scp_lite_load_ram (params[0], params[1]));
				}
				else
				{
					print_error("Incorrect format for load-ram command: load-ram\n");
					return ERR_BAD_PARAMETER;
				}
				break;
			case COMMAND_UNKNOWN:
			default:
				print_error("Command <%s> is not supported or Unknown\n", line);
				return ERR_CMD_UNKNOWN;
		}
	}

	fclose (fpscript);

	return ERR_OK;
}
