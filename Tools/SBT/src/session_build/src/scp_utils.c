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
#include <regex.h>
#include <errno.h>


#include "scp_definitions.h"
#include "session_build.h"
#include "aes.h"
#include "log.h"


u8 who;
char *source[2] = { "host", "bl" };
FILE * fp_packetlist_g;
int compteur_g = 0;



static char params[MAX_PARAMS][MAX_STRING];
static int nb_params;

const char * mode_name[] = {"", "", "", "", "SCP_FLORA_RSA", "",
		"MSP_MAXQ1852_ECDSA", "SCP_ECDSA", "SCP_LITE_ECDSA", "SCP_PAOLA", NULL};

const char * pp_name[] = {"", "", "", "", "","", "", "", "RSA_4096", "RSA_2048", "ECDSA", NULL};

void display_frame(uint8_t * frame, size_t frame_size)
{
	unsigned int i;
	if (frame_size != 0)
	{
		print_debug(" ");
		for (i = 0; i < frame_size; i++){
			print_d("%02x", frame[i]);
			fprintf (fp_g, "%02x", frame[i]);
		}
		print_d("\n");
		fprintf (fp_g, "\n");
	}
}

void host (void)
{
	who = HOST;
}
int whoami (void)
{
	return who;
}

void target (void)
{
	who = TARGET;
}

int open_packetlist_file(void){

	char filename[MAX_STRING];

	if(strcmp(config_g.output_dir,"") != 0){
		sprintf (filename, "%s/packet.list", config_g.output_dir);
	}else{
		sprintf (filename, "packet.list");
	}

	fp_packetlist_g = fopen (filename, "wb");
	if (fp_packetlist_g == NULL)
	{
		print_error("on opening %s : %s\n", filename, strerror(errno));
		return errno;
	}

	return ERR_OK;
}

int close_packetlist_file(void){

	return fclose (fp_packetlist_g);
}




int write_packet(uint8_t * frame, size_t frame_size, const char * name_file)
{
	FILE *pFile;
	char filename[MAX_STRING];


	if(strcmp(config_g.output_dir,"") != 0){
		sprintf (filename, "%s/%s.%07d.%s.%s.packet", config_g.output_dir, config_g.output_file, compteur_g, source[who], name_file);
		fprintf (fp_packetlist_g, "%s.%07d.%s.%s.packet\n", config_g.output_file, compteur_g, source[who], name_file);
	}else{
		sprintf (filename, "%s.%07d.%s.%s.packet", config_g.output_file, compteur_g, source[who], name_file);
		fprintf (fp_packetlist_g, "%s\n", filename);
	}

	pFile = fopen (filename, "wb");
	if (pFile == NULL)
	{
		print_error("on opening %s : %s\n", filename, strerror(errno));
		return ERR_FILE_ERROR;
	}

	fwrite(frame, 1, frame_size, pFile);

	print_debug("Packet %s created\n", filename);

	fclose(pFile);

	return ERR_OK;
}


int packet_send (uint8_t * frame, size_t frame_size, const char * message, const char * name_file)
{
	int result;

	compteur_g++;

	print_debug("<%s>.%07d.%s\n", who ? "host" : "target", compteur_g, message);
	fprintf (fp_g, "<%s>.%07d.%s\n", who ? "host" : "target", compteur_g, message);

	display_frame(frame, frame_size);
	ASSERT_OK(write_packet(frame, frame_size, name_file));

	return ERR_OK;
}

int replace_extra_params(char * param){
	regex_t regex;
	regmatch_t rm[2];

	int ret;
	char value_str[2];
	int idx = 0;

	ret = regcomp(&regex, "^\\s*%PARAM_([0-9])%\\s*$", REG_EXTENDED);
	if (ret) {
		return ERR_REGEXP_ERROR;
	}

	ret = regexec(&regex, param, 2, rm, 0);
	regfree(&regex);
	if (!ret) {
		memcpy(value_str, &param[rm[1].rm_so], 1);
		value_str[1] = '\0';
		idx = strtol(value_str, NULL, 10) - 1;
		strcpy(param, config_g.extra_param[idx]);
	}else {
		return ERR_INVALID_OPTION_FORMAT;
	}

	return ERR_OK;
}


int process_command (char *line)
{
	int i, j, k, l;
	int found_l;
	char loline[MAX_STRING];


	for (i = 0; i < (int) strlen (line); i++){
		loline[i] = (char) tolower ((int) line[i]);
	}

	/* parse every command */
	for (found_l = 0, i = 0; i < MAX_SCP_COMMAND; i++){
		/* if command found */
		if (strstr (loline, idf_scp_cmd[i].name) != NULL)
		{
			found_l = 1;
			/* process params, if any */
			/* 1st skip the identifier string */
			j = strlen (idf_scp_cmd[i].name);
			/* 2nd, skip whitespaces */
			nb_params = 0;
			/* while not eol */
			while (j < (int) strlen (line) - 1)
			{
				/* while space, go on */
				while (' ' == line[j]){
					j++;
				}
				/* if not eol */
				if (j == (int) strlen (line) - 1){
					break;
				}

				/* else */

				k = 0;
				/* while no space, retrieve param */
				while (' ' != line[j] && j < (int) strlen (line) - 1)
				{
					params[nb_params][k] = line[j];
					k++;
					j++;
				}
				params[nb_params][k] = '\0';
				k++;
				/* display param */
				print_debug("params[%d]:", nb_params);
				for (l = 0; l < k - 1; l++){
					print_d("%c", params[nb_params][l]);
				}
				print_d("\n");

				replace_extra_params(params[nb_params]);

				nb_params++;
				/* go on */
			}
			break;
		}
	}

	if (0 == found_l){
		return COMMAND_UNKNOWN;
	}
	/* return command identifier, even unknown command */
	return (i);
}
