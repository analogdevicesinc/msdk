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

#ifndef __SCP_H__
#define __SCP_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Generate a SCP Data link Layer packet
 * @param ctl_code Data link control code
 * @param payload_l Payload
 * @param payload_len Payload Length
 * @param msg  Debug message  - Packet name
 * @return ERR_OK if OK, and error_code otherwise.
 */
int data_link(ctl_code_t ctl_code, const uint8_t *payload_l, uint16_t payload_len, const char *msg);

/**
 * Generate an SCP Connection request packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int connection_request(void);

/**
 * Generate an SCP Connection reply packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int connection_reply(void);

/**
 * Generate an SCP Disconnection request packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int disconnection_request(void);

/**
 * Generate an SCP Disconnection reply packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int disconnection_reply(void);

/**
 * Generate an SCP Acknowledge packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int ack(void);

/**
 * Generate an SCP ECHO request packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int echo_req(void);

/**
 * Generate an SCP ECHO reply packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int echo_reply(void);

/**
 * Generate a Dummy dump packet for the SCP sender
 * @return ERR_OK if OK, and error_code otherwise.
 */
int dump(void);

/**
 * Generate a SCP Session Layer packet
 * @param cmd Session Command
 * @param data Session data
 * @param data_len Session Length
 * @param msg Debug message  - Packet name
 * @return ERR_OK if OK, and error_code otherwise.
 */
int session_layer(session_cmd_t cmd, const uint8_t *data, uint16_t data_len, const char *msg);

/**
 * Generate a generic SCP response packet
 * @param msg
 * @return ERR_OK if OK, and error_code otherwise.
 */
int generic_response(char *msg);

/**
 * Perform an SCP Hello request operation
 * @return ERR_OK if OK, and error_code otherwise.
 */
int hello_request(void);

/**
 * Generate an SCP Hello response packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int hello_reply(void);

/**
 * Perform an SCP write CRK operation
 * @param signaturefile File of the CRK signed by the MRK
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_crk(char *signaturefile);

/**
 * Perform an SCP rewrite CRK operation
 * @param oldsignaturefile File of the old CRK
 * @param newsignaturefile File of the new CRK signed by the MRK
 * @return ERR_OK if OK, and error_code otherwise.
 */
int rewrite_crk(const char *oldsignaturefile, const char *newsignaturefile);

/**
 * Perform an SCP erase operation
 * @param sfilename file representing the area to be erased
 * @param address_offset Address Offset
 * @return ERR_OK if OK, and error_code otherwise.
 */
int del_mem(char *sfilename, size_t address_offset);

/**
 * Perform an SCP write operation
 * @param data data to verify
 * @param data_len Length of data
 * @param data_addr Address where to write the data
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_mem(const uint8_t *data, size_t data_len, size_t data_addr);

/**
 * Perform an SCP verify operation
 * @param data data to verify
 * @param data_len Length of data
 * @param data_addr Address where to verify the data
 * @return ERR_OK if OK, and error_code otherwise.
 */
int verify_data(const uint8_t *data, size_t data_len, size_t data_addr);

/**
 * Perform an SCP erase operation
 * @param start_addr Address of the beginning of the area to erase
 * @param length Length of the area to erase
 * @return ERR_OK if OK, and error_code otherwise.
 */
int del_data(size_t start_addr, size_t length);

/**
 * Perform an SCP write timeout operation
 * @param timeout_target_char target
 * @param timeout_value Timeout value
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_timeout(scp_target_t timeout_target_char, size_t timeout_value);

/**
 * Perform an SCP write param operation
 * @param param_target_char target
 * @param param_value Parameter value
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_params(scp_target_t param_target_char, const uint8_t *param_value);

/**
 * Perform an SCP write stimulus operation
 * @param stims_target_char target
 * @param stims_value Stimulus value
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_stims(scp_target_t stims_target_char, uint8_t stims_value);

/**
 * Perform an SCP write deact operation
 * @param deact_target_char target
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_deact(scp_target_t deact_target_char);

/**
 * Perform an SCP kill-chip operation
 * @return ERR_OK if OK, and error_code otherwise.
 */
int kill_chip(void);

/**
 * Perform an SCP kill-chip2 operation
 * @return ERR_OK if OK, and error_code otherwise.
 */
int kill_chip2(void);

/**
 * Perform an SCP execute-code operation
 * @param address execute code address
 * @return ERR_OK if OK, and error_code otherwise.
 */
int execute_code(size_t address);

/**
 * Perform an SCP write-app-version operation
 * @param version Version to write
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_app_version(size_t version);

/**
 * Perform an SCP write-OTP operation
 * @param offset OTP address offset
 * @param data data to write
 * @param data_length data length
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_otp(size_t offset, const uint8_t *data, size_t data_length);

/**
 * Perform an SCP erase and write operation with the data in a file
 * @param sfilename file to read the data from
 * @param address_offset address offset
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_file(char *sfilename, size_t address_offset);

/**
 * Perform an SCP write operation with the data in a file
 * @param sfilename file to read the data from
 * @param address_offset address offset
 * @return ERR_OK if OK, and error_code otherwise.
 */
int write_only(char *sfilename, size_t address_offset);

/**
 * Perform an SCP verify operation with the data in a file
 * @param sfilename file to read the data from
 * @param address_offset address offset
 * @param b_dump Include or not dump packet
 * @return ERR_OK if OK, and error_code otherwise.
 */
int verify_file(char *sfilename, size_t address_offset, char b_dump);

/**
 * Parse a SCP script and store all operation in an array to be processed later
 * @param filename scp script to parse
 * @param script operation array
 * @param list_length number of operations
 * @return ERR_OK if OK, and error_code otherwise.
 */
int parse_scp_script(const char *filename, script_cmd_list_t **script, size_t *list_length);

/**
 * Process SCP operation
 * @param script scp operation to process
 * @param list_length number of operation
 * @return ERR_OK if OK, and error_code otherwise.
 */
int process_script(script_cmd_list_t *script, size_t list_length);

#ifdef __cplusplus
}
#endif

#endif /* __SCP_H__ */
