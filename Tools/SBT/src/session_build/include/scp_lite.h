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

#ifndef __SCP_LITE_H__
#define __SCP_LITE_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Format and generate a SCP-lite command
 * @param data
 * @param data_len
 * @param address
 * @return ERR_OK if success otherwise error code
 */
int scp_lite_cmd(const uint8_t *data, size_t data_len, const uint8_t *address);

/**
 * Format and generate a SCP-lite response
 * @return ERR_OK if success otherwise error code
 */
int scp_lite_response(void);

/**
 * Read a S19 data file and generate a corresponding command/response
 * @param ptr_address_offset
 * @param sfilename
 * @return ERR_OK if success otherwise error code
 */
int scp_lite_load_ram(char *ptr_address_offset, const char *sfilename);

/**
 * Process an SCP-lite script
 * @return ERR_OK if success otherwise error code
 */
int process_script_scp_lite_ecdsa(void);

#ifdef __cplusplus
}
#endif

#endif /* __SCP_LITE_H__ */
