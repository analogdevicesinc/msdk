/**
 * @file
 * @brief   Provides ISO14443 Type B activation commands
 */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*/

#ifndef EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_CMD_H_
#define EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup NFC_PCD_EMV_LVL1_PART3BC ISO14443-3B Low Level Commands
 *
 * @ingroup NFC_PCD_EMV_LVL1_PART3BF
 *
 * Implements basic Type B Low Level commands used primarily for Activation.  Typically
 * the routines detailed in @ref NFC_PCD_EMV_LVL1_PART3BF are used for activation.  These
 * low level commands may be called directly for Non EMV applications.
 *
 * @{
 */

#include <stdint.h>
#include "iso14443_3_common.h"

/**
  * @defgroup NFC_PCD_EMV_LVL1_TYPEB_INIT_DEFINES Type B Initialization Defines
  *
  * EMV Level 1 Type B defines for initialization and anticollision
  *
  * @{
  */
#define PUPI_SIZE (4) /**< Length of PUPI Pseudo-Unique PICC Identifier */

#define ISO3B_ATQB_MINLEN 12 /**< Min allowed number of bytes in ATQB response */
#define ISO3B_ATQB_MAXLEN 13 /**< Max allowed number of bytes in ATQB response */

#define ISO3B_ATQB_BYTE1 0x50 /**< ATQB byte 1 should always be this, used for error checks */
/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_TYPEB_ENUM_DEFINES */

/**
 * @brief Send WUPB
 *
 * Creates and sends the WUPB command to the PICC and looks for a correct response
 *
 * @param[in] atq       Pointer to the ATQB data received from the PICC if successful
 * @param[in] atq_len   Number of bytes in the ATQ data
 * @param[in] doretry   Binary value, 1=3 tries, 0=single try
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Received ATQA
 * @retval #ISO14443_3_ERR_PROTOCOL
 * @retval #ISO14443_3_ERR_TIMEOUT No proper response seen within the timeout
 */
int32_t iso_14443_3b_cmd_req_wup(uint8_t *atq, int32_t *atq_len, uint8_t doretry);

/**
 * @brief Sends ATTRIB command, and captures response
 *
 * Creates and sends the ATTRIB command to the PICC and looks for the correct response
 * Creates the ATTRIB command starting with 0x1D, then adds 4 bytes of PUPI, then adds the 4 parameters
 * and finally adds the info supplied in *inf
 *
 * @param[in] pupi  Pointer to 4 bytes of PUPI used to create the ATTRIB command
 * @param[in] para1 First of 4 parameter bytes used to create the ATTRIB command
 * @param[in] para2 Second of 4 parameter bytes used to create the ATTRIB command
 * @param[in] para3 Third of 4 parameter bytes used to create the ATTRIB command
 * @param[in] para4 Fourth of 4 parameter bytes used to create the ATTRIB command
 * @param[in] inf   Pointer to INF data used to create the ATTRIB command
 * @param[in] inf_len   Number of bytes in the INF data to be used
 * @param[in] timeout   Timeout for receiving the response from the PICC
 * @param[in,out] attrib_resp   Buffer to save attrib response. No bounds checking is done on this
 *                      buffer other than verifying it is not NULL.  Per EMV Book D,
 *                      ATTRIB_RESP has a basic length of 1 byte, but may optionally contain
 *                      a higher layer response of indeterminate length.  For maximal saftey
 *                      a buffer of 256 bytes is recommended.
 * @param[in,out] attrib_resp_len Number of bytes written to attrib_resp
 *
 * @return  The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval  #ISO14443_3_ERR_SUCCESS
 * @retval  #ISO14443_3_ERR_PROTOCOL Incorrect response from PICC
 * @retval  #ISO14443_3_ERR_COLLISION
 * @retval  #ISO14443_3_ERR_CONTAINS_HIGH_INF ATTRIB response received containing HIGH INF data
 * @retval  #ISO14443_3_ERR_TIMEOUT No response seen within timeout
 * @retval  #ISO14443_3_ERR_EARLY_RESPONSE Response from PICC comes too soon after the ATTRIB command is sent
 */
int32_t iso_14443_3b_cmd_attrib(uint8_t *pupi, uint8_t para1, uint8_t para2, uint8_t para3,
                                uint8_t para4, uint8_t *inf, uint32_t *inf_len, uint32_t timeout,
                                uint8_t *attrib_resp, int32_t *attrib_resp_len);

/**
 * @brief Halt PICC
 *
 * Sends the HALT command to the PICC, and looks for a response from the PICC
 * The response from the PICC should be 0x00 followed by 2 bytes of CRC_B
 * This routine builds the HALT B command using the PUPI supplied by the calling routine,
 * it appends the PUPI to 0x50 to create the HALT B command
 *
 * @param[in] pupi  Pointer to 4 bytes of PUPI to append to create the HALT B command
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval  #ISO14443_3_ERR_SUCCESS
 * @retval  #ISO14443_3_ERR_PROTOCOL Incorrect response from PICC
 */
int32_t iso_14443_3b_cmd_halt(uint8_t *pupi);

/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_PART3BC */

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_CMD_H_
