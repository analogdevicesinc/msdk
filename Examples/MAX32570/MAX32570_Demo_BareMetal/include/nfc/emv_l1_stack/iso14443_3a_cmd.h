/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef __ISO_14443_3A_CMD_H__
#define __ISO_14443_3A_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup NFC_PCD_EMV_LVL1_PART3AC ISO14443-3A Low Level Commands
 *
 * @ingroup NFC_PCD_EMV_LVL1_PART3AF
 *
 * Implements basic Type A Low Level commands used primarily for Activation.  Typically
 * the routines detailed in @ref NFC_PCD_EMV_LVL1_PART3AF are used during activate.  These
 * low level commands can be called directly for Non EMV applications.
 *
 * @{
 */

#include <stdint.h>

/**
 * @defgroup NFC_PCD_EMV_LVL1_TYPEA_INIT_DEFINES Type A Initialization Defines
 *
 * EMV Level 1 Type A defines for initialization and anticollision
 *
 * @{
 */
#define ISO_14443_3A_CMD_WUPA 0x52 /**< WUPA - Wake A command */
#define ISO_14443_3A_CMD_SEQA 0x26 /**< SEQA - SEQ A command */

#define ISO_14443_3A_CMD_ANTICOLL_SEL_L1 0x93 /**< ANTI-collision Level 1 */
#define ISO_14443_3A_CMD_ANTICOLL_SEL_L2 0x95 /**< ANTI-collision Level 2 */
#define ISO_14443_3A_CMD_ANTICOLL_SEL_L3 0x97 /**< ANTI-collision Level 3 */

#define ISO_14443_3A_CMD_SELECT_SEL_L1 0x93 /**< ANTI-collision Select Level 1 */
#define ISO_14443_3A_CMD_SELECT_SEL_L2 0x95 /**< ANTI-collision Select Level 2 */
#define ISO_14443_3A_CMD_SELECT_SEL_L3 0x97 /**< ANTI-collision Select Level 3 */

#define ATQA_LEN 2 /**< Length of ATQA command */
#define UID_EACH_LEN 5 /**< UID Length, includes bcc and ct value */
/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_TYPEA_ENUM_DEFINES */

/**
 * @brief Send WUPA
 *
 * Send a WUPA command to the PICC and listen for a REQA from the PICC
 *
 * @param[in] req       The one byte WUPA command to send
 * @param[out] atq      The response received from the PICC (should be ATQA) if successful
 * @param[in] doretry   Binary value, 1=do up to 3 tries, 0=no retries
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Received ATQA
 * @retval #ISO14443_3_ERR_PROTOCOL
 * @retval #ISO14443_3_ERR_TIMEOUT No proper response seen within the timeout
 */
int32_t iso_14443_3a_cmd_req_wupa(uint8_t req, uint8_t* atq, uint8_t doretry);

/**
 * @brief Send 1 step of the anti-collision
 *
 * Send one step of the anti-collision protocol, look for response from PICC
 *
 * Retry a max of 3 times if response timesout
 *
 * @param[in] sel   The first byte of the anti-collision step (typically 0x93 for first step, 0x95
 * for second, or 0x97 for third)
 *      @note 0x20 is appended to sel by this routine
 * @param[in, out] uid   The UID response received from the PICC if successful
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Received UID
 * @retval #ISO14443_3_ERR_PROTOCOL
 * @retval #ISO14443_3_ERR_TIMEOUT No proper response seen within the timeout
 */
int32_t iso_14443_3a_cmd_anticoll(uint8_t sel, uint8_t* uid);

/**
 * @brief Attempt to Select the PICC
 *
 * Send the Select command to the PICC, look for response from PICC
 *
 * Retry a max of 3 times if response timesout
 *
 * @param[in] sel   The first byte of the anti-collision step (typically 0x93 for first step, 0x95
 * for second, or 0x97 for third)
 *      @note 0x70 and the UID received in previous anti-collision steps are appended to sel by this
 * routine
 * @param[in] uid   This is appended to the select command, use the UID parameter returned by @ref
 * iso_14443_3a_cmd_anticoll
 * @param[in, out] sak   Response from the PICC if successful (should be a SAK response)
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Received SAK
 * @retval #ISO14443_3_ERR_PROTOCOL
 * @retval #ISO14443_3_ERR_TIMEOUT No proper response seen within the timeout
 */
int32_t iso_14443_3a_cmd_select(uint8_t sel, uint8_t* uid, uint8_t* sak);

/**
 * @brief Send Request for ATS
 *
 * Send a RATS command to the PICC and receive the ATS response
 * This routine builds the RATS command using the FSDI and CID supplied by the calling routine
 *
 * Retry a max of 3 times if response timesout or responds early
 *
 * @param[in] fsdi  fsdi used in the RATS command
 * @param[in] cid   cid used in the RATS command
 * @param[in, out] ats  Pointer to the ATS data received from the PICC
 * @param[in, out] ats_len   Number of bytes in the ATS data received from the PICC
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Received ATS
 * @retval #ISO14443_3_ERR_PROTOCOL
 * @retval #ISO14443_3_ERR_EARLY_RESPONSE Response received too soon after tranmission by the PCD
 * @retval #ISO14443_3_ERR_TIMEOUT No proper response seen within the timeout
 */
int32_t iso_14443_3a_cmd_rats(uint8_t fsdi, uint8_t cid, uint8_t* ats, uint32_t* ats_len);

/**
 * @brief Halt PICC
 *
 * Sends the HALT command to the PICC, and looks for a response from the PICC
 * The PICC should NOT send a response
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS (No response seen from PICC within timeout)
 * @retval #ISO14443_3_ERR_OTHER Failure of any kind
 */
int32_t iso_14443_3a_cmd_halt(void);

/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_PART3A */

#ifdef __cplusplus
}
#endif

#endif
