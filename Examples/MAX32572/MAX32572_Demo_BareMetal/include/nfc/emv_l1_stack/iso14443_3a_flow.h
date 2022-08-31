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

#ifndef __ISO14443_3A_FLOW_H__
#define __ISO14443_3A_FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup NFC_PCD_EMV_LVL1_PART3AF ISO14443-3A Activation
 *
 * @ingroup NFC_PCD_EMV_LVL1_STACK
 *
 * Implements Type A Activation proceedures
 *
 * @see @ref NFC_PCD_EMV_LVL1_PART3AC
 *
 * @{
 */

/**
 * Get the last received SAK
 * - Can be used to decipher MIFARE card type
 *
 * @return Last received SAK value
 */
uint8_t get_last_sak();

/**
 * @brief Check for presence of Type A PICC (Card)
 *
 * PCD will execute ISO14443-3A POLLING procedure with PICC.
 *
 * PCD will send WUPA command and if there is a response, send HALT command
 *  otherwise return ISO_14443_ERR_TIMEOUT.
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Card found and HALT sent
 * @retval #ISO14443_3_ERR_TIMEOUT No card found
 */
int32_t iso_14443_3a_polling();

/**
 * @brief Check for presence of Type A PICC (Card) and capture response
 *
 * PCD will execute ISO14443-3A polling procedure with PICC.
 *
 * Copies raw ATQA response into atqa_resp.
 *
 * @param atqa_resp Buffer to save polling response: ATQA  No bounds checking is done on this
 *                  buffer other than verifying it is not NULL.  Per EMV Book D, ATQA has a
 *                  length of 2 bytes.
 *
 * @param atqa_resp_len Number of bytes written to atqa_resp
 *
 * @note If atqa_resp is NULL, no data will be saved
 *
 * @return The following or other @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Card found and HALT sent
 * @retval #ISO14443_3_ERR_TIMEOUT No card found
 */
int32_t iso_14443_3a_polling_response(uint8_t* atqa_resp, int32_t* atqa_resp_len);

/**
 * @brief Perform required anticollision checks
 *
 * PCD will execute ISO14443-3A COLLISON DETECTION procedure with PICC.
 *
 * Implements "D_EMV_Contactless_Communication_Protocol" chapter "9.3.2 Type A Collision Detection".
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS No collision
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 *
 * @note Do not call this function without first calling iso_14443_3a_polling and getting a
 * #ISO14443_3_ERR_SUCCESS indicating at least one Type A card found
 */
int32_t iso_14443_3a_collision_detect();

/**
 * @brief Perform required anticollision checks and capture response
 *
 * PCD will execute ISO14443-3A collision detection procedure with PICC.
 *
 * Implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.3.2 Type A Collision Detection".
 *
 * - Copy raw ATQA response into atqa_resp
 * - Copy raw UID responses into uid_resp
 *
 * @param atqa_resp Buffer to save polling response: ATQA.  No bounds checking is done on this
 *                  buffer other than verifying it is not NULL.  Per EMV Book D, ATQA has a
 *                  length of 2 bytes.
 *
 * @param atqa_resp_len Number of bytes written to atqa_resp
 *
 * @param uid_resp Buffer to save anticollision response: UID.  No bounds checking is done on this
 *                 buffer other than verifying it is not NULL.  Per EMV Book D, UID has a
 *                 max length of 15 bytes.
 *
 * @param uid_resp_len Number of bytes written to uid_resp
 *
 * @param sak_resp SAK received from select command Per EMV Book D, SAK is one byte
 *
 * @note If atqa_resp, uid_resp, or sak_resp is NULL, no data will be saved respectively
 *
 * @return The following or other @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS No collision
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 *
 * @note Do not call this function without first calling iso_14443_3a_polling and getting a
 * #ISO14443_3_ERR_SUCCESS indicating at least one Type A card found
 */
int32_t iso_14443_3a_collision_detect_response(uint8_t* atqa_resp, int32_t* atqa_resp_len,
    uint8_t* uid_resp, int32_t* uid_resp_len, uint8_t* sak_resp);

/**
 * @brief Activate a Type A PICC (Card)
 *
 * PCD will execute ISO14443-3A ACTIVATION procedure with PICC.
 *
 * Implements "D_EMV_Contactless_Communication_Protocol" chapter "9.4.1 Type A ACTIVATION".
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 *
 * @note Only use this function after a successful call to iso_14443_3a_collision_detect
 */
int32_t iso_14443_3a_active();

/**
 * @brief Activate a Type A PICC (Card) and capture response
 *
 * PCD will execute ISO14443-3A ACTIVATION procedure with PICC.
 *
 * Implements "D_EMV_Contactless_Communication_Protocol" chapter "9.4.1 Type A ACTIVATION".
 *
 * - Copy raw ATS response into ats_resp
 *
 * @param ats_resp Buffer to save Answer To Select response.  No bounds checking is done on this
 *                  buffer other than verifying it is not NULL.  Per EMV Book D, PICC are not
 *                  to reply with an ATS having a length more than 20 bytes.
 *
 * @param ats_resp_len Number of bytes written to ats_resp
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 * @note Only use this function after a successful call to iso_14443_3a_collision_detect
 */
int32_t iso_14443_3a_active_response(uint8_t* ats_resp, int32_t* ats_resp_len);

/**
 * @brief Remove a Type A PICC (Card)
 *
 * PCD will execute ISO14443-3A REMOVAL procedure with PICC.
 *
 * Implements "D_EMV_Contactless_Communication_Protocol" chapter "9.5 Removal".
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Successful removal
 */
int32_t iso_14443_3a_remove();

/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_PART3AF */

#ifdef __cplusplus
}
#endif

#endif
