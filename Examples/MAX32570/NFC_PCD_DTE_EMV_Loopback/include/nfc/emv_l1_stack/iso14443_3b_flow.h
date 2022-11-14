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

#ifndef EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_FLOW_H_
#define EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_FLOW_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup NFC_PCD_EMV_LVL1_PART3BF ISO14443-3B Activation
 *
 * @ingroup NFC_PCD_EMV_LVL1_STACK
 *
 * Implements Type B Activation procedures
 *
 * @see @ref NFC_PCD_EMV_LVL1_PART3BC
 *
 * @{
 */

/**
 * @brief Check for presence of Type B PICC (Card)
 *
 * PCD will execute ISO14443-3B polling procedure with PICC
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Card is found
 * @retval #ISO14443_3_ERR_TIMEOUT No card detected
 */
int32_t iso_14443_3b_polling(void);

/**
 * @brief Check for presence of Type B PICC (Card) and capture response
 *
 * PCD will execute ISO14443-3B polling procedure with PICC
 *
 * Copy raw ATQB response into atqb_resp
 *
 * @param atqb_resp Buffer to save polling response: ATQB  No bounds checking is done on this
 *                  buffer other than verifying it is not NULL.  Per EMV Book D, ATQB has a
 *                  max length of 13 bytes without CRCB.
 * @param atqb_resp_len Number of bytes written to atqb
 *
 * @note If atqb_resp is NULL, no data will be saved
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Card is found
 * @retval #ISO14443_3_ERR_TIMEOUT No card detected
 */
int32_t iso_14443_3b_polling_response(uint8_t *atqb_resp, int32_t *atqb_resp_len);

/**
 * @brief Perform required anticollision checks
 *
 * PCD will execute ISO14443-3B collision detection procedure with PICC
 * implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.3.3 Type B Collision Detection"
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS No collision
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 */
int32_t iso_14443_3b_collision_detect(void);

/**
 * @brief Perform required anticollision checks and capture response
 *
 * PCD will execute ISO14443-3B collision detection procedure with PICC
 * implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.3.3 Type B Collision Detection"
 *
 * - Copy raw ATQB response into atqb_resp
 *
 * @param atqb_resp Buffer to save polling response: ATQB  No bounds checking is done on this
 *                  buffer other than verifying it is not NULL.  Per EMV Book D, ATQB has a
 *                  max length of 13 bytes without CRCB.
 * @param atqb_resp_len Number of bytes written to atqb
 *
 * @note If atqb_resp is NULL, no data will be saved
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS No collision
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 */
int32_t iso_14443_3b_collision_detect_response(uint8_t *atqb_resp, int32_t *atqb_resp_len);

/**
 * @brief Activate a Type B PICC (Card)
 *
 * PCD will execute ISO14443-3B activation procedure with PICC
 * implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.4.2 Type B Activation"
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS
 * @retval #ISO14443_3_ERR_CMD
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 * @note Only use this function after a successful call to iso_14443_3b_collision_detect
 */
int32_t iso_14443_3b_active(void);

/**
 * @brief Activate a Type B PICC (Card) and capture response
 *
 * - Copy raw ATTRIB response into attrib_resp
 *
 * @param attrib_resp   Buffer to save attrib response. No bounds checking is done on this
 *                      buffer other than verifying it is not NULL.  Per EMV Book D,
 *                      ATTRIB_RESP has a basic length of 1 byte, but may optionally contain
 *                      a higher layer response of indeterminate length.  For maximal safety
 *                      a buffer of 256 bytes is recommended.
 * @param attrib_resp_len Number of bytes written to attrib_resp
 *
 * PCD will execute ISO14443-3B activation procedure with PICC
 * implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.4.2 Type B Activation"
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS
 * @retval #ISO14443_3_ERR_CMD
 * @retval  #ISO14443_3_ERR_CONTAINS_HIGH_INF ATTRIB response received containing HIGH INF data
 * @retval #ISO14443_3_ERR_PROTOCOL Format error
 * @retval #ISO14443_3_ERR_OTHER
 * @note Only use this function after a successful call to iso_14443_3b_collision_detect
 */
int32_t iso_14443_3b_active_response(uint8_t *attrib_resp, int32_t *attrib_resp_len);

/**
 * @brief Remove a Type B PICC (Card)
 *
 * PCD will execute ISO14443-3B removal procedure with PICC
 * implements "D_EMV_Contactless_Communicaton_Protocol" chapter "9.5 Removal" for Type B
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS Successful removal
 */
int32_t iso_14443_3b_remove(void);

/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_PART3BF */

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32570_NFC_PCD_DTE_EMV_LOOPBACK_INCLUDE_NFC_EMV_L1_STACK_ISO14443_3B_FLOW_H_
