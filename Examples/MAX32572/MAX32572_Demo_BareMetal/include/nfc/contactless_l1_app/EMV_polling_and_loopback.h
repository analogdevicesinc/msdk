/**
 * @file
 * @brief   Provides reference polling routines for NFC EMV activation
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

#ifndef _EMV_POLLING_AND_LOOPBACK_H_
#define _EMV_POLLING_AND_LOOPBACK_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup EMV_POLLING_AND_LOOPBACK Polling and Loopback
 *
 * @ingroup MML
 *
 * @{
 */

/**
 * @defgroup POLLING_CONFIG EMV Polling Configuration Switches
 *
 * Enable different behaviors in the EMV L1 Polling and Loopback routines
 *
 * @{
 */

/**
 * Treat a ATTRIB Response with High INF set as not and error
 *
 * @note    This is incompatible with EMV L1 testing, some digital tests
 *          will fail.
 */
// Use this wrapper to force Doxygen to document this disabled option
#ifdef _DOXYGEN_
#define IGNORE_HIGH_INF
#endif

// Use this define to actually enable the feature
#undef IGNORE_HIGH_INF

/**
 * Display all activation responses from cards:
 * ATQ, Anticollision, ATTRIB, Selection, ATS etc.
 */
#define LOG_ACTIVATION_RESPONSES
/** @} */ /* @defgroup POLLING_CONFIG */

/**
 * @defgroup POLLING_DELAYS EMV Polling Timeout Values
 *
 * Used for various loopback requests and polling procedures
 *
 * @{
 */
#define TIMEOUT_POWEROFF_MS 15 /**< Milliseconds to delay for a power off procedure */
/** @} */ /* @defgroup POLLING_DELAYS */

/**
 * @defgroup LOOPBACK_RAPDUS EMV Loopback Procedure Requests
 *
 * Procedure Requests from the L1 Tester
 *
 * @{
 */
#define REMOVALPROCEDURE 0x70 /**< RAPDU signaling a request for removal procedure */
#define POWEROFFPROCEDURE 0x72 /**< RAPDU signaling a request for poweroff procedure */
#define RESETPROCEDURE 0x80 /**< RAPDU signaling a request for reset procedure */
/** @} */ /* @defgroup LOOPBACK_RAPDUS */

/**
 * @defgroup POLLING_RESPONSES EMV Polling Return Values
 *
 * Results of the requested polling
 *
 * @{
 */
#define TYPE_A_READY 0x0A /**< Type A card found and activated */
#define TYPE_B_READY 0x0B /**< Type B card found and activated */
#define TYPE_A_NON_ISO14443_4_READY \
    0x1A /**< Type A card found but not compliant with Half Duplex Block Transport */
#define TYPE_B_NON_ISO14443_4_READY \
    0x1B /**< Type B card found but not compliant with Half Duplex Block Transport */
#define NO_CARD_FOUND 0xFF /**< Polled for all supported technologies, but no support card found */
#define CARD_FOUND_WITH_ERROR \
    0xFE /**< Found a card but failed to activate due communications error or unsupported card */
#define COLLISION_DETECTED 0xFD /**< Card or cards found, but failed to activate due to collision */
#define POLLING_TERMINATED \
    0x01 /**< Polling terminated by call back function @ref callback_check_for_loop_termination_t */
#define EXCHANGE_COMPLETE 0x00 /**< EMV L1 Exchange finished, Card found */
/** @} */ /* @defgroup POLLING_RESPONSES */

/** uid of last card found during polling */
typedef struct {
    uint8_t uid[15]; /**< uid of last card found during polling */
    uint32_t uid_length; /**< Length of last uid found during polling */
} uid_storage_t;

/**
 * @defgroup GLOBAL_RAPDU_VARS Global RAPDU Variables
 *
 * Global RAPDU Variables utilized by polling functions
 *
 * @{
 */
extern uint8_t rapdu[261]; /**< Shared RAPDU buffer */
extern int32_t rapdulen; /**< Length of current RAPDU in the shared buffer */
extern int32_t rapdu_displayed; /**< Display flag, used to satisfy required EMV DTE logging */
/** @} */ /* @defgroup GLOBAL_RAPDU_VARS */

/**
 * Polling termination callback function definition
 *
 * Passed to @ref singleemvl1exchange to determine if it should exit the
 * loopback test mode.
 */
typedef int32_t (*callback_check_for_loop_termination_t)(void);

/**
 * @brief Implements EMV card polling for use in application/demo environment
 *
 * Implements EMV card polling for use in application/demo environment
 *
 * @param loop_num how many times through the loop before returning
 *
 * @return @ref POLLING_RESPONSES
 */
int32_t emvl1_poll_for_card(uint32_t loop_num);

/**
 * @brief Implements EMV card polling for use in Level 1 loop-back testing environment
 *
 * Implements EMV card polling for use in Level 1 loop-back testing environment
 *
 * @param callback @ref callback_check_for_loop_termination_t
 *          Callback to check if this polling loop should be terminated.
 *          For instance the operator desires to switch to a different test mode.
 *
 * @note    If @ref callback_check_for_loop_termination_t is NULL, loop will
 *              not return until a card is found and one exchange is completed.
 *
 * @retval  #POLLING_TERMINATED
 * @retval  #EXCHANGE_COMPLETE
 */
int32_t singleemvl1exchange(callback_check_for_loop_termination_t callback);

/**
 * @brief Implements EMV card polling for use in Level 1 interoperability loop-back testing environment
 *
 * Implements EMV card polling for use in Level 1 interoperability loop-back testing environment.
 * Indicates failure with red led, and low pitched buzz, indicates pass with green led, and high
 * pitched buzz.
 *
 * @param callback @ref callback_check_for_loop_termination_t
 *          Callback to check if this polling loop should be terminated.
 *          For instance the operator desires to switch to a different test mode.
 *
 * @note    If @ref callback_check_for_loop_termination_t is NULL, loop will
 *              not return until a card is found and one exchange is completed.
 *
 * @retval  #POLLING_TERMINATED
 * @retval  #EXCHANGE_COMPLETE
 */
int32_t singleemvl1interopexchange(callback_check_for_loop_termination_t callback);

/**
 * @brief Get last UID found during polling
 *
 * Get last UID found during polling. Per EMV Book D, UID has a max length of 15 bytes.
 *
 * @return  @ref uid_storage_t Structure containing the UID of the last card found during polling
 */
uid_storage_t get_stored_uid(void);

/** @} */ /* @defgroup EMV_POLLING_AND_LOOPBACK */

#ifdef __cplusplus
}
#endif

#endif /* _EMV_POLLING_AND_LOOPBACK_H_ */
