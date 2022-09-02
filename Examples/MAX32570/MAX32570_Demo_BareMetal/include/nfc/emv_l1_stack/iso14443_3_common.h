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

#ifndef __ISO14443_3_STATUS_H__
#define __ISO14443_3_STATUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup NFC_PCD_EMV_LVL1_STACK EMV Contactless PCD L1 Stack
 *
 * @ingroup MML
 *
 * This library implements a EMV Contactless PCD L1 Stack
 * as detailed by
 * [EMV Book D: Contactless Communication Protocol](https://www.emvco.com/specifications.aspx?id=21)
 *
 * It can also be used to handle applications outside the scope of EMV including
 * Proprietary, access control, transport, card applications, etc.
 *
 * Its primary interfaces functions support:
 * - Polling for cards in the field
 * - Activating cards, handling collision detection
 * - APDU Transport
 *
 * @{
 *
 */

#include <stdint.h>
#include "mml_nfc_pcd_rf_driver.h"

/** @defgroup NFC_PCD_EMV_LVL1_STACK_VER_HIST EMV L1 Stack Version History
 *
 * L1 Stack Version, Release History, and Change Log
 *
 * @{
 *
 * @par 4.0.0 - 05/12/2020
 *  - Initial release supporting MAX32570.
 *
 *      Primarily targeted to support EMV Types A and B.
 *      Also includes basic "as is" support for Type F and V (ISO15693)
 *
 * @par 3.4.0 - 10/25/2019
 *  - Slight modification to SendAPDU to return ISO14443_3_ERR_TIMEOUT in cases
 *      where three retries fail, and the last error is ISO14443_3_ERR_TIMEOUT
 *      instead of returning ISO14443_3_ERR_PROTOCOL.  Change should only effect
 *      EMV Level 2 behavior.
 *
 * @par 3.3.0 - 06/28/2019
 *  - EMV L1 Stack now released as source with in the DTE.  PBM library
 *      and RF Driver library are still binary only.
 *  - Fixed issues with TB311 where invalid retransmission time was used.
 *  - Updated to support new RF driver features: Configurable Early Limit,
 *      and new error codes.
 *  - Added Interoperability Loop-back with Success and Failure indications
 *  - Added addition margin of 200ppm to long counts in the L1 stack to
 *      account for crystal and temperature variations of the source clock
 *
 * @par 3.2.0 - 05/03/2019
 *  - Added ability to abort current transactions @ref set_abort_check_callback
 *      This callback is called after every @ref mml_nfc_pcd_transceive and it
 *      allows the application to regain control and stop the current transaction.
 *
 * @par 3.1.1 - 04/05/2019
 *  - Version increment to match updated RF driver
 *
 * @par 3.1.0 - 02/22/2019
 *  - Version increment to match updated RF driver
 *  - Small modifications to use new RF driver functions and parameter structures.
 *
 * @par 3.0.0 - 01/25/2019
 *  - CRC functions moved into RF driver
 *  - Update to use new structure for transceive parameters
 *
 * @par 2.1.0 - 08/23/2018
 *  - Significant throughput performance enhancements, now utilizes the new `delay_till_send`
 *      parameter of the @ref mml_nfc_pcd_transceive to time each packet to transmit.  This allows upper
 *      stack PCD processing during the inter-packet delay with minimal increases in inter-packet
 *      latency.
 *  - New function @ref nfc_set_delay_till_next_send_fc may be used to set `delay_till_send` for the
 *      immediately following transceive.  After every transceive it is reset to @ref ISO14443_FDT_MIN
 *  - Renamed `nfc_delay_ms` to `nfc_yield_ms` to better represent its actual behavior.
 *  - Renamed `nfc_delay_us` to `nfc_block_for_us` to better represent its actual behavior.
 *  - @ref nfc_pcd_reset_wait added for better precision of reset timing
 *  - Some unnecessary delays removed from polling routines
 *
 * @par 2.0.0 - 07/18/2018
 *  - Use dedicated NFC timers
 *  - Modify handling of Early Response Errors from RF Driver
 *  - Remove double reset delay on card removal EMV test cases TA002, and TB002
 *
 * @par 1.4.1 - 01/10/2018
 *  - Version increment, refer to @ref MML_NFC_PCD_DRIVER_VER_HIST
 *
 * @par 1.4.0 - 09/01/2017
 *  - L1 stack documentation using Doxygen
 *      - API function details
 *      - Porting file details
 *      - Usage and Interactions guide
 *  - Addition of dedicated Polling and Loop-Back Routines
 *  - Added support to retrieve activation responses
 *  - Dedicated error code for some Type B ATTRIB Responses with High INF set
 *  - Reworked L1 stack delays to use portable functions in RF Driver Porting File
 *
 * @par 1.3.1 - 06/23/2017
 *  - Updated to latest MML SDK release version 3.0.0
 *
 * @par 1.3.0 - 05/05/2017
 *  - Support for MAX32560 EVKIT Version 3
 *  - Additional changes for improved compatability with CMSIS system backend
 *
 * @par 1.2.0 - 03/16/2017
 *  - Updates to support EMV Contactless version 2.6b
 *      - Added minimum retransmission time of 3ms to the following commands:
 *          - WUPA
 *          - ANTICOLLISION
 *          - RATS
 *          - WUPB
 *          - ATTRIB
 *  - Updated Copyright headers
 *  - Ported from Cobra SDK to new MML CMSIS SDK
 *  - Supporting only A2 silicon
 *
 * @par 1.1.0 - 10/12/2016
 *  - First public release
 *  - Supporting only A1 silicon
 *  - Built as binary library for release
 *  - Added structure and method to get and set current analog configuration
 *  - Tuned for MAX32560 EVKIT Version 2
 *
 * @par 1.0.0 - 09/13/2016
 *  - Used in EMV L1 Certification
 *  - Supporting only A1 silicon
 */
/** @} */ /* @defgroup NFC_PCD_EMV_LVL1_STACK_VER_HIST */

/**
 * @defgroup NFC_PCD_EMV_LVL1_COMMON Common Routines
 *
 * @ingroup NFC_PCD_EMV_LVL1_STACK
 *
 * @{
 */

/**
  * @defgroup NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES ERROR_CODES
  *
  * EMV Level 1 Error Codes from library routines
  *
  * @{
  */
#define ISO14443_3_ERR_SUCCESS (0x0000) /**< No Errors, Success */
//Command error

#define ISO14443_3_ERR_TIMEOUT      (0x0001) /**< Transmit or Receive operation timeout */
#define ISO14443_3_ERR_TRANSMISSION (0x0005) /**< Error during transmission */

//Flow error
#define ISO14443_3_ERR_CMD (0x0020) /**< Cmd execute error */
#define ISO14443_3_ERR_SEQ (0x0021) /**< Flow sequence error */

#define ISO14443_3_ERR_PROTOCOL            (0x0022) /**< Protocol error */
#define ISO14443_3_ERR_COLLISION           (0x0023) /**< Data Collision */
#define ISO14443_3_ERR_EARLY_RESPONSE      (0x0024) /**< Response from Card came too soon */
#define ISO14443_3_ERR_NON_ISO14443_4_CARD (0x0025) /**< Non ISO14443 Card type detected */
#define ISO14443_3_ERR_CONTAINS_HIGH_INF \
    (0x0026) /**< ATTRIB Response received with HIGH INF set (Not allowed by EMV) */
#define ISO14443_3_ERR_ABORTED \
    (0x0027) /**< Current transaction has been aborted by application request */

#define ISO14443_3_ERR_OTHER (0x00ff) /**< Other, unspecified error */
/** @} */                             /* End of @defgroup NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES */

/**
  * @defgroup NFC_PCD_EMV_LVL1_DRIVER_STD_VALUES STANDARD CONSTANTS
  *
  * EMV L1 (ISO14443) Constants
  *
  * @{
  */
#define FSDI_DEFAULT_VALUE (8) /**< FSDI nominal value */

#define FSCI_DEFAULT_VALUE (2) /**< FSCI nominal value */
#define FSCI_MAX_VALUE     (8) /**< FSCI maximum value */

#define FWI_DEFAULT_VALUE (4)  /**< FWI nominal value */
#define FWI_MAX_VALUE     (14) /**< FWI maximum value */

#define SFGI_DEFAULT_VALUE (0)  /**< SFGI nominal value */
#define SFGI_MAX_VALUE     (14) /**< SFGI maximum value */

#define WAKEUP_NOTRETRY 0 /**< value for NO retry on Wake */
#define WAKEUP_DORETRY  1 /**< value for retry on Wake */

#define TPDELAY_US    5100              /**< TP (Pause) time in microseconds, 5.1~10ms per spec */
#define TPDELAY_IN_FC US2FC(TPDELAY_US) /**< TP (Pause) Delay of 5.1ms per spec in fc */

#define TRESET_US 5100 /**< Reset time in microseconds, 5.1~10ms per spec */

#define TMIN_RETRANSMISSION_US 3000 /**< New for EMV 2.6b Minimum retransmission time FDT in us */
#define TMIN_RETRANSMISSION_FC \
    US2FC(TMIN_RETRANSMISSION_US) /**< New for EMV 2.6b Minimum retransmission time FDT in fc */

#define PROTOCOL_DISREGARD_BITS 0xF1 /**< Mask off bits b4-b2 per 6.3.2.10 */

// time unit all fc
#define ISO14443_FWT_MAX 67108864UL /**< maximum FWT = 4096 x 2^14 */
#define ISO14443_FWT_DEFAULT \
    114688 /**< nominal FWT in 'fc' units =FWTmax+ AFWT = 4096 * 2^4  + 49152 = 114688 */
#define ISO14443_FWT_DELTA 49152 /**< delta FWT, use for attrib & apdu */

#define ISO14443_FDT_A_EXTRA_MARGIN \
    16 /**< Provide a bit of allowed margin on reception timing.  Relaxes early limit, and timeout.  Units are fc */
#define ISO14443_FDT_A_PICC_MIN \
    (1172) /**< Time when PICC is to respond during activation to PCD command. EMV spec 4.21.  @note we don't care about last bit value, timestamp happens the same regardless. */
#define ISO14443_FDT_A_EARLY_LIMIT \
    (ISO14443_FDT_A_PICC_MIN -     \
     128) /**< Defines the Deaf Time before which any response from PICC is ignored.  EMV spec 4.8.1.3 as FDTa,picc,min - 128fc */
#define ISO14443_FWT_A_ACT         \
    (ISO14443_FDT_A_PICC_MIN + 1 + \
     6) /**< FWT max during activation = FDTa,picc,min + 1fc + .4us (5.424fc, round up to 6) */

#define ISO14443_FDT_B_EXTRA_MARGIN \
    0 /**< Provide a bit of allowed margin on reception timing.  Relaxes early limit, and timeout.  Units are fc */
#define ISO14443_FDT_B_PICC_MIN \
    (1008 +                     \
     1264) /**< FDTmin Type B, defines the deaf time: TR0min (1008fc) + TR1min (1264fc) => 1008+1264 = 2272 */
#define ISO14443_FWT_ATQB (7680) /**< typeB FWT */

// TODO: Probably need a new include file for vicinity maybe ISO as well
// TODO: timing values for vicinity need more work as well
#define ISO15693_FDT_VICC_MIN   (3320) /**< FDTmin Type V, */
#define ISO15693_FWT_ACTIVATION (8800) /**< FWTmax Type V, */

#define ISO14443_FWT_ACTIVATION (71680) /**< typeA FWT during activation */
#define ISO14443_FDT_MIN \
    (6780) /**< minimum FDT in 'fc' units, (frame delay time) between PICC and pcd new command */

// This is added to long timings in excess of 100,000
#define CRYSTAL_PPM_MARGIN_MULTIPLIER 20
#define CRYSTAL_PPM_MARGIN_DIVISOR    1000000

#define ISO14443_FDT_MIN_US \
    FC2US(                  \
        ISO14443_FDT_MIN) /**< minimum FDT in microseconds, (frame delay time) between PICC and pcd new command */
/** @} */                 /* End of @defgroup NFC_PCD_EMV_LVL1_DRIVER_STD_VALUES */

/* Defines *********************************************************************/
#define MAX_BUFFER_LEN 511      /**< Max length of @ref gcommonbuffer */
extern uint8_t gcommonbuffer[]; /**< Buffer used by all internal routines */
/**
 * @brief GetCommonBuffer
 *  Get a pointer to a shared common buffer of MAX_BUFFER_LEN bytes
 */
/* Macros *********************************************************************/
#define GetCommonBuffer() gcommonbuffer

/* Structures *****************************************************************/
typedef struct {
    uint8_t Pro_Type;
    uint8_t FSCI;
    uint8_t FWI;
    uint8_t SFGI;

    uint8_t NAD_support;
    uint8_t CID_support;
} ATSConfig_t;

/**
 * Abort transaction callback function definition
 *
 * @note    The abort function should return 1 (TRUE) if abort is desired
 */
typedef int32_t (*abort_check_callback_t)(void);

/**
 * @brief Setup abort callback
 *
 * Sets the callback function used to determine if the application needs to
 * abort the current transaction.  This is required for some Level 2 test cases
 * to ensure responsiveness for user access.
 *
 * In the case that the call back returns 1 (TRUE), the current polling will
 * stop and return @ref ISO14443_3_ERR_ABORTED.
 *
 * @param[in]   abort_callback  Function to call for abort check
 *
 * @note    The abort function should return 1 (TRUE) if abort is desired
 */
void set_abort_check_callback(abort_check_callback_t abort_callback);

/**
 * @brief Gets the current ATS
 *
 * Gets the current ATS configuration values
 *
 * @param[in, out] cfg
 * @return pointer to ATSConfig_t structure with ATS values
 */
void get_ats(ATSConfig_t* cfg);

/**
 * @brief Set the current ATS
 *
 * Sets the current ATS configuration values
 *
 * @param[in] pro_type  Protocol to use for ATS
 * @param[in] fsci      FSC time (integer)
 * @param[in] fwi       Frame wait time (integer 0-14) FWT is calculated from this value
 * @param[in] sfgi      SFG (integer)
 * @param[in] nad       1 for NAD supported, 0 for not supported
 * @param[in] cid       1 for CID supported, 0 for not supported
 */
void set_ats(uint8_t pro_type, uint8_t fsci, uint8_t fwi, uint8_t sfgi, uint8_t nad, uint8_t cid);

/**
 * @brief Yield (Delay) for X milliseconds
 *
 * Yield (Delay)  for a specified number of milliseconds.
 *
 * @param yield_ms    Number of milliseconds to yield before returning
 *
 * @note This function calls out to mml_nfc_pcd_task_sleep for improved portability
 */
void nfc_yield_ms(uint32_t yield_ms);

/**
 * @brief Set the inter-packet delay (FDTpcd) to use for the next packet
 *
 * Set the amount of time the RF driver should delay sending the next requested
 * packet out. This defaults to @ref ISO14443_FDT_MIN
 *
 * @param delay    Number of fc to delay before sending next packet
 *
 * @note This function does not actually wait.  The RF driver will enforce
 *   the exact timing based on end of previous TX or RX as required.
 */
void nfc_set_delay_till_next_send_fc(uint32_t delay);

/**
 * @brief Block (Delay) for X microseconds
 *
 * Block (Delay) for a specified number of microseconds.
 *
 * @param block_us    Number of microseconds to delay before returning
 */
void nfc_block_for_us(uint32_t block_us);

/**
 * @brief Wait for minimum duration required for EMV PCD Reset
 *
 * Minimum duration as of 2.6b is 5.1ms.  We will task yield for 5ms, and block for another
 * 100us.
 */
void nfc_pcd_reset_wait(void);

/**
 * @brief Enable EMV field
 *
 * Turns on the 13.56 MHz field, and performs any calibration needed to be ready to communicate
 */
void poweron_operatingfield(void);

/**
 * @brief Disables EMV field
 *
 * Turns off the 13.56 MHz field
 */
void poweroff_operatingfield(void);

/**
 * @brief Perform EMV Reset
 *
 * Perform an NFC reset action to cause PICCs to reset.
 * Turns off the 13.56Mhz field, waits 5 mSec, turns the field back on and waits another 5 mSec
 *
 * @note The field is on and the PICC should be reset and ready to communicate after this routine returns
 * @return Always returns 0
 */
int32_t nfc_reset(void);

/**
 * @brief Dump hex data
 *
 * prints data in a hex format according to the debug level
 *
 * @param[in] dbg_level Debug level indicating level of debug output needed for this to be printed out
 * @param[in] buf       Buffer with data to print out
 * @param[in] len       Length of data to print out
 * @param[in] send      Binary value: 1 means data send out, 0 is incoming data
 */
void hexdump(int32_t dbg_level, uint8_t* buf, int32_t len, int32_t send);

/**
 * @brief Send and Receive EMV/NFC Data
 *
 * PCD will transmit a buffer and then wait for a response from the PICC
 * This function handles the low level control of the NFC peripherial.
 * Including:
 *  - Timing error checking for FDT - Frame Delay Time (EMV default values used)
 *  - Timing error checking for FWT - Frame Wait Time (value supplied by caller)
 *  - Communications errors
 *  - Loading and Unloading the FIFO
 *     - AFE configuration for card type (A or B)
 *     - Frame setup for NFC uart
 *
 * @param[in] protocol      @ref NFC_PCD_ISO_EMV_PROTOCOL_TYPES
 * @param[in] frametype     @ref NFC_PCD_ISO_EMV_FRAMETYPES
 * @param[in] tx_buf        Pointer to buffer with data to transmit (up to 511 bytes)
 * @param[in] tx_len        Length of data (number of bytes) to transmit
 * @param[in, out] rx_buf   Pointer to buffer to put receive data in (up to 511 bytes including CRC)
 * @param[in, out] rx_len   Length of received data
 * @param[in] timeout       Timeout in 1/Fc clocks before returning timeout error
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS          Transmit and receive are successful
 * @retval #ISO14443_3_ERR_TIMEOUT          Timeout occurred while waiting on receive data
 * @retval #ISO14443_3_ERR_TRANSMISSION     Transmission protocol error
 * @retval #ISO14443_3_ERR_COLLISION        Data collision error
 * @retval #ISO14443_3_ERR_EARLY_RESPONSE   PICC responded too early
 * @retval #ISO14443_3_ERR_OTHER            All other types of error
 */
int32_t nfc_pcd_transceive(uint8_t protocol, uint8_t frametype, uint8_t* tx_buf, uint32_t tx_len,
                           uint8_t* rx_buf, uint32_t* rx_len, uint32_t timeout);
/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_STACK */
/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_COMMON */

#ifdef __cplusplus
}
#endif

#endif /* __ISO14443_3_STATUS_H__ */
