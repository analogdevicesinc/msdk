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

#ifndef _ISO14443_4_TRANSITIVE_H_
#define _ISO14443_4_TRANSITIVE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup NFC_PCD_EMV_LVL1_PART4 ISO14443-4 APDU Transport
 *
 * @ingroup NFC_PCD_EMV_LVL1_STACK
 *
 * @ref SendAPDU is the primary interface between EMV Level 2 and Level 1.
 *
 * @{
 */

/**
 * @brief Reset block number
 *
 * Resets the block number for block transmission to 0
 */
void seqnuminit(void);

/**
 * @brief Sends and Receives an APDU
 *
 * Primary interface to EMV Level 2 Stack
 *
 * Transmit and Receive APDU to and from a PICC (Card).
 * Implements [EMV Book D: Contactless Communication Protocol](https://www.emvco.com/specifications.aspx?id=21)
 * chapter "10 Half-Duplex Block Transmission Protocol"
 *
 * @param[in] capdu             data buffer to send to Card (up to 256 bytes)
 * @param[in] capdu_len         number of bytes in buffer to send to Card
 * @param[in, out] rapdu        data buffer received from Card (up to 256 bytes)
 * @param[in, out] rapdu_len    number of bytes in buffer from Card
 *
 * @return The following @ref NFC_PCD_EMV_LVL1_DRIVER_ERROR_CODES
 * @retval #ISO14443_3_ERR_SUCCESS for a successful transmission or other error code for a failure
 * @retval #ISO14443_3_ERR_PROTOCOL for a protocol error.
 */
int32_t SendAPDU(uint8_t *capdu, int32_t capdu_len, uint8_t *rapdu, int32_t *rapdu_len);

/** @} */ /* End of @defgroup NFC_PCD_EMV_LVL1_PART4 */

#ifdef __cplusplus
}
#endif

#endif
