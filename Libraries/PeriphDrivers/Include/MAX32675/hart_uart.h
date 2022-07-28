/**
 * @file    afe.h
 * @brief   Analog Front End (AFE) communications r
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

#ifndef _HART_UART_H_
#define _HART_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/***** Includes *******/
#include "stdint.h"
#include "afe.h"
#include "afe_adc_zero_regs.h"
#include "afe_adc_one_regs.h"
#include "afe_dac_regs.h"
#include "afe_hart_regs.h"
#include "mxc_sys.h"
#include "mxc_assert.h"

/***** Definitions *****/
#define NORMAL_HART_TRANSCEIVE_MODE 0
#define HART_TEST_MODE_TX_1200      1
#define HART_TEST_MODE_TX_2200      2

/***** Function Prototypes *****/
/**
 * @brief   Setup UART connection to HART.
 * @note    Test mode is required for physical layer test to output constant bit
 *          frequencies.
 *
 * @param   test_mode   Select the test mode to run the HART. (Check Definitions)
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_setup(uint32_t test_mode);

/**
 * @brief   Enable HART.
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_enable(void);

/**
 * @brief   Disable HART.
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_disable(void);

/**
 * @brief   Select HART transmission test at 1200 Hz.
 */
void hart_uart_test_transmit_1200(void);

/**
 * @brief   Select HART transmission test at 2200 Hz.
 */
void hart_uart_test_transmit_2200(void);

/**
 * @brief   Send data to HART.
 *
 * @param   data        The buffer of data to write 
 * @param   length      The number of bytes to write
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_send(uint8_t* data, uint32_t length);

/**
 * @brief   Unload the data received from the HART.
 *
 * @param   buffer            The buffer to store data in
 * @param   packet_length     The number of bytes 
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int hart_uart_get_received_packet(uint8_t* buffer, uint32_t* packet_length);

/**
 * @brief   The processing function for HART UART interrupts.
 *
 * @note    When using the HART UART, the application must call this
 *          function periodically. This can be done from within the UART interrupt
 *          handler or periodically by the application if UART interrupts are disabled.
 */
void hart_uart_handler(void);

/**@} end of group hart_uart */

#ifdef __cplusplus
}
#endif

#endif /* _HART_UART_H_ */
