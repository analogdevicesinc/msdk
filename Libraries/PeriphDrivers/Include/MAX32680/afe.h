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

#ifndef _AFE_H_
#define _AFE_H_

#ifdef __cplusplus
extern "C" {
#endif

/***** Includes *******/
#include "afe.h"
#include "afe_adc_zero_regs.h"
#include "afe_adc_one_regs.h"
#include "afe_dac_regs.h"
#include "afe_hart_regs.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "infoblock.h"

/***** Definitions *****/
#define AFE_REG_ADDR_BANK_POS	23
#define AFE_REG_ADDR_BANK		((uint32_t)(0x03 << AFE_REG_ADDR_BANK_POS))
#define AFE_REG_ADDR_POS		16
#define AFE_REG_ADDR			((uint32_t)(0x7F << AFE_REG_ADDR_POS))
#define AFE_REG_ADDR_LEN_POS	0
#define AFE_REG_ADDR_LEN		((uint32_t)(0x07 << AFE_REG_ADDR_LEN_POS))
#define AFE_REG_ADDR_READ_BIT   0x80
#define AFE_CRC_LEN				1

#define AFE_ADC0_BANK			0
#define AFE_ADC1_BANK			1
#define AFE_DAC_BANK			2
#define AFE_HART_BANK			3

/***** Function Prototypes *****/
/**
 * @brief   Setup the AFE for transactions.
 */
int afe_setup(void);

/**
 * @brief   Writes data to AFE register.
 *
 * @param   target_reg  The register to write the data into
 * @param   value       The data to write
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_write_register(uint32_t target_reg, uint32_t value);

/**
 * @brief   Read data from AFE register.
 *
 * @param   target_reg  The register to read the data from
 * @param   value       Buffer to store data in 
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_read_register(uint32_t target_reg, uint32_t* value);

/**
 * @brief   Load AFE Trims.
 * @note    Uncomment DUMP_TRIM_DATA in afe.c to print trime data.
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_load_trims(void);

/**
 * @brief   Dumps the AFE registers.
 *
 * @param   reg_bank    Register banks to dump. Check definitions in afe.h.
 */
void afe_dump_registers(uint32_t reg_bank);

/**@} end of group afe */

#ifdef __cplusplus
}
#endif

#endif /* _AFE_H_ */
