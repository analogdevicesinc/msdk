/**
 * @file    afe.h
 * @brief   Analog Front End (AFE) communications r
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_H_

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
#define AFE_REG_ADDR_BANK_POS 23
#define AFE_REG_ADDR_BANK ((uint32_t)(0x03 << AFE_REG_ADDR_BANK_POS))
#define AFE_REG_ADDR_POS 16
#define AFE_REG_ADDR ((uint32_t)(0x7F << AFE_REG_ADDR_POS))
#define AFE_REG_ADDR_LEN_POS 0
#define AFE_REG_ADDR_LEN ((uint32_t)(0x07 << AFE_REG_ADDR_LEN_POS))
#define AFE_REG_ADDR_READ_BIT 0x80
#define AFE_CRC_LEN 1

#define AFE_ADC0_BANK 0
#define AFE_ADC1_BANK 1
#define AFE_DAC_BANK 2
#define AFE_HART_BANK 3

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
int afe_read_register(uint32_t target_reg, uint32_t *value);

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

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_AFE_H_
