/**
 * @file    afe.h
 * @brief   Analog Front End (AFE) communications r
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_H_

#ifdef __cplusplus
extern "C" {
#endif

/***** Includes *******/
#include "afe.h"
#include "afe_adc_zero_regs.h"
#include "afe_adc_one_regs.h"
#include "afe_dac_regs.h"
#include "afe_hart_regs.h"
#include "tmr.h"
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
 * @param   tmr    Pointer to Timer registers to use for internal AFE timing
 */
int afe_setup(mxc_tmr_regs_t *tmr);

/**
 * @brief  Puts the AFE into a RESET state to recover from errors, or reduce power consumption
 * @note   Must call afe_load_trims to restore AFE functionality after a reset.
 */
void afe_reset(void);

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
 * @brief   Writes data to AFE register in the specified bank.
 *
 * @param   target_reg  The register to write the data into
 * @param   reg_bank    register bank
 * @param   value       The data to write
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_bank_write_register(uint32_t target_reg, uint8_t reg_bank, uint32_t value);

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
 * @brief   Read data from AFE register in the specified bank.
 *
 * @param   target_reg  The register to read the data from
 * @param   reg_bank    register bank
 * @param   value       Buffer to store data in
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_bank_read_register(uint32_t target_reg, uint8_t reg_bank, uint32_t *value);

/**
 * @brief   Load AFE Trims.
 * @note    Uncomment DUMP_TRIM_DATA in afe.c to print trime data.
 * @param   tmr    Pointer to Timer registers to use for internal AFE timing
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int afe_load_trims(mxc_tmr_regs_t *tmr);

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

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_AFE_H_
