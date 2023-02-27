/**
 * @file        max11261_regs.h
 * @brief       MAX11261 register definitions.
 * @details     This header file contains definitions of MAX11261 registers and
 *              their bitfields.
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

#ifndef MAX11261_REGS_H_
#define MAX11261_REGS_H_

/**
 * @ingroup  max11261_commands
 * @defgroup MAX11261_COMMAND_BYTE
 * @brief    Command byte.
 * @{
 */
#define MAX11261_CMD_READ(addr) ((0x03UL << 6) | (addr << 1) | (0x01UL << 0))
#define MAX11261_CMD_WRITE(addr) ((0x03UL << 6) | (addr << 1))
#define MAX11261_CMD_POWERDOWN 0x90
#define MAX11261_CMD_CALIBRATE 0xA0
#define MAX11261_CMD_SEQUENCER(r) 0xB0 | (r & 0x0F)

/**@} end of group MAX11261_COMMAND_BYTE */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_STAT MAX11261_STAT
 * @brief    Status Register.
 * @{
 */
#define MAX11261_STAT 0x00

#define MAX11261_STAT_RDY_POS 0 /**< STAT_RDY Position */
#define MAX11261_STAT_RDY ((uint32_t)(0x01UL << MAX11261_STAT_RDY_POS))

#define MAX11261_STAT_MSAT_POS 1 /**< STAT_MSAT Position */
#define MAX11261_STAT_MSAT ((uint32_t)(0x01UL << MAX11261_STAT_MSAT_POS))

#define MAX11261_STAT_PDSTAT_POS 2 /**< STAT_PDSTAT Position */
#define MAX11261_STAT_PDSTAT ((uint32_t)(0x03UL << MAX11261_STAT_PDSTAT_POS))
#define MAX11261_STAT_PDSTAT_CONVERSION ((uint32_t)(0x00UL << MAX11261_STAT_PDSTAT_POS))
#define MAX11261_STAT_PDSTAT_SLEEP ((uint32_t)(0x01UL << MAX11261_STAT_PDSTAT_POS))
#define MAX11261_STAT_PDSTAT_STANDBY ((uint32_t)(0x02UL << MAX11261_STAT_PDSTAT_POS))
#define MAX11261_STAT_PDSTAT_RESET ((uint32_t)(0x03UL << MAX11261_STAT_PDSTAT_POS))

#define MAX11261_STAT_RATE_POS 4 /**< STAT_RATE Position */
#define MAX11261_STAT_RATE ((uint32_t)(0x0FUL << MAX11261_STAT_RATE_POS))

#define MAX11261_STAT_AOR_POS 8 /**< STAT_AOR Position */
#define MAX11261_STAT_AOR ((uint32_t)(0x01UL << MAX11261_STAT_AOR_POS))

#define MAX11261_STAT_DOR_POS 9 /**< STAT_DOR Position */
#define MAX11261_STAT_DOR ((uint32_t)(0x01UL << MAX11261_STAT_DOR_POS))

#define MAX11261_STAT_SYSGOR_POS 10 /**< STAT_SYSGOR Position */
#define MAX11261_STAT_SYSGOR ((uint32_t)(0x01UL << MAX11261_STAT_SYSGOR_POS))

#define MAX11261_STAT_ERROR_POS 11 /**< STAT_ERROR Position */
#define MAX11261_STAT_ERROR ((uint32_t)(0x01UL << MAX11261_STAT_ERROR_POS))

#define MAX11261_STAT_GPOERR_POS 12 /**< STAT_GPOERR Position */
#define MAX11261_STAT_GPOERR ((uint32_t)(0x01UL << MAX11261_STAT_GPOERR_POS))

#define MAX11261_STAT_ORDERR_POS 13 /**< STAT_ORDERR Position */
#define MAX11261_STAT_ORDERR ((uint32_t)(0x01UL << MAX11261_STAT_ORDERR_POS))

#define MAX11261_STAT_REFDET_POS 14 /**< STAT_REFDET Position */
#define MAX11261_STAT_REFDET ((uint32_t)(0x01UL << MAX11261_STAT_REFDET_POS))

#define MAX11261_STAT_SCANERR_POS 15 /**< STAT_SCANERR Position */
#define MAX11261_STAT_SCANERR ((uint32_t)(0x01UL << MAX11261_STAT_SCANERR_POS))

#define MAX11261_STAT_SRDY_POS 16 /**< STAT_SRDY Position */
#define MAX11261_STAT_SRDY ((uint32_t)(0x03FUL << MAX11261_STAT_SRDY_POS))

#define MAX11261_STAT_INRESET_POS 22 /**< STAT_INRESET Position */
#define MAX11261_STAT_INRESET ((uint32_t)(0x01UL << MAX11261_STAT_INRESET_POS))

/**@} end of group MAX11261_STAT_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_CTRL1 MAX11261_CTRL1
 * @brief    Control Register 1.
 * @{
 */
#define MAX11261_CTRL1 0x01

#define MAX11261_CTRL1_CONTSC_POS 0 /**< CONTSC Position */
#define MAX11261_CTRL1_CONTSC ((uint8_t)(0x01UL << MAX11261_CTRL1_CONTSC_POS))

#define MAX11261_CTRL1_SCYCLE_POS 1 /**< SCYCLE Position */
#define MAX11261_CTRL1_SCYCLE ((uint8_t)(0x01UL << MAX11261_CTRL1_SCYCLE_POS))
#define MAX11261_CTRL1_SCYCLE_CONT ((uint8_t)(0x00UL << MAX11261_CTRL1_SCYCLE_POS))
#define MAX11261_CTRL1_SCYCLE_SINGLE ((uint8_t)(0x01UL << MAX11261_CTRL1_SCYCLE_POS))

#define MAX11261_CTRL1_FORMAT_POS 2 /**< FORMAT Position */
#define MAX11261_CTRL1_FORMAT ((uint8_t)(0x01UL << MAX11261_CTRL1_FORMAT_POS))
#define MAX11261_CTRL1_FORMAT_TWOS_COMP ((uint8_t)(0x00UL << MAX11261_CTRL1_FORMAT_POS))
#define MAX11261_CTRL1_FORMAT_OFFSET_BIN ((uint8_t)(0x01UL << MAX11261_CTRL1_FORMAT_POS))

#define MAX11261_CTRL1_U_B_POS 3 /**< U_B Position */
#define MAX11261_CTRL1_U_B ((uint8_t)(0x01UL << MAX11261_CTRL1_U_B_POS))
#define MAX11261_CTRL1_U_B_BIPOLAR ((uint8_t)(0x00UL << MAX11261_CTRL1_U_B_POS))
#define MAX11261_CTRL1_U_B_UNIPOLAR ((uint8_t)(0x01UL << MAX11261_CTRL1_U_B_POS))

#define MAX11261_CTRL1_PD_POS 4 /**< PD Position */
#define MAX11261_CTRL1_PD ((uint8_t)(0x03UL << MAX11261_CTRL1_PD_POS))
#define MAX11261_CTRL1_PD_SLEEP ((uint8_t)(0x01UL << MAX11261_CTRL1_PD_POS))
#define MAX11261_CTRL1_PD_STANDBY ((uint8_t)(0x02UL << MAX11261_CTRL1_PD_POS))
#define MAX11261_CTRL1_PD_RESET ((uint8_t)(0x03UL << MAX11261_CTRL1_PD_POS))

#define MAX11261_CTRL1_CAL_POS 6 /**< CAL Position */
#define MAX11261_CTRL1_CAL ((uint8_t)(0x03UL << MAX11261_CTRL1_CAL_POS))
#define MAX11261_CTRL1_CAL_SELF ((uint8_t)(0x00UL << MAX11261_CTRL1_CAL_POS))
/**@} end of group MAX11261_CTRL1_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_CTRL2 MAX11261_CTRL2
 * @brief    Control Register 2.
 * @{
 */
#define MAX11261_CTRL2 0x02

#define MAX11261_CTRL2_PGA_POS 0 /**< PGA Position */
#define MAX11261_CTRL2_PGA ((uint8_t)(0x07UL << MAX11261_CTRL2_PGA_POS))

#define MAX11261_CTRL2_PGAEN_POS 3 /**< PGAEN Position */
#define MAX11261_CTRL2_PGAEN ((uint8_t)(0x01UL << MAX11261_CTRL2_PGAEN_POS))

#define MAX11261_CTRL2_LPMODE_POS 4 /**< LPMODE Position */
#define MAX11261_CTRL2_LPMODE ((uint8_t)(0x01UL << MAX11261_CTRL2_LPMODE_POS))

#define MAX11261_CTRL2_LDOEN_POS 5 /**< LDOEN Position */
#define MAX11261_CTRL2_LDOEN ((uint8_t)(0x01UL << MAX11261_CTRL2_LDOEN_POS))

#define MAX11261_CTRL2_CSSEN_POS 6 /**< CSSEN Position */
#define MAX11261_CTRL2_CSSEN ((uint8_t)(0x01UL << MAX11261_CTRL2_CSSEN_POS))

#define MAX11261_CTRL2_RST_POS 7 /**< RST Position */
#define MAX11261_CTRL2_RST ((uint8_t)(0x01UL << MAX11261_CTRL2_RST_POS))
/**@} end of group MAX11261_CTRL2_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_CTRL3 MAX11261_CTRL3
 * @brief    Control Register 3.
 * @{
 */
#define MAX11261_CTRL3 0x03

#define MAX11261_CTRL3_NOSCO_POS 0 /**< NOSCO Position */
#define MAX11261_CTRL3_NOSCO ((uint8_t)(0x01UL << MAX11261_CTRL3_NOSCO_POS))

#define MAX11261_CTRL3_NOSCG_POS 1 /**< NOSCG Position */
#define MAX11261_CTRL3_NOSCG ((uint8_t)(0x01UL << MAX11261_CTRL3_NOSCG_POS))

#define MAX11261_CTRL3_NOSYSO_POS 2 /**< NOSYSO Position */
#define MAX11261_CTRL3_NOSYSO ((uint8_t)(0x01UL << MAX11261_CTRL3_NOSYSO_POS))

#define MAX11261_CTRL3_NOSYSG_POS 3 /**< NOSYSG Position */
#define MAX11261_CTRL3_NOSYSG ((uint8_t)(0x01UL << MAX11261_CTRL3_NOSYSG_POS))

#define MAX11261_CTRL3_CALREGSEL_POS 4 /**< CALREGSEL Position */
#define MAX11261_CTRL3_CALREGSEL ((uint8_t)(0x01UL << MAX11261_CTRL3_CALREGSEL_POS))

#define MAX11261_CTRL3_SYNC_POS 5 /**< SYNC Position */
#define MAX11261_CTRL3_SYNC ((uint8_t)(0x01UL << MAX11261_CTRL3_SYNC_POS))

#define MAX11261_CTRL3_SYNCZ_POS 6 /**< SYNCZ Position */
#define MAX11261_CTRL3_SYNCZ ((uint8_t)(0x01UL << MAX11261_CTRL3_SYNCZ_POS))

/**@} end of group MAX11261_CTRL3_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_SEQ MAX11261_SEQ
 * @brief    Sequencer Register.
 * @{
 */
#define MAX11261_SEQ 0x04

#define MAX11261_SEQ_SIF_FREQ_POS 0 /**< SIF_FREQ Position */
#define MAX11261_SEQ_SIF_FREQ ((uint32_t)(0x03UL << MAX11261_SEQ_SIF_FREQ_POS))

#define MAX11261_SEQ_RDYBEN_POS 8 /**< RDYBEN Position */
#define MAX11261_SEQ_RDYBEN ((uint32_t)(0x01UL << MAX11261_SEQ_RDYBEN_POS))

#define MAX11261_SEQ_MDREN_POS 9 /**< MDREN Position */
#define MAX11261_SEQ_MDREN ((uint32_t)(0x01UL << MAX11261_SEQ_MDREN_POS))

#define MAX11261_SEQ_GPODREN_POS 10 /**< GPODREN Position */
#define MAX11261_SEQ_GPODREN ((uint32_t)(0x01UL << MAX11261_SEQ_GPODREN_POS))

#define MAX11261_SEQ_MODE_POS 11 /**< MODE Position */
#define MAX11261_SEQ_MODE ((uint32_t)(0x03UL << MAX11261_SEQ_MODE_POS))

#define MAX11261_SEQ_MUX_POS 13 /**< MUX Position */
#define MAX11261_SEQ_MUX ((uint32_t)(0x07UL << MAX11261_SEQ_MUX_POS))

/**@} end of group MAX11261_SEQ_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_CHMAP1 MAX11261_CHMAP1
 * @brief    Channel Map Register.
 * @{
 */
#define MAX11261_CHMAP1 0x05

#define MAX11261_CHMAP1_CH3_GPOEN_POS 0 /**< CH3_GPOEN Position */
#define MAX11261_CHMAP1_CH3_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH3_GPOEN_POS))

#define MAX11261_CHMAP1_CH3_EN_POS 1 /**< CH3_EN Position */
#define MAX11261_CHMAP1_CH3_EN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH3_EN_POS))

#define MAX11261_CHMAP1_CH3_ORD_POS 2 /**< CH3_ORD Position */
#define MAX11261_CHMAP1_CH3_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH3_ORD_POS))

#define MAX11261_CHMAP1_CH3_GPO_POS 5 /**< CH3_GPO Position */
#define MAX11261_CHMAP1_CH3_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH3_GPO_POS))

#define MAX11261_CHMAP1_CH4_GPOEN_POS 8 /**< CH4_GPOEN Position */
#define MAX11261_CHMAP1_CH4_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH4_GPOEN_POS))

#define MAX11261_CHMAP1_CH4_EN_POS 9 /**< CH4_EN Position */
#define MAX11261_CHMAP1_CH4_EN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH4_EN_POS))

#define MAX11261_CHMAP1_CH4_ORD_POS 10 /**< CH4_ORD Position */
#define MAX11261_CHMAP1_CH4_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH4_ORD_POS))

#define MAX11261_CHMAP1_CH4_GPO_POS 13 /**< CH4_GPO Position */
#define MAX11261_CHMAP1_CH4_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH4_GPO_POS))

#define MAX11261_CHMAP1_CH5_GPOEN_POS 16 /**< CH5_GPOEN Position */
#define MAX11261_CHMAP1_CH5_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH5_GPOEN_POS))

#define MAX11261_CHMAP1_CH5_EN_POS 17 /**< CH5_EN Position */
#define MAX11261_CHMAP1_CH5_EN ((uint32_t)(0x01UL << MAX11261_CHMAP1_CH5_EN_POS))

#define MAX11261_CHMAP1_CH5_ORD_POS 18 /**< CH5_ORD Position */
#define MAX11261_CHMAP1_CH5_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH5_ORD_POS))

#define MAX11261_CHMAP1_CH5_GPO_POS 21 /**< CH5_GPO Position */
#define MAX11261_CHMAP1_CH5_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP1_CH5_GPO_POS))

/**@} end of group MAX11261_CHMAP1_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_CHMAP0 MAX11261_CHMAP0
 * @brief    Channel Map Register.
 * @{
 */
#define MAX11261_CHMAP0 0x06

#define MAX11261_CHMAP0_CH0_GPOEN_POS 0 /**< CH0_GPOEN Position */
#define MAX11261_CHMAP0_CH0_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH0_GPOEN_POS))

#define MAX11261_CHMAP0_CH0_EN_POS 1 /**< CH0_EN Position */
#define MAX11261_CHMAP0_CH0_EN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH0_EN_POS))

#define MAX11261_CHMAP0_CH0_ORD_POS 2 /**< CH0_ORD Position */
#define MAX11261_CHMAP0_CH0_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH0_ORD_POS))

#define MAX11261_CHMAP0_CH0_GPO_POS 5 /**< CH0_GPO Position */
#define MAX11261_CHMAP0_CH0_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH0_GPO_POS))

#define MAX11261_CHMAP0_CH1_GPOEN_POS 8 /**< CH1_GPOEN Position */
#define MAX11261_CHMAP0_CH1_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH1_GPOEN_POS))

#define MAX11261_CHMAP0_CH1_EN_POS 9 /**< CH1_EN Position */
#define MAX11261_CHMAP0_CH1_EN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH1_EN_POS))

#define MAX11261_CHMAP0_CH1_ORD_POS 10 /**< CH1_ORD Position */
#define MAX11261_CHMAP0_CH1_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH1_ORD_POS))

#define MAX11261_CHMAP0_CH1_GPO_POS 13 /**< CH1_GPO Position */
#define MAX11261_CHMAP0_CH1_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH1_GPO_POS))

#define MAX11261_CHMAP0_CH2_GPOEN_POS 16 /**< CH2_GPOEN Position */
#define MAX11261_CHMAP0_CH2_GPOEN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH2_GPOEN_POS))

#define MAX11261_CHMAP0_CH2_EN_POS 17 /**< CH2_EN Position */
#define MAX11261_CHMAP0_CH2_EN ((uint32_t)(0x01UL << MAX11261_CHMAP0_CH2_EN_POS))

#define MAX11261_CHMAP0_CH2_ORD_POS 18 /**< CH2_ORD Position */
#define MAX11261_CHMAP0_CH2_ORD ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH2_ORD_POS))

#define MAX11261_CHMAP0_CH2_GPO_POS 21 /**< CH2_GPO Position */
#define MAX11261_CHMAP0_CH2_GPO ((uint32_t)(0x07UL << MAX11261_CHMAP0_CH2_GPO_POS))

/**@} end of group MAX11261_CHMAP0_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_DELAY MAX11261_DELAY
 * @brief    Delay Register.
 * @{
 */
#define MAX11261_DELAY 0x07

#define MAX11261_DELAY_GPO_POS 0 /**< GPO Position */
#define MAX11261_DELAY_GPO ((uint32_t)(0xFFUL << MAX11261_DELAY_GPO_POS))

#define MAX11261_DELAY_MUX_POS 8 /**< MUX Position */
#define MAX11261_DELAY_MUX ((uint32_t)(0xFFUL << MAX11261_DELAY_MUX_POS))

#define MAX11261_DELAY_AUTOSCAN_POS 16 /**< AUTOSCAN Position */
#define MAX11261_DELAY_AUTOSCAN ((uint32_t)(0xFFUL << MAX11261_DELAY_AUTOSCAN_POS))

/**@} end of group MAX11261_DELAY_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW0 MAX11261_LIMIT_LOW0
 * @brief    Lower Limit Register for Channel 0.
 * @{
 */
#define MAX11261_LIMIT_LOW0 0x08

#define MAX11261_LIMIT_LOW0_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW0_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW0_D_POS))

/**@} end of group MAX11261_LIMIT_LOW0_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW1 MAX11261_LIMIT_LOW1
 * @brief    Lower Limit Register for Channel 1.
 * @{
 */
#define MAX11261_LIMIT_LOW1 0x09

#define MAX11261_LIMIT_LOW1_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW1_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW1_D_POS))

/**@} end of group MAX11261_LIMIT_LOW1_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW2 MAX11261_LIMIT_LOW2
 * @brief    Lower Limit Register for Channel 2.
 * @{
 */
#define MAX11261_LIMIT_LOW2 0x10

#define MAX11261_LIMIT_LOW2_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW2_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW2_D_POS))

/**@} end of group MAX11261_LIMIT_LOW2_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW3 MAX11261_LIMIT_LOW3
 * @brief    Lower Limit Register for Channel 3.
 * @{
 */
#define MAX11261_LIMIT_LOW3 0x11

#define MAX11261_LIMIT_LOW3_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW3_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW3_D_POS))

/**@} end of group MAX11261_LIMIT_LOW3_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW4 MAX11261_LIMIT_LOW4
 * @brief    Lower Limit Register for Channel 4.
 * @{
 */
#define MAX11261_LIMIT_LOW4 0x12

#define MAX11261_LIMIT_LOW4_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW4_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW4_D_POS))

/**@} end of group MAX11261_LIMIT_LOW4_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_LOW5 MAX11261_LIMIT_LOW5
 * @brief    Lower Limit Register for Channel 5.
 * @{
 */
#define MAX11261_LIMIT_LOW5 0x13

#define MAX11261_LIMIT_LOW5_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_LOW5_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_LOW5_D_POS))

/**@} end of group MAX11261_LIMIT_LOW5_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_SOC MAX11261_SOC
 * @brief    System Offset Calibration Register.
 * @{
 */
#define MAX11261_SOC 0x0E

#define MAX11261_SOC_D_POS 0 /**< D Position */
#define MAX11261_SOC_D ((uint32_t)(0xFFFFFFUL << MAX11261_SOC_D_POS))

/**@} end of group MAX11261_SOC_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_SGC MAX11261_SGC
 * @brief    System Gain Calibration Register.
 * @{
 */
#define MAX11261_SGC 0x0F

#define MAX11261_SGC_D_POS 0 /**< D Position */
#define MAX11261_SGC_D ((uint32_t)(0xFFFFFFUL << MAX11261_SGC_D_POS))

/**@} end of group MAX11261_SGC_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_SCOC MAX11261_SCOC
 * @brief    Self-Calibration Offset Calibration Register.
 * @{
 */
#define MAX11261_SCOC 0x10

#define MAX11261_SCOC_D_POS 0 /**< D Position */
#define MAX11261_SCOC_D ((uint32_t)(0xFFFFFFUL << MAX11261_SCOC_D_POS))

/**@} end of group MAX11261_SCOC_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_SCGC MAX11261_SCGC
 * @brief    Self-Calibration Gain Calibration Register.
 * @{
 */
#define MAX11261_SCGC 0x11

#define MAX11261_SCGC_D_POS 0 /**< D Position */
#define MAX11261_SCGC_D ((uint32_t)(0xFFFFFFUL << MAX11261_SCGC_D_POS))

/**@} end of group MAX11261_SCGC_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_GPO_DIR MAX11261_GPO_DIR
 * @brief    GPO Direct Access Register.
 * @{
 */
#define MAX11261_GPO_DIR 0x12

#define MAX11261_GPO_DIR_GPO_POS 0 /**< GPO Position */
#define MAX11261_GPO_DIR_GPO ((uint32_t)(0x1FUL << MAX11261_GPO_DIR_GPO_POS))

/**@} end of group MAX11261_GPO_DIR_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_FIFO MAX11261_FIFO
 * @brief    FIFO Register.
 * @{
 */
#define MAX11261_FIFO 0x13

#define MAX11261_FIFO_D_POS 0 /**< D Position */
#define MAX11261_FIFO_D ((uint32_t)(0xFFFFFFUL << MAX11261_FIFO_D_POS))

#define MAX11261_FIFO_OOR_POS 24 /**< OOR Position */
#define MAX11261_FIFO_OOR ((uint32_t)(0x01UL << MAX11261_FIFO_OOR_POS))

#define MAX11261_FIFO_CH_POS 25 /**< CH Position */
#define MAX11261_FIFO_CH ((uint32_t)(0x07UL << MAX11261_FIFO_CH_POS))

#define MAX11261_FIFO_OVW_POS 31 /**< OVW Position */
#define MAX11261_FIFO_OVW ((uint32_t)(0x01UL << MAX11261_FIFO_OVW_POS))

/**@} end of group MAX11261_FIFO_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_FIFO_LEVEL MAX11261_FIFO_LEVEL
 * @brief    FIFO Level Register.
 * @{
 */
#define MAX11261_FIFO_LEVEL 0x14

#define MAX11261_FIFO_LEVEL_FIFO_LEVEL_POS 0 /**< FIFO_LEVEL Position */
#define MAX11261_FIFO_LEVEL_FIFO_LEVEL ((uint8_t)(0x7FUL << MAX11261_FIFO_LEVEL_FIFO_LEVEL_POS))

/**@} end of group MAX11261_FIFO_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_FIFO_CTRL MAX11261_FIFO_CTRL
 * @brief    FIFO Control Register.
 * @{
 */
#define MAX11261_FIFO_CTRL 0x15

#define MAX11261_FIFO_CTRL_RST_POS 0 /**< RST Position */
#define MAX11261_FIFO_CTRL_RST ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_RST_POS))

#define MAX11261_FIFO_CTRL_FIFO1_8_INTEN_POS 1 /**< FIFO1_8_INTEN Position */
#define MAX11261_FIFO_CTRL_FIFO1_8_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_FIFO1_8_INTEN_POS))

#define MAX11261_FIFO_CTRL_FIFO2_8_INTEN_POS 2 /**< FIFO2_8_INTEN Position */
#define MAX11261_FIFO_CTRL_FIFO2_8_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_FIFO2_8_INTEN_POS))

#define MAX11261_FIFO_CTRL_FIFO4_8_INTEN_POS 3 /**< FIFO4_8_INTEN Position */
#define MAX11261_FIFO_CTRL_FIFO4_8_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_FIFO4_8_INTEN_POS))

#define MAX11261_FIFO_CTRL_FIFO6_8_INTEN_POS 4 /**< FIFO6_8_INTEN Position */
#define MAX11261_FIFO_CTRL_FIFO6_8_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_FIFO6_8_INTEN_POS))

#define MAX11261_FIFO_CTRL_FIFO7_8_INTEN_POS 5 /**< FIFO7_8_INTEN Position */
#define MAX11261_FIFO_CTRL_FIFO7_8_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_FIFO7_8_INTEN_POS))

#define MAX11261_FIFO_CTRL_OVW_INTEN_POS 6 /**< OVW_INTEN Position */
#define MAX11261_FIFO_CTRL_OVW_INTEN ((uint8_t)(0x01UL << MAX11261_FIFO_CTRL_OVW_INTEN_POS))

/**@} end of group MAX11261_FIFO_CTRL_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_INPUT_INT_EN MAX11261_INPUT_INT_EN
 * @brief    Input Interrupt Enable Register.
 * @{
 */
#define MAX11261_INPUT_INT_EN 0x16

#define MAX11261_INPUT_INT_EN_CH_POS 0 /**< CH Position */
#define MAX11261_INPUT_INT_EN_CH ((uint8_t)(0x3FUL << MAX11261_INPUT_INT_EN_CH_POS))

#define MAX11261_INPUT_INT_EN_RDYB_POS 7 /**< RDYB Position */
#define MAX11261_INPUT_INT_EN_RDYB ((uint8_t)(0x01UL << MAX11261_INPUT_INT_EN_RDYB_POS))

/**@} end of group MAX11261_INPUT_INT_EN_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_INT_STAT MAX11261_INT_STAT
 * @brief    Interrupt Status Register.
 * @{
 */
#define MAX11261_INT_STAT 0x17

#define MAX11261_INPUT_INT_STAT_CH_POS 0 /**< CH Position */
#define MAX11261_INPUT_INT_STAT_CH ((uint32_t)(0x3FUL << MAX11261_INPUT_INT_STAT_CH_POS))

#define MAX11261_INPUT_INT_STAT_FIFO1_8_POS 9 /**< FIFO1_8 Position */
#define MAX11261_INPUT_INT_STAT_FIFO1_8 ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO1_8_POS))

#define MAX11261_INPUT_INT_STAT_FIFO2_8_POS 10 /**< FIFO2_8 Position */
#define MAX11261_INPUT_INT_STAT_FIFO2_8 ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO2_8_POS))

#define MAX11261_INPUT_INT_STAT_FIFO4_8_POS 11 /**< FIFO4_8 Position */
#define MAX11261_INPUT_INT_STAT_FIFO4_8 ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO4_8_POS))

#define MAX11261_INPUT_INT_STAT_FIFO6_8_POS 12 /**< FIFO6_8 Position */
#define MAX11261_INPUT_INT_STAT_FIFO6_8 ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO6_8_POS))

#define MAX11261_INPUT_INT_STAT_FIFO7_8_POS 13 /**< FIFO7_8 Position */
#define MAX11261_INPUT_INT_STAT_FIFO7_8 ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO7_8_POS))

#define MAX11261_INPUT_INT_STAT_FIFO_OVW_POS 14 /**< FIFO_OVW Position */
#define MAX11261_INPUT_INT_STAT_FIFO_OVW \
    ((uint32_t)(0x01UL << MAX11261_INPUT_INT_STAT_FIFO_OVW_POS))

/**@} end of group MAX11261_INT_STAT_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_HPF MAX11261_HPF
 * @brief    High-Pass Digital Filter Control Register.
 * @{
 */
#define MAX11261_HPF 0x18

#define MAX11261_HPF_FREQUENCY_POS 0 /**< FREQUENCY Position */
#define MAX11261_HPF_FREQUENCY ((uint8_t)(0x07UL << MAX11261_HPF_FREQUENCY_POS))

#define MAX11261_HPF_CMP_MODE_POS 3 /**< CMP_MODE Position */
#define MAX11261_HPF_CMP_MODE ((uint8_t)(0x03UL << MAX11261_HPF_CMP_MODE_POS))

/**@} end of group MAX11261_HPF_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH0 MAX11261_LIMIT_HIGH0
 * @brief    Higher Limit Register for Channel 0.
 * @{
 */
#define MAX11261_LIMIT_HIGH0 0x19

#define MAX11261_LIMIT_HIGH0_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH0_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH0_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH0_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH1 MAX11261_LIMIT_HIGH1
 * @brief    Higher Limit Register for Channel 1.
 * @{
 */
#define MAX11261_LIMIT_HIGH1 0x1A

#define MAX11261_LIMIT_HIGH1_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH1_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH1_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH1_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH2 MAX11261_LIMIT_HIGH2
 * @brief    Higher Limit Register for Channel 2.
 * @{
 */
#define MAX11261_LIMIT_HIGH2 0x1B

#define MAX11261_LIMIT_HIGH2_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH2_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH2_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH2_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH3 MAX11261_LIMIT_HIGH3
 * @brief    Higher Limit Register for Channel 3.
 * @{
 */
#define MAX11261_LIMIT_HIGH3 0x1C

#define MAX11261_LIMIT_HIGH3_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH3_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH3_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH3_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH4 MAX11261_LIMIT_HIGH4
 * @brief    Higher Limit Register for Channel 4.
 * @{
 */
#define MAX11261_LIMIT_HIGH4 0x1D

#define MAX11261_LIMIT_HIGH4_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH4_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH4_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH4_Register */

/**
 * @ingroup  max11261_registers
 * @defgroup MAX11261_LIMIT_HIGH5 MAX11261_LIMIT_HIGH5
 * @brief    Higher Limit Register for Channel 5.
 * @{
 */
#define MAX11261_LIMIT_HIGH5 0x1E

#define MAX11261_LIMIT_HIGH5_D_POS 0 /**< D Position */
#define MAX11261_LIMIT_HIGH5_D ((uint32_t)(0xFFFFFFUL << MAX11261_LIMIT_HIGH5_D_POS))

/**@} end of group MAX11261_LIMIT_HIGH5_Register */

#endif /* MAX11261_REGS_H_ */
