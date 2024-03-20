/**
 * @file    spixfc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixfc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     spixfc
 * @defgroup    spixfc_registers SPIXFC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @details     SPI XiP Flash Configuration Controller
 */

/**
 * @ingroup spixfc_registers
 * Structure type to access the SPIXFC Registers.
 */
typedef struct {
    __IO uint32_t ctrl0;                /**< <tt>\b 0x00:</tt> SPIXFC CTRL0 Register */
    __IO uint32_t sspol;                /**< <tt>\b 0x04:</tt> SPIXFC SSPOL Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x08:</tt> SPIXFC CTRL1 Register */
    __IO uint32_t ctrl2;                /**< <tt>\b 0x0C:</tt> SPIXFC CTRL2 Register */
    __IO uint32_t ctrl3;                /**< <tt>\b 0x10:</tt> SPIXFC CTRL3 Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x14:</tt> SPIXFC INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x18:</tt> SPIXFC INTEN Register */
    __IO uint32_t header;               /**< <tt>\b 0x1C:</tt> SPIXFC HEADER Register */
    __IO uint32_t autoctrl;             /**< <tt>\b 0x20:</tt> SPIXFC AUTOCTRL Register */
    __IO uint32_t autocmd;              /**< <tt>\b 0x24:</tt> SPIXFC AUTOCMD Register */
} mxc_spixfc_regs_t;

/* Register offsets for module SPIXFC */
/**
 * @ingroup    spixfc_registers
 * @defgroup   SPIXFC_Register_Offsets Register Offsets
 * @brief      SPIXFC Peripheral Register Offsets from the SPIXFC Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXFC_CTRL0                 ((uint32_t)0x00000000UL) /**< Offset from SPIXFC Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_SSPOL                 ((uint32_t)0x00000004UL) /**< Offset from SPIXFC Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFC_CTRL1                 ((uint32_t)0x00000008UL) /**< Offset from SPIXFC Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXFC_CTRL2                 ((uint32_t)0x0000000CUL) /**< Offset from SPIXFC Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXFC_CTRL3                 ((uint32_t)0x00000010UL) /**< Offset from SPIXFC Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXFC_INTFL                 ((uint32_t)0x00000014UL) /**< Offset from SPIXFC Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXFC_INTEN                 ((uint32_t)0x00000018UL) /**< Offset from SPIXFC Base Address: <tt> 0x0018</tt> */
#define MXC_R_SPIXFC_HEADER                ((uint32_t)0x0000001CUL) /**< Offset from SPIXFC Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXFC_AUTOCTRL              ((uint32_t)0x00000020UL) /**< Offset from SPIXFC Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXFC_AUTOCMD               ((uint32_t)0x00000024UL) /**< Offset from SPIXFC Base Address: <tt> 0x0024</tt> */
/**@} end of group spixfc_registers */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CTRL0 SPIXFC_CTRL0
 * @brief    Control Register.
 * @{
 */
#define MXC_F_SPIXFC_CTRL0_SSEL_POS                    0 /**< CTRL0_SSEL Position */
#define MXC_F_SPIXFC_CTRL0_SSEL                        ((uint32_t)(0x7UL << MXC_F_SPIXFC_CTRL0_SSEL_POS)) /**< CTRL0_SSEL Mask */
#define MXC_V_SPIXFC_CTRL0_SSEL_SLAVE_0                ((uint32_t)0x0UL) /**< CTRL0_SSEL_SLAVE_0 Value */
#define MXC_S_SPIXFC_CTRL0_SSEL_SLAVE_0                (MXC_V_SPIXFC_CTRL0_SSEL_SLAVE_0 << MXC_F_SPIXFC_CTRL0_SSEL_POS) /**< CTRL0_SSEL_SLAVE_0 Setting */
#define MXC_V_SPIXFC_CTRL0_SSEL_SLAVE_1                ((uint32_t)0x1UL) /**< CTRL0_SSEL_SLAVE_1 Value */
#define MXC_S_SPIXFC_CTRL0_SSEL_SLAVE_1                (MXC_V_SPIXFC_CTRL0_SSEL_SLAVE_1 << MXC_F_SPIXFC_CTRL0_SSEL_POS) /**< CTRL0_SSEL_SLAVE_1 Setting */

#define MXC_F_SPIXFC_CTRL0_THREE_WIRE_POS              3 /**< CTRL0_THREE_WIRE Position */
#define MXC_F_SPIXFC_CTRL0_THREE_WIRE                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL0_THREE_WIRE_POS)) /**< CTRL0_THREE_WIRE Mask */

#define MXC_F_SPIXFC_CTRL0_MODE_POS                    4 /**< CTRL0_MODE Position */
#define MXC_F_SPIXFC_CTRL0_MODE                        ((uint32_t)(0x3UL << MXC_F_SPIXFC_CTRL0_MODE_POS)) /**< CTRL0_MODE Mask */
#define MXC_V_SPIXFC_CTRL0_MODE_SPI_MODE_0             ((uint32_t)0x0UL) /**< CTRL0_MODE_SPI_MODE_0 Value */
#define MXC_S_SPIXFC_CTRL0_MODE_SPI_MODE_0             (MXC_V_SPIXFC_CTRL0_MODE_SPI_MODE_0 << MXC_F_SPIXFC_CTRL0_MODE_POS) /**< CTRL0_MODE_SPI_MODE_0 Setting */
#define MXC_V_SPIXFC_CTRL0_MODE_SPI_MODE_3             ((uint32_t)0x3UL) /**< CTRL0_MODE_SPI_MODE_3 Value */
#define MXC_S_SPIXFC_CTRL0_MODE_SPI_MODE_3             (MXC_V_SPIXFC_CTRL0_MODE_SPI_MODE_3 << MXC_F_SPIXFC_CTRL0_MODE_POS) /**< CTRL0_MODE_SPI_MODE_3 Setting */

#define MXC_F_SPIXFC_CTRL0_PGSZ_POS                    6 /**< CTRL0_PGSZ Position */
#define MXC_F_SPIXFC_CTRL0_PGSZ                        ((uint32_t)(0x3UL << MXC_F_SPIXFC_CTRL0_PGSZ_POS)) /**< CTRL0_PGSZ Mask */
#define MXC_V_SPIXFC_CTRL0_PGSZ_4_BYTES                ((uint32_t)0x0UL) /**< CTRL0_PGSZ_4_BYTES Value */
#define MXC_S_SPIXFC_CTRL0_PGSZ_4_BYTES                (MXC_V_SPIXFC_CTRL0_PGSZ_4_BYTES << MXC_F_SPIXFC_CTRL0_PGSZ_POS) /**< CTRL0_PGSZ_4_BYTES Setting */
#define MXC_V_SPIXFC_CTRL0_PGSZ_8_BYTES                ((uint32_t)0x1UL) /**< CTRL0_PGSZ_8_BYTES Value */
#define MXC_S_SPIXFC_CTRL0_PGSZ_8_BYTES                (MXC_V_SPIXFC_CTRL0_PGSZ_8_BYTES << MXC_F_SPIXFC_CTRL0_PGSZ_POS) /**< CTRL0_PGSZ_8_BYTES Setting */
#define MXC_V_SPIXFC_CTRL0_PGSZ_16_BYTES               ((uint32_t)0x2UL) /**< CTRL0_PGSZ_16_BYTES Value */
#define MXC_S_SPIXFC_CTRL0_PGSZ_16_BYTES               (MXC_V_SPIXFC_CTRL0_PGSZ_16_BYTES << MXC_F_SPIXFC_CTRL0_PGSZ_POS) /**< CTRL0_PGSZ_16_BYTES Setting */
#define MXC_V_SPIXFC_CTRL0_PGSZ_32_BYTES               ((uint32_t)0x3UL) /**< CTRL0_PGSZ_32_BYTES Value */
#define MXC_S_SPIXFC_CTRL0_PGSZ_32_BYTES               (MXC_V_SPIXFC_CTRL0_PGSZ_32_BYTES << MXC_F_SPIXFC_CTRL0_PGSZ_POS) /**< CTRL0_PGSZ_32_BYTES Setting */

#define MXC_F_SPIXFC_CTRL0_HICLK_POS                   8 /**< CTRL0_HICLK Position */
#define MXC_F_SPIXFC_CTRL0_HICLK                       ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL0_HICLK_POS)) /**< CTRL0_HICLK Mask */
#define MXC_V_SPIXFC_CTRL0_HICLK_16_SCLK               ((uint32_t)0x0UL) /**< CTRL0_HICLK_16_SCLK Value */
#define MXC_S_SPIXFC_CTRL0_HICLK_16_SCLK               (MXC_V_SPIXFC_CTRL0_HICLK_16_SCLK << MXC_F_SPIXFC_CTRL0_HICLK_POS) /**< CTRL0_HICLK_16_SCLK Setting */

#define MXC_F_SPIXFC_CTRL0_LOCLK_POS                   12 /**< CTRL0_LOCLK Position */
#define MXC_F_SPIXFC_CTRL0_LOCLK                       ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL0_LOCLK_POS)) /**< CTRL0_LOCLK Mask */
#define MXC_V_SPIXFC_CTRL0_LOCLK_16_SCLK               ((uint32_t)0x0UL) /**< CTRL0_LOCLK_16_SCLK Value */
#define MXC_S_SPIXFC_CTRL0_LOCLK_16_SCLK               (MXC_V_SPIXFC_CTRL0_LOCLK_16_SCLK << MXC_F_SPIXFC_CTRL0_LOCLK_POS) /**< CTRL0_LOCLK_16_SCLK Setting */

#define MXC_F_SPIXFC_CTRL0_SSACT_POS                   16 /**< CTRL0_SSACT Position */
#define MXC_F_SPIXFC_CTRL0_SSACT                       ((uint32_t)(0x3UL << MXC_F_SPIXFC_CTRL0_SSACT_POS)) /**< CTRL0_SSACT Mask */
#define MXC_V_SPIXFC_CTRL0_SSACT_0_CLKS                ((uint32_t)0x0UL) /**< CTRL0_SSACT_0_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSACT_0_CLKS                (MXC_V_SPIXFC_CTRL0_SSACT_0_CLKS << MXC_F_SPIXFC_CTRL0_SSACT_POS) /**< CTRL0_SSACT_0_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSACT_2_CLKS                ((uint32_t)0x1UL) /**< CTRL0_SSACT_2_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSACT_2_CLKS                (MXC_V_SPIXFC_CTRL0_SSACT_2_CLKS << MXC_F_SPIXFC_CTRL0_SSACT_POS) /**< CTRL0_SSACT_2_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSACT_4_CLKS                ((uint32_t)0x2UL) /**< CTRL0_SSACT_4_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSACT_4_CLKS                (MXC_V_SPIXFC_CTRL0_SSACT_4_CLKS << MXC_F_SPIXFC_CTRL0_SSACT_POS) /**< CTRL0_SSACT_4_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSACT_8_CLKS                ((uint32_t)0x3UL) /**< CTRL0_SSACT_8_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSACT_8_CLKS                (MXC_V_SPIXFC_CTRL0_SSACT_8_CLKS << MXC_F_SPIXFC_CTRL0_SSACT_POS) /**< CTRL0_SSACT_8_CLKS Setting */

#define MXC_F_SPIXFC_CTRL0_SSINACT_POS                 18 /**< CTRL0_SSINACT Position */
#define MXC_F_SPIXFC_CTRL0_SSINACT                     ((uint32_t)(0x3UL << MXC_F_SPIXFC_CTRL0_SSINACT_POS)) /**< CTRL0_SSINACT Mask */
#define MXC_V_SPIXFC_CTRL0_SSINACT_4_CLKS              ((uint32_t)0x0UL) /**< CTRL0_SSINACT_4_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSINACT_4_CLKS              (MXC_V_SPIXFC_CTRL0_SSINACT_4_CLKS << MXC_F_SPIXFC_CTRL0_SSINACT_POS) /**< CTRL0_SSINACT_4_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSINACT_6_CLKS              ((uint32_t)0x1UL) /**< CTRL0_SSINACT_6_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSINACT_6_CLKS              (MXC_V_SPIXFC_CTRL0_SSINACT_6_CLKS << MXC_F_SPIXFC_CTRL0_SSINACT_POS) /**< CTRL0_SSINACT_6_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSINACT_8_CLKS              ((uint32_t)0x2UL) /**< CTRL0_SSINACT_8_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSINACT_8_CLKS              (MXC_V_SPIXFC_CTRL0_SSINACT_8_CLKS << MXC_F_SPIXFC_CTRL0_SSINACT_POS) /**< CTRL0_SSINACT_8_CLKS Setting */
#define MXC_V_SPIXFC_CTRL0_SSINACT_12_CLKS             ((uint32_t)0x3UL) /**< CTRL0_SSINACT_12_CLKS Value */
#define MXC_S_SPIXFC_CTRL0_SSINACT_12_CLKS             (MXC_V_SPIXFC_CTRL0_SSINACT_12_CLKS << MXC_F_SPIXFC_CTRL0_SSINACT_POS) /**< CTRL0_SSINACT_12_CLKS Setting */

#define MXC_F_SPIXFC_CTRL0_IOSMPL_POS                  20 /**< CTRL0_IOSMPL Position */
#define MXC_F_SPIXFC_CTRL0_IOSMPL                      ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL0_IOSMPL_POS)) /**< CTRL0_IOSMPL Mask */

/**@} end of group SPIXFC_CTRL0_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_SSPOL SPIXFC_SSPOL
 * @brief    SPIX Controller Slave Select Polarity Register.
 * @{
 */
#define MXC_F_SPIXFC_SSPOL_SSPOL_POS                   0 /**< SSPOL_SSPOL Position */
#define MXC_F_SPIXFC_SSPOL_SSPOL                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_SSPOL_SSPOL_POS)) /**< SSPOL_SSPOL Mask */

#define MXC_F_SPIXFC_SSPOL_FCPOL_POS                   8 /**< SSPOL_FCPOL Position */
#define MXC_F_SPIXFC_SSPOL_FCPOL                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_SSPOL_FCPOL_POS)) /**< SSPOL_FCPOL Mask */

/**@} end of group SPIXFC_SSPOL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CTRL1 SPIXFC_CTRL1
 * @brief    SPIX Controller General Controller Register.
 * @{
 */
#define MXC_F_SPIXFC_CTRL1_EN_POS                      0 /**< CTRL1_EN Position */
#define MXC_F_SPIXFC_CTRL1_EN                          ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_EN_POS)) /**< CTRL1_EN Mask */

#define MXC_F_SPIXFC_CTRL1_TX_FIFO_EN_POS              1 /**< CTRL1_TX_FIFO_EN Position */
#define MXC_F_SPIXFC_CTRL1_TX_FIFO_EN                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_TX_FIFO_EN_POS)) /**< CTRL1_TX_FIFO_EN Mask */

#define MXC_F_SPIXFC_CTRL1_RX_FIFO_EN_POS              2 /**< CTRL1_RX_FIFO_EN Position */
#define MXC_F_SPIXFC_CTRL1_RX_FIFO_EN                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_RX_FIFO_EN_POS)) /**< CTRL1_RX_FIFO_EN Mask */

#define MXC_F_SPIXFC_CTRL1_BB_EN_POS                   3 /**< CTRL1_BB_EN Position */
#define MXC_F_SPIXFC_CTRL1_BB_EN                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_BB_EN_POS)) /**< CTRL1_BB_EN Mask */

#define MXC_F_SPIXFC_CTRL1_SSDR_POS                    4 /**< CTRL1_SSDR Position */
#define MXC_F_SPIXFC_CTRL1_SSDR                        ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SSDR_POS)) /**< CTRL1_SSDR Mask */

#define MXC_F_SPIXFC_CTRL1_FCDR_POS                    5 /**< CTRL1_FCDR Position */
#define MXC_F_SPIXFC_CTRL1_FCDR                        ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_FCDR_POS)) /**< CTRL1_FCDR Mask */

#define MXC_F_SPIXFC_CTRL1_SCLKDR_POS                  6 /**< CTRL1_SCLKDR Position */
#define MXC_F_SPIXFC_CTRL1_SCLKDR                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SCLKDR_POS)) /**< CTRL1_SCLKDR Mask */

#define MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS            8 /**< CTRL1_SDIO_DATA_IN Position */
#define MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN                ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS)) /**< CTRL1_SDIO_DATA_IN Mask */
#define MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO0          ((uint32_t)0x0UL) /**< CTRL1_SDIO_DATA_IN_SDIO0 Value */
#define MXC_S_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO0          (MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO0 << MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS) /**< CTRL1_SDIO_DATA_IN_SDIO0 Setting */
#define MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO1          ((uint32_t)0x1UL) /**< CTRL1_SDIO_DATA_IN_SDIO1 Value */
#define MXC_S_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO1          (MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO1 << MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS) /**< CTRL1_SDIO_DATA_IN_SDIO1 Setting */
#define MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO2          ((uint32_t)0x2UL) /**< CTRL1_SDIO_DATA_IN_SDIO2 Value */
#define MXC_S_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO2          (MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO2 << MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS) /**< CTRL1_SDIO_DATA_IN_SDIO2 Setting */
#define MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO3          ((uint32_t)0x3UL) /**< CTRL1_SDIO_DATA_IN_SDIO3 Value */
#define MXC_S_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO3          (MXC_V_SPIXFC_CTRL1_SDIO_DATA_IN_SDIO3 << MXC_F_SPIXFC_CTRL1_SDIO_DATA_IN_POS) /**< CTRL1_SDIO_DATA_IN_SDIO3 Setting */

#define MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS             12 /**< CTRL1_BB_DATA_OUT Position */
#define MXC_F_SPIXFC_CTRL1_BB_DATA_OUT                 ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS)) /**< CTRL1_BB_DATA_OUT Mask */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO0           ((uint32_t)0x0UL) /**< CTRL1_BB_DATA_OUT_SDIO0 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_SDIO0           (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO0 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS) /**< CTRL1_BB_DATA_OUT_SDIO0 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO1           ((uint32_t)0x1UL) /**< CTRL1_BB_DATA_OUT_SDIO1 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_SDIO1           (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO1 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS) /**< CTRL1_BB_DATA_OUT_SDIO1 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO2           ((uint32_t)0x2UL) /**< CTRL1_BB_DATA_OUT_SDIO2 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_SDIO2           (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO2 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS) /**< CTRL1_BB_DATA_OUT_SDIO2 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO3           ((uint32_t)0x3UL) /**< CTRL1_BB_DATA_OUT_SDIO3 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_SDIO3           (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_SDIO3 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_POS) /**< CTRL1_BB_DATA_OUT_SDIO3 Setting */

#define MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS          16 /**< CTRL1_BB_DATA_OUT_EN Position */
#define MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN              ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS)) /**< CTRL1_BB_DATA_OUT_EN Mask */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO0        ((uint32_t)0x0UL) /**< CTRL1_BB_DATA_OUT_EN_SDIO0 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO0        (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO0 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS) /**< CTRL1_BB_DATA_OUT_EN_SDIO0 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO1        ((uint32_t)0x1UL) /**< CTRL1_BB_DATA_OUT_EN_SDIO1 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO1        (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO1 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS) /**< CTRL1_BB_DATA_OUT_EN_SDIO1 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO2        ((uint32_t)0x2UL) /**< CTRL1_BB_DATA_OUT_EN_SDIO2 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO2        (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO2 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS) /**< CTRL1_BB_DATA_OUT_EN_SDIO2 Setting */
#define MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO3        ((uint32_t)0x3UL) /**< CTRL1_BB_DATA_OUT_EN_SDIO3 Value */
#define MXC_S_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO3        (MXC_V_SPIXFC_CTRL1_BB_DATA_OUT_EN_SDIO3 << MXC_F_SPIXFC_CTRL1_BB_DATA_OUT_EN_POS) /**< CTRL1_BB_DATA_OUT_EN_SDIO3 Setting */

#define MXC_F_SPIXFC_CTRL1_SIMPLE_EN_POS               20 /**< CTRL1_SIMPLE_EN Position */
#define MXC_F_SPIXFC_CTRL1_SIMPLE_EN                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SIMPLE_EN_POS)) /**< CTRL1_SIMPLE_EN Mask */

#define MXC_F_SPIXFC_CTRL1_SIMPLE_RX_POS               21 /**< CTRL1_SIMPLE_RX Position */
#define MXC_F_SPIXFC_CTRL1_SIMPLE_RX                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SIMPLE_RX_POS)) /**< CTRL1_SIMPLE_RX Mask */

#define MXC_F_SPIXFC_CTRL1_SIMPLE_SS_POS               22 /**< CTRL1_SIMPLE_SS Position */
#define MXC_F_SPIXFC_CTRL1_SIMPLE_SS                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SIMPLE_SS_POS)) /**< CTRL1_SIMPLE_SS Mask */

#define MXC_F_SPIXFC_CTRL1_SCLK_FB_POS                 24 /**< CTRL1_SCLK_FB Position */
#define MXC_F_SPIXFC_CTRL1_SCLK_FB                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SCLK_FB_POS)) /**< CTRL1_SCLK_FB Mask */

#define MXC_F_SPIXFC_CTRL1_SCLK_FB_INV_POS             25 /**< CTRL1_SCLK_FB_INV Position */
#define MXC_F_SPIXFC_CTRL1_SCLK_FB_INV                 ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL1_SCLK_FB_INV_POS)) /**< CTRL1_SCLK_FB_INV Mask */

/**@} end of group SPIXFC_CTRL1_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CTRL2 SPIXFC_CTRL2
 * @brief    SPIX Controller FIFO Control and Status Register.
 * @{
 */
#define MXC_F_SPIXFC_CTRL2_TX_AE_LVL_POS               0 /**< CTRL2_TX_AE_LVL Position */
#define MXC_F_SPIXFC_CTRL2_TX_AE_LVL                   ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL2_TX_AE_LVL_POS)) /**< CTRL2_TX_AE_LVL Mask */

#define MXC_F_SPIXFC_CTRL2_TX_CNT_POS                  8 /**< CTRL2_TX_CNT Position */
#define MXC_F_SPIXFC_CTRL2_TX_CNT                      ((uint32_t)(0x1FUL << MXC_F_SPIXFC_CTRL2_TX_CNT_POS)) /**< CTRL2_TX_CNT Mask */

#define MXC_F_SPIXFC_CTRL2_RX_AF_LVL_POS               16 /**< CTRL2_RX_AF_LVL Position */
#define MXC_F_SPIXFC_CTRL2_RX_AF_LVL                   ((uint32_t)(0x1FUL << MXC_F_SPIXFC_CTRL2_RX_AF_LVL_POS)) /**< CTRL2_RX_AF_LVL Mask */

#define MXC_F_SPIXFC_CTRL2_RX_CNT_POS                  24 /**< CTRL2_RX_CNT Position */
#define MXC_F_SPIXFC_CTRL2_RX_CNT                      ((uint32_t)(0x3FUL << MXC_F_SPIXFC_CTRL2_RX_CNT_POS)) /**< CTRL2_RX_CNT Mask */

/**@} end of group SPIXFC_CTRL2_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CTRL3 SPIXFC_CTRL3
 * @brief    SPIX Controller Special Control Register.
 * @{
 */
#define MXC_F_SPIXFC_CTRL3_SAMPLE_POS                  0 /**< CTRL3_SAMPLE Position */
#define MXC_F_SPIXFC_CTRL3_SAMPLE                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL3_SAMPLE_POS)) /**< CTRL3_SAMPLE Mask */

#define MXC_F_SPIXFC_CTRL3_MISO_FC_EN_POS              1 /**< CTRL3_MISO_FC_EN Position */
#define MXC_F_SPIXFC_CTRL3_MISO_FC_EN                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL3_MISO_FC_EN_POS)) /**< CTRL3_MISO_FC_EN Mask */

#define MXC_F_SPIXFC_CTRL3_SDIO_OUT_VAL_POS            4 /**< CTRL3_SDIO_OUT_VAL Position */
#define MXC_F_SPIXFC_CTRL3_SDIO_OUT_VAL                ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL3_SDIO_OUT_VAL_POS)) /**< CTRL3_SDIO_OUT_VAL Mask */

#define MXC_F_SPIXFC_CTRL3_SDIO_OUT_EN_POS             8 /**< CTRL3_SDIO_OUT_EN Position */
#define MXC_F_SPIXFC_CTRL3_SDIO_OUT_EN                 ((uint32_t)(0xFUL << MXC_F_SPIXFC_CTRL3_SDIO_OUT_EN_POS)) /**< CTRL3_SDIO_OUT_EN Mask */

#define MXC_F_SPIXFC_CTRL3_SCLKINH3_POS                16 /**< CTRL3_SCLKINH3 Position */
#define MXC_F_SPIXFC_CTRL3_SCLKINH3                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_CTRL3_SCLKINH3_POS)) /**< CTRL3_SCLKINH3 Mask */

/**@} end of group SPIXFC_CTRL3_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INTFL SPIXFC_INTFL
 * @brief    SPIX Controller Interrupt Status Register.
 * @{
 */
#define MXC_F_SPIXFC_INTFL_TX_STALLED_POS              0 /**< INTFL_TX_STALLED Position */
#define MXC_F_SPIXFC_INTFL_TX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_STALLED_POS)) /**< INTFL_TX_STALLED Mask */

#define MXC_F_SPIXFC_INTFL_RX_STALLED_POS              1 /**< INTFL_RX_STALLED Position */
#define MXC_F_SPIXFC_INTFL_RX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_STALLED_POS)) /**< INTFL_RX_STALLED Mask */

#define MXC_F_SPIXFC_INTFL_TX_RDY_POS                  2 /**< INTFL_TX_RDY Position */
#define MXC_F_SPIXFC_INTFL_TX_RDY                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_RDY_POS)) /**< INTFL_TX_RDY Mask */

#define MXC_F_SPIXFC_INTFL_RX_DONE_POS                 3 /**< INTFL_RX_DONE Position */
#define MXC_F_SPIXFC_INTFL_RX_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_DONE_POS)) /**< INTFL_RX_DONE Mask */

#define MXC_F_SPIXFC_INTFL_TX_AE_POS                   4 /**< INTFL_TX_AE Position */
#define MXC_F_SPIXFC_INTFL_TX_AE                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_AE_POS)) /**< INTFL_TX_AE Mask */

#define MXC_F_SPIXFC_INTFL_RX_AF_POS                   5 /**< INTFL_RX_AF Position */
#define MXC_F_SPIXFC_INTFL_RX_AF                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_AF_POS)) /**< INTFL_RX_AF Mask */

/**@} end of group SPIXFC_INTFL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INTEN SPIXFC_INTEN
 * @brief    SPIX Controller Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPIXFC_INTEN_TX_STALLED_POS              0 /**< INTEN_TX_STALLED Position */
#define MXC_F_SPIXFC_INTEN_TX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_STALLED_POS)) /**< INTEN_TX_STALLED Mask */

#define MXC_F_SPIXFC_INTEN_RX_STALLED_POS              1 /**< INTEN_RX_STALLED Position */
#define MXC_F_SPIXFC_INTEN_RX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_STALLED_POS)) /**< INTEN_RX_STALLED Mask */

#define MXC_F_SPIXFC_INTEN_TX_RDY_POS                  2 /**< INTEN_TX_RDY Position */
#define MXC_F_SPIXFC_INTEN_TX_RDY                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_RDY_POS)) /**< INTEN_TX_RDY Mask */

#define MXC_F_SPIXFC_INTEN_RX_DONE_POS                 3 /**< INTEN_RX_DONE Position */
#define MXC_F_SPIXFC_INTEN_RX_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_DONE_POS)) /**< INTEN_RX_DONE Mask */

#define MXC_F_SPIXFC_INTEN_TX_AE_POS                   4 /**< INTEN_TX_AE Position */
#define MXC_F_SPIXFC_INTEN_TX_AE                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_AE_POS)) /**< INTEN_TX_AE Mask */

#define MXC_F_SPIXFC_INTEN_RX_AF_POS                   5 /**< INTEN_RX_AF Position */
#define MXC_F_SPIXFC_INTEN_RX_AF                       ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_AF_POS)) /**< INTEN_RX_AF Mask */

/**@} end of group SPIXFC_INTEN_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_HEADER SPIXFC_HEADER
 * @brief    Simple Header
 * @{
 */
#define MXC_F_SPIXFC_HEADER_TX_BIDIR_POS               0 /**< HEADER_TX_BIDIR Position */
#define MXC_F_SPIXFC_HEADER_TX_BIDIR                   ((uint32_t)(0x3FFFUL << MXC_F_SPIXFC_HEADER_TX_BIDIR_POS)) /**< HEADER_TX_BIDIR Mask */

#define MXC_F_SPIXFC_HEADER_RX_ONLY_POS                16 /**< HEADER_RX_ONLY Position */
#define MXC_F_SPIXFC_HEADER_RX_ONLY                    ((uint32_t)(0x3FFFUL << MXC_F_SPIXFC_HEADER_RX_ONLY_POS)) /**< HEADER_RX_ONLY Mask */

/**@} end of group SPIXFC_HEADER_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_SPIXFC_REGS_H_
