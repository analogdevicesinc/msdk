/**
 * @file    spixr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXR Peripheral Module.
 * @note    This file is @generated.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All rights Reserved.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_

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
 * @ingroup     spixr
 * @defgroup    spixr_registers SPIXR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXR Peripheral Module.
 * @details     SPIXR peripheral.
 */

/**
 * @ingroup spixr_registers
 * Structure type to access the SPIXR Registers.
 */
typedef struct {
    union {
        __IO uint32_t fifo32;           /**< <tt>\b 0x00:</tt> SPIXR FIFO32 Register */
        __IO uint16_t fifo16[2];        /**< <tt>\b 0x00:</tt> SPIXR FIFO16 Register */
        __IO uint8_t  fifo8[4];         /**< <tt>\b 0x00:</tt> SPIXR FIFO8 Register */
    };
    __IO uint32_t ctrl0;                /**< <tt>\b 0x04:</tt> SPIXR CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x08:</tt> SPIXR CTRL1 Register */
    __IO uint32_t ctrl2;                /**< <tt>\b 0x0C:</tt> SPIXR CTRL2 Register */
    __IO uint32_t sstime;               /**< <tt>\b 0x10:</tt> SPIXR SSTIME Register */
    __IO uint32_t brgctrl;              /**< <tt>\b 0x14:</tt> SPIXR BRGCTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t dma;                  /**< <tt>\b 0x1C:</tt> SPIXR DMA Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x20:</tt> SPIXR INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x24:</tt> SPIXR INTEN Register */
    __IO uint32_t wkfl;                 /**< <tt>\b 0x28:</tt> SPIXR WKFL Register */
    __IO uint32_t wken;                 /**< <tt>\b 0x2C:</tt> SPIXR WKEN Register */
    __I  uint32_t status;               /**< <tt>\b 0x30:</tt> SPIXR STATUS Register */
    __IO uint32_t xmemctrl;             /**< <tt>\b 0x34:</tt> SPIXR XMEMCTRL Register */
} mxc_spixr_regs_t;

/* Register offsets for module SPIXR */
/**
 * @ingroup    spixr_registers
 * @defgroup   SPIXR_Register_Offsets Register Offsets
 * @brief      SPIXR Peripheral Register Offsets from the SPIXR Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXR_FIFO32                 ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_FIFO16                 ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_FIFO8                  ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_CTRL0                  ((uint32_t)0x00000004UL) /**< Offset from SPIXR Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXR_CTRL1                  ((uint32_t)0x00000008UL) /**< Offset from SPIXR Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXR_CTRL2                  ((uint32_t)0x0000000CUL) /**< Offset from SPIXR Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXR_SSTIME                 ((uint32_t)0x00000010UL) /**< Offset from SPIXR Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXR_BRGCTRL                ((uint32_t)0x00000014UL) /**< Offset from SPIXR Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXR_DMA                    ((uint32_t)0x0000001CUL) /**< Offset from SPIXR Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXR_INTFL                  ((uint32_t)0x00000020UL) /**< Offset from SPIXR Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXR_INTEN                  ((uint32_t)0x00000024UL) /**< Offset from SPIXR Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPIXR_WKFL                   ((uint32_t)0x00000028UL) /**< Offset from SPIXR Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPIXR_WKEN                   ((uint32_t)0x0000002CUL) /**< Offset from SPIXR Base Address: <tt> 0x002C</tt> */
#define MXC_R_SPIXR_STATUS                 ((uint32_t)0x00000030UL) /**< Offset from SPIXR Base Address: <tt> 0x0030</tt> */
#define MXC_R_SPIXR_XMEMCTRL               ((uint32_t)0x00000034UL) /**< Offset from SPIXR Base Address: <tt> 0x0034</tt> */
/**@} end of group spixr_registers */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_FIFO32 SPIXR_FIFO32
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_FIFO32_DATA_POS                    0 /**< FIFO32_DATA Position */
#define MXC_F_SPIXR_FIFO32_DATA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPIXR_FIFO32_DATA_POS)) /**< FIFO32_DATA Mask */

/**@} end of group SPIXR_FIFO32_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_FIFO16 SPIXR_FIFO16
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_FIFO16_DATA_POS                    0 /**< FIFO16_DATA Position */
#define MXC_F_SPIXR_FIFO16_DATA                        ((uint16_t)(0xFFFFUL << MXC_F_SPIXR_FIFO16_DATA_POS)) /**< FIFO16_DATA Mask */

/**@} end of group SPIXR_FIFO16_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_FIFO8 SPIXR_FIFO8
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_FIFO8_DATA_POS                     0 /**< FIFO8_DATA Position */
#define MXC_F_SPIXR_FIFO8_DATA                         ((uint8_t)(0xFFUL << MXC_F_SPIXR_FIFO8_DATA_POS)) /**< FIFO8_DATA Mask */

/**@} end of group SPIXR_FIFO8_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL0 SPIXR_CTRL0
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL0_EN_POS                       0 /**< CTRL0_EN Position */
#define MXC_F_SPIXR_CTRL0_EN                           ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_EN_POS)) /**< CTRL0_EN Mask */

#define MXC_F_SPIXR_CTRL0_MST_MODE_POS                 1 /**< CTRL0_MST_MODE Position */
#define MXC_F_SPIXR_CTRL0_MST_MODE                     ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_MST_MODE_POS)) /**< CTRL0_MST_MODE Mask */

#define MXC_F_SPIXR_CTRL0_SSIO_POS                     4 /**< CTRL0_SSIO Position */
#define MXC_F_SPIXR_CTRL0_SSIO                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_SSIO_POS)) /**< CTRL0_SSIO Mask */

#define MXC_F_SPIXR_CTRL0_TX_START_POS                 5 /**< CTRL0_TX_START Position */
#define MXC_F_SPIXR_CTRL0_TX_START                     ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_TX_START_POS)) /**< CTRL0_TX_START Mask */

#define MXC_F_SPIXR_CTRL0_SS_CTRL_POS                  8 /**< CTRL0_SS_CTRL Position */
#define MXC_F_SPIXR_CTRL0_SS_CTRL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_SS_CTRL_POS)) /**< CTRL0_SS_CTRL Mask */

#define MXC_F_SPIXR_CTRL0_SS_POS                       16 /**< CTRL0_SS Position */
#define MXC_F_SPIXR_CTRL0_SS                           ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL0_SS_POS)) /**< CTRL0_SS Mask */
#define MXC_V_SPIXR_CTRL0_SS_SS0                       ((uint32_t)0x1UL) /**< CTRL0_SS_SS0 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS0                       (MXC_V_SPIXR_CTRL0_SS_SS0 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS0 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS1                       ((uint32_t)0x2UL) /**< CTRL0_SS_SS1 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS1                       (MXC_V_SPIXR_CTRL0_SS_SS1 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS1 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS2                       ((uint32_t)0x4UL) /**< CTRL0_SS_SS2 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS2                       (MXC_V_SPIXR_CTRL0_SS_SS2 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS2 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS3                       ((uint32_t)0x8UL) /**< CTRL0_SS_SS3 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS3                       (MXC_V_SPIXR_CTRL0_SS_SS3 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS3 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS4                       ((uint32_t)0x10UL) /**< CTRL0_SS_SS4 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS4                       (MXC_V_SPIXR_CTRL0_SS_SS4 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS4 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS5                       ((uint32_t)0x20UL) /**< CTRL0_SS_SS5 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS5                       (MXC_V_SPIXR_CTRL0_SS_SS5 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS5 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS6                       ((uint32_t)0x40UL) /**< CTRL0_SS_SS6 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS6                       (MXC_V_SPIXR_CTRL0_SS_SS6 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS6 Setting */
#define MXC_V_SPIXR_CTRL0_SS_SS7                       ((uint32_t)0x80UL) /**< CTRL0_SS_SS7 Value */
#define MXC_S_SPIXR_CTRL0_SS_SS7                       (MXC_V_SPIXR_CTRL0_SS_SS7 << MXC_F_SPIXR_CTRL0_SS_POS) /**< CTRL0_SS_SS7 Setting */

/**@} end of group SPIXR_CTRL0_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL1 SPIXR_CTRL1
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL1_TX_NUM_CHAR_POS              0 /**< CTRL1_TX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL1_TX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL1_TX_NUM_CHAR_POS)) /**< CTRL1_TX_NUM_CHAR Mask */

#define MXC_F_SPIXR_CTRL1_RX_NUM_CHAR_POS              16 /**< CTRL1_RX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL1_RX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL1_RX_NUM_CHAR_POS)) /**< CTRL1_RX_NUM_CHAR Mask */

/**@} end of group SPIXR_CTRL1_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL2 SPIXR_CTRL2
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL2_CLKPHA_POS                   0 /**< CTRL2_CLKPHA Position */
#define MXC_F_SPIXR_CTRL2_CLKPHA                       ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_CLKPHA_POS)) /**< CTRL2_CLKPHA Mask */

#define MXC_F_SPIXR_CTRL2_CLKPOL_POS                   1 /**< CTRL2_CLKPOL Position */
#define MXC_F_SPIXR_CTRL2_CLKPOL                       ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_CLKPOL_POS)) /**< CTRL2_CLKPOL Mask */

#define MXC_F_SPIXR_CTRL2_SCLK_FB_INV_POS              4 /**< CTRL2_SCLK_FB_INV Position */
#define MXC_F_SPIXR_CTRL2_SCLK_FB_INV                  ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_SCLK_FB_INV_POS)) /**< CTRL2_SCLK_FB_INV Mask */

#define MXC_F_SPIXR_CTRL2_NUMBITS_POS                  8 /**< CTRL2_NUMBITS Position */
#define MXC_F_SPIXR_CTRL2_NUMBITS                      ((uint32_t)(0xFUL << MXC_F_SPIXR_CTRL2_NUMBITS_POS)) /**< CTRL2_NUMBITS Mask */
#define MXC_V_SPIXR_CTRL2_NUMBITS_0                    ((uint32_t)0x0UL) /**< CTRL2_NUMBITS_0 Value */
#define MXC_S_SPIXR_CTRL2_NUMBITS_0                    (MXC_V_SPIXR_CTRL2_NUMBITS_0 << MXC_F_SPIXR_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_0 Setting */

#define MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS               12 /**< CTRL2_DATA_WIDTH Position */
#define MXC_F_SPIXR_CTRL2_DATA_WIDTH                   ((uint32_t)(0x3UL << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS)) /**< CTRL2_DATA_WIDTH Mask */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_MONO              ((uint32_t)0x0UL) /**< CTRL2_DATA_WIDTH_MONO Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_MONO              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_MONO << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_MONO Setting */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_DUAL              ((uint32_t)0x1UL) /**< CTRL2_DATA_WIDTH_DUAL Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_DUAL              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_DUAL << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_DUAL Setting */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_QUAD              ((uint32_t)0x2UL) /**< CTRL2_DATA_WIDTH_QUAD Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_QUAD              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_QUAD << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_QUAD Setting */

#define MXC_F_SPIXR_CTRL2_THREE_WIRE_POS               15 /**< CTRL2_THREE_WIRE Position */
#define MXC_F_SPIXR_CTRL2_THREE_WIRE                   ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_THREE_WIRE_POS)) /**< CTRL2_THREE_WIRE Mask */

#define MXC_F_SPIXR_CTRL2_SSPOL_POS                    16 /**< CTRL2_SSPOL Position */
#define MXC_F_SPIXR_CTRL2_SSPOL                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL2_SSPOL_POS)) /**< CTRL2_SSPOL Mask */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS0_HIGH               ((uint32_t)0x1UL) /**< CTRL2_SSPOL_SS0_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS0_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS0_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS0_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS1_HIGH               ((uint32_t)0x2UL) /**< CTRL2_SSPOL_SS1_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS1_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS1_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS1_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS2_HIGH               ((uint32_t)0x4UL) /**< CTRL2_SSPOL_SS2_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS2_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS2_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS2_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS3_HIGH               ((uint32_t)0x8UL) /**< CTRL2_SSPOL_SS3_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS3_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS3_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS3_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS4_HIGH               ((uint32_t)0x10UL) /**< CTRL2_SSPOL_SS4_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS4_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS4_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS4_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS5_HIGH               ((uint32_t)0x20UL) /**< CTRL2_SSPOL_SS5_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS5_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS5_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS5_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS6_HIGH               ((uint32_t)0x40UL) /**< CTRL2_SSPOL_SS6_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS6_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS6_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS6_HIGH Setting */
#define MXC_V_SPIXR_CTRL2_SSPOL_SS7_HIGH               ((uint32_t)0x80UL) /**< CTRL2_SSPOL_SS7_HIGH Value */
#define MXC_S_SPIXR_CTRL2_SSPOL_SS7_HIGH               (MXC_V_SPIXR_CTRL2_SSPOL_SS7_HIGH << MXC_F_SPIXR_CTRL2_SSPOL_POS) /**< CTRL2_SSPOL_SS7_HIGH Setting */

/**@} end of group SPIXR_CTRL2_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_SSTIME SPIXR_SSTIME
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_SSTIME_PRE_POS                     0 /**< SSTIME_PRE Position */
#define MXC_F_SPIXR_SSTIME_PRE                         ((uint32_t)(0xFFUL << MXC_F_SPIXR_SSTIME_PRE_POS)) /**< SSTIME_PRE Mask */
#define MXC_V_SPIXR_SSTIME_PRE_256                     ((uint32_t)0x0UL) /**< SSTIME_PRE_256 Value */
#define MXC_S_SPIXR_SSTIME_PRE_256                     (MXC_V_SPIXR_SSTIME_PRE_256 << MXC_F_SPIXR_SSTIME_PRE_POS) /**< SSTIME_PRE_256 Setting */

#define MXC_F_SPIXR_SSTIME_POST_POS                    8 /**< SSTIME_POST Position */
#define MXC_F_SPIXR_SSTIME_POST                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_SSTIME_POST_POS)) /**< SSTIME_POST Mask */
#define MXC_V_SPIXR_SSTIME_POST_256                    ((uint32_t)0x0UL) /**< SSTIME_POST_256 Value */
#define MXC_S_SPIXR_SSTIME_POST_256                    (MXC_V_SPIXR_SSTIME_POST_256 << MXC_F_SPIXR_SSTIME_POST_POS) /**< SSTIME_POST_256 Setting */

#define MXC_F_SPIXR_SSTIME_INACT_POS                   16 /**< SSTIME_INACT Position */
#define MXC_F_SPIXR_SSTIME_INACT                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_SSTIME_INACT_POS)) /**< SSTIME_INACT Mask */
#define MXC_V_SPIXR_SSTIME_INACT_256                   ((uint32_t)0x0UL) /**< SSTIME_INACT_256 Value */
#define MXC_S_SPIXR_SSTIME_INACT_256                   (MXC_V_SPIXR_SSTIME_INACT_256 << MXC_F_SPIXR_SSTIME_INACT_POS) /**< SSTIME_INACT_256 Setting */

/**@} end of group SPIXR_SSTIME_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_BRGCTRL SPIXR_BRGCTRL
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPIXR_BRGCTRL_LO_POS                     0 /**< BRGCTRL_LO Position */
#define MXC_F_SPIXR_BRGCTRL_LO                         ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRGCTRL_LO_POS)) /**< BRGCTRL_LO Mask */
#define MXC_V_SPIXR_BRGCTRL_LO_DIS                     ((uint32_t)0x0UL) /**< BRGCTRL_LO_DIS Value */
#define MXC_S_SPIXR_BRGCTRL_LO_DIS                     (MXC_V_SPIXR_BRGCTRL_LO_DIS << MXC_F_SPIXR_BRGCTRL_LO_POS) /**< BRGCTRL_LO_DIS Setting */

#define MXC_F_SPIXR_BRGCTRL_HI_POS                     8 /**< BRGCTRL_HI Position */
#define MXC_F_SPIXR_BRGCTRL_HI                         ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRGCTRL_HI_POS)) /**< BRGCTRL_HI Mask */
#define MXC_V_SPIXR_BRGCTRL_HI_DIS                     ((uint32_t)0x0UL) /**< BRGCTRL_HI_DIS Value */
#define MXC_S_SPIXR_BRGCTRL_HI_DIS                     (MXC_V_SPIXR_BRGCTRL_HI_DIS << MXC_F_SPIXR_BRGCTRL_HI_POS) /**< BRGCTRL_HI_DIS Setting */

#define MXC_F_SPIXR_BRGCTRL_CLKDIV_POS                 16 /**< BRGCTRL_CLKDIV Position */
#define MXC_F_SPIXR_BRGCTRL_CLKDIV                     ((uint32_t)(0xFUL << MXC_F_SPIXR_BRGCTRL_CLKDIV_POS)) /**< BRGCTRL_CLKDIV Mask */

/**@} end of group SPIXR_BRGCTRL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DMA SPIXR_DMA
 * @brief    Register for controlling DMA.
 * @{
 */
#define MXC_F_SPIXR_DMA_TX_THD_VAL_POS                 0 /**< DMA_TX_THD_VAL Position */
#define MXC_F_SPIXR_DMA_TX_THD_VAL                     ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_TX_THD_VAL_POS)) /**< DMA_TX_THD_VAL Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_EN_POS                 6 /**< DMA_TX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_EN_POS)) /**< DMA_TX_FIFO_EN Mask */

#define MXC_F_SPIXR_DMA_TX_FLUSH_POS                   7 /**< DMA_TX_FLUSH Position */
#define MXC_F_SPIXR_DMA_TX_FLUSH                       ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FLUSH_POS)) /**< DMA_TX_FLUSH Mask */

#define MXC_F_SPIXR_DMA_TX_LVL_POS                     8 /**< DMA_TX_LVL Position */
#define MXC_F_SPIXR_DMA_TX_LVL                         ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_TX_LVL_POS)) /**< DMA_TX_LVL Mask */

#define MXC_F_SPIXR_DMA_TX_EN_POS                      15 /**< DMA_TX_EN Position */
#define MXC_F_SPIXR_DMA_TX_EN                          ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_EN_POS)) /**< DMA_TX_EN Mask */

#define MXC_F_SPIXR_DMA_RX_THD_LVL_POS                 16 /**< DMA_RX_THD_LVL Position */
#define MXC_F_SPIXR_DMA_RX_THD_LVL                     ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_THD_LVL_POS)) /**< DMA_RX_THD_LVL Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_EN_POS                 22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */

#define MXC_F_SPIXR_DMA_RX_FLUSH_POS                   23 /**< DMA_RX_FLUSH Position */
#define MXC_F_SPIXR_DMA_RX_FLUSH                       ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FLUSH_POS)) /**< DMA_RX_FLUSH Mask */

#define MXC_F_SPIXR_DMA_RX_LVL_POS                     24 /**< DMA_RX_LVL Position */
#define MXC_F_SPIXR_DMA_RX_LVL                         ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_LVL_POS)) /**< DMA_RX_LVL Mask */

#define MXC_F_SPIXR_DMA_RX_EN_POS                      31 /**< DMA_RX_EN Position */
#define MXC_F_SPIXR_DMA_RX_EN                          ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_EN_POS)) /**< DMA_RX_EN Mask */

/**@} end of group SPIXR_DMA_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INTFL SPIXR_INTFL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPIXR_INTFL_TX_THD_POS                   0 /**< INTFL_TX_THD Position */
#define MXC_F_SPIXR_INTFL_TX_THD                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_THD_POS)) /**< INTFL_TX_THD Mask */

#define MXC_F_SPIXR_INTFL_TX_EM_POS                    1 /**< INTFL_TX_EM Position */
#define MXC_F_SPIXR_INTFL_TX_EM                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_EM_POS)) /**< INTFL_TX_EM Mask */

#define MXC_F_SPIXR_INTFL_RX_THD_POS                   2 /**< INTFL_RX_THD Position */
#define MXC_F_SPIXR_INTFL_RX_THD                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_THD_POS)) /**< INTFL_RX_THD Mask */

#define MXC_F_SPIXR_INTFL_RX_FULL_POS                  3 /**< INTFL_RX_FULL Position */
#define MXC_F_SPIXR_INTFL_RX_FULL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_FULL_POS)) /**< INTFL_RX_FULL Mask */

#define MXC_F_SPIXR_INTFL_SSA_POS                      4 /**< INTFL_SSA Position */
#define MXC_F_SPIXR_INTFL_SSA                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_SSA_POS)) /**< INTFL_SSA Mask */

#define MXC_F_SPIXR_INTFL_SSD_POS                      5 /**< INTFL_SSD Position */
#define MXC_F_SPIXR_INTFL_SSD                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_SSD_POS)) /**< INTFL_SSD Mask */

#define MXC_F_SPIXR_INTFL_FAULT_POS                    8 /**< INTFL_FAULT Position */
#define MXC_F_SPIXR_INTFL_FAULT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_FAULT_POS)) /**< INTFL_FAULT Mask */

#define MXC_F_SPIXR_INTFL_ABORT_POS                    9 /**< INTFL_ABORT Position */
#define MXC_F_SPIXR_INTFL_ABORT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_ABORT_POS)) /**< INTFL_ABORT Mask */

#define MXC_F_SPIXR_INTFL_MST_DONE_POS                 11 /**< INTFL_MST_DONE Position */
#define MXC_F_SPIXR_INTFL_MST_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_MST_DONE_POS)) /**< INTFL_MST_DONE Mask */

#define MXC_F_SPIXR_INTFL_TX_OV_POS                    12 /**< INTFL_TX_OV Position */
#define MXC_F_SPIXR_INTFL_TX_OV                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_OV_POS)) /**< INTFL_TX_OV Mask */

#define MXC_F_SPIXR_INTFL_TX_UN_POS                    13 /**< INTFL_TX_UN Position */
#define MXC_F_SPIXR_INTFL_TX_UN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_UN_POS)) /**< INTFL_TX_UN Mask */

#define MXC_F_SPIXR_INTFL_RX_OV_POS                    14 /**< INTFL_RX_OV Position */
#define MXC_F_SPIXR_INTFL_RX_OV                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_OV_POS)) /**< INTFL_RX_OV Mask */

#define MXC_F_SPIXR_INTFL_RX_UN_POS                    15 /**< INTFL_RX_UN Position */
#define MXC_F_SPIXR_INTFL_RX_UN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_UN_POS)) /**< INTFL_RX_UN Mask */

/**@} end of group SPIXR_INTFL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INTEN SPIXR_INTEN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPIXR_INTEN_TX_THD_POS                   0 /**< INTEN_TX_THD Position */
#define MXC_F_SPIXR_INTEN_TX_THD                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_THD_POS)) /**< INTEN_TX_THD Mask */

#define MXC_F_SPIXR_INTEN_TX_EM_POS                    1 /**< INTEN_TX_EM Position */
#define MXC_F_SPIXR_INTEN_TX_EM                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_EM_POS)) /**< INTEN_TX_EM Mask */

#define MXC_F_SPIXR_INTEN_RX_THD_POS                   2 /**< INTEN_RX_THD Position */
#define MXC_F_SPIXR_INTEN_RX_THD                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_THD_POS)) /**< INTEN_RX_THD Mask */

#define MXC_F_SPIXR_INTEN_RX_FULL_POS                  3 /**< INTEN_RX_FULL Position */
#define MXC_F_SPIXR_INTEN_RX_FULL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_FULL_POS)) /**< INTEN_RX_FULL Mask */

#define MXC_F_SPIXR_INTEN_SSA_POS                      4 /**< INTEN_SSA Position */
#define MXC_F_SPIXR_INTEN_SSA                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_SSA_POS)) /**< INTEN_SSA Mask */

#define MXC_F_SPIXR_INTEN_SSD_POS                      5 /**< INTEN_SSD Position */
#define MXC_F_SPIXR_INTEN_SSD                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_SSD_POS)) /**< INTEN_SSD Mask */

#define MXC_F_SPIXR_INTEN_FAULT_POS                    8 /**< INTEN_FAULT Position */
#define MXC_F_SPIXR_INTEN_FAULT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_FAULT_POS)) /**< INTEN_FAULT Mask */

#define MXC_F_SPIXR_INTEN_ABORT_POS                    9 /**< INTEN_ABORT Position */
#define MXC_F_SPIXR_INTEN_ABORT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_ABORT_POS)) /**< INTEN_ABORT Mask */

#define MXC_F_SPIXR_INTEN_MST_DONE_POS                 11 /**< INTEN_MST_DONE Position */
#define MXC_F_SPIXR_INTEN_MST_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_MST_DONE_POS)) /**< INTEN_MST_DONE Mask */

#define MXC_F_SPIXR_INTEN_TX_OV_POS                    12 /**< INTEN_TX_OV Position */
#define MXC_F_SPIXR_INTEN_TX_OV                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_OV_POS)) /**< INTEN_TX_OV Mask */

#define MXC_F_SPIXR_INTEN_TX_UN_POS                    13 /**< INTEN_TX_UN Position */
#define MXC_F_SPIXR_INTEN_TX_UN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_UN_POS)) /**< INTEN_TX_UN Mask */

#define MXC_F_SPIXR_INTEN_RX_OV_POS                    14 /**< INTEN_RX_OV Position */
#define MXC_F_SPIXR_INTEN_RX_OV                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_OV_POS)) /**< INTEN_RX_OV Mask */

#define MXC_F_SPIXR_INTEN_RX_UN_POS                    15 /**< INTEN_RX_UN Position */
#define MXC_F_SPIXR_INTEN_RX_UN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_UN_POS)) /**< INTEN_RX_UN Mask */

/**@} end of group SPIXR_INTEN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WKFL SPIXR_WKFL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPIXR_WKFL_TX_THD_POS                    0 /**< WKFL_TX_THD Position */
#define MXC_F_SPIXR_WKFL_TX_THD                        ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_TX_THD_POS)) /**< WKFL_TX_THD Mask */

#define MXC_F_SPIXR_WKFL_TX_EM_POS                     1 /**< WKFL_TX_EM Position */
#define MXC_F_SPIXR_WKFL_TX_EM                         ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_TX_EM_POS)) /**< WKFL_TX_EM Mask */

#define MXC_F_SPIXR_WKFL_RX_THD_POS                    2 /**< WKFL_RX_THD Position */
#define MXC_F_SPIXR_WKFL_RX_THD                        ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_RX_THD_POS)) /**< WKFL_RX_THD Mask */

#define MXC_F_SPIXR_WKFL_RX_FULL_POS                   3 /**< WKFL_RX_FULL Position */
#define MXC_F_SPIXR_WKFL_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_RX_FULL_POS)) /**< WKFL_RX_FULL Mask */

/**@} end of group SPIXR_WKFL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WKEN SPIXR_WKEN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPIXR_WKEN_TX_THD_POS                    0 /**< WKEN_TX_THD Position */
#define MXC_F_SPIXR_WKEN_TX_THD                        ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_TX_THD_POS)) /**< WKEN_TX_THD Mask */

#define MXC_F_SPIXR_WKEN_TX_EM_POS                     1 /**< WKEN_TX_EM Position */
#define MXC_F_SPIXR_WKEN_TX_EM                         ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_TX_EM_POS)) /**< WKEN_TX_EM Mask */

#define MXC_F_SPIXR_WKEN_RX_THD_POS                    2 /**< WKEN_RX_THD Position */
#define MXC_F_SPIXR_WKEN_RX_THD                        ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_RX_THD_POS)) /**< WKEN_RX_THD Mask */

#define MXC_F_SPIXR_WKEN_RX_FULL_POS                   3 /**< WKEN_RX_FULL Position */
#define MXC_F_SPIXR_WKEN_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_RX_FULL_POS)) /**< WKEN_RX_FULL Mask */

/**@} end of group SPIXR_WKEN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_STATUS SPIXR_STATUS
 * @brief    SPI Status register.
 * @{
 */
#define MXC_F_SPIXR_STATUS_BUSY_POS                    0 /**< STATUS_BUSY Position */
#define MXC_F_SPIXR_STATUS_BUSY                        ((uint32_t)(0x1UL << MXC_F_SPIXR_STATUS_BUSY_POS)) /**< STATUS_BUSY Mask */

/**@} end of group SPIXR_STATUS_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_XMEMCTRL SPIXR_XMEMCTRL
 * @brief    Register to control external memory.
 * @{
 */
#define MXC_F_SPIXR_XMEMCTRL_RD_CMD_POS                0 /**< XMEMCTRL_RD_CMD Position */
#define MXC_F_SPIXR_XMEMCTRL_RD_CMD                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_RD_CMD_POS)) /**< XMEMCTRL_RD_CMD Mask */

#define MXC_F_SPIXR_XMEMCTRL_WR_CMD_POS                8 /**< XMEMCTRL_WR_CMD Position */
#define MXC_F_SPIXR_XMEMCTRL_WR_CMD                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_WR_CMD_POS)) /**< XMEMCTRL_WR_CMD Mask */

#define MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK_POS             16 /**< XMEMCTRL_DUMMY_CLK Position */
#define MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK                 ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK_POS)) /**< XMEMCTRL_DUMMY_CLK Mask */

#define MXC_F_SPIXR_XMEMCTRL_EN_POS                    31 /**< XMEMCTRL_EN Position */
#define MXC_F_SPIXR_XMEMCTRL_EN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_XMEMCTRL_EN_POS)) /**< XMEMCTRL_EN Mask */

/**@} end of group SPIXR_XMEMCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_
