/**
 * @file    usbhs_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the USBHS Peripheral Module.
 * @note    This file is @generated.
 * @ingroup usbhs_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_USBHS_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_USBHS_REGS_H_

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
 * @ingroup     usbhs
 * @defgroup    usbhs_registers USBHS_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the USBHS Peripheral Module.
 * @details     USB 2.0 High-speed Controller.
 */

/**
 * @ingroup usbhs_registers
 * Structure type to access the USBHS Registers.
 */
typedef struct {
    __IO uint8_t  faddr;                /**< <tt>\b 0x00:</tt> USBHS FADDR Register */
    __IO uint8_t  power;                /**< <tt>\b 0x01:</tt> USBHS POWER Register */
    __IO uint16_t intrin;               /**< <tt>\b 0x02:</tt> USBHS INTRIN Register */
    __IO uint16_t introut;              /**< <tt>\b 0x04:</tt> USBHS INTROUT Register */
    __IO uint16_t intrinen;             /**< <tt>\b 0x06:</tt> USBHS INTRINEN Register */
    __IO uint16_t introuten;            /**< <tt>\b 0x08:</tt> USBHS INTROUTEN Register */
    __IO uint8_t  intrusb;              /**< <tt>\b 0x0A:</tt> USBHS INTRUSB Register */
    __IO uint8_t  intrusben;            /**< <tt>\b 0x0B:</tt> USBHS INTRUSBEN Register */
    __IO uint16_t frame;                /**< <tt>\b 0x0C:</tt> USBHS FRAME Register */
    __IO uint8_t  index;                /**< <tt>\b 0x0E:</tt> USBHS INDEX Register */
    __IO uint8_t  testmode;             /**< <tt>\b 0x0F:</tt> USBHS TESTMODE Register */
    __IO uint16_t inmaxp;               /**< <tt>\b 0x10:</tt> USBHS INMAXP Register */
    union {
        __IO uint8_t  csr0;             /**< <tt>\b 0x12:</tt> USBHS CSR0 Register */
        __IO uint8_t  incsrl;           /**< <tt>\b 0x12:</tt> USBHS INCSRL Register */
    };
    __IO uint8_t  incsru;               /**< <tt>\b 0x13:</tt> USBHS INCSRU Register */
    __IO uint16_t outmaxp;              /**< <tt>\b 0x14:</tt> USBHS OUTMAXP Register */
    __IO uint8_t  outcsrl;              /**< <tt>\b 0x16:</tt> USBHS OUTCSRL Register */
    __IO uint8_t  outcsru;              /**< <tt>\b 0x17:</tt> USBHS OUTCSRU Register */
    union {
        __IO uint16_t count0;           /**< <tt>\b 0x18:</tt> USBHS COUNT0 Register */
        __IO uint16_t outcount;         /**< <tt>\b 0x18:</tt> USBHS OUTCOUNT Register */
    };
    __R  uint16_t rsv_0x1a_0x1f[3];
    __IO uint32_t fifo0;                /**< <tt>\b 0x20:</tt> USBHS FIFO0 Register */
    __IO uint32_t fifo1;                /**< <tt>\b 0x24:</tt> USBHS FIFO1 Register */
    __IO uint32_t fifo2;                /**< <tt>\b 0x28:</tt> USBHS FIFO2 Register */
    __IO uint32_t fifo3;                /**< <tt>\b 0x2c:</tt> USBHS FIFO3 Register */
    __IO uint32_t fifo4;                /**< <tt>\b 0x30:</tt> USBHS FIFO4 Register */
    __IO uint32_t fifo5;                /**< <tt>\b 0x34:</tt> USBHS FIFO5 Register */
    __IO uint32_t fifo6;                /**< <tt>\b 0x38:</tt> USBHS FIFO6 Register */
    __IO uint32_t fifo7;                /**< <tt>\b 0x3c:</tt> USBHS FIFO7 Register */
    __IO uint32_t fifo8;                /**< <tt>\b 0x40:</tt> USBHS FIFO8 Register */
    __IO uint32_t fifo9;                /**< <tt>\b 0x44:</tt> USBHS FIFO9 Register */
    __IO uint32_t fifo10;               /**< <tt>\b 0x48:</tt> USBHS FIFO10 Register */
    __IO uint32_t fifo11;               /**< <tt>\b 0x4c:</tt> USBHS FIFO11 Register */
    __IO uint32_t fifo12;               /**< <tt>\b 0x50:</tt> USBHS FIFO12 Register */
    __IO uint32_t fifo13;               /**< <tt>\b 0x54:</tt> USBHS FIFO13 Register */
    __IO uint32_t fifo14;               /**< <tt>\b 0x58:</tt> USBHS FIFO14 Register */
    __IO uint32_t fifo15;               /**< <tt>\b 0x5c:</tt> USBHS FIFO15 Register */
    __R  uint32_t rsv_0x60_0x6b[3];
    __IO uint16_t hwvers;               /**< <tt>\b 0x6c:</tt> USBHS HWVERS Register */
    __R  uint16_t rsv_0x6e_0x77[5];
    __IO uint8_t  epinfo;               /**< <tt>\b 0x78:</tt> USBHS EPINFO Register */
    __IO uint8_t  raminfo;              /**< <tt>\b 0x79:</tt> USBHS RAMINFO Register */
    __IO uint8_t  softreset;            /**< <tt>\b 0x7A:</tt> USBHS SOFTRESET Register */
    __R  uint8_t  rsv_0x7b_0x7f[5];
    __IO uint16_t ctuch;                /**< <tt>\b 0x80:</tt> USBHS CTUCH Register */
    __IO uint16_t cthsrtn;              /**< <tt>\b 0x82:</tt> USBHS CTHSRTN Register */
    __R  uint32_t rsv_0x84_0x3ff[223];
    __IO uint32_t mxm_usb_reg_00;       /**< <tt>\b 0x400:</tt> USBHS MXM_USB_REG_00 Register */
    __IO uint32_t m31_phy_utmi_reset;   /**< <tt>\b 0x404:</tt> USBHS M31_PHY_UTMI_RESET Register */
    __IO uint32_t m31_phy_utmi_vcontrol; /**< <tt>\b 0x408:</tt> USBHS M31_PHY_UTMI_VCONTROL Register */
    __IO uint32_t m31_phy_clk_en;       /**< <tt>\b 0x40C:</tt> USBHS M31_PHY_CLK_EN Register */
    __IO uint32_t m31_phy_ponrst;       /**< <tt>\b 0x410:</tt> USBHS M31_PHY_PONRST Register */
    __IO uint32_t m31_phy_noncry_rstb;  /**< <tt>\b 0x414:</tt> USBHS M31_PHY_NONCRY_RSTB Register */
    __IO uint32_t m31_phy_noncry_en;    /**< <tt>\b 0x418:</tt> USBHS M31_PHY_NONCRY_EN Register */
    __R  uint32_t rsv_0x41c;
    __IO uint32_t m31_phy_u2_compliance_en; /**< <tt>\b 0x420:</tt> USBHS M31_PHY_U2_COMPLIANCE_EN Register */
    __IO uint32_t m31_phy_u2_compliance_dac_adj; /**< <tt>\b 0x424:</tt> USBHS M31_PHY_U2_COMPLIANCE_DAC_ADJ Register */
    __IO uint32_t m31_phy_u2_compliance_dac_adj_en; /**< <tt>\b 0x428:</tt> USBHS M31_PHY_U2_COMPLIANCE_DAC_ADJ_EN Register */
    __IO uint32_t m31_phy_clk_rdy;      /**< <tt>\b 0x42C:</tt> USBHS M31_PHY_CLK_RDY Register */
    __IO uint32_t m31_phy_pll_en;       /**< <tt>\b 0x430:</tt> USBHS M31_PHY_PLL_EN Register */
    __IO uint32_t m31_phy_bist_ok;      /**< <tt>\b 0x434:</tt> USBHS M31_PHY_BIST_OK Register */
    __IO uint32_t m31_phy_data_oe;      /**< <tt>\b 0x438:</tt> USBHS M31_PHY_DATA_OE Register */
    __IO uint32_t m31_phy_oscouten;     /**< <tt>\b 0x43C:</tt> USBHS M31_PHY_OSCOUTEN Register */
    __IO uint32_t m31_phy_lpm_alive;    /**< <tt>\b 0x440:</tt> USBHS M31_PHY_LPM_ALIVE Register */
    __IO uint32_t m31_phy_hs_bist_mode; /**< <tt>\b 0x444:</tt> USBHS M31_PHY_HS_BIST_MODE Register */
    __IO uint32_t m31_phy_coreclkin;    /**< <tt>\b 0x448:</tt> USBHS M31_PHY_CORECLKIN Register */
    __IO uint32_t m31_phy_xtlsel;       /**< <tt>\b 0x44C:</tt> USBHS M31_PHY_XTLSEL Register */
    __IO uint32_t m31_phy_ls_en;        /**< <tt>\b 0x450:</tt> USBHS M31_PHY_LS_EN Register */
    __IO uint32_t m31_phy_debug_sel;    /**< <tt>\b 0x454:</tt> USBHS M31_PHY_DEBUG_SEL Register */
    __IO uint32_t m31_phy_debug_out;    /**< <tt>\b 0x458:</tt> USBHS M31_PHY_DEBUG_OUT Register */
    __IO uint32_t m31_phy_outclksel;    /**< <tt>\b 0x45C:</tt> USBHS M31_PHY_OUTCLKSEL Register */
    __IO uint32_t m31_phy_xcfgi_31_0;   /**< <tt>\b 0x460:</tt> USBHS M31_PHY_XCFGI_31_0 Register */
    __IO uint32_t m31_phy_xcfgi_63_32;  /**< <tt>\b 0x464:</tt> USBHS M31_PHY_XCFGI_63_32 Register */
    __IO uint32_t m31_phy_xcfgi_95_64;  /**< <tt>\b 0x468:</tt> USBHS M31_PHY_XCFGI_95_64 Register */
    __IO uint32_t m31_phy_xcfgi_127_96; /**< <tt>\b 0x46C:</tt> USBHS M31_PHY_XCFGI_127_96 Register */
    __IO uint32_t m31_phy_xcfgi_137_128; /**< <tt>\b 0x470:</tt> USBHS M31_PHY_XCFGI_137_128 Register */
    __IO uint32_t m31_phy_xcfg_hs_coarse_tune_num; /**< <tt>\b 0x474:</tt> USBHS M31_PHY_XCFG_HS_COARSE_TUNE_NUM Register */
    __IO uint32_t m31_phy_xcfg_hs_fine_tune_num; /**< <tt>\b 0x478:</tt> USBHS M31_PHY_XCFG_HS_FINE_TUNE_NUM Register */
    __IO uint32_t m31_phy_xcfg_fs_coarse_tune_num; /**< <tt>\b 0x47C:</tt> USBHS M31_PHY_XCFG_FS_COARSE_TUNE_NUM Register */
    __IO uint32_t m31_phy_xcfg_fs_fine_tune_num; /**< <tt>\b 0x480:</tt> USBHS M31_PHY_XCFG_FS_FINE_TUNE_NUM Register */
    __IO uint32_t m31_phy_xcfg_lock_range_max; /**< <tt>\b 0x484:</tt> USBHS M31_PHY_XCFG_LOCK_RANGE_MAX Register */
    __IO uint32_t m31_phy_xcfgi_lock_range_min; /**< <tt>\b 0x488:</tt> USBHS M31_PHY_XCFGI_LOCK_RANGE_MIN Register */
    __IO uint32_t m31_phy_xcfg_ob_rsel; /**< <tt>\b 0x48C:</tt> USBHS M31_PHY_XCFG_OB_RSEL Register */
    __IO uint32_t m31_phy_xcfg_oc_rsel; /**< <tt>\b 0x490:</tt> USBHS M31_PHY_XCFG_OC_RSEL Register */
    __IO uint32_t m31_phy_xcfgo;        /**< <tt>\b 0x494:</tt> USBHS M31_PHY_XCFGO Register */
    __IO uint32_t mxm_int;              /**< <tt>\b 0x498:</tt> USBHS MXM_INT Register */
    __IO uint32_t mxm_int_en;           /**< <tt>\b 0x49C:</tt> USBHS MXM_INT_EN Register */
    __IO uint32_t mxm_suspend;          /**< <tt>\b 0x4A0:</tt> USBHS MXM_SUSPEND Register */
    __IO uint32_t mxm_reg_a4;           /**< <tt>\b 0x4A4:</tt> USBHS MXM_REG_A4 Register */
} mxc_usbhs_regs_t;

/* Register offsets for module USBHS */
/**
 * @ingroup    usbhs_registers
 * @defgroup   USBHS_Register_Offsets Register Offsets
 * @brief      USBHS Peripheral Register Offsets from the USBHS Base Peripheral Address.
 * @{
 */
#define MXC_R_USBHS_FADDR                  ((uint32_t)0x00000000UL) /**< Offset from USBHS Base Address: <tt> 0x0000</tt> */
#define MXC_R_USBHS_POWER                  ((uint32_t)0x00000001UL) /**< Offset from USBHS Base Address: <tt> 0x0001</tt> */
#define MXC_R_USBHS_INTRIN                 ((uint32_t)0x00000002UL) /**< Offset from USBHS Base Address: <tt> 0x0002</tt> */
#define MXC_R_USBHS_INTROUT                ((uint32_t)0x00000004UL) /**< Offset from USBHS Base Address: <tt> 0x0004</tt> */
#define MXC_R_USBHS_INTRINEN               ((uint32_t)0x00000006UL) /**< Offset from USBHS Base Address: <tt> 0x0006</tt> */
#define MXC_R_USBHS_INTROUTEN              ((uint32_t)0x00000008UL) /**< Offset from USBHS Base Address: <tt> 0x0008</tt> */
#define MXC_R_USBHS_INTRUSB                ((uint32_t)0x0000000AUL) /**< Offset from USBHS Base Address: <tt> 0x000A</tt> */
#define MXC_R_USBHS_INTRUSBEN              ((uint32_t)0x0000000BUL) /**< Offset from USBHS Base Address: <tt> 0x000B</tt> */
#define MXC_R_USBHS_FRAME                  ((uint32_t)0x0000000CUL) /**< Offset from USBHS Base Address: <tt> 0x000C</tt> */
#define MXC_R_USBHS_INDEX                  ((uint32_t)0x0000000EUL) /**< Offset from USBHS Base Address: <tt> 0x000E</tt> */
#define MXC_R_USBHS_TESTMODE               ((uint32_t)0x0000000FUL) /**< Offset from USBHS Base Address: <tt> 0x000F</tt> */
#define MXC_R_USBHS_INMAXP                 ((uint32_t)0x00000010UL) /**< Offset from USBHS Base Address: <tt> 0x0010</tt> */
#define MXC_R_USBHS_CSR0                   ((uint32_t)0x00000012UL) /**< Offset from USBHS Base Address: <tt> 0x0012</tt> */
#define MXC_R_USBHS_INCSRL                 ((uint32_t)0x00000012UL) /**< Offset from USBHS Base Address: <tt> 0x0012</tt> */
#define MXC_R_USBHS_INCSRU                 ((uint32_t)0x00000013UL) /**< Offset from USBHS Base Address: <tt> 0x0013</tt> */
#define MXC_R_USBHS_OUTMAXP                ((uint32_t)0x00000014UL) /**< Offset from USBHS Base Address: <tt> 0x0014</tt> */
#define MXC_R_USBHS_OUTCSRL                ((uint32_t)0x00000016UL) /**< Offset from USBHS Base Address: <tt> 0x0016</tt> */
#define MXC_R_USBHS_OUTCSRU                ((uint32_t)0x00000017UL) /**< Offset from USBHS Base Address: <tt> 0x0017</tt> */
#define MXC_R_USBHS_COUNT0                 ((uint32_t)0x00000018UL) /**< Offset from USBHS Base Address: <tt> 0x0018</tt> */
#define MXC_R_USBHS_OUTCOUNT               ((uint32_t)0x00000018UL) /**< Offset from USBHS Base Address: <tt> 0x0018</tt> */
#define MXC_R_USBHS_FIFO0                  ((uint32_t)0x00000020UL) /**< Offset from USBHS Base Address: <tt> 0x0020</tt> */
#define MXC_R_USBHS_FIFO1                  ((uint32_t)0x00000024UL) /**< Offset from USBHS Base Address: <tt> 0x0024</tt> */
#define MXC_R_USBHS_FIFO2                  ((uint32_t)0x00000028UL) /**< Offset from USBHS Base Address: <tt> 0x0028</tt> */
#define MXC_R_USBHS_FIFO3                  ((uint32_t)0x0000002CUL) /**< Offset from USBHS Base Address: <tt> 0x002C</tt> */
#define MXC_R_USBHS_FIFO4                  ((uint32_t)0x00000030UL) /**< Offset from USBHS Base Address: <tt> 0x0030</tt> */
#define MXC_R_USBHS_FIFO5                  ((uint32_t)0x00000034UL) /**< Offset from USBHS Base Address: <tt> 0x0034</tt> */
#define MXC_R_USBHS_FIFO6                  ((uint32_t)0x00000038UL) /**< Offset from USBHS Base Address: <tt> 0x0038</tt> */
#define MXC_R_USBHS_FIFO7                  ((uint32_t)0x0000003CUL) /**< Offset from USBHS Base Address: <tt> 0x003C</tt> */
#define MXC_R_USBHS_FIFO8                  ((uint32_t)0x00000040UL) /**< Offset from USBHS Base Address: <tt> 0x0040</tt> */
#define MXC_R_USBHS_FIFO9                  ((uint32_t)0x00000044UL) /**< Offset from USBHS Base Address: <tt> 0x0044</tt> */
#define MXC_R_USBHS_FIFO10                 ((uint32_t)0x00000048UL) /**< Offset from USBHS Base Address: <tt> 0x0048</tt> */
#define MXC_R_USBHS_FIFO11                 ((uint32_t)0x0000004CUL) /**< Offset from USBHS Base Address: <tt> 0x004C</tt> */
#define MXC_R_USBHS_FIFO12                 ((uint32_t)0x00000050UL) /**< Offset from USBHS Base Address: <tt> 0x0050</tt> */
#define MXC_R_USBHS_FIFO13                 ((uint32_t)0x00000054UL) /**< Offset from USBHS Base Address: <tt> 0x0054</tt> */
#define MXC_R_USBHS_FIFO14                 ((uint32_t)0x00000058UL) /**< Offset from USBHS Base Address: <tt> 0x0058</tt> */
#define MXC_R_USBHS_FIFO15                 ((uint32_t)0x0000005CUL) /**< Offset from USBHS Base Address: <tt> 0x005C</tt> */
#define MXC_R_USBHS_HWVERS                 ((uint32_t)0x0000006CUL) /**< Offset from USBHS Base Address: <tt> 0x006C</tt> */
#define MXC_R_USBHS_EPINFO                 ((uint32_t)0x00000078UL) /**< Offset from USBHS Base Address: <tt> 0x0078</tt> */
#define MXC_R_USBHS_RAMINFO                ((uint32_t)0x00000079UL) /**< Offset from USBHS Base Address: <tt> 0x0079</tt> */
#define MXC_R_USBHS_SOFTRESET              ((uint32_t)0x0000007AUL) /**< Offset from USBHS Base Address: <tt> 0x007A</tt> */
#define MXC_R_USBHS_CTUCH                  ((uint32_t)0x00000080UL) /**< Offset from USBHS Base Address: <tt> 0x0080</tt> */
#define MXC_R_USBHS_CTHSRTN                ((uint32_t)0x00000082UL) /**< Offset from USBHS Base Address: <tt> 0x0082</tt> */
#define MXC_R_USBHS_MXM_USB_REG_00         ((uint32_t)0x00000400UL) /**< Offset from USBHS Base Address: <tt> 0x0400</tt> */
#define MXC_R_USBHS_M31_PHY_UTMI_RESET     ((uint32_t)0x00000404UL) /**< Offset from USBHS Base Address: <tt> 0x0404</tt> */
#define MXC_R_USBHS_M31_PHY_UTMI_VCONTROL  ((uint32_t)0x00000408UL) /**< Offset from USBHS Base Address: <tt> 0x0408</tt> */
#define MXC_R_USBHS_M31_PHY_CLK_EN         ((uint32_t)0x0000040CUL) /**< Offset from USBHS Base Address: <tt> 0x040C</tt> */
#define MXC_R_USBHS_M31_PHY_PONRST         ((uint32_t)0x00000410UL) /**< Offset from USBHS Base Address: <tt> 0x0410</tt> */
#define MXC_R_USBHS_M31_PHY_NONCRY_RSTB    ((uint32_t)0x00000414UL) /**< Offset from USBHS Base Address: <tt> 0x0414</tt> */
#define MXC_R_USBHS_M31_PHY_NONCRY_EN      ((uint32_t)0x00000418UL) /**< Offset from USBHS Base Address: <tt> 0x0418</tt> */
#define MXC_R_USBHS_M31_PHY_U2_COMPLIANCE_EN ((uint32_t)0x00000420UL) /**< Offset from USBHS Base Address: <tt> 0x0420</tt> */
#define MXC_R_USBHS_M31_PHY_U2_COMPLIANCE_DAC_ADJ ((uint32_t)0x00000424UL) /**< Offset from USBHS Base Address: <tt> 0x0424</tt> */
#define MXC_R_USBHS_M31_PHY_U2_COMPLIANCE_DAC_ADJ_EN ((uint32_t)0x00000428UL) /**< Offset from USBHS Base Address: <tt> 0x0428</tt> */
#define MXC_R_USBHS_M31_PHY_CLK_RDY        ((uint32_t)0x0000042CUL) /**< Offset from USBHS Base Address: <tt> 0x042C</tt> */
#define MXC_R_USBHS_M31_PHY_PLL_EN         ((uint32_t)0x00000430UL) /**< Offset from USBHS Base Address: <tt> 0x0430</tt> */
#define MXC_R_USBHS_M31_PHY_BIST_OK        ((uint32_t)0x00000434UL) /**< Offset from USBHS Base Address: <tt> 0x0434</tt> */
#define MXC_R_USBHS_M31_PHY_DATA_OE        ((uint32_t)0x00000438UL) /**< Offset from USBHS Base Address: <tt> 0x0438</tt> */
#define MXC_R_USBHS_M31_PHY_OSCOUTEN       ((uint32_t)0x0000043CUL) /**< Offset from USBHS Base Address: <tt> 0x043C</tt> */
#define MXC_R_USBHS_M31_PHY_LPM_ALIVE      ((uint32_t)0x00000440UL) /**< Offset from USBHS Base Address: <tt> 0x0440</tt> */
#define MXC_R_USBHS_M31_PHY_HS_BIST_MODE   ((uint32_t)0x00000444UL) /**< Offset from USBHS Base Address: <tt> 0x0444</tt> */
#define MXC_R_USBHS_M31_PHY_CORECLKIN      ((uint32_t)0x00000448UL) /**< Offset from USBHS Base Address: <tt> 0x0448</tt> */
#define MXC_R_USBHS_M31_PHY_XTLSEL         ((uint32_t)0x0000044CUL) /**< Offset from USBHS Base Address: <tt> 0x044C</tt> */
#define MXC_R_USBHS_M31_PHY_LS_EN          ((uint32_t)0x00000450UL) /**< Offset from USBHS Base Address: <tt> 0x0450</tt> */
#define MXC_R_USBHS_M31_PHY_DEBUG_SEL      ((uint32_t)0x00000454UL) /**< Offset from USBHS Base Address: <tt> 0x0454</tt> */
#define MXC_R_USBHS_M31_PHY_DEBUG_OUT      ((uint32_t)0x00000458UL) /**< Offset from USBHS Base Address: <tt> 0x0458</tt> */
#define MXC_R_USBHS_M31_PHY_OUTCLKSEL      ((uint32_t)0x0000045CUL) /**< Offset from USBHS Base Address: <tt> 0x045C</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_31_0     ((uint32_t)0x00000460UL) /**< Offset from USBHS Base Address: <tt> 0x0460</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_63_32    ((uint32_t)0x00000464UL) /**< Offset from USBHS Base Address: <tt> 0x0464</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_95_64    ((uint32_t)0x00000468UL) /**< Offset from USBHS Base Address: <tt> 0x0468</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_127_96   ((uint32_t)0x0000046CUL) /**< Offset from USBHS Base Address: <tt> 0x046C</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_137_128  ((uint32_t)0x00000470UL) /**< Offset from USBHS Base Address: <tt> 0x0470</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_HS_COARSE_TUNE_NUM ((uint32_t)0x00000474UL) /**< Offset from USBHS Base Address: <tt> 0x0474</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_HS_FINE_TUNE_NUM ((uint32_t)0x00000478UL) /**< Offset from USBHS Base Address: <tt> 0x0478</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_FS_COARSE_TUNE_NUM ((uint32_t)0x0000047CUL) /**< Offset from USBHS Base Address: <tt> 0x047C</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_FS_FINE_TUNE_NUM ((uint32_t)0x00000480UL) /**< Offset from USBHS Base Address: <tt> 0x0480</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_LOCK_RANGE_MAX ((uint32_t)0x00000484UL) /**< Offset from USBHS Base Address: <tt> 0x0484</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGI_LOCK_RANGE_MIN ((uint32_t)0x00000488UL) /**< Offset from USBHS Base Address: <tt> 0x0488</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_OB_RSEL   ((uint32_t)0x0000048CUL) /**< Offset from USBHS Base Address: <tt> 0x048C</tt> */
#define MXC_R_USBHS_M31_PHY_XCFG_OC_RSEL   ((uint32_t)0x00000490UL) /**< Offset from USBHS Base Address: <tt> 0x0490</tt> */
#define MXC_R_USBHS_M31_PHY_XCFGO          ((uint32_t)0x00000494UL) /**< Offset from USBHS Base Address: <tt> 0x0494</tt> */
#define MXC_R_USBHS_MXM_INT                ((uint32_t)0x00000498UL) /**< Offset from USBHS Base Address: <tt> 0x0498</tt> */
#define MXC_R_USBHS_MXM_INT_EN             ((uint32_t)0x0000049CUL) /**< Offset from USBHS Base Address: <tt> 0x049C</tt> */
#define MXC_R_USBHS_MXM_SUSPEND            ((uint32_t)0x000004A0UL) /**< Offset from USBHS Base Address: <tt> 0x04A0</tt> */
#define MXC_R_USBHS_MXM_REG_A4             ((uint32_t)0x000004A4UL) /**< Offset from USBHS Base Address: <tt> 0x04A4</tt> */
/**@} end of group usbhs_registers */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FADDR USBHS_FADDR
 * @brief    Function address register.
 * @{
 */
#define MXC_F_USBHS_FADDR_ADDR_POS                     0 /**< FADDR_ADDR Position */
#define MXC_F_USBHS_FADDR_ADDR                         ((uint8_t)(0x7FUL << MXC_F_USBHS_FADDR_ADDR_POS)) /**< FADDR_ADDR Mask */

#define MXC_F_USBHS_FADDR_UPDATE_POS                   7 /**< FADDR_UPDATE Position */
#define MXC_F_USBHS_FADDR_UPDATE                       ((uint8_t)(0x1UL << MXC_F_USBHS_FADDR_UPDATE_POS)) /**< FADDR_UPDATE Mask */

/**@} end of group USBHS_FADDR_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_POWER USBHS_POWER
 * @brief    Power management register.
 * @{
 */
#define MXC_F_USBHS_POWER_EN_SUSPENDM_POS              0 /**< POWER_EN_SUSPENDM Position */
#define MXC_F_USBHS_POWER_EN_SUSPENDM                  ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_EN_SUSPENDM_POS)) /**< POWER_EN_SUSPENDM Mask */

#define MXC_F_USBHS_POWER_SUSPEND_POS                  1 /**< POWER_SUSPEND Position */
#define MXC_F_USBHS_POWER_SUSPEND                      ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_SUSPEND_POS)) /**< POWER_SUSPEND Mask */

#define MXC_F_USBHS_POWER_RESUME_POS                   2 /**< POWER_RESUME Position */
#define MXC_F_USBHS_POWER_RESUME                       ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_RESUME_POS)) /**< POWER_RESUME Mask */

#define MXC_F_USBHS_POWER_RESET_POS                    3 /**< POWER_RESET Position */
#define MXC_F_USBHS_POWER_RESET                        ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_RESET_POS)) /**< POWER_RESET Mask */

#define MXC_F_USBHS_POWER_HS_MODE_POS                  4 /**< POWER_HS_MODE Position */
#define MXC_F_USBHS_POWER_HS_MODE                      ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_HS_MODE_POS)) /**< POWER_HS_MODE Mask */

#define MXC_F_USBHS_POWER_HS_ENABLE_POS                5 /**< POWER_HS_ENABLE Position */
#define MXC_F_USBHS_POWER_HS_ENABLE                    ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_HS_ENABLE_POS)) /**< POWER_HS_ENABLE Mask */

#define MXC_F_USBHS_POWER_SOFTCONN_POS                 6 /**< POWER_SOFTCONN Position */
#define MXC_F_USBHS_POWER_SOFTCONN                     ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_SOFTCONN_POS)) /**< POWER_SOFTCONN Mask */

#define MXC_F_USBHS_POWER_ISO_UPDATE_POS               7 /**< POWER_ISO_UPDATE Position */
#define MXC_F_USBHS_POWER_ISO_UPDATE                   ((uint8_t)(0x1UL << MXC_F_USBHS_POWER_ISO_UPDATE_POS)) /**< POWER_ISO_UPDATE Mask */

/**@} end of group USBHS_POWER_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTRIN USBHS_INTRIN
 * @brief    Interrupt register for EP0 and IN EP1-15.
 * @{
 */
#define MXC_F_USBHS_INTRIN_EP15_IN_INT_POS             15 /**< INTRIN_EP15_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP15_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP15_IN_INT_POS)) /**< INTRIN_EP15_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP14_IN_INT_POS             14 /**< INTRIN_EP14_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP14_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP14_IN_INT_POS)) /**< INTRIN_EP14_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP13_IN_INT_POS             13 /**< INTRIN_EP13_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP13_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP13_IN_INT_POS)) /**< INTRIN_EP13_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP12_IN_INT_POS             12 /**< INTRIN_EP12_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP12_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP12_IN_INT_POS)) /**< INTRIN_EP12_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP11_IN_INT_POS             11 /**< INTRIN_EP11_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP11_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP11_IN_INT_POS)) /**< INTRIN_EP11_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP10_IN_INT_POS             10 /**< INTRIN_EP10_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP10_IN_INT                 ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP10_IN_INT_POS)) /**< INTRIN_EP10_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP9_IN_INT_POS              9 /**< INTRIN_EP9_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP9_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP9_IN_INT_POS)) /**< INTRIN_EP9_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP8_IN_INT_POS              8 /**< INTRIN_EP8_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP8_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP8_IN_INT_POS)) /**< INTRIN_EP8_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP7_IN_INT_POS              7 /**< INTRIN_EP7_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP7_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP7_IN_INT_POS)) /**< INTRIN_EP7_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP6_IN_INT_POS              6 /**< INTRIN_EP6_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP6_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP6_IN_INT_POS)) /**< INTRIN_EP6_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP5_IN_INT_POS              5 /**< INTRIN_EP5_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP5_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP5_IN_INT_POS)) /**< INTRIN_EP5_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP4_IN_INT_POS              4 /**< INTRIN_EP4_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP4_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP4_IN_INT_POS)) /**< INTRIN_EP4_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP3_IN_INT_POS              3 /**< INTRIN_EP3_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP3_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP3_IN_INT_POS)) /**< INTRIN_EP3_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP2_IN_INT_POS              2 /**< INTRIN_EP2_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP2_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP2_IN_INT_POS)) /**< INTRIN_EP2_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP1_IN_INT_POS              1 /**< INTRIN_EP1_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP1_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP1_IN_INT_POS)) /**< INTRIN_EP1_IN_INT Mask */

#define MXC_F_USBHS_INTRIN_EP0_IN_INT_POS              0 /**< INTRIN_EP0_IN_INT Position */
#define MXC_F_USBHS_INTRIN_EP0_IN_INT                  ((uint16_t)(0x1UL << MXC_F_USBHS_INTRIN_EP0_IN_INT_POS)) /**< INTRIN_EP0_IN_INT Mask */

/**@} end of group USBHS_INTRIN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTROUT USBHS_INTROUT
 * @brief    Interrupt register for OUT EP 1-15.
 * @{
 */
#define MXC_F_USBHS_INTROUT_EP15_OUT_INT_POS           15 /**< INTROUT_EP15_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP15_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP15_OUT_INT_POS)) /**< INTROUT_EP15_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP14_OUT_INT_POS           14 /**< INTROUT_EP14_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP14_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP14_OUT_INT_POS)) /**< INTROUT_EP14_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP13_OUT_INT_POS           13 /**< INTROUT_EP13_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP13_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP13_OUT_INT_POS)) /**< INTROUT_EP13_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP12_OUT_INT_POS           12 /**< INTROUT_EP12_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP12_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP12_OUT_INT_POS)) /**< INTROUT_EP12_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP11_OUT_INT_POS           11 /**< INTROUT_EP11_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP11_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP11_OUT_INT_POS)) /**< INTROUT_EP11_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP10_OUT_INT_POS           10 /**< INTROUT_EP10_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP10_OUT_INT               ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP10_OUT_INT_POS)) /**< INTROUT_EP10_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP9_OUT_INT_POS            9 /**< INTROUT_EP9_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP9_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP9_OUT_INT_POS)) /**< INTROUT_EP9_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP8_OUT_INT_POS            8 /**< INTROUT_EP8_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP8_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP8_OUT_INT_POS)) /**< INTROUT_EP8_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP7_OUT_INT_POS            7 /**< INTROUT_EP7_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP7_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP7_OUT_INT_POS)) /**< INTROUT_EP7_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP6_OUT_INT_POS            6 /**< INTROUT_EP6_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP6_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP6_OUT_INT_POS)) /**< INTROUT_EP6_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP5_OUT_INT_POS            5 /**< INTROUT_EP5_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP5_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP5_OUT_INT_POS)) /**< INTROUT_EP5_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP4_OUT_INT_POS            4 /**< INTROUT_EP4_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP4_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP4_OUT_INT_POS)) /**< INTROUT_EP4_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP3_OUT_INT_POS            3 /**< INTROUT_EP3_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP3_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP3_OUT_INT_POS)) /**< INTROUT_EP3_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP2_OUT_INT_POS            2 /**< INTROUT_EP2_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP2_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP2_OUT_INT_POS)) /**< INTROUT_EP2_OUT_INT Mask */

#define MXC_F_USBHS_INTROUT_EP1_OUT_INT_POS            1 /**< INTROUT_EP1_OUT_INT Position */
#define MXC_F_USBHS_INTROUT_EP1_OUT_INT                ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUT_EP1_OUT_INT_POS)) /**< INTROUT_EP1_OUT_INT Mask */

/**@} end of group USBHS_INTROUT_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTRINEN USBHS_INTRINEN
 * @brief    Interrupt enable for EP 0 and IN EP 1-15.
 * @{
 */
#define MXC_F_USBHS_INTRINEN_EP15_IN_INT_EN_POS        15 /**< INTRINEN_EP15_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP15_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP15_IN_INT_EN_POS)) /**< INTRINEN_EP15_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP14_IN_INT_EN_POS        14 /**< INTRINEN_EP14_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP14_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP14_IN_INT_EN_POS)) /**< INTRINEN_EP14_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP13_IN_INT_EN_POS        13 /**< INTRINEN_EP13_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP13_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP13_IN_INT_EN_POS)) /**< INTRINEN_EP13_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP12_IN_INT_EN_POS        12 /**< INTRINEN_EP12_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP12_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP12_IN_INT_EN_POS)) /**< INTRINEN_EP12_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP11_IN_INT_EN_POS        11 /**< INTRINEN_EP11_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP11_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP11_IN_INT_EN_POS)) /**< INTRINEN_EP11_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP10_IN_INT_EN_POS        10 /**< INTRINEN_EP10_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP10_IN_INT_EN            ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP10_IN_INT_EN_POS)) /**< INTRINEN_EP10_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP9_IN_INT_EN_POS         9 /**< INTRINEN_EP9_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP9_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP9_IN_INT_EN_POS)) /**< INTRINEN_EP9_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP8_IN_INT_EN_POS         8 /**< INTRINEN_EP8_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP8_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP8_IN_INT_EN_POS)) /**< INTRINEN_EP8_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP7_IN_INT_EN_POS         7 /**< INTRINEN_EP7_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP7_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP7_IN_INT_EN_POS)) /**< INTRINEN_EP7_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP6_IN_INT_EN_POS         6 /**< INTRINEN_EP6_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP6_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP6_IN_INT_EN_POS)) /**< INTRINEN_EP6_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP5_IN_INT_EN_POS         5 /**< INTRINEN_EP5_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP5_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP5_IN_INT_EN_POS)) /**< INTRINEN_EP5_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP4_IN_INT_EN_POS         4 /**< INTRINEN_EP4_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP4_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP4_IN_INT_EN_POS)) /**< INTRINEN_EP4_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP3_IN_INT_EN_POS         3 /**< INTRINEN_EP3_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP3_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP3_IN_INT_EN_POS)) /**< INTRINEN_EP3_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP2_IN_INT_EN_POS         2 /**< INTRINEN_EP2_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP2_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP2_IN_INT_EN_POS)) /**< INTRINEN_EP2_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP1_IN_INT_EN_POS         1 /**< INTRINEN_EP1_IN_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP1_IN_INT_EN             ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP1_IN_INT_EN_POS)) /**< INTRINEN_EP1_IN_INT_EN Mask */

#define MXC_F_USBHS_INTRINEN_EP0_INT_EN_POS            0 /**< INTRINEN_EP0_INT_EN Position */
#define MXC_F_USBHS_INTRINEN_EP0_INT_EN                ((uint16_t)(0x1UL << MXC_F_USBHS_INTRINEN_EP0_INT_EN_POS)) /**< INTRINEN_EP0_INT_EN Mask */

/**@} end of group USBHS_INTRINEN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTROUTEN USBHS_INTROUTEN
 * @brief    Interrupt enable for OUT EP 1-15.
 * @{
 */
#define MXC_F_USBHS_INTROUTEN_EP15_OUT_INT_EN_POS      15 /**< INTROUTEN_EP15_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP15_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP15_OUT_INT_EN_POS)) /**< INTROUTEN_EP15_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP14_OUT_INT_EN_POS      14 /**< INTROUTEN_EP14_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP14_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP14_OUT_INT_EN_POS)) /**< INTROUTEN_EP14_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP13_OUT_INT_EN_POS      13 /**< INTROUTEN_EP13_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP13_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP13_OUT_INT_EN_POS)) /**< INTROUTEN_EP13_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP12_OUT_INT_EN_POS      12 /**< INTROUTEN_EP12_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP12_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP12_OUT_INT_EN_POS)) /**< INTROUTEN_EP12_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP11_OUT_INT_EN_POS      11 /**< INTROUTEN_EP11_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP11_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP11_OUT_INT_EN_POS)) /**< INTROUTEN_EP11_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP10_OUT_INT_EN_POS      10 /**< INTROUTEN_EP10_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP10_OUT_INT_EN          ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP10_OUT_INT_EN_POS)) /**< INTROUTEN_EP10_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP9_OUT_INT_EN_POS       9 /**< INTROUTEN_EP9_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP9_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP9_OUT_INT_EN_POS)) /**< INTROUTEN_EP9_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP8_OUT_INT_EN_POS       8 /**< INTROUTEN_EP8_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP8_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP8_OUT_INT_EN_POS)) /**< INTROUTEN_EP8_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP7_OUT_INT_EN_POS       7 /**< INTROUTEN_EP7_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP7_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP7_OUT_INT_EN_POS)) /**< INTROUTEN_EP7_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP6_OUT_INT_EN_POS       6 /**< INTROUTEN_EP6_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP6_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP6_OUT_INT_EN_POS)) /**< INTROUTEN_EP6_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP5_OUT_INT_EN_POS       5 /**< INTROUTEN_EP5_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP5_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP5_OUT_INT_EN_POS)) /**< INTROUTEN_EP5_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP4_OUT_INT_EN_POS       4 /**< INTROUTEN_EP4_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP4_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP4_OUT_INT_EN_POS)) /**< INTROUTEN_EP4_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP3_OUT_INT_EN_POS       3 /**< INTROUTEN_EP3_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP3_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP3_OUT_INT_EN_POS)) /**< INTROUTEN_EP3_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP2_OUT_INT_EN_POS       2 /**< INTROUTEN_EP2_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP2_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP2_OUT_INT_EN_POS)) /**< INTROUTEN_EP2_OUT_INT_EN Mask */

#define MXC_F_USBHS_INTROUTEN_EP1_OUT_INT_EN_POS       1 /**< INTROUTEN_EP1_OUT_INT_EN Position */
#define MXC_F_USBHS_INTROUTEN_EP1_OUT_INT_EN           ((uint16_t)(0x1UL << MXC_F_USBHS_INTROUTEN_EP1_OUT_INT_EN_POS)) /**< INTROUTEN_EP1_OUT_INT_EN Mask */

/**@} end of group USBHS_INTROUTEN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTRUSB USBHS_INTRUSB
 * @brief    Interrupt register for common USB interrupts.
 * @{
 */
#define MXC_F_USBHS_INTRUSB_SOF_INT_POS                3 /**< INTRUSB_SOF_INT Position */
#define MXC_F_USBHS_INTRUSB_SOF_INT                    ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSB_SOF_INT_POS)) /**< INTRUSB_SOF_INT Mask */

#define MXC_F_USBHS_INTRUSB_RESET_INT_POS              2 /**< INTRUSB_RESET_INT Position */
#define MXC_F_USBHS_INTRUSB_RESET_INT                  ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSB_RESET_INT_POS)) /**< INTRUSB_RESET_INT Mask */

#define MXC_F_USBHS_INTRUSB_RESUME_INT_POS             1 /**< INTRUSB_RESUME_INT Position */
#define MXC_F_USBHS_INTRUSB_RESUME_INT                 ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSB_RESUME_INT_POS)) /**< INTRUSB_RESUME_INT Mask */

#define MXC_F_USBHS_INTRUSB_SUSPEND_INT_POS            0 /**< INTRUSB_SUSPEND_INT Position */
#define MXC_F_USBHS_INTRUSB_SUSPEND_INT                ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSB_SUSPEND_INT_POS)) /**< INTRUSB_SUSPEND_INT Mask */

/**@} end of group USBHS_INTRUSB_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INTRUSBEN USBHS_INTRUSBEN
 * @brief    Interrupt enable for common USB interrupts.
 * @{
 */
#define MXC_F_USBHS_INTRUSBEN_SOF_INT_EN_POS           3 /**< INTRUSBEN_SOF_INT_EN Position */
#define MXC_F_USBHS_INTRUSBEN_SOF_INT_EN               ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSBEN_SOF_INT_EN_POS)) /**< INTRUSBEN_SOF_INT_EN Mask */

#define MXC_F_USBHS_INTRUSBEN_RESET_INT_EN_POS         2 /**< INTRUSBEN_RESET_INT_EN Position */
#define MXC_F_USBHS_INTRUSBEN_RESET_INT_EN             ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSBEN_RESET_INT_EN_POS)) /**< INTRUSBEN_RESET_INT_EN Mask */

#define MXC_F_USBHS_INTRUSBEN_RESUME_INT_EN_POS        1 /**< INTRUSBEN_RESUME_INT_EN Position */
#define MXC_F_USBHS_INTRUSBEN_RESUME_INT_EN            ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSBEN_RESUME_INT_EN_POS)) /**< INTRUSBEN_RESUME_INT_EN Mask */

#define MXC_F_USBHS_INTRUSBEN_SUSPEND_INT_EN_POS       0 /**< INTRUSBEN_SUSPEND_INT_EN Position */
#define MXC_F_USBHS_INTRUSBEN_SUSPEND_INT_EN           ((uint8_t)(0x1UL << MXC_F_USBHS_INTRUSBEN_SUSPEND_INT_EN_POS)) /**< INTRUSBEN_SUSPEND_INT_EN Mask */

/**@} end of group USBHS_INTRUSBEN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FRAME USBHS_FRAME
 * @brief    Frame number.
 * @{
 */
#define MXC_F_USBHS_FRAME_FRAMENUM_POS                 0 /**< FRAME_FRAMENUM Position */
#define MXC_F_USBHS_FRAME_FRAMENUM                     ((uint16_t)(0x7FFUL << MXC_F_USBHS_FRAME_FRAMENUM_POS)) /**< FRAME_FRAMENUM Mask */

/**@} end of group USBHS_FRAME_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INDEX USBHS_INDEX
 * @brief    Index for banked registers.
 * @{
 */
#define MXC_F_USBHS_INDEX_INDEX_POS                    0 /**< INDEX_INDEX Position */
#define MXC_F_USBHS_INDEX_INDEX                        ((uint8_t)(0xFUL << MXC_F_USBHS_INDEX_INDEX_POS)) /**< INDEX_INDEX Mask */

/**@} end of group USBHS_INDEX_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_TESTMODE USBHS_TESTMODE
 * @brief    USB 2.0 test mode enable register.
 * @{
 */
#define MXC_F_USBHS_TESTMODE_FORCE_FS_POS              5 /**< TESTMODE_FORCE_FS Position */
#define MXC_F_USBHS_TESTMODE_FORCE_FS                  ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_FORCE_FS_POS)) /**< TESTMODE_FORCE_FS Mask */

#define MXC_F_USBHS_TESTMODE_FORCE_HS_POS              4 /**< TESTMODE_FORCE_HS Position */
#define MXC_F_USBHS_TESTMODE_FORCE_HS                  ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_FORCE_HS_POS)) /**< TESTMODE_FORCE_HS Mask */

#define MXC_F_USBHS_TESTMODE_TEST_PKT_POS              3 /**< TESTMODE_TEST_PKT Position */
#define MXC_F_USBHS_TESTMODE_TEST_PKT                  ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_TEST_PKT_POS)) /**< TESTMODE_TEST_PKT Mask */

#define MXC_F_USBHS_TESTMODE_TEST_K_POS                2 /**< TESTMODE_TEST_K Position */
#define MXC_F_USBHS_TESTMODE_TEST_K                    ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_TEST_K_POS)) /**< TESTMODE_TEST_K Mask */

#define MXC_F_USBHS_TESTMODE_TEST_J_POS                1 /**< TESTMODE_TEST_J Position */
#define MXC_F_USBHS_TESTMODE_TEST_J                    ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_TEST_J_POS)) /**< TESTMODE_TEST_J Mask */

#define MXC_F_USBHS_TESTMODE_TEST_SE0_NAK_POS          0 /**< TESTMODE_TEST_SE0_NAK Position */
#define MXC_F_USBHS_TESTMODE_TEST_SE0_NAK              ((uint8_t)(0x1UL << MXC_F_USBHS_TESTMODE_TEST_SE0_NAK_POS)) /**< TESTMODE_TEST_SE0_NAK Mask */

/**@} end of group USBHS_TESTMODE_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INMAXP USBHS_INMAXP
 * @brief    Maximum packet size for INx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_INMAXP_MAXPACKETSIZE_POS           0 /**< INMAXP_MAXPACKETSIZE Position */
#define MXC_F_USBHS_INMAXP_MAXPACKETSIZE               ((uint16_t)(0x7FFUL << MXC_F_USBHS_INMAXP_MAXPACKETSIZE_POS)) /**< INMAXP_MAXPACKETSIZE Mask */

#define MXC_F_USBHS_INMAXP_NUMPACKMINUS1_POS           11 /**< INMAXP_NUMPACKMINUS1 Position */
#define MXC_F_USBHS_INMAXP_NUMPACKMINUS1               ((uint16_t)(0x1FUL << MXC_F_USBHS_INMAXP_NUMPACKMINUS1_POS)) /**< INMAXP_NUMPACKMINUS1 Mask */

/**@} end of group USBHS_INMAXP_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_CSR0 USBHS_CSR0
 * @brief    Control status register for EP 0 (when INDEX == 0).
 * @{
 */
#define MXC_F_USBHS_CSR0_SERV_SETUP_END_POS            7 /**< CSR0_SERV_SETUP_END Position */
#define MXC_F_USBHS_CSR0_SERV_SETUP_END                ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_SERV_SETUP_END_POS)) /**< CSR0_SERV_SETUP_END Mask */

#define MXC_F_USBHS_CSR0_SERV_OUTPKTRDY_POS            6 /**< CSR0_SERV_OUTPKTRDY Position */
#define MXC_F_USBHS_CSR0_SERV_OUTPKTRDY                ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_SERV_OUTPKTRDY_POS)) /**< CSR0_SERV_OUTPKTRDY Mask */

#define MXC_F_USBHS_CSR0_SEND_STALL_POS                5 /**< CSR0_SEND_STALL Position */
#define MXC_F_USBHS_CSR0_SEND_STALL                    ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_SEND_STALL_POS)) /**< CSR0_SEND_STALL Mask */

#define MXC_F_USBHS_CSR0_SETUP_END_POS                 4 /**< CSR0_SETUP_END Position */
#define MXC_F_USBHS_CSR0_SETUP_END                     ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_SETUP_END_POS)) /**< CSR0_SETUP_END Mask */

#define MXC_F_USBHS_CSR0_DATA_END_POS                  3 /**< CSR0_DATA_END Position */
#define MXC_F_USBHS_CSR0_DATA_END                      ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_DATA_END_POS)) /**< CSR0_DATA_END Mask */

#define MXC_F_USBHS_CSR0_SENT_STALL_POS                2 /**< CSR0_SENT_STALL Position */
#define MXC_F_USBHS_CSR0_SENT_STALL                    ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_SENT_STALL_POS)) /**< CSR0_SENT_STALL Mask */

#define MXC_F_USBHS_CSR0_INPKTRDY_POS                  1 /**< CSR0_INPKTRDY Position */
#define MXC_F_USBHS_CSR0_INPKTRDY                      ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_INPKTRDY_POS)) /**< CSR0_INPKTRDY Mask */

#define MXC_F_USBHS_CSR0_OUTPKTRDY_POS                 0 /**< CSR0_OUTPKTRDY Position */
#define MXC_F_USBHS_CSR0_OUTPKTRDY                     ((uint8_t)(0x1UL << MXC_F_USBHS_CSR0_OUTPKTRDY_POS)) /**< CSR0_OUTPKTRDY Mask */

/**@} end of group USBHS_CSR0_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INCSRL USBHS_INCSRL
 * @brief    Control status lower register for INx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_INCSRL_INCOMPTX_POS                7 /**< INCSRL_INCOMPTX Position */
#define MXC_F_USBHS_INCSRL_INCOMPTX                    ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_INCOMPTX_POS)) /**< INCSRL_INCOMPTX Mask */

#define MXC_F_USBHS_INCSRL_CLRDATATOG_POS              6 /**< INCSRL_CLRDATATOG Position */
#define MXC_F_USBHS_INCSRL_CLRDATATOG                  ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_CLRDATATOG_POS)) /**< INCSRL_CLRDATATOG Mask */

#define MXC_F_USBHS_INCSRL_SENTSTALL_POS               5 /**< INCSRL_SENTSTALL Position */
#define MXC_F_USBHS_INCSRL_SENTSTALL                   ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_SENTSTALL_POS)) /**< INCSRL_SENTSTALL Mask */

#define MXC_F_USBHS_INCSRL_SENDSTALL_POS               4 /**< INCSRL_SENDSTALL Position */
#define MXC_F_USBHS_INCSRL_SENDSTALL                   ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_SENDSTALL_POS)) /**< INCSRL_SENDSTALL Mask */

#define MXC_F_USBHS_INCSRL_FLUSHFIFO_POS               3 /**< INCSRL_FLUSHFIFO Position */
#define MXC_F_USBHS_INCSRL_FLUSHFIFO                   ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_FLUSHFIFO_POS)) /**< INCSRL_FLUSHFIFO Mask */

#define MXC_F_USBHS_INCSRL_UNDERRUN_POS                2 /**< INCSRL_UNDERRUN Position */
#define MXC_F_USBHS_INCSRL_UNDERRUN                    ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_UNDERRUN_POS)) /**< INCSRL_UNDERRUN Mask */

#define MXC_F_USBHS_INCSRL_FIFONOTEMPTY_POS            1 /**< INCSRL_FIFONOTEMPTY Position */
#define MXC_F_USBHS_INCSRL_FIFONOTEMPTY                ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_FIFONOTEMPTY_POS)) /**< INCSRL_FIFONOTEMPTY Mask */

#define MXC_F_USBHS_INCSRL_INPKTRDY_POS                0 /**< INCSRL_INPKTRDY Position */
#define MXC_F_USBHS_INCSRL_INPKTRDY                    ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRL_INPKTRDY_POS)) /**< INCSRL_INPKTRDY Mask */

/**@} end of group USBHS_INCSRL_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_INCSRU USBHS_INCSRU
 * @brief    Control status upper register for INx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_INCSRU_AUTOSET_POS                 7 /**< INCSRU_AUTOSET Position */
#define MXC_F_USBHS_INCSRU_AUTOSET                     ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRU_AUTOSET_POS)) /**< INCSRU_AUTOSET Mask */

#define MXC_F_USBHS_INCSRU_ISO_POS                     6 /**< INCSRU_ISO Position */
#define MXC_F_USBHS_INCSRU_ISO                         ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRU_ISO_POS)) /**< INCSRU_ISO Mask */

#define MXC_F_USBHS_INCSRU_MODE_POS                    5 /**< INCSRU_MODE Position */
#define MXC_F_USBHS_INCSRU_MODE                        ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRU_MODE_POS)) /**< INCSRU_MODE Mask */

#define MXC_F_USBHS_INCSRU_FRCDATATOG_POS              3 /**< INCSRU_FRCDATATOG Position */
#define MXC_F_USBHS_INCSRU_FRCDATATOG                  ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRU_FRCDATATOG_POS)) /**< INCSRU_FRCDATATOG Mask */

#define MXC_F_USBHS_INCSRU_DPKTBUFDIS_POS              1 /**< INCSRU_DPKTBUFDIS Position */
#define MXC_F_USBHS_INCSRU_DPKTBUFDIS                  ((uint8_t)(0x1UL << MXC_F_USBHS_INCSRU_DPKTBUFDIS_POS)) /**< INCSRU_DPKTBUFDIS Mask */

/**@} end of group USBHS_INCSRU_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_OUTMAXP USBHS_OUTMAXP
 * @brief    Maximum packet size for OUTx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_OUTMAXP_NUMPACKMINUS1_POS          11 /**< OUTMAXP_NUMPACKMINUS1 Position */
#define MXC_F_USBHS_OUTMAXP_NUMPACKMINUS1              ((uint16_t)(0x1FUL << MXC_F_USBHS_OUTMAXP_NUMPACKMINUS1_POS)) /**< OUTMAXP_NUMPACKMINUS1 Mask */

#define MXC_F_USBHS_OUTMAXP_MAXPACKETSIZE_POS          0 /**< OUTMAXP_MAXPACKETSIZE Position */
#define MXC_F_USBHS_OUTMAXP_MAXPACKETSIZE              ((uint16_t)(0x7FFUL << MXC_F_USBHS_OUTMAXP_MAXPACKETSIZE_POS)) /**< OUTMAXP_MAXPACKETSIZE Mask */

/**@} end of group USBHS_OUTMAXP_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_OUTCSRL USBHS_OUTCSRL
 * @brief    Control status lower register for OUTx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_OUTCSRL_CLRDATATOG_POS             7 /**< OUTCSRL_CLRDATATOG Position */
#define MXC_F_USBHS_OUTCSRL_CLRDATATOG                 ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_CLRDATATOG_POS)) /**< OUTCSRL_CLRDATATOG Mask */

#define MXC_F_USBHS_OUTCSRL_SENTSTALL_POS              6 /**< OUTCSRL_SENTSTALL Position */
#define MXC_F_USBHS_OUTCSRL_SENTSTALL                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_SENTSTALL_POS)) /**< OUTCSRL_SENTSTALL Mask */

#define MXC_F_USBHS_OUTCSRL_SENDSTALL_POS              5 /**< OUTCSRL_SENDSTALL Position */
#define MXC_F_USBHS_OUTCSRL_SENDSTALL                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_SENDSTALL_POS)) /**< OUTCSRL_SENDSTALL Mask */

#define MXC_F_USBHS_OUTCSRL_FLUSHFIFO_POS              4 /**< OUTCSRL_FLUSHFIFO Position */
#define MXC_F_USBHS_OUTCSRL_FLUSHFIFO                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_FLUSHFIFO_POS)) /**< OUTCSRL_FLUSHFIFO Mask */

#define MXC_F_USBHS_OUTCSRL_DATAERROR_POS              3 /**< OUTCSRL_DATAERROR Position */
#define MXC_F_USBHS_OUTCSRL_DATAERROR                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_DATAERROR_POS)) /**< OUTCSRL_DATAERROR Mask */

#define MXC_F_USBHS_OUTCSRL_OVERRUN_POS                2 /**< OUTCSRL_OVERRUN Position */
#define MXC_F_USBHS_OUTCSRL_OVERRUN                    ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_OVERRUN_POS)) /**< OUTCSRL_OVERRUN Mask */

#define MXC_F_USBHS_OUTCSRL_FIFOFULL_POS               1 /**< OUTCSRL_FIFOFULL Position */
#define MXC_F_USBHS_OUTCSRL_FIFOFULL                   ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_FIFOFULL_POS)) /**< OUTCSRL_FIFOFULL Mask */

#define MXC_F_USBHS_OUTCSRL_OUTPKTRDY_POS              0 /**< OUTCSRL_OUTPKTRDY Position */
#define MXC_F_USBHS_OUTCSRL_OUTPKTRDY                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRL_OUTPKTRDY_POS)) /**< OUTCSRL_OUTPKTRDY Mask */

/**@} end of group USBHS_OUTCSRL_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_OUTCSRU USBHS_OUTCSRU
 * @brief    Control status upper register for OUTx endpoint (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_OUTCSRU_AUTOCLEAR_POS              7 /**< OUTCSRU_AUTOCLEAR Position */
#define MXC_F_USBHS_OUTCSRU_AUTOCLEAR                  ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRU_AUTOCLEAR_POS)) /**< OUTCSRU_AUTOCLEAR Mask */

#define MXC_F_USBHS_OUTCSRU_ISO_POS                    6 /**< OUTCSRU_ISO Position */
#define MXC_F_USBHS_OUTCSRU_ISO                        ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRU_ISO_POS)) /**< OUTCSRU_ISO Mask */

#define MXC_F_USBHS_OUTCSRU_DISNYET_POS                4 /**< OUTCSRU_DISNYET Position */
#define MXC_F_USBHS_OUTCSRU_DISNYET                    ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRU_DISNYET_POS)) /**< OUTCSRU_DISNYET Mask */

#define MXC_F_USBHS_OUTCSRU_DPKTBUFDIS_POS             1 /**< OUTCSRU_DPKTBUFDIS Position */
#define MXC_F_USBHS_OUTCSRU_DPKTBUFDIS                 ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRU_DPKTBUFDIS_POS)) /**< OUTCSRU_DPKTBUFDIS Mask */

#define MXC_F_USBHS_OUTCSRU_INCOMPRX_POS               0 /**< OUTCSRU_INCOMPRX Position */
#define MXC_F_USBHS_OUTCSRU_INCOMPRX                   ((uint8_t)(0x1UL << MXC_F_USBHS_OUTCSRU_INCOMPRX_POS)) /**< OUTCSRU_INCOMPRX Mask */

/**@} end of group USBHS_OUTCSRU_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_COUNT0 USBHS_COUNT0
 * @brief    Number of received bytes in EP 0 FIFO (INDEX == 0).
 * @{
 */
#define MXC_F_USBHS_COUNT0_COUNT0_POS                  0 /**< COUNT0_COUNT0 Position */
#define MXC_F_USBHS_COUNT0_COUNT0                      ((uint16_t)(0x7FUL << MXC_F_USBHS_COUNT0_COUNT0_POS)) /**< COUNT0_COUNT0 Mask */

/**@} end of group USBHS_COUNT0_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_OUTCOUNT USBHS_OUTCOUNT
 * @brief    Number of received bytes in OUT EPx FIFO (x == INDEX).
 * @{
 */
#define MXC_F_USBHS_OUTCOUNT_OUTCOUNT_POS              0 /**< OUTCOUNT_OUTCOUNT Position */
#define MXC_F_USBHS_OUTCOUNT_OUTCOUNT                  ((uint16_t)(0x1FFFUL << MXC_F_USBHS_OUTCOUNT_OUTCOUNT_POS)) /**< OUTCOUNT_OUTCOUNT Mask */

/**@} end of group USBHS_OUTCOUNT_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO0 USBHS_FIFO0
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO0_USBHS_FIFO0_POS              0 /**< FIFO0_USBHS_FIFO0 Position */
#define MXC_F_USBHS_FIFO0_USBHS_FIFO0                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO0_USBHS_FIFO0_POS)) /**< FIFO0_USBHS_FIFO0 Mask */

/**@} end of group USBHS_FIFO0_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO1 USBHS_FIFO1
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO1_USBHS_FIFO1_POS              0 /**< FIFO1_USBHS_FIFO1 Position */
#define MXC_F_USBHS_FIFO1_USBHS_FIFO1                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO1_USBHS_FIFO1_POS)) /**< FIFO1_USBHS_FIFO1 Mask */

/**@} end of group USBHS_FIFO1_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO2 USBHS_FIFO2
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO2_USBHS_FIFO2_POS              0 /**< FIFO2_USBHS_FIFO2 Position */
#define MXC_F_USBHS_FIFO2_USBHS_FIFO2                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO2_USBHS_FIFO2_POS)) /**< FIFO2_USBHS_FIFO2 Mask */

/**@} end of group USBHS_FIFO2_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO3 USBHS_FIFO3
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO3_USBHS_FIFO3_POS              0 /**< FIFO3_USBHS_FIFO3 Position */
#define MXC_F_USBHS_FIFO3_USBHS_FIFO3                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO3_USBHS_FIFO3_POS)) /**< FIFO3_USBHS_FIFO3 Mask */

/**@} end of group USBHS_FIFO3_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO4 USBHS_FIFO4
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO4_USBHS_FIFO4_POS              0 /**< FIFO4_USBHS_FIFO4 Position */
#define MXC_F_USBHS_FIFO4_USBHS_FIFO4                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO4_USBHS_FIFO4_POS)) /**< FIFO4_USBHS_FIFO4 Mask */

/**@} end of group USBHS_FIFO4_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO5 USBHS_FIFO5
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO5_USBHS_FIFO5_POS              0 /**< FIFO5_USBHS_FIFO5 Position */
#define MXC_F_USBHS_FIFO5_USBHS_FIFO5                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO5_USBHS_FIFO5_POS)) /**< FIFO5_USBHS_FIFO5 Mask */

/**@} end of group USBHS_FIFO5_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO6 USBHS_FIFO6
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO6_USBHS_FIFO6_POS              0 /**< FIFO6_USBHS_FIFO6 Position */
#define MXC_F_USBHS_FIFO6_USBHS_FIFO6                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO6_USBHS_FIFO6_POS)) /**< FIFO6_USBHS_FIFO6 Mask */

/**@} end of group USBHS_FIFO6_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO7 USBHS_FIFO7
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO7_USBHS_FIFO7_POS              0 /**< FIFO7_USBHS_FIFO7 Position */
#define MXC_F_USBHS_FIFO7_USBHS_FIFO7                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO7_USBHS_FIFO7_POS)) /**< FIFO7_USBHS_FIFO7 Mask */

/**@} end of group USBHS_FIFO7_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO8 USBHS_FIFO8
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO8_USBHS_FIFO8_POS              0 /**< FIFO8_USBHS_FIFO8 Position */
#define MXC_F_USBHS_FIFO8_USBHS_FIFO8                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO8_USBHS_FIFO8_POS)) /**< FIFO8_USBHS_FIFO8 Mask */

/**@} end of group USBHS_FIFO8_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO9 USBHS_FIFO9
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO9_USBHS_FIFO9_POS              0 /**< FIFO9_USBHS_FIFO9 Position */
#define MXC_F_USBHS_FIFO9_USBHS_FIFO9                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO9_USBHS_FIFO9_POS)) /**< FIFO9_USBHS_FIFO9 Mask */

/**@} end of group USBHS_FIFO9_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO10 USBHS_FIFO10
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO10_USBHS_FIFO10_POS            0 /**< FIFO10_USBHS_FIFO10 Position */
#define MXC_F_USBHS_FIFO10_USBHS_FIFO10                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO10_USBHS_FIFO10_POS)) /**< FIFO10_USBHS_FIFO10 Mask */

/**@} end of group USBHS_FIFO10_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO11 USBHS_FIFO11
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO11_USBHS_FIFO11_POS            0 /**< FIFO11_USBHS_FIFO11 Position */
#define MXC_F_USBHS_FIFO11_USBHS_FIFO11                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO11_USBHS_FIFO11_POS)) /**< FIFO11_USBHS_FIFO11 Mask */

/**@} end of group USBHS_FIFO11_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO12 USBHS_FIFO12
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO12_USBHS_FIFO12_POS            0 /**< FIFO12_USBHS_FIFO12 Position */
#define MXC_F_USBHS_FIFO12_USBHS_FIFO12                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO12_USBHS_FIFO12_POS)) /**< FIFO12_USBHS_FIFO12 Mask */

/**@} end of group USBHS_FIFO12_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO13 USBHS_FIFO13
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO13_USBHS_FIFO13_POS            0 /**< FIFO13_USBHS_FIFO13 Position */
#define MXC_F_USBHS_FIFO13_USBHS_FIFO13                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO13_USBHS_FIFO13_POS)) /**< FIFO13_USBHS_FIFO13 Mask */

/**@} end of group USBHS_FIFO13_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO14 USBHS_FIFO14
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO14_USBHS_FIFO14_POS            0 /**< FIFO14_USBHS_FIFO14 Position */
#define MXC_F_USBHS_FIFO14_USBHS_FIFO14                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO14_USBHS_FIFO14_POS)) /**< FIFO14_USBHS_FIFO14 Mask */

/**@} end of group USBHS_FIFO14_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_FIFO15 USBHS_FIFO15
 * @brief    Read for OUT data FIFO, write for IN data FIFO.
 * @{
 */
#define MXC_F_USBHS_FIFO15_USBHS_FIFO15_POS            0 /**< FIFO15_USBHS_FIFO15 Position */
#define MXC_F_USBHS_FIFO15_USBHS_FIFO15                ((uint32_t)(0xFFFFFFFFUL << MXC_F_USBHS_FIFO15_USBHS_FIFO15_POS)) /**< FIFO15_USBHS_FIFO15 Mask */

/**@} end of group USBHS_FIFO15_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_HWVERS USBHS_HWVERS
 * @brief    HWVERS
 * @{
 */
#define MXC_F_USBHS_HWVERS_USBHS_HWVERS_POS            0 /**< HWVERS_USBHS_HWVERS Position */
#define MXC_F_USBHS_HWVERS_USBHS_HWVERS                ((uint16_t)(0xFFFFUL << MXC_F_USBHS_HWVERS_USBHS_HWVERS_POS)) /**< HWVERS_USBHS_HWVERS Mask */

/**@} end of group USBHS_HWVERS_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_EPINFO USBHS_EPINFO
 * @brief    Endpoint hardware information.
 * @{
 */
#define MXC_F_USBHS_EPINFO_OUTENDPOINTS_POS            4 /**< EPINFO_OUTENDPOINTS Position */
#define MXC_F_USBHS_EPINFO_OUTENDPOINTS                ((uint8_t)(0xFUL << MXC_F_USBHS_EPINFO_OUTENDPOINTS_POS)) /**< EPINFO_OUTENDPOINTS Mask */

#define MXC_F_USBHS_EPINFO_INTENDPOINTS_POS            0 /**< EPINFO_INTENDPOINTS Position */
#define MXC_F_USBHS_EPINFO_INTENDPOINTS                ((uint8_t)(0xFUL << MXC_F_USBHS_EPINFO_INTENDPOINTS_POS)) /**< EPINFO_INTENDPOINTS Mask */

/**@} end of group USBHS_EPINFO_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_RAMINFO USBHS_RAMINFO
 * @brief    RAM width information.
 * @{
 */
#define MXC_F_USBHS_RAMINFO_RAMBITS_POS                0 /**< RAMINFO_RAMBITS Position */
#define MXC_F_USBHS_RAMINFO_RAMBITS                    ((uint8_t)(0xFUL << MXC_F_USBHS_RAMINFO_RAMBITS_POS)) /**< RAMINFO_RAMBITS Mask */

/**@} end of group USBHS_RAMINFO_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_SOFTRESET USBHS_SOFTRESET
 * @brief    Software reset register.
 * @{
 */
#define MXC_F_USBHS_SOFTRESET_RSTXS_POS                1 /**< SOFTRESET_RSTXS Position */
#define MXC_F_USBHS_SOFTRESET_RSTXS                    ((uint8_t)(0x1UL << MXC_F_USBHS_SOFTRESET_RSTXS_POS)) /**< SOFTRESET_RSTXS Mask */

#define MXC_F_USBHS_SOFTRESET_RSTS_POS                 0 /**< SOFTRESET_RSTS Position */
#define MXC_F_USBHS_SOFTRESET_RSTS                     ((uint8_t)(0x1UL << MXC_F_USBHS_SOFTRESET_RSTS_POS)) /**< SOFTRESET_RSTS Mask */

/**@} end of group USBHS_SOFTRESET_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_CTUCH USBHS_CTUCH
 * @brief    Chirp timeout timer setting.
 * @{
 */
#define MXC_F_USBHS_CTUCH_C_T_UCH_POS                  0 /**< CTUCH_C_T_UCH Position */
#define MXC_F_USBHS_CTUCH_C_T_UCH                      ((uint16_t)(0xFFFFUL << MXC_F_USBHS_CTUCH_C_T_UCH_POS)) /**< CTUCH_C_T_UCH Mask */

/**@} end of group USBHS_CTUCH_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_CTHSRTN USBHS_CTHSRTN
 * @brief    Sets delay between HS resume to UTM normal operating mode.
 * @{
 */
#define MXC_F_USBHS_CTHSRTN_C_T_HSTRN_POS              0 /**< CTHSRTN_C_T_HSTRN Position */
#define MXC_F_USBHS_CTHSRTN_C_T_HSTRN                  ((uint16_t)(0xFFFFUL << MXC_F_USBHS_CTHSRTN_C_T_HSTRN_POS)) /**< CTHSRTN_C_T_HSTRN Mask */

/**@} end of group USBHS_CTHSRTN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_MXM_INT USBHS_MXM_INT
 * @brief    USB Added Maxim Interrupt Flag Register.
 * @{
 */
#define MXC_F_USBHS_MXM_INT_VBUS_POS                   0 /**< MXM_INT_VBUS Position */
#define MXC_F_USBHS_MXM_INT_VBUS                       ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_INT_VBUS_POS)) /**< MXM_INT_VBUS Mask */

#define MXC_F_USBHS_MXM_INT_NOVBUS_POS                 1 /**< MXM_INT_NOVBUS Position */
#define MXC_F_USBHS_MXM_INT_NOVBUS                     ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_INT_NOVBUS_POS)) /**< MXM_INT_NOVBUS Mask */

/**@} end of group USBHS_MXM_INT_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_MXM_INT_EN USBHS_MXM_INT_EN
 * @brief    USB Added Maxim Interrupt Enable Register.
 * @{
 */
#define MXC_F_USBHS_MXM_INT_EN_VBUS_POS                0 /**< MXM_INT_EN_VBUS Position */
#define MXC_F_USBHS_MXM_INT_EN_VBUS                    ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_INT_EN_VBUS_POS)) /**< MXM_INT_EN_VBUS Mask */

#define MXC_F_USBHS_MXM_INT_EN_NOVBUS_POS              1 /**< MXM_INT_EN_NOVBUS Position */
#define MXC_F_USBHS_MXM_INT_EN_NOVBUS                  ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_INT_EN_NOVBUS_POS)) /**< MXM_INT_EN_NOVBUS Mask */

/**@} end of group USBHS_MXM_INT_EN_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_MXM_SUSPEND USBHS_MXM_SUSPEND
 * @brief    USB Added Maxim Suspend Register.
 * @{
 */
#define MXC_F_USBHS_MXM_SUSPEND_SEL_POS                0 /**< MXM_SUSPEND_SEL Position */
#define MXC_F_USBHS_MXM_SUSPEND_SEL                    ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_SUSPEND_SEL_POS)) /**< MXM_SUSPEND_SEL Mask */

/**@} end of group USBHS_MXM_SUSPEND_Register */

/**
 * @ingroup  usbhs_registers
 * @defgroup USBHS_MXM_REG_A4 USBHS_MXM_REG_A4
 * @brief    USB Added Maxim Power Status Register
 * @{
 */
#define MXC_F_USBHS_MXM_REG_A4_VRST_VDDB_N_A_POS       0 /**< MXM_REG_A4_VRST_VDDB_N_A Position */
#define MXC_F_USBHS_MXM_REG_A4_VRST_VDDB_N_A           ((uint32_t)(0x1UL << MXC_F_USBHS_MXM_REG_A4_VRST_VDDB_N_A_POS)) /**< MXM_REG_A4_VRST_VDDB_N_A Mask */

/**@} end of group USBHS_MXM_REG_A4_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_USBHS_REGS_H_
