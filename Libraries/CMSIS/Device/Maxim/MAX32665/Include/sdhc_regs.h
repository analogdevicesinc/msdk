/**
 * @file    sdhc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SDHC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sdhc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SDHC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SDHC_REGS_H_

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
 * @ingroup     sdhc
 * @defgroup    sdhc_registers SDHC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SDHC Peripheral Module.
 * @details     SDHC/SDIO Controller
 */

/**
 * @ingroup sdhc_registers
 * Structure type to access the SDHC Registers.
 */
typedef struct {
    __IO uint32_t sdma;                 /**< <tt>\b 0x00:</tt> SDHC SDMA Register */
    __IO uint16_t blk_size;             /**< <tt>\b 0x04:</tt> SDHC BLK_SIZE Register */
    __IO uint16_t blk_cnt;              /**< <tt>\b 0x06:</tt> SDHC BLK_CNT Register */
    __IO uint32_t arg_1;                /**< <tt>\b 0x08:</tt> SDHC ARG_1 Register */
    __IO uint16_t trans;                /**< <tt>\b 0x0C:</tt> SDHC TRANS Register */
    __IO uint16_t cmd;                  /**< <tt>\b 0x0E:</tt> SDHC CMD Register */
    __IO uint32_t resp[4];              /**< <tt>\b 0x010:</tt> SDHC RESP Register */
    __IO uint32_t buffer;               /**< <tt>\b 0x20:</tt> SDHC BUFFER Register */
    __I  uint32_t present;              /**< <tt>\b 0x024:</tt> SDHC PRESENT Register */
    __IO uint8_t  host_cn_1;            /**< <tt>\b 0x028:</tt> SDHC HOST_CN_1 Register */
    __IO uint8_t  pwr;                  /**< <tt>\b 0x029:</tt> SDHC PWR Register */
    __IO uint8_t  blk_gap;              /**< <tt>\b 0x02A:</tt> SDHC BLK_GAP Register */
    __IO uint8_t  wakeup;               /**< <tt>\b 0x02B:</tt> SDHC WAKEUP Register */
    __IO uint16_t clk_cn;               /**< <tt>\b 0x02C:</tt> SDHC CLK_CN Register */
    __IO uint8_t  to;                   /**< <tt>\b 0x02E:</tt> SDHC TO Register */
    __IO uint8_t  sw_reset;             /**< <tt>\b 0x02F:</tt> SDHC SW_RESET Register */
    __IO uint16_t int_stat;             /**< <tt>\b 0x030:</tt> SDHC INT_STAT Register */
    __IO uint16_t er_int_stat;          /**< <tt>\b 0x032:</tt> SDHC ER_INT_STAT Register */
    __IO uint16_t int_en;               /**< <tt>\b 0x034:</tt> SDHC INT_EN Register */
    __IO uint16_t er_int_en;            /**< <tt>\b 0x36:</tt> SDHC ER_INT_EN Register */
    __IO uint16_t int_signal;           /**< <tt>\b 0x038:</tt> SDHC INT_SIGNAL Register */
    __IO uint16_t er_int_signal;        /**< <tt>\b 0x03A:</tt> SDHC ER_INT_SIGNAL Register */
    __IO uint16_t auto_cmd_er;          /**< <tt>\b 0x03C:</tt> SDHC AUTO_CMD_ER Register */
    __IO uint16_t host_cn_2;            /**< <tt>\b 0x03E:</tt> SDHC HOST_CN_2 Register */
    __I  uint32_t cfg_0;                /**< <tt>\b 0x040:</tt> SDHC CFG_0 Register */
    __I  uint32_t cfg_1;                /**< <tt>\b 0x044:</tt> SDHC CFG_1 Register */
    __I  uint32_t max_curr_cfg;         /**< <tt>\b 0x048:</tt> SDHC MAX_CURR_CFG Register */
    __R  uint32_t rsv_0x4c;
    __O  uint16_t force_cmd;            /**< <tt>\b 0x050:</tt> SDHC FORCE_CMD Register */
    __IO uint16_t force_event_int_stat; /**< <tt>\b 0x052:</tt> SDHC FORCE_EVENT_INT_STAT Register */
    __IO uint8_t  adma_er;              /**< <tt>\b 0x054:</tt> SDHC ADMA_ER Register */
    __R  uint8_t  rsv_0x55_0x57[3];
    __IO uint32_t adma_addr_0;          /**< <tt>\b 0x058:</tt> SDHC ADMA_ADDR_0 Register */
    __IO uint32_t adma_addr_1;          /**< <tt>\b 0x05C:</tt> SDHC ADMA_ADDR_1 Register */
    __I  uint16_t preset_0;             /**< <tt>\b 0x060:</tt> SDHC PRESET_0 Register */
    __I  uint16_t preset_1;             /**< <tt>\b 0x062:</tt> SDHC PRESET_1 Register */
    __I  uint16_t preset_2;             /**< <tt>\b 0x064:</tt> SDHC PRESET_2 Register */
    __I  uint16_t preset_3;             /**< <tt>\b 0x066:</tt> SDHC PRESET_3 Register */
    __I  uint16_t preset_4;             /**< <tt>\b 0x068:</tt> SDHC PRESET_4 Register */
    __I  uint16_t preset_5;             /**< <tt>\b 0x06A:</tt> SDHC PRESET_5 Register */
    __I  uint16_t preset_6;             /**< <tt>\b 0x06C:</tt> SDHC PRESET_6 Register */
    __I  uint16_t preset_7;             /**< <tt>\b 0x06E:</tt> SDHC PRESET_7 Register */
    __R  uint32_t rsv_0x70_0xfb[35];
    __I  uint16_t slot_int;             /**< <tt>\b 0x0FC:</tt> SDHC SLOT_INT Register */
    __IO uint16_t host_cn_ver;          /**< <tt>\b 0x0FE:</tt> SDHC HOST_CN_VER Register */
} mxc_sdhc_regs_t;

/* Register offsets for module SDHC */
/**
 * @ingroup    sdhc_registers
 * @defgroup   SDHC_Register_Offsets Register Offsets
 * @brief      SDHC Peripheral Register Offsets from the SDHC Base Peripheral Address.
 * @{
 */
#define MXC_R_SDHC_SDMA                    ((uint32_t)0x00000000UL) /**< Offset from SDHC Base Address: <tt> 0x0000</tt> */
#define MXC_R_SDHC_BLK_SIZE                ((uint32_t)0x00000004UL) /**< Offset from SDHC Base Address: <tt> 0x0004</tt> */
#define MXC_R_SDHC_BLK_CNT                 ((uint32_t)0x00000006UL) /**< Offset from SDHC Base Address: <tt> 0x0006</tt> */
#define MXC_R_SDHC_ARG_1                   ((uint32_t)0x00000008UL) /**< Offset from SDHC Base Address: <tt> 0x0008</tt> */
#define MXC_R_SDHC_TRANS                   ((uint32_t)0x0000000CUL) /**< Offset from SDHC Base Address: <tt> 0x000C</tt> */
#define MXC_R_SDHC_CMD                     ((uint32_t)0x0000000EUL) /**< Offset from SDHC Base Address: <tt> 0x000E</tt> */
#define MXC_R_SDHC_RESP                    ((uint32_t)0x00000010UL) /**< Offset from SDHC Base Address: <tt> 0x0010</tt> */
#define MXC_R_SDHC_BUFFER                  ((uint32_t)0x00000020UL) /**< Offset from SDHC Base Address: <tt> 0x0020</tt> */
#define MXC_R_SDHC_PRESENT                 ((uint32_t)0x00000024UL) /**< Offset from SDHC Base Address: <tt> 0x0024</tt> */
#define MXC_R_SDHC_HOST_CN_1               ((uint32_t)0x00000028UL) /**< Offset from SDHC Base Address: <tt> 0x0028</tt> */
#define MXC_R_SDHC_PWR                     ((uint32_t)0x00000029UL) /**< Offset from SDHC Base Address: <tt> 0x0029</tt> */
#define MXC_R_SDHC_BLK_GAP                 ((uint32_t)0x0000002AUL) /**< Offset from SDHC Base Address: <tt> 0x002A</tt> */
#define MXC_R_SDHC_WAKEUP                  ((uint32_t)0x0000002BUL) /**< Offset from SDHC Base Address: <tt> 0x002B</tt> */
#define MXC_R_SDHC_CLK_CN                  ((uint32_t)0x0000002CUL) /**< Offset from SDHC Base Address: <tt> 0x002C</tt> */
#define MXC_R_SDHC_TO                      ((uint32_t)0x0000002EUL) /**< Offset from SDHC Base Address: <tt> 0x002E</tt> */
#define MXC_R_SDHC_SW_RESET                ((uint32_t)0x0000002FUL) /**< Offset from SDHC Base Address: <tt> 0x002F</tt> */
#define MXC_R_SDHC_INT_STAT                ((uint32_t)0x00000030UL) /**< Offset from SDHC Base Address: <tt> 0x0030</tt> */
#define MXC_R_SDHC_ER_INT_STAT             ((uint32_t)0x00000032UL) /**< Offset from SDHC Base Address: <tt> 0x0032</tt> */
#define MXC_R_SDHC_INT_EN                  ((uint32_t)0x00000034UL) /**< Offset from SDHC Base Address: <tt> 0x0034</tt> */
#define MXC_R_SDHC_ER_INT_EN               ((uint32_t)0x00000036UL) /**< Offset from SDHC Base Address: <tt> 0x0036</tt> */
#define MXC_R_SDHC_INT_SIGNAL              ((uint32_t)0x00000038UL) /**< Offset from SDHC Base Address: <tt> 0x0038</tt> */
#define MXC_R_SDHC_ER_INT_SIGNAL           ((uint32_t)0x0000003AUL) /**< Offset from SDHC Base Address: <tt> 0x003A</tt> */
#define MXC_R_SDHC_AUTO_CMD_ER             ((uint32_t)0x0000003CUL) /**< Offset from SDHC Base Address: <tt> 0x003C</tt> */
#define MXC_R_SDHC_HOST_CN_2               ((uint32_t)0x0000003EUL) /**< Offset from SDHC Base Address: <tt> 0x003E</tt> */
#define MXC_R_SDHC_CFG_0                   ((uint32_t)0x00000040UL) /**< Offset from SDHC Base Address: <tt> 0x0040</tt> */
#define MXC_R_SDHC_CFG_1                   ((uint32_t)0x00000044UL) /**< Offset from SDHC Base Address: <tt> 0x0044</tt> */
#define MXC_R_SDHC_MAX_CURR_CFG            ((uint32_t)0x00000048UL) /**< Offset from SDHC Base Address: <tt> 0x0048</tt> */
#define MXC_R_SDHC_FORCE_CMD               ((uint32_t)0x00000050UL) /**< Offset from SDHC Base Address: <tt> 0x0050</tt> */
#define MXC_R_SDHC_FORCE_EVENT_INT_STAT    ((uint32_t)0x00000052UL) /**< Offset from SDHC Base Address: <tt> 0x0052</tt> */
#define MXC_R_SDHC_ADMA_ER                 ((uint32_t)0x00000054UL) /**< Offset from SDHC Base Address: <tt> 0x0054</tt> */
#define MXC_R_SDHC_ADMA_ADDR_0             ((uint32_t)0x00000058UL) /**< Offset from SDHC Base Address: <tt> 0x0058</tt> */
#define MXC_R_SDHC_ADMA_ADDR_1             ((uint32_t)0x0000005CUL) /**< Offset from SDHC Base Address: <tt> 0x005C</tt> */
#define MXC_R_SDHC_PRESET_0                ((uint32_t)0x00000060UL) /**< Offset from SDHC Base Address: <tt> 0x0060</tt> */
#define MXC_R_SDHC_PRESET_1                ((uint32_t)0x00000062UL) /**< Offset from SDHC Base Address: <tt> 0x0062</tt> */
#define MXC_R_SDHC_PRESET_2                ((uint32_t)0x00000064UL) /**< Offset from SDHC Base Address: <tt> 0x0064</tt> */
#define MXC_R_SDHC_PRESET_3                ((uint32_t)0x00000066UL) /**< Offset from SDHC Base Address: <tt> 0x0066</tt> */
#define MXC_R_SDHC_PRESET_4                ((uint32_t)0x00000068UL) /**< Offset from SDHC Base Address: <tt> 0x0068</tt> */
#define MXC_R_SDHC_PRESET_5                ((uint32_t)0x0000006AUL) /**< Offset from SDHC Base Address: <tt> 0x006A</tt> */
#define MXC_R_SDHC_PRESET_6                ((uint32_t)0x0000006CUL) /**< Offset from SDHC Base Address: <tt> 0x006C</tt> */
#define MXC_R_SDHC_PRESET_7                ((uint32_t)0x0000006EUL) /**< Offset from SDHC Base Address: <tt> 0x006E</tt> */
#define MXC_R_SDHC_SLOT_INT                ((uint32_t)0x000000FCUL) /**< Offset from SDHC Base Address: <tt> 0x00FC</tt> */
#define MXC_R_SDHC_HOST_CN_VER             ((uint32_t)0x000000FEUL) /**< Offset from SDHC Base Address: <tt> 0x00FE</tt> */
/**@} end of group sdhc_registers */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_SDMA SDHC_SDMA
 * @brief    SDMA System Address / Argument 2.
 * @{
 */
#define MXC_F_SDHC_SDMA_ADDR_POS                       0 /**< SDMA_ADDR Position */
#define MXC_F_SDHC_SDMA_ADDR                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_SDMA_ADDR_POS)) /**< SDMA_ADDR Mask */

/**@} end of group SDHC_SDMA_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_BLK_SIZE SDHC_BLK_SIZE
 * @brief    Block Size.
 * @{
 */
#define MXC_F_SDHC_BLK_SIZE_TRANS_POS                  0 /**< BLK_SIZE_TRANS Position */
#define MXC_F_SDHC_BLK_SIZE_TRANS                      ((uint16_t)(0xFFFUL << MXC_F_SDHC_BLK_SIZE_TRANS_POS)) /**< BLK_SIZE_TRANS Mask */

#define MXC_F_SDHC_BLK_SIZE_HOST_BUFF_POS              12 /**< BLK_SIZE_HOST_BUFF Position */
#define MXC_F_SDHC_BLK_SIZE_HOST_BUFF                  ((uint16_t)(0x7UL << MXC_F_SDHC_BLK_SIZE_HOST_BUFF_POS)) /**< BLK_SIZE_HOST_BUFF Mask */

/**@} end of group SDHC_BLK_SIZE_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_BLK_CNT SDHC_BLK_CNT
 * @brief    Block Count.
 * @{
 */
#define MXC_F_SDHC_BLK_CNT_COUNT_POS                   0 /**< BLK_CNT_COUNT Position */
#define MXC_F_SDHC_BLK_CNT_COUNT                       ((uint16_t)(0xFFFFUL << MXC_F_SDHC_BLK_CNT_COUNT_POS)) /**< BLK_CNT_COUNT Mask */

/**@} end of group SDHC_BLK_CNT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ARG_1 SDHC_ARG_1
 * @brief    Argument 1.
 * @{
 */
#define MXC_F_SDHC_ARG_1_CMD_POS                       0 /**< ARG_1_CMD Position */
#define MXC_F_SDHC_ARG_1_CMD                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_ARG_1_CMD_POS)) /**< ARG_1_CMD Mask */

/**@} end of group SDHC_ARG_1_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_TRANS SDHC_TRANS
 * @brief    Transfer Mode.
 * @{
 */
#define MXC_F_SDHC_TRANS_DMA_EN_POS                    0 /**< TRANS_DMA_EN Position */
#define MXC_F_SDHC_TRANS_DMA_EN                        ((uint16_t)(0x1UL << MXC_F_SDHC_TRANS_DMA_EN_POS)) /**< TRANS_DMA_EN Mask */

#define MXC_F_SDHC_TRANS_BLK_CNT_EN_POS                1 /**< TRANS_BLK_CNT_EN Position */
#define MXC_F_SDHC_TRANS_BLK_CNT_EN                    ((uint16_t)(0x1UL << MXC_F_SDHC_TRANS_BLK_CNT_EN_POS)) /**< TRANS_BLK_CNT_EN Mask */

#define MXC_F_SDHC_TRANS_AUTO_CMD_EN_POS               2 /**< TRANS_AUTO_CMD_EN Position */
#define MXC_F_SDHC_TRANS_AUTO_CMD_EN                   ((uint16_t)(0x3UL << MXC_F_SDHC_TRANS_AUTO_CMD_EN_POS)) /**< TRANS_AUTO_CMD_EN Mask */
#define MXC_V_SDHC_TRANS_AUTO_CMD_EN_DISABLE           ((uint16_t)0x0UL) /**< TRANS_AUTO_CMD_EN_DISABLE Value */
#define MXC_S_SDHC_TRANS_AUTO_CMD_EN_DISABLE           (MXC_V_SDHC_TRANS_AUTO_CMD_EN_DISABLE << MXC_F_SDHC_TRANS_AUTO_CMD_EN_POS) /**< TRANS_AUTO_CMD_EN_DISABLE Setting */
#define MXC_V_SDHC_TRANS_AUTO_CMD_EN_CMD12             ((uint16_t)0x1UL) /**< TRANS_AUTO_CMD_EN_CMD12 Value */
#define MXC_S_SDHC_TRANS_AUTO_CMD_EN_CMD12             (MXC_V_SDHC_TRANS_AUTO_CMD_EN_CMD12 << MXC_F_SDHC_TRANS_AUTO_CMD_EN_POS) /**< TRANS_AUTO_CMD_EN_CMD12 Setting */
#define MXC_V_SDHC_TRANS_AUTO_CMD_EN_CMD23             ((uint16_t)0x2UL) /**< TRANS_AUTO_CMD_EN_CMD23 Value */
#define MXC_S_SDHC_TRANS_AUTO_CMD_EN_CMD23             (MXC_V_SDHC_TRANS_AUTO_CMD_EN_CMD23 << MXC_F_SDHC_TRANS_AUTO_CMD_EN_POS) /**< TRANS_AUTO_CMD_EN_CMD23 Setting */

#define MXC_F_SDHC_TRANS_READ_WRITE_POS                4 /**< TRANS_READ_WRITE Position */
#define MXC_F_SDHC_TRANS_READ_WRITE                    ((uint16_t)(0x1UL << MXC_F_SDHC_TRANS_READ_WRITE_POS)) /**< TRANS_READ_WRITE Mask */

#define MXC_F_SDHC_TRANS_MULTI_POS                     5 /**< TRANS_MULTI Position */
#define MXC_F_SDHC_TRANS_MULTI                         ((uint16_t)(0x1UL << MXC_F_SDHC_TRANS_MULTI_POS)) /**< TRANS_MULTI Mask */

/**@} end of group SDHC_TRANS_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_CMD SDHC_CMD
 * @brief    Command.
 * @{
 */
#define MXC_F_SDHC_CMD_RESP_TYPE_POS                   0 /**< CMD_RESP_TYPE Position */
#define MXC_F_SDHC_CMD_RESP_TYPE                       ((uint16_t)(0x3UL << MXC_F_SDHC_CMD_RESP_TYPE_POS)) /**< CMD_RESP_TYPE Mask */

#define MXC_F_SDHC_CMD_CRC_CHK_EN_POS                  3 /**< CMD_CRC_CHK_EN Position */
#define MXC_F_SDHC_CMD_CRC_CHK_EN                      ((uint16_t)(0x1UL << MXC_F_SDHC_CMD_CRC_CHK_EN_POS)) /**< CMD_CRC_CHK_EN Mask */

#define MXC_F_SDHC_CMD_IDX_CHK_EN_POS                  4 /**< CMD_IDX_CHK_EN Position */
#define MXC_F_SDHC_CMD_IDX_CHK_EN                      ((uint16_t)(0x1UL << MXC_F_SDHC_CMD_IDX_CHK_EN_POS)) /**< CMD_IDX_CHK_EN Mask */

#define MXC_F_SDHC_CMD_DATA_PRES_SEL_POS               5 /**< CMD_DATA_PRES_SEL Position */
#define MXC_F_SDHC_CMD_DATA_PRES_SEL                   ((uint16_t)(0x1UL << MXC_F_SDHC_CMD_DATA_PRES_SEL_POS)) /**< CMD_DATA_PRES_SEL Mask */

#define MXC_F_SDHC_CMD_TYPE_POS                        6 /**< CMD_TYPE Position */
#define MXC_F_SDHC_CMD_TYPE                            ((uint16_t)(0x3UL << MXC_F_SDHC_CMD_TYPE_POS)) /**< CMD_TYPE Mask */

#define MXC_F_SDHC_CMD_IDX_POS                         8 /**< CMD_IDX Position */
#define MXC_F_SDHC_CMD_IDX                             ((uint16_t)(0x3FUL << MXC_F_SDHC_CMD_IDX_POS)) /**< CMD_IDX Mask */

/**@} end of group SDHC_CMD_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_RESP SDHC_RESP
 * @brief    Response 0 Register 0-15.
 * @{
 */
#define MXC_F_SDHC_RESP_CMD_RESP_POS                   0 /**< RESP_CMD_RESP Position */
#define MXC_F_SDHC_RESP_CMD_RESP                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_RESP_CMD_RESP_POS)) /**< RESP_CMD_RESP Mask */

/**@} end of group SDHC_RESP_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_BUFFER SDHC_BUFFER
 * @brief    Buffer Data Port.
 * @{
 */
#define MXC_F_SDHC_BUFFER_DATA_POS                     0 /**< BUFFER_DATA Position */
#define MXC_F_SDHC_BUFFER_DATA                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_BUFFER_DATA_POS)) /**< BUFFER_DATA Mask */

/**@} end of group SDHC_BUFFER_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESENT SDHC_PRESENT
 * @brief    Present State.
 * @{
 */
#define MXC_F_SDHC_PRESENT_CMD_POS                     0 /**< PRESENT_CMD Position */
#define MXC_F_SDHC_PRESENT_CMD                         ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_CMD_POS)) /**< PRESENT_CMD Mask */

#define MXC_F_SDHC_PRESENT_DAT_POS                     1 /**< PRESENT_DAT Position */
#define MXC_F_SDHC_PRESENT_DAT                         ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_DAT_POS)) /**< PRESENT_DAT Mask */

#define MXC_F_SDHC_PRESENT_DAT_LINE_ACTIVE_POS         2 /**< PRESENT_DAT_LINE_ACTIVE Position */
#define MXC_F_SDHC_PRESENT_DAT_LINE_ACTIVE             ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_DAT_LINE_ACTIVE_POS)) /**< PRESENT_DAT_LINE_ACTIVE Mask */

#define MXC_F_SDHC_PRESENT_RETUNING_POS                3 /**< PRESENT_RETUNING Position */
#define MXC_F_SDHC_PRESENT_RETUNING                    ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_RETUNING_POS)) /**< PRESENT_RETUNING Mask */

#define MXC_F_SDHC_PRESENT_WRITE_TRANSFER_POS          8 /**< PRESENT_WRITE_TRANSFER Position */
#define MXC_F_SDHC_PRESENT_WRITE_TRANSFER              ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_WRITE_TRANSFER_POS)) /**< PRESENT_WRITE_TRANSFER Mask */

#define MXC_F_SDHC_PRESENT_READ_TRANSFER_POS           9 /**< PRESENT_READ_TRANSFER Position */
#define MXC_F_SDHC_PRESENT_READ_TRANSFER               ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_READ_TRANSFER_POS)) /**< PRESENT_READ_TRANSFER Mask */

#define MXC_F_SDHC_PRESENT_BUFFER_WRITE_POS            10 /**< PRESENT_BUFFER_WRITE Position */
#define MXC_F_SDHC_PRESENT_BUFFER_WRITE                ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_BUFFER_WRITE_POS)) /**< PRESENT_BUFFER_WRITE Mask */

#define MXC_F_SDHC_PRESENT_BUFFER_READ_POS             11 /**< PRESENT_BUFFER_READ Position */
#define MXC_F_SDHC_PRESENT_BUFFER_READ                 ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_BUFFER_READ_POS)) /**< PRESENT_BUFFER_READ Mask */

#define MXC_F_SDHC_PRESENT_CARD_INSERTED_POS           16 /**< PRESENT_CARD_INSERTED Position */
#define MXC_F_SDHC_PRESENT_CARD_INSERTED               ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_CARD_INSERTED_POS)) /**< PRESENT_CARD_INSERTED Mask */

#define MXC_F_SDHC_PRESENT_CARD_STATE_POS              17 /**< PRESENT_CARD_STATE Position */
#define MXC_F_SDHC_PRESENT_CARD_STATE                  ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_CARD_STATE_POS)) /**< PRESENT_CARD_STATE Mask */

#define MXC_F_SDHC_PRESENT_CARD_DETECT_POS             18 /**< PRESENT_CARD_DETECT Position */
#define MXC_F_SDHC_PRESENT_CARD_DETECT                 ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_CARD_DETECT_POS)) /**< PRESENT_CARD_DETECT Mask */

#define MXC_F_SDHC_PRESENT_WP_POS                      19 /**< PRESENT_WP Position */
#define MXC_F_SDHC_PRESENT_WP                          ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_WP_POS)) /**< PRESENT_WP Mask */

#define MXC_F_SDHC_PRESENT_DAT_SIGNAL_LEVEL_POS        20 /**< PRESENT_DAT_SIGNAL_LEVEL Position */
#define MXC_F_SDHC_PRESENT_DAT_SIGNAL_LEVEL            ((uint32_t)(0xFUL << MXC_F_SDHC_PRESENT_DAT_SIGNAL_LEVEL_POS)) /**< PRESENT_DAT_SIGNAL_LEVEL Mask */

#define MXC_F_SDHC_PRESENT_CMD_SIGNAL_LEVEL_POS        24 /**< PRESENT_CMD_SIGNAL_LEVEL Position */
#define MXC_F_SDHC_PRESENT_CMD_SIGNAL_LEVEL            ((uint32_t)(0x1UL << MXC_F_SDHC_PRESENT_CMD_SIGNAL_LEVEL_POS)) /**< PRESENT_CMD_SIGNAL_LEVEL Mask */

/**@} end of group SDHC_PRESENT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_HOST_CN_1 SDHC_HOST_CN_1
 * @brief    Host Control 1.
 * @{
 */
#define MXC_F_SDHC_HOST_CN_1_LED_CN_POS                0 /**< HOST_CN_1_LED_CN Position */
#define MXC_F_SDHC_HOST_CN_1_LED_CN                    ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_LED_CN_POS)) /**< HOST_CN_1_LED_CN Mask */

#define MXC_F_SDHC_HOST_CN_1_DATA_TRANSFER_WIDTH_POS   1 /**< HOST_CN_1_DATA_TRANSFER_WIDTH Position */
#define MXC_F_SDHC_HOST_CN_1_DATA_TRANSFER_WIDTH       ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_DATA_TRANSFER_WIDTH_POS)) /**< HOST_CN_1_DATA_TRANSFER_WIDTH Mask */

#define MXC_F_SDHC_HOST_CN_1_HS_EN_POS                 2 /**< HOST_CN_1_HS_EN Position */
#define MXC_F_SDHC_HOST_CN_1_HS_EN                     ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_HS_EN_POS)) /**< HOST_CN_1_HS_EN Mask */

#define MXC_F_SDHC_HOST_CN_1_DMA_SELECT_POS            3 /**< HOST_CN_1_DMA_SELECT Position */
#define MXC_F_SDHC_HOST_CN_1_DMA_SELECT                ((uint8_t)(0x3UL << MXC_F_SDHC_HOST_CN_1_DMA_SELECT_POS)) /**< HOST_CN_1_DMA_SELECT Mask */

#define MXC_F_SDHC_HOST_CN_1_EXT_DATA_TRANSFER_WIDTH_POS 5 /**< HOST_CN_1_EXT_DATA_TRANSFER_WIDTH Position */
#define MXC_F_SDHC_HOST_CN_1_EXT_DATA_TRANSFER_WIDTH   ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_EXT_DATA_TRANSFER_WIDTH_POS)) /**< HOST_CN_1_EXT_DATA_TRANSFER_WIDTH Mask */

#define MXC_F_SDHC_HOST_CN_1_CARD_DETECT_TEST_POS      6 /**< HOST_CN_1_CARD_DETECT_TEST Position */
#define MXC_F_SDHC_HOST_CN_1_CARD_DETECT_TEST          ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_CARD_DETECT_TEST_POS)) /**< HOST_CN_1_CARD_DETECT_TEST Mask */

#define MXC_F_SDHC_HOST_CN_1_CARD_DETECT_SIGNAL_POS    7 /**< HOST_CN_1_CARD_DETECT_SIGNAL Position */
#define MXC_F_SDHC_HOST_CN_1_CARD_DETECT_SIGNAL        ((uint8_t)(0x1UL << MXC_F_SDHC_HOST_CN_1_CARD_DETECT_SIGNAL_POS)) /**< HOST_CN_1_CARD_DETECT_SIGNAL Mask */

/**@} end of group SDHC_HOST_CN_1_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PWR SDHC_PWR
 * @brief    Power Control.
 * @{
 */
#define MXC_F_SDHC_PWR_BUS_POWER_POS                   0 /**< PWR_BUS_POWER Position */
#define MXC_F_SDHC_PWR_BUS_POWER                       ((uint8_t)(0x1UL << MXC_F_SDHC_PWR_BUS_POWER_POS)) /**< PWR_BUS_POWER Mask */

#define MXC_F_SDHC_PWR_BUS_VOLT_SEL_POS                1 /**< PWR_BUS_VOLT_SEL Position */
#define MXC_F_SDHC_PWR_BUS_VOLT_SEL                    ((uint8_t)(0x7UL << MXC_F_SDHC_PWR_BUS_VOLT_SEL_POS)) /**< PWR_BUS_VOLT_SEL Mask */

/**@} end of group SDHC_PWR_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_BLK_GAP SDHC_BLK_GAP
 * @brief    Block Gap Control.
 * @{
 */
#define MXC_F_SDHC_BLK_GAP_STOP_POS                    0 /**< BLK_GAP_STOP Position */
#define MXC_F_SDHC_BLK_GAP_STOP                        ((uint8_t)(0x1UL << MXC_F_SDHC_BLK_GAP_STOP_POS)) /**< BLK_GAP_STOP Mask */

#define MXC_F_SDHC_BLK_GAP_CONT_POS                    1 /**< BLK_GAP_CONT Position */
#define MXC_F_SDHC_BLK_GAP_CONT                        ((uint8_t)(0x1UL << MXC_F_SDHC_BLK_GAP_CONT_POS)) /**< BLK_GAP_CONT Mask */

#define MXC_F_SDHC_BLK_GAP_READ_WAIT_POS               2 /**< BLK_GAP_READ_WAIT Position */
#define MXC_F_SDHC_BLK_GAP_READ_WAIT                   ((uint8_t)(0x1UL << MXC_F_SDHC_BLK_GAP_READ_WAIT_POS)) /**< BLK_GAP_READ_WAIT Mask */

#define MXC_F_SDHC_BLK_GAP_INTR_POS                    3 /**< BLK_GAP_INTR Position */
#define MXC_F_SDHC_BLK_GAP_INTR                        ((uint8_t)(0x1UL << MXC_F_SDHC_BLK_GAP_INTR_POS)) /**< BLK_GAP_INTR Mask */

/**@} end of group SDHC_BLK_GAP_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_WAKEUP SDHC_WAKEUP
 * @brief    Wakeup Control.
 * @{
 */
#define MXC_F_SDHC_WAKEUP_CARD_INT_POS                 0 /**< WAKEUP_CARD_INT Position */
#define MXC_F_SDHC_WAKEUP_CARD_INT                     ((uint8_t)(0x1UL << MXC_F_SDHC_WAKEUP_CARD_INT_POS)) /**< WAKEUP_CARD_INT Mask */

#define MXC_F_SDHC_WAKEUP_CARD_INS_POS                 1 /**< WAKEUP_CARD_INS Position */
#define MXC_F_SDHC_WAKEUP_CARD_INS                     ((uint8_t)(0x1UL << MXC_F_SDHC_WAKEUP_CARD_INS_POS)) /**< WAKEUP_CARD_INS Mask */

#define MXC_F_SDHC_WAKEUP_CARD_REM_POS                 2 /**< WAKEUP_CARD_REM Position */
#define MXC_F_SDHC_WAKEUP_CARD_REM                     ((uint8_t)(0x1UL << MXC_F_SDHC_WAKEUP_CARD_REM_POS)) /**< WAKEUP_CARD_REM Mask */

/**@} end of group SDHC_WAKEUP_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_CLK_CN SDHC_CLK_CN
 * @brief    Clock Control.
 * @{
 */
#define MXC_F_SDHC_CLK_CN_INTERNAL_CLK_EN_POS          0 /**< CLK_CN_INTERNAL_CLK_EN Position */
#define MXC_F_SDHC_CLK_CN_INTERNAL_CLK_EN              ((uint16_t)(0x1UL << MXC_F_SDHC_CLK_CN_INTERNAL_CLK_EN_POS)) /**< CLK_CN_INTERNAL_CLK_EN Mask */

#define MXC_F_SDHC_CLK_CN_INTERNAL_CLK_STABLE_POS      1 /**< CLK_CN_INTERNAL_CLK_STABLE Position */
#define MXC_F_SDHC_CLK_CN_INTERNAL_CLK_STABLE          ((uint16_t)(0x1UL << MXC_F_SDHC_CLK_CN_INTERNAL_CLK_STABLE_POS)) /**< CLK_CN_INTERNAL_CLK_STABLE Mask */

#define MXC_F_SDHC_CLK_CN_SD_CLK_EN_POS                2 /**< CLK_CN_SD_CLK_EN Position */
#define MXC_F_SDHC_CLK_CN_SD_CLK_EN                    ((uint16_t)(0x1UL << MXC_F_SDHC_CLK_CN_SD_CLK_EN_POS)) /**< CLK_CN_SD_CLK_EN Mask */

#define MXC_F_SDHC_CLK_CN_CLK_GEN_SEL_POS              5 /**< CLK_CN_CLK_GEN_SEL Position */
#define MXC_F_SDHC_CLK_CN_CLK_GEN_SEL                  ((uint16_t)(0x1UL << MXC_F_SDHC_CLK_CN_CLK_GEN_SEL_POS)) /**< CLK_CN_CLK_GEN_SEL Mask */

#define MXC_F_SDHC_CLK_CN_UPPER_SDCLK_FREQ_SEL_POS     6 /**< CLK_CN_UPPER_SDCLK_FREQ_SEL Position */
#define MXC_F_SDHC_CLK_CN_UPPER_SDCLK_FREQ_SEL         ((uint16_t)(0x3UL << MXC_F_SDHC_CLK_CN_UPPER_SDCLK_FREQ_SEL_POS)) /**< CLK_CN_UPPER_SDCLK_FREQ_SEL Mask */

#define MXC_F_SDHC_CLK_CN_SDCLK_FREQ_SEL_POS           8 /**< CLK_CN_SDCLK_FREQ_SEL Position */
#define MXC_F_SDHC_CLK_CN_SDCLK_FREQ_SEL               ((uint16_t)(0xFFUL << MXC_F_SDHC_CLK_CN_SDCLK_FREQ_SEL_POS)) /**< CLK_CN_SDCLK_FREQ_SEL Mask */

/**@} end of group SDHC_CLK_CN_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_TO SDHC_TO
 * @brief    Timeout Control.
 * @{
 */
#define MXC_F_SDHC_TO_DATA_COUNT_VALUE_POS             0 /**< TO_DATA_COUNT_VALUE Position */
#define MXC_F_SDHC_TO_DATA_COUNT_VALUE                 ((uint8_t)(0x7UL << MXC_F_SDHC_TO_DATA_COUNT_VALUE_POS)) /**< TO_DATA_COUNT_VALUE Mask */

/**@} end of group SDHC_TO_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_SW_RESET SDHC_SW_RESET
 * @brief    Software Reset.
 * @{
 */
#define MXC_F_SDHC_SW_RESET_RESET_ALL_POS              0 /**< SW_RESET_RESET_ALL Position */
#define MXC_F_SDHC_SW_RESET_RESET_ALL                  ((uint8_t)(0x1UL << MXC_F_SDHC_SW_RESET_RESET_ALL_POS)) /**< SW_RESET_RESET_ALL Mask */

#define MXC_F_SDHC_SW_RESET_RESET_CMD_POS              1 /**< SW_RESET_RESET_CMD Position */
#define MXC_F_SDHC_SW_RESET_RESET_CMD                  ((uint8_t)(0x1UL << MXC_F_SDHC_SW_RESET_RESET_CMD_POS)) /**< SW_RESET_RESET_CMD Mask */

#define MXC_F_SDHC_SW_RESET_RESET_DAT_POS              2 /**< SW_RESET_RESET_DAT Position */
#define MXC_F_SDHC_SW_RESET_RESET_DAT                  ((uint8_t)(0x1UL << MXC_F_SDHC_SW_RESET_RESET_DAT_POS)) /**< SW_RESET_RESET_DAT Mask */

/**@} end of group SDHC_SW_RESET_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_INT_STAT SDHC_INT_STAT
 * @brief    Normal Interrupt Status.
 * @{
 */
#define MXC_F_SDHC_INT_STAT_CMD_COMP_POS               0 /**< INT_STAT_CMD_COMP Position */
#define MXC_F_SDHC_INT_STAT_CMD_COMP                   ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_CMD_COMP_POS)) /**< INT_STAT_CMD_COMP Mask */

#define MXC_F_SDHC_INT_STAT_TRANS_COMP_POS             1 /**< INT_STAT_TRANS_COMP Position */
#define MXC_F_SDHC_INT_STAT_TRANS_COMP                 ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_TRANS_COMP_POS)) /**< INT_STAT_TRANS_COMP Mask */

#define MXC_F_SDHC_INT_STAT_BLK_GAP_EVENT_POS          2 /**< INT_STAT_BLK_GAP_EVENT Position */
#define MXC_F_SDHC_INT_STAT_BLK_GAP_EVENT              ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_BLK_GAP_EVENT_POS)) /**< INT_STAT_BLK_GAP_EVENT Mask */

#define MXC_F_SDHC_INT_STAT_DMA_POS                    3 /**< INT_STAT_DMA Position */
#define MXC_F_SDHC_INT_STAT_DMA                        ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_DMA_POS)) /**< INT_STAT_DMA Mask */

#define MXC_F_SDHC_INT_STAT_BUFF_WR_READY_POS          4 /**< INT_STAT_BUFF_WR_READY Position */
#define MXC_F_SDHC_INT_STAT_BUFF_WR_READY              ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_BUFF_WR_READY_POS)) /**< INT_STAT_BUFF_WR_READY Mask */

#define MXC_F_SDHC_INT_STAT_BUFF_RD_READY_POS          5 /**< INT_STAT_BUFF_RD_READY Position */
#define MXC_F_SDHC_INT_STAT_BUFF_RD_READY              ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_BUFF_RD_READY_POS)) /**< INT_STAT_BUFF_RD_READY Mask */

#define MXC_F_SDHC_INT_STAT_CARD_INSERTION_POS         6 /**< INT_STAT_CARD_INSERTION Position */
#define MXC_F_SDHC_INT_STAT_CARD_INSERTION             ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_CARD_INSERTION_POS)) /**< INT_STAT_CARD_INSERTION Mask */

#define MXC_F_SDHC_INT_STAT_CARD_REMOVAL_POS           7 /**< INT_STAT_CARD_REMOVAL Position */
#define MXC_F_SDHC_INT_STAT_CARD_REMOVAL               ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_CARD_REMOVAL_POS)) /**< INT_STAT_CARD_REMOVAL Mask */

#define MXC_F_SDHC_INT_STAT_CARD_INTR_POS              8 /**< INT_STAT_CARD_INTR Position */
#define MXC_F_SDHC_INT_STAT_CARD_INTR                  ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_CARD_INTR_POS)) /**< INT_STAT_CARD_INTR Mask */

#define MXC_F_SDHC_INT_STAT_RETUNING_POS               12 /**< INT_STAT_RETUNING Position */
#define MXC_F_SDHC_INT_STAT_RETUNING                   ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_RETUNING_POS)) /**< INT_STAT_RETUNING Mask */

#define MXC_F_SDHC_INT_STAT_ERR_INTR_POS               15 /**< INT_STAT_ERR_INTR Position */
#define MXC_F_SDHC_INT_STAT_ERR_INTR                   ((uint16_t)(0x1UL << MXC_F_SDHC_INT_STAT_ERR_INTR_POS)) /**< INT_STAT_ERR_INTR Mask */

/**@} end of group SDHC_INT_STAT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ER_INT_STAT SDHC_ER_INT_STAT
 * @brief    Error Interrupt Status.
 * @{
 */
#define MXC_F_SDHC_ER_INT_STAT_CMD_TO_POS              0 /**< ER_INT_STAT_CMD_TO Position */
#define MXC_F_SDHC_ER_INT_STAT_CMD_TO                  ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_CMD_TO_POS)) /**< ER_INT_STAT_CMD_TO Mask */

#define MXC_F_SDHC_ER_INT_STAT_CMD_CRC_POS             1 /**< ER_INT_STAT_CMD_CRC Position */
#define MXC_F_SDHC_ER_INT_STAT_CMD_CRC                 ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_CMD_CRC_POS)) /**< ER_INT_STAT_CMD_CRC Mask */

#define MXC_F_SDHC_ER_INT_STAT_CMD_END_BIT_POS         2 /**< ER_INT_STAT_CMD_END_BIT Position */
#define MXC_F_SDHC_ER_INT_STAT_CMD_END_BIT             ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_CMD_END_BIT_POS)) /**< ER_INT_STAT_CMD_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_STAT_CMD_IDX_POS             3 /**< ER_INT_STAT_CMD_IDX Position */
#define MXC_F_SDHC_ER_INT_STAT_CMD_IDX                 ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_CMD_IDX_POS)) /**< ER_INT_STAT_CMD_IDX Mask */

#define MXC_F_SDHC_ER_INT_STAT_DATA_TO_POS             4 /**< ER_INT_STAT_DATA_TO Position */
#define MXC_F_SDHC_ER_INT_STAT_DATA_TO                 ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_DATA_TO_POS)) /**< ER_INT_STAT_DATA_TO Mask */

#define MXC_F_SDHC_ER_INT_STAT_DATA_CRC_POS            5 /**< ER_INT_STAT_DATA_CRC Position */
#define MXC_F_SDHC_ER_INT_STAT_DATA_CRC                ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_DATA_CRC_POS)) /**< ER_INT_STAT_DATA_CRC Mask */

#define MXC_F_SDHC_ER_INT_STAT_DATA_END_BIT_POS        6 /**< ER_INT_STAT_DATA_END_BIT Position */
#define MXC_F_SDHC_ER_INT_STAT_DATA_END_BIT            ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_DATA_END_BIT_POS)) /**< ER_INT_STAT_DATA_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_STAT_CURRENT_LIMIT_POS       7 /**< ER_INT_STAT_CURRENT_LIMIT Position */
#define MXC_F_SDHC_ER_INT_STAT_CURRENT_LIMIT           ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_CURRENT_LIMIT_POS)) /**< ER_INT_STAT_CURRENT_LIMIT Mask */

#define MXC_F_SDHC_ER_INT_STAT_AUTO_CMD_12_POS         8 /**< ER_INT_STAT_AUTO_CMD_12 Position */
#define MXC_F_SDHC_ER_INT_STAT_AUTO_CMD_12             ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_AUTO_CMD_12_POS)) /**< ER_INT_STAT_AUTO_CMD_12 Mask */

#define MXC_F_SDHC_ER_INT_STAT_ADMA_POS                9 /**< ER_INT_STAT_ADMA Position */
#define MXC_F_SDHC_ER_INT_STAT_ADMA                    ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_ADMA_POS)) /**< ER_INT_STAT_ADMA Mask */

#define MXC_F_SDHC_ER_INT_STAT_DMA_POS                 12 /**< ER_INT_STAT_DMA Position */
#define MXC_F_SDHC_ER_INT_STAT_DMA                     ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_STAT_DMA_POS)) /**< ER_INT_STAT_DMA Mask */

/**@} end of group SDHC_ER_INT_STAT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_INT_EN SDHC_INT_EN
 * @brief    Normal Interrupt Status Enable.
 * @{
 */
#define MXC_F_SDHC_INT_EN_CMD_COMP_POS                 0 /**< INT_EN_CMD_COMP Position */
#define MXC_F_SDHC_INT_EN_CMD_COMP                     ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_CMD_COMP_POS)) /**< INT_EN_CMD_COMP Mask */

#define MXC_F_SDHC_INT_EN_TRANS_COMP_POS               1 /**< INT_EN_TRANS_COMP Position */
#define MXC_F_SDHC_INT_EN_TRANS_COMP                   ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_TRANS_COMP_POS)) /**< INT_EN_TRANS_COMP Mask */

#define MXC_F_SDHC_INT_EN_BLK_GAP_POS                  2 /**< INT_EN_BLK_GAP Position */
#define MXC_F_SDHC_INT_EN_BLK_GAP                      ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_BLK_GAP_POS)) /**< INT_EN_BLK_GAP Mask */

#define MXC_F_SDHC_INT_EN_DMA_POS                      3 /**< INT_EN_DMA Position */
#define MXC_F_SDHC_INT_EN_DMA                          ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_DMA_POS)) /**< INT_EN_DMA Mask */

#define MXC_F_SDHC_INT_EN_BUFFER_WR_POS                4 /**< INT_EN_BUFFER_WR Position */
#define MXC_F_SDHC_INT_EN_BUFFER_WR                    ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_BUFFER_WR_POS)) /**< INT_EN_BUFFER_WR Mask */

#define MXC_F_SDHC_INT_EN_BUFFER_RD_POS                5 /**< INT_EN_BUFFER_RD Position */
#define MXC_F_SDHC_INT_EN_BUFFER_RD                    ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_BUFFER_RD_POS)) /**< INT_EN_BUFFER_RD Mask */

#define MXC_F_SDHC_INT_EN_CARD_INSERT_POS              6 /**< INT_EN_CARD_INSERT Position */
#define MXC_F_SDHC_INT_EN_CARD_INSERT                  ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_CARD_INSERT_POS)) /**< INT_EN_CARD_INSERT Mask */

#define MXC_F_SDHC_INT_EN_CARD_REMOVAL_POS             7 /**< INT_EN_CARD_REMOVAL Position */
#define MXC_F_SDHC_INT_EN_CARD_REMOVAL                 ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_CARD_REMOVAL_POS)) /**< INT_EN_CARD_REMOVAL Mask */

#define MXC_F_SDHC_INT_EN_CARD_INT_POS                 8 /**< INT_EN_CARD_INT Position */
#define MXC_F_SDHC_INT_EN_CARD_INT                     ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_CARD_INT_POS)) /**< INT_EN_CARD_INT Mask */

#define MXC_F_SDHC_INT_EN_RETUNING_POS                 12 /**< INT_EN_RETUNING Position */
#define MXC_F_SDHC_INT_EN_RETUNING                     ((uint16_t)(0x1UL << MXC_F_SDHC_INT_EN_RETUNING_POS)) /**< INT_EN_RETUNING Mask */

/**@} end of group SDHC_INT_EN_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ER_INT_EN SDHC_ER_INT_EN
 * @brief    Error Interrupt Status Enable.
 * @{
 */
#define MXC_F_SDHC_ER_INT_EN_CMD_TO_POS                0 /**< ER_INT_EN_CMD_TO Position */
#define MXC_F_SDHC_ER_INT_EN_CMD_TO                    ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_CMD_TO_POS)) /**< ER_INT_EN_CMD_TO Mask */

#define MXC_F_SDHC_ER_INT_EN_CMD_CRC_POS               1 /**< ER_INT_EN_CMD_CRC Position */
#define MXC_F_SDHC_ER_INT_EN_CMD_CRC                   ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_CMD_CRC_POS)) /**< ER_INT_EN_CMD_CRC Mask */

#define MXC_F_SDHC_ER_INT_EN_CMD_END_BIT_POS           2 /**< ER_INT_EN_CMD_END_BIT Position */
#define MXC_F_SDHC_ER_INT_EN_CMD_END_BIT               ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_CMD_END_BIT_POS)) /**< ER_INT_EN_CMD_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_EN_CMD_IDX_POS               3 /**< ER_INT_EN_CMD_IDX Position */
#define MXC_F_SDHC_ER_INT_EN_CMD_IDX                   ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_CMD_IDX_POS)) /**< ER_INT_EN_CMD_IDX Mask */

#define MXC_F_SDHC_ER_INT_EN_DATA_TO_POS               4 /**< ER_INT_EN_DATA_TO Position */
#define MXC_F_SDHC_ER_INT_EN_DATA_TO                   ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_DATA_TO_POS)) /**< ER_INT_EN_DATA_TO Mask */

#define MXC_F_SDHC_ER_INT_EN_DATA_CRC_POS              5 /**< ER_INT_EN_DATA_CRC Position */
#define MXC_F_SDHC_ER_INT_EN_DATA_CRC                  ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_DATA_CRC_POS)) /**< ER_INT_EN_DATA_CRC Mask */

#define MXC_F_SDHC_ER_INT_EN_DATA_END_BIT_POS          6 /**< ER_INT_EN_DATA_END_BIT Position */
#define MXC_F_SDHC_ER_INT_EN_DATA_END_BIT              ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_DATA_END_BIT_POS)) /**< ER_INT_EN_DATA_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_EN_AUTO_CMD_POS              8 /**< ER_INT_EN_AUTO_CMD Position */
#define MXC_F_SDHC_ER_INT_EN_AUTO_CMD                  ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_AUTO_CMD_POS)) /**< ER_INT_EN_AUTO_CMD Mask */

#define MXC_F_SDHC_ER_INT_EN_ADMA_POS                  9 /**< ER_INT_EN_ADMA Position */
#define MXC_F_SDHC_ER_INT_EN_ADMA                      ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_ADMA_POS)) /**< ER_INT_EN_ADMA Mask */

#define MXC_F_SDHC_ER_INT_EN_TUNING_POS                10 /**< ER_INT_EN_TUNING Position */
#define MXC_F_SDHC_ER_INT_EN_TUNING                    ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_TUNING_POS)) /**< ER_INT_EN_TUNING Mask */

#define MXC_F_SDHC_ER_INT_EN_VENDOR_POS                12 /**< ER_INT_EN_VENDOR Position */
#define MXC_F_SDHC_ER_INT_EN_VENDOR                    ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_EN_VENDOR_POS)) /**< ER_INT_EN_VENDOR Mask */

/**@} end of group SDHC_ER_INT_EN_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_INT_SIGNAL SDHC_INT_SIGNAL
 * @brief    Normal Interrupt Signal Enable.
 * @{
 */
#define MXC_F_SDHC_INT_SIGNAL_CMD_COMP_POS             0 /**< INT_SIGNAL_CMD_COMP Position */
#define MXC_F_SDHC_INT_SIGNAL_CMD_COMP                 ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_CMD_COMP_POS)) /**< INT_SIGNAL_CMD_COMP Mask */

#define MXC_F_SDHC_INT_SIGNAL_TRANS_COMP_POS           1 /**< INT_SIGNAL_TRANS_COMP Position */
#define MXC_F_SDHC_INT_SIGNAL_TRANS_COMP               ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_TRANS_COMP_POS)) /**< INT_SIGNAL_TRANS_COMP Mask */

#define MXC_F_SDHC_INT_SIGNAL_BLK_GAP_POS              2 /**< INT_SIGNAL_BLK_GAP Position */
#define MXC_F_SDHC_INT_SIGNAL_BLK_GAP                  ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_BLK_GAP_POS)) /**< INT_SIGNAL_BLK_GAP Mask */

#define MXC_F_SDHC_INT_SIGNAL_DMA_POS                  3 /**< INT_SIGNAL_DMA Position */
#define MXC_F_SDHC_INT_SIGNAL_DMA                      ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_DMA_POS)) /**< INT_SIGNAL_DMA Mask */

#define MXC_F_SDHC_INT_SIGNAL_BUFFER_WR_POS            4 /**< INT_SIGNAL_BUFFER_WR Position */
#define MXC_F_SDHC_INT_SIGNAL_BUFFER_WR                ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_BUFFER_WR_POS)) /**< INT_SIGNAL_BUFFER_WR Mask */

#define MXC_F_SDHC_INT_SIGNAL_BUFFER_RD_POS            5 /**< INT_SIGNAL_BUFFER_RD Position */
#define MXC_F_SDHC_INT_SIGNAL_BUFFER_RD                ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_BUFFER_RD_POS)) /**< INT_SIGNAL_BUFFER_RD Mask */

#define MXC_F_SDHC_INT_SIGNAL_CARD_INSERT_POS          6 /**< INT_SIGNAL_CARD_INSERT Position */
#define MXC_F_SDHC_INT_SIGNAL_CARD_INSERT              ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_CARD_INSERT_POS)) /**< INT_SIGNAL_CARD_INSERT Mask */

#define MXC_F_SDHC_INT_SIGNAL_CARD_REMOVAL_POS         7 /**< INT_SIGNAL_CARD_REMOVAL Position */
#define MXC_F_SDHC_INT_SIGNAL_CARD_REMOVAL             ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_CARD_REMOVAL_POS)) /**< INT_SIGNAL_CARD_REMOVAL Mask */

#define MXC_F_SDHC_INT_SIGNAL_CARD_INT_POS             8 /**< INT_SIGNAL_CARD_INT Position */
#define MXC_F_SDHC_INT_SIGNAL_CARD_INT                 ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_CARD_INT_POS)) /**< INT_SIGNAL_CARD_INT Mask */

#define MXC_F_SDHC_INT_SIGNAL_RETUNING_POS             12 /**< INT_SIGNAL_RETUNING Position */
#define MXC_F_SDHC_INT_SIGNAL_RETUNING                 ((uint16_t)(0x1UL << MXC_F_SDHC_INT_SIGNAL_RETUNING_POS)) /**< INT_SIGNAL_RETUNING Mask */

/**@} end of group SDHC_INT_SIGNAL_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ER_INT_SIGNAL SDHC_ER_INT_SIGNAL
 * @brief    Error Interrupt Signal Enable.
 * @{
 */
#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_TO_POS            0 /**< ER_INT_SIGNAL_CMD_TO Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_TO                ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_CMD_TO_POS)) /**< ER_INT_SIGNAL_CMD_TO Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_CRC_POS           1 /**< ER_INT_SIGNAL_CMD_CRC Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_CRC               ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_CMD_CRC_POS)) /**< ER_INT_SIGNAL_CMD_CRC Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_END_BIT_POS       2 /**< ER_INT_SIGNAL_CMD_END_BIT Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_END_BIT           ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_CMD_END_BIT_POS)) /**< ER_INT_SIGNAL_CMD_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_IDX_POS           3 /**< ER_INT_SIGNAL_CMD_IDX Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_CMD_IDX               ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_CMD_IDX_POS)) /**< ER_INT_SIGNAL_CMD_IDX Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_TO_POS           4 /**< ER_INT_SIGNAL_DATA_TO Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_TO               ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_DATA_TO_POS)) /**< ER_INT_SIGNAL_DATA_TO Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_CRC_POS          5 /**< ER_INT_SIGNAL_DATA_CRC Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_CRC              ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_DATA_CRC_POS)) /**< ER_INT_SIGNAL_DATA_CRC Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_END_BIT_POS      6 /**< ER_INT_SIGNAL_DATA_END_BIT Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_DATA_END_BIT          ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_DATA_END_BIT_POS)) /**< ER_INT_SIGNAL_DATA_END_BIT Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_CURR_LIM_POS          7 /**< ER_INT_SIGNAL_CURR_LIM Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_CURR_LIM              ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_CURR_LIM_POS)) /**< ER_INT_SIGNAL_CURR_LIM Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_AUTO_CMD_POS          8 /**< ER_INT_SIGNAL_AUTO_CMD Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_AUTO_CMD              ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_AUTO_CMD_POS)) /**< ER_INT_SIGNAL_AUTO_CMD Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_ADMA_POS              9 /**< ER_INT_SIGNAL_ADMA Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_ADMA                  ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_ADMA_POS)) /**< ER_INT_SIGNAL_ADMA Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_TUNING_POS            10 /**< ER_INT_SIGNAL_TUNING Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_TUNING                ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_TUNING_POS)) /**< ER_INT_SIGNAL_TUNING Mask */

#define MXC_F_SDHC_ER_INT_SIGNAL_TAR_RESP_POS          12 /**< ER_INT_SIGNAL_TAR_RESP Position */
#define MXC_F_SDHC_ER_INT_SIGNAL_TAR_RESP              ((uint16_t)(0x1UL << MXC_F_SDHC_ER_INT_SIGNAL_TAR_RESP_POS)) /**< ER_INT_SIGNAL_TAR_RESP Mask */

/**@} end of group SDHC_ER_INT_SIGNAL_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_AUTO_CMD_ER SDHC_AUTO_CMD_ER
 * @brief    Auto CMD Error Status.
 * @{
 */
#define MXC_F_SDHC_AUTO_CMD_ER_NOT_EXCUTED_POS         0 /**< AUTO_CMD_ER_NOT_EXCUTED Position */
#define MXC_F_SDHC_AUTO_CMD_ER_NOT_EXCUTED             ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_NOT_EXCUTED_POS)) /**< AUTO_CMD_ER_NOT_EXCUTED Mask */

#define MXC_F_SDHC_AUTO_CMD_ER_TO_POS                  1 /**< AUTO_CMD_ER_TO Position */
#define MXC_F_SDHC_AUTO_CMD_ER_TO                      ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_TO_POS)) /**< AUTO_CMD_ER_TO Mask */

#define MXC_F_SDHC_AUTO_CMD_ER_CRC_POS                 2 /**< AUTO_CMD_ER_CRC Position */
#define MXC_F_SDHC_AUTO_CMD_ER_CRC                     ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_CRC_POS)) /**< AUTO_CMD_ER_CRC Mask */

#define MXC_F_SDHC_AUTO_CMD_ER_END_BIT_POS             3 /**< AUTO_CMD_ER_END_BIT Position */
#define MXC_F_SDHC_AUTO_CMD_ER_END_BIT                 ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_END_BIT_POS)) /**< AUTO_CMD_ER_END_BIT Mask */

#define MXC_F_SDHC_AUTO_CMD_ER_INDEX_POS               4 /**< AUTO_CMD_ER_INDEX Position */
#define MXC_F_SDHC_AUTO_CMD_ER_INDEX                   ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_INDEX_POS)) /**< AUTO_CMD_ER_INDEX Mask */

#define MXC_F_SDHC_AUTO_CMD_ER_NOT_ISSUED_POS          7 /**< AUTO_CMD_ER_NOT_ISSUED Position */
#define MXC_F_SDHC_AUTO_CMD_ER_NOT_ISSUED              ((uint16_t)(0x1UL << MXC_F_SDHC_AUTO_CMD_ER_NOT_ISSUED_POS)) /**< AUTO_CMD_ER_NOT_ISSUED Mask */

/**@} end of group SDHC_AUTO_CMD_ER_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_HOST_CN_2 SDHC_HOST_CN_2
 * @brief    Host Control 2.
 * @{
 */
#define MXC_F_SDHC_HOST_CN_2_UHS_POS                   0 /**< HOST_CN_2_UHS Position */
#define MXC_F_SDHC_HOST_CN_2_UHS                       ((uint16_t)(0x3UL << MXC_F_SDHC_HOST_CN_2_UHS_POS)) /**< HOST_CN_2_UHS Mask */

#define MXC_F_SDHC_HOST_CN_2_SIGNAL_V1_8_POS           3 /**< HOST_CN_2_SIGNAL_V1_8 Position */
#define MXC_F_SDHC_HOST_CN_2_SIGNAL_V1_8               ((uint16_t)(0x1UL << MXC_F_SDHC_HOST_CN_2_SIGNAL_V1_8_POS)) /**< HOST_CN_2_SIGNAL_V1_8 Mask */

#define MXC_F_SDHC_HOST_CN_2_DRIVER_STRENGTH_POS       4 /**< HOST_CN_2_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_HOST_CN_2_DRIVER_STRENGTH           ((uint16_t)(0x3UL << MXC_F_SDHC_HOST_CN_2_DRIVER_STRENGTH_POS)) /**< HOST_CN_2_DRIVER_STRENGTH Mask */

#define MXC_F_SDHC_HOST_CN_2_EXCUTE_POS                6 /**< HOST_CN_2_EXCUTE Position */
#define MXC_F_SDHC_HOST_CN_2_EXCUTE                    ((uint16_t)(0x1UL << MXC_F_SDHC_HOST_CN_2_EXCUTE_POS)) /**< HOST_CN_2_EXCUTE Mask */

#define MXC_F_SDHC_HOST_CN_2_SAMPLING_CLK_POS          7 /**< HOST_CN_2_SAMPLING_CLK Position */
#define MXC_F_SDHC_HOST_CN_2_SAMPLING_CLK              ((uint16_t)(0x1UL << MXC_F_SDHC_HOST_CN_2_SAMPLING_CLK_POS)) /**< HOST_CN_2_SAMPLING_CLK Mask */

#define MXC_F_SDHC_HOST_CN_2_ASYNCH_INT_POS            14 /**< HOST_CN_2_ASYNCH_INT Position */
#define MXC_F_SDHC_HOST_CN_2_ASYNCH_INT                ((uint16_t)(0x1UL << MXC_F_SDHC_HOST_CN_2_ASYNCH_INT_POS)) /**< HOST_CN_2_ASYNCH_INT Mask */

#define MXC_F_SDHC_HOST_CN_2_PRESET_VAL_EN_POS         15 /**< HOST_CN_2_PRESET_VAL_EN Position */
#define MXC_F_SDHC_HOST_CN_2_PRESET_VAL_EN             ((uint16_t)(0x1UL << MXC_F_SDHC_HOST_CN_2_PRESET_VAL_EN_POS)) /**< HOST_CN_2_PRESET_VAL_EN Mask */

/**@} end of group SDHC_HOST_CN_2_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_CFG_0 SDHC_CFG_0
 * @brief    Capabilities 0-31.
 * @{
 */
#define MXC_F_SDHC_CFG_0_CLK_FREQ_POS                  0 /**< CFG_0_CLK_FREQ Position */
#define MXC_F_SDHC_CFG_0_CLK_FREQ                      ((uint32_t)(0x3FUL << MXC_F_SDHC_CFG_0_CLK_FREQ_POS)) /**< CFG_0_CLK_FREQ Mask */

#define MXC_F_SDHC_CFG_0_TO_CLK_UNIT_POS               7 /**< CFG_0_TO_CLK_UNIT Position */
#define MXC_F_SDHC_CFG_0_TO_CLK_UNIT                   ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_TO_CLK_UNIT_POS)) /**< CFG_0_TO_CLK_UNIT Mask */

#define MXC_F_SDHC_CFG_0_TO_CLK_FREQ_POS               8 /**< CFG_0_TO_CLK_FREQ Position */
#define MXC_F_SDHC_CFG_0_TO_CLK_FREQ                   ((uint32_t)(0xFFUL << MXC_F_SDHC_CFG_0_TO_CLK_FREQ_POS)) /**< CFG_0_TO_CLK_FREQ Mask */

#define MXC_F_SDHC_CFG_0_MAX_BLK_LEN_POS               16 /**< CFG_0_MAX_BLK_LEN Position */
#define MXC_F_SDHC_CFG_0_MAX_BLK_LEN                   ((uint32_t)(0x3UL << MXC_F_SDHC_CFG_0_MAX_BLK_LEN_POS)) /**< CFG_0_MAX_BLK_LEN Mask */

#define MXC_F_SDHC_CFG_0_BIT_8_POS                     18 /**< CFG_0_BIT_8 Position */
#define MXC_F_SDHC_CFG_0_BIT_8                         ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_BIT_8_POS)) /**< CFG_0_BIT_8 Mask */

#define MXC_F_SDHC_CFG_0_ADMA2_POS                     19 /**< CFG_0_ADMA2 Position */
#define MXC_F_SDHC_CFG_0_ADMA2                         ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_ADMA2_POS)) /**< CFG_0_ADMA2 Mask */

#define MXC_F_SDHC_CFG_0_HS_POS                        21 /**< CFG_0_HS Position */
#define MXC_F_SDHC_CFG_0_HS                            ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_HS_POS)) /**< CFG_0_HS Mask */

#define MXC_F_SDHC_CFG_0_SDMA_POS                      22 /**< CFG_0_SDMA Position */
#define MXC_F_SDHC_CFG_0_SDMA                          ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_SDMA_POS)) /**< CFG_0_SDMA Mask */

#define MXC_F_SDHC_CFG_0_SUSPEND_POS                   23 /**< CFG_0_SUSPEND Position */
#define MXC_F_SDHC_CFG_0_SUSPEND                       ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_SUSPEND_POS)) /**< CFG_0_SUSPEND Mask */

#define MXC_F_SDHC_CFG_0_V3_3_POS                      24 /**< CFG_0_V3_3 Position */
#define MXC_F_SDHC_CFG_0_V3_3                          ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_V3_3_POS)) /**< CFG_0_V3_3 Mask */

#define MXC_F_SDHC_CFG_0_V3_0_POS                      25 /**< CFG_0_V3_0 Position */
#define MXC_F_SDHC_CFG_0_V3_0                          ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_V3_0_POS)) /**< CFG_0_V3_0 Mask */

#define MXC_F_SDHC_CFG_0_V1_8_POS                      26 /**< CFG_0_V1_8 Position */
#define MXC_F_SDHC_CFG_0_V1_8                          ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_V1_8_POS)) /**< CFG_0_V1_8 Mask */

#define MXC_F_SDHC_CFG_0_BIT_64_SYS_BUS_POS            28 /**< CFG_0_BIT_64_SYS_BUS Position */
#define MXC_F_SDHC_CFG_0_BIT_64_SYS_BUS                ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_BIT_64_SYS_BUS_POS)) /**< CFG_0_BIT_64_SYS_BUS Mask */

#define MXC_F_SDHC_CFG_0_ASYNC_INT_POS                 29 /**< CFG_0_ASYNC_INT Position */
#define MXC_F_SDHC_CFG_0_ASYNC_INT                     ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_0_ASYNC_INT_POS)) /**< CFG_0_ASYNC_INT Mask */

#define MXC_F_SDHC_CFG_0_SLOT_TYPE_POS                 30 /**< CFG_0_SLOT_TYPE Position */
#define MXC_F_SDHC_CFG_0_SLOT_TYPE                     ((uint32_t)(0x3UL << MXC_F_SDHC_CFG_0_SLOT_TYPE_POS)) /**< CFG_0_SLOT_TYPE Mask */

/**@} end of group SDHC_CFG_0_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_CFG_1 SDHC_CFG_1
 * @brief    Capabilities 32-63.
 * @{
 */
#define MXC_F_SDHC_CFG_1_SDR50_POS                     0 /**< CFG_1_SDR50 Position */
#define MXC_F_SDHC_CFG_1_SDR50                         ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_SDR50_POS)) /**< CFG_1_SDR50 Mask */

#define MXC_F_SDHC_CFG_1_SDR104_POS                    1 /**< CFG_1_SDR104 Position */
#define MXC_F_SDHC_CFG_1_SDR104                        ((uint32_t)(0x0UL << MXC_F_SDHC_CFG_1_SDR104_POS)) /**< CFG_1_SDR104 Mask */

#define MXC_F_SDHC_CFG_1_DDR50_POS                     2 /**< CFG_1_DDR50 Position */
#define MXC_F_SDHC_CFG_1_DDR50                         ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_DDR50_POS)) /**< CFG_1_DDR50 Mask */

#define MXC_F_SDHC_CFG_1_DRIVER_A_POS                  4 /**< CFG_1_DRIVER_A Position */
#define MXC_F_SDHC_CFG_1_DRIVER_A                      ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_DRIVER_A_POS)) /**< CFG_1_DRIVER_A Mask */

#define MXC_F_SDHC_CFG_1_DRIVER_C_POS                  5 /**< CFG_1_DRIVER_C Position */
#define MXC_F_SDHC_CFG_1_DRIVER_C                      ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_DRIVER_C_POS)) /**< CFG_1_DRIVER_C Mask */

#define MXC_F_SDHC_CFG_1_DRIVER_D_POS                  6 /**< CFG_1_DRIVER_D Position */
#define MXC_F_SDHC_CFG_1_DRIVER_D                      ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_DRIVER_D_POS)) /**< CFG_1_DRIVER_D Mask */

#define MXC_F_SDHC_CFG_1_TIMER_CNT_TUNING_POS          8 /**< CFG_1_TIMER_CNT_TUNING Position */
#define MXC_F_SDHC_CFG_1_TIMER_CNT_TUNING              ((uint32_t)(0xFUL << MXC_F_SDHC_CFG_1_TIMER_CNT_TUNING_POS)) /**< CFG_1_TIMER_CNT_TUNING Mask */

#define MXC_F_SDHC_CFG_1_TUNING_SDR50_POS              13 /**< CFG_1_TUNING_SDR50 Position */
#define MXC_F_SDHC_CFG_1_TUNING_SDR50                  ((uint32_t)(0x1UL << MXC_F_SDHC_CFG_1_TUNING_SDR50_POS)) /**< CFG_1_TUNING_SDR50 Mask */

#define MXC_F_SDHC_CFG_1_RETUNING_POS                  14 /**< CFG_1_RETUNING Position */
#define MXC_F_SDHC_CFG_1_RETUNING                      ((uint32_t)(0x3UL << MXC_F_SDHC_CFG_1_RETUNING_POS)) /**< CFG_1_RETUNING Mask */

#define MXC_F_SDHC_CFG_1_CLK_MULTI_POS                 16 /**< CFG_1_CLK_MULTI Position */
#define MXC_F_SDHC_CFG_1_CLK_MULTI                     ((uint32_t)(0xFFUL << MXC_F_SDHC_CFG_1_CLK_MULTI_POS)) /**< CFG_1_CLK_MULTI Mask */

/**@} end of group SDHC_CFG_1_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_MAX_CURR_CFG SDHC_MAX_CURR_CFG
 * @brief    Maximum Current Capabilities.
 * @{
 */
#define MXC_F_SDHC_MAX_CURR_CFG_V3_3_POS               0 /**< MAX_CURR_CFG_V3_3 Position */
#define MXC_F_SDHC_MAX_CURR_CFG_V3_3                   ((uint32_t)(0xFFUL << MXC_F_SDHC_MAX_CURR_CFG_V3_3_POS)) /**< MAX_CURR_CFG_V3_3 Mask */

#define MXC_F_SDHC_MAX_CURR_CFG_V3_0_POS               8 /**< MAX_CURR_CFG_V3_0 Position */
#define MXC_F_SDHC_MAX_CURR_CFG_V3_0                   ((uint32_t)(0xFFUL << MXC_F_SDHC_MAX_CURR_CFG_V3_0_POS)) /**< MAX_CURR_CFG_V3_0 Mask */

#define MXC_F_SDHC_MAX_CURR_CFG_V1_8_POS               16 /**< MAX_CURR_CFG_V1_8 Position */
#define MXC_F_SDHC_MAX_CURR_CFG_V1_8                   ((uint32_t)(0xFFUL << MXC_F_SDHC_MAX_CURR_CFG_V1_8_POS)) /**< MAX_CURR_CFG_V1_8 Mask */

/**@} end of group SDHC_MAX_CURR_CFG_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_FORCE_CMD SDHC_FORCE_CMD
 * @brief    Force Event for Auto CMD Error Status.
 * @{
 */
#define MXC_F_SDHC_FORCE_CMD_NOT_EXCU_POS              0 /**< FORCE_CMD_NOT_EXCU Position */
#define MXC_F_SDHC_FORCE_CMD_NOT_EXCU                  ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_NOT_EXCU_POS)) /**< FORCE_CMD_NOT_EXCU Mask */

#define MXC_F_SDHC_FORCE_CMD_TO_POS                    1 /**< FORCE_CMD_TO Position */
#define MXC_F_SDHC_FORCE_CMD_TO                        ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_TO_POS)) /**< FORCE_CMD_TO Mask */

#define MXC_F_SDHC_FORCE_CMD_CRC_POS                   2 /**< FORCE_CMD_CRC Position */
#define MXC_F_SDHC_FORCE_CMD_CRC                       ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_CRC_POS)) /**< FORCE_CMD_CRC Mask */

#define MXC_F_SDHC_FORCE_CMD_END_BIT_POS               3 /**< FORCE_CMD_END_BIT Position */
#define MXC_F_SDHC_FORCE_CMD_END_BIT                   ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_END_BIT_POS)) /**< FORCE_CMD_END_BIT Mask */

#define MXC_F_SDHC_FORCE_CMD_INDEX_POS                 4 /**< FORCE_CMD_INDEX Position */
#define MXC_F_SDHC_FORCE_CMD_INDEX                     ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_INDEX_POS)) /**< FORCE_CMD_INDEX Mask */

#define MXC_F_SDHC_FORCE_CMD_NOT_ISSUED_POS            7 /**< FORCE_CMD_NOT_ISSUED Position */
#define MXC_F_SDHC_FORCE_CMD_NOT_ISSUED                ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_CMD_NOT_ISSUED_POS)) /**< FORCE_CMD_NOT_ISSUED Mask */

/**@} end of group SDHC_FORCE_CMD_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_FORCE_EVENT_INT_STAT SDHC_FORCE_EVENT_INT_STAT
 * @brief    Force Event for Error Interrupt Status.
 * @{
 */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_TO_POS     0 /**< FORCE_EVENT_INT_STAT_CMD_TO Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_TO         ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_TO_POS)) /**< FORCE_EVENT_INT_STAT_CMD_TO Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_CRC_POS    1 /**< FORCE_EVENT_INT_STAT_CMD_CRC Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_CRC        ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_CRC_POS)) /**< FORCE_EVENT_INT_STAT_CMD_CRC Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_END_BIT_POS 2 /**< FORCE_EVENT_INT_STAT_CMD_END_BIT Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_END_BIT    ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_END_BIT_POS)) /**< FORCE_EVENT_INT_STAT_CMD_END_BIT Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_INDEX_POS  3 /**< FORCE_EVENT_INT_STAT_CMD_INDEX Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_INDEX      ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_CMD_INDEX_POS)) /**< FORCE_EVENT_INT_STAT_CMD_INDEX Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_TO_POS    4 /**< FORCE_EVENT_INT_STAT_DATA_TO Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_TO        ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_TO_POS)) /**< FORCE_EVENT_INT_STAT_DATA_TO Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_CRC_POS   5 /**< FORCE_EVENT_INT_STAT_DATA_CRC Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_CRC       ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_CRC_POS)) /**< FORCE_EVENT_INT_STAT_DATA_CRC Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_END_BIT_POS 6 /**< FORCE_EVENT_INT_STAT_DATA_END_BIT Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_END_BIT   ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_DATA_END_BIT_POS)) /**< FORCE_EVENT_INT_STAT_DATA_END_BIT Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CURR_LIMIT_POS 7 /**< FORCE_EVENT_INT_STAT_CURR_LIMIT Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_CURR_LIMIT     ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_CURR_LIMIT_POS)) /**< FORCE_EVENT_INT_STAT_CURR_LIMIT Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_AUTO_CMD_POS   8 /**< FORCE_EVENT_INT_STAT_AUTO_CMD Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_AUTO_CMD       ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_AUTO_CMD_POS)) /**< FORCE_EVENT_INT_STAT_AUTO_CMD Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_ADMA_POS       9 /**< FORCE_EVENT_INT_STAT_ADMA Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_ADMA           ((uint16_t)(0x1UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_ADMA_POS)) /**< FORCE_EVENT_INT_STAT_ADMA Mask */

#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_VENDOR_POS     12 /**< FORCE_EVENT_INT_STAT_VENDOR Position */
#define MXC_F_SDHC_FORCE_EVENT_INT_STAT_VENDOR         ((uint16_t)(0x7UL << MXC_F_SDHC_FORCE_EVENT_INT_STAT_VENDOR_POS)) /**< FORCE_EVENT_INT_STAT_VENDOR Mask */

/**@} end of group SDHC_FORCE_EVENT_INT_STAT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ADMA_ER SDHC_ADMA_ER
 * @brief    ADMA Error Status.
 * @{
 */
#define MXC_F_SDHC_ADMA_ER_STATE_POS                   0 /**< ADMA_ER_STATE Position */
#define MXC_F_SDHC_ADMA_ER_STATE                       ((uint8_t)(0x3UL << MXC_F_SDHC_ADMA_ER_STATE_POS)) /**< ADMA_ER_STATE Mask */

#define MXC_F_SDHC_ADMA_ER_LEN_MISMATCH_POS            2 /**< ADMA_ER_LEN_MISMATCH Position */
#define MXC_F_SDHC_ADMA_ER_LEN_MISMATCH                ((uint8_t)(0x1UL << MXC_F_SDHC_ADMA_ER_LEN_MISMATCH_POS)) /**< ADMA_ER_LEN_MISMATCH Mask */

/**@} end of group SDHC_ADMA_ER_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ADMA_ADDR_0 SDHC_ADMA_ADDR_0
 * @brief    ADMA System Address 0-31.
 * @{
 */
#define MXC_F_SDHC_ADMA_ADDR_0_ADDR_POS                0 /**< ADMA_ADDR_0_ADDR Position */
#define MXC_F_SDHC_ADMA_ADDR_0_ADDR                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_ADMA_ADDR_0_ADDR_POS)) /**< ADMA_ADDR_0_ADDR Mask */

/**@} end of group SDHC_ADMA_ADDR_0_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_ADMA_ADDR_1 SDHC_ADMA_ADDR_1
 * @brief    ADMA System Address 32-63.
 * @{
 */
#define MXC_F_SDHC_ADMA_ADDR_1_ADDR_POS                0 /**< ADMA_ADDR_1_ADDR Position */
#define MXC_F_SDHC_ADMA_ADDR_1_ADDR                    ((uint32_t)(0xFFFFFFFFUL << MXC_F_SDHC_ADMA_ADDR_1_ADDR_POS)) /**< ADMA_ADDR_1_ADDR Mask */

/**@} end of group SDHC_ADMA_ADDR_1_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_0 SDHC_PRESET_0
 * @brief    Preset Value for Initialization.
 * @{
 */
#define MXC_F_SDHC_PRESET_0_SDCLK_FREQ_POS             0 /**< PRESET_0_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_0_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_0_SDCLK_FREQ_POS)) /**< PRESET_0_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_0_CLK_GEN_POS                10 /**< PRESET_0_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_0_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_0_CLK_GEN_POS)) /**< PRESET_0_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_0_DRIVER_STRENGTH_POS        14 /**< PRESET_0_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_0_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_0_DRIVER_STRENGTH_POS)) /**< PRESET_0_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_0_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_1 SDHC_PRESET_1
 * @brief    Preset Value for Default Speed.
 * @{
 */
#define MXC_F_SDHC_PRESET_1_SDCLK_FREQ_POS             0 /**< PRESET_1_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_1_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_1_SDCLK_FREQ_POS)) /**< PRESET_1_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_1_CLK_GEN_POS                10 /**< PRESET_1_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_1_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_1_CLK_GEN_POS)) /**< PRESET_1_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_1_DRIVER_STRENGTH_POS        14 /**< PRESET_1_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_1_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_1_DRIVER_STRENGTH_POS)) /**< PRESET_1_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_1_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_2 SDHC_PRESET_2
 * @brief    Preset Value for High Speed.
 * @{
 */
#define MXC_F_SDHC_PRESET_2_SDCLK_FREQ_POS             0 /**< PRESET_2_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_2_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_2_SDCLK_FREQ_POS)) /**< PRESET_2_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_2_CLK_GEN_POS                10 /**< PRESET_2_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_2_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_2_CLK_GEN_POS)) /**< PRESET_2_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_2_DRIVER_STRENGTH_POS        14 /**< PRESET_2_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_2_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_2_DRIVER_STRENGTH_POS)) /**< PRESET_2_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_2_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_3 SDHC_PRESET_3
 * @brief    Preset Value for SDR12.
 * @{
 */
#define MXC_F_SDHC_PRESET_3_SDCLK_FREQ_POS             0 /**< PRESET_3_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_3_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_3_SDCLK_FREQ_POS)) /**< PRESET_3_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_3_CLK_GEN_POS                10 /**< PRESET_3_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_3_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_3_CLK_GEN_POS)) /**< PRESET_3_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_3_DRIVER_STRENGTH_POS        14 /**< PRESET_3_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_3_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_3_DRIVER_STRENGTH_POS)) /**< PRESET_3_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_3_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_4 SDHC_PRESET_4
 * @brief    Preset Value for SDR25.
 * @{
 */
#define MXC_F_SDHC_PRESET_4_SDCLK_FREQ_POS             0 /**< PRESET_4_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_4_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_4_SDCLK_FREQ_POS)) /**< PRESET_4_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_4_CLK_GEN_POS                10 /**< PRESET_4_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_4_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_4_CLK_GEN_POS)) /**< PRESET_4_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_4_DRIVER_STRENGTH_POS        14 /**< PRESET_4_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_4_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_4_DRIVER_STRENGTH_POS)) /**< PRESET_4_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_4_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_5 SDHC_PRESET_5
 * @brief    Preset Value for SDR50.
 * @{
 */
#define MXC_F_SDHC_PRESET_5_SDCLK_FREQ_POS             0 /**< PRESET_5_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_5_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_5_SDCLK_FREQ_POS)) /**< PRESET_5_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_5_CLK_GEN_POS                10 /**< PRESET_5_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_5_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_5_CLK_GEN_POS)) /**< PRESET_5_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_5_DRIVER_STRENGTH_POS        14 /**< PRESET_5_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_5_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_5_DRIVER_STRENGTH_POS)) /**< PRESET_5_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_5_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_6 SDHC_PRESET_6
 * @brief    Preset Value for SDR104.
 * @{
 */
#define MXC_F_SDHC_PRESET_6_SDCLK_FREQ_POS             0 /**< PRESET_6_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_6_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_6_SDCLK_FREQ_POS)) /**< PRESET_6_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_6_CLK_GEN_POS                10 /**< PRESET_6_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_6_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_6_CLK_GEN_POS)) /**< PRESET_6_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_6_DRIVER_STRENGTH_POS        14 /**< PRESET_6_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_6_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_6_DRIVER_STRENGTH_POS)) /**< PRESET_6_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_6_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_PRESET_7 SDHC_PRESET_7
 * @brief    Preset Value for DDR50.
 * @{
 */
#define MXC_F_SDHC_PRESET_7_SDCLK_FREQ_POS             0 /**< PRESET_7_SDCLK_FREQ Position */
#define MXC_F_SDHC_PRESET_7_SDCLK_FREQ                 ((uint16_t)(0x3FFUL << MXC_F_SDHC_PRESET_7_SDCLK_FREQ_POS)) /**< PRESET_7_SDCLK_FREQ Mask */

#define MXC_F_SDHC_PRESET_7_CLK_GEN_POS                10 /**< PRESET_7_CLK_GEN Position */
#define MXC_F_SDHC_PRESET_7_CLK_GEN                    ((uint16_t)(0x1UL << MXC_F_SDHC_PRESET_7_CLK_GEN_POS)) /**< PRESET_7_CLK_GEN Mask */

#define MXC_F_SDHC_PRESET_7_DRIVER_STRENGTH_POS        14 /**< PRESET_7_DRIVER_STRENGTH Position */
#define MXC_F_SDHC_PRESET_7_DRIVER_STRENGTH            ((uint16_t)(0x3UL << MXC_F_SDHC_PRESET_7_DRIVER_STRENGTH_POS)) /**< PRESET_7_DRIVER_STRENGTH Mask */

/**@} end of group SDHC_PRESET_7_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_SLOT_INT SDHC_SLOT_INT
 * @brief    Slot Interrupt Status.
 * @{
 */
#define MXC_F_SDHC_SLOT_INT_INT_SIGNALS_POS            0 /**< SLOT_INT_INT_SIGNALS Position */
#define MXC_F_SDHC_SLOT_INT_INT_SIGNALS                ((uint16_t)(0x1UL << MXC_F_SDHC_SLOT_INT_INT_SIGNALS_POS)) /**< SLOT_INT_INT_SIGNALS Mask */

/**@} end of group SDHC_SLOT_INT_Register */

/**
 * @ingroup  sdhc_registers
 * @defgroup SDHC_HOST_CN_VER SDHC_HOST_CN_VER
 * @brief    Host Controller Version.
 * @{
 */
#define MXC_F_SDHC_HOST_CN_VER_SPEC_VER_POS            0 /**< HOST_CN_VER_SPEC_VER Position */
#define MXC_F_SDHC_HOST_CN_VER_SPEC_VER                ((uint16_t)(0xFFUL << MXC_F_SDHC_HOST_CN_VER_SPEC_VER_POS)) /**< HOST_CN_VER_SPEC_VER Mask */

#define MXC_F_SDHC_HOST_CN_VER_VEND_VER_POS            8 /**< HOST_CN_VER_VEND_VER Position */
#define MXC_F_SDHC_HOST_CN_VER_VEND_VER                ((uint16_t)(0xFFUL << MXC_F_SDHC_HOST_CN_VER_VEND_VER_POS)) /**< HOST_CN_VER_VEND_VER Mask */

/**@} end of group SDHC_HOST_CN_VER_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SDHC_REGS_H_
