/**
 * @file    csi2_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the CSI2_REVA Peripheral Module.
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

#ifndef _CSI2_REVA_REGS_H_
#define _CSI2_REVA_REGS_H_

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
 * @ingroup     csi2_reva
 * @defgroup    csi2_reva_registers CSI2_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the CSI2_REVA Peripheral Module.
 * @details Camera Serial Interface Registers.
 */

/**
 * @ingroup csi2_reva_registers
 * Structure type to access the CSI2_REVA Registers.
 */
typedef struct {
    __IO uint32_t cfg_num_lanes;        /**< <tt>\b 0x000:</tt> CSI2_REVA CFG_NUM_LANES Register */
    __IO uint32_t cfg_clk_lane_en;      /**< <tt>\b 0x004:</tt> CSI2_REVA CFG_CLK_LANE_EN Register */
    __IO uint32_t cfg_data_lane_en;     /**< <tt>\b 0x008:</tt> CSI2_REVA CFG_DATA_LANE_EN Register */
    __IO uint32_t cfg_flush_count;      /**< <tt>\b 0x00C:</tt> CSI2_REVA CFG_FLUSH_COUNT Register */
    __IO uint32_t cfg_bit_err;          /**< <tt>\b 0x010:</tt> CSI2_REVA CFG_BIT_ERR Register */
    __IO uint32_t irq_status;           /**< <tt>\b 0x014:</tt> CSI2_REVA IRQ_STATUS Register */
    __IO uint32_t irq_enable;           /**< <tt>\b 0x018:</tt> CSI2_REVA IRQ_ENABLE Register */
    __IO uint32_t irq_clr;              /**< <tt>\b 0x01C:</tt> CSI2_REVA IRQ_CLR Register */
    __IO uint32_t ulps_clk_status;      /**< <tt>\b 0x020:</tt> CSI2_REVA ULPS_CLK_STATUS Register */
    __IO uint32_t ulps_status;          /**< <tt>\b 0x024:</tt> CSI2_REVA ULPS_STATUS Register */
    __IO uint32_t ulps_clk_mark_status; /**< <tt>\b 0x028:</tt> CSI2_REVA ULPS_CLK_MARK_STATUS Register */
    __IO uint32_t ulps_mark_status;     /**< <tt>\b 0x02C:</tt> CSI2_REVA ULPS_MARK_STATUS Register */
    __IO uint32_t ppi_errsot_hs;        /**< <tt>\b 0x030:</tt> CSI2_REVA PPI_ERRSOT_HS Register */
    __IO uint32_t ppi_errsotsync_hs;    /**< <tt>\b 0x034:</tt> CSI2_REVA PPI_ERRSOTSYNC_HS Register */
    __IO uint32_t ppi_erresc;           /**< <tt>\b 0x038:</tt> CSI2_REVA PPI_ERRESC Register */
    __IO uint32_t ppi_errsyncesc;       /**< <tt>\b 0x03C:</tt> CSI2_REVA PPI_ERRSYNCESC Register */
    __IO uint32_t ppi_errcontrol;       /**< <tt>\b 0x040:</tt> CSI2_REVA PPI_ERRCONTROL Register */
    __IO uint32_t cfg_cphy_en;          /**< <tt>\b 0x044:</tt> CSI2_REVA CFG_CPHY_EN Register */
    __IO uint32_t cfg_ppi_16_en;        /**< <tt>\b 0x048:</tt> CSI2_REVA CFG_PPI_16_EN Register */
    __IO uint32_t cfg_packet_interface_en; /**< <tt>\b 0x04C:</tt> CSI2_REVA CFG_PACKET_INTERFACE_EN Register */
    __IO uint32_t cfg_vcx_en;           /**< <tt>\b 0x050:</tt> CSI2_REVA CFG_VCX_EN Register */
    __IO uint32_t cfg_byte_data_format; /**< <tt>\b 0x054:</tt> CSI2_REVA CFG_BYTE_DATA_FORMAT Register */
    __IO uint32_t cfg_disable_payload_0; /**< <tt>\b 0x058:</tt> CSI2_REVA CFG_DISABLE_PAYLOAD_0 Register */
    __IO uint32_t cfg_disable_payload_1; /**< <tt>\b 0x05C:</tt> CSI2_REVA CFG_DISABLE_PAYLOAD_1 Register */
    __R  uint32_t rsv_0x60_0x7f[8];
    __IO uint32_t cfg_vid_ignore_vc;    /**< <tt>\b 0x080:</tt> CSI2_REVA CFG_VID_IGNORE_VC Register */
    __IO uint32_t cfg_vid_vc;           /**< <tt>\b 0x084:</tt> CSI2_REVA CFG_VID_VC Register */
    __IO uint32_t cfg_p_fifo_send_level; /**< <tt>\b 0x088:</tt> CSI2_REVA CFG_P_FIFO_SEND_LEVEL Register */
    __IO uint32_t cfg_vid_vsync;        /**< <tt>\b 0x08C:</tt> CSI2_REVA CFG_VID_VSYNC Register */
    __IO uint32_t cfg_vid_hsync_fp;     /**< <tt>\b 0x090:</tt> CSI2_REVA CFG_VID_HSYNC_FP Register */
    __IO uint32_t cfg_vid_hsync;        /**< <tt>\b 0x094:</tt> CSI2_REVA CFG_VID_HSYNC Register */
    __IO uint32_t cfg_vid_hsync_bp;     /**< <tt>\b 0x098:</tt> CSI2_REVA CFG_VID_HSYNC_BP Register */
    __R  uint32_t rsv_0x9c_0x3ff[217];
    __IO uint32_t cfg_databus16_sel;    /**< <tt>\b 0x400:</tt> CSI2_REVA CFG_DATABUS16_SEL Register */
    __IO uint32_t cfg_d0_swap_sel;      /**< <tt>\b 0x404:</tt> CSI2_REVA CFG_D0_SWAP_SEL Register */
    __IO uint32_t cfg_d1_swap_sel;      /**< <tt>\b 0x408:</tt> CSI2_REVA CFG_D1_SWAP_SEL Register */
    __IO uint32_t cfg_d2_swap_sel;      /**< <tt>\b 0x40C:</tt> CSI2_REVA CFG_D2_SWAP_SEL Register */
    __IO uint32_t cfg_d3_swap_sel;      /**< <tt>\b 0x410:</tt> CSI2_REVA CFG_D3_SWAP_SEL Register */
    __IO uint32_t cfg_c0_swap_sel;      /**< <tt>\b 0x414:</tt> CSI2_REVA CFG_C0_SWAP_SEL Register */
    __IO uint32_t cfg_dpdn_swap;        /**< <tt>\b 0x418:</tt> CSI2_REVA CFG_DPDN_SWAP Register */
    __IO uint32_t rg_cfgclk_1us_cnt;    /**< <tt>\b 0x41C:</tt> CSI2_REVA RG_CFGCLK_1US_CNT Register */
    __IO uint32_t rg_hsrx_clk_pre_time_grp0; /**< <tt>\b 0x420:</tt> CSI2_REVA RG_HSRX_CLK_PRE_TIME_GRP0 Register */
    __IO uint32_t rg_hsrx_data_pre_time_grp0; /**< <tt>\b 0x424:</tt> CSI2_REVA RG_HSRX_DATA_PRE_TIME_GRP0 Register */
    __IO uint32_t reset_deskew;         /**< <tt>\b 0x428:</tt> CSI2_REVA RESET_DESKEW Register */
    __IO uint32_t pma_rdy;              /**< <tt>\b 0x42C:</tt> CSI2_REVA PMA_RDY Register */
    __IO uint32_t xcfgi_dw00;           /**< <tt>\b 0x430:</tt> CSI2_REVA XCFGI_DW00 Register */
    __IO uint32_t xcfgi_dw01;           /**< <tt>\b 0x434:</tt> CSI2_REVA XCFGI_DW01 Register */
    __IO uint32_t xcfgi_dw02;           /**< <tt>\b 0x438:</tt> CSI2_REVA XCFGI_DW02 Register */
    __IO uint32_t xcfgi_dw03;           /**< <tt>\b 0x43C:</tt> CSI2_REVA XCFGI_DW03 Register */
    __IO uint32_t xcfgi_dw04;           /**< <tt>\b 0x440:</tt> CSI2_REVA XCFGI_DW04 Register */
    __IO uint32_t xcfgi_dw05;           /**< <tt>\b 0x444:</tt> CSI2_REVA XCFGI_DW05 Register */
    __IO uint32_t xcfgi_dw06;           /**< <tt>\b 0x448:</tt> CSI2_REVA XCFGI_DW06 Register */
    __IO uint32_t xcfgi_dw07;           /**< <tt>\b 0x44C:</tt> CSI2_REVA XCFGI_DW07 Register */
    __IO uint32_t xcfgi_dw08;           /**< <tt>\b 0x450:</tt> CSI2_REVA XCFGI_DW08 Register */
    __IO uint32_t xcfgi_dw09;           /**< <tt>\b 0x454:</tt> CSI2_REVA XCFGI_DW09 Register */
    __IO uint32_t xcfgi_dw0a;           /**< <tt>\b 0x458:</tt> CSI2_REVA XCFGI_DW0A Register */
    __IO uint32_t xcfgi_dw0b;           /**< <tt>\b 0x45C:</tt> CSI2_REVA XCFGI_DW0B Register */
    __IO uint32_t xcfgi_dw0c;           /**< <tt>\b 0x460:</tt> CSI2_REVA XCFGI_DW0C Register */
    __IO uint32_t xcfgi_dw0d;           /**< <tt>\b 0x464:</tt> CSI2_REVA XCFGI_DW0D Register */
    __IO uint32_t gpio_mode;            /**< <tt>\b 0x468:</tt> CSI2_REVA GPIO_MODE Register */
    __IO uint32_t gpio_dp_ie;           /**< <tt>\b 0x46C:</tt> CSI2_REVA GPIO_DP_IE Register */
    __IO uint32_t gpio_dn_ie;           /**< <tt>\b 0x470:</tt> CSI2_REVA GPIO_DN_IE Register */
    __IO uint32_t gpio_dp_c;            /**< <tt>\b 0x474:</tt> CSI2_REVA GPIO_DP_C Register */
    __IO uint32_t gpio_dn_c;            /**< <tt>\b 0x478:</tt> CSI2_REVA GPIO_DN_C Register */
    __IO uint32_t vcontrol;             /**< <tt>\b 0x47C:</tt> CSI2_REVA VCONTROL Register */
    __IO uint32_t mpsov1;               /**< <tt>\b 0x480:</tt> CSI2_REVA MPSOV1 Register */
    __IO uint32_t mpsov2;               /**< <tt>\b 0x484:</tt> CSI2_REVA MPSOV2 Register */
    __IO uint32_t mpsov3;               /**< <tt>\b 0x488:</tt> CSI2_REVA MPSOV3 Register */
    __R  uint32_t rsv_0x48c;
    __IO uint32_t rg_cdrx_dsirx_en;     /**< <tt>\b 0x490:</tt> CSI2_REVA RG_CDRX_DSIRX_EN Register */
    __IO uint32_t rg_cdrx_l012_sublvds_en; /**< <tt>\b 0x494:</tt> CSI2_REVA RG_CDRX_L012_SUBLVDS_EN Register */
    __IO uint32_t rg_cdrx_l012_hsrt_ctrl; /**< <tt>\b 0x498:</tt> CSI2_REVA RG_CDRX_L012_HSRT_CTRL Register */
    __IO uint32_t rg_cdrx_bisths_pll_en; /**< <tt>\b 0x49C:</tt> CSI2_REVA RG_CDRX_BISTHS_PLL_EN Register */
    __IO uint32_t rg_cdrx_bisths_pll_pre_div2; /**< <tt>\b 0x4A0:</tt> CSI2_REVA RG_CDRX_BISTHS_PLL_PRE_DIV2 Register */
    __IO uint32_t rg_cdrx_bisths_pll_fbk_int; /**< <tt>\b 0x4A4:</tt> CSI2_REVA RG_CDRX_BISTHS_PLL_FBK_INT Register */
    __IO uint32_t dbg1_mux_sel;         /**< <tt>\b 0x4A8:</tt> CSI2_REVA DBG1_MUX_SEL Register */
    __IO uint32_t dbg2_mux_sel;         /**< <tt>\b 0x4AC:</tt> CSI2_REVA DBG2_MUX_SEL Register */
    __IO uint32_t dbg1_mux_dout;        /**< <tt>\b 0x4B0:</tt> CSI2_REVA DBG1_MUX_DOUT Register */
    __IO uint32_t dbg2_mux_dout;        /**< <tt>\b 0x4B4:</tt> CSI2_REVA DBG2_MUX_DOUT Register */
    __IO uint32_t aon_power_ready_n;    /**< <tt>\b 0x4B8:</tt> CSI2_REVA AON_POWER_READY_N Register */
    __IO uint32_t dphy_rst_n;           /**< <tt>\b 0x4BC:</tt> CSI2_REVA DPHY_RST_N Register */
    __IO uint32_t rxbyteclkhs_inv;      /**< <tt>\b 0x4C0:</tt> CSI2_REVA RXBYTECLKHS_INV Register */
    __R  uint32_t rsv_0x4c4_0x4ff[15];
    __IO uint32_t vfifo_cfg0;           /**< <tt>\b 0x500:</tt> CSI2_REVA VFIFO_CFG0 Register */
    __IO uint32_t vfifo_cfg1;           /**< <tt>\b 0x504:</tt> CSI2_REVA VFIFO_CFG1 Register */
    __IO uint32_t vfifo_ctrl;           /**< <tt>\b 0x508:</tt> CSI2_REVA VFIFO_CTRL Register */
    __IO uint32_t vfifo_sts;            /**< <tt>\b 0x50C:</tt> CSI2_REVA VFIFO_STS Register */
    __IO uint32_t vfifo_line_num;       /**< <tt>\b 0x510:</tt> CSI2_REVA VFIFO_LINE_NUM Register */
    __IO uint32_t vfifo_pixel_num;      /**< <tt>\b 0x514:</tt> CSI2_REVA VFIFO_PIXEL_NUM Register */
    __IO uint32_t vfifo_line_cnt;       /**< <tt>\b 0x518:</tt> CSI2_REVA VFIFO_LINE_CNT Register */
    __IO uint32_t vfifo_pixel_cnt;      /**< <tt>\b 0x51C:</tt> CSI2_REVA VFIFO_PIXEL_CNT Register */
    __IO uint32_t vfifo_frame_sts;      /**< <tt>\b 0x520:</tt> CSI2_REVA VFIFO_FRAME_STS Register */
    __IO uint32_t vfifo_raw_ctrl;       /**< <tt>\b 0x524:</tt> CSI2_REVA VFIFO_RAW_CTRL Register */
    __IO uint32_t vfifo_raw_buf0_addr;  /**< <tt>\b 0x528:</tt> CSI2_REVA VFIFO_RAW_BUF0_ADDR Register */
    __IO uint32_t vfifo_raw_buf1_addr;  /**< <tt>\b 0x52C:</tt> CSI2_REVA VFIFO_RAW_BUF1_ADDR Register */
    __IO uint32_t vfifo_ahbm_ctrl;      /**< <tt>\b 0x530:</tt> CSI2_REVA VFIFO_AHBM_CTRL Register */
    __IO uint32_t vfifo_ahbm_sts;       /**< <tt>\b 0x534:</tt> CSI2_REVA VFIFO_AHBM_STS Register */
    __IO uint32_t vfifo_ahbm_start_addr; /**< <tt>\b 0x538:</tt> CSI2_REVA VFIFO_AHBM_START_ADDR Register */
    __IO uint32_t vfifo_ahbm_addr_range; /**< <tt>\b 0x53C:</tt> CSI2_REVA VFIFO_AHBM_ADDR_RANGE Register */
    __IO uint32_t vfifo_ahbm_max_trans; /**< <tt>\b 0x540:</tt> CSI2_REVA VFIFO_AHBM_MAX_TRANS Register */
    __IO uint32_t vfifo_ahbm_trans_cnt; /**< <tt>\b 0x544:</tt> CSI2_REVA VFIFO_AHBM_TRANS_CNT Register */
    __R  uint32_t rsv_0x548_0x5ff[46];
    __IO uint32_t rx_eint_vff_ie;       /**< <tt>\b 0x600:</tt> CSI2_REVA RX_EINT_VFF_IE Register */
    __IO uint32_t rx_eint_vff_if;       /**< <tt>\b 0x604:</tt> CSI2_REVA RX_EINT_VFF_IF Register */
    __IO uint32_t rx_eint_ppi_ie;       /**< <tt>\b 0x608:</tt> CSI2_REVA RX_EINT_PPI_IE Register */
    __IO uint32_t rx_eint_ppi_if;       /**< <tt>\b 0x60C:</tt> CSI2_REVA RX_EINT_PPI_IF Register */
    __IO uint32_t rx_eint_ctrl_ie;      /**< <tt>\b 0x610:</tt> CSI2_REVA RX_EINT_CTRL_IE Register */
    __IO uint32_t rx_eint_ctrl_if;      /**< <tt>\b 0x614:</tt> CSI2_REVA RX_EINT_CTRL_IF Register */
    __R  uint32_t rsv_0x618_0x6ff[58];
    __IO uint32_t ppi_stopstate;        /**< <tt>\b 0x700:</tt> CSI2_REVA PPI_STOPSTATE Register */
    __IO uint32_t ppi_turnaround_cfg;   /**< <tt>\b 0x704:</tt> CSI2_REVA PPI_TURNAROUND_CFG Register */
} mxc_csi2_reva_regs_t;

/* Register offsets for module CSI2_REVA */
/**
 * @ingroup    csi2_reva_registers
 * @defgroup   CSI2_REVA_Register_Offsets Register Offsets
 * @brief      CSI2_REVA Peripheral Register Offsets from the CSI2_REVA Base Peripheral Address.
 * @{
 */
 #define MXC_R_CSI2_REVA_CFG_NUM_LANES      ((uint32_t)0x00000000UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_CLK_LANE_EN    ((uint32_t)0x00000004UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_DATA_LANE_EN   ((uint32_t)0x00000008UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_FLUSH_COUNT    ((uint32_t)0x0000000CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_BIT_ERR        ((uint32_t)0x00000010UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_CSI2_REVA_IRQ_STATUS         ((uint32_t)0x00000014UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_CSI2_REVA_IRQ_ENABLE         ((uint32_t)0x00000018UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0018</tt> */ 
 #define MXC_R_CSI2_REVA_IRQ_CLR            ((uint32_t)0x0000001CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x001C</tt> */ 
 #define MXC_R_CSI2_REVA_ULPS_CLK_STATUS    ((uint32_t)0x00000020UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0020</tt> */ 
 #define MXC_R_CSI2_REVA_ULPS_STATUS        ((uint32_t)0x00000024UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0024</tt> */ 
 #define MXC_R_CSI2_REVA_ULPS_CLK_MARK_STATUS ((uint32_t)0x00000028UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0028</tt> */ 
 #define MXC_R_CSI2_REVA_ULPS_MARK_STATUS   ((uint32_t)0x0000002CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x002C</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_ERRSOT_HS      ((uint32_t)0x00000030UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0030</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_ERRSOTSYNC_HS  ((uint32_t)0x00000034UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0034</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_ERRESC         ((uint32_t)0x00000038UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0038</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_ERRSYNCESC     ((uint32_t)0x0000003CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x003C</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_ERRCONTROL     ((uint32_t)0x00000040UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0040</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_CPHY_EN        ((uint32_t)0x00000044UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0044</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_PPI_16_EN      ((uint32_t)0x00000048UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0048</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_PACKET_INTERFACE_EN ((uint32_t)0x0000004CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x004C</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VCX_EN         ((uint32_t)0x00000050UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0050</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_BYTE_DATA_FORMAT ((uint32_t)0x00000054UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0054</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_DISABLE_PAYLOAD_0 ((uint32_t)0x00000058UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0058</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_DISABLE_PAYLOAD_1 ((uint32_t)0x0000005CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x005C</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_IGNORE_VC  ((uint32_t)0x00000080UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0080</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_VC         ((uint32_t)0x00000084UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0084</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_P_FIFO_SEND_LEVEL ((uint32_t)0x00000088UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0088</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_VSYNC      ((uint32_t)0x0000008CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x008C</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_HSYNC_FP   ((uint32_t)0x00000090UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0090</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_HSYNC      ((uint32_t)0x00000094UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0094</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_VID_HSYNC_BP   ((uint32_t)0x00000098UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0098</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_DATABUS16_SEL  ((uint32_t)0x00000400UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0400</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_D0_SWAP_SEL    ((uint32_t)0x00000404UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0404</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_D1_SWAP_SEL    ((uint32_t)0x00000408UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0408</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_D2_SWAP_SEL    ((uint32_t)0x0000040CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x040C</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_D3_SWAP_SEL    ((uint32_t)0x00000410UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0410</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_C0_SWAP_SEL    ((uint32_t)0x00000414UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0414</tt> */ 
 #define MXC_R_CSI2_REVA_CFG_DPDN_SWAP      ((uint32_t)0x00000418UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0418</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CFGCLK_1US_CNT  ((uint32_t)0x0000041CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x041C</tt> */ 
 #define MXC_R_CSI2_REVA_RG_HSRX_CLK_PRE_TIME_GRP0 ((uint32_t)0x00000420UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0420</tt> */ 
 #define MXC_R_CSI2_REVA_RG_HSRX_DATA_PRE_TIME_GRP0 ((uint32_t)0x00000424UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0424</tt> */ 
 #define MXC_R_CSI2_REVA_RESET_DESKEW       ((uint32_t)0x00000428UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0428</tt> */ 
 #define MXC_R_CSI2_REVA_PMA_RDY            ((uint32_t)0x0000042CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x042C</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW00         ((uint32_t)0x00000430UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0430</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW01         ((uint32_t)0x00000434UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0434</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW02         ((uint32_t)0x00000438UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0438</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW03         ((uint32_t)0x0000043CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x043C</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW04         ((uint32_t)0x00000440UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0440</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW05         ((uint32_t)0x00000444UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0444</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW06         ((uint32_t)0x00000448UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0448</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW07         ((uint32_t)0x0000044CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x044C</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW08         ((uint32_t)0x00000450UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0450</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW09         ((uint32_t)0x00000454UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0454</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW0A         ((uint32_t)0x00000458UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0458</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW0B         ((uint32_t)0x0000045CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x045C</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW0C         ((uint32_t)0x00000460UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0460</tt> */ 
 #define MXC_R_CSI2_REVA_XCFGI_DW0D         ((uint32_t)0x00000464UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0464</tt> */ 
 #define MXC_R_CSI2_REVA_GPIO_MODE          ((uint32_t)0x00000468UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0468</tt> */ 
 #define MXC_R_CSI2_REVA_GPIO_DP_IE         ((uint32_t)0x0000046CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x046C</tt> */ 
 #define MXC_R_CSI2_REVA_GPIO_DN_IE         ((uint32_t)0x00000470UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0470</tt> */ 
 #define MXC_R_CSI2_REVA_GPIO_DP_C          ((uint32_t)0x00000474UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0474</tt> */ 
 #define MXC_R_CSI2_REVA_GPIO_DN_C          ((uint32_t)0x00000478UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0478</tt> */ 
 #define MXC_R_CSI2_REVA_VCONTROL           ((uint32_t)0x0000047CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x047C</tt> */ 
 #define MXC_R_CSI2_REVA_MPSOV1             ((uint32_t)0x00000480UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0480</tt> */ 
 #define MXC_R_CSI2_REVA_MPSOV2             ((uint32_t)0x00000484UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0484</tt> */ 
 #define MXC_R_CSI2_REVA_MPSOV3             ((uint32_t)0x00000488UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0488</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_DSIRX_EN   ((uint32_t)0x00000490UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0490</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_L012_SUBLVDS_EN ((uint32_t)0x00000494UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0494</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_L012_HSRT_CTRL ((uint32_t)0x00000498UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0498</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_BISTHS_PLL_EN ((uint32_t)0x0000049CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x049C</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2 ((uint32_t)0x000004A0UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04A0</tt> */ 
 #define MXC_R_CSI2_REVA_RG_CDRX_BISTHS_PLL_FBK_INT ((uint32_t)0x000004A4UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04A4</tt> */ 
 #define MXC_R_CSI2_REVA_DBG1_MUX_SEL       ((uint32_t)0x000004A8UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04A8</tt> */ 
 #define MXC_R_CSI2_REVA_DBG2_MUX_SEL       ((uint32_t)0x000004ACUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04AC</tt> */ 
 #define MXC_R_CSI2_REVA_DBG1_MUX_DOUT      ((uint32_t)0x000004B0UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04B0</tt> */ 
 #define MXC_R_CSI2_REVA_DBG2_MUX_DOUT      ((uint32_t)0x000004B4UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04B4</tt> */ 
 #define MXC_R_CSI2_REVA_AON_POWER_READY_N  ((uint32_t)0x000004B8UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04B8</tt> */ 
 #define MXC_R_CSI2_REVA_DPHY_RST_N         ((uint32_t)0x000004BCUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04BC</tt> */ 
 #define MXC_R_CSI2_REVA_RXBYTECLKHS_INV    ((uint32_t)0x000004C0UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x04C0</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_CFG0         ((uint32_t)0x00000500UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0500</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_CFG1         ((uint32_t)0x00000504UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0504</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_CTRL         ((uint32_t)0x00000508UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0508</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_STS          ((uint32_t)0x0000050CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x050C</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_LINE_NUM     ((uint32_t)0x00000510UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0510</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_PIXEL_NUM    ((uint32_t)0x00000514UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0514</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_LINE_CNT     ((uint32_t)0x00000518UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0518</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_PIXEL_CNT    ((uint32_t)0x0000051CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x051C</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_FRAME_STS    ((uint32_t)0x00000520UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0520</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_RAW_CTRL     ((uint32_t)0x00000524UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0524</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_RAW_BUF0_ADDR ((uint32_t)0x00000528UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0528</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_RAW_BUF1_ADDR ((uint32_t)0x0000052CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x052C</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_CTRL    ((uint32_t)0x00000530UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0530</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_STS     ((uint32_t)0x00000534UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0534</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_START_ADDR ((uint32_t)0x00000538UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0538</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_ADDR_RANGE ((uint32_t)0x0000053CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x053C</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_MAX_TRANS ((uint32_t)0x00000540UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0540</tt> */ 
 #define MXC_R_CSI2_REVA_VFIFO_AHBM_TRANS_CNT ((uint32_t)0x00000544UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0544</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_VFF_IE     ((uint32_t)0x00000600UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0600</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_VFF_IF     ((uint32_t)0x00000604UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0604</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_PPI_IE     ((uint32_t)0x00000608UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0608</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_PPI_IF     ((uint32_t)0x0000060CUL) /**< Offset from CSI2_REVA Base Address: <tt> 0x060C</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_CTRL_IE    ((uint32_t)0x00000610UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0610</tt> */ 
 #define MXC_R_CSI2_REVA_RX_EINT_CTRL_IF    ((uint32_t)0x00000614UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0614</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_STOPSTATE      ((uint32_t)0x00000700UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0700</tt> */ 
 #define MXC_R_CSI2_REVA_PPI_TURNAROUND_CFG ((uint32_t)0x00000704UL) /**< Offset from CSI2_REVA Base Address: <tt> 0x0704</tt> */ 
/**@} end of group csi2_reva_registers */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_NUM_LANES CSI2_REVA_CFG_NUM_LANES
 * @brief    CFG_NUM_LANES.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_NUM_LANES_LANES_POS        0 /**< CFG_NUM_LANES_LANES Position */
 #define MXC_F_CSI2_REVA_CFG_NUM_LANES_LANES            ((uint32_t)(0xFUL << MXC_F_CSI2_REVA_CFG_NUM_LANES_LANES_POS)) /**< CFG_NUM_LANES_LANES Mask */

/**@} end of group CSI2_REVA_CFG_NUM_LANES_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_CLK_LANE_EN CSI2_REVA_CFG_CLK_LANE_EN
 * @brief    CFG_CLK_LANE_EN.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_CLK_LANE_EN_EN_POS         0 /**< CFG_CLK_LANE_EN_EN Position */
 #define MXC_F_CSI2_REVA_CFG_CLK_LANE_EN_EN             ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_CLK_LANE_EN_EN_POS)) /**< CFG_CLK_LANE_EN_EN Mask */

/**@} end of group CSI2_REVA_CFG_CLK_LANE_EN_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_DATA_LANE_EN CSI2_REVA_CFG_DATA_LANE_EN
 * @brief    CFG_DATA_LANE_EN.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_DATA_LANE_EN_EN_POS        0 /**< CFG_DATA_LANE_EN_EN Position */
 #define MXC_F_CSI2_REVA_CFG_DATA_LANE_EN_EN            ((uint32_t)(0xFFUL << MXC_F_CSI2_REVA_CFG_DATA_LANE_EN_EN_POS)) /**< CFG_DATA_LANE_EN_EN Mask */

/**@} end of group CSI2_REVA_CFG_DATA_LANE_EN_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_FLUSH_COUNT CSI2_REVA_CFG_FLUSH_COUNT
 * @brief    CFG_FLUSH_COUNT.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_FLUSH_COUNT_COUNT_POS      0 /**< CFG_FLUSH_COUNT_COUNT Position */
 #define MXC_F_CSI2_REVA_CFG_FLUSH_COUNT_COUNT          ((uint32_t)(0xFUL << MXC_F_CSI2_REVA_CFG_FLUSH_COUNT_COUNT_POS)) /**< CFG_FLUSH_COUNT_COUNT Mask */

/**@} end of group CSI2_REVA_CFG_FLUSH_COUNT_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_BIT_ERR CSI2_REVA_CFG_BIT_ERR
 * @brief    CFG_BIT_ERR.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_MBE_POS            0 /**< CFG_BIT_ERR_MBE Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_MBE                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_BIT_ERR_MBE_POS)) /**< CFG_BIT_ERR_MBE Mask */

 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_SBE_POS            1 /**< CFG_BIT_ERR_SBE Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_SBE                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_BIT_ERR_SBE_POS)) /**< CFG_BIT_ERR_SBE Mask */

 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_HEADER_POS         2 /**< CFG_BIT_ERR_HEADER Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_HEADER             ((uint32_t)(0x1FUL << MXC_F_CSI2_REVA_CFG_BIT_ERR_HEADER_POS)) /**< CFG_BIT_ERR_HEADER Mask */

 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_CRC_POS            7 /**< CFG_BIT_ERR_CRC Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_CRC                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_BIT_ERR_CRC_POS)) /**< CFG_BIT_ERR_CRC Mask */

 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_SEND_LVL_POS 8 /**< CFG_BIT_ERR_VID_ERR_SEND_LVL Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_SEND_LVL   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_SEND_LVL_POS)) /**< CFG_BIT_ERR_VID_ERR_SEND_LVL Mask */

 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_FIFO_WR_OV_POS 9 /**< CFG_BIT_ERR_VID_ERR_FIFO_WR_OV Position */
 #define MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_FIFO_WR_OV ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_BIT_ERR_VID_ERR_FIFO_WR_OV_POS)) /**< CFG_BIT_ERR_VID_ERR_FIFO_WR_OV Mask */

/**@} end of group CSI2_REVA_CFG_BIT_ERR_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_IRQ_STATUS CSI2_REVA_IRQ_STATUS
 * @brief    IRQ_STATUS.
 * @{
 */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_CRC_POS             0 /**< IRQ_STATUS_CRC Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_CRC                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_CRC_POS)) /**< IRQ_STATUS_CRC Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_SBE_POS             1 /**< IRQ_STATUS_SBE Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_SBE                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_SBE_POS)) /**< IRQ_STATUS_SBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_MBE_POS             2 /**< IRQ_STATUS_MBE Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_MBE                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_MBE_POS)) /**< IRQ_STATUS_MBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_ACTIVE_POS     3 /**< IRQ_STATUS_ULPS_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_ACTIVE         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_ACTIVE_POS)) /**< IRQ_STATUS_ULPS_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_MARK_ACTIVE_POS 4 /**< IRQ_STATUS_ULPS_MARK_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_MARK_ACTIVE    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_ULPS_MARK_ACTIVE_POS)) /**< IRQ_STATUS_ULPS_MARK_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_SEND_LVL_POS 5 /**< IRQ_STATUS_VID_ERR_SEND_LVL Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_SEND_LVL    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_SEND_LVL_POS)) /**< IRQ_STATUS_VID_ERR_SEND_LVL Mask */

 #define MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_FIFO_WR_OV_POS 6 /**< IRQ_STATUS_VID_ERR_FIFO_WR_OV Position */
 #define MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_FIFO_WR_OV  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_STATUS_VID_ERR_FIFO_WR_OV_POS)) /**< IRQ_STATUS_VID_ERR_FIFO_WR_OV Mask */

/**@} end of group CSI2_REVA_IRQ_STATUS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_IRQ_ENABLE CSI2_REVA_IRQ_ENABLE
 * @brief    IRQ_ENABLE.
 * @{
 */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_CRC_POS             0 /**< IRQ_ENABLE_CRC Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_CRC                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_CRC_POS)) /**< IRQ_ENABLE_CRC Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_SBE_POS             1 /**< IRQ_ENABLE_SBE Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_SBE                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_SBE_POS)) /**< IRQ_ENABLE_SBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_MBE_POS             2 /**< IRQ_ENABLE_MBE Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_MBE                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_MBE_POS)) /**< IRQ_ENABLE_MBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_ACTIVE_POS     3 /**< IRQ_ENABLE_ULPS_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_ACTIVE         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_ACTIVE_POS)) /**< IRQ_ENABLE_ULPS_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_MARK_ACTIVE_POS 4 /**< IRQ_ENABLE_ULPS_MARK_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_MARK_ACTIVE    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_ULPS_MARK_ACTIVE_POS)) /**< IRQ_ENABLE_ULPS_MARK_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_SEND_LVL_POS 5 /**< IRQ_ENABLE_VID_ERR_SEND_LVL Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_SEND_LVL    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_SEND_LVL_POS)) /**< IRQ_ENABLE_VID_ERR_SEND_LVL Mask */

 #define MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_FIFO_WR_OV_POS 6 /**< IRQ_ENABLE_VID_ERR_FIFO_WR_OV Position */
 #define MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_FIFO_WR_OV  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_ENABLE_VID_ERR_FIFO_WR_OV_POS)) /**< IRQ_ENABLE_VID_ERR_FIFO_WR_OV Mask */

/**@} end of group CSI2_REVA_IRQ_ENABLE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_IRQ_CLR CSI2_REVA_IRQ_CLR
 * @brief    IRQ_CLR.
 * @{
 */
 #define MXC_F_CSI2_REVA_IRQ_CLR_CRC_POS                0 /**< IRQ_CLR_CRC Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_CRC                    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_CRC_POS)) /**< IRQ_CLR_CRC Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_SBE_POS                1 /**< IRQ_CLR_SBE Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_SBE                    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_SBE_POS)) /**< IRQ_CLR_SBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_MBE_POS                2 /**< IRQ_CLR_MBE Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_MBE                    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_MBE_POS)) /**< IRQ_CLR_MBE Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_ULPS_ACTIVE_POS        3 /**< IRQ_CLR_ULPS_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_ULPS_ACTIVE            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_ULPS_ACTIVE_POS)) /**< IRQ_CLR_ULPS_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_ULPS_MARK_ACTIVE_POS   4 /**< IRQ_CLR_ULPS_MARK_ACTIVE Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_ULPS_MARK_ACTIVE       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_ULPS_MARK_ACTIVE_POS)) /**< IRQ_CLR_ULPS_MARK_ACTIVE Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_SEND_LVL_POS   5 /**< IRQ_CLR_VID_ERR_SEND_LVL Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_SEND_LVL       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_SEND_LVL_POS)) /**< IRQ_CLR_VID_ERR_SEND_LVL Mask */

 #define MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_FIFO_WR_OV_POS 6 /**< IRQ_CLR_VID_ERR_FIFO_WR_OV Position */
 #define MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_FIFO_WR_OV     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_IRQ_CLR_VID_ERR_FIFO_WR_OV_POS)) /**< IRQ_CLR_VID_ERR_FIFO_WR_OV Mask */

/**@} end of group CSI2_REVA_IRQ_CLR_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_ULPS_CLK_STATUS CSI2_REVA_ULPS_CLK_STATUS
 * @brief    ULPS_CLK_STATUS.
 * @{
 */
 #define MXC_F_CSI2_REVA_ULPS_CLK_STATUS_FIFO_POS       0 /**< ULPS_CLK_STATUS_FIFO Position */
 #define MXC_F_CSI2_REVA_ULPS_CLK_STATUS_FIFO           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_CLK_STATUS_FIFO_POS)) /**< ULPS_CLK_STATUS_FIFO Mask */

/**@} end of group CSI2_REVA_ULPS_CLK_STATUS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_ULPS_STATUS CSI2_REVA_ULPS_STATUS
 * @brief    ULPS_STATUS.
 * @{
 */
 #define MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE0_POS     0 /**< ULPS_STATUS_DATA_LANE0 Position */
 #define MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE0         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE0_POS)) /**< ULPS_STATUS_DATA_LANE0 Mask */

 #define MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE1_POS     1 /**< ULPS_STATUS_DATA_LANE1 Position */
 #define MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE1         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_STATUS_DATA_LANE1_POS)) /**< ULPS_STATUS_DATA_LANE1 Mask */

/**@} end of group CSI2_REVA_ULPS_STATUS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_ULPS_CLK_MARK_STATUS CSI2_REVA_ULPS_CLK_MARK_STATUS
 * @brief    ULPS_CLK_MARK_STATUS.
 * @{
 */
 #define MXC_F_CSI2_REVA_ULPS_CLK_MARK_STATUS_CLK_LANE_POS 0 /**< ULPS_CLK_MARK_STATUS_CLK_LANE Position */
 #define MXC_F_CSI2_REVA_ULPS_CLK_MARK_STATUS_CLK_LANE  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_CLK_MARK_STATUS_CLK_LANE_POS)) /**< ULPS_CLK_MARK_STATUS_CLK_LANE Mask */

/**@} end of group CSI2_REVA_ULPS_CLK_MARK_STATUS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_ULPS_MARK_STATUS CSI2_REVA_ULPS_MARK_STATUS
 * @brief    ULPS_MARK_STATUS.
 * @{
 */
 #define MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE0_POS 0 /**< ULPS_MARK_STATUS_DATA_LANE0 Position */
 #define MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE0    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE0_POS)) /**< ULPS_MARK_STATUS_DATA_LANE0 Mask */

 #define MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE1_POS 1 /**< ULPS_MARK_STATUS_DATA_LANE1 Position */
 #define MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE1    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_ULPS_MARK_STATUS_DATA_LANE1_POS)) /**< ULPS_MARK_STATUS_DATA_LANE1 Mask */

/**@} end of group CSI2_REVA_ULPS_MARK_STATUS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_DISABLE_PAYLOAD_0 CSI2_REVA_CFG_DISABLE_PAYLOAD_0
 * @brief    CFG_DISABLE_PAYLOAD_0.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_NULL_POS 0 /**< CFG_DISABLE_PAYLOAD_0_NULL Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_NULL     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_NULL_POS)) /**< CFG_DISABLE_PAYLOAD_0_NULL Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_BLANK_POS 1 /**< CFG_DISABLE_PAYLOAD_0_BLANK Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_BLANK    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_BLANK_POS)) /**< CFG_DISABLE_PAYLOAD_0_BLANK Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_EMBEDDED_POS 2 /**< CFG_DISABLE_PAYLOAD_0_EMBEDDED Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_EMBEDDED ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_EMBEDDED_POS)) /**< CFG_DISABLE_PAYLOAD_0_EMBEDDED Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_POS 8 /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_POS 9 /**< CFG_DISABLE_PAYLOAD_0_YUV420_10BIT Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV420_10BIT Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG_POS 10 /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP_POS 12 /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP_POS 13 /**< CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_8BIT_POS 14 /**< CFG_DISABLE_PAYLOAD_0_YUV422_8BIT Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_8BIT ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_8BIT_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV422_8BIT Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_10BIT_POS 15 /**< CFG_DISABLE_PAYLOAD_0_YUV422_10BIT Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_10BIT ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_YUV422_10BIT_POS)) /**< CFG_DISABLE_PAYLOAD_0_YUV422_10BIT Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB444_POS 16 /**< CFG_DISABLE_PAYLOAD_0_RGB444 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB444   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB444_POS)) /**< CFG_DISABLE_PAYLOAD_0_RGB444 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB555_POS 17 /**< CFG_DISABLE_PAYLOAD_0_RGB555 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB555   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB555_POS)) /**< CFG_DISABLE_PAYLOAD_0_RGB555 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB565_POS 18 /**< CFG_DISABLE_PAYLOAD_0_RGB565 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB565   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB565_POS)) /**< CFG_DISABLE_PAYLOAD_0_RGB565 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB666_POS 19 /**< CFG_DISABLE_PAYLOAD_0_RGB666 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB666   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB666_POS)) /**< CFG_DISABLE_PAYLOAD_0_RGB666 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB888_POS 20 /**< CFG_DISABLE_PAYLOAD_0_RGB888 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB888   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RGB888_POS)) /**< CFG_DISABLE_PAYLOAD_0_RGB888 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW6_POS 24 /**< CFG_DISABLE_PAYLOAD_0_RAW6 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW6     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW6_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW6 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW7_POS 25 /**< CFG_DISABLE_PAYLOAD_0_RAW7 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW7     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW7_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW7 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW8_POS 26 /**< CFG_DISABLE_PAYLOAD_0_RAW8 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW8     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW8_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW8 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW10_POS 27 /**< CFG_DISABLE_PAYLOAD_0_RAW10 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW10    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW10_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW10 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW12_POS 28 /**< CFG_DISABLE_PAYLOAD_0_RAW12 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW12    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW12_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW12 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW14_POS 29 /**< CFG_DISABLE_PAYLOAD_0_RAW14 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW14    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW14_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW14 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW16_POS 30 /**< CFG_DISABLE_PAYLOAD_0_RAW16 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW16    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW16_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW16 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW20_POS 31 /**< CFG_DISABLE_PAYLOAD_0_RAW20 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW20    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_0_RAW20_POS)) /**< CFG_DISABLE_PAYLOAD_0_RAW20 Mask */

/**@} end of group CSI2_REVA_CFG_DISABLE_PAYLOAD_0_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_DISABLE_PAYLOAD_1 CSI2_REVA_CFG_DISABLE_PAYLOAD_1
 * @brief    CFG_DISABLE_PAYLOAD_1.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30_POS 0 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31_POS 1 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32_POS 2 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33_POS 3 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34_POS 4 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35_POS 5 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36_POS 6 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36 Mask */

 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37_POS 7 /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37 Position */
 #define MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37_POS)) /**< CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37 Mask */

/**@} end of group CSI2_REVA_CFG_DISABLE_PAYLOAD_1_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_DATABUS16_SEL CSI2_REVA_CFG_DATABUS16_SEL
 * @brief    CFG_DATABUS16_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_DATABUS16_SEL_EN_POS       0 /**< CFG_DATABUS16_SEL_EN Position */
 #define MXC_F_CSI2_REVA_CFG_DATABUS16_SEL_EN           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DATABUS16_SEL_EN_POS)) /**< CFG_DATABUS16_SEL_EN Mask */

/**@} end of group CSI2_REVA_CFG_DATABUS16_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_D0_SWAP_SEL CSI2_REVA_CFG_D0_SWAP_SEL
 * @brief    CFG_D0_SWAP_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS        0 /**< CFG_D0_SWAP_SEL_SRC Position */
 #define MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC            ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS)) /**< CFG_D0_SWAP_SEL_SRC Mask */
 #define MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L0 ((uint32_t)0x0UL) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L0 Value */
 #define MXC_S_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L0 (MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L0 << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L0 Setting */
 #define MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L1 ((uint32_t)0x1UL) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L1 Value */
 #define MXC_S_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L1 (MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L1 << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L1 Setting */
 #define MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L2 ((uint32_t)0x2UL) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L2 Value */
 #define MXC_S_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L2 (MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L2 << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L2 Setting */
 #define MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L3 ((uint32_t)0x3UL) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L3 Value */
 #define MXC_S_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L3 (MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L3 << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L3 Setting */
 #define MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L4 ((uint32_t)0x4UL) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L4 Value */
 #define MXC_S_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L4 (MXC_V_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L4 << MXC_F_CSI2_REVA_CFG_D0_SWAP_SEL_SRC_POS) /**< CFG_D0_SWAP_SEL_SRC_PAD_CDRX_L4 Setting */

/**@} end of group CSI2_REVA_CFG_D0_SWAP_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_D1_SWAP_SEL CSI2_REVA_CFG_D1_SWAP_SEL
 * @brief    CFG_D1_SWAP_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS        0 /**< CFG_D1_SWAP_SEL_SRC Position */
 #define MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC            ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS)) /**< CFG_D1_SWAP_SEL_SRC Mask */
 #define MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L0 ((uint32_t)0x0UL) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L0 Value */
 #define MXC_S_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L0 (MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L0 << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L0 Setting */
 #define MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L1 ((uint32_t)0x1UL) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L1 Value */
 #define MXC_S_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L1 (MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L1 << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L1 Setting */
 #define MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L2 ((uint32_t)0x2UL) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L2 Value */
 #define MXC_S_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L2 (MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L2 << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L2 Setting */
 #define MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L3 ((uint32_t)0x3UL) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L3 Value */
 #define MXC_S_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L3 (MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L3 << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L3 Setting */
 #define MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L4 ((uint32_t)0x4UL) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L4 Value */
 #define MXC_S_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L4 (MXC_V_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L4 << MXC_F_CSI2_REVA_CFG_D1_SWAP_SEL_SRC_POS) /**< CFG_D1_SWAP_SEL_SRC_PAD_CDRX_L4 Setting */

/**@} end of group CSI2_REVA_CFG_D1_SWAP_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_D2_SWAP_SEL CSI2_REVA_CFG_D2_SWAP_SEL
 * @brief    CFG_D2_SWAP_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS        0 /**< CFG_D2_SWAP_SEL_SRC Position */
 #define MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC            ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS)) /**< CFG_D2_SWAP_SEL_SRC Mask */
 #define MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L0 ((uint32_t)0x0UL) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L0 Value */
 #define MXC_S_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L0 (MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L0 << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L0 Setting */
 #define MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L1 ((uint32_t)0x1UL) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L1 Value */
 #define MXC_S_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L1 (MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L1 << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L1 Setting */
 #define MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L2 ((uint32_t)0x2UL) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L2 Value */
 #define MXC_S_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L2 (MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L2 << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L2 Setting */
 #define MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L3 ((uint32_t)0x3UL) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L3 Value */
 #define MXC_S_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L3 (MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L3 << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L3 Setting */
 #define MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L4 ((uint32_t)0x4UL) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L4 Value */
 #define MXC_S_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L4 (MXC_V_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L4 << MXC_F_CSI2_REVA_CFG_D2_SWAP_SEL_SRC_POS) /**< CFG_D2_SWAP_SEL_SRC_PAD_CDRX_L4 Setting */

/**@} end of group CSI2_REVA_CFG_D2_SWAP_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_D3_SWAP_SEL CSI2_REVA_CFG_D3_SWAP_SEL
 * @brief    CFG_D3_SWAP_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS        0 /**< CFG_D3_SWAP_SEL_SRC Position */
 #define MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC            ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS)) /**< CFG_D3_SWAP_SEL_SRC Mask */
 #define MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L0 ((uint32_t)0x0UL) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L0 Value */
 #define MXC_S_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L0 (MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L0 << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L0 Setting */
 #define MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L1 ((uint32_t)0x1UL) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L1 Value */
 #define MXC_S_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L1 (MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L1 << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L1 Setting */
 #define MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L2 ((uint32_t)0x2UL) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L2 Value */
 #define MXC_S_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L2 (MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L2 << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L2 Setting */
 #define MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L3 ((uint32_t)0x3UL) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L3 Value */
 #define MXC_S_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L3 (MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L3 << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L3 Setting */
 #define MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L4 ((uint32_t)0x4UL) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L4 Value */
 #define MXC_S_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L4 (MXC_V_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L4 << MXC_F_CSI2_REVA_CFG_D3_SWAP_SEL_SRC_POS) /**< CFG_D3_SWAP_SEL_SRC_PAD_CDRX_L4 Setting */

/**@} end of group CSI2_REVA_CFG_D3_SWAP_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_C0_SWAP_SEL CSI2_REVA_CFG_C0_SWAP_SEL
 * @brief    CFG_C0_SWAP_SEL.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS        0 /**< CFG_C0_SWAP_SEL_SRC Position */
 #define MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC            ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS)) /**< CFG_C0_SWAP_SEL_SRC Mask */
 #define MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L0 ((uint32_t)0x0UL) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L0 Value */
 #define MXC_S_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L0 (MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L0 << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L0 Setting */
 #define MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L1 ((uint32_t)0x1UL) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L1 Value */
 #define MXC_S_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L1 (MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L1 << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L1 Setting */
 #define MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L2 ((uint32_t)0x2UL) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L2 Value */
 #define MXC_S_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L2 (MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L2 << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L2 Setting */
 #define MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L3 ((uint32_t)0x3UL) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L3 Value */
 #define MXC_S_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L3 (MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L3 << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L3 Setting */
 #define MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L4 ((uint32_t)0x4UL) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L4 Value */
 #define MXC_S_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L4 (MXC_V_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L4 << MXC_F_CSI2_REVA_CFG_C0_SWAP_SEL_SRC_POS) /**< CFG_C0_SWAP_SEL_SRC_PAD_CDRX_L4 Setting */

/**@} end of group CSI2_REVA_CFG_C0_SWAP_SEL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_CFG_DPDN_SWAP CSI2_REVA_CFG_DPDN_SWAP
 * @brief    CFG_DPDN_SWAP.
 * @{
 */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE0_POS 0 /**< CFG_DPDN_SWAP_SWAP_DATA_LANE0 Position */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE0  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE0_POS)) /**< CFG_DPDN_SWAP_SWAP_DATA_LANE0 Mask */

 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE1_POS 1 /**< CFG_DPDN_SWAP_SWAP_DATA_LANE1 Position */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE1  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE1_POS)) /**< CFG_DPDN_SWAP_SWAP_DATA_LANE1 Mask */

 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE2_POS 2 /**< CFG_DPDN_SWAP_SWAP_DATA_LANE2 Position */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE2  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE2_POS)) /**< CFG_DPDN_SWAP_SWAP_DATA_LANE2 Mask */

 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE3_POS 3 /**< CFG_DPDN_SWAP_SWAP_DATA_LANE3 Position */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE3  ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_DATA_LANE3_POS)) /**< CFG_DPDN_SWAP_SWAP_DATA_LANE3 Mask */

 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_CLK_LANE_POS 4 /**< CFG_DPDN_SWAP_SWAP_CLK_LANE Position */
 #define MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_CLK_LANE    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_CFG_DPDN_SWAP_SWAP_CLK_LANE_POS)) /**< CFG_DPDN_SWAP_SWAP_CLK_LANE Mask */

/**@} end of group CSI2_REVA_CFG_DPDN_SWAP_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RESET_DESKEW CSI2_REVA_RESET_DESKEW
 * @brief    RESET_DESKEW.
 * @{
 */
 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE0_POS    0 /**< RESET_DESKEW_DATA_LANE0 Position */
 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE0        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE0_POS)) /**< RESET_DESKEW_DATA_LANE0 Mask */

 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE1_POS    1 /**< RESET_DESKEW_DATA_LANE1 Position */
 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE1        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE1_POS)) /**< RESET_DESKEW_DATA_LANE1 Mask */

 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE2_POS    2 /**< RESET_DESKEW_DATA_LANE2 Position */
 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE2        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE2_POS)) /**< RESET_DESKEW_DATA_LANE2 Mask */

 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE3_POS    3 /**< RESET_DESKEW_DATA_LANE3 Position */
 #define MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE3        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RESET_DESKEW_DATA_LANE3_POS)) /**< RESET_DESKEW_DATA_LANE3 Mask */

/**@} end of group CSI2_REVA_RESET_DESKEW_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VCONTROL CSI2_REVA_VCONTROL
 * @brief    PMA_RDY.
 * @{
 */
 #define MXC_F_CSI2_REVA_VCONTROL_NORMAL_MODE_POS       0 /**< VCONTROL_NORMAL_MODE Position */
 #define MXC_F_CSI2_REVA_VCONTROL_NORMAL_MODE           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_NORMAL_MODE_POS)) /**< VCONTROL_NORMAL_MODE Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_TEST_POS     1 /**< VCONTROL_LP_RX_DC_TEST Position */
 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_TEST         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_TEST_POS)) /**< VCONTROL_LP_RX_DC_TEST Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_1_POS        2 /**< VCONTROL_LP_RX_DC_1 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_1            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_1_POS)) /**< VCONTROL_LP_RX_DC_1 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_0_POS        3 /**< VCONTROL_LP_RX_DC_0 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_0            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_LP_RX_DC_0_POS)) /**< VCONTROL_LP_RX_DC_0 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_1_POS         4 /**< VCONTROL_CAL_SEN_1 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_1             ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_1_POS)) /**< VCONTROL_CAL_SEN_1 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_0_POS         5 /**< VCONTROL_CAL_SEN_0 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_0             ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_CAL_SEN_0_POS)) /**< VCONTROL_CAL_SEN_0 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_HSRT_0_POS            7 /**< VCONTROL_HSRT_0 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_HSRT_0                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_HSRT_0_POS)) /**< VCONTROL_HSRT_0 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_HSRT_1_POS            8 /**< VCONTROL_HSRT_1 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_HSRT_1                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_HSRT_1_POS)) /**< VCONTROL_HSRT_1 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_PARTBERT_POS    10 /**< VCONTROL_LP_RX_PARTBERT Position */
 #define MXC_F_CSI2_REVA_VCONTROL_LP_RX_PARTBERT        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_LP_RX_PARTBERT_POS)) /**< VCONTROL_LP_RX_PARTBERT Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_HS_INT_LOOPBACK_POS   11 /**< VCONTROL_HS_INT_LOOPBACK Position */
 #define MXC_F_CSI2_REVA_VCONTROL_HS_INT_LOOPBACK       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_HS_INT_LOOPBACK_POS)) /**< VCONTROL_HS_INT_LOOPBACK Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_HS_RX_PARTBERT_POS    27 /**< VCONTROL_HS_RX_PARTBERT Position */
 #define MXC_F_CSI2_REVA_VCONTROL_HS_RX_PARTBERT        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_HS_RX_PARTBERT_POS)) /**< VCONTROL_HS_RX_PARTBERT Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_HS_RX_PRBS9_POS       28 /**< VCONTROL_HS_RX_PRBS9 Position */
 #define MXC_F_CSI2_REVA_VCONTROL_HS_RX_PRBS9           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_HS_RX_PRBS9_POS)) /**< VCONTROL_HS_RX_PRBS9 Mask */

 #define MXC_F_CSI2_REVA_VCONTROL_SUSPEND_MODE_POS      31 /**< VCONTROL_SUSPEND_MODE Position */
 #define MXC_F_CSI2_REVA_VCONTROL_SUSPEND_MODE          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VCONTROL_SUSPEND_MODE_POS)) /**< VCONTROL_SUSPEND_MODE Mask */

/**@} end of group CSI2_REVA_VCONTROL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RG_CDRX_DSIRX_EN CSI2_REVA_RG_CDRX_DSIRX_EN
 * @brief    RG_CDRX_DSIRX_EN.
 * @{
 */
 #define MXC_F_CSI2_REVA_RG_CDRX_DSIRX_EN_RXMODE_POS    0 /**< RG_CDRX_DSIRX_EN_RXMODE Position */
 #define MXC_F_CSI2_REVA_RG_CDRX_DSIRX_EN_RXMODE        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RG_CDRX_DSIRX_EN_RXMODE_POS)) /**< RG_CDRX_DSIRX_EN_RXMODE Mask */

/**@} end of group CSI2_REVA_RG_CDRX_DSIRX_EN_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2 CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2
 * @brief    RG_CDRX_BISTHS_PLL_PRE_DIV2.
 * @{
 */
 #define MXC_F_CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2_RXMODE_POS 0 /**< RG_CDRX_BISTHS_PLL_PRE_DIV2_RXMODE Position */
 #define MXC_F_CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2_RXMODE ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2_RXMODE_POS)) /**< RG_CDRX_BISTHS_PLL_PRE_DIV2_RXMODE Mask */

/**@} end of group CSI2_REVA_RG_CDRX_BISTHS_PLL_PRE_DIV2_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_CFG0 CSI2_REVA_VFIFO_CFG0
 * @brief    Video FIFO Configuration Register 0.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_VC_POS              0 /**< VFIFO_CFG0_VC Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_VC                  ((uint32_t)(0x3UL << MXC_F_CSI2_REVA_VFIFO_CFG0_VC_POS)) /**< VFIFO_CFG0_VC Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS         6 /**< VFIFO_CFG0_DMAMODE Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE             ((uint32_t)(0x3UL << MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS)) /**< VFIFO_CFG0_DMAMODE Mask */
 #define MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_NO_DMA      ((uint32_t)0x0UL) /**< VFIFO_CFG0_DMAMODE_NO_DMA Value */
 #define MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_NO_DMA      (MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_NO_DMA << MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS) /**< VFIFO_CFG0_DMAMODE_NO_DMA Setting */
 #define MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_DMA_REQ     ((uint32_t)0x1UL) /**< VFIFO_CFG0_DMAMODE_DMA_REQ Value */
 #define MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_DMA_REQ     (MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_DMA_REQ << MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS) /**< VFIFO_CFG0_DMAMODE_DMA_REQ Setting */
 #define MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_THD    ((uint32_t)0x2UL) /**< VFIFO_CFG0_DMAMODE_FIFO_THD Value */
 #define MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_THD    (MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_THD << MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS) /**< VFIFO_CFG0_DMAMODE_FIFO_THD Setting */
 #define MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_FULL   ((uint32_t)0x3UL) /**< VFIFO_CFG0_DMAMODE_FIFO_FULL Value */
 #define MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_FULL   (MXC_V_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_FULL << MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE_POS) /**< VFIFO_CFG0_DMAMODE_FIFO_FULL Setting */

 #define MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS         8 /**< VFIFO_CFG0_AHBWAIT Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT             ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS)) /**< VFIFO_CFG0_AHBWAIT Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM_POS          9 /**< VFIFO_CFG0_FIFORM Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM_POS)) /**< VFIFO_CFG0_FIFORM Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE_POS           10 /**< VFIFO_CFG0_ERRDE Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE               ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE_POS)) /**< VFIFO_CFG0_ERRDE Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG0_FBWM_POS            11 /**< VFIFO_CFG0_FBWM Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG0_FBWM                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG0_FBWM_POS)) /**< VFIFO_CFG0_FBWM Mask */

/**@} end of group CSI2_REVA_VFIFO_CFG0_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_CFG1 CSI2_REVA_VFIFO_CFG1
 * @brief    Video FIFO Configuration Register 1.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_AHBWCYC_POS         0 /**< VFIFO_CFG1_AHBWCYC Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_AHBWCYC             ((uint32_t)(0xFFFFUL << MXC_F_CSI2_REVA_VFIFO_CFG1_AHBWCYC_POS)) /**< VFIFO_CFG1_AHBWCYC Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_WAIT_FIRST_FS_POS   16 /**< VFIFO_CFG1_WAIT_FIRST_FS Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_WAIT_FIRST_FS       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_WAIT_FIRST_FS_POS)) /**< VFIFO_CFG1_WAIT_FIRST_FS Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_FRAME_CTRL_POS 17 /**< VFIFO_CFG1_ACCU_FRAME_CTRL Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_FRAME_CTRL     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_FRAME_CTRL_POS)) /**< VFIFO_CFG1_ACCU_FRAME_CTRL Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CTRL_POS  18 /**< VFIFO_CFG1_ACCU_LINE_CTRL Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CTRL      ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CTRL_POS)) /**< VFIFO_CFG1_ACCU_LINE_CTRL Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CNT_POS   19 /**< VFIFO_CFG1_ACCU_LINE_CNT Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CNT       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_LINE_CNT_POS)) /**< VFIFO_CFG1_ACCU_LINE_CNT Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_CNT_POS  20 /**< VFIFO_CFG1_ACCU_PIXEL_CNT Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_CNT      ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_CNT_POS)) /**< VFIFO_CFG1_ACCU_PIXEL_CNT Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_ZERO_POS 21 /**< VFIFO_CFG1_ACCU_PIXEL_ZERO Position */
 #define MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_ZERO     ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CFG1_ACCU_PIXEL_ZERO_POS)) /**< VFIFO_CFG1_ACCU_PIXEL_ZERO Mask */

/**@} end of group CSI2_REVA_VFIFO_CFG1_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_CTRL CSI2_REVA_VFIFO_CTRL
 * @brief    Video FIFO Control Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN_POS          0 /**< VFIFO_CTRL_FIFOEN Position */
 #define MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN_POS)) /**< VFIFO_CTRL_FIFOEN Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CTRL_FLUSH_POS           4 /**< VFIFO_CTRL_FLUSH Position */
 #define MXC_F_CSI2_REVA_VFIFO_CTRL_FLUSH               ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_CTRL_FLUSH_POS)) /**< VFIFO_CTRL_FLUSH Mask */

 #define MXC_F_CSI2_REVA_VFIFO_CTRL_THD_POS             8 /**< VFIFO_CTRL_THD Position */
 #define MXC_F_CSI2_REVA_VFIFO_CTRL_THD                 ((uint32_t)(0x7FUL << MXC_F_CSI2_REVA_VFIFO_CTRL_THD_POS)) /**< VFIFO_CTRL_THD Mask */

/**@} end of group CSI2_REVA_VFIFO_CTRL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_STS CSI2_REVA_VFIFO_STS
 * @brief    Video FIFO Status Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FEMPTY_POS           0 /**< VFIFO_STS_FEMPTY Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FEMPTY               ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FEMPTY_POS)) /**< VFIFO_STS_FEMPTY Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FTHD_POS             1 /**< VFIFO_STS_FTHD Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FTHD                 ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FTHD_POS)) /**< VFIFO_STS_FTHD Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FFULL_POS            2 /**< VFIFO_STS_FFULL Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FFULL                ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FFULL_POS)) /**< VFIFO_STS_FFULL Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_UNDERRUN_POS         3 /**< VFIFO_STS_UNDERRUN Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_UNDERRUN             ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_UNDERRUN_POS)) /**< VFIFO_STS_UNDERRUN Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_OVERRUN_POS          4 /**< VFIFO_STS_OVERRUN Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_OVERRUN              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_OVERRUN_POS)) /**< VFIFO_STS_OVERRUN Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_OUTSYNC_POS          5 /**< VFIFO_STS_OUTSYNC Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_OUTSYNC              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_OUTSYNC_POS)) /**< VFIFO_STS_OUTSYNC Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FMTERR_POS           6 /**< VFIFO_STS_FMTERR Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FMTERR               ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FMTERR_POS)) /**< VFIFO_STS_FMTERR Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_AHBWTO_POS           7 /**< VFIFO_STS_AHBWTO Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_AHBWTO               ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_AHBWTO_POS)) /**< VFIFO_STS_AHBWTO Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FS_POS               8 /**< VFIFO_STS_FS Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FS                   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FS_POS)) /**< VFIFO_STS_FS Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FE_POS               9 /**< VFIFO_STS_FE Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FE                   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_FE_POS)) /**< VFIFO_STS_FE Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_LS_POS               10 /**< VFIFO_STS_LS Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_LS                   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_LS_POS)) /**< VFIFO_STS_LS Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_LE_POS               11 /**< VFIFO_STS_LE Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_LE                   ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_STS_LE_POS)) /**< VFIFO_STS_LE Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FELT_POS             16 /**< VFIFO_STS_FELT Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FELT                 ((uint32_t)(0x7FUL << MXC_F_CSI2_REVA_VFIFO_STS_FELT_POS)) /**< VFIFO_STS_FELT Mask */

 #define MXC_F_CSI2_REVA_VFIFO_STS_FMT_POS              24 /**< VFIFO_STS_FMT Position */
 #define MXC_F_CSI2_REVA_VFIFO_STS_FMT                  ((uint32_t)(0x3FUL << MXC_F_CSI2_REVA_VFIFO_STS_FMT_POS)) /**< VFIFO_STS_FMT Mask */

/**@} end of group CSI2_REVA_VFIFO_STS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_LINE_NUM CSI2_REVA_VFIFO_LINE_NUM
 * @brief    Video FIFO CSI Line Number Per Frame.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_LINE_NUM_LINE_NUM_POS    0 /**< VFIFO_LINE_NUM_LINE_NUM Position */
 #define MXC_F_CSI2_REVA_VFIFO_LINE_NUM_LINE_NUM        ((uint32_t)(0x1FFFUL << MXC_F_CSI2_REVA_VFIFO_LINE_NUM_LINE_NUM_POS)) /**< VFIFO_LINE_NUM_LINE_NUM Mask */

/**@} end of group CSI2_REVA_VFIFO_LINE_NUM_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_PIXEL_NUM CSI2_REVA_VFIFO_PIXEL_NUM
 * @brief    Video FIFO CSI Pixel Number Per Line.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_PIXEL_NUM_PIXEL_NUM_POS  0 /**< VFIFO_PIXEL_NUM_PIXEL_NUM Position */
 #define MXC_F_CSI2_REVA_VFIFO_PIXEL_NUM_PIXEL_NUM      ((uint32_t)(0x3FFFUL << MXC_F_CSI2_REVA_VFIFO_PIXEL_NUM_PIXEL_NUM_POS)) /**< VFIFO_PIXEL_NUM_PIXEL_NUM Mask */

/**@} end of group CSI2_REVA_VFIFO_PIXEL_NUM_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_LINE_CNT CSI2_REVA_VFIFO_LINE_CNT
 * @brief    Video FIFO CSI Line Count.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_LINE_CNT_LINE_CNT_POS    0 /**< VFIFO_LINE_CNT_LINE_CNT Position */
 #define MXC_F_CSI2_REVA_VFIFO_LINE_CNT_LINE_CNT        ((uint32_t)(0xFFFUL << MXC_F_CSI2_REVA_VFIFO_LINE_CNT_LINE_CNT_POS)) /**< VFIFO_LINE_CNT_LINE_CNT Mask */

/**@} end of group CSI2_REVA_VFIFO_LINE_CNT_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_PIXEL_CNT CSI2_REVA_VFIFO_PIXEL_CNT
 * @brief    Video FIFO CSI Pixel Count.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_PIXEL_CNT_PIXEL_CNT_POS  0 /**< VFIFO_PIXEL_CNT_PIXEL_CNT Position */
 #define MXC_F_CSI2_REVA_VFIFO_PIXEL_CNT_PIXEL_CNT      ((uint32_t)(0x1FFFUL << MXC_F_CSI2_REVA_VFIFO_PIXEL_CNT_PIXEL_CNT_POS)) /**< VFIFO_PIXEL_CNT_PIXEL_CNT Mask */

/**@} end of group CSI2_REVA_VFIFO_PIXEL_CNT_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_FRAME_STS CSI2_REVA_VFIFO_FRAME_STS
 * @brief    Video FIFO Frame Status Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_FRAME_STS_FRAME_STATE_POS 0 /**< VFIFO_FRAME_STS_FRAME_STATE Position */
 #define MXC_F_CSI2_REVA_VFIFO_FRAME_STS_FRAME_STATE    ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_VFIFO_FRAME_STS_FRAME_STATE_POS)) /**< VFIFO_FRAME_STS_FRAME_STATE Mask */

 #define MXC_F_CSI2_REVA_VFIFO_FRAME_STS_ERROR_CODE_POS 3 /**< VFIFO_FRAME_STS_ERROR_CODE Position */
 #define MXC_F_CSI2_REVA_VFIFO_FRAME_STS_ERROR_CODE     ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_VFIFO_FRAME_STS_ERROR_CODE_POS)) /**< VFIFO_FRAME_STS_ERROR_CODE Mask */

/**@} end of group CSI2_REVA_VFIFO_FRAME_STS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_RAW_CTRL CSI2_REVA_VFIFO_RAW_CTRL
 * @brief    Video FIFO RAW-to-RGB Control Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_CEN_POS     0 /**< VFIFO_RAW_CTRL_RAW_CEN Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_CEN         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_CEN_POS)) /**< VFIFO_RAW_CTRL_RAW_CEN Mask */

 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_AFO_POS  1 /**< VFIFO_RAW_CTRL_RAW_FF_AFO Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_AFO      ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_AFO_POS)) /**< VFIFO_RAW_CTRL_RAW_FF_AFO Mask */

 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_FO_POS   4 /**< VFIFO_RAW_CTRL_RAW_FF_FO Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_FO       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FF_FO_POS)) /**< VFIFO_RAW_CTRL_RAW_FF_FO Mask */

 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS     8 /**< VFIFO_RAW_CTRL_RAW_FMT Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT         ((uint32_t)(0x3UL << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS)) /**< VFIFO_RAW_CTRL_RAW_FMT Mask */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB ((uint32_t)0x0UL) /**< VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS) /**< VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG ((uint32_t)0x1UL) /**< VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS) /**< VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG ((uint32_t)0x2UL) /**< VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS) /**< VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR ((uint32_t)0x3UL) /**< VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS) /**< VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR Setting */

 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS     12 /**< VFIFO_RAW_CTRL_RGB_TYP Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP         ((uint32_t)(0x7UL << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS)) /**< VFIFO_RAW_CTRL_RGB_TYP Mask */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB444  ((uint32_t)0x0UL) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB444 Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB444  (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB444 << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB444 Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB555  ((uint32_t)0x1UL) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB555 Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB555  (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB555 << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB555 Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB565  ((uint32_t)0x2UL) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB565 Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB565  (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB565 << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB565 Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB666  ((uint32_t)0x3UL) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB666 Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB666  (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB666 << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS) /**< VFIFO_RAW_CTRL_RGB_TYP_RGB666 Setting */
 #define MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGG888  ((uint32_t)0x4UL) /**< VFIFO_RAW_CTRL_RGB_TYP_RGG888 Value */
 #define MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGG888  (MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGG888 << MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_POS) /**< VFIFO_RAW_CTRL_RGB_TYP_RGG888 Setting */

/**@} end of group CSI2_REVA_VFIFO_RAW_CTRL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_RAW_BUF0_ADDR CSI2_REVA_VFIFO_RAW_BUF0_ADDR
 * @brief    Video FIFO RAW-to-RGB Line Buffer0 Address.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_BUF0_ADDR_ADDR_POS   2 /**< VFIFO_RAW_BUF0_ADDR_ADDR Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_BUF0_ADDR_ADDR       ((uint32_t)(0x3FFFFFFFUL << MXC_F_CSI2_REVA_VFIFO_RAW_BUF0_ADDR_ADDR_POS)) /**< VFIFO_RAW_BUF0_ADDR_ADDR Mask */

/**@} end of group CSI2_REVA_VFIFO_RAW_BUF0_ADDR_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_RAW_BUF1_ADDR CSI2_REVA_VFIFO_RAW_BUF1_ADDR
 * @brief    Video FIFO RAW-to-RGB Line Buffer1 Address.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_BUF1_ADDR_ADDR_POS   2 /**< VFIFO_RAW_BUF1_ADDR_ADDR Position */
 #define MXC_F_CSI2_REVA_VFIFO_RAW_BUF1_ADDR_ADDR       ((uint32_t)(0x3FFFFFFFUL << MXC_F_CSI2_REVA_VFIFO_RAW_BUF1_ADDR_ADDR_POS)) /**< VFIFO_RAW_BUF1_ADDR_ADDR Mask */

/**@} end of group CSI2_REVA_VFIFO_RAW_BUF1_ADDR_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_CTRL CSI2_REVA_VFIFO_AHBM_CTRL
 * @brief    Video FIFO AHB Master Control Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMEN_POS     0 /**< VFIFO_AHBM_CTRL_AHBMEN Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMEN         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMEN_POS)) /**< VFIFO_AHBM_CTRL_AHBMEN Mask */

 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMCLR_POS    1 /**< VFIFO_AHBM_CTRL_AHBMCLR Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMCLR        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_AHBMCLR_POS)) /**< VFIFO_AHBM_CTRL_AHBMCLR Mask */

 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS     4 /**< VFIFO_AHBM_CTRL_BSTLEN Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN         ((uint32_t)(0x3UL << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS)) /**< VFIFO_AHBM_CTRL_BSTLEN Mask */
 #define MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_VFIFO_THD ((uint32_t)0x0UL) /**< VFIFO_AHBM_CTRL_BSTLEN_VFIFO_THD Value */
 #define MXC_S_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_VFIFO_THD (MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_VFIFO_THD << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS) /**< VFIFO_AHBM_CTRL_BSTLEN_VFIFO_THD Setting */
 #define MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_ONE_WORD ((uint32_t)0x1UL) /**< VFIFO_AHBM_CTRL_BSTLEN_ONE_WORD Value */
 #define MXC_S_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_ONE_WORD (MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_ONE_WORD << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS) /**< VFIFO_AHBM_CTRL_BSTLEN_ONE_WORD Setting */
 #define MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_FOUR_WORDS ((uint32_t)0x2UL) /**< VFIFO_AHBM_CTRL_BSTLEN_FOUR_WORDS Value */
 #define MXC_S_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_FOUR_WORDS (MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_FOUR_WORDS << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS) /**< VFIFO_AHBM_CTRL_BSTLEN_FOUR_WORDS Setting */
 #define MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_EIGHT_WORDS ((uint32_t)0x3UL) /**< VFIFO_AHBM_CTRL_BSTLEN_EIGHT_WORDS Value */
 #define MXC_S_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_EIGHT_WORDS (MXC_V_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_EIGHT_WORDS << MXC_F_CSI2_REVA_VFIFO_AHBM_CTRL_BSTLEN_POS) /**< VFIFO_AHBM_CTRL_BSTLEN_EIGHT_WORDS Setting */

/**@} end of group CSI2_REVA_VFIFO_AHBM_CTRL_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_STS CSI2_REVA_VFIFO_AHBM_STS
 * @brief    Video FIFO AHB Master Status Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_HRDY_TO_POS     0 /**< VFIFO_AHBM_STS_HRDY_TO Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_HRDY_TO         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_AHBM_STS_HRDY_TO_POS)) /**< VFIFO_AHBM_STS_HRDY_TO Mask */

 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_IDLE_TO_POS     1 /**< VFIFO_AHBM_STS_IDLE_TO Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_IDLE_TO         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_AHBM_STS_IDLE_TO_POS)) /**< VFIFO_AHBM_STS_IDLE_TO Mask */

 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_TRANS_MAX_POS   2 /**< VFIFO_AHBM_STS_TRANS_MAX Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_STS_TRANS_MAX       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_VFIFO_AHBM_STS_TRANS_MAX_POS)) /**< VFIFO_AHBM_STS_TRANS_MAX Mask */

/**@} end of group CSI2_REVA_VFIFO_AHBM_STS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_START_ADDR CSI2_REVA_VFIFO_AHBM_START_ADDR
 * @brief    Video FIFO AHB Master Start Address Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_START_ADDR_AHBM_START_ADDR_POS 2 /**< VFIFO_AHBM_START_ADDR_AHBM_START_ADDR Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_START_ADDR_AHBM_START_ADDR ((uint32_t)(0x3FFFFFFFUL << MXC_F_CSI2_REVA_VFIFO_AHBM_START_ADDR_AHBM_START_ADDR_POS)) /**< VFIFO_AHBM_START_ADDR_AHBM_START_ADDR Mask */

/**@} end of group CSI2_REVA_VFIFO_AHBM_START_ADDR_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_ADDR_RANGE CSI2_REVA_VFIFO_AHBM_ADDR_RANGE
 * @brief    Video FIFO AHB Master Address Range Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_ADDR_RANGE_AHBM_ADDR_RANGE_POS 2 /**< VFIFO_AHBM_ADDR_RANGE_AHBM_ADDR_RANGE Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_ADDR_RANGE_AHBM_ADDR_RANGE ((uint32_t)(0x3FFFUL << MXC_F_CSI2_REVA_VFIFO_AHBM_ADDR_RANGE_AHBM_ADDR_RANGE_POS)) /**< VFIFO_AHBM_ADDR_RANGE_AHBM_ADDR_RANGE Mask */

/**@} end of group CSI2_REVA_VFIFO_AHBM_ADDR_RANGE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_MAX_TRANS CSI2_REVA_VFIFO_AHBM_MAX_TRANS
 * @brief    Video FIFO AHB Master Maximal Transfer Number Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_MAX_TRANS_AHBM_MAX_TRANS_POS 0 /**< VFIFO_AHBM_MAX_TRANS_AHBM_MAX_TRANS Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_MAX_TRANS_AHBM_MAX_TRANS ((uint32_t)(0xFFFFFFFFUL << MXC_F_CSI2_REVA_VFIFO_AHBM_MAX_TRANS_AHBM_MAX_TRANS_POS)) /**< VFIFO_AHBM_MAX_TRANS_AHBM_MAX_TRANS Mask */

/**@} end of group CSI2_REVA_VFIFO_AHBM_MAX_TRANS_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_VFIFO_AHBM_TRANS_CNT CSI2_REVA_VFIFO_AHBM_TRANS_CNT
 * @brief    Video FIFO AHB Master Transfer Count Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_TRANS_CNT_AHBM_TRANS_CNT_POS 0 /**< VFIFO_AHBM_TRANS_CNT_AHBM_TRANS_CNT Position */
 #define MXC_F_CSI2_REVA_VFIFO_AHBM_TRANS_CNT_AHBM_TRANS_CNT ((uint32_t)(0xFFFFFFFFUL << MXC_F_CSI2_REVA_VFIFO_AHBM_TRANS_CNT_AHBM_TRANS_CNT_POS)) /**< VFIFO_AHBM_TRANS_CNT_AHBM_TRANS_CNT Mask */

/**@} end of group CSI2_REVA_VFIFO_AHBM_TRANS_CNT_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_VFF_IE CSI2_REVA_RX_EINT_VFF_IE
 * @brief    RX Video FIFO Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMPTY_POS     0 /**< RX_EINT_VFF_IE_FNEMPTY Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMPTY         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMPTY_POS)) /**< RX_EINT_VFF_IE_FNEMPTY Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_POS        1 /**< RX_EINT_VFF_IE_FTHD Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_POS)) /**< RX_EINT_VFF_IE_FTHD Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFULL_POS       2 /**< RX_EINT_VFF_IE_FFULL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFULL           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFULL_POS)) /**< RX_EINT_VFF_IE_FFULL Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_UNDERRUN_POS    3 /**< RX_EINT_VFF_IE_UNDERRUN Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_UNDERRUN        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_UNDERRUN_POS)) /**< RX_EINT_VFF_IE_UNDERRUN Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OVERRUN_POS     4 /**< RX_EINT_VFF_IE_OVERRUN Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OVERRUN         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OVERRUN_POS)) /**< RX_EINT_VFF_IE_OVERRUN Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OUTSYNC_POS     5 /**< RX_EINT_VFF_IE_OUTSYNC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OUTSYNC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_OUTSYNC_POS)) /**< RX_EINT_VFF_IE_OUTSYNC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FMTERR_POS      6 /**< RX_EINT_VFF_IE_FMTERR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FMTERR          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FMTERR_POS)) /**< RX_EINT_VFF_IE_FMTERR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBWTO_POS      7 /**< RX_EINT_VFF_IE_AHBWTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBWTO          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBWTO_POS)) /**< RX_EINT_VFF_IE_AHBWTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FS_POS          8 /**< RX_EINT_VFF_IE_FS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FS              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FS_POS)) /**< RX_EINT_VFF_IE_FS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FE_POS          9 /**< RX_EINT_VFF_IE_FE Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FE              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FE_POS)) /**< RX_EINT_VFF_IE_FE Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LS_POS          10 /**< RX_EINT_VFF_IE_LS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LS              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LS_POS)) /**< RX_EINT_VFF_IE_LS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LE_POS          11 /**< RX_EINT_VFF_IE_LE Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LE              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_LE_POS)) /**< RX_EINT_VFF_IE_LE Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_OVR_POS     12 /**< RX_EINT_VFF_IE_RAW_OVR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_OVR         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_OVR_POS)) /**< RX_EINT_VFF_IE_RAW_OVR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_AHBERR_POS  13 /**< RX_EINT_VFF_IE_RAW_AHBERR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_AHBERR      ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_RAW_AHBERR_POS)) /**< RX_EINT_VFF_IE_RAW_AHBERR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMP_MD_POS    16 /**< RX_EINT_VFF_IE_FNEMP_MD Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMP_MD        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMP_MD_POS)) /**< RX_EINT_VFF_IE_FNEMP_MD Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_MD_POS     17 /**< RX_EINT_VFF_IE_FTHD_MD Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_MD         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_MD_POS)) /**< RX_EINT_VFF_IE_FTHD_MD Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFUL_MD_POS     18 /**< RX_EINT_VFF_IE_FFUL_MD Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFUL_MD         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFUL_MD_POS)) /**< RX_EINT_VFF_IE_FFUL_MD Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_RDTO_POS   24 /**< RX_EINT_VFF_IE_AHBM_RDTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_RDTO       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_RDTO_POS)) /**< RX_EINT_VFF_IE_AHBM_RDTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_IDTO_POS   25 /**< RX_EINT_VFF_IE_AHBM_IDTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_IDTO       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_IDTO_POS)) /**< RX_EINT_VFF_IE_AHBM_IDTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_MAX_POS    26 /**< RX_EINT_VFF_IE_AHBM_MAX Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_MAX        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IE_AHBM_MAX_POS)) /**< RX_EINT_VFF_IE_AHBM_MAX Mask */

/**@} end of group CSI2_REVA_RX_EINT_VFF_IE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_VFF_IF CSI2_REVA_RX_EINT_VFF_IF
 * @brief    RX Video FIFO Interrupt Flag Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FNEMPTY_POS     0 /**< RX_EINT_VFF_IF_FNEMPTY Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FNEMPTY         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FNEMPTY_POS)) /**< RX_EINT_VFF_IF_FNEMPTY Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FTHD_POS        1 /**< RX_EINT_VFF_IF_FTHD Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FTHD            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FTHD_POS)) /**< RX_EINT_VFF_IF_FTHD Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FFULL_POS       2 /**< RX_EINT_VFF_IF_FFULL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FFULL           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FFULL_POS)) /**< RX_EINT_VFF_IF_FFULL Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_UNDERRUN_POS    3 /**< RX_EINT_VFF_IF_UNDERRUN Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_UNDERRUN        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_UNDERRUN_POS)) /**< RX_EINT_VFF_IF_UNDERRUN Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OVERRUN_POS     4 /**< RX_EINT_VFF_IF_OVERRUN Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OVERRUN         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OVERRUN_POS)) /**< RX_EINT_VFF_IF_OVERRUN Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OUTSYNC_POS     5 /**< RX_EINT_VFF_IF_OUTSYNC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OUTSYNC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OUTSYNC_POS)) /**< RX_EINT_VFF_IF_OUTSYNC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FMTERR_POS      6 /**< RX_EINT_VFF_IF_FMTERR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FMTERR          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FMTERR_POS)) /**< RX_EINT_VFF_IF_FMTERR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBWTO_POS      7 /**< RX_EINT_VFF_IF_AHBWTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBWTO          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBWTO_POS)) /**< RX_EINT_VFF_IF_AHBWTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FS_POS          8 /**< RX_EINT_VFF_IF_FS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FS              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FS_POS)) /**< RX_EINT_VFF_IF_FS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FE_POS          9 /**< RX_EINT_VFF_IF_FE Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FE              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FE_POS)) /**< RX_EINT_VFF_IF_FE Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LS_POS          10 /**< RX_EINT_VFF_IF_LS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LS              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LS_POS)) /**< RX_EINT_VFF_IF_LS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LE_POS          11 /**< RX_EINT_VFF_IF_LE Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LE              ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_LE_POS)) /**< RX_EINT_VFF_IF_LE Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_OVR_POS     12 /**< RX_EINT_VFF_IF_RAW_OVR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_OVR         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_OVR_POS)) /**< RX_EINT_VFF_IF_RAW_OVR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_AHBERR_POS  13 /**< RX_EINT_VFF_IF_RAW_AHBERR Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_AHBERR      ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_AHBERR_POS)) /**< RX_EINT_VFF_IF_RAW_AHBERR Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_RDTO_POS   24 /**< RX_EINT_VFF_IF_AHBM_RDTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_RDTO       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_RDTO_POS)) /**< RX_EINT_VFF_IF_AHBM_RDTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_IDTO_POS   25 /**< RX_EINT_VFF_IF_AHBM_IDTO Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_IDTO       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_IDTO_POS)) /**< RX_EINT_VFF_IF_AHBM_IDTO Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_MAX_POS    26 /**< RX_EINT_VFF_IF_AHBM_MAX Position */
 #define MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_MAX        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_VFF_IF_AHBM_MAX_POS)) /**< RX_EINT_VFF_IF_AHBM_MAX Mask */

/**@} end of group CSI2_REVA_RX_EINT_VFF_IF_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_PPI_IE CSI2_REVA_RX_EINT_PPI_IE
 * @brief    RX D-PHY Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0STOP_POS     0 /**< RX_EINT_PPI_IE_DL0STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0STOP_POS)) /**< RX_EINT_PPI_IE_DL0STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1STOP_POS     1 /**< RX_EINT_PPI_IE_DL1STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1STOP_POS)) /**< RX_EINT_PPI_IE_DL1STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_CL0STOP_POS     4 /**< RX_EINT_PPI_IE_CL0STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_CL0STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_CL0STOP_POS)) /**< RX_EINT_PPI_IE_CL0STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT0_POS   6 /**< RX_EINT_PPI_IE_DL0ECONT0 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT0       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT0_POS)) /**< RX_EINT_PPI_IE_DL0ECONT0 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT1_POS   7 /**< RX_EINT_PPI_IE_DL0ECONT1 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT1       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECONT1_POS)) /**< RX_EINT_PPI_IE_DL0ECONT1 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOT_POS     8 /**< RX_EINT_PPI_IE_DL0ESOT Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOT         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOT_POS)) /**< RX_EINT_PPI_IE_DL0ESOT Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOT_POS     9 /**< RX_EINT_PPI_IE_DL1ESOT Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOT         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOT_POS)) /**< RX_EINT_PPI_IE_DL1ESOT Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOTS_POS    12 /**< RX_EINT_PPI_IE_DL0ESOTS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOTS        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESOTS_POS)) /**< RX_EINT_PPI_IE_DL0ESOTS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOTS_POS    13 /**< RX_EINT_PPI_IE_DL1ESOTS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOTS        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESOTS_POS)) /**< RX_EINT_PPI_IE_DL1ESOTS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0EESC_POS     16 /**< RX_EINT_PPI_IE_DL0EESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0EESC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0EESC_POS)) /**< RX_EINT_PPI_IE_DL0EESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1EESC_POS     17 /**< RX_EINT_PPI_IE_DL1EESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1EESC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1EESC_POS)) /**< RX_EINT_PPI_IE_DL1EESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESESC_POS    20 /**< RX_EINT_PPI_IE_DL0ESESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESESC        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ESESC_POS)) /**< RX_EINT_PPI_IE_DL0ESESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESESC_POS    21 /**< RX_EINT_PPI_IE_DL1ESESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESESC        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ESESC_POS)) /**< RX_EINT_PPI_IE_DL1ESESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECTL_POS     24 /**< RX_EINT_PPI_IE_DL0ECTL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECTL         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL0ECTL_POS)) /**< RX_EINT_PPI_IE_DL0ECTL Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ECTL_POS     25 /**< RX_EINT_PPI_IE_DL1ECTL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ECTL         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IE_DL1ECTL_POS)) /**< RX_EINT_PPI_IE_DL1ECTL Mask */

/**@} end of group CSI2_REVA_RX_EINT_PPI_IE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_PPI_IF CSI2_REVA_RX_EINT_PPI_IF
 * @brief    RX D-PHY Interrupt Flag Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0STOP_POS     0 /**< RX_EINT_PPI_IF_DL0STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0STOP_POS)) /**< RX_EINT_PPI_IF_DL0STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1STOP_POS     1 /**< RX_EINT_PPI_IF_DL1STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1STOP_POS)) /**< RX_EINT_PPI_IF_DL1STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_CL0STOP_POS     4 /**< RX_EINT_PPI_IF_CL0STOP Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_CL0STOP         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_CL0STOP_POS)) /**< RX_EINT_PPI_IF_CL0STOP Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT0_POS   6 /**< RX_EINT_PPI_IF_DL0ECONT0 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT0       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT0_POS)) /**< RX_EINT_PPI_IF_DL0ECONT0 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT1_POS   7 /**< RX_EINT_PPI_IF_DL0ECONT1 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT1       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECONT1_POS)) /**< RX_EINT_PPI_IF_DL0ECONT1 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOT_POS     8 /**< RX_EINT_PPI_IF_DL0ESOT Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOT         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOT_POS)) /**< RX_EINT_PPI_IF_DL0ESOT Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOT_POS     9 /**< RX_EINT_PPI_IF_DL1ESOT Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOT         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOT_POS)) /**< RX_EINT_PPI_IF_DL1ESOT Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOTS_POS    12 /**< RX_EINT_PPI_IF_DL0ESOTS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOTS        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESOTS_POS)) /**< RX_EINT_PPI_IF_DL0ESOTS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOTS_POS    13 /**< RX_EINT_PPI_IF_DL1ESOTS Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOTS        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESOTS_POS)) /**< RX_EINT_PPI_IF_DL1ESOTS Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0EESC_POS     16 /**< RX_EINT_PPI_IF_DL0EESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0EESC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0EESC_POS)) /**< RX_EINT_PPI_IF_DL0EESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1EESC_POS     17 /**< RX_EINT_PPI_IF_DL1EESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1EESC         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1EESC_POS)) /**< RX_EINT_PPI_IF_DL1EESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESESC_POS    20 /**< RX_EINT_PPI_IF_DL0ESESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESESC        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ESESC_POS)) /**< RX_EINT_PPI_IF_DL0ESESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESESC_POS    21 /**< RX_EINT_PPI_IF_DL1ESESC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESESC        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ESESC_POS)) /**< RX_EINT_PPI_IF_DL1ESESC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECTL_POS     24 /**< RX_EINT_PPI_IF_DL0ECTL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECTL         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0ECTL_POS)) /**< RX_EINT_PPI_IF_DL0ECTL Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ECTL_POS     25 /**< RX_EINT_PPI_IF_DL1ECTL Position */
 #define MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ECTL         ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1ECTL_POS)) /**< RX_EINT_PPI_IF_DL1ECTL Mask */

/**@} end of group CSI2_REVA_RX_EINT_PPI_IF_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_CTRL_IE CSI2_REVA_RX_EINT_CTRL_IE
 * @brief    RX Controller Interrupt Enable Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC2_POS      0 /**< RX_EINT_CTRL_IE_EECC2 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC2          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC2_POS)) /**< RX_EINT_CTRL_IE_EECC2 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC1_POS      1 /**< RX_EINT_CTRL_IE_EECC1 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC1          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC1_POS)) /**< RX_EINT_CTRL_IE_EECC1 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_ECRC_POS       2 /**< RX_EINT_CTRL_IE_ECRC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_ECRC           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_ECRC_POS)) /**< RX_EINT_CTRL_IE_ECRC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EID_POS        3 /**< RX_EINT_CTRL_IE_EID Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EID            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EID_POS)) /**< RX_EINT_CTRL_IE_EID Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_PKTFFOV_POS    4 /**< RX_EINT_CTRL_IE_PKTFFOV Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_PKTFFOV        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_PKTFFOV_POS)) /**< RX_EINT_CTRL_IE_PKTFFOV Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSA_POS   8 /**< RX_EINT_CTRL_IE_DL0ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSA_POS)) /**< RX_EINT_CTRL_IE_DL0ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSA_POS   9 /**< RX_EINT_CTRL_IE_DL1ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSA_POS)) /**< RX_EINT_CTRL_IE_DL1ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSM_POS   12 /**< RX_EINT_CTRL_IE_DL0ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL0ULPSM_POS)) /**< RX_EINT_CTRL_IE_DL0ULPSM Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSM_POS   13 /**< RX_EINT_CTRL_IE_DL1ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_DL1ULPSM_POS)) /**< RX_EINT_CTRL_IE_DL1ULPSM Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSA_POS   16 /**< RX_EINT_CTRL_IE_CL0ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSA_POS)) /**< RX_EINT_CTRL_IE_CL0ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSM_POS   17 /**< RX_EINT_CTRL_IE_CL0ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_CL0ULPSM_POS)) /**< RX_EINT_CTRL_IE_CL0ULPSM Mask */

/**@} end of group CSI2_REVA_RX_EINT_CTRL_IE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_RX_EINT_CTRL_IF CSI2_REVA_RX_EINT_CTRL_IF
 * @brief    RX Controller Interrupt Flag Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC2_POS      0 /**< RX_EINT_CTRL_IF_EECC2 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC2          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC2_POS)) /**< RX_EINT_CTRL_IF_EECC2 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC1_POS      1 /**< RX_EINT_CTRL_IF_EECC1 Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC1          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EECC1_POS)) /**< RX_EINT_CTRL_IF_EECC1 Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_ECRC_POS       2 /**< RX_EINT_CTRL_IF_ECRC Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_ECRC           ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_ECRC_POS)) /**< RX_EINT_CTRL_IF_ECRC Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EID_POS        3 /**< RX_EINT_CTRL_IF_EID Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EID            ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_EID_POS)) /**< RX_EINT_CTRL_IF_EID Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_PKTFFOV_POS    4 /**< RX_EINT_CTRL_IF_PKTFFOV Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_PKTFFOV        ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_PKTFFOV_POS)) /**< RX_EINT_CTRL_IF_PKTFFOV Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSA_POS   8 /**< RX_EINT_CTRL_IF_DL0ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSA_POS)) /**< RX_EINT_CTRL_IF_DL0ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSA_POS   9 /**< RX_EINT_CTRL_IF_DL1ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSA_POS)) /**< RX_EINT_CTRL_IF_DL1ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSM_POS   12 /**< RX_EINT_CTRL_IF_DL0ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL0ULPSM_POS)) /**< RX_EINT_CTRL_IF_DL0ULPSM Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSM_POS   13 /**< RX_EINT_CTRL_IF_DL1ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_DL1ULPSM_POS)) /**< RX_EINT_CTRL_IF_DL1ULPSM Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSA_POS   16 /**< RX_EINT_CTRL_IF_CL0ULPSA Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSA       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSA_POS)) /**< RX_EINT_CTRL_IF_CL0ULPSA Mask */

 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSM_POS   17 /**< RX_EINT_CTRL_IF_CL0ULPSM Position */
 #define MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSM       ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_RX_EINT_CTRL_IF_CL0ULPSM_POS)) /**< RX_EINT_CTRL_IF_CL0ULPSM Mask */

/**@} end of group CSI2_REVA_RX_EINT_CTRL_IF_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_PPI_STOPSTATE CSI2_REVA_PPI_STOPSTATE
 * @brief    DPHY PPI Stop State Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_DL0STOP_POS      0 /**< PPI_STOPSTATE_DL0STOP Position */
 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_DL0STOP          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_STOPSTATE_DL0STOP_POS)) /**< PPI_STOPSTATE_DL0STOP Mask */

 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_DL1STOP_POS      1 /**< PPI_STOPSTATE_DL1STOP Position */
 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_DL1STOP          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_STOPSTATE_DL1STOP_POS)) /**< PPI_STOPSTATE_DL1STOP Mask */

 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_CL0STOP_POS      2 /**< PPI_STOPSTATE_CL0STOP Position */
 #define MXC_F_CSI2_REVA_PPI_STOPSTATE_CL0STOP          ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_STOPSTATE_CL0STOP_POS)) /**< PPI_STOPSTATE_CL0STOP Mask */

/**@} end of group CSI2_REVA_PPI_STOPSTATE_Register */

/**
 * @ingroup  csi2_reva_registers
 * @defgroup CSI2_REVA_PPI_TURNAROUND_CFG CSI2_REVA_PPI_TURNAROUND_CFG
 * @brief    DPHY PPI Turn-Around Configuration Register.
 * @{
 */
 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TAREQ_POS 0 /**< PPI_TURNAROUND_CFG_DL0TAREQ Position */
 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TAREQ    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TAREQ_POS)) /**< PPI_TURNAROUND_CFG_DL0TAREQ Mask */

 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TADIS_POS 1 /**< PPI_TURNAROUND_CFG_DL0TADIS Position */
 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TADIS    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0TADIS_POS)) /**< PPI_TURNAROUND_CFG_DL0TADIS Mask */

 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0FRCRX_POS 2 /**< PPI_TURNAROUND_CFG_DL0FRCRX Position */
 #define MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0FRCRX    ((uint32_t)(0x1UL << MXC_F_CSI2_REVA_PPI_TURNAROUND_CFG_DL0FRCRX_POS)) /**< PPI_TURNAROUND_CFG_DL0FRCRX Mask */

/**@} end of group CSI2_REVA_PPI_TURNAROUND_CFG_Register */

#ifdef __cplusplus
}
#endif

#endif /* _CSI2_REVA_REGS_H_ */
