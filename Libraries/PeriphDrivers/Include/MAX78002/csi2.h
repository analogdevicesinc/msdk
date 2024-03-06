/**
 * @file    csi2.h
 * @brief   Camera Serial Interface 2 (CSI-2) function prototypes and data types.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CSI2_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CSI2_H_

/* **** Includes **** */
#include <stdbool.h>
#include "csi2_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup csi2 Camera Serial Interface 2 (CSI-2)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

typedef struct _mxc_csi2_req_t mxc_csi2_req_t;

/**
 * @brief   Enumeration type for the CSI-2 PPI Clock Inversion.
 */
typedef enum {
    MXC_CSI2_PPI_NO_INVERT, ///< No inversion of PPI input clock
    MXC_CSI2_PPI_INVERT, ///< Invert PPI input clock
} mxc_csi2_ppi_clk_t;

/**
 * @brief   Enumeration type for the CSI-2 Payload 0 data types.
 */
typedef enum {
    MXC_CSI2_PL0_DISABLE_ALL = 0xFFFFFFFF, ///< Disable payload0 data
    MXC_CSI2_PL0_NULL = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_NULL, ///< NULL payload data
    MXC_CSI2_PL0_BLANK = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_BLANK, ///< BLANK payload data
    MXC_CSI2_PL0_EMBEDDED = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_EMBEDDED, ///< EMBEDDED payload data
    MXC_CSI2_PL0_YUV420_8BIT = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT, ///< YUV420 8-bit data
    MXC_CSI2_PL0_YUV420_10BIT =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT, ///< YUV420 10-bit data
    MXC_CSI2_PL0_LEG_YUV420_8BIT =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_LEG, ///< Legacy YUV420 8-bit data
    MXC_CSI2_PL0_YUV422_8BIT_CSP =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV420_8BIT_CSP, ///< YUV422 8-bit CSP data
    MXC_CSI2_PL0_YUV422_10BIT_CSP =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV420_10BIT_CSP, ///< YUV422 10-bit CSP data
    MXC_CSI2_PL0_YUV422_8BIT = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV422_8BIT, ///< YUV422 8-bit data
    MXC_CSI2_PL0_YUV422_10BIT =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_YUV422_10BIT, ///< YUV422 10-bit data
    MXC_CSI2_PL0_RGB444 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RGB444, ///< RGB444 data
    MXC_CSI2_PL0_RGB555 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RGB555, ///< RGB555 data
    MXC_CSI2_PL0_RGB565 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RGB565, ///< RGB565 data
    MXC_CSI2_PL0_RGB666 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RGB666, ///< RGB666 data
    MXC_CSI2_PL0_RGB888 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RGB888, ///< RGB888 data
    MXC_CSI2_PL0_RAW6 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW6, ///< RAW6 data
    MXC_CSI2_PL0_RAW7 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW7, ///< RAW7 data
    MXC_CSI2_PL0_RAW8 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW8, ///< RAW8 data
    MXC_CSI2_PL0_RAW10 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW10, ///< RAW10 data
    MXC_CSI2_PL0_RAW12 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW12, ///< RAW12 data
    MXC_CSI2_PL0_RAW14 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW14, ///< RAW14 data
    MXC_CSI2_PL0_RAW16 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW16, ///< RAW16 data
    MXC_CSI2_PL0_RAW20 = ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_0_RAW20, ///< RAW20 data
} mxc_csi2_payload0_t;

/**
 * @brief   Enumeration type for the CSI-2 Payload 1 Datatypes.
 */
typedef enum {
    MXC_CSI2_PL1_DISABLE_ALL = 0x0001FFFF, ///< Disable payload1 data
    MXC_CSI2_PL1_UD_0x30 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE30, ///< User defined type 0x30 data
    MXC_CSI2_PL1_UD_0x31 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE31, ///< User defined type 0x31 data
    MXC_CSI2_PL1_UD_0x32 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE32, ///< User defined type 0x32 data
    MXC_CSI2_PL1_UD_0x33 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE33, ///< User defined type 0x33 data
    MXC_CSI2_PL1_UD_0x34 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE34, ///< User defined type 0x34 data
    MXC_CSI2_PL1_UD_0x35 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE35, ///< User defined type 0x35 data
    MXC_CSI2_PL1_UD_0x36 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE36, ///< User defined type 0x36 data
    MXC_CSI2_PL1_UD_0x37 =
        ~MXC_F_CSI2_CFG_DISABLE_PAYLOAD_1_USR_DEF_TYPE37, ///< User defined type 0x37 data
} mxc_csi2_payload1_t;

/**
 * @brief   Enumeration type for the CSI-2 Lane Control Source Selections.
 */
typedef enum {
    MXC_CSI2_PAD_CDRX_PN_L0, ///< Connected to PAD_CDRX_L0P/N
    MXC_CSI2_PAD_CDRX_PN_L1, ///< Connected to PAD_CDRX_L1P/N
    MXC_CSI2_PAD_CDRX_PN_L2, ///< Connected to PAD_CDRX_L2P/N
    MXC_CSI2_PAD_CDRX_PN_L3, ///< Connected to PAD_CDRX_L3P/N
    MXC_CSI2_PAD_CDRX_PN_L4, ///< Connected to PAD_CDRX_L4P/N
} mxc_csi2_swap_sel_t;

/**
 * @brief   Enumeration type for CSI-2 VFIFO DMA Mode.
 */
typedef enum {
    MXC_CSI2_DMA_NO_DMA = MXC_S_CSI2_VFIFO_CFG0_DMAMODE_NO_DMA, ///< No DMA Mode
    MXC_CSI2_DMA_SEND_REQUEST = MXC_S_CSI2_VFIFO_CFG0_DMAMODE_DMA_REQ, ///< DMA Request Mode
    MXC_CSI2_DMA_FIFO_ABV_THD =
        MXC_S_CSI2_VFIFO_CFG0_DMAMODE_FIFO_THD, ///< Above FIFO Threshold Mode
    MXC_CSI2_DMA_FIFO_FULL = MXC_S_CSI2_VFIFO_CFG0_DMAMODE_FIFO_FULL, ///< FIFO Full Mode
} mxc_csi2_dma_mode_t;

/**
 * @brief   Enumeration type for CSI-2 VFIFO Read Mode.
 */
typedef enum {
    MXC_CSI2_READ_ONE_BY_ONE, ///< One by One FIFO Read Mode
    MXC_CSI2_READ_DIRECT_ADDR, ///< Direct Addressing FIFO Read Mode
} mxc_csi2_fiform_t;

/**
 * @brief   Enumeration type for CSI-2 VIFO Error Detection.
 */
typedef enum {
    MXC_CSI2_ERR_DETECT_DISABLE, ///< Disable Error Detection
    MXC_CSI2_ERR_DETECT_ENABLE, ///< Enable Error Detection
} mxc_csi2_err_det_t;

/**
 * @brief   Enumeration type for the CSI-2 VFIFO Bandwidth.
 */
typedef enum {
    MXC_CSI2_NORMAL_BW, ///< Normal Bandwidth Mode
    MXC_CSI2_FULL_BW, ///< Full Bandwidth Mode
} mxc_csi2_fbwm_t;

/**
 * @brief   Enumeration type for the CSI-2 RGB Type.
 */
typedef enum {
    MXC_CSI2_TYPE_RGB444 = MXC_S_CSI2_VFIFO_RAW_CTRL_RGB_TYP_RGB444, ///< RGB444 Type
    MXC_CSI2_TYPE_RGB555 = MXC_S_CSI2_VFIFO_RAW_CTRL_RGB_TYP_RGB555, ///< RGB555 Type
    MXC_CSI2_TYPE_RGB565 = MXC_S_CSI2_VFIFO_RAW_CTRL_RGB_TYP_RGB565, ///< RGB565 Type
    MXC_CSI2_TYPE_RGB666 = MXC_S_CSI2_VFIFO_RAW_CTRL_RGB_TYP_RGB666, ///< RGB666 Type
    MXC_CSI2_TYPE_RGB888 = MXC_S_CSI2_VFIFO_RAW_CTRL_RGB_TYP_RGG888, ///< RGB888 Type
} mxc_csi2_rgb_type_t;

/**
 * @brief   Enumeration type for the CSI-2 RAW Format.
 */
typedef enum {
    MXC_CSI2_FORMAT_RGRG_GBGB =
        (0x0 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FMT_POS), ///< RGRG_GBGB Format
    MXC_CSI2_FORMAT_GRGR_BGBG =
        (0x1 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FMT_POS), ///< GRGR_BGBG Format
    MXC_CSI2_FORMAT_GBGB_RGRG =
        (0x2 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FMT_POS), ///< GBGB_RGRG Format
    MXC_CSI2_FORMAT_BGBG_GRGR =
        (0x3 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FMT_POS), ///M BGBG_GRGR Format
} mxc_csi2_raw_format_t;

/**
 * @brief   Enumeration type for the CSI-2 Autoflush.
 */
typedef enum {
    MXC_CSI2_AUTOFLUSH_DISABLE = 0, ///< Disable FIFO Automatic Flush-Out
    MXC_CSI2_AUTOFLUSH_ENABLE, ///< Enable FIFO Automatic Flush-Out
} mxc_csi2_raw_autoflush_t;

/**
 * @brief   Enumeration type for the CSI-2 AHB Wait option.
 */
typedef enum {
    MXC_CSI2_AHBWAIT_DISABLE = 0, ///< Disable AHB Wait
    MXC_CSI2_AHBWAIT_ENABLE, ///< Enable AHB Wait
} mxc_csi2_ahbwait_t;

/**
 * @brief   Enumeration type for the CSI-2 DMA Frame option.
 */
typedef enum {
    MXC_CSI2_DMA_WHOLE_FRAME = 0, ///< DMA entire frames at a time
    MXC_CSI2_DMA_LINE_BY_LINE, ///< DMA line by line per frame
} mxc_csi2_dma_frame_t;

typedef struct _mxc_csi2_capture_stats_t {
    bool success;
    uint32_t ctrl_err;
    uint32_t ppi_err;
    uint32_t vfifo_err;
    size_t frame_size;
    size_t bytes_captured;
} mxc_csi2_capture_stats_t;

/**
 * @brief   The callback routine used to indicate to indicate transaction has terminated.  This is currently unused until multi-frame exposures are implemented.
 * @param   req          The details of the image capture.
 * @param   result       See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_csi2_frame_handler_cb_t)(mxc_csi2_req_t *req, int result);

/**
 * @brief   The callback routine used to handle incoming image data from the camera sensor.
 *          It is triggered once per row. Application code should implement this
 *          and ensure that the callback is fast enough to keep up with the incoming data.
 * @param[in] data  Pointer to the received bytes in memory.
 * @param[in] len   The number of bytes received.
 * 
 * @return  This function should return 0 on success.  If non-zero, the CSI-2 controller will end the frame capture
 */
typedef int (*mxc_csi2_line_handler_cb_t)(uint8_t *data, unsigned int len);

/**
 * @brief  Selects control source signals for data and clock lanes.
 */
typedef struct {
    mxc_csi2_swap_sel_t d0_swap_sel; ///< Data Lane 0 Control Source Select
    mxc_csi2_swap_sel_t d1_swap_sel; ///< Data Lane 1 Control Source Select
    mxc_csi2_swap_sel_t d2_swap_sel; ///< Data Lane 2 Control Source Select
    mxc_csi2_swap_sel_t d3_swap_sel; ///< Data Lane 3 Control Source Select
    mxc_csi2_swap_sel_t c0_swap_sel; ///< Clock Lane Control Source Select
} mxc_csi2_lane_src_t;

/**
 * @brief  Struct containing RX Controller and D-PHY configuration details.
 */
typedef struct {
    mxc_csi2_ppi_clk_t invert_ppi_clk; ///< Invert the PPI input clock
    uint32_t num_lanes; ///< Configure number of lanes
    uint32_t flush_cnt; ///< Flush Count
    mxc_csi2_payload0_t payload0; ///< Payload 0 data select
    mxc_csi2_payload1_t payload1; ///< Payload 1 data select
    mxc_csi2_lane_src_t lane_src; ///< Data and Clock lane sources
} mxc_csi2_ctrl_cfg_t;

/**
 * @brief  Struct containing parameters for the VFIFO.
 */
typedef struct {
    uint32_t virtual_channel; ///< Virtual Channel (0-3)
    uint32_t flow_ctrl; ///< Flow control selection
    uint32_t wait_en; ///< AHB Wait Enable
    uint32_t wait_cyc; ///< AHB Maximal Wait Cycles

    mxc_csi2_dma_frame_t dma_whole_frame; ///< DMA whole frame or line by line
    mxc_csi2_dma_mode_t dma_mode; ///< DMA Mode
    mxc_csi2_fiform_t fifo_rd_mode; ///< FIFO Read Mode
    mxc_csi2_err_det_t err_det_en; ///< Error Detection Enable
    uint32_t rx_thd; ///< FIFO Threshold
    mxc_csi2_fbwm_t bandwidth_mode; ///< Full Band Width Mode Enable
} mxc_csi2_vfifo_cfg_t;

/**
 * @brief  The information required to capture images.
 */
struct _mxc_csi2_req_t {
    uint32_t pixels_per_line; ///< Image Width
    uint32_t lines_per_frame; ///< Image Height
    uint32_t bits_per_pixel_odd; ///< Bits Per Pixel Odd
    uint32_t bits_per_pixel_even; ///< Bits Per Pixel Even
    uint32_t frame_num; ///< Number of frames to capture
    mxc_csi2_line_handler_cb_t
        line_handler; ///< Callback triggered for each image row.  Application code must implement this to offload data.

    uint8_t process_raw_to_rgb; ///< Select if processing RAW data to RGB type
    mxc_csi2_rgb_type_t rgb_type; ///< Select final processed RGB type
    mxc_csi2_raw_format_t raw_format; ///< Select RAW format
    mxc_csi2_raw_autoflush_t autoflush; ///< FIFO Automatic Flush-Out enable
    uint32_t raw_buf0_addr; ///< RAW Buffer 0 Address (line by line)
    uint32_t raw_buf1_addr; ///< RAW Buffer 1 Address (line by line)
};

/* **** Function Prototypes **** */

/******************************************/
/* Global Control/Configuration Functions */
/******************************************/

/**
 * @brief      Initialize CSI-2.
 * @param      req        The details of the image for capture.
 * @param      ctrl_cfg   Configuration details for RX Controller.
 * @param      vfifo_cfg  Configuration details for VFIFO.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_Init(mxc_csi2_req_t *req, mxc_csi2_ctrl_cfg_t *ctrl_cfg,
                  mxc_csi2_vfifo_cfg_t *vfifo_cfg);

/**
 * @brief      Shutdown CSI-2.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_Shutdown(void);

/**
 * @brief      Enable the blocks and start the CSI-2.
 * @param      num_data_lanes    Number of data lanes used.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_Start(int num_data_lanes);

/**
 * @brief      Disable the blocks and stop the CSI-2.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_Stop(void);

/**
 * @brief      Capture an image frame using DMA. Same as MXC_CSI2_CaptureFrameDMA.
 * @param      num_data_lanes    Number of data lanes used.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_CaptureFrame(int num_data_lanes);

/**
 * @brief      Capture an image frame using DMA.
 * @param      num_data_lanes    Number of data lanes used.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_CaptureFrameDMA();

/**
 * @brief      Select Lane Control Source for D0-D4 and C0.
 * @param      src    The lane control source signal assignments.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_SetLaneCtrlSource(mxc_csi2_lane_src_t *src);

/**
 * @brief      Get Lane Control Source for D0-D4 and C0.
 * @param      src    Pointer to hold lane control source signal assignments.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_GetLaneCtrlSource(mxc_csi2_lane_src_t *src);

/**
 * @brief      Grab the configured image details.
 * @param[out]      imgLen   Pointer to the total length of the image (in bytes).
 * @param[out]      w        Pointer to image width (in pixels).
 * @param[out]      h        Pointer to image height (in pixels).
 */
void MXC_CSI2_GetImageDetails(uint32_t *imgLen, uint32_t *w, uint32_t *h);

/********************************/
/* CSI2 RX Controller Functions */
/********************************/

/**
 * @brief      Configure the RX Controller.
 * @param      cfg    Struct containing RX Controller config parameters.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_CTRL_Config(mxc_csi2_ctrl_cfg_t *cfg);

/**
 * @brief      Enable RX Controller Interrupts.
 * @param      mask   The interrupt to be enabled.
 */
void MXC_CSI2_CTRL_EnableInt(uint32_t mask);

/**
 * @brief      Disable RX Controller Interrupts.
 * @param      mask   The interrupt to be disable.
 */
void MXC_CSI2_CTRL_DisableInt(uint32_t mask);

/**
 * @brief      Gets the interrupt flags that are currently set.
 * @return     The interrupt flags.
 */
int MXC_CSI2_CTRL_GetFlags(void);

/**
 * @brief      Clears the interrupt flags.
 * @param      flags    mask of flags to be cleared.
 */
void MXC_CSI2_CTRL_ClearFlags(uint32_t flags);

/************************/
/* CSI2 VFIFO Functions */
/************************/

/**
 * @brief      Clears the interrupt flags.
 * @param      flags    mask of flags to be cleared.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_Config(mxc_csi2_vfifo_cfg_t *cfg);

/**
 * @brief      Set Next FIFO Trigger Mode: FIFO Not Empty, Above Threshold, Full.
 * @param      ff_not_empty   Set trigger when FIFO not empty.
 * @param      ff_abv_thd 	  Set trigger when FIFO above threshold.
 * @param      ff_full		  Set trigger when FIFO Full.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_NextFIFOTrigMode(uint8_t ff_not_empty, uint8_t ff_abv_thd, uint8_t ff_full);

/**
 * @brief      Enable VFIFO Interrupts.
 * @param      mask   The interrupt to be enabled.
 * @param      edge   Edge triggered or level triggered FIFO detection.
 */
void MXC_CSI2_VFIFO_EnableInt(uint32_t mask, uint32_t edge);

/**
 * @brief      Change FIFO detection mode for FIFO-specific VFIFO interrupts.
 * @param      mask   The interrupt to be enabled.
 * @param      edge   Edge triggered (true) or level triggered (false) FIFO detection.
 */
void MXC_CSI2_VFIFO_ChangeIntMode(uint32_t mask, uint32_t edge);

/**
 * @brief      Disable VFIFO Interrupts.
 * @param      mask   The interrupt to be disable.
 */
void MXC_CSI2_VFIFO_DisableInt(uint32_t mask);

/**
 * @brief      Gets the interrupt flags that are currently set for VFIFO.
 * @return     The interrupt flags.
 */
int MXC_CSI2_VFIFO_GetFlags(void);

/**
 * @brief      Clears the interrupt flags for VFIFO.
 * @param      flags    mask of flags to be cleared.
 */
void MXC_CSI2_VFIFO_ClearFlags(uint32_t flags);

/**
 * @brief      Only enables the VFIFO.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_Enable(void);

/**
 * @brief      Only disables the VFIFO.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_Disable(void);

/**
 * @brief      Set parameters for processing RAW image data to RGB type data.
 * @note       Payload data type from Init function should be set to RAW.
 * @param      req    Struct containing parameters for RAW to RGB.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_ProcessRAWtoRGB(mxc_csi2_req_t *req);

/**
 * @brief      Sets the Payload data type.
 * @note       Parameters negative logic. 0 - enabled, 1 disabled. Passing set value disables data type.
 * @param      payload0    Select payload data type from mxc_csi2_payload0_t.
 * @param      payload1    Select payload data type from mxc_csi2_payload1_t.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_SetPayloadType(mxc_csi2_payload0_t payload0, mxc_csi2_payload1_t payload1);

/**
 * @brief      Get the Payload data type.
 * @note       Value of registers are negative logic. 0 - enabled, 1 disabled. Compare 
 *             with mxc_csi2_payloadn_t enum types or check user guide for specific value.
 * @param      payload0    Pointer to save the value of DISABLE_PAYLOAD_0 register.
 * @param      payload1    Pointer to save the value DISABLE_PAYLOAD_1 register.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_GetPayloadType(uint32_t *payload0, uint32_t *payload1);

/**
 * @brief      Sets the DMA Mode for VFIFO.
 * @param      dma_mode    Select DMA mode (condition to trigger) from mxc_csi2_dma_mode_t.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_SetDMAMode(mxc_csi2_dma_mode_t dma_mode);

/**
 * @brief      Get the currently set DMA Mode for VFIFO.
 * @return     The currently set DMA mode.
 */
mxc_csi2_dma_mode_t MXC_CSI2_VFIFO_GetDMAMode(void);

/**
 * @brief      Sets the RGB Type when reading CSI2 FIFO.
 * @param      rgb_type    Select RGB type from mxc_csi2_rgb_type_t.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_SetRGBType(mxc_csi2_rgb_type_t rgb_type);

/**
 * @brief      Get the currently set RGB type.
 * @return     The currently set RGB Type.
 */
mxc_csi2_rgb_type_t MXC_CSI2_VFIFO_GetRGBType(void);

/**
 * @brief      Sets the RAW format type.
 * @param      raw_format  Select Bayer Filter Pattern format type from mxc_csi2_raw_format_t.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_VFIFO_SetRAWFormat(mxc_csi2_raw_format_t raw_format);

/**
 * @brief      Gets the currently set Bayer Filter pattern RAW format type.
 * @return     The currently set RAW format.
 */
mxc_csi2_raw_format_t MXC_CSI2_VFIFO_GetRAWFormat(void);

/**
 * @brief      Get the remaining current FIFO count in VFIFO.
 * @note	   RGB Type can vary entity width.
 * @return     Number of entities currently in the FIFO.
 */
int MXC_CSI2_VFIFO_GetFIFOEntityCount(void);

/**
 * @brief      Set the AHB Wait (enable/disable)
 * @param 	   wait_en	   The setting to set AHBWAIT using type mxc_csi2_ahbwait_t.
 */
void MXC_CSI2_VFIFO_SetAHBWait(mxc_csi2_ahbwait_t wait_en);

/**
 * @brief      Retrieves whether AHB Wait is enabled or disabled.
 * @return 	   State of AHB Wait
 */
mxc_csi2_ahbwait_t MXC_CSI2_VFIFO_GetAHBWait(void);

/***********************************************/
/* CSI2 PHY Protocol Interface (PPI) Functions */
/***********************************************/

/**
 * @brief      Enable PHY Protocol interface (PPI) Interrupts.
 * @param      mask   The interrupt to be enabled.
 */
void MXC_CSI2_PPI_EnableInt(uint32_t mask);

/**
 * @brief      Disable PHY Protocol interface (PPI) Interrupts.
 * @param      mask   The interrupt to be disable.
 */
void MXC_CSI2_PPI_DisableInt(uint32_t mask);

/**
 * @brief      Gets the interrupt flags that are currently set for PPI.
 * @return     The interrupt flags.
 */
int MXC_CSI2_PPI_GetFlags(void);

/**
 * @brief      Clears the interrupt flags for PPI.
 * @param      flags    mask of flags to be cleared.
 */
void MXC_CSI2_PPI_ClearFlags(uint32_t flags);

/**
 * @brief      Stops the PPI by disabling interrupts.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_PPI_Stop(void);

/************************************/
/* CSI2 DMA - Used for all features */
/************************************/

bool MXC_CSI2_DMA_Frame_Complete(void);

mxc_csi2_capture_stats_t MXC_CSI2_GetCaptureStats();

/**
 * @brief      Clears the interrupt flags for PPI.
 * @param      flags    mask of flags to be cleared.
 * @return     #E_NO_ERROR if everything is successful.
 */
int MXC_CSI2_DMA_Config(uint8_t *dst_addr, uint32_t byte_cnt, uint32_t burst_size);

/**
 * @brief      Gets the acquired DMA channel used for CSI-2 operations.
 * @return     The channel currently used for CSI-2 operations.
 */
int MXC_CSI2_DMA_GetChannel(void);

/**
 * @brief      Gets the current DMA line count when using line by line DMA requests.
 * @note       In other words, gets the number of lines the DMA interrupt was triggered.
 * @return     The current number of lines processed using DMA.
 */
int MXC_CSI2_DMA_GetCurrentLineCnt(void);

/**
 * @brief      Gets the current DMA Frame End Count when using whole frame DMA requests.
 * @note       In other words, gets the number of frames the DMA interrupt was triggered.
 * @return     The current number of frames processed using DMA.
 */
int MXC_CSI2_DMA_GetCurrentFrameEndCnt(void);

/**
 * @brief      The processing function for DMA and the CSI-2.
 * @note       When using the DMA functions, the application must call this function
 *                     periodically. This can be done within the DMA Interrupt Handler.
 * @param      a    Required input parameter for calling this function but unused.
 * @param      b    Required input parameter for calling this function but unused.
 */
void MXC_CSI2_DMA_Callback(int a, int b);

/**@} end of group csi2 */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_CSI2_H_
