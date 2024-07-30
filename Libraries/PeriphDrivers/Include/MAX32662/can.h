/**
 * @file    can.h
 * @brief   CAN function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_CAN_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_CAN_H_

/* **** Includes **** */
#include <stdint.h>
#include <stdbool.h>
#include "can_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup can CAN
 * @ingroup periphlibs
 * @{
 */

#define MXC_CAN_FILT_PER_OBJ 2

// SEG1 = PROPAGATION_SEGMENT + PHASE_SEG1
#define MXC_CAN_SEG1_SHIFT 16
#define MXC_CAN_SEG1(seg1_tq) ((seg1_tq & 0xFF) << MXC_CAN_SEG1_SHIFT)
#define MXC_CAN_SEG2_SHIFT 8
#define MXC_CAN_SEG2(seg2_tq) ((seg2_tq & 0xFF) << MXC_CAN_SEG2_SHIFT)
#define MXC_CAN_SJW_SHIFT 0
#define MXC_CAN_SJW(sjw_tq) ((sjw_tq & 0xFF) << MXC_CAN_SJW_SHIFT)
#define MXC_CAN_BIT_SEGMENTS(seg1_tq, seg2_tq, sjw_tq) \
    (MXC_CAN_SEG1(seg1_tq) | MXC_CAN_SEG2(seg2_tq) | MXC_CAN_SJW(sjw_tq))

#define MXC_CAN_NOMINAL_MAX_SEG1TQ 16
#define MXC_CAN_NOMINAL_MAX_SEG2TQ 8
#define MXC_CAN_NOMINAL_MAX_SJWTQ 4
#define MXC_CAN_NOMINAL_MAX_PRESCALER 0x400
#define MXC_CAN_FD_DATA_MAX_SEG1TQ 64
#define MXC_CAN_FD_DATA_MAX_SEG2TQ 16
#define MXC_CAN_FD_DATA_MAX_SJWTQ 16
#define MXC_CAN_FD_DATA_MAX_PRESCALER 0x400

#define MXC_CAN_FILT_OP_TYPE_SHIFT 0
#define MXC_CAN_FILT_OP_TYPE_MASK 0x000F
#define MXC_CAN_FILT_SEL_SHIFT 4
#define MXC_CAN_FILT_SEL_MASK 0x00F0

#define MXC_CAN_MSG_INFO_IDE_BIT 0x80000000
#define MXC_CAN_STANDARD_ID(id) (id & 0x7FFUL)
#define MXC_CAN_EXTENDED_ID(id) ((id & 0x1FFFFFFFUL) | MXC_CAN_MSG_INFO_IDE_BIT)

#define MXC_CAN_BUF_CFG_IDE 0x80
#define MXC_CAN_BUF_CFG_SRR 0x40
#define MXC_CAN_BUF_CFG_RTR(rtr) ((!!rtr) << 6)
#define MXC_CAN_BUF_CFG_FDF(fdf) ((!!fdf) << 5)
#define MXC_CAN_BUF_CFG_BRS(brs) ((!!brs) << 4)
#define MXC_CAN_BUF_CFG_DLC(dlc) (dlc & 0xF)

#define MXC_CAN_BUF_CFG_EXT_ID_TX1(id) ((id & (0xFF << 21)) >> 21)
#define MXC_CAN_BUF_CFG_EXT_ID_TX2(id) ((id & (0xFF << 13)) >> 13)
#define MXC_CAN_BUF_CFG_EXT_ID_TX3(id) ((id & (0xFF << 5)) >> 5)
#define MXC_CAN_BUF_CFG_EXT_ID_TX4(id) ((id & 0x1F) << 3)
#define MXC_CAN_BUF_CFG_EXT_ID_RX1(rxdata) (rxdata << 21)
#define MXC_CAN_BUF_CFG_EXT_ID_RX2(rxdata) (rxdata << 13)
#define MXC_CAN_BUF_CFG_EXT_ID_RX3(rxdata) (rxdata << 5)
#define MXC_CAN_BUF_CFG_EXT_ID_RX4(rxdata) ((rxdata & 0xF8) >> 3)
#define MXC_CAN_BUF_CFG_EXT_ID_RTR(rtr) ((!!rtr) << 2)
#define MXC_CAN_BUF_CFG_EXT_ID_ESI(esi) ((!!esi) << 1)

#define MXC_CAN_BUF_CFG_STD_ID_TX1(id) ((id & (0xFF << 3)) >> 3)
#define MXC_CAN_BUF_CFG_STD_ID_TX2(id) ((id & 0x7) << 5)
#define MXC_CAN_BUF_CFG_STD_ID_RX1(rxdata) (rxdata << 3)
#define MXC_CAN_BUF_CFG_STD_ID_RX2(rxdata) ((rxdata & 0xE0) >> 5)
#define MXC_CAN_BUF_CFG_STD_ID_RTR(rtr) ((!!rtr) << 6)
#define MXC_CAN_BUF_CFG_STD_ID_ESI(esi) ((!!esi) << 3)

#define MXC_CAN_LEC_NO_ERR 0U
#define MXC_CAN_LEC_BIT_ERR 1U
#define MXC_CAN_LEC_STUFF_ERR 2U
#define MXC_CAN_LEC_CRC_ERR 3U
#define MXC_CAN_LEC_FORM_ERR 4U
#define MXC_CAN_LEC_ACK_ERR 5U
#define MXC_CAN_ECC_ERROR_CODE_MASK                                                   \
    (MXC_F_CAN_REVA_ECC_ACKER | MXC_F_CAN_REVA_ECC_FRMER | MXC_F_CAN_REVA_ECC_CRCER | \
     MXC_F_CAN_REVA_ECC_STFER | MXC_F_CAN_REVA_ECC_BER)

#define MXC_CAN_UNIT_STATE_INACTIVE 0U
#define MXC_CAN_UNIT_STATE_ACTIVE 1U
#define MXC_CAN_UNIT_STATE_PASSIVE 2U
#define MXC_CAN_UNIT_STATE_BUS_OFF 3U

#define MXC_CAN_DMA_LEN(msg_id) (msg_id & MXC_CAN_MSG_INFO_IDE_BIT ? 5 : 3)

#define MXC_CAN_TXSCNT_MAX 0x80
#define MXC_CAN_ERRPSV_THRESH 0x80

/**
 * @brief  Struct containing information about the version of the CAN library
 */
typedef struct {
    uint16_t api; ///< CMSIS API Version
    uint16_t drv; ///< Maxim CAN SDK Version
} mxc_can_drv_version_t;

/**
 * @brief  Struct containing the capabilities of the CAN driver.
 */
typedef struct {
    uint32_t num_objects; ///< Number of objects available
    uint32_t
        reentrant_operation; ///< Reentrant calls to MessageSend/Read, ObjectConfigure, and CAN_Control supported
    uint32_t fd_mode; ///< CAN FD supported
    uint32_t restricted_mode; ///< Restricted mode supported
    uint32_t monitor_mode; ///< Monitor mode supported
    uint32_t internal_loopback; ///< Internal loopback supported
    uint32_t external_loopback; ///< External loopback supported
    uint32_t rsv; ///< Reserved for future use
} mxc_can_capabilities_t;

/**
 * @brief  Selects power state of the CAN peripherals
 */
typedef enum {
    MXC_CAN_PWR_CTRL_OFF, ///< Shut off power to peripherals
    MXC_CAN_PWR_CTRL_SLEEP, ///< Put peripherals to sleep
    MXC_CAN_PWR_CTRL_FULL, ///< Peripherals fully awake
} mxc_can_pwr_ctrl_t;

/**
 * @brief  Selects which bitrate to perform operation on
 */
typedef enum {
    MXC_CAN_BITRATE_SEL_NOMINAL, ///< Set bitrate for classic CAN frames
    MXC_CAN_BITRATE_SEL_FD_DATA, ///< Reserved for future use. Not supported on MAX32662, included to prevent build errors.
} mxc_can_bitrate_sel_t;

/**
 * @brief  Selects the CAN driver's mode of operation
 */
typedef enum {
    MXC_CAN_MODE_INITIALIZATION, ///< Reset mode
    MXC_CAN_MODE_NORMAL, ///< Normal operating mode
    MXC_CAN_MODE_RESTRICTED, ///< Restricted mode
    MXC_CAN_MODE_MONITOR, ///< Listen-only mode
    MXC_CAN_MODE_LOOPBACK, ///< Loopback mode
    MXC_CAN_MODE_RSV, ///< Reserved for future use
    MXC_CAN_MODE_LOOPBACK_W_TXD, ///< Loopback mode with transmit pin disconnected
} mxc_can_mode_t;

/**
 * @brief  Struct containing information about the objects associated with a particular CAN driver.
 */
typedef struct {
    int32_t tx; ///< Object supports transmission
    int32_t rx; ///< Object supports receive
    int32_t rx_rtr_tx_data; ///< Object supports RTR reception and automatic data frame transmission
    int32_t tx_rtr_rx_data; ///< Object supports RTR transmission and automatic data fram reception
    int32_t multiple_filters; ///< Number of filters supported by the object
    int32_t exact_filtering; ///< Object can support exact message ID filters
    int32_t mask_filtering; ///< Object can support mask message ID filters
    int32_t range_filtering; ///< Object can support range message ID filters
    int32_t message_depth; ///< Message depth of transmit and receive buffers
    int32_t reserved; ///< Reserved for future use
} mxc_can_obj_capabilities_t;

/**
 * @brief  Type used to select which operation to perform on message ID filter. Mask together one choice from each group to make configuration selection
 */
typedef enum {
    // Select one from Group 1 {
    MXC_CAN_FILT_CFG_EXACT_ADD = 0, ///< Add exact filter
    MXC_CAN_FILT_CFG_EXACT_DEL = 1, ///< Remove exact filter
    MXC_CAN_FILT_CFG_RSV1 = 2, ///< NOT SUPPORTED ON MAX32690
    MXC_CAN_FILT_CFG_RSV2 = 3, ///< NOT SUPPORTED ON MAX32690
    MXC_CAN_FILT_CFG_MASK_ADD = 4, ///< Add maskable filter
    MXC_CAN_FILT_CFG_MASK_DEL = 5, ///< Remove maskable filter
    // } end group 1

    // Select one from group 2 {
    MXC_CAN_FILT_CFG_DUAL_GEN = 0
                                << MXC_CAN_FILT_SEL_SHIFT, ///< Reccomended only for middleware use
    MXC_CAN_FILT_CFG_DUAL1_STD_ID =
        1
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on dual filter 1, for 11-bit message ID's
    MXC_CAN_FILT_CFG_DUAL1_EXT_ID =
        2
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on dual filter 1, for 29-bit message ID's
    MXC_CAN_FILT_CFG_DUAL2_STD_ID =
        3
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on dual filter 2, for 11-bit message ID's
    MXC_CAN_FILT_CFG_DUAL2_EXT_ID =
        4
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on dual filter 2, for 29-bit message ID's
    MXC_CAN_FILT_CFG_SINGLE_STD_ID =
        5
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on single filter, for 11-bit message ID's
    MXC_CAN_FILT_CFG_SINGLE_EXT_ID =
        6
        << MXC_CAN_FILT_SEL_SHIFT, ///< Perform operation on single filter, for 29-bit message ID's
    // } end group 2
} mxc_can_filt_cfg_t;

/**
 * @brief  Struct detailing the current status of the CAN driver.
 */
typedef struct {
    uint32_t unit_state; ///< State of the CAN bus
    uint32_t last_error_code; ///< Last error code recorded
    uint32_t tx_err_cnt; ///< Number of transmission errors
    uint32_t rx_err_cnt; ///< Number of receive errors
    uint32_t can_idx; ///< Index of CAN peripheral status was retrieved for
} mxc_can_stat_t;

/**
 * @brief  Contains information about the message to be sent or the message received.
 */
typedef struct {
    uint32_t msg_id; ///< Message ID
    uint32_t rtr; ///< Remote transmit request frame
    uint32_t fdf; ///< FD frame
    uint32_t brs; ///< FD format bit rate switch
    uint32_t esi; ///< FD format error state indicator
    uint32_t dlc; ///< Data length code
    uint32_t rsv; ///< Reserved for future use
} mxc_can_msg_info_t;

/**
 * @brief  Used to set the features available to a CAN object
 */
typedef enum {
    MXC_CAN_OBJ_CFG_INACTIVE, ///< Object disabled
    MXC_CAN_OBJ_CFG_TXRX, ///< Object can transmit and/or receive messages
    MXC_CAN_OBJ_CFG_RSV, ///< Reserved for future use
    MXC_CAN_OBJ_CFG_RX_RTR_TX_DATA, ///< NOT SUPPORTED ON MAX32690
    MXC_CAN_OBJ_CFG_TX_RTR_RX_DATA, ///< NOT SUPPORTED ON MAX32690
} mxc_can_obj_cfg_t;

/**
 * @brief  Selects the control operation for the CAN driver to perform
 */
typedef enum {
    MXC_CAN_CTRL_SET_FD_MODE, ///< No effect on MAX32690 (FD mode always enabled when CAN active)
    MXC_CAN_CTRL_ABORT_TX, ///< Abort transmission
    MXC_CAN_CTRL_RETRANSMISSION, ///< Enable/disable auto retransmission on error
    MXC_CAN_CTRL_TRANSCEIVER_DLY, ///< Set transceiver delay
} mxc_can_ctrl_t;

/**
 * @brief  State which bus has entered to trigger unit event
 */
typedef enum {
    MXC_CAN_UNIT_EVT_INACTIVE, ///< Peripherals entered inactive state (sleep, shutdown)
    MXC_CAN_UNIT_EVT_ACTIVE, ///< Peripherals entered active state
    MXC_CAN_UNIT_EVT_WARNING, ///< Peripheral received error warning
    MXC_CAN_UNIT_EVT_PASSIVE, ///< Peripheral entered passive state
    MXC_CAN_UNIT_EVT_BUS_OFF, ///< Bus turned off
} mxc_can_unit_evt_t;

/**
 * @brief  Selects which object to notify/handle
 */
typedef enum {
    MXC_CAN_OBJ_EVT_TX_COMPLETE, ///< Transmission complete
    MXC_CAN_OBJ_EVT_RX, ///< Message received
    MXC_CAN_OBJ_EVT_RX_OVERRUN, ///< RXFIFO overflow
} mxc_can_obj_evt_t;

/**
 * @brief  Struct containing information about CAN message
 */
typedef struct {
    mxc_can_msg_info_t
        *msg_info; ///< Pointer to struct containing information about the format of the message
    uint8_t *
        data; ///< Pointer to array of data bytes (either data to transmit or where to store data received)
    uint8_t
        data_sz; ///< MessageSend - number of data bytes to transmit, MessageRead - maximum number of data bytes that can be stored in "data"
} mxc_can_req_t;

///< Callback used when a bus event occurs
typedef void (*mxc_can_unit_event_cb_t)(uint32_t can_idx, uint32_t event);

///< Callback used when a transmission event occurs
typedef void (*mxc_can_object_event_cb_t)(uint32_t can_idx, uint32_t event);

/**
 * @brief   Get information about the version of the CMSIS API and Maxim CAN SDK
 *
 * @return  Struct containing version information
 */
mxc_can_drv_version_t MXC_CAN_GetVersion(void);

/**
 * @brief   Get information about the capabilities of the Maxim CAN SDK
 *
 * @return  Struct containing capabilities information
 */
mxc_can_capabilities_t MXC_CAN_GetCapabilities(void);

/**
 * @brief   Initializes CAN event callbacks
 * @note    On default this function enables CAN peripheral clock and CAN gpio pins.
 *          if you wish to manage clock and gpio related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file.
 *          By this flag this function will remove clock and gpio related codes from file.
 * 
 * @param can_idx   Index of the CAN peripheral to initialize
 * @param cfg       Specifies how to configure CAN peripheral (see MXC_CAN_ObjectConfigure)
 * @param unit_cb   Pointer to unit event callback function
 * @param obj_cb    Pointer to object event callback function
 * @param map       Selects pin mapping (0 - CANA Pins; 1 - CANB Pins)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_Init(uint32_t can_idx, mxc_can_obj_cfg_t cfg, mxc_can_unit_event_cb_t unit_cb,
                 mxc_can_object_event_cb_t obj_cb, uint8_t map);

/**
 * @brief   Free CAN resources (does not reset or disable CAN peripherals)
 * @note    On default this function enables CAN peripheral clock.
 *          if you wish to manage clock related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file.
 *          By this flag this function will remove clock related codes from file.
 *
 * @param can_idx   Index of CAN peripheral to un-initialize (shutdown)
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_UnInit(uint32_t can_idx);

/**
 * @brief   Change Power state of the CAN peripherals
 * @note    On default this function enables CAN peripheral clock.
 *          if you wish to manage clock related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file.
 *          By this flag this function will remove clock related codes from file.
 * 
 * @param can_idx   Index of CAN peripheral to alter power settings for
 * @param pwr       Desired power state of the CAN peripherals
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_PowerControl(uint32_t can_idx, mxc_can_pwr_ctrl_t pwr);

/**
 * @brief   Enables interrupts in the interrupt and extended interrupt enable registetrs
 * 
 * @param can_idx   Index of the CAN peripheral to enable interrupts for (0 - CAN0, 1 - CAN1)
 * @param en        Mask of interrupts to enable in the interrupt enable register
 * @param ext_en    Mask of interrupts to enable in the extended interrupt enable register
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_EnableInt(uint32_t can_idx, uint8_t en, uint8_t ext_en);

/**
 * @brief   Disables interrupts in the interrupt and extended interrupt enable registers
 * 
 * @param can_idx   Index of the CAN peripheral to disable interrupts for (0 - CAN0, 1 - CAN1)
 * @param dis       Mask of interrupts to disable in the interrupt enable register
 * @param ext_dis   Mask of interrupts to disable in the extended interrupt enable register
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_DisableInt(uint32_t can_idx, uint8_t dis, uint8_t ext_dis);

/**
 * @brief   Reads interrupt status flags
 * 
 * @param can_idx   Retrieve interrupt flags for CAN peripheral specified by this parameter (0 - CAN0, 1 - CAN1)
 * @param flags     Interrupt status flags in the INTFL register
 * @param ext_flags Interrupt status flags in the EINTFL register
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_GetFlags(uint32_t can_idx, uint8_t *flags, uint8_t *ext_flags);

/**
 * @brief   Clears interrupts flags
 * 
 * @param can_idx   Clear interrupt flags for CAN peripheral specified by this parameter (0 - CAN0, 1 - CAN1)
 * @param flags     Mask of interrupt flags to clear in INTFL register
 * @param ext_flags Mask of interrupt flags to clear in EINTFL register
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_ClearFlags(uint32_t can_idx, uint8_t flags, uint8_t ext_flags);

/**
 * @brief   Returns the bit rate of the CAN clock
 *
 * @param can_idx   Index of CAN peripheral to get clock rate of
 * 
 * @return  Frequency of CAN clock
 */
int MXC_CAN_GetClock(uint32_t can_idx);

/**
 * @brief   Returns the bit rate of the CAN clock
 * 
 * @param can_idx   Selects CAN peripheral (0 - CAN0, 1 - CAN1) to retrieve bit rate for
 * @param sel       Select which bitrate to return
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. If successful, returns value of bit rate.
 * 
 * @warning MAX32662 does not support CAN FD, passing MXC_CAN_BITRATE_SEL_FD_DATA will return an error.
 */
int MXC_CAN_GetBitRate(uint32_t can_idx, mxc_can_bitrate_sel_t sel);

/**
 * @brief   Sets CAN clock frequency and sets time quanta values 
 * 
 * @param can_idx       Index of CAN peripheral to set bitrate for
 * @param sel           Selects which bitrate to set (nominal/FD arbitration phase or FD data phase)
 * @param bitrate       Desired bitrate
 * @param bit_segments  Mask of number of time quanta in each bit segment see MXC_CAN_BIT_SEGMENTS(seg1_tq, seg2_tq, sjw_tq) defined above     
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, passing MXC_CAN_BITRATE_SEL_FD_DATA will return an error.
 */
int MXC_CAN_SetBitRate(uint32_t can_idx, mxc_can_bitrate_sel_t sel, uint32_t bitrate,
                       uint32_t bit_segments);

/**
 * @brief   Sets the operating mode of the CAN peripherals
 * 
 * @param can_idx   Index of CAN peripheral
 * @param mode      Selects the mode of the CAN peripherals
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_SetMode(uint32_t can_idx, mxc_can_mode_t mode);

/**
 * @brief   Get the capabilities of the CAN object specified by can_idx
 *
 * @param can_idx   Index of the CAN peripheral to get capabilities of
 * 
 * @return  Object capabilities information
 */
mxc_can_obj_capabilities_t MXC_CAN_ObjectGetCapabilities(uint32_t can_idx);

/**
 * @brief   Setup message ID filter on CAN peripheral
 * 
 * @param can_idx   Pointer to CAN instance
 * @param cfg       Specifies how the filter should be configured
 * @param id        Exact ID for exact filter type. Base ID for maskable filter type.
 * @param arg       Mask for maskable filter type. Bits set to 0 are "don't care" bits and will always be accepted regardless of value, 1's are compared with ID. (Inverse of AMR register function.)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_ObjectSetFilter(uint32_t can_idx, mxc_can_filt_cfg_t cfg, uint32_t id, uint32_t arg);

/**
 * @brief   Configure CAN object
 * @note    On default this function enables CAN gpio pins.
 *          if you wish to manage gpio related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file.
 *          By this flag this function will remove gpio related codes from file.
 * 
 * @param can_idx   Index of CAN peripheral instance
 * @param cfg       Specifies how the filter should be configured
 * @param map       Selects pin mapping (0 - CANA Pins; 1 - CANB Pins)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_ObjectConfigure(uint32_t can_idx, mxc_can_obj_cfg_t cfg, uint8_t map);

/**
 * @brief   Write data to be transmitted to TX FIFO (does not need to be called before message send)
 * 
 * @param can_idx   Index of CAN peripheral to read RX data for
 * @param info      Pointer to struct containing information about the type of CAN message to send
 * @param data      Buffer of data bytes to be transmitted
 * @param size      Number of data bytes in "data"
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting info->fdf will return an error.
 */
int MXC_CAN_WriteTXFIFO(uint32_t can_idx, mxc_can_msg_info_t *info, const uint8_t *data,
                        uint8_t size);

/**
 * @brief   Reads data from RX FIFO if data available
 * 
 * @param can_idx   Index of CAN peripheral to read RX data for
 * @param info      Pointer to struct to store message information in
 * @param data      Buffer to store received data bytes
 * @param size      Maximum number of data bytes that can be stored in "data"
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting info->fdf will return an error.
 */
int MXC_CAN_ReadRXFIFO(uint32_t can_idx, mxc_can_msg_info_t *info, uint8_t *data, uint8_t size);

/**
 * @brief   Send message (this is a blocking function).
 * 
 * @param can_idx   Index of the CAN peripheral to send the message from
 * @param req       Contains information about the format and data of message to send
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 */
int MXC_CAN_MessageSend(uint32_t can_idx, mxc_can_req_t *req);

/**
 * @brief   Send message (non-blocking).
 * 
 * @param can_idx   Index of the CAN peripheral to send the message from
 * @param req       Contains information about the format and data of message to send
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 * @warning The structure pointed to by 'req' must remain unchanged and in scope until the 
 *          TX complete event has been signaled.
 */
int MXC_CAN_MessageSendAsync(uint32_t can_idx, mxc_can_req_t *req);

/**
 * @brief   Send message (non-blocking DMA).
 * 
 * @param can_idx   Index of the CAN peripheral to send the message from
 * @param req       Contains information about the format and data of message to send
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 * @warning The structure pointed to by 'req' must remain unchanged and in scope until the 
 *          TX complete event has been signaled.
 */
int MXC_CAN_MessageSendDMA(uint32_t can_idx, mxc_can_req_t *req);

/**
 * @brief   Read received message if any. (this is a blocking function)
 * 
 * @param can_idx   Index of the CAN peripheral to read the message for
 * @param req       Pointer to struct that stores CAN message information
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 */
int MXC_CAN_MessageRead(uint32_t can_idx, mxc_can_req_t *req);

/**
 * @brief   Set up CAN device for asynchronus data receive (non-blocking)
 * 
 * @param can_idx   Index of the CAN peripheral to set up asynchronus reads for
 * @param req       Pointer to struct that stores CAN message information
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 * @warning The structure pointed to by 'req' must remain unchanged and in scope until the 
 *          RX complete event has been signaled.
 */
int MXC_CAN_MessageReadAsync(uint32_t can_idx, mxc_can_req_t *req);

/**
 * @brief   Set up CAN device for DMA data receive
 * 
 * @param can_idx   Index of the CAN peripheral to receive data
 * @param req       Pointer to struct that stores CAN message information, initialize "msg_info" to expected configuration of the message to be received (Needed to ensure proper DMA length is set, if these are not known use MXC_CAN_MessageReadAsync instead.) 
 * @param dma_cb    Pointer to DMA callback function.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, setting req->msg_info->fdf will return an error.
 * @warning The structure pointed to by 'req' must remain unchanged and in scope until the 
 *          RX complete event has been signaled.
 */
int MXC_CAN_MessageReadDMA(uint32_t can_idx, mxc_can_req_t *req, void (*dma_cb)(int, int));

/**
 * @brief   General interrupt handler for MessageSendAsync and MessageReadAsync
 * 
 * @param can_idx   Index of the CAN peripheral to handle interrupts for
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_Handler(uint32_t can_idx);

/**
 * @brief   Perform control operation on CAN peripheral(s)
 * 
 * @param can_idx   Index of CAN peripheral to perform control function on
 * @param ctrl      Operation to perform on the CAN peripherals
 * @param ctrl_arg  Depends on ctrl. RETRANSMISSION: 1-Enable, 0-Disable; TRANSCEIVER_DELAY: number of time quanta to delay; any other value of control this parameter is ignored
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 * 
 * @warning MAX32662 does not support CAN FD, passing MXC_CAN_CTRL_SET_FD_MODE will return an error.
 */
int MXC_CAN_Control(uint32_t can_idx, mxc_can_ctrl_t ctrl, uint32_t ctrl_arg);

/**
 * @brief   Configure wakeup timer settings (must be called before entering sleep mode)
 * 
 * @param can_idx           Index of CAN peripheral to configure wakeup timer for
 * @param prescaler         Value to scale the CAN clock by to generate the wakup clock signal
 * @param wup_filter_tm     Value to set Wake-up filter time register to 
 * @param wup_expire_tm     Value to set wake-up expire time register to
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CAN_SetWakeupTimer(uint32_t can_idx, uint8_t prescaler, uint16_t wup_filter_tm,
                           uint32_t wup_expire_tm);

/*
 * @brief   Get status of the bus unit
 *
 * @param can_idx   Index of CAN peripheralto get status of
 *
 * @return  Information about the bus and error status 
 */
mxc_can_stat_t MXC_CAN_GetStatus(uint32_t can_idx);

/**
 * @brief   Notify unit event handler of event that transpired
 * 
 * @param can_idx   Index of CAN peripheral which the event transpired on
 * 
 * @param event     Event that occured
 */
void MXC_CAN_SignalUnitEvent(uint32_t can_idx, mxc_can_unit_evt_t event);

/**
 * @brief   Notify object event handler of event that transpired
 * 
 * @param can_idx   Index of the CAN peripheral which had the event
 * @param event     Event that occured
 */
void MXC_CAN_SignalObjectEvent(uint32_t can_idx, mxc_can_obj_evt_t event);

/**@} end of group can */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_CAN_H_
