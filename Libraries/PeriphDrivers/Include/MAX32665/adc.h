/**
 * @file    adc.h
 * @brief   Analog to Digital Converter(ADC) function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_ADC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_ADC_H_

/* **** Includes **** */
#include <stdint.h>
#include "adc_regs.h"
#include "mcr_regs.h"
#include "dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup adc Analog to Digital Converter (ADC)
 * @ingroup periphlibs
 * @{
 */

///< Macros to select ADC channels
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN0 ((uint32_t)(0x00000000UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN1 ((uint32_t)(0x00000001UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN2 ((uint32_t)(0x00000002UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN3 ((uint32_t)(0x00000003UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN4 ((uint32_t)(0x00000004UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN5 ((uint32_t)(0x00000005UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN6 ((uint32_t)(0x00000006UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_AIN7 ((uint32_t)(0x00000007UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VCOREA ((uint32_t)(0x00000008UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VCOREB ((uint32_t)(0x00000009UL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VRXOUT ((uint32_t)(0x0000000AUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VTXOUT ((uint32_t)(0x0000000BUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VDDA ((uint32_t)(0x0000000CUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VDDB ((uint32_t)(0x0000000DUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VDDI0 ((uint32_t)(0x0000000EUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VDDI0H ((uint32_t)(0x0000000FUL))
#define MXC_V_ADC_CTRL_ADC_CHSEL_VREGI ((uint32_t)(0x00000010UL))

/***************************************************************************************************************
                                    DATA STRUCTURES FOR ADC INITIALIZATION
***************************************************************************************************************/
/**
  * @brief  Enumeration type for the ADC Input Channels
  *
  */
typedef enum {
    MXC_ADC_CH_0 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN0, ///< Select Channel 0
    MXC_ADC_CH_1 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN1, ///< Select Channel 1
    MXC_ADC_CH_2 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN2, ///< Select Channel 2
    MXC_ADC_CH_3 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN3, ///< Select Channel 3
    MXC_ADC_CH_4 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN4, ///< Channel 0 divided by 5
    MXC_ADC_CH_5 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN5, ///< Channel 1 divided by 5
    MXC_ADC_CH_6 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN6, ///< VDDB divided by 4
    MXC_ADC_CH_7 = MXC_V_ADC_CTRL_ADC_CHSEL_AIN7, ///< VDD18 input select
    MXC_ADC_CH_VCOREA = MXC_V_ADC_CTRL_ADC_CHSEL_VCOREA, ///< VDD12 input select
    MXC_ADC_CH_VCOREB = MXC_V_ADC_CTRL_ADC_CHSEL_VCOREB,
    MXC_ADC_CH_VRXOUT = MXC_V_ADC_CTRL_ADC_CHSEL_VRXOUT, ///< VRTC divided by 2
    MXC_ADC_CH_VTXOUT = MXC_V_ADC_CTRL_ADC_CHSEL_VTXOUT, ///< TMON input select
    MXC_ADC_CH_VDDA = MXC_V_ADC_CTRL_ADC_CHSEL_VDDA,
    MXC_ADC_CH_VDDB = MXC_V_ADC_CTRL_ADC_CHSEL_VDDB,
    MXC_ADC_CH_VDDIO = MXC_V_ADC_CTRL_ADC_CHSEL_VDDI0,
    MXC_ADC_CH_VDDIOH = MXC_V_ADC_CTRL_ADC_CHSEL_VDDI0H,
    MXC_ADC_CH_VREGI = MXC_V_ADC_CTRL_ADC_CHSEL_VREGI,
} mxc_adc_chsel_t;

/**
 * @brief   Enumeration type for the ADC Monitors
 *          4 Monitors exist and can be mapped to any ADC channels
 *
 */
typedef enum {
    MXC_ADC_MONITOR_0,
    MXC_ADC_MONITOR_1,
    MXC_ADC_MONITOR_2,
    MXC_ADC_MONITOR_3,
} mxc_adc_monitor_t;

/**
 * @brief   Enumeration type for ADC Scale values
 *          Internal ADC channels automatically use the most appropriate scale
 *
 */
typedef enum {
    MXC_ADC_SCALE_2X, ///< ADC Scale by 2x (this scales ADC Reference by 1/2)
    MXC_ADC_SCALE_1, ///< ADC Scale by 1x (no scaling)
    MXC_ADC_SCALE_2, ///< ADC Scale by 1/2
    MXC_ADC_SCALE_3, ///< ADC Scale by 1/3
    MXC_ADC_SCALE_4, ///< ADC Scale by 1/4
    MXC_ADC_SCALE_6, ///< ADC Scale by 1/6 (this uses 1/3 and an additional 1/2 scaling)
    MXC_ADC_SCALE_8, ///< ADC Scale by 1/8 (this uses 1/4 and an additional 1/2 scaling)
} mxc_adc_scale_t;

/**
 * @brief   Callback used when a conversion event is complete
 *
 */
typedef void (*mxc_adc_complete_cb_t)(void *req, int error);

/**
 * @brief   Callback used when a monitor detects that a channel has reached a limit
 *
 */
typedef void (*mxc_adc_monitor_cb_t)(void *req, int error);

/**
 * @brief   Used to set up a monitor to watch a channel
 *
 */
typedef struct {
    mxc_adc_monitor_t monitor; ///< Monitor to use
    mxc_adc_scale_t scale; ///< Channel scale to use (if external channel)
    mxc_adc_chsel_t channel; ///< Channel to use
    int lowThreshold; ///< Low Threshold for monitor (RAW ADC counts)
    int highThreshold; ///< High Threshold for monitor (RAW ADC counts)
    mxc_adc_monitor_cb_t callback; ///< Function to call when the channel crosses threshold
} mxc_adc_monitor_req_t;

/**
 * @brief   Used to set up channel for conversion
 *
 */
typedef struct {
    mxc_adc_chsel_t channel; ///< Channel to use
    mxc_adc_scale_t scale; ///< Channel scale to use (if external channel)
    int rawADCValue; ///< Result of the conversion
    mxc_adc_complete_cb_t callback; ///< Function to call when callback is complete
} mxc_adc_conversion_req_t;

/**
 * @brief   Performs the ADC startup procedure
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_ADC_Init(void);

/**
 * @brief   Shuts down the ADC
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_ADC_Shutdown(void);

/**
 * @brief   Checks if the ADC is busy (performing a conversion)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_ADC_Busy(void);

/**
 * @brief   Enable specific ADC interrupts
 *
 * @param   flags mask of interrupt flags to enables
 */
void MXC_ADC_EnableInt(uint32_t flags);

/**
 * @brief   Disable specific ADC interrupts
 *
 * @param   flags mask of interrupt flags to enables
 */
void MXC_ADC_DisableInt(uint32_t flags);

/**
 * @brief   Performs the ADC startup procedure
 *
 * @return  active flags
 */
int MXC_ADC_GetFlags(void);

/**
 * @brief   Performs the ADC startup procedure
 *
 * @param   flags mask of flags to clear
 */
void MXC_ADC_ClearFlags(uint32_t flags);

/**
 * @brief   Sets the ADC conversion speed
 *
 * @param   hz conversion frequency
 *
 * @return  Actual conversion speed, or \ref MXC_Error_Codes for Error.
 */
int MXC_ADC_SetConversionSpeed(uint32_t hz);

/**
 * @brief   Gets the current ADC conversion speed
 *
 * @return  Actual conversion speed, or \ref MXC_Error_Codes for Error.
 */
int MXC_ADC_GetConversionSpeed(void);

/**
 * @brief   Set the data alignment
 *
 * @param   msbJustify set this bit to fill the 12 most significant bits of the data registers
 */
void MXC_ADC_SetDataAlignment(int msbJustify);

/**
 * @brief   Sets the scaling used for conversions on external channels
 * @note    Internal channels are approx. known and have fixed scaling
 *          Externals channels can be scaled with standard scaling (1-4x)
 *          Or by using a separate 1/2 input scale, or a 1/2 ref scale (total range 0.5-8x)
 * @param   scale requested scale or \ref mxc_adc_scale_t
 */
void MXC_ADC_SetExtScale(mxc_adc_scale_t scale);

/**
 * @brief   Enable channel high/low monitor
 * @note    This function only enables an already configured monitor
 *
 * @param   monitor The monitor to enable or \ref mxc_adc_monitor_t
 */
void MXC_ADC_EnableMonitor(mxc_adc_monitor_t monitor);

/**
 * @brief   Disable channel high/low monitor
 * @note    This function only disables an already configured monitor
 *
 * @param   monitor The monitor to disable or \ref mxc_adc_monitor_t
 */
void MXC_ADC_DisableMonitor(mxc_adc_monitor_t monitor);

/**
 * @brief   Set the high limit for a specific monitor
 * @note    setting a value of 0 disables this limit
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 * @param   threshold the limit to set
 */
void MXC_ADC_SetMonitorHighThreshold(mxc_adc_monitor_t monitor, uint32_t threshold);

/**
 * @brief   Set the high limit for a specific monitor
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 *
 * @return  the monitor's high threshold
 */
int MXC_ADC_GetMonitorHighThreshold(mxc_adc_monitor_t monitor);

/**
 * @brief   Set the low limit for a specific monitor
 * @note    setting a value of 0 disables this limit
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 * @param   threshold the limit to set
 */
void MXC_ADC_SetMonitorLowThreshold(mxc_adc_monitor_t monitor, uint32_t threshold);

/**
 * @brief   Set the low limit for a specific monitor
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 *
 * @return  the monitor's low threshold
 */
int MXC_ADC_GetMonitorLowThreshold(mxc_adc_monitor_t monitor);

/**
 * @brief   Set a monitor to use a specific channel
 * @note    The monitor must be enabled separately
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 * @param   channel the channel to monitor or \ref mxc_adc_chsel_t
 */
void MXC_ADC_SetMonitorChannel(mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel);

/**
 * @brief   Get the channel used by a monitor
 *
 * @param   monitor the monitor to set the limit on or \ref mxc_adc_monitor_t
 *
 * @return  the channel being monitored
 */
int MXC_ADC_GetMonitorChannel(mxc_adc_monitor_t monitor);

/**
 * @brief   Set a callback to be called when a monitor goes out of range
 * @note    The ADC interrupt must be enabled and MXC_ADC_Handler() called in the ISR
 *
 * @param   monitor the monitor to register callback for or \ref mxc_adc_monitor_t
 * @param   callback the function called when the limit is hit or \ref mxc_adc_monitor_cb_t
 */
void MXC_ADC_EnableMonitorAsync(mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback);

/**
 * @brief   Disable a callback for a monitor
 *
 * @param   monitor the monitor to unregister callback for or \ref mxc_adc_monitor_t
 */
void MXC_ADC_DisableMonitorAsync(mxc_adc_monitor_t monitor);

/**
 * @brief   Perform a conversion on a specific channel
 * @note    The channel must be configured separately
 *
 * @param   channel the channel to perform the conversion on or \ref mxc_adc_chsel_t
 *
 * @return  Raw conversion value, or \ref MXC_Error_Codes for error.
 */
int MXC_ADC_StartConversion(mxc_adc_chsel_t channel);

/**
 * @brief   Perform a conversion on a specific channel
 * @note    The channel must be configured separately
 *          The ADC interrupt must be enabled and MXC_ADC_Handler() called in the ISR
 *          places data in the error parameter of the callback function
 *
 * @param   channel the channel to perform the conversion on or \ref mxc_adc_chsel_t
 * @param   callback the function to call when the conversion is complete
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_ADC_StartConversionAsync(mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback);

/**
 * @brief   Perform a conversion on a specific channel
 * @note    The channel must be configured separately
 *
 * @param   channel  the channel to perform the conversion on or \ref mxc_adc_chsel_t
 * @param   dma      Pointer to DMA registers
 * @param   data     return rax adc data
 * @param   callback DMA complete callback
 *
 * @return  Raw conversion value, or \ref MXC_Error_Codes for error.
 */
int MXC_ADC_StartConversionDMA(mxc_adc_chsel_t channel, mxc_dma_regs_t *dma, uint16_t *data,
                               void (*callback)(int, int));

/**
 * @brief      Call this function from the ADC ISR when using Async API
 *             functions
 *
 * @return     Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_ADC_Handler(void);

/**
 * @brief      Perform a conversion on a specific channel
 * @note       The result will be placed back in the request structure
 *
 * @param      req   The structure containing all information for the conversion
 *
 * @return     \ref MXC_Error_Codes for error.
 */

int MXC_ADC_Convert(mxc_adc_conversion_req_t *req);

/**
 * @brief      Perform a conversion on a specific channel
 * @note       The result will be placed back in the request structure The ADC
 *             interrupt must be enabled and MXC_ADC_Handler() called in the ISR
 *
 * @param      req   The structure containing all information for the conversion
 *
 * @return     return E_NO_ERROR OR E_BUSY
 */
int MXC_ADC_ConvertAsync(mxc_adc_conversion_req_t *req);

/**
 * @brief   Monitor a specific channel for an out of range event
 * @note    synchronously waits for an out of range event to occur on the monitor
 * @param   req The structure containing all information for monitoring
 */
void MXC_ADC_Monitor(mxc_adc_monitor_req_t req);

/**
 * @brief   Monitor a specific channel for an out of range event
 * @note    If a callback is included, the ADC interrupt must be enabled
 *          and MXC_ADC_Handler() called in the ISR
 *
 * @param   req The structure containing all information for monitoring
 */
void MXC_ADC_MonitorAsync(mxc_adc_monitor_req_t req);

/**
 * @brief Gets the result from the previous ADC conversion
 * @param      outdata Pointer to store the ADC data conversion result
 * @return     #E_OVERFLOW   ADC overflow error
 * @return     #E_NO_ERROR   Data returned in \p outdata parameter
 */
int MXC_ADC_GetData(uint16_t *outdata);
/**@} end of group adc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_ADC_H_
