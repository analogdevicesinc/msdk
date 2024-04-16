/**
 * @file    smon.h
 * @brief   Security Monitor.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SMON_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SMON_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "smon_regs.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup smon Security Monitor(SMON)
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief   External Sensor Number
 *
 */
typedef enum {
    MXC_SMON_EXTSENSOR_0 = MXC_F_SMON_EXTSCN_EXTS_EN0,

    // Deprecated Name
    SMON_EXTSENSOR_0 = MXC_SMON_EXTSENSOR_0,
} mxc_smon_ext_sensor_t;

/**
 * @brief   Enum for clock divider
 *
 */
typedef enum {
    MXC_SMON_CLK_DIVIDE_1 = MXC_S_SMON_EXTSCN_DIVCLK_DIV1,
    MXC_SMON_CLK_DIVIDE_2 = MXC_S_SMON_EXTSCN_DIVCLK_DIV2,
    MXC_SMON_CLK_DIVIDE_4 = MXC_S_SMON_EXTSCN_DIVCLK_DIV4,
    MXC_SMON_CLK_DIVIDE_8 = MXC_S_SMON_EXTSCN_DIVCLK_DIV8,
    MXC_SMON_CLK_DIVIDE_16 = MXC_S_SMON_EXTSCN_DIVCLK_DIV16,
    MXC_SMON_CLK_DIVIDE_32 = MXC_S_SMON_EXTSCN_DIVCLK_DIV32,
    MXC_SMON_CLK_DIVIDE_64 = MXC_S_SMON_EXTSCN_DIVCLK_DIV64,

    // Deprecated names
    SMON_CLK_DIVIDE_1 = MXC_SMON_CLK_DIVIDE_1,
    SMON_CLK_DIVIDE_2 = MXC_SMON_CLK_DIVIDE_2,
    SMON_CLK_DIVIDE_4 = MXC_SMON_CLK_DIVIDE_4,
    SMON_CLK_DIVIDE_8 = MXC_SMON_CLK_DIVIDE_8,
    SMON_CLK_DIVIDE_16 = MXC_SMON_CLK_DIVIDE_16,
    SMON_CLK_DIVIDE_32 = MXC_SMON_CLK_DIVIDE_32,
    SMON_CLK_DIVIDE_64 = MXC_SMON_CLK_DIVIDE_64,
} mxc_smon_clk_divide_t;

/**
 * @brief   Enum for Frequency Divider
 *
 */
typedef enum {
    MXC_SMON_FREQ_DIVIDE_4 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ2000HZ,
    MXC_SMON_FREQ_DIVIDE_8 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ1000HZ,
    MXC_SMON_FREQ_DIVIDE_16 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ500HZ,
    MXC_SMON_FREQ_DIVIDE_32 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ250HZ,
    MXC_SMON_FREQ_DIVIDE_64 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ125HZ,
    MXC_SMON_FREQ_DIVIDE_128 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ63HZ,
    MXC_SMON_FREQ_DIVIDE_256 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ31HZ,

    // Deprecated Names
    SMON_FREQ_DIVIDE_4 = MXC_SMON_FREQ_DIVIDE_4,
    SMON_FREQ_DIVIDE_8 = MXC_SMON_FREQ_DIVIDE_8,
    SMON_FREQ_DIVIDE_16 = MXC_SMON_FREQ_DIVIDE_16,
    SMON_FREQ_DIVIDE_32 = MXC_SMON_FREQ_DIVIDE_32,
    SMON_FREQ_DIVIDE_64 = MXC_SMON_FREQ_DIVIDE_64,
    SMON_FREQ_DIVIDE_128 = MXC_SMON_FREQ_DIVIDE_128,
    SMON_FREQ_DIVIDE_256 = MXC_SMON_FREQ_DIVIDE_256,
} mxc_smon_freq_divide_t;

/**
 * @brief   Voltage Monitor Thresholds
 *
 */
typedef enum {
    MXC_SMON_VTM_THD_1_6, ///< 1.6 V
    MXC_SMON_VTM_THD_2_2, ///< 2.2 V
    MXC_SMON_VTM_THD_2_8, ///< 2.8 V

    // Deprecated names
    SMON_VTM_THRESHOLD_1_6 = MXC_SMON_VTM_THD_1_6, ///< 1.6 V
    SMON_VTM_THRESHOLD_2_2 = MXC_SMON_VTM_THD_2_2, ///< 2.2 V
    SMON_VTM_THRESHOLD_2_8 = MXC_SMON_VTM_THD_2_8, ///< 2.8 V
} mxc_smon_vtm_t;

/**
 * @brief   Temperature Sensor Thresholds
 *
 */
typedef enum {
    MXC_SMON_TEMP_THD_NEG_50, ///< -50 *C
    MXC_SMON_TEMP_THD_NEG_30, ///< -30 *C

    // Deprecated names
    SMON_TEMP_THRESHOLD_NEG_50 = MXC_SMON_TEMP_THD_NEG_50, ///< -50 *C
    SMON_TEMP_THRESHOLD_NEG_30 = MXC_SMON_TEMP_THD_NEG_30, ///< -30 *C
} mxc_smon_temp_t;

/**
 * @brief   Digital Fault Interrupt mode
 *
 */
typedef enum {
    MXC_SMON_DFD_INTR_NMI, ///< DRS/NMI
    MXC_SMON_DFD_INTR_PFW, ///< PFW IRQ

    // Deprecated names
    SMON_DFD_INTERRUPT_NMI = MXC_SMON_DFD_INTR_NMI, ///< DRS/NMI
    SMON_DFD_INTERRUPT_PFW = MXC_SMON_DFD_INTR_PFW, ///< PFW IRQ
} mxc_smon_interrupt_mode_t;

/**
 * @brief   Digital Fault Low Power mode
 *
 */
typedef enum {
    MXC_SMON_LP_DFD_ENABLE, ///< DFD enabled during LowPower mode
    MXC_SMON_LP_DFD_DISABLE, ///< DFD disabled during LowPower mode

    // Deprecated
    SMON_DFD_LOWPOWER_ENABLE = MXC_SMON_LP_DFD_ENABLE, ///< DFD enabled during LowPower mode
    SMON_DFD_LOWPOWER_DISABLE = MXC_SMON_LP_DFD_DISABLE, ///< DFD disabled during LowPower mode
} mxc_smon_lowpower_mode_t;

/**
 * @brief   Register to check if busy
 *
 */
typedef enum {
    MXC_SMON_BUSY_EXTSENSOR = MXC_F_SMON_SECST_EXTSRS,
    MXC_SMON_BUSY_INTSENSOR = MXC_F_SMON_SECST_INTSRS,
    MXC_SMON_BUSY_SECALARM = MXC_F_SMON_SECST_SECALRS,
    MXC_SMON_BUSY_ALL =
        (MXC_F_SMON_SECST_EXTSRS | MXC_F_SMON_SECST_INTSRS | MXC_F_SMON_SECST_SECALRS),

    // Deprecated names
    SMON_EXTSENSOR = MXC_SMON_BUSY_EXTSENSOR,
    SMON_INTSENSOR = MXC_SMON_BUSY_INTSENSOR,
    SMON_SECALARM = MXC_SMON_BUSY_SECALARM,
} mxc_smon_busy_t;

/**
 * @brief   The information required to configure an external sensor
 *
 */
typedef struct {
    mxc_smon_ext_sensor_t sensorNumber;
    mxc_smon_clk_divide_t clockDivide;
    mxc_smon_freq_divide_t freqDivide;
    uint8_t errorCount;
    uint8_t data;
} mxc_smon_ext_cfg_t;

/**
 * @brief   Initialize Security Monitor
 *
 */
void MXC_SMON_Init(void);

/**
 * @brief   Shutdown Security Monitor
 *
 */
void MXC_SMON_Shutdown(void);

/**
 * @brief   Enables desired External Sensor
 *
 * @param   cfg         configuration for setting up external sensor
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_ExtSensorEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay);

/**
 * @brief   Set frequency for external frequency
 *
 * @param   cfg         configuration for setting up external sensor
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SetSensorFrequency(mxc_smon_ext_cfg_t *cfg);

/**
 * @brief   Set number of acceptable errors for external sensor
 *
 * @param   errorCount  Error count 0 - 31
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SetErrorCount(uint8_t errorCount);

/**
 * @brief   Enable Temperature Sensor
 *
 * @param   threshold   temperatue threshold, \ref mxc_smon_temp_t
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_TempSensorEnable(mxc_smon_temp_t threshold, uint32_t delay);

/**
 * @brief   Set Temperature Threshold
 *
 * @param   threshold   temperature threshold, \ref mxc_smon_temp_t
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SetTempThreshold(mxc_smon_temp_t threshold);

/**
 * @brief   Enable Voltage Monitor
 *
 * @param   threshold   voltage threshold, \ref mxc_smon_vtm_t
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_VoltageMonitorEnable(mxc_smon_vtm_t threshold, uint32_t delay);

/**
 * @brief   Set Voltage Monitor Threshold
 *
 * @param   threshold   voltage threshold, \ref mxc_smon_vtm_t
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SetVTMThreshold(mxc_smon_vtm_t threshold);

/**
 * @brief   Enbale Active Die Shield Monitoring
 *
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_ActiveDieShieldEnable(uint32_t delay);

/**
 * @brief   Enable Self Destruct Byte on External Sensor 0
 *
 * @param   cfg         configuration for setting up SDBE, \ref mxc_smon_ext_cfg_t
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SelfDestructByteEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay);

/**
 * @brief   Enables PUF Trim Erase on DRS
 *
 */
void MXC_SMON_EnablePUFTrimErase(void);

/**
 * @brief   Disables PUF Trim Erase on DRS
 *
 */
void MXC_SMON_DisablePUFTrimErase(void);

/**
 * @brief   Enbale Digital Fault Detector
 *
 * @param   interruptMode   interrupt mode, \ref mxc_smon_interrupt_mode_t
 * @param   lowPowerMode    low power mode, \ref mxc_smon_lowpower_mode_t
 * @param   delay           timeout delay
 *
 * @return  int            see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_DigitalFaultDetectorEnable(mxc_smon_interrupt_mode_t interruptMode,
                                        mxc_smon_lowpower_mode_t lowPowerMode, uint32_t delay);

/**
 * @brief   Get Flags set in Security Alarm Register
 *
 * @return  uint32_t        SECALM register
 */
uint32_t MXC_SMON_GetFlags(void);

/**
 * @brief   Clear flags set in Security Alarm Register
 *
 * @param   flags       flags to clear from Security Alarm register
 */
void MXC_SMON_ClearFlags(uint32_t flags);

/**
 * @brief   Lock the EXTSCN register to generate DRS/NMI
 *
 */
void MXC_SMON_ExtSensorLock(void);

/**
 * @brief   Lock the INTSCN register to generate DRS/NMI
 *
 */
void MXC_SMON_IntSensorLock(void);

/**
 * @brief   Checks if the registers are busy before wirting to it
 *
 * @param   reg         see \ref mxc_smon_busy_t for registers
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_isBusy(mxc_smon_busy_t reg, uint32_t delay);

#ifdef __cplusplus
}
#endif

/**@} end of group smon  */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_SMON_H_
