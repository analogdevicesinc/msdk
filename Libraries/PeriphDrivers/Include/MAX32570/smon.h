/**
 * @file    smon.h
 * @brief   Security Monitor.
 */

/* *****************************************************************************
 * Copyright(C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
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
 **************************************************************************** */

#ifndef _SMON_H_
#define _SMON_H_

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
    SMON_EXTSENSOR_0 = MXC_F_SMON_EXTSCN_EXTS_EN0,
    SMON_EXTSENSOR_1 = MXC_F_SMON_EXTSCN_EXTS_EN1,
    SMON_EXTSENSOR_2 = MXC_F_SMON_EXTSCN_EXTS_EN2,
    SMON_EXTSENSOR_3 = MXC_F_SMON_EXTSCN_EXTS_EN3,
    SMON_EXTSENSOR_4 = MXC_F_SMON_EXTSCN_EXTS_EN4,
    SMON_EXTSENSOR_5 = MXC_F_SMON_EXTSCN_EXTS_EN5,
} mxc_smon_ext_sensor_t;

/**
 * @brief   Enum for Clock Divider
 *
 */
typedef enum {
    SMON_CLK_DIVIDE_1 = MXC_S_SMON_EXTSCN_DIVCLK_DIV1,
    SMON_CLK_DIVIDE_2 = MXC_S_SMON_EXTSCN_DIVCLK_DIV2,
    SMON_CLK_DIVIDE_4 = MXC_S_SMON_EXTSCN_DIVCLK_DIV4,
    SMON_CLK_DIVIDE_8 = MXC_S_SMON_EXTSCN_DIVCLK_DIV8,
    SMON_CLK_DIVIDE_16 = MXC_S_SMON_EXTSCN_DIVCLK_DIV16,
    SMON_CLK_DIVIDE_32 = MXC_S_SMON_EXTSCN_DIVCLK_DIV32,
    SMON_CLK_DIVIDE_64 = MXC_S_SMON_EXTSCN_DIVCLK_DIV64,
} mxc_smon_clk_divide_t;

/**
 * @brief   Enum for Frequency Divider
 *
 */
typedef enum {
    SMON_FREQ_DIVIDE_4 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ2000HZ,
    SMON_FREQ_DIVIDE_8 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ1000HZ,
    SMON_FREQ_DIVIDE_16 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ500HZ,
    SMON_FREQ_DIVIDE_32 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ250HZ,
    SMON_FREQ_DIVIDE_64 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ125HZ,
    SMON_FREQ_DIVIDE_128 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ63HZ,
    SMON_FREQ_DIVIDE_256 = MXC_S_SMON_EXTSCN_EXTFRQ_FREQ31HZ,
} mxc_smon_freq_divide_t;

/**
 * @brief   Voltage Monitor Thresholds
 *
 */
typedef enum {
    SMON_VTM_THRESHOLD_1_6, ///< 1.6 V
    SMON_VTM_THRESHOLD_2_2, ///< 2.2 V
    SMON_VTM_THRESHOLD_2_8, ///< 2.8 V
} mxc_smon_vtm_t;

/**
 * @brief   Temperature Sensor Thresholds
 *
 */
typedef enum {
    SMON_TEMP_THRESHOLD_NEG_50, ///< -50 *C
    SMON_TEMP_THRESHOLD_NEG_30, ///< -30 *C
} mxc_smon_temp_t;

/**
 * @brief   Digital Fault Interrupt mode
 *
 */
typedef enum {
    SMON_DFD_INTERRUPT_NMI, ///< DRS/NMI
    SMON_DFD_INTERRUPT_PFW, ///< PFW IRQ
} mxc_smon_interrupt_mode_t;

/**
 * @brief   Digital Fault Low Power mode
 *
 */
typedef enum {
    SMON_DFD_LOWPOWER_ENABLE, ///< DFD enabled during LowPower mode
    SMON_DFD_LOWPOWER_DISABLE, ///< DFD disabled during LowPower mode
} mxc_smon_lowpower_mode_t;

/**
 * @brief   Register to check if busy
 *
 */
typedef enum {
    SMON_EXTSENSOR = MXC_F_SMON_SECST_EXTSRS,
    SMON_INTSENSOR = MXC_F_SMON_SECST_INTSRS,
    SMON_SECALARM = MXC_F_SMON_SECST_SECALRS,
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
void MXC_SMON_Init();

/**
 * @brief   Shutdown Security Monitor
 *
 */
void MXC_SMON_Shutdown();

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
 * @param   threshold   temperature threshold, \ref mxc_smon_temp_t
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
 * @param   cfg         configuration for setting up SDBE
 * @param   delay       timeout delay
 *
 * @return  int         see \ref MXC_Error_Codes for a list of return codes
 */
int MXC_SMON_SelfDestructByteEnable(mxc_smon_ext_cfg_t *cfg, uint32_t delay);

/**
 * @brief   Enables PUF Trim Erase on DRS
 *
 */
void MXC_SMON_EnablePUFTrimErase();

/**
 * @brief   Disables PUF Trim Erase on DRS
 *
 */
void MXC_SMON_DisablePUFTrimErase();

/**
 * @brief   Enbale Digital Fault Detector
 *
 * @param   interruptMode   interrupt mode, \ref mxc_smon_interrupt_mode_t
 * @param   lowPowerMode    low power mode, \ref mxc_smon_lowpower_mode_t
 * @param   delay          timeout delay
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
uint32_t MXC_SMON_GetFlags();

/**
 * @brief   Clear flags set in Security Alarm Register
 *
 * @param   flags           flags to clear from Security Alarm register
 */
void MXC_SMON_ClearFlags(uint32_t flags);

/**
 * @brief   Lock the EXTSCN register to generate DRS/NMI
 *
 */
void MXC_SMON_ExtSensorLock();

/**
 * @brief   Lock the INTSCN register to generate DRS/NMI
 *
 */
void MXC_SMON_IntSensorLock();

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

#endif /* _SMON_H_ */