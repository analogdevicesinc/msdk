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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_

#include <stddef.h>
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "smon.h"
#include "smon_reva_regs.h"

/**
 * @brief   External Sensor Number
 *
 */
typedef enum {
    SMON_REVA_EXTSENSOR_0 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN0,
    SMON_REVA_EXTSENSOR_1 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN1,
    SMON_REVA_EXTSENSOR_2 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN2,
    SMON_REVA_EXTSENSOR_3 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN3,
    SMON_REVA_EXTSENSOR_4 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN4,
    SMON_REVA_EXTSENSOR_5 = MXC_F_SMON_REVA_EXTSCN_EXTS_EN5,
} mxc_smon_reva_ext_sensor_t;

/**
 * @brief   Enum for Clock Divider
 *
 */
typedef enum {
    SMON_REVA_CLK_DIVIDE_1 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV1,
    SMON_REVA_CLK_DIVIDE_2 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV2,
    SMON_REVA_CLK_DIVIDE_4 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV4,
    SMON_REVA_CLK_DIVIDE_8 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV8,
    SMON_REVA_CLK_DIVIDE_16 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV16,
    SMON_REVA_CLK_DIVIDE_32 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV32,
    SMON_REVA_CLK_DIVIDE_64 = MXC_S_SMON_REVA_EXTSCN_DIVCLK_DIV64,
} mxc_smon_reva_clk_divide_t;

/**
 * @brief   Enum for Frequency Divider
 *
 */
typedef enum {
    SMON_REVA_FREQ_DIVIDE_4 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ2000HZ,
    SMON_REVA_FREQ_DIVIDE_8 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ1000HZ,
    SMON_REVA_FREQ_DIVIDE_16 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ500HZ,
    SMON_REVA_FREQ_DIVIDE_32 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ250HZ,
    SMON_REVA_FREQ_DIVIDE_64 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ125HZ,
    SMON_REVA_FREQ_DIVIDE_128 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ63HZ,
    SMON_REVA_FREQ_DIVIDE_256 = MXC_S_SMON_REVA_EXTSCN_EXTFRQ_FREQ31HZ,
} mxc_smon_reva_freq_divide_t;

/**
 * @brief   Voltage Monitor Thresholds
 *
 */
typedef enum {
    SMON_REVA_VTM_THRESHOLD_1_6, ///< 1.6 V
    SMON_REVA_VTM_THRESHOLD_2_2, ///< 2.2 V
    SMON_REVA_VTM_THRESHOLD_2_8, ///< 2.8 V
} mxc_smon_reva_vtm_t;

/**
 * @brief   Temperature Sensor Thresholds
 *
 */
typedef enum {
    SMON_REVA_TEMP_THRESHOLD_NEG_50, ///< -50 *C
    SMON_REVA_TEMP_THRESHOLD_NEG_30, ///< -30 *C
} mxc_smon_reva_temp_t;

/**
 * @brief   Digital Fault Interrupt mode
 *
 */
typedef enum {
    SMON_REVA_DFD_INTERRUPT_NMI, ///< DRS/NMI
    SMON_REVA_DFD_INTERRUPT_PFW, ///< PFW IRQ
} mxc_smon_reva_interrupt_mode_t;

/**
 * @brief   Digital Fault Low Power mode
 *
 */
typedef enum {
    SMON_REVA_DFD_LOWPOWER_ENABLE, ///< DFD enabled during LowPower mode
    SMON_REVA_DFD_LOWPOWER_DISABLE, ///< DFD disabled during LowPower mode
} mxc_smon_reva_lowpower_mode_t;

/**
 * @brief   Register to check if busy
 *
 */
typedef enum {
    SMON_REVA_EXTSENSOR = MXC_F_SMON_REVA_SECST_EXTSRS,
    SMON_REVA_INTSENSOR = MXC_F_SMON_REVA_SECST_INTSRS,
    SMON_REVA_SECALARM = MXC_F_SMON_REVA_SECST_SECALRS,
} mxc_smon_reva_busy_t;

/**
 * @brief   The information required to configure an external sensor
 *
 */
typedef struct {
    mxc_smon_reva_ext_sensor_t sensorNumber;
    mxc_smon_reva_clk_divide_t clockDivide;
    mxc_smon_reva_freq_divide_t freqDivide;
    uint8_t errorCount;
    uint8_t data;
} mxc_smon_reva_ext_cfg_t;

int MXC_SMON_RevA_ExtSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg,
                                  uint32_t delay);

int MXC_SMON_RevA_SetSensorFrequency(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg);

int MXC_SMON_RevA_SetErrorCount(mxc_smon_reva_regs_t *smon, uint8_t errorCount);

int MXC_SMON_RevA_TempSensorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_temp_t threshold,
                                   uint32_t delay);

int MXC_SMON_RevA_SetTempThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_reva_temp_t threshold);

int MXC_SMON_RevA_VoltageMonitorEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_vtm_t threshold,
                                       uint32_t delay);

int MXC_SMON_RevA_SetVTMThreshold(mxc_smon_reva_regs_t *smon, mxc_smon_reva_vtm_t threshold);

int MXC_SMON_RevA_ActiveDieShieldEnable(mxc_smon_reva_regs_t *smon, uint32_t delay);

int MXC_SMON_RevA_SelfDestructByteEnable(mxc_smon_reva_regs_t *smon, mxc_smon_reva_ext_cfg_t *cfg,
                                         uint32_t delay);

void MXC_SMON_RevA_EnablePUFTrimErase(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_DisablePUFTrimErase(mxc_smon_reva_regs_t *smon);

int MXC_SMON_RevA_DigitalFaultDetectorEnable(mxc_smon_reva_regs_t *smon,
                                             mxc_smon_reva_interrupt_mode_t interruptMode,
                                             mxc_smon_reva_lowpower_mode_t lowPowerMode,
                                             uint32_t delay);

uint32_t MXC_SMON_RevA_GetFlags(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_ClearFlags(mxc_smon_reva_regs_t *smon, uint32_t flags);

void MXC_SMON_RevA_ExtSensorLock(mxc_smon_reva_regs_t *smon);

void MXC_SMON_RevA_IntSensorLock(mxc_smon_reva_regs_t *smon);

int MXC_SMON_RevA_isBusy(mxc_smon_reva_regs_t *smon, mxc_smon_reva_busy_t reg, uint32_t delay);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SMON_SMON_REVA_H_
