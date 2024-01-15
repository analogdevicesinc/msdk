/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7320_DRIVER_H_
#define LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7320_DRIVER_H_

#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADT7320 registers */
#define ADT7320_00_STATUS 0x00
#define ADT7320_01_CONFIGURATION 0x01
#define ADT7320_02_TEMPERATURE_VALUE 0x02
#define ADT7320_03_ID 0x03
#define ADT7320_04_TCRIT_SETPOINT 0x04
#define ADT7320_05_THYST_SETPOINT 0x05
#define ADT7320_06_THIGH_SETPOINT 0x06
#define ADT7320_07_TLOW_SETPOINT 0x07

/* ADT7320_REG_STATUS definition */
#define ADT7320_STATUS_T_LOW (1 << 4)
#define ADT7320_STATUS_T_HIGH (1 << 5)
#define ADT7320_STATUS_T_CRIT (1 << 6)
#define ADT7320_STATUS_RDY (1 << 7)

/* ADT7320_REG_CONFIG definition */
#define ADT7320_CONFIG_FAULT_QUEUE(x) (x & 0x3)
#define ADT7320_CONFIG_CT_POL_POS (2)
#define ADT7320_CONFIG_INT_POL_POS (3)
#define ADT7320_CONFIG_INT_CT_MODE_POS (4)
#define ADT7320_CONFIG_OP_MODE(x) ((x & 0x3) << 5)
#define ADT7320_CONFIG_RESOLUTION_POS (7)

/* ADT7320 device ID */
#define ADT7320_DEVICE_ID 0xC3

typedef enum _ADC_RESOLUTION { ADC_RES_13, ADC_RES_16 } adt7320_adc_resolution_t;

typedef enum _FAULT_QUEUE {
    FAULT_QUEUE_1 = 0x0,
    FAULT_QUEUE_2,
    FAULT_QUEUE_3,
    FAULT_QUEUE_4
} ad7320_FaultQueue_t;

typedef enum _PIN_POLARITY { ACTIVE_LOW = 0x0, ACTIVE_HIGH } ad7320_PinPolarity_t;

typedef enum _INT_MODE { INTERRUPT_MODE = 0x0, COMPARATOR_MODE } ad7320_IntCtMode_t;

typedef enum _OPERATION_MODE {
    MODE_CONTINUOUS_CONVERSION = 0x0,
    MODE_ONE_SHOT,
    MODE_1_SPS,
    MODE_SHUTDOWN
} ad7320_OperationMode_t;

/**
   * @brief Initialize ADT7320 Temperature Sensor
   *
   * @details Initialized the ADT7320 Temperature sensor.
   *
   * @param spi SPI instance for communication with sensor
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_init(mxc_spi_regs_t *spi);

/**
   * @brief Sets the fault queue
   *
   * @details This setting sets the number of undertemperature/overtemperature faults that can
   * occur before setting the INT and CT pins. This helps to avoid false triggering due to
   * temperature noise.
   *
   * @param value Fault queue value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetFaultQueue(ad7320_FaultQueue_t value);

/**
   * @brief Sets the CT pin polarity
   *
   * @param value CT pin polarity value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetCTPolarity(ad7320_PinPolarity_t value);

/**
   * @brief Sets the INT pin polarity
   *
   * @param value INT pin polarity value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetIntPolarity(ad7320_PinPolarity_t value);

/**
   * @brief Sets the Interrupt or Comparator mode
   *
   * @param value Mode value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetIntCtMode(ad7320_IntCtMode_t value);

/**
   * @brief Sets the operation mode
   *
   * @param value Operation mode value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetOperationMode(ad7320_OperationMode_t value);

/**
   * @brief Sets the ADC Resolution
   *
   * @param value ADC resolution value
   *    13-bit resolution: Sign bit + 12 bits gives a temperature resolution of 0.0625°C
   *    16-bit resolution. Sign bit + 15 bits gives a temperature resolution of 0.0078°C.
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetADCResolution(adt7320_adc_resolution_t value);

/**
   * @brief Sets the hysteresis value
   *
   * @param value Hysteresis value, from 0°C to 15°C. Stored in straight binary format.
   *    The default setting is 5°C.
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetHysteresisValue(uint8_t value);

/**
   * @brief Sets the Critical Over Temperature limit
   *
   * @param value 16-bit critical overtemperature limit, stored in twos complement format
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetOverTemperatureLimit(float value);

/**
   * @brief Sets the Over Temperature limit
   *
   * @param value 16-bit overtemperature limit, stored in twos complement format
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetCriticalOverTemperatureLimit(float value);

/**
   * @brief Sets the Under Temperature limit
   *
   * @param value 16-bit undertemperature limit, stored in twos complement format
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_SetUnderTemperatureLimit(float value);

/**
   * @brief Reads the Temperature value
   *
   * @param value Pointer to write temperature value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadTemperature(float *temperature);

/**
   * @brief Reads the Status register value
   *
   * @param value Pointer to write status register value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadStatusRegister(uint8_t *reg);

/**
   * @brief Reads the Configuration register value
   *
   * @param value Pointer to write configuration register value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadConfigurationRegister(uint8_t *reg);

/**
   * @brief Reads the Device ID register value
   *
   * @param value Pointer to write Device ID register value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadID(uint8_t *ID);

/**
   * @brief Reads the Critical Over Temperature limit
   *
   * @param value Pointer to write Critical Over Temperature limit
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadCriticalOverTemperatureLimit(float *value);

/**
   * @brief Reads the Over Temperature limit
   *
   * @param value Pointer to write Over Temperature limit
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadOverTemperatureLimit(float *value);

/**
   * @brief Reads the Low Temperature limit
   *
   * @param value Pointer to write Low Temperature limit
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadLowTemperatureLimit(float *value);

/**
   * @brief Reads the Hysteresis value
   *
   * @param value Pointer to write Hysteresis value
   * @return int #E_NO_ERROR if successful, @ref MXC_Error_Codes "error" if unsuccessful.
   */
int adt7320_ReadTHysteresisSetPoint(uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_TEMPSENSOR_ADT7320_DRIVER_H_
