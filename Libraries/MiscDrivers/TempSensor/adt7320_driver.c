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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "spi.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "adt7320_driver.h"

// clang-format off
#define ADT7320_16BIT_NEG 16
#define ADT7320_16BIT_SIGN 0x8000
#define ADT7320_16BIT_DIV 128
#define ADT7320_13BIT_NEG 13
#define ADT7320_13BIT_SIGN 0x1000
#define ADT7320_13BIT_DIV 16
// clang-format on

static mxc_spi_req_t spi_req;

static int adt7320_WriteRegister(uint8_t addr, uint8_t *txBuffer, uint8_t len);
static int adt7320_ReadRegister(uint8_t addr, uint8_t *rxBuffer, uint8_t expectedLen);

int adt7320_init(mxc_spi_regs_t *spi)
{
    uint8_t ID;

    if (!spi)
        return E_NULL_PTR;

    spi_req.spi = spi;

    if (adt7320_ReadID(&ID) != E_NO_ERROR) {
        printf("\n\nCommunication Error!");
        return E_COMM_ERR;
    }
    if (ID != ADT7320_DEVICE_ID) {
        printf("\n\nSensor ID Error Expected: 0xC3, Read: 0x%02x\n", ID);
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

int adt7320_SetFaultQueue(ad7320_FaultQueue_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~ADT7320_CONFIG_FAULT_QUEUE(FAULT_QUEUE_4);
    registerValue |= ADT7320_CONFIG_FAULT_QUEUE(value);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetCTPolarity(ad7320_PinPolarity_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~(0x01 << ADT7320_CONFIG_CT_POL_POS);
    registerValue |= (value << ADT7320_CONFIG_CT_POL_POS);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetIntPolarity(ad7320_PinPolarity_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~(0x01 << ADT7320_CONFIG_INT_POL_POS);
    registerValue |= (value << ADT7320_CONFIG_INT_POL_POS);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetIntCtMode(ad7320_IntCtMode_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~(0x01 << ADT7320_CONFIG_INT_CT_MODE_POS);
    registerValue |= (value << ADT7320_CONFIG_INT_CT_MODE_POS);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetOperationMode(ad7320_OperationMode_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~ADT7320_CONFIG_OP_MODE(MODE_SHUTDOWN);
    registerValue |= ADT7320_CONFIG_OP_MODE(value);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetADCResolution(adt7320_adc_resolution_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadRegister(ADT7320_01_CONFIGURATION, &registerValue, 1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    registerValue &= ~(0x01 << ADT7320_CONFIG_RESOLUTION_POS);
    registerValue |= (value << ADT7320_CONFIG_RESOLUTION_POS);

    return adt7320_WriteRegister(ADT7320_01_CONFIGURATION, &registerValue, 1);
}

int adt7320_SetHysteresisValue(uint8_t value)
{
    uint8_t registerValue = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    registerValue = value & 0x0F;

    return adt7320_WriteRegister(ADT7320_05_THYST_SETPOINT, &registerValue, 1);
}

int adt7320_SetOverTemperatureLimit(float value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    int rawValue = (int)(value / resolution);
    uint16_t registerValue = (uint16_t)rawValue;
    uint8_t reg[2] = { 0 };

    reg[0] = registerValue >> 8;
    reg[1] = registerValue & 0xFF;

    return adt7320_WriteRegister(ADT7320_06_THIGH_SETPOINT, reg, 2);
}

int adt7320_SetCriticalOverTemperatureLimit(float value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    int rawValue = (int)(value / resolution);
    uint16_t registerValue = (uint16_t)rawValue;
    uint8_t reg[2] = { 0 };

    reg[0] = registerValue >> 8;
    reg[1] = registerValue & 0xFF;

    return adt7320_WriteRegister(ADT7320_04_TCRIT_SETPOINT, reg, 2);
}

int adt7320_SetUnderTemperatureLimit(float value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    int rawValue = (int)(value / resolution);
    uint16_t registerValue = (uint16_t)rawValue;
    uint8_t reg[2] = { 0 };

    reg[0] = registerValue >> 8;
    reg[1] = registerValue & 0xFF;

    return adt7320_WriteRegister(ADT7320_07_TLOW_SETPOINT, reg, 2);
}

int adt7320_ReadStatusRegister(uint8_t *reg)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    return adt7320_ReadRegister(ADT7320_00_STATUS, reg, 1);
}

int adt7320_ReadTemperature(float *temperature)
{
    uint8_t buf[2] = { 0 };
    uint16_t temp = 0;
    uint8_t config = 0;
    float temp_c = 0;

    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    if (adt7320_ReadConfigurationRegister(&config) != E_NO_ERROR)
        return E_COMM_ERR;

    if (adt7320_ReadRegister(0x02, buf, 2) != E_NO_ERROR)
        return E_COMM_ERR;

    temp |= (buf[0] << 8) | buf[1];

    if ((config >> ADT7320_CONFIG_RESOLUTION_POS) == 1) {
        if (temp & ADT7320_16BIT_SIGN)
            /*! Negative temperature */
            temp_c = (float)((int32_t)temp - ADT7320_16BIT_NEG) / ADT7320_16BIT_DIV;
        else
            /*! Positive temperature */
            temp_c = (float)temp / ADT7320_16BIT_DIV;
    } else {
        temp >>= 3;
        if (temp & ADT7320_13BIT_SIGN)
            /*! Negative temperature */
            temp_c = (float)((int32_t)temp - ADT7320_13BIT_NEG) / ADT7320_13BIT_DIV;
        else
            /*! Positive temperature */
            temp_c = (float)temp / ADT7320_13BIT_DIV;
    }

    *temperature = temp_c;

    return E_NO_ERROR;
}

int adt7320_ReadConfigurationRegister(uint8_t *reg)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    return adt7320_ReadRegister(ADT7320_01_CONFIGURATION, reg, 1);
}

int adt7320_ReadID(uint8_t *ID)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    return adt7320_ReadRegister(ADT7320_03_ID, ID, 1);
}

int adt7320_ReadCriticalOverTemperatureLimit(float *value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    uint16_t regValue = 0;
    uint8_t reg[2] = { 0 };

    if (adt7320_ReadRegister(ADT7320_04_TCRIT_SETPOINT, reg, 2) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    regValue |= (reg[0] << 8) | reg[1];

    *value = ((float)regValue * resolution);
    return E_NO_ERROR;
}

int adt7320_ReadOverTemperatureLimit(float *value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    uint16_t regValue = 0;
    uint8_t reg[2] = { 0 };

    if (adt7320_ReadRegister(ADT7320_06_THIGH_SETPOINT, reg, 2) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    regValue |= (reg[0] << 8) | reg[1];
    *value = ((float)regValue * resolution);
    return E_NO_ERROR;
}

int adt7320_ReadLowTemperatureLimit(float *value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    float resolution = 0.0078;
    uint16_t regValue = 0;
    uint8_t reg[2] = { 0 };

    if (adt7320_ReadRegister(ADT7320_07_TLOW_SETPOINT, reg, 2) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    regValue |= (reg[0] << 8) | reg[1];
    *value = ((float)regValue * resolution);
    return E_NO_ERROR;
}

int adt7320_ReadTHysteresisSetPoint(uint8_t *value)
{
    if (spi_req.spi == NULL)
        return E_NULL_PTR;

    return adt7320_ReadRegister(ADT7320_05_THYST_SETPOINT, value, 1);
}

static int adt7320_ReadRegister(uint8_t addr, uint8_t *rxBuffer, uint8_t expectedLen)
{
    int ret = E_NO_ERROR;
    uint8_t cmd = 0x00;
    uint8_t sendData[3] = { 0 };
    uint8_t recData[3] = { 0 };

    memset(sendData, 0xFF, sizeof(sendData));

    cmd |= 0x01 << 6;
    cmd |= addr << 3;

    sendData[0] = cmd;

    spi_req.txData = sendData;
    spi_req.rxData = recData;
    spi_req.txLen = expectedLen + 1;
    spi_req.rxLen = expectedLen + 1;
    spi_req.ssIdx = 0;
    spi_req.ssDeassert = 1;
    spi_req.txCnt = 0;
    spi_req.rxCnt = 0;

    MXC_SPI_SetDataSize(spi_req.spi, 8);
    ret = MXC_SPI_MasterTransaction(&spi_req);
    if (ret == E_NO_ERROR) {
        memcpy(rxBuffer, &recData[1], expectedLen);
    }
    return ret;
}

static int adt7320_WriteRegister(uint8_t addr, uint8_t *txBuffer, uint8_t len)
{
    uint8_t sendData[5] = { 0 };

    sendData[0] = addr << 3;
    memcpy(&sendData[1], txBuffer, len);

    spi_req.txData = sendData;
    spi_req.rxData = NULL;
    spi_req.txLen = len + 1;
    spi_req.rxLen = 0;
    spi_req.ssIdx = 0;
    spi_req.ssDeassert = 1;
    spi_req.txCnt = 0;
    spi_req.rxCnt = 0;

    MXC_SPI_SetDataSize(spi_req.spi, 8);
    return MXC_SPI_MasterTransaction(&spi_req);
}
