/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Example IMU service implementation.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "svc_imu.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef IMU_SEC_PERMIT_READ
#define IMU_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef IMU_SEC_PERMIT_WRITE
#define IMU_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/* Custom UUIDs for IMU Service and Characteristics (LSB) */
// IMU Service: 12345678-1234-1234-1234-1234567890AB
static const uint8_t imuServiceUuid[] = {0xAB, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
// Gyroscope Characteristic: 12345678-1234-1234-1234-1234567890AC
#define GYRO_CHAR_UUID 0xAC, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
static const uint8_t gyroCharUuid[] = {GYRO_CHAR_UUID};

// IMU Service Declaration
// static const uint8_t imuService[] = {UINT16_TO_BYTES(0x180D)}; // Example UUID
// static const uint16_t imuServiceLen = size0x00, 0x00of(imuService);
static const uint16_t imuServiceLen = sizeof(imuServiceUuid);

/* Gyroscope */ 
// Gyroscope Characteristic Declaration
static const uint8_t gyroChar[] = {ATT_PROP_NOTIFY, 
                                    UINT16_TO_BYTES(IMU_GYRO_VAL_HDL),
                                    GYRO_CHAR_UUID};
static const uint16_t gyroCharLen = sizeof(gyroChar);
// Gyroscope Characteristic Value
#if DATA_LEN_8
static uint8_t gyroVal[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // TODO: Make this 8 + 1 bytes
#else
static uint8_t gyroVal[] = {0x00, 0x00}; // TODO: Make this 8 or 8 + 1 bytes
#endif // DATA_LEN_8
static const uint16_t gyroValLen = sizeof(gyroVal);
// Gyroscope client characteristic configuration
static uint8_t gyroCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t gyroCccLen = sizeof(gyroCcc);
static const uint8_t gyroDesc[] = "IMU Sensor";  // or any custom name
static const uint16_t gyroDescLen = sizeof(gyroDesc);

// Attribute List for IMU Service
static const attsAttr_t imuAttrList[] = {
    // IMU Service Declaration
    {
        attPrimSvcUuid,
        (uint8_t *)imuServiceUuid,
        (uint16_t *)&imuServiceLen,
        sizeof(imuServiceUuid),
        0, // No special settings
        ATTS_PERMIT_READ
    },
    // Gyroscope Characteristic Declaration
    {
        attChUuid,
        (uint8_t *)gyroChar,
        (uint16_t *)&gyroCharLen,
        sizeof(gyroChar),
        0, // No special settings
        ATTS_PERMIT_READ
    },
    // Gyroscope Characteristic Value
    {
        gyroCharUuid,           // Full 128-bit UUID
        (uint8_t *)gyroVal,     // Value will be updated dynamically
        (uint16_t *)&gyroValLen,
        sizeof(gyroVal),
        ATTS_SET_UUID_128,      // 128-bit UUID
        ATTS_PERMIT_READ
    },
    // // Gyroscope Characteristic User Description
    // {
    //     attChUserDescUuid,
    //     (uint8_t *)gyroDesc,
    //     (uint16_t *)&gyroDescLen,
    //     sizeof(gyroDesc),
    //     0, // No special settings
    //     ATTS_PERMIT_READ
    // },
    // Gyroscope CCCD
    {
        attCliChCfgUuid,
        (uint8_t *)gyroCcc,
        (uint16_t *)&gyroCccLen,
        sizeof(gyroCcc),
        ATTS_SET_CCC,
        (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
    }
};

// IMU Service Group
static attsGroup_t imuSvcGroup = {
    NULL,
    (attsAttr_t *)imuAttrList,
    NULL,
    NULL,
    IMU_START_HDL, // Start handle
    IMU_END_HDL // End handle
};

// Add IMU Service to GATT Server
void SvcImuAddGroup(void)
{
    AttsAddGroup(&imuSvcGroup);
}

// Remove IMU Service from GATT Server
void SvcImuRemoveGroup(void)
{
    AttsRemoveGroup(IMU_START_HDL);
}

// Register Callbacks for IMU Service
void SvcImuCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
    imuSvcGroup.readCback = readCback;
    imuSvcGroup.writeCback = writeCback;
}
