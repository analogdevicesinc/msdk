/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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

#ifndef LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_HID_H_
#define LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_HID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file  hid.h
 * @brief Human Interface Device Class
 */
#define HID_MAX_PACKET    64

/// USB HID class requests
#define HID_GET_REPORT    0x01
#define HID_GET_IDLE      0x02
#define HID_GET_PROTOCOL  0x03
#define HID_SET_REPORT    0x09
#define HID_SET_IDLE      0x0a
#define HID_SET_PROTOCOL  0x0b

/// Class-specific descriptor types for GET_DESCRIPTOR
#define DESC_HID          0x21
#define DESC_REPORT       0x22

/// HID Descriptor
#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bFunctionalLength;
  uint8_t  bDescriptorType;
  uint16_t bcdHID;
  uint8_t  bCountryCode;
  uint8_t  bNumDescriptors;
  uint8_t  bHIDDescriptorType;
  uint16_t wDescriptorLength;
} hid_descriptor_t;

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_HID_H_
