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

#ifndef LIBRARIES_MAXUSB_INCLUDE_CORE_USB_PROTOCOL_H_
#define LIBRARIES_MAXUSB_INCLUDE_CORE_USB_PROTOCOL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SETUP message byte offsets */
#define SETUP_bmRequestType   0
#define SETUP_bRequest        1
#define SETUP_wValueL         2
#define SETUP_wValueH         3
#define SETUP_wIndexL         4
#define SETUP_wIndexH         5
#define SETUP_wLengthL        6
#define SETUP_wLengthH        7

typedef struct {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} MXC_USB_SetupPkt;

/* Bitmasks for the bit-field bmRequestType */
#define RT_DEV_TO_HOST            0x80

#define RT_TYPE_MASK              0x60
#define RT_TYPE_STD               0x00
#define RT_TYPE_CLASS             0x20
#define RT_TYPE_VENDOR            0x40

#define RT_RECIP_MASK             0x1f
#define RT_RECIP_DEVICE           0x00
#define RT_RECIP_IFACE            0x01
#define RT_RECIP_ENDP             0x02
#define RT_RECIP_OTHER            0x03

/* Standard Device Requests for bRequest */
#define SDR_GET_STATUS            0x00
#define SDR_CLEAR_FEATURE         0x01
#define SDR_SET_FEATURE           0x03
#define SDR_SET_ADDRESS           0x05
#define SDR_GET_DESCRIPTOR        0x06
#define SDR_SET_DESCRIPTOR        0x07
#define SDR_GET_CONFIG            0x08
#define SDR_SET_CONFIG            0x09
#define SDR_GET_INTERFACE         0x0a
#define SDR_SET_INTERFACE         0x0b
#define SDR_SYNCH_FRAME           0x0c

/* Descriptor types for *_DESCRIPTOR */
#define DESC_DEVICE               1
#define DESC_CONFIG               2
#define DESC_STRING               3
#define DESC_INTERFACE            4
#define DESC_ENDPOINT             5
#define DESC_DEVICE_QUAL          6
#define DESC_OTHER_SPEED          7
#define DESC_IFACE_PWR            8

/* Feature types for *_FEATURE */
#define FEAT_ENDPOINT_HALT        0
#define FEAT_REMOTE_WAKE          1
#define FEAT_TEST_MODE            2

/* Get Status bit positions */
#define STATUS_EP_HALT            0x1
#define STATUS_DEV_SELF_POWERED   0x1
#define STATUS_DEV_REMOTE_WAKE    0x2

/* bmAttributes bit positions */
#define BMATT_REMOTE_WAKE         0x20
#define BMATT_SELF_POWERED        0x40

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
} MXC_USB_device_descriptor_t;

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t wTotalLength;
  uint8_t  bNumInterfaces;
  uint8_t  bConfigurationValue;
  uint8_t  iConfiguration;
  uint8_t  bmAttributes;
  uint8_t  bMaxPower;
} MXC_USB_configuration_descriptor_t;

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bInterfaceNumber;
  uint8_t  bAlternateSetting;
  uint8_t  bNumEndpoints;
  uint8_t  bInterfaceClass;
  uint8_t  bInterfaceSubClass;
  uint8_t  bInterfaceProtocol;
  uint8_t  iInterface;
} MXC_USB_interface_descriptor_t;

#define USB_EP_NUM_MASK   0x0F

#ifndef USE_ZEPHYR_USB_STACK
#define USB_EP_DIR_MASK   0x80
#endif

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t  bInterval;
} MXC_USB_endpoint_descriptor_t;

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize;
  uint8_t  bNumConfigurations;
  uint8_t  bReserved;
} MXC_USB_device_qualifier_descriptor_t;

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t wTotalLength;
  uint8_t  bNumInterfaces;
  uint8_t  bConfigurationValue;
  uint8_t  iConfiguration;
  uint8_t  bmAttributes;
  uint8_t  bMaxPower;
} MXC_USB_other_speed_configuration_descriptor_t;

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_MAXUSB_INCLUDE_CORE_USB_PROTOCOL_H_
