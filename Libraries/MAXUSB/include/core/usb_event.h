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

/*
 * Low-layer API calls
 *
 * These do not change, and provide the basis by which usb.c acceses the 
 *  hardware. All usbio.c drivers will provide these calls, or return an
 *  error if the function is not supported.
 * 
 */

#ifndef LIBRARIES_MAXUSB_INCLUDE_CORE_USB_EVENT_H_
#define LIBRARIES_MAXUSB_INCLUDE_CORE_USB_EVENT_H_

#include "usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file MXC_USB_event.h
 * @brief Defines the API used for USB event handling
 *
 */

/***** Definitions *****/
typedef struct {
  int (*func)(maxusb_event_t, void *);
  void *cbdata;
} MXC_USB_event_callback_t;

/** 
 * @brief Register a callback for and enable the specified event
 * @param event   event number
 * @param func    function to be called
 * @param cbdata  parameter to call callback function with
 * @return This function returns zero (0) for success, non-zero for failure
 */
int MXC_USB_EventEnable(maxusb_event_t event, int (*callback)(maxusb_event_t, void *), void *cbdata);

/** 
 * @brief Enable the specified event
 * @param event   event number
 * @return This function returns zero (0) for success, non-zero for failure
 */
int MXC_USB_EventDisable(maxusb_event_t event);

/** 
 * @brief Clear the specified event
 * @param event   event number
 * @return This function returns zero (0) for success, non-zero for failure
 */
int MXC_USB_EventClear(maxusb_event_t event);

/** 
 * @brief Processes USB events
 * This function should be called from the USB interrupt vector or periodically
 * from the application.
 */
void MXC_USB_EventHandler(void);

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_MAXUSB_INCLUDE_CORE_USB_EVENT_H_
