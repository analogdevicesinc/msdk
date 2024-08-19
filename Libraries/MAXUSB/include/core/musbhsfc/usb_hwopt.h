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

#ifndef LIBRARIES_MAXUSB_INCLUDE_CORE_MUSBHSFC_USB_HWOPT_H_
#define LIBRARIES_MAXUSB_INCLUDE_CORE_MUSBHSFC_USB_HWOPT_H_

#include "mxc_device.h"
#include "usbhs_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration options for MUSBHSFC */
typedef struct {
  unsigned int enable_hs; /* 0 = full-speed only, otherwise high-speed if negotiated */
  void (*delay_us)(unsigned int usec); /* User-supplied function to delay usec micro-seconds */
  int (*init_callback)(void); /* User-supplied function for initializing the USB block */
  int (*shutdown_callback)(void); /* User-supplied function for shutting down the USB block */
} maxusb_cfg_options_t;

/** 
 * @brief Put the transceiver into a low power state.
 */
void MXC_USB_Sleep(void);

/** 
 * @brief Power up the USB transceiver, must be called once the device wakes from sleep.
 */
void MXC_USB_Wakeup(void);

/** 
 * @brief Send a remote wakeup signal to the host.
 */
void MXC_USB_RemoteWakeup(void);

/*
 * @brief USB internal DMA engine interrupt handler
 */
void MXC_USB_DmaIsr(void);

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_MAXUSB_INCLUDE_CORE_MUSBHSFC_USB_HWOPT_H_
