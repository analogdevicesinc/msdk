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

#ifndef LIBRARIES_MAXUSB_INCLUDE_CORE_ARM_USB_HWOPT_H_
#define LIBRARIES_MAXUSB_INCLUDE_CORE_ARM_USB_HWOPT_H_

#include "mxc_device.h"
#include "usb_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* There are no configuration options for this target */
typedef void maxusb_cfg_options_t;

#define MAXUSB_ENTER_CRITICAL() __disable_irq()
#define MAXUSB_EXIT_CRITICAL() __enable_irq()

/** 
 * @brief Put the transceiver into a low power state.
 */
static inline void MXC_USB_Sleep(void)
{
    MXC_USB->dev_cn |= MXC_F_USB_DEV_CN_ULPM;
}

/** 
 * @brief Power up the USB transceiver, must be called once the device wakes from sleep.
 */
static inline void MXC_USB_Wakeup(void)
{
    MXC_USB->dev_cn &= ~MXC_F_USB_DEV_CN_ULPM;
}

/** 
 * @brief Send a remote wakeup signal to the host.
 */
static inline void MXC_USB_RemoteWakeup(void)
{
    MXC_USB->dev_cn |= MXC_F_USB_DEV_CN_SIGRWU;
}

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MAXUSB_INCLUDE_CORE_ARM_USB_HWOPT_H_
