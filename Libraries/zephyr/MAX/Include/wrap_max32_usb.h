/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_USB_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_USB_H_

/***** Includes *****/
#include <usb.h>
#include <usb_event.h>
#include <mcr_regs.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)

static inline int Wrap_MXC_USB_Init(maxusb_cfg_options_t *options)
{
    return MXC_USB_Init(options);
}

/*
 *  MAX32690 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690)

static inline int Wrap_MXC_USB_Init(maxusb_cfg_options_t *options)
{
    MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_0P9EN;

    return MXC_USB_Init(options);
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_USB_H_
