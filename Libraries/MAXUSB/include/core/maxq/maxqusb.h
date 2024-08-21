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

#ifndef LIBRARIES_MAXUSB_INCLUDE_CORE_MAXQ_MAXQUSB_H_
#define LIBRARIES_MAXUSB_INCLUDE_CORE_MAXQ_MAXQUSB_H_
#include <stdint.h>

#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
#define MXC_USB_NUM_EP          (6)
#else
#define MXC_USB_NUM_EP          (4)
#endif

#define MXC_USB_INSTANCES       (1)
#define MXC_USB_MAX_PACKET      (64)
#define MXC_USB_MAXQ_EP_SIZE    (64)

#endif //LIBRARIES_MAXUSB_INCLUDE_CORE_MAXQ_MAXQUSB_H_

