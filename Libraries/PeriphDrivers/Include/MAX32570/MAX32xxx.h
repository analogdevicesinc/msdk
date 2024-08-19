/**
 * @file    MAX32xxx.h
 * @brief   Includes all the required dependancies.
 */

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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MAX32XXX_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MAX32XXX_H_

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#ifdef LIB_BOARD
#include "board.h"
#endif

/*
 *  Peripheral Driver Includes
 */
#include "adc.h"
#include "cameraif.h"
#include "clcd.h"
#include "ctb.h"
#include "dma.h"
#include "emac.h"
#include "flc.h"
#include "gpio.h"
#include "htmr.h"
#include "i2c.h"
#include "icc.h"
#include "lp.h"
#include "owm.h"
#include "pt.h"
#include "rtc.h"
#include "sc.h"
#include "sdhc.h"
#include "sema.h"
#include "sfcc.h"
#include "skbd.h"
#include "smon.h"
#include "spi.h"
#include "spixf.h"
#include "spixr.h"
#include "srcc.h"
#include "tmr.h"
#include "uart.h"
#include "wdt.h"

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MAX32XXX_H_
