/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_H_

/**
 *  System Includes
 */
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
#include "led.h"
#include "pb.h"
#endif

/**
 *  Peripheral Driver Includes
 */
#include "aes.h"
#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "i3c.h"
#include "lp.h"
#include "rtc.h"
#include "nspc.h"
#include "spi.h"
#include "tmr.h"
#include "trng.h"
#include "uart.h"
#include "wdt.h"
#include "wut.h"

/**
 *  Peripheral Drivers only accessible in Secure world
 */
#if (CONFIG_TRUSTED_EXECUTION_SECURE != 0)
#include "flc.h"
#include "icc.h"
#include "mpc.h"
#include "spc.h"
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_H_
