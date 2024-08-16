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
#ifndef EXAMPLES_MAX32672_ADC_DMA_RMS_DEBUG_H_
#define EXAMPLES_MAX32672_ADC_DMA_RMS_DEBUG_H_

#include "mxc_device.h"
#include "gpio.h"

/* Pin definitions for the Debug GPIO Toggling */
#define DBG_IRQ_PORT MXC_GPIO0
#define DBG_IRQ_PIN MXC_GPIO_PIN_6
#define DBG_SQ_PORT MXC_GPIO0
#define DBG_SQ_PIN MXC_GPIO_PIN_7

/* MACROS to help Toggle some test GPIOS */
#define DBG_IRQ_START() (MXC_GPIO_OutSet(DBG_IRQ_PORT, DBG_IRQ_PIN))
#define DBG_IRQ_END() (MXC_GPIO_OutClr(DBG_IRQ_PORT, DBG_IRQ_PIN))
#define DBG_SQ_START() (MXC_GPIO_OutSet(DBG_SQ_PORT, DBG_SQ_PIN))
#define DBG_SQ_END() (MXC_GPIO_OutClr(DBG_SQ_PORT, DBG_SQ_PIN))

/**
 * Initializes the debug functionality. Must be called before doing any GPIO
 * toggles
 */
void DebugInitialize(void);

#endif // EXAMPLES_MAX32672_ADC_DMA_RMS_DEBUG_H_
