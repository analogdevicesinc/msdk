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
#ifndef EXAMPLES_MAX32650_CLCD_FREERTOS_USAGETRACE_H_
#define EXAMPLES_MAX32650_CLCD_FREERTOS_USAGETRACE_H_
#include "gpio.h"
#include "mxc_device.h"
#include "tmr.h"

/****************************************
 * This code intends to provide a basic wrapper of functionality for tracking
 * amount of time spent in a block of code. It provides several methods for
 * doing so, enabled by the various #defines:
 *
 *  - USAGE_TMR - Measure time in a code block using a timer peripheral
 *  - USAGE_GPIO_OUT - Set a GPIO high while the block is executing
 *  - USAGE_NO_IRQ - Disable interrupts while this block is running
*/

/* If USAGE_TMR, configure the peripheral timer */
#ifdef USAGE_TMR
#define USAGE_TMR_INST MXC_TMR3
#endif

/* If USAGE_GPIO_OUT, configure the GPIO signal */
#ifdef USAGE_GPIO_OUT
#define USAGE_GPIO_PORT MXC_GPIO2
#define USAGE_GPIO_PIN MXC_GPIO_PIN_0
#endif

/* Enum used for notifying the source during the hook*/
typedef enum { USAGE_SRC_TMR } usage_src_t;

/**
 * Initializes the UsageTrace functionality. Needs to be called before Start and
 * End are used
*/
void UsageTraceInit(void);

/**
 * Prototype type for the UserHook. For timer based tracing, it is up to the
 * user to implement this and determine what to do with the timing data
 *
 * @param count - Timer count in the units defined by the source
 * @param src - Source of the timer data. In case multiple sources are enabled
*/
void UsageTraceUserHook(uint32_t count, usage_src_t src);

/**
 * Inline function to be placed at the start of the code to be measured.
 * Disables interrupts (if defined) and starts all defined timing mechanisms
 */
static inline void _UsageTraceStart(void)
{
#ifdef USAGE_NO_IRQ
    __disable_irq();
#endif

#ifdef USAGE_TMR
    USAGE_TMR_INST->cnt = 0;
#endif

#ifdef USAGE_GPIO_OUT
    USAGE_GPIO_PORT->out_set = USAGE_GPIO_PIN;
#endif
}

/**
 * Inline function to be placed at the end of the code to be measured.
 * Stops all defined timing mechanisms, calls the user hook (if necessary) and
 * re-enables interrupts if they were disabled.
 */
static inline void _UsageTraceEnd(void)
{
#ifdef USAGE_GPIO_OUT
    USAGE_GPIO_PORT->out_clr = USAGE_GPIO_PIN;
#endif

#ifdef USAGE_TMR
    uint32_t countTmr;
    countTmr = USAGE_TMR_INST->cnt;
#endif

#ifdef USAGE_NO_IRQ
    __enable_irq();
#endif

//Defer the user hook until after interrupts are enabled and all counts are
//gathered to not skew the timing, and let the user do what they want in the
//hook
#ifdef USAGE_TMR
    UsageTraceUserHook(countTmr, USAGE_SRC_TMR);
#endif
}

#endif //EXAMPLES_MAX32650_CLCD_FREERTOS_USAGETRACE_H_
