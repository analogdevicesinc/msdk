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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_SYSTEM_MAX32680_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_SYSTEM_MAX32680_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

/* NOTE: HIRC was previously named CRYPTO */
#ifdef CRYPTO_FREQ
#warning WARNING: CRYPTO_FREQ does not exist in MAX32680, replace with HIRC_FREQ!
#define HIRC_FREQ CRYPTO_FREQ
#endif

/* NOTE: EXTCLK needs to be defined by user based on the clock they supply */
#ifndef EXTCLK_FREQ
#define EXTCLK_FREQ 75000000
#endif

/* NOTE: This is the nominal value for INRO. The actual value may vary from chip to chip. 
         Update if use of this oscillator requires precise timing.*/
/* NOTE: INRO was previously named NANORING */
#ifndef INRO_FREQ
#define INRO_FREQ 8000
#endif

#ifndef IPO_FREQ
#define IPO_FREQ 100000000
#endif

#ifndef ISO_FREQ
#define ISO_FREQ 60000000
#endif

/* NOTE: ERFO_FREQ (16MHz-32MHz) needs to be defined by user based on the clock they supply */
#ifndef ERFO_FREQ
#define ERFO_FREQ 32000000
#endif

#ifndef IBRO_FREQ
#define IBRO_FREQ 7372800
#endif

#ifndef ERTCO_FREQ
#define ERTCO_FREQ 32768
#endif

#ifndef HIRC_FREQ
#define HIRC_FREQ IPO_FREQ
#endif

extern uint32_t SystemCoreClock; /*!< System Clock Frequency (Core Clock)  */
#ifdef PeripheralClock
#warning PeripheralClock define is being overidden.
#else
#define PeripheralClock (SystemCoreClock / 2) /*!< Peripheral Clock Frequency */
#endif

/*
 * Initialize the system
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void);

/*
 * Update SystemCoreClock variable
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_SYSTEM_MAX32680_H_
