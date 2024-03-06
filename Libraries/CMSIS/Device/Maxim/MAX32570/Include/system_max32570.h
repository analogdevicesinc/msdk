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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SYSTEM_MAX32570_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SYSTEM_MAX32570_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

/* NOTE: HIRC was previously named CRYPTO */
#ifdef CRYPTO_FREQ
#warning WARNING: CRYPTO_FREQ does not exist in MAX32570, replace with ISO_FREQ!
#define ISO_FREQ CRYPTO_FREQ
#endif

/* NOTE: ISO was previously HIRC. */
#ifndef ISO_FREQ
#define ISO_FREQ 75000000
#endif

/* NOTE: This is the nominal value for INRO. The actual value may vary from chip to chip. 
         Update if use of this oscillator requires precise timing.*/
/* NOTE: INO was previosly LIRC8  */
#ifndef INRO_FREQ
#define INRO_FREQ 8000
#endif

/* NOTE: IPO was previously HIRC96. */
#ifndef IPO_FREQ
#define IPO_FREQ 150000000
#endif

/* NOTE: IBRO was previously HIRC8M. */
#ifndef IBRO_FREQ
#define IBRO_FREQ 7372800
#endif

/* NOTE: ERFO was previously XTAL27M. */
#ifndef ERFO_FREQ
#define ERFO_FREQ 27120000
#endif

/* NOTE: ERTCO was previously XTAL32K. */
#ifndef ERTCO_FREQ
#define ERTCO_FREQ 32768
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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SYSTEM_MAX32570_H_
