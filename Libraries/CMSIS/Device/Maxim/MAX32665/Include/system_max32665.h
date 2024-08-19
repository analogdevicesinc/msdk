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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_MAX32665_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_MAX32665_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

/* NOTE: HIRC was previously named CRYPTO */
#ifdef CRYPTO_FREQ
#warning WARNING: CRYPTO_FREQ does not exist in MAX32665, replace with HIRC_FREQ!
#define HIRC_FREQ CRYPTO_FREQ
#endif

/* NOTE: HIRC was previously named CRYPTO */
#ifndef HIRC_FREQ
#define HIRC_FREQ 60000000
#endif

/* NOTE: This is the nominal value for LIRC8. The actual value may vary from chip to chip. 
         Update if use of this oscillator requires precise timing.*/
/* NOTE: LIRC8 was previously named NANORING */
#ifndef LIRC8_FREQ
#define LIRC8_FREQ 8000
#endif

#ifndef HIRC96_FREQ
#define HIRC96_FREQ 96000000
#endif

#ifndef HIRC8_FREQ
#define HIRC8_FREQ 7372800
#endif

/* NOTE: ERFO_FREQ (32MHz) needs to be defined by user based on the clock they supply */
#ifndef ERFO_FREQ
#define ERFO_FREQ 32000000
#endif

#ifndef XTAL32M_FREQ
#define XTAL32M_FREQ 32000000
#endif

#ifndef XTAL32K_FREQ
#define XTAL32K_FREQ 32768
#endif

extern uint32_t SystemCoreClock; /*!< System Clock Frequency (Core Clock)  */
#ifndef PeripheralClock
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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_MAX32665_H_
