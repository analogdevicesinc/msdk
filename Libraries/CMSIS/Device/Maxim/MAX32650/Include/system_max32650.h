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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SYSTEM_MAX32650_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SYSTEM_MAX32650_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#ifndef CRYPTO_FREQ
#define CRYPTO_FREQ_A1 40000000
#define CRYPTO_FREQ_A3 50000000
#endif

#ifndef HFX_FREQ
#define HFX_FREQ 24000000
#endif

/* NOTE: This is the nominal value for NANORING. The actual value may vary from chip to chip. 
         Update if use of this oscillator requires precise timing.*/
#ifndef NANORING_FREQ
#define NANORING_FREQ 8000
#endif

#ifndef HIRC96_FREQ
#define HIRC96_FREQ 120000000
#endif

#ifndef HIRC8_FREQ
#define HIRC8_FREQ 8000000
#endif

#ifndef IBRO_FREQ
#define IBRO_FREQ 7372800
#endif

#ifndef ERTCO_FREQ
#define ERTCO_FREQ 32768
#endif

extern uint32_t SystemCoreClock; /*!< System Clock Frequency (Core Clock)  */
extern uint8_t ChipRevision; /*!< System Clock Frequency (Core Clock)  */
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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SYSTEM_MAX32650_H_
