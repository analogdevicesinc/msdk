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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SYSTEM_MAX32657_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SYSTEM_MAX32657_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

/* NOTE: EXTCLK needs to be defined by user based on the clock they supply */
#ifndef EXTCLK_FREQ
#define EXTCLK_FREQ 75000000
#endif

/* NOTE: This is the nominal value for INRO. The actual value may vary from chip to chip. 
         Update if use of this oscillator requires precise timing.*/
/* NOTE: INRO was previously named NANORING */
#ifndef INRO_FREQ
#define INRO_FREQ 131000
#endif

#ifndef IPO_FREQ
#define IPO_FREQ 50000000
#endif

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

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/**
 * @brief This function is called in Secure code just before control
 *  is transferred to non-secure world. Only available when
 *  trustzone feature is used.
 *
 * You may over-ride this function in your program by defining a custom
 *  NonSecure_Init(), but care should be taken to reproduce the initialization
 *  steps to non-secure code.
 * 
 * Caller must be aware of configuring MPC, SPC, and NSPC before this
 *  function is called.
 * 
 * @return  Should not return if successful. If Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int NonSecure_Init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SYSTEM_MAX32657_H_
