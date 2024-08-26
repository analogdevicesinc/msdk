/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      LED driver implementation.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 *
 * Copyright (c) 2022-2023 Analog Devices, Inc.
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
 */
/*************************************************************************************************/

#include "pal_led.h"
#include "gpio.h"
#include "led.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief      Control block. */
static struct {
  bool_t init;
} palLedCb;

/**************************************************************************************************
  Functions: Initialization
**************************************************************************************************/

/**************************************************************************************************
  Functions: Status and Control
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Set LED on, make sure we have enough LEDs.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
static void palLedOn(uint8_t led)
{
  /* Make sure we have enough LEDs */
  if(num_leds > led) {
    LED_On(led);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Set LED off, make sure we have enough LEDs.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
static void palLedOff(uint8_t led)
{
  /* Make sure we have enough LEDs */
  if(num_leds > led) {
    LED_Off(led);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize LEDs.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedInit(void)
{
  LED_Init();
  palLedCb.init = TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief      De-initialize LEDs.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedDeInit(void)
{
  PalLedOff(PAL_LED_ID_CPU_ACTIVE);
  PalLedOff(PAL_LED_ID_ERROR);
  palLedCb.init = FALSE;
}


/*************************************************************************************************/
/*!
 *  \brief      Set LED on.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedOn(uint8_t ledId)
{
  if(!palLedCb.init) {
    return;
  }

  switch (ledId) {
    case PAL_LED_ID_CPU_ACTIVE:
#ifndef __riscv
      palLedOn(1);
#else
      palLedOn(0);
#endif
      return;
    case PAL_LED_ID_ERROR:
      palLedOn(0);
      return;

    default:
      palLedOn(ledId);
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Set LED off.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedOff(uint8_t ledId)
{
  if(!palLedCb.init) {
    return;
  }

  switch (ledId) {
    case PAL_LED_ID_CPU_ACTIVE:
#ifndef __riscv
      palLedOff(1);
#else
      palLedOff(0);
#endif
      return;
    case PAL_LED_ID_ERROR:
      palLedOff(0);
      return;

    default:
      palLedOff(ledId);
      break;
  }
}
