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
 */
/*************************************************************************************************/

#include "pal_led.h"
#include "gpio.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef PAL_BB_LED_ENABLED
#define PAL_BB_LED_ENABLED    1
#endif

#define PAL_BB_LED_TX         0x81
#define PAL_BB_LED_RX         0x82
#define PAL_BB_LED_RX_OK      0x84
#define PAL_BB_LED_RX_TO      0x88
#define PAL_BB_LED_RX_CRC     0x90

/* BSP LED Driver */
extern const unsigned int num_leds;
extern int LED_Init(void);
extern void LED_On(unsigned int idx);
extern void LED_Off(unsigned int idx);
extern const mxc_gpio_cfg_t led_pin[];

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
  palLedOff(0);
  palLedOff(1);
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
      break;
  }

#if (PAL_BB_LED_ENABLED == 1)
    if(ledId & 0x80){
      /* Remap the mask for the BB LEDs */
      int i;
      for(i = 0; i < 7; i++) {
        if(ledId & (0x1 << i)) {
          palLedOn(2+i);
        }
      }
    }
#endif
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
      break;
  }

#if (PAL_BB_LED_ENABLED == 1)
    if(ledId & 0x80){
      /* Remap the mask for the BB LEDs */
      int i;
      for(i = 0; i < 7; i++) {
        if(ledId & (0x1 << i)) {
          palLedOff(2+i);
        }
      }
    }
#endif
}
/*************************************************************************************************/
/*!
 *  \brief      Set LED On Fast as possible, by eliminating overhead.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedFastOn(uint8_t id)
{
    #if LED_ON == 0
        led_pin[id].port->out_clr = led_pin[id].mask;
    #else
        led_pin[id].port->out_set = led_pin[id].mask;
    #endif
}
/*************************************************************************************************/
/*!
 *  \brief      Set LED Off Fast as possible, by eliminating overhead.
 *
 *  \param      ledId           LED ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalLedFastOff(uint8_t id)
{
  #if LED_ON == 0
        led_pin[id].port->out_set = led_pin[id].mask;
    #else
        led_pin[id].port->out_clr = led_pin[id].mask;
    #endif
}