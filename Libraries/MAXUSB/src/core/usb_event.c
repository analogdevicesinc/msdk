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

#include <string.h>
#include "usb.h"
#include "usb_event.h"

/***** File Scope Data *****/
static maxusb_usbio_events_t events;
static MXC_USB_event_callback_t callback[MAXUSB_NUM_EVENTS];

/******************************************************************************/
int MXC_USB_EventEnable(maxusb_event_t event, int (*func)(maxusb_event_t, void *), void *cbdata)
{
  if ( (event >= MAXUSB_NUM_EVENTS) || (func == NULL) ) {
    return -1;
  }

  callback[event].func = func;
  callback[event].cbdata = cbdata;

  return MXC_USB_IrqEnable(event);
}

/******************************************************************************/
int MXC_USB_EventDisable(maxusb_event_t event)
{
  int result = -1;

  if (event >= MAXUSB_NUM_EVENTS) {
    return -1;
  }

  result = MXC_USB_IrqDisable(event);

  callback[event].func = NULL;
  callback[event].cbdata = NULL;

  return result;
}

/******************************************************************************/
int MXC_USB_EventClear(maxusb_event_t event)
{
  if (event >= MAXUSB_NUM_EVENTS) {
    return -1;
  }

  return MXC_USB_IrqClear(event);
}

/******************************************************************************/
void MXC_USB_EventHandler(void)
{
  MXC_USB_IrqHandler(&events);

  if (events.novbus && callback[MAXUSB_EVENT_NOVBUS].func)
  {
    callback[MAXUSB_EVENT_NOVBUS].func(MAXUSB_EVENT_NOVBUS, callback[MAXUSB_EVENT_NOVBUS].cbdata);
  }

  if (events.vbus && callback[MAXUSB_EVENT_VBUS].func)
  {
    callback[MAXUSB_EVENT_VBUS].func(MAXUSB_EVENT_VBUS, callback[MAXUSB_EVENT_VBUS].cbdata);
  }

  if (events.brst && callback[MAXUSB_EVENT_BRST].func)
  {
    callback[MAXUSB_EVENT_BRST].func(MAXUSB_EVENT_BRST, callback[MAXUSB_EVENT_BRST].cbdata);
  }

  if (events.brstdn && callback[MAXUSB_EVENT_BRSTDN].func)
  {
    callback[MAXUSB_EVENT_BRSTDN].func(MAXUSB_EVENT_BRSTDN, callback[MAXUSB_EVENT_BRSTDN].cbdata);
  }

  if (events.dpact && callback[MAXUSB_EVENT_DPACT].func)
  {
    callback[MAXUSB_EVENT_DPACT].func(MAXUSB_EVENT_DPACT, callback[MAXUSB_EVENT_DPACT].cbdata);
  }

  if (events.rwudn && callback[MAXUSB_EVENT_RWUDN].func)
  {
    callback[MAXUSB_EVENT_RWUDN].func(MAXUSB_EVENT_RWUDN, callback[MAXUSB_EVENT_RWUDN].cbdata);
  }

  if (events.bact && callback[MAXUSB_EVENT_BACT].func)
  {
    callback[MAXUSB_EVENT_BACT].func(MAXUSB_EVENT_BACT, callback[MAXUSB_EVENT_BACT].cbdata);
  }

  if (events.susp && callback[MAXUSB_EVENT_SUSP].func)
  {
    callback[MAXUSB_EVENT_SUSP].func(MAXUSB_EVENT_SUSP, callback[MAXUSB_EVENT_SUSP].cbdata);
  }

  if (events.sudav && callback[MAXUSB_EVENT_SUDAV].func)
  {
    callback[MAXUSB_EVENT_SUDAV].func(MAXUSB_EVENT_SUDAV, callback[MAXUSB_EVENT_SUDAV].cbdata);
  }
}
/**  */
