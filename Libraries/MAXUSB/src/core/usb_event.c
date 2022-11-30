/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
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
    if ((event >= MAXUSB_NUM_EVENTS) || (func == NULL)) {
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

    if (events.novbus && callback[MAXUSB_EVENT_NOVBUS].func) {
        callback[MAXUSB_EVENT_NOVBUS].func(MAXUSB_EVENT_NOVBUS,
                                           callback[MAXUSB_EVENT_NOVBUS].cbdata);
    }

    if (events.vbus && callback[MAXUSB_EVENT_VBUS].func) {
        callback[MAXUSB_EVENT_VBUS].func(MAXUSB_EVENT_VBUS, callback[MAXUSB_EVENT_VBUS].cbdata);
    }

    if (events.brst && callback[MAXUSB_EVENT_BRST].func) {
        callback[MAXUSB_EVENT_BRST].func(MAXUSB_EVENT_BRST, callback[MAXUSB_EVENT_BRST].cbdata);
    }

    if (events.brstdn && callback[MAXUSB_EVENT_BRSTDN].func) {
        callback[MAXUSB_EVENT_BRSTDN].func(MAXUSB_EVENT_BRSTDN,
                                           callback[MAXUSB_EVENT_BRSTDN].cbdata);
    }

    if (events.dpact && callback[MAXUSB_EVENT_DPACT].func) {
        callback[MAXUSB_EVENT_DPACT].func(MAXUSB_EVENT_DPACT, callback[MAXUSB_EVENT_DPACT].cbdata);
    }

    if (events.rwudn && callback[MAXUSB_EVENT_RWUDN].func) {
        callback[MAXUSB_EVENT_RWUDN].func(MAXUSB_EVENT_RWUDN, callback[MAXUSB_EVENT_RWUDN].cbdata);
    }

    if (events.bact && callback[MAXUSB_EVENT_BACT].func) {
        callback[MAXUSB_EVENT_BACT].func(MAXUSB_EVENT_BACT, callback[MAXUSB_EVENT_BACT].cbdata);
    }

    if (events.susp && callback[MAXUSB_EVENT_SUSP].func) {
        callback[MAXUSB_EVENT_SUSP].func(MAXUSB_EVENT_SUSP, callback[MAXUSB_EVENT_SUSP].cbdata);
    }

    if (events.sudav && callback[MAXUSB_EVENT_SUDAV].func) {
        callback[MAXUSB_EVENT_SUDAV].func(MAXUSB_EVENT_SUDAV, callback[MAXUSB_EVENT_SUDAV].cbdata);
    }
}
/**  */
