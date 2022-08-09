/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

/**
 * @file main.c
 * @brief Demonstrates how to configure a the USB device controller as a HID keyboard class device
 * @details The EvKit should enumerate as a HID Keyboard device after loading the project and
 *          attaching a cable from the PC to the USB connector on the Evaluation Kit.
 *          1. LED0 (P2.17) will illuminate once enumeration and configuration is complete.
 *          2. Open a text editor on the PC host and place cursor in edit box.
 *          3. Pressing pushbutton SW1 (P0.16) will cause a message to be typed in on a virtual keyboard one character at a time.
 *
 */

#include <stdio.h>
#include <stddef.h>
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "led.h"
#include "pb.h"
#include "mxc_delay.h"
#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "hid_kbd.h"
#include "descriptors.h"

/***** Definitions *****/
#define EVENT_ENUM_COMP     MAXUSB_NUM_EVENTS
#define EVENT_REMOTE_WAKE   (EVENT_ENUM_COMP + 1)

#define STRINGIFY(x)        #x
#define TOSTRING(x)         STRINGIFY(x)

/***** Global Data *****/
int remoteWake;
volatile int configured;
volatile int suspended;
volatile unsigned int evtFlags;

/***** Function Prototypes *****/
static int setconfigCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int setfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int clrfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int eventCallback(maxusb_event_t evt, void* data);
static void usbAppSleep(void);
static void usbAppWakeup(void);
static void buttonCallback(void* pb);
void usDelay(unsigned int usec);
int usbStartupCallback();
int usbShutdownCallback();


/******************************************************************************/
void USB_IRQHandler(void)
{
    MXC_USB_EventHandler();
}

/******************************************************************************/
int main(void)
{
    maxusb_cfg_options_t usb_opts;
    
    printf("\n\n***** " TOSTRING(TARGET) " USB HID Keyboard Example *****\n");
    printf("Waiting for VBUS...\n");
    
    /* Initialize state */
    configured = 0;
    suspended = 0;
    evtFlags = 0;
    remoteWake = 0;
    
    /* Start out in full speed */
    usb_opts.enable_hs = 0;
    usb_opts.delay_us = usDelay; /* Function which will be used for delays */
    usb_opts.init_callback = usbStartupCallback;
    usb_opts.shutdown_callback = usbShutdownCallback;
    
    /* Initialize the usb module */
    if (MXC_USB_Init(&usb_opts) != 0) {
        printf("MXC_USB_Init() failed\n");
        
        while (1);
    }
    
    /* Initialize the enumeration module */
    if (enum_init() != 0) {
        printf("enum_init() failed\n");
        
        while (1);
    }
    
    /* Register enumeration data */
    enum_register_descriptor(ENUM_DESC_DEVICE, (uint8_t*) &device_descriptor, 0);
    enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t*) &config_descriptor, 0);
    enum_register_descriptor(ENUM_DESC_STRING, lang_id_desc, 0);
    enum_register_descriptor(ENUM_DESC_STRING, mfg_id_desc, 1);
    enum_register_descriptor(ENUM_DESC_STRING, prod_id_desc, 2);
    enum_register_descriptor(ENUM_DESC_STRING, serial_id_desc, 3);
    
    /* Handle configuration */
    enum_register_callback(ENUM_SETCONFIG, setconfigCallback, NULL);
    
    /* Handle feature set/clear */
    enum_register_callback(ENUM_SETFEATURE, setfeatureCallback, NULL);
    enum_register_callback(ENUM_CLRFEATURE, clrfeatureCallback, NULL);
    
    /* Initialize the class driver */
    if (hidkbd_init(&config_descriptor.interface_descriptor, &config_descriptor.hid_descriptor, report_descriptor) != 0) {
        printf("hidkbd_init() failed\n");
        
        while (1);
    }
    
    /* Register callbacks */
    MXC_USB_EventEnable(MAXUSB_EVENT_NOVBUS, eventCallback, NULL);
    MXC_USB_EventEnable(MAXUSB_EVENT_VBUS, eventCallback, NULL);
    
    /* Register callback for keyboard events */
    if (PB_RegisterCallback(0, buttonCallback) != E_NO_ERROR) {
        printf("PB_RegisterCallback() failed\n");
        
        while (1);
    }
    
    /* Start with USB in low power mode */
    usbAppSleep();
    NVIC_EnableIRQ(USB_IRQn);
    
    /* Wait for events */
    while (1) {
    
        if (suspended || !configured) {
            LED_Off(0);
        }
        else {
            LED_On(0);
        }
        
        if (evtFlags) {
            /* Display events */
            if (MXC_GETBIT(&evtFlags, MAXUSB_EVENT_NOVBUS)) {
                MXC_CLRBIT(&evtFlags, MAXUSB_EVENT_NOVBUS);
                printf("VBUS Disconnect\n");
            }
            else if (MXC_GETBIT(&evtFlags, MAXUSB_EVENT_VBUS)) {
                MXC_CLRBIT(&evtFlags, MAXUSB_EVENT_VBUS);
                printf("VBUS Connect\n");
            }
            else if (MXC_GETBIT(&evtFlags, MAXUSB_EVENT_BRST)) {
                MXC_CLRBIT(&evtFlags, MAXUSB_EVENT_BRST);
                printf("Bus Reset\n");
            }
            else if (MXC_GETBIT(&evtFlags, MAXUSB_EVENT_SUSP)) {
                MXC_CLRBIT(&evtFlags, MAXUSB_EVENT_SUSP);
                printf("Suspended\n");
            }
            else if (MXC_GETBIT(&evtFlags, MAXUSB_EVENT_DPACT)) {
                MXC_CLRBIT(&evtFlags, MAXUSB_EVENT_DPACT);
                printf("Resume\n");
            }
            else if (MXC_GETBIT(&evtFlags, EVENT_ENUM_COMP)) {
                MXC_CLRBIT(&evtFlags, EVENT_ENUM_COMP);
                printf("Enumeration complete. Press SW2 to send character.\n");
            }
            else if (MXC_GETBIT(&evtFlags, EVENT_REMOTE_WAKE)) {
                MXC_CLRBIT(&evtFlags, EVENT_REMOTE_WAKE);
                printf("Remote Wakeup\n");
            }
        }
    }
}


/******************************************************************************/
int usbStartupCallback()
{
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
    MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_0P9EN;
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_USB);

    return E_NO_ERROR;
}

/******************************************************************************/
int usbShutdownCallback()
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_USB);
    
    return E_NO_ERROR;
}

/******************************************************************************/
/* User-supplied function to delay usec micro-seconds */
void usDelay(unsigned int usec)
{
    /* mxc_delay() takes unsigned long, so can't use it directly */
    MXC_Delay(usec);
}

/******************************************************************************/
static void usbAppSleep(void)
{
    /* TODO: Place low-power code here */
    suspended = 1;
}

/******************************************************************************/
static void usbAppWakeup(void)
{
    /* TODO: Place low-power code here */
    suspended = 0;
}

/******************************************************************************/
static void buttonCallback(void* pb)
{
    static const uint8_t chars[] = "Maxim Integrated\n";
    static int i = 0;
    int count = 0;
    int button_pressed = 0;
    
    //determine if interrupt triggered by bounce or a true button press
    while (PB_Get(0) && !button_pressed) {
        count++;
        
        if (count > 1000) {
            button_pressed = 1;
        }
    }
    
    if (button_pressed) {
        LED_Toggle(0);
        
        if (configured) {
            if (suspended && remoteWake) {
                /* The bus is suspended. Wake up the host */
                suspended = 0;
                usbAppWakeup();
                MXC_USB_RemoteWakeup();
                MXC_SETBIT(&evtFlags, EVENT_REMOTE_WAKE);
            }
            else {
                if (i >= (sizeof(chars) - 1)) {
                    i = 0;
                }
                
                hidkbd_keypress(chars[i++]);
            }
        }
    }
}

/******************************************************************************/
static int setconfigCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    /* Confirm the configuration value */
    if (sud->wValue == config_descriptor.config_descriptor.bConfigurationValue) {
        configured = 1;
        MXC_SETBIT(&evtFlags, EVENT_ENUM_COMP);
        return hidkbd_configure(config_descriptor.endpoint_descriptor.bEndpointAddress & USB_EP_NUM_MASK);
    }
    else if (sud->wValue == 0) {
        configured = 0;
        return hidkbd_deconfigure();
    }
    
    return -1;
}

/******************************************************************************/
static int setfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remoteWake = 1;
    }
    else {
        // Unknown callback
        return -1;
    }
    
    return 0;
}

/******************************************************************************/
static int clrfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remoteWake = 0;
    }
    else {
        // Unknown callback
        return -1;
    }
    
    return 0;
}

/******************************************************************************/
static int eventCallback(maxusb_event_t evt, void* data)
{
    /* Set event flag */
    MXC_SETBIT(&evtFlags, evt);
    
    switch (evt) {
    case MAXUSB_EVENT_NOVBUS:
        MXC_USB_EventDisable(MAXUSB_EVENT_BRST);
        MXC_USB_EventDisable(MAXUSB_EVENT_SUSP);
        MXC_USB_EventDisable(MAXUSB_EVENT_DPACT);
        MXC_USB_Disconnect();
        configured = 0;
        enum_clearconfig();
        hidkbd_deconfigure();
        usbAppSleep();
        break;
        
    case MAXUSB_EVENT_VBUS:
        MXC_USB_EventClear(MAXUSB_EVENT_BRST);
        MXC_USB_EventEnable(MAXUSB_EVENT_BRST, eventCallback, NULL);
        MXC_USB_EventClear(MAXUSB_EVENT_SUSP);
        MXC_USB_EventEnable(MAXUSB_EVENT_SUSP, eventCallback, NULL);
        MXC_USB_Connect();
        usbAppSleep();
        break;
        
    case MAXUSB_EVENT_BRST:
        usbAppWakeup();
        enum_clearconfig();
        hidkbd_deconfigure();
        configured = 0;
        suspended = 0;
        break;
        
    case MAXUSB_EVENT_SUSP:
        usbAppSleep();
        break;
        
    case MAXUSB_EVENT_DPACT:
        usbAppWakeup();
        break;
        
    default:
        break;
    }
    
    return 0;
}
