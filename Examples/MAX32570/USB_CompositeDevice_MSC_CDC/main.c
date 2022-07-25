/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

#include <stdio.h>
#include <stddef.h>

#include <MAX32xxx.h>

#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "cdc_acm.h"
#include "msc.h"
#include "descriptors.h"
#include "mscmem.h"

/* **** Definitions **** */
#define EVENT_ENUM_COMP   MAXUSB_NUM_EVENTS
#define EVENT_REMOTE_WAKE (EVENT_ENUM_COMP + 1)

#define BUFFER_SIZE 64

#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

/* **** Global Data **** */
volatile int configured;
volatile int suspended;
volatile unsigned int event_flags;
int remote_wake_en;
static volatile int usb_read_complete;

/* This EP assignment must match the Configuration Descriptor */
static msc_cfg_t msc_cfg = {
    1,                    /* EP OUT */
    MXC_USBHS_MAX_PACKET, /* OUT max packet size */
    2,                    /* EP IN */
    MXC_USBHS_MAX_PACKET, /* IN max packet size */
};

static const msc_idstrings_t ids = {
    "MAXIM",       /* Vendor string.  Maximum of 8 bytes */
    "MSC Example", /* Product string.  Maximum of 16 bytes */
    "1.0"          /* Version string.  Maximum of 4 bytes */
};

/* This EP assignment must match the Configuration Descriptor */
static acm_cfg_t acm_cfg = {
    3,                    /* EP OUT */
    MXC_USBHS_MAX_PACKET, /* OUT max packet size */
    4,                    /* EP IN */
    MXC_USBHS_MAX_PACKET, /* IN max packet size */
    5,                    /* EP Notify */
    MXC_USBHS_MAX_PACKET, /* Notify max packet size */
};

/* Functions to control "disk" memory. See msc.h for definitions. */
static const msc_mem_t mem = {
    mscmem_Init, mscmem_Start, mscmem_Stop, mscmem_Ready, mscmem_Size, mscmem_Read, mscmem_Write,
};

/* **** Function Prototypes **** */
static int setconfigCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int setfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int clrfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata);
static int eventCallback(maxusb_event_t evt, void* data);
static void usbAppSleep(void);
static void usbAppWakeup(void);
static int usbReadCallback(void);
int usbStartupCallback();
int usbShutdownCallback();
static void echoUSB(void);

/**
 * User-supplied function to delay usec micro-seconds
 *
 * @param[in]  usec  The usec time to delay.
 */
void delay_us(unsigned int usec)
{
    /* mxc_delay() takes unsigned long, so can't use it directly */
    MXC_Delay(usec);
}

/* ************************************************************************** */

void USB_IRQHandler(void)
{
    MXC_USB_EventHandler();
}

/* ************************************************************************** */
int main(void)
{
    maxusb_cfg_options_t usb_opts;

    printf("\n\n****** " TOSTRING(
        TARGET) " USB Composite Device (CDC and Mass Storage) Example ******\n");
    printf("Waiting for VBUS...\n");

    /* Initialize state */
    configured     = 0;
    suspended      = 0;
    event_flags    = 0;
    remote_wake_en = 0;

    /* Start out in full speed */
    usb_opts.enable_hs         = 0;
    usb_opts.delay_us          = delay_us; /* Function which will be used for delays */
    usb_opts.init_callback     = usbStartupCallback;
    usb_opts.shutdown_callback = usbShutdownCallback;

    /* Initialize the usb module */
    if (MXC_USB_Init(&usb_opts) != 0) {
        printf("usb_init() failed\n");

        while (1)
            ;
    }

    /* Initialize the enumeration module */
    if (enum_init() != 0) {
        printf("enum_init() failed\n");

        while (1)
            ;
    }

    /* Register enumeration data */
    enum_register_descriptor(ENUM_DESC_DEVICE, (uint8_t*)&composite_device_descriptor, 0);
    enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t*)&composite_config_descriptor, 0);
    enum_register_descriptor(ENUM_DESC_STRING, lang_id_desc, 0);
    enum_register_descriptor(ENUM_DESC_STRING, mfg_id_desc, 1);
    enum_register_descriptor(ENUM_DESC_STRING, prod_id_desc, 2);
    enum_register_descriptor(ENUM_DESC_STRING, serial_id_desc, 3);
    enum_register_descriptor(ENUM_DESC_STRING, cdcacm_func_desc, 4);
    enum_register_descriptor(ENUM_DESC_STRING, msc_func_desc, 5);

    /* Handle configuration */
    enum_register_callback(ENUM_SETCONFIG, setconfigCallback, NULL);

    /* Handle feature set/clear */
    enum_register_callback(ENUM_SETFEATURE, setfeatureCallback, NULL);
    enum_register_callback(ENUM_CLRFEATURE, clrfeatureCallback, NULL);

    /* Initialize the class driver */
    if (msc_init(&composite_config_descriptor.msc_interface_descriptor, &ids, &mem) != 0) {
        printf("msc_init() failed\n");

        while (1)
            ;
    }

    if (acm_init(&composite_config_descriptor.comm_interface_descriptor) != 0) {
        printf("acm_init() failed\n");

        while (1)
            ;
    }

    /* Register callbacks */
    MXC_USB_EventEnable(MAXUSB_EVENT_NOVBUS, eventCallback, NULL);
    MXC_USB_EventEnable(MAXUSB_EVENT_VBUS, eventCallback, NULL);
    acm_register_callback(ACM_CB_READ_READY, usbReadCallback);

    usb_read_complete = 0;

    /* Start with USB in low power mode */
    usbAppSleep();
    NVIC_EnableIRQ(USB_IRQn);

    /* Wait for events */
    while (1) {
        echoUSB();

        if (suspended || !configured) {
            LED_Off(0);
        } else {
            LED_On(0);
        }

        if (event_flags) {
            /* Display events */
            if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_NOVBUS)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_NOVBUS);
                printf("VBUS Disconnect\n");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_VBUS)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_VBUS);
                printf("VBUS Connect\n");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_BRST)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_BRST);
                printf("Bus Reset\n");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_SUSP)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_SUSP);
                printf("Suspended\n");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_DPACT)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_DPACT);
                printf("Resume\n");
            } else if (MXC_GETBIT(&event_flags, EVENT_ENUM_COMP)) {
                MXC_CLRBIT(&event_flags, EVENT_ENUM_COMP);
                printf("Enumeration complete.\n");
            } else if (MXC_GETBIT(&event_flags, EVENT_REMOTE_WAKE)) {
                MXC_CLRBIT(&event_flags, EVENT_REMOTE_WAKE);
                printf("Remote Wakeup\n");
            }
        }
    }
}

/******************************************************************************/
int usbStartupCallback()
{
    // Startup the HIRC96M clock if it's not on already
    if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IPO_EN)) {
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IPO_EN;

        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IPO_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);

    return E_NO_ERROR;
}

/******************************************************************************/
int usbShutdownCallback()
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_USB);

    return E_NO_ERROR;
}

/* ************************************************************************** */

static int setfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 1;
    } else {
        // Unknown callback
        return -1;
    }

    return 0;
}

/* ************************************************************************** */

static int clrfeatureCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 0;
    } else {
        // Unknown callback
        return -1;
    }

    return 0;
}

/* ************************************************************************** */

static void usbAppSleep(void)
{
    /* TODO: Place low-power code here */
    suspended = 1;
}

/* ************************************************************************** */

static void usbAppWakeup(void)
{
    /* TODO: Place low-power code here */
    suspended = 0;
}

/* ************************************************************************** */

static int setconfigCallback(MXC_USB_SetupPkt* sud, void* cbdata)
{
    /* Confirm the configuration value */
    if (sud->wValue == composite_config_descriptor.config_descriptor.bConfigurationValue) {
        //      on++;
        configured = 1;
        MXC_SETBIT(&event_flags, EVENT_ENUM_COMP);
        msc_cfg.out_ep = composite_config_descriptor.endpoint_descriptor_1.bEndpointAddress & 0x7;
        msc_cfg.out_maxpacket = composite_config_descriptor.endpoint_descriptor_1.wMaxPacketSize;
        msc_cfg.in_ep = composite_config_descriptor.endpoint_descriptor_2.bEndpointAddress & 0x7;
        msc_cfg.in_maxpacket = composite_config_descriptor.endpoint_descriptor_2.wMaxPacketSize;

        acm_cfg.out_ep = composite_config_descriptor.endpoint_descriptor_4.bEndpointAddress & 0x7;
        acm_cfg.out_maxpacket = composite_config_descriptor.endpoint_descriptor_4.wMaxPacketSize;
        acm_cfg.in_ep = composite_config_descriptor.endpoint_descriptor_5.bEndpointAddress & 0x7;
        acm_cfg.in_maxpacket = composite_config_descriptor.endpoint_descriptor_5.wMaxPacketSize;
        acm_cfg.notify_ep =
            composite_config_descriptor.endpoint_descriptor_3.bEndpointAddress & 0x7;
        acm_cfg.notify_maxpacket = composite_config_descriptor.endpoint_descriptor_3.wMaxPacketSize;

        msc_configure(&msc_cfg,
                      composite_config_descriptor.msc_interface_descriptor.bInterfaceNumber);
        return acm_configure(&acm_cfg);
    } else if (sud->wValue == 0) {
        configured = 0;
        msc_deconfigure(composite_config_descriptor.msc_interface_descriptor.bInterfaceNumber);
        return acm_deconfigure();
    }

    return -1;
}

/* ************************************************************************** */

static int eventCallback(maxusb_event_t evt, void* data)
{
    /* Set event flag */
    MXC_SETBIT(&event_flags, evt);

    switch (evt) {
        case MAXUSB_EVENT_NOVBUS:
            MXC_USB_EventDisable(MAXUSB_EVENT_BRST);
            MXC_USB_EventDisable(MAXUSB_EVENT_SUSP);
            MXC_USB_EventDisable(MAXUSB_EVENT_DPACT);
            MXC_USB_Disconnect();
            configured = 0;
            enum_clearconfig();
            acm_deconfigure();
            msc_deconfigure(composite_config_descriptor.msc_interface_descriptor.bInterfaceNumber);
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
            acm_deconfigure();
            msc_deconfigure(composite_config_descriptor.msc_interface_descriptor.bInterfaceNumber);
            configured = 0;
            suspended  = 0;
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

/******************************************************************************/
static void echoUSB(void)
{
    int chars;
    uint8_t buffer[BUFFER_SIZE];

    if ((chars = acm_canread()) > 0) {
        if (chars > BUFFER_SIZE) {
            chars = BUFFER_SIZE;
        }

        // Read the data from USB
        if (acm_read(buffer, chars) != chars) {
            printf("acm_read() failed\n");
            return;
        }

        // Echo it back
        if (acm_present()) {
            if (acm_write(buffer, chars) != chars) {
                printf("acm_write() failed\n");
            }
        }
    }
}

/******************************************************************************/
static int usbReadCallback(void)
{
    usb_read_complete = 1;
    return 0;
}