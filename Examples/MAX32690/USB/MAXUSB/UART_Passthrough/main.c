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

/**
 * @file
 * @brief   USB CDC-ACM example
 * @details This project creates a virtual COM port, which loops back data sent to it.
 *          Load the project, connect a cable from the PC to the USB connector
 *          on the Evaluation Kit, and observe that the PC now recognizes a new COM port.
 *          A driver for the COM port, if needed, is located in the Driver/ subdirectory.
 *
 */

/* **** Includes **** */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "mxc_errors.h"
#include "mcr_regs.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "board.h"
#include "led.h"
#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "cdc_acm.h"
#include "descriptors.h"
#include "uart.h"
#include "ring_buffer.h"
#include "tmr.h"
#include "mxc_device.h"
#include "mxc_sys.h"

/* **** Definitions **** */
#define EVENT_ENUM_COMP MAXUSB_NUM_EVENTS
#define EVENT_REMOTE_WAKE (EVENT_ENUM_COMP + 1)

#define BUFFER_SIZE 64

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define OST_CLOCK_SOURCE MXC_TMR_IBRO_CLK
#define OST_TIMER MXC_TMR4
#define OST_FREQ 1000 // 1khz = 1ms

/* **** Global Data **** */
volatile int configured;
volatile int suspended;
volatile unsigned int event_flags;
int remote_wake_en;

#define PASSTHROUGH_UART MXC_UART2
static mxc_uart_req_t rx_req;
static mxc_uart_req_t tx_req;

uint8_t txData[512] = { 0 };
uint8_t rxData[512] = { 0 };
bool txOngoing = false;

static RingBuffer ring_buffer;

/* **** Function Prototypes **** */
static int setconfig_callback(MXC_USB_SetupPkt *sud, void *cbdata);
static int setfeatureCallback(MXC_USB_SetupPkt *sud, void *cbdata);
static int clrfeatureCallback(MXC_USB_SetupPkt *sud, void *cbdata);
static int event_callback(maxusb_event_t evt, void *data);
static void usbAppSleep(void);
static void usbAppWakeup(void);
static int usb_read_callback(void);
static void echo_usb(void);
int usbStartupCallback(void);
int usbShutdownCallback(void);

/* **** File Scope Variables **** */

/* This EP assignment must match the Configuration Descriptor */
static acm_cfg_t acm_cfg = {
    2, /* EP OUT */
    MXC_USBHS_MAX_PACKET, /* OUT max packet size */
    3, /* EP IN */
    MXC_USBHS_MAX_PACKET, /* IN max packet size */
    4, /* EP Notify */
    MXC_USBHS_MAX_PACKET, /* Notify max packet size */
};

static volatile int usb_read_complete;

/* User-supplied function to delay usec micro-seconds */
void delay_us(unsigned int usec)
{
    /* mxc_delay() takes unsigned long, so can't use it directly */
    MXC_Delay(usec);
}

void restart_oneshot_timer(void)
{
    MXC_TMR_Start(OST_TIMER);
}
void startTx(void)
{
    uint32_t txAmt = ring_buffer_get_data_cnt(&ring_buffer);

    for (uint8_t i = 0; i < txAmt; i++) {
        MXC_ASSERT(ring_buffer_get(&ring_buffer, &txData[i]));
    }

    acm_write(txData, txAmt);
}
void uartRxCallback(mxc_uart_req_t *req, int error)
{
    for (uint32_t i = 0; i < req->rxCnt; i++) {
        ring_buffer_put(&ring_buffer, rxData[i]);
    }

    /* 
        If we dont have a full buffer just keep restarting until idle. 
        Othewise immediately start
    */
    if (ring_buffer_get_data_cnt(&ring_buffer) != 64) {
        restart_oneshot_timer();
    } else {
        startTx();
    }

    MXC_UART_TransactionDMA(&rx_req);
}

void txCallback(mxc_uart_req_t *req, int error) {}
void oneshot_init(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(OST_TIMER, OST_CLOCK_SOURCE, 128, OST_FREQ);

    MXC_TMR_Shutdown(OST_TIMER);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.bitMode = TMR_BIT_MODE_32;
    tmr.clock = OST_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    if (MXC_TMR_Init(OST_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    MXC_TMR_EnableInt(OST_TIMER);
    NVIC_EnableIRQ(TMR4_IRQn);
}
void TMR4_IRQHandler(void)
{
    MXC_TMR_ClearFlags(OST_TIMER);
    startTx();
}
uint32_t format_product_id(const char *id, uint32_t len, uint8_t *pid_buf)
{

    const uint32_t total_len =  2*(len + 1);
    pid_buf[0] = total_len;
    pid_buf[1] = 3; //bDescriptorType
    pid_buf += 2;

    for(uint32_t i = 0; i < 2*len; i++)
    {
        pid_buf[i] = i % 2 ? 0 : id[i / 2];
    }


    return total_len;
}
/* ************************************************************************** */
int main(void)
{
    ring_buffer_init(&ring_buffer);
    oneshot_init();

    
    MXC_UART_Init(PASSTHROUGH_UART, 115200, MXC_UART_APB_CLK);
    rx_req.uart = PASSTHROUGH_UART;
    rx_req.rxData = rxData;
    rx_req.rxLen = 1;
    rx_req.callback = uartRxCallback;

    tx_req.uart = PASSTHROUGH_UART;

    MXC_UART_SetAutoDMAHandlers(PASSTHROUGH_UART, true);
    MXC_UART_TransactionDMA(&rx_req);

    maxusb_cfg_options_t usb_opts;

    uint8_t checksum = 0;
    uint8_t usn[MXC_SYS_USN_LEN] = {0};

    MXC_ASSERT(MXC_SYS_GetUSN(usn, &checksum) == E_NO_ERROR);


    char id[32] = {0};
    const uint32_t sn = usn[6] | usn[7] << 8 | usn[8] << 16;
    snprintf(id, sizeof(id), "MAX32690 UART %u", sn);


    __attribute__((aligned(4))) uint8_t prod_id_desc[64] = {0};
    format_product_id(id, strlen(id), prod_id_desc);

    printf("\n\n***** " TOSTRING(TARGET) " USB CDC-ACM Example *****\n");
    printf("Waiting for VBUS...\n");

    /* Initialize state */
    configured = 0;
    suspended = 0;
    event_flags = 0;
    remote_wake_en = 0;

    /* Start out in full speed */
    usb_opts.enable_hs = 1;
    usb_opts.delay_us = delay_us; /* Function which will be used for delays */
    usb_opts.init_callback = usbStartupCallback;
    usb_opts.shutdown_callback = usbShutdownCallback;

    /* Initialize the usb module */
    if (MXC_USB_Init(&usb_opts) != 0) {
        printf("usb_init() failed\n");
        while (1) {}
    }

    /* Initialize the enumeration module */
    if (enum_init() != 0) {
        printf("enum_init() failed\n");
        while (1) {}
    }

    /* Register enumeration data */
    enum_register_descriptor(ENUM_DESC_DEVICE, (uint8_t *)&device_descriptor, 0);
    enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t *)&config_descriptor, 0);

    if (usb_opts.enable_hs) {
        /* Two additional descriptors needed for high-speed operation */
        enum_register_descriptor(ENUM_DESC_OTHER, (uint8_t *)&config_descriptor_hs, 0);
        enum_register_descriptor(ENUM_DESC_QUAL, (uint8_t *)&device_qualifier_descriptor, 0);
    }

    enum_register_descriptor(ENUM_DESC_STRING, lang_id_desc, 0);
    enum_register_descriptor(ENUM_DESC_STRING, mfg_id_desc, 1);
    enum_register_descriptor(ENUM_DESC_STRING, prod_id_desc, 2);
    enum_register_descriptor(ENUM_DESC_STRING, serial_id_desc, 3);
    enum_register_descriptor(ENUM_DESC_STRING, cdcacm_func_desc, 4);

    /* Handle configuration */
    enum_register_callback(ENUM_SETCONFIG, setconfig_callback, NULL);

    /* Handle feature set/clear */
    enum_register_callback(ENUM_SETFEATURE, setfeatureCallback, NULL);
    enum_register_callback(ENUM_CLRFEATURE, clrfeatureCallback, NULL);

    /* Initialize the class driver */
    if (acm_init(&config_descriptor.comm_interface_descriptor) != 0) {
        printf("acm_init() failed\n");
        while (1) {}
    }

    /* Register callbacks */
    MXC_USB_EventEnable(MAXUSB_EVENT_NOVBUS, event_callback, NULL);
    MXC_USB_EventEnable(MAXUSB_EVENT_VBUS, event_callback, NULL);
    acm_register_callback(ACM_CB_READ_READY, usb_read_callback);

    usb_read_complete = 0;

    /* Start with USB in low power mode */
    usbAppSleep();
    NVIC_EnableIRQ(USB_IRQn);

    /* Wait for events */
    while (1) {
        echo_usb();

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
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_BRSTDN)) { ///
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_BRSTDN);
                printf("Bus Reset Done: %s speed\n",
                       (MXC_USB_GetStatus() & MAXUSB_STATUS_HIGH_SPEED) ? "High" : "Full");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_SUSP)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_SUSP);
                printf("Suspended\n");
            } else if (MXC_GETBIT(&event_flags, MAXUSB_EVENT_DPACT)) {
                MXC_CLRBIT(&event_flags, MAXUSB_EVENT_DPACT);
                printf("Resume\n");
            } else if (MXC_GETBIT(&event_flags, EVENT_ENUM_COMP)) {
                MXC_CLRBIT(&event_flags, EVENT_ENUM_COMP);
                printf("Enumeration complete...\n");
            } else if (MXC_GETBIT(&event_flags, EVENT_REMOTE_WAKE)) {
                MXC_CLRBIT(&event_flags, EVENT_REMOTE_WAKE);
                printf("Remote Wakeup\n");
            }
        }
    }
}

/* ************************************************************************** */
static void echo_usb(void)
{
    int chars;
    uint8_t buffer[BUFFER_SIZE];

    if ((chars = acm_canread()) > 0) {
        if (chars > BUFFER_SIZE) {
            chars = BUFFER_SIZE;
        }

        /* Read the data from USB */
        if (acm_read(buffer, chars) != chars) {
            printf("acm_read() failed\n");
            return;
        }

        /* Echo it back */
        if (acm_present()) {
            // if (acm_write(buffer, chars) != chars) {
            //     printf("acm_write() failed\n");
            // }

            memcpy(rxData, buffer, chars);
            tx_req.txData = rxData;
            tx_req.txLen = chars;
            MXC_UART_TransactionDMA(&tx_req);
        }
    }
}

/******************************************************************************/
int usbStartupCallback(void)
{
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
    MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_0P9EN;
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_USB);

    return E_NO_ERROR;
}

/******************************************************************************/
int usbShutdownCallback(void)
{
    //return MXC_SYS_USBHS_Shutdown();
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_USB);

    return E_NO_ERROR;
}

/* ************************************************************************** */
static int setconfig_callback(MXC_USB_SetupPkt *sud, void *cbdata)
{
    /* Confirm the configuration value */
    if (sud->wValue == config_descriptor.config_descriptor.bConfigurationValue) {
        configured = 1;
        MXC_SETBIT(&event_flags, EVENT_ENUM_COMP);

        acm_cfg.out_ep = config_descriptor.endpoint_descriptor_4.bEndpointAddress & 0x7;
        acm_cfg.out_maxpacket = config_descriptor.endpoint_descriptor_4.wMaxPacketSize;
        acm_cfg.in_ep = config_descriptor.endpoint_descriptor_5.bEndpointAddress & 0x7;
        acm_cfg.in_maxpacket = config_descriptor.endpoint_descriptor_5.wMaxPacketSize;
        acm_cfg.notify_ep = config_descriptor.endpoint_descriptor_3.bEndpointAddress & 0x7;
        acm_cfg.notify_maxpacket = config_descriptor.endpoint_descriptor_3.wMaxPacketSize;

        return acm_configure(&acm_cfg); /* Configure the device class */
    } else if (sud->wValue == 0) {
        configured = 0;
        return acm_deconfigure();
    }

    return -1;
}

/* ************************************************************************** */
static int setfeatureCallback(MXC_USB_SetupPkt *sud, void *cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 1;
    } else {
        /* Unknown callback */
        return -1;
    }

    return 0;
}

/* ************************************************************************** */
static int clrfeatureCallback(MXC_USB_SetupPkt *sud, void *cbdata)
{
    if (sud->wValue == FEAT_REMOTE_WAKE) {
        remote_wake_en = 0;
    } else {
        /* Unknown callback */
        return -1;
    }

    return 0;
}

/* ************************************************************************** */
static void usbAppSleep(void)
{
    suspended = 1;
}

/* ************************************************************************** */
static void usbAppWakeup(void)
{
    suspended = 0;
}

/* ************************************************************************** */
static int event_callback(maxusb_event_t evt, void *data)
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
        usbAppSleep();
        break;

    case MAXUSB_EVENT_VBUS:
        MXC_USB_EventClear(MAXUSB_EVENT_BRST);
        MXC_USB_EventEnable(MAXUSB_EVENT_BRST, event_callback, NULL);
        MXC_USB_EventClear(MAXUSB_EVENT_BRSTDN); ///
        MXC_USB_EventEnable(MAXUSB_EVENT_BRSTDN, event_callback, NULL); ///
        MXC_USB_EventClear(MAXUSB_EVENT_SUSP);
        MXC_USB_EventEnable(MAXUSB_EVENT_SUSP, event_callback, NULL);
        MXC_USB_Connect();
        usbAppSleep();
        break;

    case MAXUSB_EVENT_BRST:
        usbAppWakeup();
        enum_clearconfig();
        acm_deconfigure();
        configured = 0;
        suspended = 0;
        break;

    case MAXUSB_EVENT_BRSTDN: ///
        if (MXC_USB_GetStatus() & MAXUSB_STATUS_HIGH_SPEED) {
            enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t *)&config_descriptor_hs, 0);
            enum_register_descriptor(ENUM_DESC_OTHER, (uint8_t *)&config_descriptor, 0);
        } else {
            enum_register_descriptor(ENUM_DESC_CONFIG, (uint8_t *)&config_descriptor, 0);
            enum_register_descriptor(ENUM_DESC_OTHER, (uint8_t *)&config_descriptor_hs, 0);
        }
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

/* ************************************************************************** */

static int usb_read_callback(void)
{
    usb_read_complete = 1;
    return 0;
}

/* ************************************************************************** */

void USB_IRQHandler(void)
{
    MXC_USB_EventHandler();
}

/* ************************************************************************** */

void SysTick_Handler(void)
{
    MXC_DelayHandler();
}
