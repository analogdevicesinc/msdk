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

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "cdc_acm.h"
#include "fifo.h"

/***** Definitions *****/
#define FIFO_SIZE ((2 * ACM_MAX_PACKET) + 1)

// USB CDC ACM class requests
#define ACM_SET_LINE_CODING 0x20
#define ACM_GET_LINE_CODING 0x21
#define ACM_SET_CONTROL_LINE_STATE 0x22
#define ACM_SEND_BREAK 0x23

// Control Line State bits
#define CLS_DTR (1 << 0)
#define CLS_RTS (1 << 1)

/* Offset into the notify_data to set interface # */
#define NOTIFY_IF_OFFS 4

/***** File Scope Data *****/
#ifdef MAXUSB_IGNORE_DTE
#warning "NOTE: CDC-ACM class will ignore DTE"
const int DTE_present = 1;
#else
static volatile int DTE_present = 0;
#endif
static volatile int BREAK_signal = 0;

// Endpoint numbers
static uint8_t out_ep;
static uint8_t in_ep;
static uint8_t notify_ep;
static uint8_t if_num;

/* Interface # for Comm Class (to handle class-specific requests) */
static uint8_t if_num = 0;

// Line Coding
static MXC_USB_Req_t creq;
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment = 4
#elif __GNUC__
__attribute__((aligned(4)))
#endif
static acm_line_t line_coding = { 0x00002580, // 9600 bps
                                  0, 0, 8 };

// Write (IN) data
static MXC_USB_Req_t wreq;
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment = 4
#elif __GNUC__
__attribute__((aligned(4)))
#endif
static uint8_t wepbuf[ACM_MAX_PACKET];
static uint8_t wbuf[FIFO_SIZE];
static fifo_t wfifo;

// Read (OUT) data
static volatile int rreq_complete;
static MXC_USB_Req_t rreq;
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment = 4
#elif __GNUC__
__attribute__((aligned(4)))
#endif
static uint8_t repbuf[ACM_MAX_PACKET];
static uint8_t rbuf[FIFO_SIZE];
static fifo_t rfifo;

// Notification
static MXC_USB_Req_t nreq;
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment = 4
#elif __GNUC__
__attribute__((aligned(4)))
#endif
static uint8_t notify_data[] = {
    0xa1, /* bmRequestType = Notification */
    0x20, /* bNotification = SERIAL_STATE */
    0x00, 0x00, /* wValue = 0 */
    0x00, 0x00, /* wIndex = Interface # inserted by acm_init() */
    0x02, 0x00, /* wLength = 2 */
    0x02, 0x00 /* DSR active */
};

static int (*callback[ACM_NUM_CALLBACKS])(void);

static int (*chained_func)(MXC_USB_SetupPkt *, void *);
static void *chained_cbdata;

/***** Function Prototypes *****/
static int class_req(MXC_USB_SetupPkt *sud, void *cbdata);
static void svc_out_from_host(void);
static void out_callback(void *cbdata);
static void svc_in_to_host(void *cbdata);
static int set_line_coding(void);
static int get_line_coding(void);

/******************************************************************************/
int acm_init(const MXC_USB_interface_descriptor_t *if_desc)
{
    out_ep = 0;
    in_ep = 0;
    notify_ep = 0;
    memset(callback, 0, sizeof(callback));

    /* Pull any existing class-specific callback, in case of multi-class devices */
    enum_query_callback(ENUM_CLASS_REQ, &chained_func, &chained_cbdata);

    /* Store interface number */
    if_num = if_desc->bInterfaceNumber;

    /* Handle class-specific SETUP requests */
    return enum_register_callback(ENUM_CLASS_REQ, class_req, NULL);
}

/******************************************************************************/
int acm_register_callback(acm_callback_t cbnum, int (*func)(void))
{
    if (cbnum >= ACM_NUM_CALLBACKS) {
        return -1;
    }

    callback[cbnum] = func;
    return 0;
}

/******************************************************************************/
int acm_configure(const acm_cfg_t *cfg)
{
    int err;

#ifndef MAXUSB_IGNORE_DTE
    DTE_present = 0;
#endif
    BREAK_signal = 0;

    if ((cfg->out_maxpacket > ACM_MAX_PACKET) || (cfg->in_maxpacket > ACM_MAX_PACKET) ||
        (cfg->notify_maxpacket > ACM_MAX_PACKET)) {
        return -1;
    }

    out_ep = cfg->out_ep;
    if ((err = MXC_USB_ConfigEp(out_ep, MAXUSB_EP_TYPE_OUT, cfg->out_maxpacket)) != 0) {
        acm_deconfigure();
        return err;
    }

    in_ep = cfg->in_ep;
    if ((err = MXC_USB_ConfigEp(in_ep, MAXUSB_EP_TYPE_IN, cfg->in_maxpacket)) != 0) {
        acm_deconfigure();
        return err;
    }

    notify_ep = cfg->notify_ep;
    if ((err = MXC_USB_ConfigEp(notify_ep, MAXUSB_EP_TYPE_IN, cfg->notify_maxpacket)) != 0) {
        acm_deconfigure();
        return err;
    }

    fifo_init(&wfifo, wbuf, FIFO_SIZE);
    memset(&wreq, 0, sizeof(MXC_USB_Req_t));
    wreq.ep = in_ep;
    wreq.data = wepbuf;
    wreq.callback = svc_in_to_host;
    wreq.cbdata = &wreq;
    wreq.type = MAXUSB_TYPE_PKT;

    fifo_init(&rfifo, rbuf, FIFO_SIZE);
    memset(&rreq, 0, sizeof(MXC_USB_Req_t));
    rreq.ep = out_ep;
    rreq.data = repbuf;
    rreq.reqlen = sizeof(repbuf);
    rreq.callback = out_callback;
    rreq.cbdata = &rreq;
    rreq.type = MAXUSB_TYPE_PKT;

    memset(&nreq, 0, sizeof(MXC_USB_Req_t));
    nreq.ep = notify_ep;
    notify_data[NOTIFY_IF_OFFS] = if_num;
    nreq.data = (uint8_t *)notify_data;
    nreq.reqlen = sizeof(notify_data);
    nreq.callback = NULL;
    nreq.cbdata = NULL;
    nreq.type = MAXUSB_TYPE_TRANS;

#ifdef MAXUSB_IGNORE_DTE
    /* Register an initial OUT request */
    rreq_complete = 0;
    rreq.data = repbuf;
    MXC_USB_ReadEndpoint(&rreq);
#endif

    return 0;
}

/******************************************************************************/
int acm_deconfigure(void)
{
    MXC_USB_RemoveRequest(&rreq);

    /* deconfigure EPs */
    if (out_ep != 0) {
        MXC_USB_ResetEp(out_ep);
        out_ep = 0;
    }

    if (in_ep != 0) {
        MXC_USB_ResetEp(in_ep);
        in_ep = 0;
    }

    if (notify_ep != 0) {
        MXC_USB_ResetEp(notify_ep);
        notify_ep = 0;
    }

    /* clear driver state */
    fifo_clear(&wfifo);
    fifo_clear(&rfifo);
#ifndef MAXUSB_IGNORE_DTE
    DTE_present = 0;
#endif
    BREAK_signal = 0;

    return 0;
}

/******************************************************************************/
int acm_present(void)
{
    return DTE_present;
}

/******************************************************************************/
const acm_line_t *acm_line_coding(void)
{
    return &line_coding;
}

/******************************************************************************/
int acm_canread(void)
{
    /* Write available data into the FIFO first */
    svc_out_from_host();

    return fifo_level(&rfifo);
}

/******************************************************************************/
int acm_read(uint8_t *buf, unsigned int len)
{
    unsigned int i;
    uint8_t byte;

    for (i = 0; i < len; i++) {
        while (fifo_get8(&rfifo, &byte) != 0) {
            /* Check for Break in loop */
            if (BREAK_signal) {
                return -2;
            }

#ifndef MAXUSB_IGNORE_DTE
            if (!DTE_present) {
                /* Disconnected during a read, return EOF (0) */
                return 0;
            }
#endif

            /* Write available data into the FIFO */
            svc_out_from_host();
        }

        buf[i] = byte;
    }

    return i;
}

/******************************************************************************/
int acm_write(uint8_t *buf, unsigned int len)
{
    unsigned int i = 0;

    // Write data into the FIFO
    while (len > 0) {
        if (fifo_put8(&wfifo, buf[i]) == 0) {
            /* Success */
            i++;
            len--;
        } else {
            /* Buffer full -- see if some characters can be sent to host */
            if (wreq.reqlen == 0) {
                svc_in_to_host(&wreq);
            }
        }
    }

    /* Finally, make sure characters are sent to host */
    if (wreq.reqlen == 0) {
        svc_in_to_host(&wreq);
    }

    return i;
}

/******************************************************************************/
static void svc_out_from_host(void)
{
    int newdata = 0;

    if (rreq_complete) {
        // Copy as much data into the local buffer as possible
        for (; rreq.actlen > 0; rreq.actlen--) {
            if (fifo_put8(&rfifo, *rreq.data) != 0) {
                break;
            }
            newdata = 1;
            rreq.data++;
        }

        /* After all of the data has been consumed, register the next request if
     * still configured and connected to the DTE
     */

        if (!rreq.actlen && (out_ep > 0) && DTE_present) {
            rreq_complete = 0;
            rreq.error_code = 0;
            rreq.data = repbuf;
            MXC_USB_ReadEndpoint(&rreq);
        }
    }

    // Call the callback if there is new data
    if (newdata && (callback[ACM_CB_READ_READY] != NULL)) {
        callback[ACM_CB_READ_READY]();
    }
}

/******************************************************************************/
static void out_callback(void *cbdata)
{
    rreq_complete = 1;
    svc_out_from_host();
}

/******************************************************************************/
static void svc_in_to_host(void *cbdata)
{
    int i;
    uint8_t byte;

    // An error will occur when the host has been disconnected.
    // Register the next request if still configured and there is data to send
    if ((wreq.error_code == 0) && (in_ep > 0) && !fifo_empty(&wfifo)) {
        for (i = 0; i < sizeof(wepbuf); i++) {
            if (fifo_get8(&wfifo, &byte) != 0) {
                break;
            }
            wepbuf[i] = byte;
        }

        wreq.data = wepbuf;
        wreq.reqlen = i;
        wreq.actlen = 0;

        // Register the next request
        MXC_USB_WriteEndpoint(&wreq);
    } else {
        // Clear the request length to indicate that there is not an active request
        wreq.reqlen = 0;
        wreq.error_code = 0;
    }
}

/******************************************************************************/
static int class_req(MXC_USB_SetupPkt *sud, void *cbdata)
{
    int result = -1;

    if (((sud->bmRequestType & RT_RECIP_MASK) == RT_RECIP_IFACE) && (sud->wIndex == if_num)) {
        /* Directed to our interface */
        switch (sud->bRequest) {
        case ACM_SET_LINE_CODING:
            result = set_line_coding();
            if (!result) {
                /* Success, no data stage, but used to defer ACK/STALL to application */
                result = 1;
            }
            break;
        case ACM_GET_LINE_CODING:
            result = get_line_coding();
            if (!result) {
                /* Success, with data stage */
                result = 1;
            }
            break;
        case ACM_SET_CONTROL_LINE_STATE:
#ifndef MAXUSB_IGNORE_DTE
            if (sud->wValue & CLS_DTR) {
                /* DTE is now present, enable initial notification */

                /* Prepare the serial state notification */
                MXC_USB_WriteEndpoint(&nreq);

                /* Register an initial OUT request -- only if DTE is used */
                rreq_complete = 0;
                rreq.data = repbuf;
                MXC_USB_ReadEndpoint(&rreq);

                DTE_present = 1;

                if (callback[ACM_CB_CONNECTED]) {
                    callback[ACM_CB_CONNECTED]();
                }
            } else {
                DTE_present = 0;
                /* DTE disappeared */
                MXC_USB_RemoveRequest(&rreq);
                if (callback[ACM_CB_DISCONNECTED]) {
                    callback[ACM_CB_DISCONNECTED]();
                }
            }
#endif
            /* Success, no data stage */
            result = 0;
            break;
        case ACM_SEND_BREAK:
            if (sud->wValue > 0) {
                BREAK_signal = 1;
            } else {
                BREAK_signal = 0;
            }
            /* Success, no data stage */
            result = 0;
            break;
        default:
            /* Unexpected message received -- stall */
            break;
        }
    } else {
        /* Not for this class, send to chained classes (if any) */
        if (chained_func != NULL) {
            result = chained_func(sud, chained_cbdata);
        }
    }

    return result;
}

/******************************************************************************/
static void set_line_coding_callback(void *cbdata)
{
    int result = 0;
    MXC_USB_Req_t *req = (MXC_USB_Req_t *)cbdata;

    if (!req->error_code) {
        if (callback[ACM_CB_SET_LINE_CODING]) {
            /* Application can reject this change from the host, if desired */
            result = callback[ACM_CB_SET_LINE_CODING]();
        }

        if (result == -1) {
            MXC_USB_Stall(0);
        } else {
            MXC_USB_Ackstat(0);
        }
    } else {
        /* Read request failed */
        MXC_USB_Stall(0);
    }
}

/******************************************************************************/
static int set_line_coding(void)
{
    memset(&creq, 0, sizeof(MXC_USB_Req_t));
    creq.ep = 0;
    creq.data = (uint8_t *)&line_coding;
    creq.reqlen = sizeof(line_coding);
    creq.callback = set_line_coding_callback;
    creq.cbdata = &creq;
    creq.type = MAXUSB_TYPE_TRANS;

    return MXC_USB_ReadEndpoint(&creq);
}

/******************************************************************************/
static void get_line_coding_callback(void *cbdata)
{
    MXC_USB_Req_t *req = (MXC_USB_Req_t *)cbdata;

    if (!req->error_code) {
        MXC_USB_Ackstat(0);
    } else {
        MXC_USB_Stall(0);
    }
}

/******************************************************************************/
static int get_line_coding(void)
{
    memset(&creq, 0, sizeof(MXC_USB_Req_t));
    creq.ep = 0;
    creq.data = (uint8_t *)&line_coding;
    creq.reqlen = sizeof(line_coding);
    creq.callback = get_line_coding_callback;
    creq.cbdata = &creq;
    creq.type = MAXUSB_TYPE_TRANS;

    return MXC_USB_WriteEndpoint(&creq);
}
