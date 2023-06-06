/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdio.h>
#include "usb.h"
#include "enumerate.h"
#include "usb_event.h"
#include "ccid.h"

#define CCID_CMD_HDR_LEN 10
#define CCID_MAX_XFER 0xFF

/* Structure to hold callback pointers */
typedef struct {
  int (*fnaddr)(MXC_USB_SetupPkt *, void *);
  void *cbdata;
} callback_t;

/***** File Scope Data *****/
static int (*callback[CCID_NUM_CALLBACKS])(MXC_USB_Req_t *);
static callback_t chain_class_req = {NULL, NULL};

static MXC_USB_Req_t pr_req, rp_req, notify_req; /* PC_to_RDR and RDR_to_PC requests */
static uint8_t xfer_buf[CCID_MAX_XFER];

static unsigned int ccid_iface = 0;
static uint8_t out_ep;
static uint8_t in_ep;
static uint8_t notify_ep;

static int received = 0;

/***** Function Prototypes *****/
static int class_req(MXC_USB_SetupPkt *sud, void *cbdata);
static void ccid_received(void *cbdata);
static int ccid_lodge_out(void *);

/******************************************************************************/
int ccid_init(unsigned int interface)
{
    ccid_deconfigure();

    ccid_iface = interface;

    /* Handle class-specific SETUP requests */
    return enum_register_callback(ENUM_CLASS_REQ, class_req, NULL);
}

/******************************************************************************/
void ccid_chain_class_req(int (*func)(MXC_USB_SetupPkt *, void *), void *cbdata)
{
    /* 
     * If user desires, pass any unhandled class requests to specified func.
     *
     * This is useful for composite devices which have more than one class,
     * such as a combined CDC-ACM and CCID.
     *
     */
    chain_class_req.fnaddr = func;
    chain_class_req.cbdata = cbdata;
}

/******************************************************************************/
int ccid_configure(const ccid_cfg_t *cfg)
{
    int err;

    out_ep = cfg->out_ep;
    if ((err = MXC_USB_ConfigEp(out_ep, MAXUSB_EP_TYPE_OUT, cfg->out_maxpacket)) != 0) {
        ccid_deconfigure();
        return err;
    }

    in_ep = cfg->in_ep;
    if ((err = MXC_USB_ConfigEp(in_ep, MAXUSB_EP_TYPE_IN, cfg->in_maxpacket)) != 0) {
        ccid_deconfigure();
        return err;
    }

    notify_ep = cfg->notify_ep;
    if ((err = MXC_USB_ConfigEp(notify_ep, MAXUSB_EP_TYPE_IN, cfg->notify_maxpacket)) != 0) {
        ccid_deconfigure();
        return err;
    }

    if (callback[CCID_CONFIGURED]) {
        callback[CCID_CONFIGURED](NULL);
    }

    /* Register an initial OUT request */
    return ccid_lodge_out(NULL);
}

/******************************************************************************/
int ccid_deconfigure(void)
{
    if (callback[CCID_DECONFIGURED]) {
        callback[CCID_DECONFIGURED](NULL);
    }

    /* deconfigure endpoints */
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

    /* Clear driver state */
    memset(&pr_req, 0, sizeof(MXC_USB_Req_t));
    memset(&rp_req, 0, sizeof(MXC_USB_Req_t));
    memset(&notify_req, 0, sizeof(MXC_USB_Req_t));

    return 0;
}

/******************************************************************************/
static int ccid_lodge_out(void *cbdata)
{
    received = 0;

    memset(&pr_req, 0, sizeof(MXC_USB_Req_t));
    pr_req.ep = out_ep;
    pr_req.data = xfer_buf;
    pr_req.reqlen = sizeof(xfer_buf);
    pr_req.callback = ccid_received;//ccid_dispatcher;
    pr_req.cbdata = NULL; /* Callback uses global data */
    pr_req.type = MAXUSB_TYPE_PKT;
    return MXC_USB_ReadEndpoint(&pr_req);
}

/******************************************************************************/
int ccid_register_callback(ccid_callback_t cbnum, int (*func)(MXC_USB_Req_t *))
{
    if (cbnum >= CCID_NUM_CALLBACKS) {
        return -1;
    }

    callback[cbnum] = func;
    return 0;
}

/******************************************************************************/
static int class_req(MXC_USB_SetupPkt *sud, void *cbdata)
{
    int result = -1;

    if ((((sud->bmRequestType & RT_RECIP_MASK) & RT_RECIP_IFACE) == 1) && (sud->wIndex == ccid_iface)) {
        switch (sud->bRequest) {
            case CCID_CONTROL_ABORT:
                result = 1;
            break;
            case CCID_CONTROL_GET_CLOCK_FREQUENCIES:
                /* Not supported, stall */
            break;
            case CCID_CONTROL_GET_DATA_RATES:
                /* Not supported, stall */
            break;
            default:
                /* Stall */
            break;
        }
    } else {
        /* Not destined for our interface, possibly pass through to other class */
        if (chain_class_req.fnaddr) {
            return chain_class_req.fnaddr(sud, chain_class_req.cbdata);
        }
    }

    return result;
}

/******************************************************************************/
static void ccid_received(void *cbdata) {
    received = 1;
}

int ccid_is_received() {
    return received;
}

void ccid_dispatcher(void *cbdata)
{
    int result = -1;

    /* Check for valid command header */
    if (pr_req.actlen < CCID_CMD_HDR_LEN) {
        /* Invalid length */
    } else {
        /* Parse command */
    switch (xfer_buf[0]) {
        case PC_to_RDR_IccPowerOn:
            if (callback[CCID_ICC_POWER_ON]) {
                result = callback[CCID_ICC_POWER_ON](&pr_req);
            }
        break;
        case PC_to_RDR_IccPowerOff:
            if (callback[CCID_ICC_POWER_OFF]) {
                result = callback[CCID_ICC_POWER_OFF](&pr_req);
            }
        break;
        case PC_to_RDR_GetSlotStatus:
            if (callback[CCID_GET_SLOT_STATUS]) {
                result = callback[CCID_GET_SLOT_STATUS](&pr_req);
            }
        break;
        case PC_to_RDR_XfrBlock:
            if (callback[CCID_XFR_BLOCK]) {
                result = callback[CCID_XFR_BLOCK](&pr_req);
            }
        break;
        case PC_to_RDR_GetParameters:
            if (callback[CCID_GET_PARAMETERS]) {
                result = callback[CCID_GET_PARAMETERS](&pr_req);
            }
        break;
        case PC_to_RDR_ResetParameters:
            if (callback[CCID_RESET_PARAMETERS]) {
                result = callback[CCID_RESET_PARAMETERS](&pr_req);
            }
        break;
        case PC_to_RDR_SetParameters:
            if (callback[CCID_SET_PARAMETERS]) {
                result = callback[CCID_SET_PARAMETERS](&pr_req);
            }
        break;
        case PC_to_RDR_Escape:
            if (callback[CCID_ESCAPE]) {
                result = callback[CCID_ESCAPE](&pr_req);
            }
        break;
        case PC_to_RDR_IccClock:
            if (callback[CCID_ICC_CLOCK]) {
                result = callback[CCID_ICC_CLOCK](&pr_req);
            }
        break;
        case PC_to_RDR_T0APDU:
            if (callback[CCID_T0APDU]) {
                result = callback[CCID_T0APDU](&pr_req);
            }
        break;
        case PC_to_RDR_Secure:
            if (callback[CCID_SECURE]) {
                result = callback[CCID_SECURE](&pr_req);
            }
        break;
        case PC_to_RDR_Mechanical:
            if (callback[CCID_MECHANICAL]) {
                result = callback[CCID_MECHANICAL](&pr_req);
            }
        break;
        case PC_to_RDR_Abort:
            if (callback[CCID_ABORT]) {
                result = callback[CCID_ABORT](&pr_req);
            }
        break;
        case PC_to_RDR_SetDataRateAndClockFrequency:
            if (callback[CCID_SET_DATA_RATE_AND_CLOCK_FREQUENCY]) {
                result = callback[CCID_SET_DATA_RATE_AND_CLOCK_FREQUENCY](&pr_req);
            }
        break;
        default:
            /* Unsupported, fail */
        break;
    }
    }
    if (result < 0) {
        /* Send back an interrupted partial response to indicate failure */
        memset(xfer_buf, 0, 5);
        xfer_buf[0] = RDR_to_PC_DataBlock;
        memset(xfer_buf+7, 0, 3);
        ccid_rdr_to_pc(xfer_buf, CCID_CMD_HDR_LEN);
    }
    received = 0;
}

/******************************************************************************/
int ccid_rdr_to_pc(uint8_t *buffer, unsigned int len)
{
    memset(&rp_req, 0, sizeof(MXC_USB_Req_t));
    rp_req.ep = in_ep;
    rp_req.data = buffer;
    rp_req.reqlen = len;
    rp_req.callback = (void (*)(void *))ccid_lodge_out; /* Return code ignored */
    rp_req.cbdata = NULL;
    rp_req.type = MAXUSB_TYPE_PKT;

    return MXC_USB_WriteEndpoint(&rp_req);
}

/******************************************************************************/
static void notify_complete(void *cbdata)
{
    /* Nothing here for now */
}

/******************************************************************************/
int ccid_notify(uint8_t *buffer, unsigned int len)
{
    memset(&notify_req, 0, sizeof(MXC_USB_Req_t));
    notify_req.ep = notify_ep;
    notify_req.data = buffer;
    notify_req.reqlen = len;
    notify_req.callback = notify_complete;
    notify_req.cbdata = NULL;
    notify_req.type = MAXUSB_TYPE_TRANS;
    return MXC_USB_WriteEndpoint(&notify_req);
}
