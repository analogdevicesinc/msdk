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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "maxqusb.h"
#include "usb_regs_maxq.h"
#include "usbio_maxq.h"
#include "usb.h"

typedef struct {
    volatile uint32_t buf0_desc;
    volatile uint32_t buf0_address;
    volatile uint32_t buf1_desc;
    volatile uint32_t buf1_address;
} ep_buffer_t;

typedef struct {
    ep_buffer_t out_buffer;
    ep_buffer_t in_buffer;
} ep0_buffer_t;

typedef struct {
    ep0_buffer_t ep0;
    ep_buffer_t ep[MXC_USB_NUM_EP - 1];
} ep_buffer_descriptor_t;

/* static storage for endpoint buffer descriptor table, must be 4 byte aligned on MAXQ */
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment = 4
#endif
ep_buffer_descriptor_t ep_buffer_descriptor;

/* storage for active endpoint data request objects */
static MXC_USB_Req_t *usb_request[MXC_USB_NUM_EP];

static int MXC_USB_SetBits(unsigned int reg, uint16_t bits)
{
    uint16_t x;

    if (usbio_readreg(reg, &x) < 0) {
        return -1;
    }

    return usbio_writereg(reg, x | bits);
}

static int MXC_USB_ClearBits(unsigned int reg, uint16_t bits)
{
    uint16_t x;

    if (usbio_readreg(reg, &x) < 0) {
        return -1;
    }

    return usbio_writereg(reg, x & ~bits);
}

/* Writes n bytes of data into the specified endpoint */
static int MXC_USB_WriteEpdata(unsigned int ep_num, uint8_t *data, uint16_t n)
{
    unsigned int reg;

    switch (ep_num) {
    case 0:
        reg = rEP0BUF;
        break;
    case 1:
        /* We don't allow writing of EP1 as it's an OUT endpoint */
        return -1;
    case 2:
        reg = rEP2BUF;
        break;
    case 3:
        reg = rEP3BUF;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        /* We don't allow writing of EP4 as it's an OUT endpoint */
        return -1;
    case 5:
        reg = rEP5BUF;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    return usbio_writefifo(reg, data, n);
}

/* Read n bytes of data from the specified endpoint */
int MXC_USB_ReadEpdata(unsigned int ep_num, uint8_t *data, uint16_t n)
{
    unsigned int reg;

    /* We don't allow reading from EP2/3 or EP4 (where available) */
    switch (ep_num) {
    case 0:
        reg = rEP0BUF;
        break;
    case 1:
        reg = rEP1BUF;
        break;
    case 2:
        return -1;
    case 3:
        return -1;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        reg = rEP4BUF;
        break;
    case 5:
        return -1;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    return usbio_readfifo(reg, data, n);
}

/* Load byte count register to arm endpoint for transmission */
static int MXC_USB_SetEpbytes(unsigned int ep_num, uint8_t n)
{
    unsigned int reg;

    switch (ep_num) {
    case 0:
        reg = rEP0BC;
        break;
    case 1:
        /* We don't allow writing of EP1 as it's an OUT endpoint */
        return -1;
    case 2:
        reg = rEP2BC;
        break;
    case 3:
        reg = rEP3BC;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        /* We don't allow writing of EP4 as it's an OUT endpoint */
        return -1;
    case 5:
        reg = rEP5BC;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    if (usbio_writereg(reg, n) < 0) {
        return -1;
    }

    return 0;
}

/* Get # bytes available */
static int MXC_USB_GetEpbytes(unsigned int ep_num)
{
    unsigned int reg;
    uint16_t x;

    /* Does it make sense to allow reading of the IN endpoint byte counts? */
    switch (ep_num) {
    case 0:
        reg = rEP0BC;
        break;
    case 1:
        reg = rEP1BC;
        break;
    case 2:
        reg = rEP2BC;
        break;
    case 3:
        reg = rEP3BC;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        reg = rEP4BC;
        break;
    case 5:
        reg = rEP5BC;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    if (usbio_readreg(reg, &x) < 0) {
        return -1;
    }

    return x;
}

/* disable enpoints both hardware and interrupts */
static int MXC_USB_EpDisable(unsigned int ep)
{
    int result = -1;

    switch (ep) {
    case 0:
        result = MXC_USB_ClearBits(rEPIEN, bmIN0BAVIE | bmOUT0DAVIE);
        break;
    case 1:
        result = MXC_USB_SetBits(rEPCTG, bmEP1DIS);
        if (result == 0)
            result = MXC_USB_ClearBits(rEPIEN, bmOUT1DAVIE);
        break;
    case 2:
        result = MXC_USB_SetBits(rEPCTG, bmEP2DIS);
        if (result == 0)
            result = MXC_USB_ClearBits(rEPIEN, bmIN2BAVIE);
        break;
    case 3:
        result = MXC_USB_SetBits(rEPCTG, bmEP3DIS);
        if (result == 0)
            result = MXC_USB_ClearBits(rEPIEN, bmIN3BAVIE);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = MXC_USB_SetBits(rEPCTG, bmEP4DIS);
        if (result == 0)
            result = MXC_USB_ClearBits(rEPIEN, bmOUT4DAVIE);
        break;
    case 5:
        result = MXC_USB_SetBits(rEPCTG, bmEP5DIS);
        if (result == 0)
            result = MXC_USB_ClearBits(rEPIEN, bmIN5BAVIE);
        break;
#endif
    default:
        break;
    }

    return result;
}

/* enable enpoints both hardware and interrupts */
static int MXC_USB_EpEnable(unsigned int ep)
{
    int result = -1;
    switch (ep) {
    case 0:
        result = MXC_USB_SetBits(rEPIEN, bmIN0BAVIE | bmOUT0DAVIE);
        break;
    case 1:
        result = MXC_USB_ClearBits(rEPCTG, bmEP1DIS);
        break;
    case 2:
        result = MXC_USB_ClearBits(rEPCTG, bmEP2DIS);
        break;
    case 3:
        result = MXC_USB_ClearBits(rEPCTG, bmEP3DIS);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = MXC_USB_ClearBits(rEPCTG, bmEP4DIS);
        break;
    case 5:
        result = MXC_USB_ClearBits(rEPCTG, bmEP5DIS);
        break;
#endif
    default:
        break;
    }

    return result;
}

/* Verify ep direction is correct */
static int MXC_USB_EpDirCheck(unsigned int ep, maxusb_ep_type_t type)
{
    switch (ep) {
    case 0:
        if (type != MAXUSB_EP_TYPE_CONTROL)
            return -1;
        break;
    case 1:
        if (type != MAXUSB_EP_TYPE_OUT)
            return -1;
        break;
    case 2:
        if (type != MAXUSB_EP_TYPE_IN)
            return -1;
        break;
    case 3:
        if (type != MAXUSB_EP_TYPE_IN)
            return -1;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        if (type != MAXUSB_EP_TYPE_OUT)
            return -1;
        break;
    case 5:
        if (type != MAXUSB_EP_TYPE_IN)
            return -1;
        break;
#endif
    default:
        return -1;
        break;
    }
}

/* Query the buffer available flag for IN endpoints */
/* Returns 1 if available, 0 if not, -1 on error */
int MXC_USB_IsBav(unsigned int ep_num)
{
    int result;
    uint16_t epint;

    if (usbio_readreg(rEPINT, &epint) < 0) {
        return -1;
    }

    result = 0;
    switch (ep_num) {
    case 0:
        result = ((epint & bmIN0BAV) != 0);
        break;
    case 2:
        result = ((epint & bmIN2BAV) != 0);
        break;
    case 3:
        result = ((epint & bmIN3BAV) != 0);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 5:
        result = ((epint & bmIN5BAV) != 0);
        break;
#endif
    default:
        result = -1;
        break;
    }

    return result;
}

/* Query the data available flag for OUT endpoints */
/* Returns 1 if available, 0 if not, -1 on error */
int MXC_USB_IsDav(unsigned int ep_num)
{
    int result;
    uint16_t epint;

    if (usbio_readreg(rEPINT, &epint) < 0) {
        return -1;
    }

    result = 0;
    switch (ep_num) {
    case 0:
        result = ((epint & bmOUT0DAV) != 0);
        break;
    case 1:
        result = ((epint & bmOUT1DAV) != 0);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = ((epint & bmOUT4DAV) != 0);
        break;
#endif
    default:
        result = -1;
        break;
    }

    return result;
}

void MXC_USB_HardReset(void)
{
    usbio_blind_writereg(rUSBCN, 0);
    usbio_blind_writereg(rUSBCN, bmURST);
    usbio_blind_writereg(rUSBCN, 0);
}

int MXC_USB_Init(maxusb_cfg_options_t *options)
{
    int i;

    for (i = 0; i < MXC_USB_NUM_EP; i++) {
        usb_request[i] = NULL;
    }

    // Reset USB block in case it's in a bad state
    // In some cases the USB block be stuck with UBUSY set and it
    // cannot be cleared.  This will clear it.
    MXC_USB_HardReset();

    /* Force USB IP to shutdown, per older driver */
    i = MXC_USB_Shutdown();
    if (i < 0) {
        return i;
    }

    /* Enable USB controller */
    i = MXC_USB_SetBits(rUSBCFG, bmUSBEN);
    if (i < 0) {
        return i;
    }

    /* Remove power down bit and reset */
    i = MXC_USB_ClearBits(rUSBCN, bmPWRDN | bmURST);
    if (i < 0) {
        return i;
    }

    // Ensure enpoints are disabled when we start running
    // If usb is not enabled, or is powered down this will cause lock up (UBUSY stuck)
    for (i = 0; i < MXC_USB_NUM_EP; i++) {
        if (MXC_USB_EpDisable(i) != 0)
            return -1;
    }

    return 0;
}

int MXC_USB_Shutdown(void)
{
    int x;

    /* Set all configuration bits to POR values */
    x = usbio_writereg(rUSBCN, bmOSCST | bmURST | bmPWRDN);
    if (x < 0) {
        return x;
    }
    x = usbio_writereg(rUSBCFG, 0x00);
    if (x < 0) {
        return x;
    }

    return x;
}

int MXC_USB_Connect(void)
{
    int x;

    /* allow interrupts on ep0 */
    x = MXC_USB_SetBits(rEPIEN, bmIN0BAVIE | bmOUT0DAVIE);
    if (x < 0) {
        return x;
    }

    /* Connect to bus */
    x = MXC_USB_SetBits(rUSBCN, bmCONNECT | bmVBGATE);

    return x;
}

int MXC_USB_Disconnect(void)
{
    /* Disconnect to bus */
    return MXC_USB_ClearBits(rUSBCN, bmCONNECT | bmVBGATE);
}

/**
 * NOTE: Maxq USB hardware has dedicated endpoint, that cannot be changed i.e.
 * 0 is always control, some are always in and some are out.
 * Size is always 64 as well.
 *
 * This function will be used instead to verify that correct type is selected.
 * But will also support enable and disable of endpoints.
 *
 * Switch statements are messy but, HW register bit patterns are too.
 */
int MXC_USB_ConfigEp(unsigned int ep, maxusb_ep_type_t type, unsigned int size)
{
    int result;

    if (ep >= MXC_USB_NUM_EP) {
        return -1;
    }

    if (size > MXC_USB_MAX_PACKET) {
        return -1;
    }

    if (MXC_USB_EpDirCheck(ep, type) == -1)
        return -1;

    // regardless of passed in size, on MAXQ architecture we only have size 64
    size = MXC_USB_MAXQ_EP_SIZE;

    // Hardware disable.
    // EP0 can't be HW diabled, but interrupts can be.
    // Also Interrupt enable disble
    if (type == MAXUSB_EP_TYPE_DISABLED)
        result = MXC_USB_EpDisable(ep);
    else
        result = MXC_USB_EpEnable(ep);

    return 0;
}

int MXC_USB_IsConfigured(unsigned int ep)
{
    uint16_t x;
    int result;

    if (usbio_readreg(rEPCTG, &x) < 0) {
        return -1;
    }

    // End Point is disabled if bit is a 1
    switch (ep) {
    case 0:
        return 1; // End Point 0 is always enabled
        break;
    case 1:
        result = (x & bmEP1DIS) != bmEP1DIS;
        break;
    case 2:
        result = (x & bmEP2DIS) != bmEP2DIS;
        break;
    case 3:
        result = (x & bmEP3DIS) != bmEP3DIS;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = (x & bmEP4DIS) != bmEP4DIS;
        break;
    case 5:
        result = (x & bmEP5DIS) != bmEP5DIS;
        break;
#endif
    default:
        result = -1;
        break;
    }

    return result;
}

int MXC_USB_Stall(unsigned int ep)
{
    uint16_t x;
    MXC_USB_Req_t *req;

    /* Endpoint 0 is a special case, we need to set 3 stall bits */
    switch (ep) {
    case 0:
        x = bmSTLIN0 | bmSTLOUT0 | bmSTLSTAT;
        break;
    case 1:
        x = bmSTLEP1;
        break;
    case 2:
        x = bmSTLEP2;
        break;
    case 3:
        x = bmSTLEP3;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        x = bmSTLEP4;
        break;
    case 5:
        x = bmSTLEP5;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    if (MXC_USB_SetBits(rEPSTL, x) != 0)
        return -1;

    /* clear pending requests */
    req = usb_request[ep];
    usb_request[ep] = NULL;

    if (req) {
        /* complete pending requests with error */
        req->error_code = -1;
        if (req->callback) {
            req->callback(req->cbdata);
        }
    }

    return 0;
}

/* Clear data toggle on endpoint */
int MXC_USB_ClearToggle(unsigned int ep)
{
    int result;

    /* Data toggle on endpoint 0 is handled by controller hardware */
    switch (ep) {
    case 1:
        result = MXC_USB_SetBits(rEPCTG, bmCTGEP1);
        break;
    case 2:
        result = MXC_USB_SetBits(rEPCTG, bmCTGEP2);
        break;
    case 3:
        result = MXC_USB_SetBits(rEPCTG, bmCTGEP3);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = MXC_USB_SetBits(rEPCTG, bmCTGEP4);
        break;
    case 5:
        result = MXC_USB_SetBits(rEPCTG, bmCTGEP5);
        break;
#endif
    default:
        result = -1;
        break;
    }

    return result;
}

int MXC_USB_Unstall(unsigned int ep)
{
    uint16_t x;

    /* clear the data toggle */
    if (MXC_USB_ClearToggle(ep) != 0)
        return -1;

    /* Endpoint 0 is a special case, we need to clear 3 stall bits */
    switch (ep) {
    case 0:
        x = bmSTLIN0 | bmSTLOUT0 | bmSTLSTAT;
        break;
    case 1:
        x = bmSTLEP1;
        break;
    case 2:
        x = bmSTLEP2;
        break;
    case 3:
        x = bmSTLEP3;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        x = bmSTLEP4;
        break;
    case 5:
        x = bmSTLEP5;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    return MXC_USB_ClearBits(rEPSTL, x);
}

int MXC_USB_IsStalled(unsigned int ep)
{
    uint16_t x, regval;

    /* Endpoint 0 is a special case, we need to set 3 stall bits */
    switch (ep) {
    case 0:
        x = bmSTLIN0 | bmSTLOUT0 | bmSTLSTAT;
        break;
    case 1:
        x = bmSTLEP1;
        break;
    case 2:
        x = bmSTLEP2;
        break;
    case 3:
        x = bmSTLEP3;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        x = bmSTLEP4;
        break;
    case 5:
        x = bmSTLEP5;
        break;
#endif
    default:
        /* Bail, as we weren't supposed to get here */
        return -1;
    }

    if (usbio_readreg(rEPSTL, &regval) < 0) {
        return -1;
    }

    if (regval & x) {
        /* Endpoint stalled */
        return 1;
    }

    return 0;
}

int MXC_USB_ResetEp(unsigned int ep)
{
    MXC_USB_Req_t *req;

    if (ep >= MXC_USB_NUM_EP) {
        return -1;
    }

    /* clear pending requests */
    req = usb_request[ep];
    usb_request[ep] = NULL;

    MXC_USB_EpDisable(ep);
    MXC_USB_ClearToggle(ep);

    if (req) {
        /* complete pending requests with error */
        req->error_code = -1;
        if (req->callback) {
            req->callback(req->cbdata);
        }
    }

    return 0;
}

int MXC_USB_Ackstat(unsigned int ep)
{
    // Only supported on end point 0 in MAXQ
    if (ep != 0)
        return -1;

    return MXC_USB_SetBits(rEPSTL, bmACKSTAT);
}

/* sent packet done handler*/
static void event_in_data(uint32_t irqs)
{
    uint32_t epnum, buffer_bit, data_left;
    MXC_USB_Req_t *req;
    ep_buffer_t *buf_desc;

    /* Loop for each data endpoint */
    // NOTE: this code assumes, that EPIEN, and EPINT registers have the same layout
    for (epnum = 0; epnum < MXC_USB_NUM_EP; epnum++) {
        switch (epnum) {
        case 0:
            buffer_bit = bmIN0BAV;
            break;
        case 2:
            buffer_bit = bmIN2BAV;
            break;
        case 3:
            buffer_bit = bmIN3BAV;
            break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
        case 5:
            buffer_bit = bmIN5BAV;
            break;
#endif
        default:
            buffer_bit = 0;
            break;
        }

        if ((irqs & buffer_bit) == 0) { /* Not set, next Endpoint */
            continue;
        }

        /* not sure how this could happen, safe anyway */
        if (!usb_request[epnum]) {
            continue;
        }

        req = usb_request[epnum];
        data_left = req->reqlen - req->actlen;

        // NOTE: using buffer descriptor mapping, but there is no hardware
        // requirement for this on MAXQ, simpler to maintain compatibility
        if (epnum == 0) {
            buf_desc = &ep_buffer_descriptor.ep0.in_buffer;
        } else {
            buf_desc = &ep_buffer_descriptor.ep[epnum - 1];
        }

        if (buf_desc->buf0_desc == 0) {
            /* free request first, the callback may re-issue a request */
            usb_request[epnum] = NULL;

            // MAXQHW Behavior: Clearing flag, causes retransmit
            // Disable the IN IRQ here, to avoid HW sending empty packets.
            MXC_USB_ClearBits(rEPIEN, buffer_bit);

            /* must have sent the ZLP, mark done */
            if (req->callback) {
                req->callback(req->cbdata);
            }
            continue;
        }

        if (data_left) { /* more data to send */
            if (data_left >= MXC_USB_MAXQ_EP_SIZE) {
                buf_desc->buf0_desc = MXC_USB_MAXQ_EP_SIZE;
            } else {
                buf_desc->buf0_desc = data_left;
            }

            req->actlen += buf_desc->buf0_desc;

            /* update the pointer */
            buf_desc->buf0_address += MXC_USB_MAXQ_EP_SIZE;

            // MAXQ has to use program io to load buffer, ARM uses DMA
            // Since this is interrupt context, no where to report any erorrs
            // of these functions, also, since this is embedded IP should
            // always work.
            MXC_USB_WriteEpdata(epnum, ((uint8_t *)buf_desc->buf0_address), buf_desc->buf0_desc);
            MXC_USB_SetEpbytes(epnum, buf_desc->buf0_desc);
        } else {
            /* all done sending data, either send ZLP if required or done here */
            if ((req->type == MAXUSB_TYPE_TRANS) &&
                ((req->reqlen & (MXC_USB_MAXQ_EP_SIZE - 1)) == 0)) {
                /* send ZLP per spec, last packet was full sized and nothing left to send */
                buf_desc->buf0_desc = 0;

                MXC_USB_SetEpbytes(epnum, buf_desc->buf0_desc);
            } else {
                /* free request */
                usb_request[epnum] = NULL;

                // MAXQHW Behavior: Clearing flag causes retransmit
                // Disable the IN IRQ here, to avoid HW sending empty packets.
                MXC_USB_ClearBits(rEPIEN, buffer_bit);

                /* set done return value */
                if (req->callback) {
                    req->callback(req->cbdata);
                }
            }
        }
    }
}

/* received packet */
static void event_out_data(uint32_t irqs)
{
    uint32_t epnum, buffer_bit, reqsize, rxsize;
    MXC_USB_Req_t *req;
    ep_buffer_t *buf_desc;

    /* Loop for each data endpoint */
    for (epnum = 0; epnum < MXC_USB_NUM_EP; epnum++) {
        switch (epnum) {
        case 0:
            buffer_bit = bmOUT0DAV;
            break;
        case 1:
            buffer_bit = bmOUT1DAV;
            break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
        case 4:
            buffer_bit = bmOUT4DAV;
            break;
#endif
        default:
            buffer_bit = 0;
            break;
        }

        if ((irqs & buffer_bit) == 0) {
            continue;
        }

        /* this can happen if the callback was called then ZLP received */
        if (!usb_request[epnum]) {
            continue; /* ignored, because callback must have been called */
        }

        if (epnum == 0) {
            buf_desc = &ep_buffer_descriptor.ep0.out_buffer;
        } else {
            buf_desc = &ep_buffer_descriptor.ep[epnum - 1];
        }

        req = usb_request[epnum];

        /* what was the last request size? */
        if ((req->reqlen - req->actlen) >= MXC_USB_MAXQ_EP_SIZE) {
            reqsize = MXC_USB_MAXQ_EP_SIZE;
        } else {
            reqsize = (req->reqlen - req->actlen);
        }

        // How much is available to read?
        buf_desc->buf0_desc = MXC_USB_GetEpbytes(epnum);

        /* the actual size of data written to buffer will be the lesser of the packet size and the requested size */
        if (reqsize < buf_desc->buf0_desc) {
            rxsize = reqsize;
        } else {
            rxsize = buf_desc->buf0_desc;
        }

        /* less than a full packet or zero length packet)  */
        if ((req->type == MAXUSB_TYPE_PKT) || (rxsize < MXC_USB_MAXQ_EP_SIZE) || (rxsize == 0)) {
            /* free request */
            usb_request[epnum] = NULL;

            // MAXQ must read via program io, not DMA
            MXC_USB_ReadEpdata(epnum, &req->data[req->actlen], rxsize);
            req->actlen += rxsize;

            /* call it done */
            if (req->callback) {
                req->callback(req->cbdata);
            }
        } else {
            /* not done yet, push pointers, update descriptor */
            buf_desc->buf0_address += MXC_USB_MAXQ_EP_SIZE;

            /* don't overflow */
            // NOTE: using buf0_desc to be size, or rxsize
            if ((req->reqlen - req->actlen) >= MXC_USB_MAXQ_EP_SIZE) {
                buf_desc->buf0_desc = MXC_USB_MAXQ_EP_SIZE;
            } else {
                buf_desc->buf0_desc = (req->reqlen - req->actlen);
            }

            // MAXQ must read via program io, not DMA
            MXC_USB_ReadEpdata(epnum, &req->data[req->actlen], buf_desc->buf0_desc);
            req->actlen += buf_desc->buf0_desc;
        }
    }
}

static void handle_brst(void)
{
    int i;

    /* kill any pending requests */
    for (i = 0; i < MXC_USB_NUM_EP; i++) {
        MXC_USB_ResetEp(i);
    }
}

void MXC_USB_IrqHandler(maxusb_usbio_events_t *evt)
{
    uint16_t usbint, usbien, epint, epien, in_irqs, out_irqs;

    // Clear all flags
    *((unsigned int *)evt) = 0;

    // Read and clear interrupts
    // Mask off interrupts that are not enabled
    usbio_readreg(rUSBINT, &usbint);
    usbio_readreg(rUSBIEN, &usbien);
    usbint &= usbien;
    if (usbint) {
        usbio_writereg(rUSBINT, usbint);
    }

    // Read EP interrupts
    // Mask off interrupts that are not enabled
    usbio_readreg(rEPINT, &epint);
    usbio_readreg(rEPIEN, &epien);
    epint &= epien;

    // Handle USB interrupts
    if (usbint & bmBRSTDN) {
        evt->brstdn = 1;
    }
    if (usbint & bmVBUS) {
        evt->vbus = 1;
    }
    if (usbint & bmNOVBUS) {
        evt->novbus = 1;
    }
    if (usbint & bmSUSP) {
        evt->susp = 1;
    }
    if (usbint & bmBRST) {
        handle_brst();
        evt->brst = 1;
    }
    if (usbint & bmBACT) {
        evt->bact = 1;
    }
    if (usbint & bmRWUDN) {
        evt->rwudn = 1;
    }
    if (usbint & bmDPACT) {
        evt->dpact = 1;
    }

    if (epint & bmSUDAV) {
        evt->sudav = 1;
        usbio_writereg(rEPINT, bmSUDAV);
    }

#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    in_irqs = epint & (bmIN0BAV | bmIN2BAV | bmIN3BAV | bmIN5BAV);
    out_irqs = epint & (bmOUT0DAV | bmOUT1DAV | bmOUT4DAV);
#else
    in_irqs = epint & (bmIN0BAV | bmIN2BAV | bmIN3BAV);
    out_irqs = epint & (bmOUT0DAV | bmOUT1DAV);
#endif

    // Work around MAXQHW Behavior: clearing int flag causes retransmits
    // CLEAR enable bit in event_in_data, IF all done.
    // Write_ep will SET enable bit.
    if (in_irqs) {
        event_in_data(in_irqs);
    }
    if (out_irqs) {
        event_out_data(out_irqs);
        usbio_writereg(rEPINT, out_irqs);
    }
}

int MXC_USB_IrqEnable(maxusb_event_t event)
{
    int result = -1;
    switch (event) {
    case MAXUSB_EVENT_DPACT:
        result = MXC_USB_SetBits(rUSBIEN, bmDPACT);
        break;
    case MAXUSB_EVENT_RWUDN:
        result = MXC_USB_SetBits(rUSBIEN, bmRWUDN);
        break;
    case MAXUSB_EVENT_BACT:
        result = MXC_USB_SetBits(rUSBIEN, bmBACT);
        break;
    case MAXUSB_EVENT_BRST:
        result = MXC_USB_SetBits(rUSBIEN, bmBRST);
        break;
    case MAXUSB_EVENT_SUSP:
        result = MXC_USB_SetBits(rUSBIEN, bmSUSP);
        break;
    case MAXUSB_EVENT_NOVBUS:
        result = MXC_USB_SetBits(rUSBIEN, bmNOVBUS);
        break;
    case MAXUSB_EVENT_VBUS:
        result = MXC_USB_SetBits(rUSBIEN, bmVBUS);
        break;
    case MAXUSB_EVENT_BRSTDN:
        result = MXC_USB_SetBits(rUSBIEN, bmBRSTDN);
        break;
    case MAXUSB_EVENT_SUDAV:
        result = MXC_USB_SetBits(rEPIEN, bmSUDAV);
        break;
    default:
        result = -1;
        break;
    }

    return result;
}

int MXC_USB_IrqDisable(maxusb_event_t event)
{
    int result = -1;
    switch (event) {
    case MAXUSB_EVENT_DPACT:
        result = MXC_USB_ClearBits(rUSBIEN, bmDPACT);
        break;
    case MAXUSB_EVENT_RWUDN:
        result = MXC_USB_ClearBits(rUSBIEN, bmRWUDN);
        break;
    case MAXUSB_EVENT_BACT:
        result = MXC_USB_ClearBits(rUSBIEN, bmBACT);
        break;
    case MAXUSB_EVENT_BRST:
        result = MXC_USB_ClearBits(rUSBIEN, bmBRST);
        break;
    case MAXUSB_EVENT_SUSP:
        result = MXC_USB_ClearBits(rUSBIEN, bmSUSP);
        break;
    case MAXUSB_EVENT_NOVBUS:
        result = MXC_USB_ClearBits(rUSBIEN, bmNOVBUS);
        break;
    case MAXUSB_EVENT_VBUS:
        result = MXC_USB_ClearBits(rUSBIEN, bmVBUS);
        break;
    case MAXUSB_EVENT_BRSTDN:
        result = MXC_USB_ClearBits(rUSBIEN, bmBRSTDN);
        break;
    case MAXUSB_EVENT_SUDAV:
        result = MXC_USB_ClearBits(rEPIEN, bmSUDAV);
        break;
    default:
        result = -1;
        break;
    }

    return result;
}

int MXC_USB_GetSetup(MXC_USB_SetupPkt *sud)
{
    uint8_t b[8];

    usbio_readfifo(rSUDBUF, b, 8);

    /* Fill in structure */
    sud->bmRequestType = b[SETUP_bmRequestType];
    sud->bRequest = b[SETUP_bRequest];
    sud->wValue = (b[SETUP_wValueH] << 8) + b[SETUP_wValueL];
    sud->wIndex = (b[SETUP_wIndexH] << 8) + b[SETUP_wIndexL];
    sud->wLength = (b[SETUP_wLengthH] << 8) + b[SETUP_wLengthL];

    return 0;
}

int MXC_USB_WriteEndpoint(MXC_USB_Req_t *req)
{
    int result;
    unsigned int ep = req->ep;
    uint8_t *data = req->data;
    unsigned int len = req->reqlen;
    ep_buffer_t *buf_desc;
    uint32_t buffer_bit;
    uint16_t epint;

    if (ep >= MXC_USB_NUM_EP) {
        return -1;
    }

    // Verify this is a valid ep to write
    // And get interrupt enable bit
    switch (ep) {
    case 0:
        buffer_bit = bmIN0BAVIE;
        break;
    case 1:
        return -1;
        break;
    case 2:
        buffer_bit = bmIN2BAVIE;
        break;
    case 3:
        buffer_bit = bmIN3BAVIE;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        return -1;
        break;
    case 5:
        buffer_bit = bmIN5BAVIE;
        break;
#endif
    default:
        return -1;
        break;
    }

    /* EP must be enabled (configured) */
    if (MXC_USB_IsConfigured(ep) <= 0)
        return -1;

    if (usb_request[ep] != NULL) {
        return -1;
    }

    if (!MXC_USB_IsBav(ep)) {
        return -1;
    }

    /* assign req object */
    usb_request[ep] = req;

    /* clear errors */
    req->error_code = 0;

    req->actlen = 0;

    if (ep == 0) {
        buf_desc = &ep_buffer_descriptor.ep0.in_buffer;
    } else {
        buf_desc = &ep_buffer_descriptor.ep[ep - 1];
    }

    if (len > MXC_USB_MAXQ_EP_SIZE) {
        buf_desc->buf0_desc = MXC_USB_MAXQ_EP_SIZE;
        usb_request[ep]->actlen = MXC_USB_MAXQ_EP_SIZE;
    } else {
        buf_desc->buf0_desc = len;
        usb_request[ep]->actlen = len;
    }

    /* set pointer, force single buffered */
    buf_desc->buf0_address = (uint32_t)data;

    // Get the interrupt register, required for event_int_data
    if (usbio_readreg(rEPINT, &epint) < 0) {
        return -1;
    }

    result = MXC_USB_WriteEpdata(ep, ((uint8_t *)buf_desc->buf0_address), buf_desc->buf0_desc);

    if (result != 0)
        return result;

    result = MXC_USB_SetEpbytes(ep, buf_desc->buf0_desc);

    if (result != 0)
        return result;

    // MAXQHW Behavior: clearing int flag causes retransmits
    // ENABLE the IN IRQ here, to allow processing.
    MXC_USB_SetBits(rEPIEN, buffer_bit);

    return 0;
}

int MXC_USB_ReadEndpoint(MXC_USB_Req_t *req)
{
    unsigned int ep = req->ep;
    ep_buffer_t *buf_desc;
    int result;

    if (ep >= MXC_USB_NUM_EP) {
        return -1;
    }

    // Verify this is a valid ep to read
    switch (ep) {
    case 0:
    case 1:
        break;
    case 2:
    case 3:
        return -1;
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        break;
    case 5:
        return -1;
        break;
#endif
    default:
        return -1;
        break;
    }

    /* EP must be enabled (configured) */
    if (MXC_USB_IsConfigured(ep) <= 0)
        return -1;

    /* And not stalled */
    if (MXC_USB_IsStalled(ep)) {
        return -1;
    }

    if (ep == 0) {
        buf_desc = &ep_buffer_descriptor.ep0.out_buffer;
    } else {
        buf_desc = &ep_buffer_descriptor.ep[ep - 1];
    }

    if (usb_request[ep] != NULL) {
        return -1;
    }

    /* assign the req object */
    usb_request[ep] = req;

    /* clear errors */
    req->error_code = 0;

    /* reset length */
    req->actlen = 0;

    // Enable the interrupt to process further.
    // Now only enabled here.
    switch (ep) {
    case 0:
        result = MXC_USB_SetBits(rEPIEN, bmOUT0DAVIE);
        break;
    case 1:
        result = MXC_USB_SetBits(rEPIEN, bmOUT1DAVIE);
        break;
#if (__TARGET_PROCESSOR == MAXQ1010) || (__TARGET_PROCESSOR == MAXQ1050)
    case 4:
        result = MXC_USB_SetBits(rEPIEN, bmOUT4DAVIE);
        break;
#endif
    default:
        return -1;
        break;
    }

    return result;
}
