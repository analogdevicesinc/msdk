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

#include <stdio.h>
#include <stdint.h>
#include "MAXQ1050.h"
#include "usbio_maxq.h"

/* Number of while() loop iterations before we time out on a read/write */
unsigned int param_tmo = 32;

/* Writes a value into the UADDR/UDATA indirect register interface */
/* Returns -1 on UADDR.BUSY=1 timeout, 0 otherwise */
int usbio_writereg(unsigned int reg, uint16_t data)
{
    unsigned int tmo = param_tmo;

    UADDR = reg;
    UDATA = data;

    while ((UADDR & 0x40) && --tmo) {}

    if (!tmo) {
        /* Timeout waiting for busy to clear */
        return -1;
    }

    return 0;
}

/** Writes a value into the UADDR/UDATA indirect register interface
 * Doesn't check UBUSY
 */
void usbio_blind_writereg(unsigned int reg, uint16_t data)
{
    UADDR = reg;
    UDATA = data;
}

/* Reads a value from the UADDR/UDATA indirect register interface */
/* Returns -1 on UADDR.BUSY=1 timeout, 0 otherwise */
int usbio_readreg(unsigned int reg, uint16_t *data)
{
    unsigned int tmo = param_tmo;

    UADDR = 0x80 | reg;
    while ((UADDR & 0x40) && --tmo) {}
    if (!tmo) {
        /* Timeout waiting for busy to clear */
        return -1;
    }

    *data = UDATA;

    return 0;
}

/* Reads num bytes from the UADDR/UDATA indirect register interface */
/* Returns -1 on UADDR.BUSY=1 timeout, 0 otherwise */
int usbio_readfifo(unsigned int reg, uint8_t *data, unsigned int num)
{
    int ret;
    unsigned int tmo = param_tmo;

    while ((UADDR & 0x40) && --tmo) {}
    if (!tmo) {
        /* Timeout waiting for busy to clear */
        return -1;
    }

    UADDR = 0x80 | reg;
    while (num--) {
        while ((UADDR & 0x40) && --tmo) {}
        if (!tmo) {
            /* Timeout waiting for busy to clear */
            return -1;
        }
        *data = UDATA;
        data++;
    }
    ret = 0;

    return ret;
}

/* Writes num bytes to the UADDR/UDATA indirect register interface */
/* Returns -1 on UADDR.BUSY=1 timeout, 0 otherwise */
int usbio_writefifo(unsigned int reg, uint8_t *data, unsigned int num)
{
    int ret;
    unsigned int tmo = param_tmo;

    while ((UADDR & 0x40) && --tmo) {}
    if (!tmo) {
        /* Timeout waiting for busy to clear */
        return -1;
    }

    UADDR = reg;
    while (num--) {
        UDATA = *data;
        data++;
        while ((UADDR & 0x40) && --tmo) {}
        if (!tmo) {
            /* Timeout waiting for busy to clear */
            return -1;
        }
    }
    ret = 0;

    return ret;
}
