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
#include <stdlib.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sema_reva.h"

/* Semaphores used for each mailbox */
#ifndef SEMA_BOX0_SEMA
#define SEMA_BOX0_SEMA (MXC_CFG_SEMA_INSTANCES - 2)
#endif

#ifndef SEMA_BOX1_SEMA
#define SEMA_BOX1_SEMA (MXC_CFG_SEMA_INSTANCES - 1)
#endif

#ifndef MAILBOX_SIZE
#define MAILBOX_SIZE 0
#elif ((MAILBOX_SIZE != 0) && (MAILBOX_SIZE < 16))
#error Mailbox size must be at least 16
#endif

/* Size of the payload in the mailbox */
#define MAILBOX_OVERHEAD (2 * sizeof(uint16_t))
#define MAILBOX_PAYLOAD_LEN (MAILBOX_SIZE - MAILBOX_OVERHEAD)

#ifdef __riscv
/* RISCV reads from mailbox 0 and writes to mailbox 1 */
#define SEMA_READ_BOX 0
#define SEMA_WRITE_BOX 1
#define SEMA_READ_SEMA SEMA_BOX0_SEMA
#define SEMA_WRITE_SEMA SEMA_BOX1_SEMA
#else
/* ARM reads from mailbox 1 and writes to mailbox 0 */
#define SEMA_READ_BOX 1
#define SEMA_WRITE_BOX 0
#define SEMA_READ_SEMA SEMA_BOX1_SEMA
#define SEMA_WRITE_SEMA SEMA_BOX0_SEMA
#endif

/* 
    Pointers to the mailbox locations in memory, ARM and RISCV must have matching
    views of the mailboxes. 
*/
extern uint32_t _mailbox_0;
extern uint32_t _mailbox_1;

/*  
    Mailbox Control block.

    Write location indicates where the next write will start.
    Read location indicates where next read will start. 
    write pointer == read pointer indicate empty buffer 
*/
typedef struct {
    uint16_t readLocation;
    uint16_t writeLocation;
#if (MAILBOX_SIZE == 0)
    uint8_t payload[1];
#else
    uint8_t payload[MAILBOX_SIZE - MAILBOX_OVERHEAD];
#endif
} mxcSemaBox_t;
mxcSemaBox_t *mxcSemaBox0 = (mxcSemaBox_t *)&_mailbox_0;
mxcSemaBox_t *mxcSemaBox1 = (mxcSemaBox_t *)&_mailbox_1;

/* Semaphore control block. */
typedef struct {
    uint8_t *readBuf;
    unsigned readLen;
    mxc_sema_complete_cb_t readCb;

    uint8_t *writeBuf;
    unsigned writeLen;
    mxc_sema_complete_cb_t writeCb;
} mxcSemaCb_t;
mxcSemaCb_t mxcSemaCb;

int MXC_SEMA_RevA_GetSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema)
{
    uint32_t sema_val;
    MXC_ASSERT(sema < MXC_CFG_SEMA_INSTANCES);

    // Reading the register does an atomic test and set, returns previous value
    sema_val = sema_regs->semaphores[sema];

    if (sema_val == 0) {
        return E_NO_ERROR;
    } else {
        return E_BUSY;
    }
}

int MXC_SEMA_RevA_CheckSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema)
{
    MXC_ASSERT(sema < MXC_CFG_SEMA_INSTANCES);

    if (sema_regs->status & (0x1 << sema)) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

uint32_t MXC_SEMA_RevA_Status(mxc_sema_reva_regs_t *sema_regs)
{
    return sema_regs->status;
}

void MXC_SEMA_RevA_FreeSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema)
{
    MXC_ASSERT(sema < MXC_CFG_SEMA_INSTANCES);

    sema_regs->semaphores[sema] = 0x0;
}

int MXC_SEMA_RevA_Init(mxc_sema_reva_regs_t *sema_regs)
{
    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Reset the async state */
        mxcSemaCb.readBuf = NULL;
        mxcSemaCb.readLen = 0;
        mxcSemaCb.writeBuf = NULL;
        mxcSemaCb.writeLen = 0;

        /* Enable the semaphore interrupt */
#ifndef __riscv
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_EN;
#else
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_EN;
#endif

        return E_NO_ERROR;
    }
}

int MXC_SEMA_RevA_InitBoxes(mxc_sema_reva_regs_t *sema_regs)
{
    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Reset the boxes */
        memset((void *)mxcSemaBox0, 0, MAILBOX_SIZE);
        memset((void *)mxcSemaBox1, 0, MAILBOX_SIZE);

        return E_NO_ERROR;
    }
}

static unsigned semaGetReadBoxAvailLen(void)
{
    unsigned length;
    int diff;

    if (SEMA_READ_BOX == 1) {
        diff = mxcSemaBox1->writeLocation - mxcSemaBox1->readLocation;
    } else {
        diff = mxcSemaBox0->writeLocation - mxcSemaBox0->readLocation;
    }

    /* Wrap if write pointer is behind the read pointer */
    if (diff < 0) {
        length = MAILBOX_PAYLOAD_LEN + diff;
    } else {
        length = diff;
    }

    return length;
}

static void semaReadBox(mxc_sema_reva_regs_t *sema_regs, mxcSemaBox_t *box, uint8_t *data,
                        uint16_t len)
{
    /* Copy data from the box to the buffer */
    memcpy(data, &box->payload[box->readLocation], len);

    /* Advance and wrap the pointers */
    box->readLocation += len;
    box->readLocation = box->readLocation % MAILBOX_PAYLOAD_LEN;
}

int MXC_SEMA_RevA_ReadBox(mxc_sema_reva_regs_t *sema_regs, uint8_t *data, unsigned len)
{
    int err;
    unsigned readLen;
    mxcSemaBox_t *readBox;

    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Lock the semaphore */
        err = MXC_SEMA_RevA_GetSema(sema_regs, SEMA_READ_SEMA);
        if (err != E_NO_ERROR) {
            return E_BUSY;
        }

        /* Get the available read length */
        if (len > semaGetReadBoxAvailLen()) {
            MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_READ_SEMA);
            return E_UNDERFLOW;
        }

        /* Assign the box pointer */
        if (SEMA_READ_BOX == 1) {
            readBox = mxcSemaBox1;
        } else if (SEMA_READ_BOX == 0) {
            readBox = mxcSemaBox0;
        }

        /* Portion the read */
        if (readBox->writeLocation < readBox->readLocation) {
            /* Write location is behind the read location, wrap at the boundary */
            if (len > (MAILBOX_PAYLOAD_LEN - readBox->readLocation)) {
                readLen = (MAILBOX_PAYLOAD_LEN - readBox->readLocation);
                semaReadBox(sema_regs, readBox, data, readLen);
                data += readLen;
                len -= readLen;
            }
        }

        /* Complete the read */
        semaReadBox(sema_regs, readBox, data, len);

        /* Release the semaphore */
        MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_READ_SEMA);

        /* Interrupt the peer when we're done reading */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

        return E_NO_ERROR;
    }
}

int MXC_SEMA_RevA_ReadBoxAsync(mxc_sema_reva_regs_t *sema_regs, mxc_sema_complete_cb_t cb,
                               uint8_t *data, unsigned len)
{
    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Read currently in progress */
        if (mxcSemaCb.readBuf != NULL) {
            return E_BUSY;
        }

        /* Register the read request */
        mxcSemaCb.readBuf = data;
        mxcSemaCb.readLen = len;
        mxcSemaCb.readCb = cb;

        /* Pend the local interrupt to process the request */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifdef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifdef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

        return E_NO_ERROR;
    }
}

static unsigned semaGetWriteBoxAvailLen(void)
{
    unsigned length;
    int diff;

    if (SEMA_WRITE_BOX == 1) {
        diff = mxcSemaBox1->readLocation - mxcSemaBox1->writeLocation;
    } else {
        diff = mxcSemaBox0->readLocation - mxcSemaBox0->writeLocation;
    }

    /* Wrap if read pointer is behind the write pointer */
    if (diff == 0) {
        length = MAILBOX_PAYLOAD_LEN - 1;
    } else if (diff < 0) {
        length = MAILBOX_PAYLOAD_LEN + diff - 1;
    } else {
        length = diff - 1;
    }

    return length;
}

static void semaWriteBox(mxc_sema_reva_regs_t *sema_regs, mxcSemaBox_t *box, const uint8_t *data,
                         unsigned len)
{
    /* Copy data from the buffer to the box */
    memcpy(&box->payload[box->writeLocation], data, len);
    box->writeLocation += len;
    box->writeLocation = box->writeLocation % MAILBOX_PAYLOAD_LEN;
}

int MXC_SEMA_RevA_WriteBox(mxc_sema_reva_regs_t *sema_regs, const uint8_t *data, unsigned len)
{
    mxcSemaBox_t *writeBox;
    unsigned writeLen;
    int err;

    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        err = MXC_SEMA_RevA_GetSema(sema_regs, SEMA_WRITE_SEMA);
        if (err != E_NO_ERROR) {
            return E_BUSY;
        }

        if (len > semaGetWriteBoxAvailLen()) {
            MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_WRITE_SEMA);
            return E_OVERFLOW;
        }

        if (SEMA_WRITE_BOX == 1) {
            writeBox = mxcSemaBox1;
        } else if (SEMA_WRITE_BOX == 0) {
            writeBox = mxcSemaBox0;
        }

        /* Portion the write */
        if (len > (MAILBOX_PAYLOAD_LEN - writeBox->writeLocation)) {
            writeLen = (MAILBOX_PAYLOAD_LEN - writeBox->writeLocation);
            semaWriteBox(sema_regs, writeBox, data, writeLen);
            data += writeLen;
            len -= writeLen;
        }

        /* Complete the write */
        semaWriteBox(sema_regs, writeBox, data, len);

        MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_WRITE_SEMA);

        /* Interrupt the peer when we're done writing */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

        return E_NO_ERROR;
    }
}

int MXC_SEMA_RevA_WriteBoxAsync(mxc_sema_reva_regs_t *sema_regs, mxc_sema_complete_cb_t cb,
                                const uint8_t *data, unsigned len)
{
    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Read currently in progress */
        if (mxcSemaCb.writeBuf != NULL) {
            return E_BUSY;
        }

        /* Register the read request */
        mxcSemaCb.writeBuf = (uint8_t *)data;
        mxcSemaCb.writeLen = len;
        mxcSemaCb.writeCb = cb;

        /* Pend the local interrupt to process the request */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifdef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifdef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

        return E_NO_ERROR;
    }
}

static int MXC_SEMA_RevA_WriteHandler(mxc_sema_reva_regs_t *sema_regs)
{
    mxcSemaBox_t *writeBox;
    unsigned writeAvailLen, writeLen, writeLenPart;
    int err;

    if (MAILBOX_SIZE == 0) {
        return E_NONE_AVAIL;
    } else {
        /* Check to see if we have any pending read requests */
        if (mxcSemaCb.writeBuf == NULL) {
            return E_NO_ERROR;
        }

        /* Get the write semaphore */
        err = E_BUSY;
        while (err != E_NO_ERROR) {
            err = MXC_SEMA_RevA_GetSema(sema_regs, SEMA_WRITE_SEMA);
        }

        /* Assign the writeBox pointer */
        if (SEMA_WRITE_BOX == 1) {
            writeBox = mxcSemaBox1;
        } else if (SEMA_WRITE_BOX == 0) {
            writeBox = mxcSemaBox0;
        }

        /* Check the available write length */
        writeAvailLen = semaGetWriteBoxAvailLen();
        if (mxcSemaCb.writeLen < writeAvailLen) {
            writeLen = mxcSemaCb.writeLen;
        } else {
            writeLen = writeAvailLen;
        }

        /* Return without interrupting if not writing */
        if (writeLen == 0) {
            MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_WRITE_SEMA);
            return E_NO_ERROR;
        }

        /* Portion the write */
        writeLenPart = 0;
        if (writeLen > (MAILBOX_PAYLOAD_LEN - writeBox->writeLocation)) {
            writeLenPart = (MAILBOX_PAYLOAD_LEN - writeBox->writeLocation);
            semaWriteBox(sema_regs, writeBox, mxcSemaCb.writeBuf, writeLenPart);
            mxcSemaCb.writeBuf += writeLenPart;
            writeLen -= writeLenPart;
        }

        /* Complete the write */
        semaWriteBox(sema_regs, writeBox, mxcSemaCb.writeBuf, writeLen);
        mxcSemaCb.writeBuf += writeLen;
        mxcSemaCb.writeLen -= (writeLenPart + writeLen);

        MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_WRITE_SEMA);

        /* Call the callback if we're done with the read */
        if (mxcSemaCb.writeLen == 0) {
            mxcSemaCb.writeBuf = NULL;

            if (mxcSemaCb.writeCb != NULL) {
                mxcSemaCb.writeCb(E_NO_ERROR);
            }
        }

        /* Interrupt the peer when we're done writing */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifndef __riscv
        sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
        sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

        return E_NO_ERROR;
    }
}

static int MXC_SEMA_RevA_ReadHandler(mxc_sema_reva_regs_t *sema_regs)
{
    int err;
    unsigned readAvailLen, readLen, readLenPart;
    mxcSemaBox_t *readBox;

    /* Check to see if we have any pending read requests */
    if (mxcSemaCb.readBuf == NULL) {
        return E_NO_ERROR;
    }

    /* Get the read semaphore */
    err = E_BUSY;
    while (err != E_NO_ERROR) {
        err = MXC_SEMA_RevA_GetSema(sema_regs, SEMA_READ_SEMA);
    }

    /* Assign the read box pointer */
    if (SEMA_READ_BOX == 1) {
        readBox = mxcSemaBox1;
    } else if (SEMA_READ_BOX == 0) {
        readBox = mxcSemaBox0;
    }

    /* Check the available read length */
    readAvailLen = semaGetReadBoxAvailLen();
    if (mxcSemaCb.readLen < readAvailLen) {
        readLen = mxcSemaCb.readLen;
    } else {
        readLen = readAvailLen;
    }

    /* Return without interrupting if not reading */
    if (readLen == 0) {
        MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_READ_SEMA);
        return E_NO_ERROR;
    }

    /* Portion the read */
    readLenPart = 0;
    if (readBox->writeLocation < readBox->readLocation) {
        /* Write location is behind the read location, wrap at the boundary */
        if (readLen > (MAILBOX_PAYLOAD_LEN - readBox->readLocation)) {
            readLenPart = (MAILBOX_PAYLOAD_LEN - readBox->readLocation);
            semaReadBox(sema_regs, readBox, mxcSemaCb.readBuf, readLenPart);
            mxcSemaCb.readBuf += readLenPart;
            readLen -= readLenPart;
        }
    }

    /* Complete the read */
    semaReadBox(sema_regs, readBox, mxcSemaCb.readBuf, readLen);
    mxcSemaCb.readBuf += readLen;
    mxcSemaCb.readLen -= (readLenPart + readLen);

    MXC_SEMA_RevA_FreeSema(sema_regs, SEMA_READ_SEMA);

    /* Call the callback if we're done with the read */
    if (mxcSemaCb.readLen == 0) {
        mxcSemaCb.readBuf = NULL;

        if (mxcSemaCb.readCb != NULL) {
            mxcSemaCb.readCb(E_NO_ERROR);
        }
    }

    /* Interrupt the peer when we're done reading */
#if TARGET_NUM == 32570 || TARGET_NUM == 32650 || TARGET_NUM == 32665
#ifndef __riscv
    sema_regs->irq1 |= MXC_F_SEMA_REVA_IRQ1_RV32_IRQ;
#else
    sema_regs->irq0 |= MXC_F_SEMA_REVA_IRQ0_CM4_IRQ;
#endif
#else
#ifndef __riscv
    sema_regs->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ;
#else
    sema_regs->irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ;
#endif
#endif

    return E_NO_ERROR;
}

int MXC_SEMA_RevA_Handler(mxc_sema_reva_regs_t *sema_regs)
{
    int err = 0;

    /* Process any pending read requests */
    err |= MXC_SEMA_RevA_ReadHandler(sema_regs);

    /* Process any pending write requests */
    err |= MXC_SEMA_RevA_WriteHandler(sema_regs);

    return err;
}
