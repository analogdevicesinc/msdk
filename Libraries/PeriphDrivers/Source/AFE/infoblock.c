/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
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
*******************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "infoblock.h"
#include "mxc_device.h"
#include "flc.h"

uint16_t crc15_highbitinput(uint16_t crc15val, uint8_t *input, int bitlength)
{
    uint16_t inputbit;
    uint16_t feedbackbit;
    int i;

    for (i = 0; i < bitlength; i++) {
        inputbit = (input[i / 8] >> (7 - i % 8)) & 0x01;
        feedbackbit = ((crc15val & 0x4000) >> 14) & 0x01;
        crc15val <<= 1;
        if (inputbit ^ feedbackbit) {
            crc15val ^= 0x4599;
        }
    }
    // Clean up.
    // Mask off any high bits we were ignoring in the loop.
    crc15val &= 0x7FFF;

    return crc15val;
}

int infoblock_readraw(uint32_t offset, uint8_t *data)
{
    int result;

    if ((result = MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE)) != E_NO_ERROR) {
        return result;
    }
    memcpy(data, (uint8_t *)MXC_INFO0_MEM_BASE + offset, INFOBLOCK_LINE_SIZE);
    if ((result = MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE)) != E_NO_ERROR) {
        return result;
    }

    return E_NO_ERROR;
}

int infoblock_read(uint32_t offset, uint8_t *data, int length)
{
    uint8_t oneinfoblockline[INFOBLOCK_LINE_SIZE];
    int lengthtocopy;
    uint16_t crc = 0;
    uint16_t crcexpected;
    int i;
    int result;
    lineformat_e lineformat;

    if (length > INFOBLOCK_MAXIMUM_READ_LENGTH) {
        return E_BAD_PARAM;
    }

    if (data == NULL) {
        return E_BAD_PARAM;
    }

    switch (offset) {
    case INFOBLOCK_USN_OFFSET:
        lineformat = INFOBLOCK_LINE_FORMAT_USN;
        break;
    case INFOBLOCK_ICE_LOCK_OFFSET:
        lineformat = INFOBLOCK_LINE_FORMAT_RAW;
        break;
    case INFOBLOCK_KEY_OFFSET:
        lineformat = INFOBLOCK_LINE_FORMAT_DESIGN;
        break;
    default:
        lineformat = INFOBLOCK_LINE_FORMAT_RAW;
        break;
    }

    while (length > 0) {
        if ((result = infoblock_readraw(offset, oneinfoblockline)) != E_NO_ERROR) {
            return result;
        }

        switch (lineformat) {
        case INFOBLOCK_LINE_FORMAT_USN:
            // NO CRC15, ignore lowest 15 bits
            // Lock bit is high bit, bit 63.
            // Data is middle 48 bits 62 to 15.
            // CRC15 is lower 15 bits (bits 0-14)
            // First shift data one bit left starting at the high byte.
            for (i = 7; i > 1; i--) {
                oneinfoblockline[i] <<= 1;
                oneinfoblockline[i] |= (oneinfoblockline[i - 1] & 0x80) >> 7;
            }
            // Then, shift data by two bytes
            memmove(oneinfoblockline, oneinfoblockline + INFOBLOCK_LINE_OVERHEAD, 6);
            lengthtocopy = INFOBLOCK_LINE_SIZE - INFOBLOCK_LINE_OVERHEAD;
            break;
        case INFOBLOCK_LINE_FORMAT_RAW:
            lengthtocopy = INFOBLOCK_LINE_SIZE;
            break;
        case INFOBLOCK_LINE_FORMAT_DESIGN:
            // Check for unprogrammed information block line
            crc = 0xFF;
            for (i = 0; i < sizeof(oneinfoblockline); i++) { crc &= oneinfoblockline[i]; }
            if (crc == 0xFF) {
                // Line is unprogrammed, return error.
                return E_BAD_STATE;
            }
            // Lock bit is high bit, bit 63.
            // CRC15 is middle 15 bits [62:48]
            // Data is lower 48 bits [47:0].
            // First, CRC the lock bit.
            crc = crc15_highbitinput(0, oneinfoblockline + 7, 1);
            // Then CRC the data from high to low bits.  (bit 47 to 0)
            for (i = 5; i >= 0; i--) { crc = crc15_highbitinput(crc, oneinfoblockline + i, 8); }
            crcexpected = ((oneinfoblockline[7] & 0x7F) << 8) | oneinfoblockline[6];
            if (crc != crcexpected) {
                return E_BAD_STATE;
            }
            lengthtocopy = INFOBLOCK_LINE_SIZE - INFOBLOCK_LINE_OVERHEAD;
            break;
        default:
            // NOTE: Should never get here.
            return E_BAD_STATE;
            break;
        }

        if (lengthtocopy > length) {
            lengthtocopy = length;
        }
        memcpy(data, oneinfoblockline, lengthtocopy);
        data += lengthtocopy;
        length -= lengthtocopy;
        offset += INFOBLOCK_LINE_SIZE;
    }

    return E_NO_ERROR;
}
