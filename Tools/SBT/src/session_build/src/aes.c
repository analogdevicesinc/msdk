/*******************************************************************************
* Copyright (C) 2009-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
*******************************************************************************
*
* @author: Yann Loisel <yann.loisel@maximintegrated.com>
* @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include <ucl/ucl.h>
#include <ucl/ucl_aes.h>
#include <ucl/ucl_aes_ecb.h>

#include "session_build.h"
#include "scp_definitions.h"
#include "scp_utils.h"
#include <log.h>

int aes_checksum(u8* crc, const u8* data, int size, int trunk)
{
    int i, j;
    u8 keynull[16];
    u8 h[16];
    int resu;
    u8 input[16];

    for (i = 0; i < 16; i++) { h[i] = keynull[i] = 0; }

    for (i = 0; i < size; i += 16) {
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                input[j] = h[j] ^ data[i + j];
            } else {
                input[j] = h[j];
            }
        }

        resu = ucl_aes_ecb(h, input, 16, keynull, 16, UCL_CIPHER_ENCRYPT);

        if (resu != UCL_OK) {
            print_error("AES checksum : AES ECB (%x)\n", resu);
            return (resu);
        }
    }

    for (i = 0; i < trunk; i++) { crc[i] = h[i]; }

    return ERR_OK;
}

void print_aeskey(const uint8_t* key, const char* keyname)
{
    unsigned int i;

    print_debug("\tAES Key %s :", keyname);

    for (i = 0; i < 16; i++) { print_d("%02x", key[i]); }
    print_d("\n");
}
