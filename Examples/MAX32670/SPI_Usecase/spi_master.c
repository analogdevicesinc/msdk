/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/*******************************      INCLUDES    ****************************/
#include <stdio.h>

#include "spi_config.h"

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/*******************************     Variables    ****************************/

/******************************* Static Functions ****************************/

/******************************* Public Functions ****************************/
int spi_master_init(void)
{
    int ret = 0;
    int masterMode = 1;
    int quadModeUsed = 0;
    int numSlaves = 1;
    int ssPolarity = 0;

    ret = MXC_SPI_Init(SPIx_MASTER, masterMode, quadModeUsed, numSlaves, ssPolarity, SPI_BAUD_RATE);
    if (ret) {
        return ret;
    }

    MXC_SPI_SetDataSize(SPIx_MASTER, 8);
    MXC_SPI_SetWidth(SPIx_MASTER, SPI_WIDTH_STANDARD);

    return ret;
}

int spi_master_send_rcv(unsigned char *src, unsigned int srcLen, unsigned char *dst)
{
    int ret = 0;
    mxc_spi_req_t req;

    req.spi = SPIx_MASTER;
    req.txData = (uint8_t *)src;
    req.rxData = (uint8_t *)dst;
    req.txLen = srcLen;
    req.rxLen = srcLen;
    req.ssIdx = 0; // SS0 is connected
    req.ssDeassert = 1;
    req.txCnt = 0;
    req.rxCnt = 0;
    req.completeCB = NULL;

    ret = MXC_SPI_MasterTransaction(&req);

    return ret;
}
