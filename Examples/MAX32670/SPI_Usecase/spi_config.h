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

#ifndef SPI_CONFIG_H_
#define SPI_CONFIG_H_

#include "max32670.h"
#include "spi_regs.h"
#include "spi.h"

/*** SPI Master Configuration ***/
#define SPIx_MASTER   MXC_SPI0
#define SPI_BAUD_RATE 1000000

/*** SPI Slave Configuration ***/
#define SPIx_SLAVE      MXC_SPI1
#define SPIx_IRQn       SPI1_IRQn
#define SPIx_IRQHandler SPI1_IRQHandler

//
#define TEST_BUFF_SIZE 64

/*** Functions ***/
int spi_master_init(void);
int spi_master_send_rcv(unsigned char* src, unsigned int srcLen, unsigned char* dst);

int spi_slave_init(void);
int spi_slave_send(unsigned char* src, unsigned int srcLen);
int spi_slave_rcv(unsigned char* dst, unsigned int maxLen, unsigned int* len);

#endif /* SPI_CONFIG_H_ */
