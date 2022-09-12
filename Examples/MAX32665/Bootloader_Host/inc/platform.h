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

#ifndef _PLATFORM_MAX32665_H_
#define _PLATFORM_MAX32665_H_

/*******************************      INCLUDES    ****************************/

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/******************************* Public Functions ****************************/
// i2c
int plt_i2c_init(void);
int plt_i2c_write(unsigned char *src, unsigned int len);
int plt_i2c_read(unsigned char *dst, unsigned int len);

// spi
int plt_spi_init(void);
int plt_spi_write(unsigned char *src, unsigned int len);
int plt_spi_read(unsigned char *dst, unsigned int len);

// gpio
int plt_gpio_init(void);
void plt_gpio_set(unsigned int idx, int state);
int plt_gpio_get(unsigned int idx);

// delay
void plt_delay_ms(unsigned int ms);

#endif // _PLATFORM_MAX32665_H_
