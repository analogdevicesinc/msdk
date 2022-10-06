/**
 * @file    st7735s_regs.c
 * @brief   Sitronix ST7735S LCD controller register definitions
 *          
 */

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

#ifndef EXAMPLES_MAX32672_DISPLAY_ST7735S_REGS_H_
#define EXAMPLES_MAX32672_DISPLAY_ST7735S_REGS_H_

/* Register definitions taken from CrystalFontz 
 * CFAF128128B1-0145T_SPI_Demo_Code.ino by Brent A. Crosby
 */
#define ST7735S_NOP      (0x00)
#define ST7735S_SWRESET  (0x01)
#define ST7735S_SLPOUT   (0x11)
#define ST7735S_DISPON   (0x29)
#define ST7735S_CASET    (0x2A)
#define ST7735S_RASET    (0x2B)
#define ST7735S_RAMWR    (0x2C)
#define ST7735S_RAMRD    (0x2E)
#define ST7735S_MADCTL   (0x36)
#define ST7735S_COLMOD   (0x3A)
#define ST7735S_FRMCTR1  (0xB1)
#define ST7735S_FRMCTR2  (0xB2)
#define ST7735S_FRMCTR3  (0xB3)
#define ST7735S_INVCTR   (0xB4)
#define ST7735S_PWCTR1   (0xC0)
#define ST7735S_PWCTR2   (0xC1)
#define ST7735S_PWCTR3   (0xC2)
#define ST7735S_PWCTR4   (0xC3)
#define ST7735S_PWCTR5   (0xC4)
#define ST7735S_VMCTR1   (0xC5)
#define ST7735S_GAMCTRP1 (0xE0)
#define ST7735S_GAMCTRN1 (0xE1)

#endif // EXAMPLES_MAX32672_DISPLAY_ST7735S_REGS_H_
