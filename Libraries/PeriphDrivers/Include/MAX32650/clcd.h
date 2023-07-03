/**
 * @file    clcd.h
 * @brief   Color LCD function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_CLCD_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_CLCD_H_

/* **** Includes **** */
#include "clcd_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup clcd Color LCD (CLCD)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/** 
 * Enumeration type for setting the number Bits per Pixel for the LCD screen 
 */
typedef enum {
    MXC_BPP1 = MXC_S_CLCD_CTRL_BPP_BPP1, /**< 1 Bits per Pixel.                       */
    MXC_BPP2 = MXC_S_CLCD_CTRL_BPP_BPP2, /**< 2 Bits per Pixel.                       */
    MXC_BPP4 = MXC_S_CLCD_CTRL_BPP_BPP4, /**< 4 Bits per Pixel.                       */
    MXC_BPP8 = MXC_S_CLCD_CTRL_BPP_BPP8, /**< 8 Bits per Pixel.                       */
    MXC_BPP16 = MXC_S_CLCD_CTRL_BPP_BPP16, /**< 16 Bits per Pixel.                      */
    MXC_BPP24 = MXC_S_CLCD_CTRL_BPP_BPP24, /**< 24 Bits per Pixel.                      */
} mxc_clcd_bpp_t;

/** 
 * Structure type for configuring the CLCD peripheral. 
 */
typedef struct mxc_clcd_cfg {
    uint32_t width;
    uint32_t height;
    uint32_t frequency;
    uint32_t vfrontporch;
    uint32_t vbackporch;
    uint32_t vsyncwidth;
    uint32_t hfrontporch;
    uint32_t hbackporch;
    uint32_t hsyncwidth;
    uint32_t *palette;
    uint32_t paletteSize;
    mxc_clcd_bpp_t bpp;
} mxc_clcd_cfg_t;

/* **** Function Prototypes **** */

/**
 * @brief Initialize the clcd.
 * @param      cfg The clcd configuration
 * @return #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_Init(mxc_clcd_cfg_t *cfg);

/**
 * @brief      Shutdown CLCD module.
 * @return     #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_Shutdown(void);

/**
 * @brief Configure CLCD frame module.
 * @param      cfg The clcd configuration
 * @return #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_ConfigPanel(mxc_clcd_cfg_t *cfg);

/**
 * @brief      Enable CLCD  module.
 * @return     #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_Enable(void);

/**
 * @brief      Disable CLCD  module.
 * @return     #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_Disable(void);

/**
 * @brief Set CLCD frame module.
 * @param      addr The frame address
 * @return #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_SetFrameAddr(void *addr);

/**@} end of group clcd */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_CLCD_H_
