/**
 * @file    clcd.h
 * @brief   Color LCD function prototypes and data types.
 */

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
