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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CLCD_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CLCD_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "clcd_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup clcd Color LCD
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 * @brief   Enumeration type for setting the number Bits per Pixel for the LCD screen
 *
 */
typedef enum {
    MXC_CLCD_STN,
    MXC_CLCD_TFT,
} mxc_clcd_disp_t;

/**
 * @brief   Enumeration type for the color mode used in 16bit color depth
 *
 */
typedef enum {
    RGB565,
    BGR556,
} mxc_clcd_color_t;

/**
 * @brief   The information required to completely configure the CLCD to match
 *          a particular screen.
 *
 *          This can be used with the CLCD_Config() function to configure the
 *          CLCD and prepare it for outputting data to the screen.
 */
typedef struct clcd_cfg {
    uint32_t Width; ///< The visible width of the display in pixels
    uint32_t Height; ///< The visible height of the display in pixels
    uint32_t ClkFreq; ///< The requested bit clock frequency in Hz
    uint32_t VertFrontPorch; ///< The vertical front porch size in lines
    uint32_t VertBackPorch; ///< The vertical back porch size in lines
    uint32_t VSYNCPulseWidth; ///< The width of the VSYNC pulse in line clocks
    uint32_t HorizFrontPorch; ///< The horizontal front porch size in lines
    uint32_t HorizBackPorch; ///< The horizontal back porch size in lines
    uint32_t HSYNCPulseWidth; ///< The width of the HSYNC pulse in bit clocks
    uint32_t *palette; ///< Pointer to the color palette data (used for bpp < 16)
    uint32_t paletteSize; ///< Size of the color palette data (must be 768 bytes)
    uint32_t bpp; ///< Number of bits per pixel (see users guide for values)
    void *frameBuffer; ///< Pointer to the frame buffer, this must remain allocated
    ///< the entire time the CLCD is in use, unless the frameBuffer
    ///< pointer is changed with CLCD_SetFrameBuffer
} mxc_clcd_cfg_t;

/* **** Function Prototypes **** */

/**
 * @brief   Initialize the clcd.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_Init(void);

/**
 * @brief   Shutdown CLCD module.
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_Shutdown(void);

/**
 * @brief   Enable CLCD.
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_Enable(void);

/**
 * @brief   Disable CLCD.
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_Disable(void);

/**
 * @brief   Set the CLCD clock frequency
 *
 * @param   hz   The frequency of the CLCD_CLK in Hz
 *
 * @return  The actual frequency set in Hz
 */
int MXC_CLCD_SetFrequency(int hz);

/**
 * @brief   Get the CLCD clock frequency
 * @return  The frequency in Hz
 */
int MXC_CLCD_GetFrequency(void);

/**
 * @brief   Set CLCD frame buffer address.
 * @note    The data at addr must remain valid while using the CLCD
 *
 * @param   addr Pointer to the new framebuffer
 */
void MXC_CLCD_SetFrameAddr(void *addr);

/**
 * @brief   Get CLCD frame buffer address.
 * @note    The data at addr must remain valid while using the CLCD
 *
 * @return  Pointer to the framebuffer
 */
void *MXC_CLCD_GetFrameAddr(void);

/**
 * @brief   Set CLCD Color Palette
 * @note    The color palette is not used in 16 and 24 bit color depth
 *          for either display type.
 *
 * @param   palette Pointer to the color palette data, must be 768 bytes
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_SetColorPalette(uint32_t *palette);

/**
 * @brief   Get the status of power to the display
 *
 * @return  1 if display on (PWREN pin), 0 otherwise
 */
int MXC_CLCD_IsEnabled(void);

/**
 * @brief   Set the RGB mode used by the peripheral
 *
 * @param   rgb565  See \ref mxc_clcd_color_t for a list of valid values
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_SetRGBMode(mxc_clcd_color_t rgb565);

/**
 * @brief   Get the current RGB mode
 *
 * @return  See \ref mxc_clcd_color_t for return values
 */
mxc_clcd_color_t MXC_CLCD_GetRGBMode(void);

/**
 * @brief   Set the number of bits per pixel
 * @note    Valid values are detailed in the user guide
 *
 * @param   bitsPerPixel number of bits per pixel (i.e. 8, 16, 24, etc)
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_CLCD_SetBPP(int bitsPerPixel);

/**
 * @brief   Get the number of bits per pixel
 *
 * @return  Number of bits per pixel
 */
int MXC_CLCD_GetBPP(void);

/**
 * @brief   Set the type of display to drive
 *
 * @param   type See \ref mxc_clcd_disp_t for a list of display types
 */
void MXC_CLCD_SetDisplayType(mxc_clcd_disp_t type);

/**
 * @brief   Get the type of display being driven
 *
 * @return  See \ref mxc_clcd_disp_t for a list of return values
 */
mxc_clcd_disp_t MXC_CLCD_GetDisplayType(void);

/**
 * @brief   Set frequency of the AC bias
 *
 * @param   lines Set the number of line clocks per period of the AC bias clock
 */
void MXC_CLCD_SetACBias(int lines);

/**
 * @brief   Get the current frequency of the AC bias
 *
 * @return  The number of line clocks per period of the AC bias output
 */
int MXC_CLCD_GetACBias(void);

/**
 * @brief   Get the interrupt flags
 *
 * @return  Interrupt Flags
 */
int MXC_CLCD_GetFlags(void);

/**
 * @brief   Clear interrupt flags
 *
 * @param   flags The interrupt flags to clear
 */
void MXC_CLCD_ClearFlags(int flags);

/**
 * @brief   Enable interrupts
 *
 * @param   flags The interrupt flags to enable
 */
void MXC_CLCD_EnableInt(int flags);

/**
 * @brief   Disable interrupts
 *
 * @param   flags The interrupt flags to disable
 */
void MXC_CLCD_DisableInt(int flags);

/**
 * @brief   Set Vertical Back Porch Size
 *
 * @param   lines VBP size in lines
 */
void MXC_CLCD_SetVBPSize(int lines);

/**
 * @brief   Get Vertical Back Porch Size
 *
 * @return  VBP size in lines
 */
int MXC_CLCD_GetVBPSize(void);

/**
 * @brief   Set Vertical Front Porch size
 *
 * @param   lines VFP size in lines
 */
void MXC_CLCD_SetVFPSize(int lines);

/**
 * @brief   Get Vertical Front Porch size
 *
 * @return  VFP size in lines
 */
int MXC_CLCD_GetVFPSize(void);

/**
 * @brief   Set Horizontal Back Porch size
 *
 * @param   lines HBP Size in lines
 */
void MXC_CLCD_SetHBPSize(int lines);

/**
 * @brief   Get Horizontal Back Porch size
 *
 * @return  HBP Size in lines
 */
int MXC_CLCD_GetHBPSize(void);

/**
 * @brief   Set Horizontal Front Porch size
 *
 * @param   lines HFP size in lines
 */
void MXC_CLCD_SetHFPSize(int lines);

/**
 * @brief   Get Horizontal Front Porch size
 *
 * @return  HFP size in lines
 */
int MXC_CLCD_GetHFPSize(void);

/**
 * @brief   Set the number of usable vertical pixels in the display
 *
 * @param   pixels Number of usable vertical pixels
 */
void MXC_CLCD_SetVerticalSize(int pixels);

/**
 * @brief   Get the number of usable vertical pixels in the display
 *
 * @return  Number of usable vertical pixels
 */
int MXC_CLCD_GetVerticalSize(void);

/**
 * @brief   Set the number of usable horizontal pixels in the display
 *
 * @param   pixels Number of usable horizontal pixels
 */
void MXC_CLCD_SetHorizontalSize(int pixels);

/**
 * @brief   Get the number of usable horizontal pixels in the display
 *
 * @return  Number of usable horizontal pixels
 */
int MXC_CLCD_GetHorizontalSize(void);

/**
 * @brief   Set the width of the VSYNC pulse
 *
 * @param   lines Width of the VSYNC pulse in line clocks
 */
void MXC_CLCD_SetVSYNCWidth(int lines);

/**
 * @brief   Get the width of the VSYNC pulse
 *
 * @return  Width of the VSYNC pulse in line clocks
 */
int MXC_CLCD_GetVSYNCWidth(void);

/**
 * @brief   Set the width of the HSYNC pulse
 *
 * @param  lines Width of the HSYNC pulse in line clock
s */
void MXC_CLCD_SetHSYNCWidth(int lines);

/**
 * @brief   Get the width of the HSYNC pulse
 *
 * @return  Width of the HSYNC pulse in line clocks
 */
int MXC_CLCD_GetHSYNCWidth(void);

/**
 * @brief   Configure CLCD frame module.
 * @return  #E_NO_ERROR if successful, appropriate error otherwise
 */
int MXC_CLCD_Config(mxc_clcd_cfg_t *clcd_cfg);

/**@} end of group clcd */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CLCD_H_
