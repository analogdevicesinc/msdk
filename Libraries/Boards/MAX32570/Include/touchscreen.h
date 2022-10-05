/**
 * @file
 * @brief   Touchscreen driver API header file
 */
/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef LIBRARIES_BOARDS_MAX32570_INCLUDE_TOUCHSCREEN_H_
#define LIBRARIES_BOARDS_MAX32570_INCLUDE_TOUCHSCREEN_H_

/************************************************************************************/
#define TS_MAX_BUTTONS 16
#define TS_INVALID_KEY_CODE -1

typedef enum {
    TSC_TEMP0 = (0x83 | 0x00 | 0x00),
    TSC_Y = (0x83 | 0x10 | 0x04),
    TSC_VBAT = (0x83 | 0x20 | 0x00),
    TSC_Z1 = (0x83 | 0x30 | 0x04),
    TSC_Z2 = (0x83 | 0x40 | 0x04),
    TSC_X = (0x83 | 0x50 | 0x04),
    TSC_AUX = (0x83 | 0x60 | 0x00),
    TSC_TEMP1 = (0x83 | 0x70 | 0x00),
    TSC_DIFFX = (0x81 | 0x50 | 0x00),
    TSC_DIFFY = (0x81 | 0x10 | 0x00),
    TSC_DIFFZ1 = (0x81 | 0x30 | 0x00),
    TSC_DIFFZ2 = (0x81 | 0x40 | 0x00),
    TSC_START = (0x82 | 0x00 | 0x00),
    TSC_STOP = (0x81 | 0x00 | 0x00)
} mxc_ts_touch_cmd_t;

/************************************************************************************/
/**
 * @brief      Initialize the touchscreen and display
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TS_Init(void);

/**
 * @brief      Enables touch interrupts
 *
 */
void MXC_TS_Start(void);

/**
 * @brief      Disables touch interrupts
 *
 */
void MXC_TS_Stop(void);

/**
 * @brief      Register a button
 *
* @param      x0, y0, x1, y1 location of button
* @param      on_press_expected_code  expected keycode when touch related point area
 */
int MXC_TS_AddButton(int x0, int y0, int x1, int y1, int on_press_expected_code);

/**
 * @brief      Remove a button
 *
 * @param      x0, y0, x1, y1 location of button
 */
void MXC_TS_RemoveButton(int x0, int y0, int x1, int y1);

/**
 * @brief      Remove all registered keys
 *
 */
void MXC_TS_RemoveAllButton(void);

/**
 * @brief      Read pressed key
 *
 */
int MXC_TS_GetKey(void);

#endif // LIBRARIES_BOARDS_MAX32570_INCLUDE_TOUCHSCREEN_H_
