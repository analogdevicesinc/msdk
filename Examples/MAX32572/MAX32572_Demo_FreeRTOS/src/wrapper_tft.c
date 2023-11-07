/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "MAX32xxx.h"
#include "wrapper_tft.h"

/*************************************** DEFINES *****************************/
#define TFT_LOCK_WITH_MUTEX 0 // Do not forget to enable mutex in FreeRTOSConfig.h file

/************************************** TYPE DEF *****************************/

/************************************* VARIABLES *****************************/
#if TFT_LOCK_WITH_MUTEX
static xSemaphoreHandle xLockFB = NULL;
#endif

/********************************** STATIC FUNCTIONS *************************/

/********************************** PUBLIC FUNCTIONS *************************/
void TFT_Init(void)
{
#if TFT_LOCK_WITH_MUTEX
    xLockFB = xSemaphoreCreateMutex();

    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_Init();
        xSemaphoreGive(xLockFB);
    }
#else
    MXC_TFT_Init();
#endif
}

void TFT_SetBackGroundColor(unsigned char index_color)
{
#if TFT_LOCK_WITH_MUTEX
    __disable_irq();
    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_SetBackGroundColor(index_color);
        xSemaphoreGive(xLockFB);
    }
    __enable_irq();
#else
    __disable_irq();
    MXC_TFT_SetBackGroundColor(index_color);
    __enable_irq();
#endif
}

void TFT_ShowImage(int x0, int y0, int id)
{
#if TFT_LOCK_WITH_MUTEX
    __disable_irq();
    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_ShowImage(x0, y0, id);
        xSemaphoreGive(xLockFB);
    }
    __enable_irq();
#else
    __disable_irq();
    MXC_TFT_ShowImage(x0, y0, id);
    __enable_irq();
#endif
}

void TFT_PrintFont(int x0, int y0, int id, text_t *str, area_t *area)
{
#if TFT_LOCK_WITH_MUTEX
    __disable_irq();
    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_PrintFont(x0, y0, id, str, area);
        xSemaphoreGive(xLockFB);
    }
    __enable_irq();
#else
    __disable_irq();
    MXC_TFT_PrintFont(x0, y0, id, str, area);
    __enable_irq();
#endif
}

void TFT_ClearArea(area_t *area, int color)
{
#if TFT_LOCK_WITH_MUTEX
    __disable_irq();
    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_ClearArea(area, color);
        xSemaphoreGive(xLockFB);
    }
    __enable_irq();
#else
    __disable_irq();
    MXC_TFT_ClearArea(area, color);
    __enable_irq();
#endif
}

void TFT_SetPalette(int img_id)
{
#if TFT_LOCK_WITH_MUTEX
    __disable_irq();
    if (xSemaphoreTake(xLockFB, 0xFFFF)) {
        MXC_TFT_SetPalette(img_id);
        xSemaphoreGive(xLockFB);
    }
    __enable_irq();

#else
    __disable_irq();
    MXC_TFT_SetPalette(img_id);
    __enable_irq();
#endif
}
