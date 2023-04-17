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
#ifndef MAXCAM_DEBUG_H_
#define MAXCAM_DEBUG_H_

#include <stdio.h>
#include <string.h>

//#define IMAGE_TO_UART

#ifndef IMAGE_TO_UART
#define PR_DEBUG(fmt, args...) \
    if (1)                     \
    printf("D[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_INFO(fmt, args...) \
    if (0)                    \
    printf("I[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_WARN(fmt, args...) \
    if (1)                    \
    printf("W[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#define PR_ERR(fmt, args...) \
    if (1)                   \
    printf("E[%-10s:%4d] " fmt "\r\n", S_MODULE_NAME, __LINE__, ##args) ///< TODO
#else
#define PR_DEBUG(fmt, args...)
#define PR_INFO(fmt, args...)
#define PR_WARN(fmt, args...)
#define PR_ERR(fmt, args...)
#endif
#endif /* MAXCAM_DEBUG_H_ */
