/*******************************************************************************
 * Copyright (C) 2009-2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *******************************************************************************
 *
 * @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
 *
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __WIN
#include <windows.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 */
int hex(char c1, char c2);

/**
 * Replace all occurrences of `sub` with `replace` in `str`
 * @param str
 * @param sub
 * @param replace
 * @return
 */
char* str_replace(const char* str, const char* sub, const char* replace);

/**
 *
 * @param dst
 * @param src
 * @param dest_size
 * @return
 */
size_t strlcpy(char* dst, const char* src, size_t dest_size);

/**
 * Test if a file existe
 * @param filename
 * @return TRUE is the file exist and FALSE if not
 */
int file_exist(char* filename);

int make_dir(char* dirname);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H__ */
