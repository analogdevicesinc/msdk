/*******************************************************************************
 * Copyright (C) 2009-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef __LOG_H__
#define __LOG_H__

#define LOG_L_SUCCESS 1
#define LOG_L_ERROR 1
#define LOG_L_WARN 2
#define LOG_L_INFO 3
#define LOG_L_DEBUG 4

#ifdef __WIN

#include <windows.h>
HANDLE hConsole;
short bg_color_g;

#define BLU FOREGROUND_BLUE | FOREGROUND_GREEN
#define GRN FOREGROUND_GREEN
#define RED FOREGROUND_RED

#define YEL FOREGROUND_RED | FOREGROUND_GREEN
#define WHT FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED

#define set_color(COL)                                                                             \
    do {                                                                                           \
        if (USE_COLOR)                                                                             \
            SetConsoleTextAttribute(hConsole, COL | FOREGROUND_INTENSITY | bg_color_g);            \
    } while (0)
#define reset_color()                                                                              \
    do {                                                                                           \
        if (USE_COLOR)                                                                             \
            SetConsoleTextAttribute(hConsole, WHT | bg_color_g);                                   \
    } while (0)

#else /* WINDOWS */

#define RED "\e[5m\e[1m\x1B[31m"
#define GRN "\e[1m\x1B[32m"
#define YEL "\e[1m\x1B[33m"
#define BLU "\e[1m\x1B[34m"
#define MAG "\e[1m\x1B[35m"
#define CYN "\e[1m\x1B[36m"
#define WHT "\e[1m\x1B[37m"
#define RESET "\x1B[0m"

#define set_color(COL)                                                                             \
    do {                                                                                           \
        if (USE_COLOR)                                                                             \
            printf(COL);                                                                           \
    } while (0)
#define reset_color()                                                                              \
    do {                                                                                           \
        if (USE_COLOR)                                                                             \
            printf(RESET);                                                                         \
    } while (0)

#endif

#define print_success(...) print_lvl(LOG_L_SUCCESS, "SUCCESS", GRN, __VA_ARGS__)
#define print_info(...) print_lvl(LOG_L_INFO, "INFO", BLU, __VA_ARGS__)
#define print_warn(...) print_lvl(LOG_L_WARN, "WARNING", YEL, __VA_ARGS__)
#define print_debug(...) print_lvl(LOG_L_DEBUG, "DEBUG", WHT, __VA_ARGS__)
#define print_error(...) print_lvl(LOG_L_ERROR, "ERROR", RED, __VA_ARGS__)

#define print_lvl(LVL, LVL_STR, COLR, ...)                                                         \
    do {                                                                                           \
        if (LVL <= verbose) {                                                                      \
            set_color(COLR);                                                                       \
            printf("[" LVL_STR "] - ");                                                            \
            printf(__VA_ARGS__);                                                                   \
            reset_color();                                                                         \
        }                                                                                          \
    } while (0)

#define print_g(...) print_lcol(LOG_L_SUCCESS, GRN, __VA_ARGS__)
#define print_i(...) print_lcol(LOG_L_INFO, BLU, __VA_ARGS__)
#define print_w(...) print_lcol(LOG_L_WARN, YEL, __VA_ARGS__)
#define print_d(...) print_lcol(LOG_L_DEBUG, WHT, __VA_ARGS__)
#define print_e(...) print_lcol(LOG_L_ERROR, RED, __VA_ARGS__)

#define print_cb(...) print_col(BLU, __VA_ARGS__)
#define print_cy(...) print_col(YEL, __VA_ARGS__)
#define print_cw(...) print_col(WHT, __VA_ARGS__)
#define print_cr(...) print_col(RED, __VA_ARGS__)
#define print_cg(...) print_col(GRN, __VA_ARGS__)

#define print_lcol(LVL, COLR, ...)                                                                 \
    do {                                                                                           \
        if (LVL <= verbose) {                                                                      \
            set_color(COLR);                                                                       \
            printf(__VA_ARGS__);                                                                   \
            reset_color();                                                                         \
        }                                                                                          \
    } while (0)

#define print_col(COLR, ...)                                                                       \
    do {                                                                                           \
        set_color(COLR);                                                                           \
        printf(__VA_ARGS__);                                                                       \
        reset_color();                                                                             \
    } while (0)

#endif /* __LOG_H__ */
