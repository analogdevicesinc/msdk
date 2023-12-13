/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#if defined(__GNUC__)
#include <unistd.h>
#include <sys/stat.h>
#endif /* __GNUC__ */

#if defined(__CC_ARM)
#include <rt_misc.h>
#pragma import(__use_no_semihosting_swi)

struct __FILE {
    int handle;
};
FILE __stdout;
FILE __stdin;

#endif /* __CC_ARM */

/* Defines - Compiler Specific */
#if defined(__ICCARM__)
#define STDIN_FILENO 0 // Defines that are not included in the DLIB.
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#define EBADF -1
#endif /* __ICCARM__ */

#include "mxc_device.h"
#include "board.h"

/* The following libc stub functions are required for a proper link with printf().
 * These can be tailored for a complete stdio implementation.
 * GNUC requires all functions below. IAR & KEIL only use read and write.
 */
#if defined(__GNUC__)
int _open(const char *name, int flags, int mode)
{
    return -1;
}
int _close(int file)
{
    return -1;
}
int _isatty(int file)
{
    return -1;
}
int _lseek(int file, off_t offset, int whence)
{
    return -1;
}
int _fstat(int file, struct stat *st)
{
    return -1;
}
#endif /* __GNUC__ */

/* Handle IAR and ARM/Keil Compilers for _read/_write. Keil uses fputc and
   fgetc for stdio */
#if defined(__ICCARM__) || defined(__GNUC__)

#if defined(__GNUC__) // GNUC _read function prototype
int _read(int file, char *ptr, int len)
#elif defined(__ICCARM__) // IAR Compiler _read function prototype
int __read(int file, unsigned char *ptr, size_t len)
#endif /* __GNUC__ */
{
    unsigned int n;
    int num = 0;

    switch (file) {
    case STDIN_FILENO:
        for (n = 0; n < len; n++) {
            //*ptr = UART_GetChar();
            //UART_PutChar(*ptr);
            ptr++;
            num++;
        }

        break;

    default:
        errno = EBADF;
        return -1;
    }

    return num;
}

/* newlib/libc printf() will eventually call write() to get the data to the stdout */
#if defined(__GNUC__)
// GNUC _write function prototype
int _write(int file, char *ptr, int len)
{
    int n;
#elif defined(__ICCARM__) // IAR Compiler _read function prototype
// IAR EW _write function prototype
int __write(int file, const unsigned char *ptr, size_t len)
{
    size_t n;
#endif /* __GNUC__ */

    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
        for (n = 0; n < len; n++) {
            if (*ptr == '\n') {
                //UART_PutChar('\r');
            }

            //UART_PutChar(*ptr++);
        }

        break;

    default:
        errno = EBADF;
        return -1;
    }

    return len;
}

#endif /* ( __ICCARM__ ) || ( __GNUC__ ) */

/* Handle Keil/ARM Compiler which uses fputc and fgetc for stdio */
#if defined(__CC_ARM)
int fputc(int c, FILE *f)
{
    if (c != '\n') {
        //UART_PutChar(c);
    } else {
        //UART_PutChar('\r');
        //UART_PutChar('\n');
    }

    return 0;
}

int fgetc(FILE *f)
{
    return (UART_GetChar());
}

int ferror(FILE *f)
{
    return EOF;
}

void _ttywrch(int c)
{
    if (c != '\n') {
        //UART_PutChar(c);
    } else {
        //UART_PutChar('\r');
        //UART_PutChar('\n');
    }
}

void _sys_exit(int return_code)
{
    while (1) {}
}

#endif /* __CC_ARM  */
