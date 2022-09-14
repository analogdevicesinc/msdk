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

#include <sys/errno.h>
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
/**
 * Integer to store the last character read from the FILE using fgetc().
 * Only valid if fgetc() was the last function called on the stream.
 */
int g_lastChar = 0;
/**
 * Global variable set to TRUE if fgetc() was previously called, false otherwise.
 * This variable is necessary for implementing __backspace(FILE *f) in the MDK microlib.
 */
int g_readChar = 0;

#endif /* __CC_ARM */

/* Defines - Compiler Specific */
#if defined(__ICCARM__)
#define STDIN_FILENO  0 // Defines that are not included in the DLIB.
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
#define EBADF         -1
#endif /* __ICCARM__ */

#include "mxc_device.h"
#include "mxc_sys.h"
#include "board.h"
#include "uart.h"

#define MXC_UARTn MXC_UART_GET_UART(CONSOLE_UART)
#define UART_FIFO MXC_UART_GET_FIFO(CONSOLE_UART)
/* The following libc stub functions are required for a proper link with printf().
 * These can be tailored for a complete stdio implementation.
 * GNUC requires all functions below. IAR & KEIL only use read and write.
 */
#if defined(__GNUC__)
int _open(const char* name, int flags, int mode)
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
int _fstat(int file, struct stat* st)
{
    return -1;
}
#endif /* __GNUC__ */

/* Handle IAR and ARM/Keil Compilers for _read/_write. Keil uses fputc and
   fgetc for stdio */
#if defined(__ICCARM__) || defined(__GNUC__)

#if defined(__GNUC__) // GNUC _read function prototype
int _read(int file, char* ptr, int len)
{
    int n;
#elif defined(__ICCARM__) // IAR Compiler _read function prototype
int __read(int file, unsigned char* ptr, size_t len)
{
    size_t n;
#endif                    /*  */

    int num = 0; // count of number received.

    switch (file) {
        case STDIN_FILENO:
            for (n = 0; n < len; n++) {
                *ptr = MXC_UART_ReadCharacter(MXC_UARTn); // read a byte.
                MXC_UART_WriteCharacter(MXC_UARTn, *ptr); // echo the byte.

                if (*ptr == '\r') { // check for end of line.
                    *ptr = '\n';
                    num++;
                    ptr++;

                    break;
                } else {
                    ptr++;
                    num++;
                }
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
int _write(int file, char* ptr, int len)
{
    int n;
#elif defined(__ICCARM__) // IAR Compiler _read function prototype
// IAR EW _write function prototype
int __write(int file, const unsigned char* ptr, size_t len)
{
    size_t n;
#endif                    /* __GNUC__ */

    switch (file) {
        case STDOUT_FILENO:
        case STDERR_FILENO:

            // This function should be as fast as possible
            // So we'll forgo the UART driver for now
            for (n = 0; n < len; n++) {
                if (*ptr == '\n') {
                    // Wait until there's room in the FIFO
                    while (MXC_UARTn->status & MXC_F_UART_STATUS_TX_FULL)
                        ;

                    MXC_UARTn->fifo = '\r';
                }

                // Wait until there's room in the FIFO
                while (MXC_UARTn->status & MXC_F_UART_STATUS_TX_FULL)
                    ;

                MXC_UARTn->fifo = *ptr++;
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
int fputc(int c, FILE* f)
{
    if (c != '\n') {
        MXC_UART_WriteCharacter(MXC_UARTn, c);
    } else {
        MXC_UART_WriteCharacter(MXC_UARTn, '\r');
        MXC_UART_WriteCharacter(MXC_UARTn, '\n');
    }

    return 0;
}

int __backspace(FILE* f)
{
    if (g_readChar) {
        return g_lastChar;
    } else {
        return EOF;
    }
}

int fgetc(FILE* f)
{
    g_lastChar = (int)MXC_UART_ReadCharacter(
        MXC_UARTn); /* Read the byte and save it to global for backspace */
    g_readChar = 1; /* set global to indicate g_lastChar is valid. */
    return g_lastChar;
}

int ferror(FILE* f)
{
    g_readChar = 0;
    return EOF;
}

void _ttywrch(int c)
{
    if (c != '\n') {
        MXC_UART_WriteCharacter(MXC_UARTn, c);
    } else {
        MXC_UART_WriteCharacter(MXC_UARTn, '\r');
        MXC_UART_WriteCharacter(MXC_UARTn, '\n');
    }
}

void _sys_exit(int return_code)
{
    while (1) {
    }
}

#endif /* __CC_ARM  */
