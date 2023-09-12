/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
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
*******************************************************************************
*/

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "logging.h"

/**
 * @brief Controls the logging level for the L1 Stack
 *
 * Stack Logging Level
 *
 * Defaults to output logging level messages only
 */
int32_t g_logging_level = DBG_LVL_LOG;

/**
 * @brief Conditionally print some logging information
 *
 * This function is called by @ref NFC_PCD_EMV_LVL1_STACK_LOGGING_MACROS to implement multilevel debugging messages
 *
 * @param[in]   req_level Level for this logging message, use one of @ref NFC_PCD_EMV_LVL1_STACK_LOGGING_LVLS
 * @param[in]   ... printf style formating string and variables to print etc.
 */
void do_log(int32_t req_level, ...)
{
    va_list ap;

    if (req_level <= g_logging_level) {
        va_start(ap, req_level);
        const char *fmt = va_arg(ap, const char *);
        vprintf(fmt, ap);
        va_end(ap);
    }
}
