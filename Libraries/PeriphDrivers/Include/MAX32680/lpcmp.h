/**
 * @file    lpcmp.h
 * @brief   Low Power Comparator(LPCMP) function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_LPCMP_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_LPCMP_H_

/* **** Includes **** */
#include <stdint.h>
#include "lpcmp_regs.h"
#include "mcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Comparator polarity select
 * 
 */
typedef enum {
    MXC_LPCMP_POL_RISE, //< Comparator interrupt happens on rising edge of comparator output
    MXC_LPCMP_POL_FALL, //< Comparator interrupt occurs on falling edge of comparator output
} mxc_lpcmp_polarity_t;

/**
 * @brief Comparator select
 * 
 */
typedef enum {
    MXC_LPCMP_CMP0 =
        0, //< Comparator output high when positive input is greater than negative input
    MXC_LPCMP_CMP1 =
        1, //< Comparator output high when negative input is greater than positive input
    MXC_LPCMP_CMP2 =
        2, //< Comparator output high when negative input is greater than positive input
    MXC_LPCMP_CMP3 =
        3, //< Comparator output high when negative input is greater than positive input
} mxc_lpcmp_cmpsel_t;

/**
 * @brief Typedef for pointer to comparator control register(s)
 */
typedef volatile uint32_t *mxc_lpcmp_ctrl_reg_t;

/**
 * @brief enables comparator
 * 
 * @param cmp   Selects the comparator to enable
 * 
 * @return int  \ref MXC_Error_Codes for the list of error codes
 */
int MXC_LPCMP_Init(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief Shutdown comparator
 * 
 * @param cmp   Selects the comparator to disable
 * 
 * @return int  \ref MXC_Error_Codes for the list of error codes
 */
int MXC_LPCMP_Shutdown(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief Enable interrupts
 * 
 * @param cmp   Selects the comparator to enable interrupt for
 * @param pol   Selects polarity of the interrupt
 */
int MXC_LPCMP_EnableInt(mxc_lpcmp_cmpsel_t cmp, mxc_lpcmp_polarity_t pol);

/**
 * @brief Disable interrupts
 * 
 * @param cmp   Selects the comparator to disable interrupt for
 */
int MXC_LPCMP_DisableInt(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief Returns the interrupt flags set
 * 
 * @param cmp   Selects the comparator to get interrupt falg status for 
 * 
 * @return int  interrupt flag status (1 if set, 0 if cleared)
 */
int MXC_LPCMP_GetFlags(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief Clear interrupt flags
 * 
 * @param cmp   Selects the comparator to clear interrupt flag for
 */
int MXC_LPCMP_ClearFlags(mxc_lpcmp_cmpsel_t cmp);

/**
 * @brief Select polatity
 * 
 * @param cmp   Selects the comparator to select polarity for
 * @param pol   \ref mxc_lpcmp_polarity_t
 */
int MXC_LPCMP_SelectPolarity(mxc_lpcmp_cmpsel_t cmp, mxc_lpcmp_polarity_t pol);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_LPCMP_H_
