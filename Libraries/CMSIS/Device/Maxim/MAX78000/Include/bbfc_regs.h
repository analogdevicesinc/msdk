/**
 * @file    bbfc_regs.h
 * @brief   The BBFC block has been renamed to GCFR.  
 *          This include file is for backwards compatibility only.
 * @note    This file is @deprecated.
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_

// The BBFC block has been renamed to GCFR.  
// This include file is for backwards compatibility only.

// Warning only pops up when a project builds with this file included.
#warning "MXC_BBFC (bbfc_regs.h) name is deprecated. Please use MXC_GCFR (gcfr_regs.h)."

#include "gcfr_regs.h"
#define MXC_BBFC MXC_GCFR

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_
