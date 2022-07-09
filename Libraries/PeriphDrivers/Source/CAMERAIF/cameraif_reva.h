/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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


/****** Includes *******/
#include <stdint.h>
#include "cameraif_reva_regs.h"

/***** Definitions *****/

/**
 * @brief   The list of Camera Interface Datawith options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_DATAWITH_8_BIT = 0,      ///<
    MXC_PCIF_REVA_DATAWITH_10_BIT,         ///<
    MXC_PCIF_REVA_DATAWITH_12_BIT,         ///<
} mxc_pcif_reva_datawith_t;

/**
 * @brief   The list of Camera Interface ReadMode options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_READMODE_SINGLE_MODE = 1,      ///<
    MXC_PCIF_REVA_READMODE_CONTINUES_MODE,       ///<
} mxc_pcif_reva_readmode_t;

/**
 * @brief   The list of Camera Interface TimingSel options supported
 *
 */
typedef enum {
    MXC_PCIF_REVA_TIMINGSEL_HSYNC_and_VSYNC = 0,     ///<
    MXC_PCIF_REVA_TIMINGSEL_SAV_and_EAV,             ///<
} mxc_pcif_reva_timingsel_t;

/******* Globals *******/

/****** Functions ******/
int MXC_PCIF_RevA_Init (void);
void MXC_PCIF_RevA_SetDatawidth (mxc_cameraif_reva_regs_t *cameraif, mxc_pcif_reva_datawith_t  datawith);
void MXC_PCIF_RevA_SetTimingSel (mxc_cameraif_reva_regs_t *cameraif, mxc_pcif_reva_timingsel_t timingsel);
void MXC_PCIF_RevA_SetThreshold (mxc_cameraif_reva_regs_t *cameraif, int fifo_thrsh);
void MXC_PCIF_RevA_EnableInt (mxc_cameraif_reva_regs_t *cameraif, uint32_t flags);
void MXC_PCIF_RevA_DisableInt (mxc_cameraif_reva_regs_t *cameraif, uint32_t flags);
void MXC_PCIF_RevA_Start (mxc_cameraif_reva_regs_t *cameraif, mxc_pcif_reva_readmode_t  readmode);
void MXC_PCIF_RevA_Stop (mxc_cameraif_reva_regs_t *cameraif);
unsigned int MXC_PCIF_RevA_GetData (mxc_cameraif_reva_regs_t *cameraif);
