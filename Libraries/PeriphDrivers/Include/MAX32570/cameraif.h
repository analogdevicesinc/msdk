/**
 * @file    cameraif.h
 * @brief   CAMERAIF function prototypes and data types.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All rights Reserved.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CAMERAIF_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CAMERAIF_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "cameraif_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup cameraif Camera Interface
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/**
 * @brief   The list of Camera Interface Datawith options supported
 *
 */
typedef enum {
    MXC_PCIF_DATAWIDTH_8_BIT = 0, ///<
    MXC_PCIF_DATAWIDTH_10_BIT, ///<
    MXC_PCIF_DATAWIDTH_12_BIT, ///<
} mxc_pcif_datawidth_t;

/**
 * @brief   The list of Camera Interface ReadMode options supported
 *
 */
typedef enum {
    MXC_PCIF_READMODE_SINGLE_MODE = 1, ///<
    MXC_PCIF_READMODE_CONTINUES_MODE, ///<
} mxc_pcif_readmode_t;

/**
 * @brief   The list of Camera Interface TimingSel options supported
 *
 */
typedef enum {
    MXC_PCIF_TIMINGSEL_HSYNC_and_VSYNC = 0, ///<
    MXC_PCIF_TIMINGSEL_SAV_and_EAV, ///<
} mxc_pcif_timingsel_t;

/* **** Function Prototypes **** */

/**
 * @brief   Initialize camera interface, set clock, configure gpios
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_PCIF_Init(void);

/**
 * @brief   Initialize camera interface, set clock, configure gpios
 *
 * @param  datawidth 8/10/12 bit
 */
void MXC_PCIF_SetDatawidth(mxc_pcif_datawidth_t datawidth);

/**
 * @brief   Initialize camera interface, set clock, configure gpios
 *
 * @param  timingsel There are two different timing modes. HSYNC/VSYNC and Data Stream.
 */
void MXC_PCIF_SetTimingSel(mxc_pcif_timingsel_t timingsel);

/**
 * @brief  Initialize camera interface, set clock, configure gpios
 *
 * @param  fifo_thrsh Interrupt flags
 */
void MXC_PCIF_SetThreshold(int fifo_thrsh);

/**
 * @brief   Initialize camera interface, set clock, configure gpios
 *
 * @param  flags Interrupt flags
 */
void MXC_PCIF_EnableInt(uint32_t flags);

/**
 * @brief   Initialize camera interface, set clock, configure gpios
 *
 * @param  flags Interrupt flags
 */
void MXC_PCIF_DisableInt(uint32_t flags);

/**
 * @brief  Start to capture image from camera interface
 *
 * @param  readmode Single mode or Continues mode
 */
void MXC_PCIF_Start(mxc_pcif_readmode_t readmode);

/**
 * @brief  Stop capture, disable Parallel camera interface
 *
 */
void MXC_PCIF_Stop(void);

/**
 * @brief   Read fifo of PCIF
 *
 * @return  Value of fifo
 */
unsigned int MXC_PCIF_GetData(void);

/**@} end of group cameraif */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_CAMERAIF_H_
