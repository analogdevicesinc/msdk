/**
 * @file    cameraif.h
 * @brief   CAMERAIF function prototypes and data types.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78000_CAMERAIF_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78000_CAMERAIF_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "cameraif_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup cameraif
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/**
 * @brief   The list of Camera Interface Datawidth options supported
 *
 */
typedef enum {
    MXC_PCIF_DATAWIDTH_8_BIT = 0, ///<
    MXC_PCIF_DATAWIDTH_10_BIT, ///<
    MXC_PCIF_DATAWIDTH_12_BIT, ///<
} mxc_pcif_datawidth_t;

/**
 * @brief   The list of Camera GPIO Datawidth options supported
 *
 */
typedef enum {
    MXC_PCIF_GPIO_DATAWIDTH_8_BIT = 0, ///<
    MXC_PCIF_GPIO_DATAWIDTH_10_BIT, ///<
    MXC_PCIF_GPIO_DATAWIDTH_12_BIT, ///<
} mxc_pcif_gpio_datawidth_t;

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
 * @brief Initialize the Parallel Camera Interface.
 *
 * @param gpioDataWidth   Desired datawidth for the camera interface (8, 10 or 12 bits).
 *
 * @return E_NO_ERROR if successful, otherwise E_BAD_PARAM.
 */
int MXC_PCIF_Init(mxc_pcif_gpio_datawidth_t gpioDataWidth);

/**
 * @brief   Set data width for the camera interface.
 *
 * @param  datawidth 8/10/12 bit
 */
void MXC_PCIF_SetDataWidth(mxc_pcif_datawidth_t datawidth);

/**
 * @brief   Set the desired timing mode for the camera interface.
 *
 * @param  timingsel There are two different timing modes. HSYNC/VSYNC and Data Stream.
 */
void MXC_PCIF_SetTimingSel(mxc_pcif_timingsel_t timingsel);

/**
 * @brief  Set camera FIFO threshold.
 *
 * @param  fifo_thrsh Desired FIFO threshold.
 */
void MXC_PCIF_SetThreshold(int fifo_thrsh);

/**
 * @brief   Enable camera interrupts.
 *
 * @param  flags Interrupt flags
 */
void MXC_PCIF_EnableInt(uint32_t flags);

/**
 * @brief  Disable camera interrupts.
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

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78000_CAMERAIF_H_
