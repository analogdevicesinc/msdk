/**
 * @file    i2s.h
 * @brief   I2S (Inter-Integrated Sound) driver function prototypes and data types.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_I2S_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_I2S_H_

/* **** Includes **** */
#include "mxc_sys.h"
#include "dma.h"
#include "spimss_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spimss SPIMSS
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

typedef enum {
    LEFT_JUSTIFIED = 0,
    RIGHT_JUSTIFIED = 1,
} mxc_i2s_justify_t;

typedef enum {
    STEREO_MODE = 0,
    MONO_MODE = 1,
} mxc_i2s_audio_mode_t;

/** @brief I2S audio directions */
typedef enum {
    AUDIO_OUT = 1,
    AUDIO_IN = 2,
} mxc_i2s_direction_t;

/** @brief I2S Configuration Struct */
typedef struct {
    mxc_i2s_justify_t justify;
    mxc_i2s_audio_mode_t audio_mode;
    mxc_i2s_direction_t audio_direction;
    uint16_t sample_rate;
    unsigned int start_immediately;
    unsigned int dma_reload_en;
    void *src_addr;
    void *dst_addr;
    uint32_t length;
} mxc_i2s_config_t;

/* **** Function Prototypes **** */

/**
 * @brief Initialize I2S resources
 *
 * @param      cfg I2S Configuration Struct
 * @param      dma_ctz_cb Optional function to be called when the DMA completes
 *             a transfer. Set to NULL if unused.
 *
 * @details This initialization is required before using the I2S driver
 * functions.
 * @return #E_NO_ERROR if successful
 */
int MXC_I2S_Init(const mxc_i2s_config_t *cfg, void (*dma_ctz_cb)(int, int));

/**
 * @brief      Release I2S
 * @details    Deconfigures the I2S protocol and stops DMA request
 * @return     #E_BAD_PARAM if DMA cannot be stopped, #E_NO_ERROR otherwise
 */
int MXC_I2S_Shutdown(void);

/**
 * @brief      Mute I2S Output
 * @details    Sets I2S data to zero, continues sending clock and accessing DMA
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Mute(void);

/**
 * @brief      Unmute I2S Output
 * @details    Restores I2S data
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Unmute(void);

/**
 * @brief      Pause I2S Output
 * @details    Similar to mute, but stops FIFO and DMA access, clocks continue
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Pause(void);

/**
 * @brief      Unpause I2S Output
 * @details    Similar to mute, but restarts FIFO and DMA access
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Unpause(void);

/**
 * @brief      Stops I2S Output
 * @details    Similar to pause, but also halts clock
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Stop(void);

/**
 * @brief      Starts I2S Output
 * @details    Starts I2S Output, automatically called by configure if requested
 * @return     #E_NO_ERROR
 */
int MXC_I2S_Start(void);

/**
 * @brief      Clears DMA Interrupt Flags
 * @details    Clears the DMA Interrupt flags, should be called at the end of a dma_ctz_cb
 * @return     #E_NO_ERROR
 */
int MXC_I2S_DMA_ClearFlags(void);

/**
 * @brief      Set DMA Addr (Source or Dest) and bytes to transfer
 * @param      src_addr The address to read data from (Audio Out)
 * @param      dst_addr The address to write data to (Audio In)
 * @param      count    The length of the transfer in bytes
 * @details    Sets the address to read/write data in memory and the length of
 *             the transfer. The unused addr parameter is ignored.
 * @return     #E_NO_ERROR
 */
int MXC_I2S_DMA_SetAddrCnt(void *src_addr, void *dst_addr, unsigned int count);

/**
 * @brief      Sets the DMA reload address and count
 * @param      src_addr The address to read data from (Audio Out)
 * @param      dst_addr The address to write data to (Audio In)
 * @param      count    The length of the transfer in bytes
 * @details    If DMA reload is enabled, when the DMA has transfered $count bytes
 *             (a CTZ event occurs) the src, dst, and count registers will be
 *             set to these. The DMA reload flag clears after a reload occurs.
 * @return     #E_NO_ERROR
 */
int MXC_I2S_DMA_SetReload(void *src_addr, void *dst_addr, unsigned int count);

#ifdef __cplusplus
}
#endif

/**@} end of group i2s */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_I2S_H_
