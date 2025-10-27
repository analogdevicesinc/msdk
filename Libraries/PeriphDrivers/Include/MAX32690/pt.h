/**
 * @file
 * @brief Pulse Train data types, definitions and function prototypes.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PT_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PT_H_

/* **** Includes **** */

#include <stdio.h>
#include "pt.h"
#include "gcr_regs.h"
#include "pt_regs.h"
#include "ptg_regs.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup pulsetrain Pulse Train Engine
 * @ingroup periphlibs
 * @brief This is the high level API for the pulse train engine.
 * @{
 */

/**
 * Structure type for pulse train mode configuration.
 * @note       Do not use for square wave
 */
typedef struct {
    unsigned channel; /**< PT Channel to use */
    uint32_t bps; /**< pulse train bit rate */
    uint32_t pattern; /**< Output pattern to shift out, starts at LSB */
    uint8_t
        ptLength; /**< Number of bits in pulse train, 0 = 32bits, 1 = non valid , 2 = 2 bits, ... */
    uint16_t loop; /**< Number of times to repeat the train, 0 = continuous */
    uint16_t
        loopDelay; /**< Delay between loops specified in bits Example: loopDelay = 4,  delays time  = time it takes to shift out 4 bits */
} mxc_pt_cfg_t;
/**
 * Enumeration type for the system clock scale types
 */
typedef enum {
    MXC_PT_CLK_DIV1 = 0,
    MXC_PT_CLK_DIV2,
    MXC_PT_CLK_DIV4,
    MXC_PT_CLK_DIV8,
    MXC_PT_CLK_DIV16,
    MXC_PT_CLK_DIV32,
    MXC_PT_CLK_DIV64,
    MXC_PT_CLK_DIV128,
} mxc_clk_scale_t;

/**
 * @brief      This function initializes the pulse trains to a known stopped
 *             state and sets the global PT clock scale.
 * @note       If you wish to manage clock settings externally, define the flag
 *             MSDK_NO_GPIO_CLK_INIT.
 * @param      clk_scale  Scale the system clock for the global PT clock. Has no
 *                        effect if MSDK_NO_GPIO_CLK_INIT is defined.
 */
void MXC_PT_Init(mxc_clk_scale_t clk_scale);

/**
 * @brief      Shutdown the pulse train channel/channels.
 * @details    Shutdown pulse train and if all pulse trains are shut down then
 *             turn off pulse train clock. If MSDK_NO_GPIO_CLK_INIT is defined,
 *             pulse train clock remains enabled.
 * @note       Shutdown pulse train channel/channels and delete config.
 *
 * @param      pts    Pulse train channel to operate on.
 */
void MXC_PT_Shutdown(uint32_t pts);

/**
 * @brief      Configures the pulse train in the specified mode.
 * @details    The parameters in the config structure must be set before calling
 *             this function. This function should be used for configuring pulse
 *             train mode only. If MSDK_NO_GPIO_CLK_INIT is defined, GPIO
 *             configuration is skipped.
 * @note       The pulse train cannot be running when this function is called.
 *
 * @param      cfg     Pointer to pulse train configuration.
 *
 * @return     #E_NO_ERROR if everything is successful, @ref MXC_Error_Codes
 *             "error" if unsuccessful.
 */
int MXC_PT_Config(mxc_pt_cfg_t *cfg);

/**
 * @brief      Configures the pulse train in the square wave mode.
 * @details    This function should be used for configuring square wave mode only.
 * @note       The pulse train cannot be running when this function is called
 *
 * @param      channel Pulse train channel to operate on
 * @param      freq    square wave output frequency in Hz
 *
 * @returns #E_NO_ERROR if everything is successful, \ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_PT_SqrWaveConfig(unsigned channel, uint32_t freq);

/**
 * @brief      Starts the pulse trains specified.
 *
 * @param      pts   Pulse train pts to operate on.
 */
void MXC_PT_Start(unsigned pts);

/**
 * @brief      Stops pulse trains.
 *
 * @param      pts Pulse train pts to stop.
 */
void MXC_PT_Stop(unsigned pts);

/**
 * @brief      Determines if the pulse trains selected are running
 *
 * @param      pts   Set the bits of pulse trains to check Bit0-\>pt0,
 *                   Bit1-\>pt1... etc.
 *
 * @return     0            All pulse trains are off.
 * @return     \>0          At least one pulse train is on.
 */
uint32_t MXC_PT_IsActive(uint32_t pts);

/**
 * @brief      Sets the pattern of the pulse train
 *
 * @param      pts     Pulse train pts to operate on.
 * @param      pattern Output pattern.
 *
 */
void MXC_PT_SetPattern(unsigned pts, uint32_t pattern);

/**
 * @brief      Enable Stop interrupts for the pulse trains selected.
 * 
 * @note       This is just an explicit version of MXC_PT_EnableInt()
 *
 * @param      pts   Bit mask of which pulse trains to enable. Set the bit
 *                   position of each pulse train to enable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will enable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
void MXC_PT_EnableStopInt(uint32_t pts);

/**
 * @brief      Disable Stop interrupts for the pulse trains selected.
 * 
 * @note       This is an explicit version of MXC_PT_DisableInt()
 *
 * @param      pts   Bit mask of what pulse trains to disable. Set the bit
 *                   position of each pulse train to disable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will disable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
void MXC_PT_DisableStopInt(uint32_t pts);

/**
 * @brief      Gets the pulse trains's Stop interrupt flags.
 * 
 * @note       This is an explicit version of MXC_PT_GetStopFlags()
 *
 * @return     The Pulse Train Interrupt Flags, \ref MXC_PT_STOP_INTFL_Register Register
 *             for details.
 */
uint32_t MXC_PT_GetStopFlags(void);

/**
 * @brief      Clears the pulse train's Stop interrupt flag.
 * 
 * @note       This is an explicit version of MXC_PT_ClearStopFlags()
 *
 * @param      flags  bits to clear, see \ref MXC_PT_STOP_INTFL_Register Register for details.
 */
void MXC_PT_ClearStopFlags(uint32_t flags);

#if defined(__GNUC__)
/**
 * @brief      Enable Stop interrupts for the pulse trains selected.
 * 
 * @note       This function is deprecated. Use MXC_PT_EnableStopInt instead to
 *             differentiate between the STOP and READY interrupts.
 *
 * @param      pts   Bit mask of which pulse trains to enable. Set the bit
 *                   position of each pulse train to enable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will enable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
inline
    __attribute__((deprecated("Use MXC_PT_EnableStopInt instead.  See pt.h for more details."))) void
    MXC_PT_EnableInt(uint32_t pts)
{
    MXC_PT_EnableStopInt(pts);
}

/**
 * @brief      Disable Stop interrupts for the pulse trains selected.
 * 
 * @note       This function is deprecated. Use MXC_PT_DisableStopInt instead to
 *             differentiate between the STOP and READY interrupts.
 *
 * @param      pts   Bit mask of what pulse trains to disable. Set the bit
 *                   position of each pulse train to disable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will disable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
inline __attribute__((
    deprecated("Use MXC_PT_DisableStopInt instead.  See pt.h for more details."))) void
MXC_PT_DisableInt(uint32_t pts)
{
    MXC_PT_DisableStopInt(pts);
}

/**
 * @brief      Gets the pulse trains's Stop interrupt flags.
 * 
 * @note       This function is deprecated. Use MXC_PT_GetStopFlags instead to
 *             differentiate between the STOP and READY interrupts.
 *
 * @return     The Pulse Train Interrupt Flags, \ref MXC_PT_STOP_INTFL_Register Register
 *             for details.
 */
inline __attribute__((deprecated("Use MXC_PT_GetStopFlags instead.  See pt.h for more details.")))
uint32_t
MXC_PT_GetFlags(void)
{
    return MXC_PT_GetStopFlags();
}

/**
 * @brief      Clears the pulse train's Stop interrupt flag.
 * 
 * @note       This function is deprecated. Use MXC_PT_ClearStopFlags instead to
 *             differentiate between the STOP and READY interrupts.
 *
 * @param      flags  bits to clear, see \ref MXC_PT_STOP_INTFL_Register Register for details.
 */
inline __attribute__((
    deprecated("Use MXC_PT_ClearStopFlags instead.  See pt.h for more details."))) void
MXC_PT_ClearFlags(uint32_t flags)
{
    MXC_PT_ClearStopFlags(flags);
}
#endif

/**
 * @brief      Setup and enables a pulse train to restart after another pulse
 *             train has exited its loop. Each pulse train can have up to two
 *             restart triggers.
 *
 * @param      start         Pulse train channel to start.
 * @param      stop          Pulse train channel to stop.
 * @param      restartIndex  selects which restart trigger to set (0 or 1).
 */
void MXC_PT_EnableRestart(unsigned start, unsigned stop, uint8_t restartIndex);

/**
 * @brief      Disable the restart for the specified pulse train
 *
 * @param      channel       Pulse train channel
 * @param      restartIndex  selects which restart trigger to disable (0 or 1)
 */
void MXC_PT_DisableRestart(unsigned channel, uint8_t restartIndex);

/**
 * @brief      Resynchronize individual pulse trains together. Resync will stop
 *             those resync_pts; others will be still running
 *
 * @param      pts   Pulse train modules that need to be re-synced by bit
 *                        number. Bit0-\>pt0, Bit1-\>pt1... etc.
 */
void MXC_PT_Resync(uint32_t pts);

/**
 * @brief      Enable Ready interrupts for the pulse trains selected.
 *
 * @param      pts   Bit mask of which pulse trains to enable. Set the bit
 *                   position of each pulse train to enable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will enable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
void MXC_PT_EnableReadyInt(uint32_t pts);

/**
 * @brief      Disable Ready interrupts for the pulse trains selected.
 *
 * @param      pts   Bit mask of what pulse trains to disable. Set the bit
 *                   position of each pulse train to disable it. Bit0-\>pt0,
 *                   Bit1-\>pt1... etc, 1 will disable the interrupt, 0 to leave
 *                   a PT channel in its current state.
 */
void MXC_PT_DisableReadyInt(uint32_t pts);

/**
 * @brief      Gets the pulse trains's Ready interrupt flags.
 *
 * @return     The Pulse Train Interrupt Flags, \ref MXC_PT_READY_INTFL_Register Register
 *             for details.
 */
uint32_t MXC_PT_GetReadyFlags(void);

/**
 * @brief      Clears the pulse train's Ready interrupt flag.
 *
 * @param      flags  bits to clear, see \ref MXC_PT_READY_INTFL_Register Register for details.
 */
void MXC_PT_ClearReadyFlags(uint32_t flags);

/**@} end of group pulsetrains*/

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PT_H_
