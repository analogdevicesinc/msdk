/**
 * @file    owm.h
 * @brief   Registers, Bit Masks and Bit Positions for the 1-Wire Master
 *          peripheral module.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_OWM_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_OWM_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "owm_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup periphlibs
 * @defgroup owm 1-Wire Master (OWM)
 * @{
 */

/* **** Definitions **** */

/**
 * @brief   Enumeration type for specifying options for 1-Wire external pullup mode.
 */
typedef enum {
    MXC_OWM_EXT_PU_ACT_HIGH = 0, /**< Pullup pin is active high when enabled.        */
    MXC_OWM_EXT_PU_ACT_LOW = 1, /**< Pullup pin is active low when enabled.         */
    MXC_OWM_EXT_PU_UNUSED = 2, /**< Pullup pin is not used for an external pullup. */
} mxc_owm_ext_pu_t;

/**
 * @brief   Structure type for 1-Wire Master configuration.
 */
typedef struct {
    uint8_t int_pu_en; /**< 1 = internal pullup on.   */
    mxc_owm_ext_pu_t ext_pu_mode; /**< See #mxc_owm_ext_pu_t.   */
    uint8_t long_line_mode; /**< 1 = long line mode enable.    */
    // mxc_owm_overdrive_t overdrive_spec; /**< 0 = timeslot is 12us, 1 = timeslot is 10us.   */
} mxc_owm_cfg_t;

#define READ_ROM_COMMAND 0x33 /**< Read ROM Command */
#define MATCH_ROM_COMMAND 0x55 /**< Match ROM Command */
#define SEARCH_ROM_COMMAND 0xF0 /**< Search ROM Command */
#define SKIP_ROM_COMMAND 0xCC /**< Skip ROM Command */
#define OD_SKIP_ROM_COMMAND 0x3C /**< Overdrive Skip ROM Command */
#define OD_MATCH_ROM_COMMAND 0x69 /**< Overdrive Match ROM Command */
#define RESUME_COMMAND 0xA5 /**< Resume Command */

/* **** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief   Initialize and enable OWM module.
 * @param   cfg         Pointer to OWM configuration.
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_NULL_PTR if parameter is a null pointer
 * @return  #E_BUSY if IOMAN was not configured correctly
 * @return  #E_UNINITIALIZED if OWM CLK disabled
 * @return  #E_NOT_SUPPORTED if 1MHz CLK cannot be created with given system and owm CLK
 * @return  #E_BAD_PARAM if bad cfg parameter passed in
 */
int MXC_OWM_Init(const mxc_owm_cfg_t *cfg);

/**
 * @brief   Shutdown OWM module.
 *
 */
void MXC_OWM_Shutdown(void);

/**
 * @brief   Send 1-Wire reset pulse. Will block until transaction is complete.
 *
 * @return  0 if no 1-wire devices reponded during the presence pulse, 1 otherwise
 */
int MXC_OWM_Reset(void);

/**
 * @brief   Get the presence pulse detect status.
 * 
 * @return  0 if no 1-wire devices reponded during the presence pulse, 1 otherwise
 */
int MXC_OWM_GetPresenceDetect(void);

/**
 * @brief   Send and receive one byte of data. Will block until transaction is complete.
 *
 * @param   data        data to send
 *
 * @return  data read (1 byte)
 */
int MXC_OWM_TouchByte(uint8_t data);

/**
 * @brief   Write one byte of data. Will block until transaction is complete.
 *
 * @param   data        data to send
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if data written != data parameter
 */
int MXC_OWM_WriteByte(uint8_t data);

/**
 * @brief   Read one byte of data. Will block until transaction is complete.
 *
 * @return  data read (1 byte)
 */
int MXC_OWM_ReadByte(void);

/**
 * @brief   Send and receive one bit of data. Will block until transaction is complete.
 *
 * @param   bit         bit to send
 *
 * @return  bit read
 */
int MXC_OWM_TouchBit(uint8_t bit);

/**
 * @brief   Write one bit of data. Will block until transaction is complete.
 *
 * @param   bit         bit to send
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if bit written != bit parameter
 */
int MXC_OWM_WriteBit(uint8_t bit);

/**
 * @brief   Read one bit of data. Will block until transaction is complete.
 *
 * @return  bit read
 */
int MXC_OWM_ReadBit(void);

/**
 * @brief   Write multiple bytes of data. Will block until transaction is complete.
 *
 * @param   data    Pointer to buffer for write data.
 * @param   len     Number of bytes to write.
 *
 * @return  Number of bytes written if successful
 * @return  #E_COMM_ERR if line short detected before transaction
 */
int MXC_OWM_Write(uint8_t *data, int len);

/**
 * @brief   Read multiple bytes of data. Will block until transaction is complete.
 *
 * @param   data    Pointer to buffer for read data.
 * @param   len     Number of bytes to read.
 *
 * @return Number of bytes read if successful
 * @return #E_COMM_ERR if line short detected before transaction
 */
int MXC_OWM_Read(uint8_t *data, int len);

/**
 * @brief   Starts 1-Wire communication with Read ROM command
 * @note    Only use the Read ROM command with one slave on the bus
 *
 * @param   ROMCode     Pointer to buffer for ROM code read
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset, read or write fails
 */
int MXC_OWM_ReadROM(uint8_t *ROMCode);

/**
 * @brief   Starts 1-Wire communication with Match ROM command
 *
 * @param   ROMCode     Pointer to buffer with ROM code to match
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset or write fails
 */
int MXC_OWM_MatchROM(uint8_t *ROMCode);

/**
 * @brief   Starts 1-Wire communication with Overdrive Match ROM command
 * @note    After Overdrive Match ROM command is sent, the OWM is set to
 *          overdrive speed. To set back to standard speed use MXC_OWM_SetOverdrive.
 *
 * @param   ROMCode     Pointer to buffer with ROM code to match
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset or write fails
 */
int MXC_OWM_ODMatchROM(uint8_t *ROMCode);

/**
 * @brief   Starts 1-Wire communication with Skip ROM command
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset or write fails
 */
int MXC_OWM_SkipROM(void);

/**
 * @brief   Starts 1-Wire communication with Overdrive Skip ROM command
 * @note    After Overdrive Skip ROM command is sent, the OWM is set to
 *          overdrive speed. To set back to standard speed use MXC_OWM_SetOverdrive
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset or write fails
 */
int MXC_OWM_ODSkipROM(void);

/**
 * @brief   Starts 1-Wire communication with Resume command
 *
 * @return  #E_NO_ERROR if everything is successful
 * @return  #E_COMM_ERR if reset or write fails
 */
int MXC_OWM_Resume(void);

/**
 * @brief   Starts 1-Wire communication with Search ROM command
 *
 * @param   newSearch   (1) = start new search, (0) = continue search for next ROM
 * @param   ROMCode     Pointer to buffer with ROM code found
 *
 * @return  (1) = ROM found, (0) = no new ROM found, end of search
 */
int MXC_OWM_SearchROM(int newSearch, uint8_t *ROMCode);

/**
 * @brief   Clear interrupt flags.
 *
 * @param   mask        Mask of interrupts to clear.
 */
void MXC_OWM_ClearFlags(uint32_t mask);

/**
 * @brief   Get interrupt flags.
 *
 * @return  Mask of active flags.
 */
unsigned MXC_OWM_GetFlags(void);

/**
 * @brief   Enables/Disables the External pullup
 *
 * @param   enable      (1) = enable, (0) = disable
 */
void MXC_OWM_SetExtPullup(int enable);

/**
 * @brief   Enables/Disables Overdrive speed
 *
 * @param   enable      (1) = overdrive, (0) = standard
 */
void MXC_OWM_SetOverdrive(int enable);

/**
 * @brief   Enables interrupts
 *
 * @param   flags      which owm interrupts to enable
 */
void MXC_OWM_EnableInt(int flags);

/**
 * @brief   Disables interrupts
 *
 * @param   flags      which owm interrupts to disable
 */
void MXC_OWM_DisableInt(int flags);

/**
 * @brief   Enables/Disables driving of OWM_IO low during presence detection
 *
 * @param   enable      (1) = enable, (0) = disable
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_SetForcePresenceDetect(int enable);

/**
 * @brief   Enables/Disables the Internal pullup
 *
 * @param   enable      (1) = enable, (0) = disable
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_SetInternalPullup(int enable);

/**
 * @brief   Enables/Disables the External pullup
 *
 * @param   ext_pu_mode  See mxc_owm_ext_pu_t for values
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_SetExternalPullup(mxc_owm_ext_pu_t ext_pu_mode);

/**
 * @brief   Call to correct divider if system clock has changed
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_SystemClockUpdated(void);

/**
 * @brief   Enable/Disable Search ROM Accelerator mode
 *
 * @param   enable      (1) = enable, (0) = disable
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_SetSearchROMAccelerator(int enable);

/**
 * @brief   Prepare OWM for bit bang mode
 *
 * @param   initialState  Starting value of owm
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_BitBang_Init(int initialState);

/**
 * @brief   Read current value of wire
 *
 * @return  Value of wire
 */
int MXC_OWM_BitBang_Read(void);

/**
 * @brief   Set value of wire
 *
 * @param   state       Value to drive wire to
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_BitBang_Write(int state);

/**
 * @brief   Disable Bit Bang mode
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_OWM_BitBang_Disable(void);

/**@} end of group owm */
#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_OWM_H_
