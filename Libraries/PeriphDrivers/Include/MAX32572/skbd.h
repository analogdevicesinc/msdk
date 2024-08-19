/**
 * @file    skbd.h
 * @brief   Secure Keyboard(SKBD) function prototypes and data types.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SKBD_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SKBD_H_

#include <stddef.h>
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "skbd_regs.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup skbd Secure Keyboard (SKBD)
 *
 * @ingroup periphlibs
 *
 * @{
 */

#define xstr(s) str(s)
#define str(s) #s
#define MXC_SKBD_VERS_MAJOR <VERSMAJ>
#define MXC_SKBD_VERS_MINOR <VERSMIN>
#define MXC_SKBD_VERS_PATCH <VERSPAT>
#define MXC_SKBD_VERSION_STRING \
    "v" xstr(MXC_SKBD_VERS_MAJOR) "." xstr(MXC_SKBD_VERS_MINOR) "." xstr(MXC_SKBD_VERS_PATCH)

/* COBRA adaptation */
#define MXC_KEYPAD_BASE_ERR 0 //COBRA_KEYPAD_BASE_ERR
/* Number of key registers present in the keypad interface */
#define MXC_SKBD_TOTAL_KEY_REGS 4

/**
 * @brief   Keypad errors list
 *
 */
typedef enum {
    MXC_SKBD_ERR_MIN = MXC_KEYPAD_BASE_ERR,
    MXC_SKBD_ERR_NOT_INITIALIZED, ///< Error Code: Keypad not initialized
    MXC_SKBD_ERR_ALREAD_INITIALIZED, ///< Error Code: Keypad already initialized
    MXC_SKBD_ERR_INVALID_OPERATION, ///< Error Code: Invalid keypad operation
    MXC_SKBD_ERR_OUT_OF_RANGE, ///< Error Code: Invalid parameter or value
    MXC_SKBD_ERR_OVERRUN, ///< Error Code: Keypad Over run error
    MXC_SKBD_ERR_IRQ, ///< Error Code: IRQ setup error
    MXC_SKBD_ERR_IRQ_NULL, ///< Error Code: NULL IRQ handler
    MXC_SKBD_ERR_INVALID_PIN_CONFIGURATION, ///< Error Code: One or more keypad I/O pins are overlapped or  input/output pin configurations are invalid
    MXC_SKBD_ERR_BUSY, ///< Error Code: Keypad is busy
    MXC_SKBD_ERR_UNKNOWN, ///< Error Code: Generic error for unknown behavior
    MXC_SKBD_ERR_MAX = MXC_SKBD_ERR_UNKNOWN
} mxc_skbd_errors_t;

/**
 * @brief   Keypad initialization state FSM
 *
 */
typedef enum {
    MXC_SKBD_STATE_MIN = 0,
    MXC_SKBD_STATE_NOT_INITIALIZED = MXC_SKBD_STATE_MIN, ///< State not initialized
    MXC_SKBD_STATE_INITIALIZED, ///< State initialized
    MXC_SKBD_STATE_CLOSED, ///< State closed
    MXC_SKBD_STATE_MAX = MXC_SKBD_STATE_CLOSED,
    MXC_SKBD_STATE_COUNT
} mxc_skbd_state_t;

/**
 * @brief   Keypad events
 *
 */
typedef enum {
    MXC_SKBD_EVENT_PUSH = MXC_F_SKBD_INTEN_PUSH, ///< Push Event
    MXC_SKBD_EVENT_RELEASE = MXC_F_SKBD_INTEN_RELEASE, ///< Release Event
    MXC_SKBD_EVENT_OVERRUN = MXC_F_SKBD_INTEN_OVERRUN ///< Overrun Event
} mxc_skbd_events_t;

/**
 * @brief   Keypad Interrupt Status
 *
 */
typedef enum {
    MXC_SKBD_INTERRUPT_STATUS_PUSHIS = MXC_F_SKBD_INTFL_PUSH, ///< Push Interupt flag
    MXC_SKBD_INTERRUPT_STATUS_RELEASEIS = MXC_F_SKBD_INTFL_RELEASE, ///< Release Interupt flag
    MXC_SKBD_INTERRUPT_STATUS_OVERIS = MXC_F_SKBD_INTFL_OVERRUN ///< Overrun Interupt flag
} mxc_interrupt_status_t;

/**
 * @brief   Keypad I/O's IOSEL
 *
 */
typedef enum {
    MXC_SKBD_KBDIO0 = (0x01 << 0), ///< SKBD pin 0
    MXC_SKBD_KBDIO1 = (0x01 << 1), ///< SKBD pin 1
    MXC_SKBD_KBDIO2 = (0x01 << 2), ///< SKBD pin 2
    MXC_SKBD_KBDIO3 = (0x01 << 3), ///< SKBD pin 3
    MXC_SKBD_KBDIO4 = (0x01 << 4), ///< SKBD pin 4
    MXC_SKBD_KBDIO5 = (0x01 << 5), ///< SKBD pin 5
    MXC_SKBD_KBDIO6 = (0x01 << 6), ///< SKBD pin 6
    MXC_SKBD_KBDIO7 = (0x01 << 7), ///< SKBD pin 7
    MXC_SKBD_KBDIO8 = (0x01 << 8), ///< SKBD pin 8
    MXC_SKBD_KBDIO9 = (0x01 << 9), ///< SKBD pin 9
} mxc_skbd_io_pins_t;

/**
 * @brief   Keypad Debounce Time
 *
 */
typedef enum {
    MXC_SKBD_DBTM_TIME4MS = MXC_V_SKBD_CTRL1_DBTM_TIME4MS, ///< SKBD Debounce Time 4ms
    MXC_SKBD_DBTM_TIME5MS = MXC_V_SKBD_CTRL1_DBTM_TIME5MS, ///< SKBD Debounce Time 5ms
    MXC_SKBD_DBTM_TIME6MS = MXC_V_SKBD_CTRL1_DBTM_TIME6MS, ///< SKBD Debounce Time 6ms
    MXC_SKBD_DBTM_TIME7MS = MXC_V_SKBD_CTRL1_DBTM_TIME7MS, ///< SKBD Debounce Time 7ms
    MXC_SKBD_DBTM_TIME8MS = MXC_V_SKBD_CTRL1_DBTM_TIME8MS, ///< SKBD Debounce Time 8ms
    MXC_SKBD_DBTM_TIME10MS = MXC_V_SKBD_CTRL1_DBTM_TIME10MS, ///< SKBD Debounce Time 10ms
    MXC_SKBD_DBTM_TIME11MS = MXC_V_SKBD_CTRL1_DBTM_TIME11MS, ///< SKBD Debounce Time 11ms
    MXC_SKBD_DBTM_TIME12MS = MXC_V_SKBD_CTRL1_DBTM_TIME12MS, ///< SKBD Debounce Time 12ms
} mxc_skbd_debounce_time_t;

/**
 * @brief   Keypad IRQ handler function
 *
 */
typedef void (*irq_handler_t)(void);

/**
 * @brief   Keypad configuration structure
 *
 */
typedef struct {
    uint16_t ioselect; ///< I/O pin direction selection for the corresponding keypad pins
    unsigned int reg_erase; ///< key register erase flag on key is released
    int outputs; ///< Specifies the keypad pins to be configured as output
    int inputs; ///< Specifies the keypad pins to be configured as input
    uint32_t debounce; ///< Keypad Debouncing Time
    irq_handler_t irq_handler; ///< IRQ handler
} mxc_skbd_config_t;

/**
 * @brief   Keypad channel context information
 *
 */
typedef struct {
    unsigned int first_init; ///< 1 - initialize
    unsigned int irq; ///< Interrupt request(IRQ) number
    irq_handler_t irq_handler; ///< IRQ handler
    mxc_skbd_state_t state; ///< keypad initialization state
} mxc_skbd_req_t;

/**
 * @brief   Keyboard Key's scan codes
 *
 */
typedef struct {
    /*
     * key scan code format as follows
     *      key(x) bits[3-0] : Input scan code
     *      key(x) bits[7-4] : Output scan code
     *      key(x) bit[8]    : Next Key Flag
     */
    uint16_t key0; ///< Key0 scan code
    uint16_t key1; ///< Key1 scan code
    uint16_t key2; ///< Key2 scan code
    uint16_t key3; ///< Key3 scan code
} mxc_skbd_keys_t;

/**
 * @brief   Return the version of the SKBD driver
 *
 * @return  const char*
 */
const char *MXC_SKBD_GetVersion(void);

/**
 * @brief   Configure Keypad interrupt
 *
 * @return  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_PreInit(void);

/**
 * @brief   The function is used to initialize the keypad controller
 * @param   [in] config                             Configuration to set
 * @retval  NO_ERROR                                No error
 * @retval  COMMON_ERR_UNKNOWN                      Unknown error
 * @retval  MXC_SKBD_ERR_IRQ_NULL                   Handler is null
 * @retval  MXC_SKBD_ERR_INVALID_PIN_CONFIGURATION  One or more keypad I/O pins are overlapped or
 *                                                  input/output pin configurations are invalid
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_Init(mxc_skbd_config_t config);

/**
 * @brief   Function is used to enable the interrupt events
 * @param   [in] events                             Input interrupt events to set
 * @retval  NO_ERROR                                No error
 * @retval  COMMON_ERR_UNKNOWN                      Unknown error
 * @retval  MXC_SKBD_ERR_NOT_INITIALIZED            SKBD not initialized
 */
int MXC_SKBD_EnableInterruptEvents(unsigned int events);

/**
 * @brief   Function to disable the interrupt events
 * @param   [in] events                             Input interrupt events to disable
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_DisableInterruptEvents(unsigned int events);

/**
 * @brief   Function is used to clear the interrupt events
 * @param   [in] status                             Interrupt status to clear
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_ClearInterruptStatus(unsigned int status);

/**
 * @brief   Function is used to read the interrupt status
 * @param   [in] status                             Interrupt status to clear
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_InterruptStatus(unsigned int *status);

/**
 * @brief   Function to read the key scan codes
 * @param   [in] keys                               Pointer on the keyboard Key's scan codes
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_ReadKeys(mxc_skbd_keys_t *keys);

/**
 * @brief   Function to close the keypad
 * @retval  int                                     see \ref mxc_skbd_errors_t for a list of return codes
 */
int MXC_SKBD_Close(void);

/** @} end of group skbd */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SKBD_H_
