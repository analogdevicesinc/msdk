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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SKBD_SKBD_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SKBD_SKBD_REVA_H_

#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "skbd.h"
#include "skbd_reva_regs.h"

#define xstr(s) str(s)
#define str(s) #s
#define MXC_SKBD_REVA_VERS_MAJOR <VERSMAJ>
#define MXC_SKBD_REVA_VERS_MINOR <VERSMIN>
#define MXC_SKBD_REVA_VERS_PATCH <VERSPAT>
#define MXC_SKBD_REVA_VERSION_STRING                                                \
    "v" xstr(MXC_SKBD_REVA_VERS_MAJOR) "." xstr(MXC_SKBD_REVA_VERS_MINOR) "." xstr( \
        MXC_SKBD_REVA_VERS_PATCH)

/* COBRA adaptation */
#define MXC_KEYPAD_REVA_BASE_ERR 0 //COBRA_KEYPAD_BASE_ERR
/* Number of key registers present in the keypad interface */
#define MXC_SKBD_REVA_TOTAL_KEY_REGS 4

/**
 * @brief   Keypad errors list
 *
 */
typedef enum {
    MXC_SKBD_REVA_ERR_MIN = MXC_KEYPAD_REVA_BASE_ERR,
    MXC_SKBD_REVA_ERR_NOT_INITIALIZED, ///< Error Code: Keypad not initialized
    MXC_SKBD_REVA_ERR_ALREAD_INITIALIZED, ///< Error Code: Keypad already initialized
    MXC_SKBD_REVA_ERR_INVALID_OPERATION, ///< Error Code: Invalid keypad operation
    MXC_SKBD_REVA_ERR_OUT_OF_RANGE, ///< Error Code: Invalid parameter or value
    MXC_SKBD_REVA_ERR_OVERRUN, ///< Error Code: Keypad Over run error
    MXC_SKBD_REVA_ERR_IRQ, ///< Error Code: IRQ setup error
    MXC_SKBD_REVA_ERR_IRQ_NULL, ///< Error Code: NULL IRQ handler
    MXC_SKBD_REVA_ERR_INVALID_PIN_CONFIGURATION, ///< Error Code: One or more keypad I/O pins are overlapped or  input/output pin configurations are invalid
    MXC_SKBD_REVA_ERR_BUSY, ///< Error Code: Keypad is busy
    MXC_SKBD_REVA_ERR_UNKNOWN, ///< Error Code: Generic error for unknown behavior
    MXC_SKBD_REVA_ERR_MAX = MXC_SKBD_REVA_ERR_UNKNOWN
} mxc_skbd_reva_errors_t;

/**
 * @brief   Keypad initialization state FSM
 *
 */
typedef enum {
    MXC_SKBD_REVA_STATE_MIN = 0,
    MXC_SKBD_REVA_STATE_NOT_INITIALIZED = MXC_SKBD_REVA_STATE_MIN, ///< State not initialized
    MXC_SKBD_REVA_STATE_INITIALIZED, ///< State initialized
    MXC_SKBD_REVA_STATE_CLOSED, ///< State closed
    MXC_SKBD_REVA_STATE_MAX = MXC_SKBD_REVA_STATE_CLOSED,
    MXC_SKBD_REVA_STATE_COUNT
} mxc_skbd_reva_state_t;

/**
 * @brief   Keypad events
 *
 */
typedef enum {
    MXC_SKBD_REVA_EVENT_PUSH = MXC_F_SKBD_REVA_IER_PUSHIE, ///< Push Event
    MXC_SKBD_REVA_EVENT_RELEASE = MXC_F_SKBD_REVA_IER_RELEASEIE, ///< Release Event
    MXC_SKBD_REVA_EVENT_OVERRUN = MXC_F_SKBD_REVA_IER_OVERIE ///< Overrun Event
} mxc_skbd_reva_events_t;

/**
 * @brief   Keypad Interrupt Status
 *
 */
typedef enum {
    MXC_SKBD_REVA_INTERRUPT_STATUS_PUSHIS = MXC_F_SKBD_REVA_ISR_PUSHIS, ///< Push Interupt flag
    MXC_SKBD_REVA_INTERRUPT_STATUS_RELEASEIS =
        MXC_F_SKBD_REVA_ISR_RELEASEIS, ///< Release Interupt flag
    MXC_SKBD_REVA_INTERRUPT_STATUS_OVERIS = MXC_F_SKBD_REVA_ISR_OVERIS ///< Overrun Interupt flag
} mxc_skbd_reva_interrupt_status_t;

/**
 * @brief   Keypad I/O's IOSEL
 *
 */
typedef enum {
    MXC_SKBD_REVA_KBDIO0 = (0x01 << 0), ///< SKBD pin 0
    MXC_SKBD_REVA_KBDIO1 = (0x01 << 1), ///< SKBD pin 1
    MXC_SKBD_REVA_KBDIO2 = (0x01 << 2), ///< SKBD pin 2
    MXC_SKBD_REVA_KBDIO3 = (0x01 << 3), ///< SKBD pin 3
    MXC_SKBD_REVA_KBDIO4 = (0x01 << 4), ///< SKBD pin 4
    MXC_SKBD_REVA_KBDIO5 = (0x01 << 5), ///< SKBD pin 5
    MXC_SKBD_REVA_KBDIO6 = (0x01 << 6), ///< SKBD pin 6
    MXC_SKBD_REVA_KBDIO7 = (0x01 << 7), ///< SKBD pin 7
    MXC_SKBD_REVA_KBDIO8 = (0x01 << 8), ///< SKBD pin 8
    MXC_SKBD_REVA_KBDIO9 = (0x01 << 9), ///< SKBD pin 9
} mxc_skbd_reva_io_pins_t;

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
} mxc_skbd_reva_config_t;

/**
 * @brief   Keypad channel context information
 *
 */
typedef struct {
    unsigned int first_init; ///< 1 - initialize
    unsigned int irq; ///< Interrupt request(IRQ) number
    irq_handler_t irq_handler; ///< IRQ handler
    mxc_skbd_reva_state_t state; ///< keypad initialization state
} mxc_skbd_reva_req_t;

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
    uint16_t key0_reva; ///< Key0 scan code
    uint16_t key1_reva; ///< Key1 scan code
    uint16_t key2_reva; ///< Key2 scan code
    uint16_t key3_reva; ///< Key3 scan code
} mxc_skbd_reva_keys_t;

int MXC_SKBD_RevA_PreInit(void);

int MXC_SKBD_RevA_Init(mxc_skbd_reva_regs_t *skbd, mxc_skbd_config_t config);

int MXC_SKBD_RevA_EnableInterruptEvents(mxc_skbd_reva_regs_t *skbd, unsigned int events);

int MXC_SKBD_RevA_DisableInterruptEvents(mxc_skbd_reva_regs_t *skbd, unsigned int events);

int MXC_SKBD_RevA_ClearInterruptStatus(mxc_skbd_reva_regs_t *skbd, unsigned int status);

int MXC_SKBD_RevA_InterruptStatus(mxc_skbd_reva_regs_t *skbd, unsigned int *status);

int MXC_SKBD_RevA_ReadKeys(mxc_skbd_reva_regs_t *skbd, mxc_skbd_reva_keys_t *keys);

int MXC_SKBD_RevA_Close(void);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SKBD_SKBD_REVA_H_
