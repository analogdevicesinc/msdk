/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief      I2S driver implementation.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include "pal_sys.h"
#include "pal_twi.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "i2c.h"
#include "i2c_regs.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Allow compiler directive override. */
#ifndef PAL_TWI_MAX_DEVICE
#define PAL_TWI_MAX_DEVICE 4
#endif

/*! \brief TWI instance ID. */
#ifndef PAL_TWI_INSTANCE_ID
#define PAL_TWI_INSTANCE_ID 0
#endif

#ifndef PAL_TWI_FREQ
#define PAL_TWI_FREQ 400000
#endif

/*! \brief      Get next handle value, includes wrap around. */
#define PAL_TWI_GET_NEXT_HANDLE(h) (((h) + 1) & (PAL_TWI_MAX_DEVICE - 1))

#ifdef DEBUG

/*! \brief      Parameter check. */
#define PAL_TWI_PARAM_CHECK(expr)                    \
    {                                                \
        if (!(expr)) {                               \
            palTwiCb.drvState = PAL_TWI_STATE_ERROR; \
            return;                                  \
        }                                            \
    }

/*! \brief      Parameter check, with return value. */
#define PAL_TWI_PARAM_CHECK_RET(expr, rv)            \
    {                                                \
        if (!(expr)) {                               \
            palTwiCb.drvState = PAL_TWI_STATE_ERROR; \
            return (rv);                             \
        }                                            \
    }

#else

/*! \brief      Parameter check (disabled). */
#define PAL_TWI_PARAM_CHECK(expr)

/*! \brief      Parameter check, with return value (disabled). */
#define PAL_TWI_PARAM_CHECK_RET(expr, rv)

#endif

/**************************************************************************************************
  Type Definitions
**************************************************************************************************/

/*! \brief      Commands state. */
typedef enum {
    PAL_TWI_CMD_IDLE, /*!< Idle state. */
    PAL_TWI_CMD_TX_DATA, /*!< Write data state. */
    PAL_TWI_CMD_RX_DATA /*!< Read data state. */
} PalTwiCmdState_t;

/*! \brief      Device configuration. */
typedef struct {
    bool_t opPending; /*!< Operation pending flag. */
    PalTwiDevConfig_t devCfg; /*!< Device configuration. */
} PalTwiDevCtx_t;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

/*************************************************************************************************/
void I2C0_IRQHandler(void)
{
    MXC_I2C_AsyncHandler(MXC_I2C0);
}

/*************************************************************************************************/
void I2C1_IRQHandler(void)
{
    MXC_I2C_AsyncHandler(MXC_I2C1);
}

/*************************************************************************************************/
void I2C2_IRQHandler(void)
{
    MXC_I2C_AsyncHandler(MXC_I2C2);
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize TWI resources.
 */
/*************************************************************************************************/
void PalTwiInit(void) {}

/*************************************************************************************************/
/*!
 *  \brief      De-Initialize the TWI resources.
 */
/*************************************************************************************************/
void PalTwiDeInit(void) {}

/*************************************************************************************************/
/*!
 *  \brief      Register a device on the TWI bus.
 *
 *  \param      pDevCfg     Device configuration.
 *
 *  \return     Device handle.
 */
/*************************************************************************************************/
uint8_t PalTwiRegisterDevice(PalTwiDevConfig_t *pDevCfg)
{
    return 0;
}

/**************************************************************************************************
  Functions: Control and Status
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Get the current state.
 *
 *  \return     Current state.
 *
 *  Return the current state of the TWI.
 */
/*************************************************************************************************/
PalTwiState_t PalTwiGetState(void)
{
    return PAL_TWI_STATE_UNINIT;
}

/**************************************************************************************************
  Functions: Data Transfer
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Always start an operation before reading or writing on TWI interface.
 *
 *  \param      handle      Device handle.
 */
/*************************************************************************************************/
void PalTwiStartOperation(uint8_t handle) {}

/*************************************************************************************************/
/*!
 *  \brief      Always stop an operation after reading or writing on TWI interface.
 *
 *  \param      handle      Device handle.
 */
/*************************************************************************************************/
void PalTwiStopOperation(uint8_t handle) {}

/*************************************************************************************************/
/*!
 *  \brief      Read data from TWI interface.
 *
 *  \param      handle      Device handle.
 *  \param      pData       Read buffer.
 *  \param      len         Number of bytes to write.
 *
 *  Read \a len bytes from \a pData to the TWI device.
 */
/*************************************************************************************************/
void PalTwiReadData(uint8_t handle, uint8_t *pData, uint8_t len) {}

/*************************************************************************************************/
/*!
 *  \brief      Write data to TWI interface.
 *
 *  \param      handle      Device handle.
 *  \param      pData       Write buffer.
 *  \param      len         Number of bytes to write.
 *
 *  Transfer \a len bytes from \a pData to the TWI device.
 */
/*************************************************************************************************/
void PalTwiWriteData(uint8_t handle, const uint8_t *pData, uint8_t len) {}
