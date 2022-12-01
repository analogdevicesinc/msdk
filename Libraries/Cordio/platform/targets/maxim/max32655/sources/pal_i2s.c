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
#include "pal_i2s.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifdef DEBUG

/*! \brief      Parameter check. */
#define PAL_I2S_PARAM_CHECK(expr)           { if (!(expr)) { palI2sCb.state = PAL_I2S_STATE_ERROR; return; } }

/*! \brief      Parameter check, with return value. */
#define PAL_I2S_PARAM_CHECK_RET(expr, rv)   { if (!(expr)) { palI2sCb.state = PAL_I2S_STATE_ERROR; return (rv); } }

#else

/*! \brief      Parameter check (disabled). */
#define PAL_I2S_PARAM_CHECK(expr)

/*! \brief      Parameter check, with return value (disabled). */
#define PAL_I2S_PARAM_CHECK_RET(expr, rv)

#endif

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief Control block. */
struct {
  PalI2sState_t      state:8;       /*!< Current state. */
  PalI2sCompCback_t  compCback;     /*!< Completion callback. */
  void               *pCtx;         /*!< Client operation context. */
} palI2sCb;

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

/**************************************************************************************************
  Global Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Initialize I2S resources.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sInit(void)
{

}

/*************************************************************************************************/
/*!
 *  \brief      De-initialize I2S resource.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sDeInit(void)
{

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
 *  Return the current state of the I2S driver.
 */
/*************************************************************************************************/
PalI2sState_t PalI2sGetState(void)
{
  return palI2sCb.state;
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize I2S configuration.
 *
 *  \param      pCfg         Pointer to I2s Configuration.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sConfig(PalI2sConfig_t *pCfg)
{

}

/*************************************************************************************************/
/*!
 *  \brief      De-initialize I2S configuration.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sDeConfig(void)
{

}

/**************************************************************************************************
  Functions: Data Transfer
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Read data from I2S interface.
 *
 *  \param      pData       Read buffer.
 *  \param      len         Number of bytes to read.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sReadData(uint8_t *pData, uint16_t len)
{

}

/*************************************************************************************************/
/*!
 *  \brief      Write data to I2S interface.
 *
 *  \param      pData       Write buffer.
 *  \param      len         Number of bytes to write.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalI2sWriteData(const uint8_t *pData, uint16_t len)
{

}
