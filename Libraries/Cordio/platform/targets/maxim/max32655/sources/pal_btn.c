/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      Button driver implementation.
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

#include "pal_btn.h"
#include "pb.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      Device control block. */
struct {
    PalBtnActionCback_t actionCback; /*!< Action call back functions. */
    PalBtnState_t state; /*!< State of the buttons. */
} palBtnCb;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Button callback handler.
 *
 *  \param      pb        gpio_cfg_t Pointer to the button that was pressed.
 *
 *  \return     None.
 *
 *  Figure out which button interrupted and if it's pressed or depressed.
 */
/*************************************************************************************************/
void palBtnCallback(void *pb)
{
    unsigned i;
    int button = -1;
    PalBtnState_t btnState = PAL_BTN_POS_UP;

    /* Figure out which button caused this interrupt */
    for (i = 0; i < num_pbs; i++) {
        /* This is the button */
        if ((void *)&pb_pin[i] == pb) {
            button = (int)i;
            if (PB_Get(i)) {
                btnState = PAL_BTN_POS_DOWN;
            } else {
                btnState = PAL_BTN_POS_UP;
            }
            break;
        }
    }

    /* We didn't find the button */
    if (button == -1) {
        return;
    }

    /* App UI btn index starts at 1 */
    button++;

    /* Call the registered callback */
    if (palBtnCb.actionCback != NULL) {
        palBtnCb.actionCback((uint8_t)button, btnState);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize buttons.
 *
 *  \param      actCback    Button event callback (called in ISR context).
 *
 *  \return     None.
 *
 *  Initialize buttons corresponding to set ordinal bit position in btnMask.
 */
/*************************************************************************************************/
void PalBtnInit(PalBtnActionCback_t actCback)
{
    unsigned i;

    PB_Init();

    /* Initialize callback */
    palBtnCb.actionCback = actCback;

    /* Register the callback for all of the buttons */
    for (i = 0; i < num_pbs; i++) {
        PB_RegisterCallbackRiseFall(i, palBtnCallback);
        PB_IntEnable(i);
    }

    palBtnCb.state = PAL_BTN_STATE_READY;
}

/*************************************************************************************************/
/*!
 *  \brief      De-Initialize buttons.
 *
 *  \return     None.
 *
 *  De-Initialize all buttons.
 */
/*************************************************************************************************/
void PalBtnDeInit(void)
{
    unsigned i;

    /* Initialize callback */
    palBtnCb.actionCback = NULL;

    /* Register the callback for all of the buttons */
    for (i = 0; i < num_pbs; i++) {
        PB_IntDisable(i);
    }

    palBtnCb.state = PAL_BTN_STATE_UNINIT;
}

/*************************************************************************************************/
/*!
 *  \brief      Get state of the buttons.
 *
 *  \return     State of the buttons
 *
 *  Get the current button state.
 */
/*************************************************************************************************/
PalBtnState_t PalBtnGetState(void)
{
    return palBtnCb.state;
}

/*************************************************************************************************/
/*!
 *  \brief      Get button position.
 *
 *  \param      btnId           Button ID.
 *
 *  \return     Button position.
 *
 *  Get the current button position.
 */
/*************************************************************************************************/
PalBtnPos_t PalBtnGetPosition(uint8_t btnId)
{
    return PB_Get(btnId);
}
