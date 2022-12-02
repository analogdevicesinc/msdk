/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile client - Device Configuration.
 *
 *  Copyright (c) 2017-2018 Arm Ltd. All Rights Reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include <stddef.h>
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdx_defs.h"
#include "wdxc/wdxc_api.h"
#include "wdxc/wdxc_main.h"
#include "dm_api.h"
#include "app_api.h"

/**************************************************************************************************
  External Variables
**************************************************************************************************/
extern wdxcCb_t wdxcCb;

/*************************************************************************************************/
/*!
 *  \brief  Send a request to Disconnect And Reset.
 *
 *  \param  connId          Connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxcDcSendDisconnectAndReset(dmConnId_t connId)
{
    uint8_t buf[WDX_DC_HDR_LEN];
    uint8_t *p = buf;
    uint16_t handle;

    WSF_ASSERT(wdxcCb.conn[connId - 1].pHdlList != NULL)

    handle = wdxcCb.conn[connId - 1].pHdlList[WDXC_DC_HDL_IDX];

    UINT8_TO_BSTREAM(p, WDX_DC_OP_SET);
    UINT8_TO_BSTREAM(p, WDX_DC_ID_DISCONNECT_AND_RESET);

    AttcWriteReq(connId, handle, WDX_DC_HDR_LEN, buf);
}
