/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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

#ifndef LIBRARIES_MAXUSB_INCLUDE_DBG_LOG_DBG_LOG_H_
#define LIBRARIES_MAXUSB_INCLUDE_DBG_LOG_DBG_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DBG_EVT_START = 0,
  DBG_EVT_RESET,
  DBG_EVT_SETUP,
  DBG_EVT_READ,
  DBG_EVT_WRITE,
  DBG_EVT_USB_INT_START,
  DBG_EVT_USB_INT_END,
  DBG_EVT_IN,
  DBG_EVT_IN_SPUR,
  DBG_EVT_OUT,
  DBG_EVT_OUT_SPUR,
  DBG_EVT_OUT_DMA,
  DBG_EVT_OUT_DMA_END,
  DBG_EVT_DMA_INT_START,
  DBG_EVT_DMA_INT_SPUR,
  DBG_EVT_DMA_INT_IN,
  DBG_EVT_DMA_INT_OUT,
  DBG_EVT_DMA_INT_END,
  DBG_EVT_REQ_LODGE,
  DBG_EVT_REQ_LODGE_DMA,
  DBG_EVT_REQ_REMOVE,
  DBG_EVT_REQ_REMOVE_DMA,
  DBG_EVT_CLR_OUTPKTRDY,
  DBG_EVT_OUTCOUNT,
  DBG_EVT_REQLEN,
  DBG_EVT_ACTLEN,
  DBG_EVT_SETUP_IDLE,
  DBG_EVT_SETUP_NODATA,
  DBG_EVT_SETUP_DATA_OUT,
  DBG_EVT_SETUP_DATA_IN,
  DBG_EVT_SETUP_END,
  DBG_EVT_SENT_STALL,
  DBG_EVT_ACKSTAT,
  DBG_EVT_TRIGGER,
  DBG_EVT_MAX_EVT
} dbg_evt_type_t;

int dbg_log_init(void);
int dbg_log_add(uint32_t t, dbg_evt_type_t e, uint32_t e_p, char *txt);
void dbg_log_print(int num);

#ifdef __cplusplus
}
#endif

#endif //LIBRARIES_MAXUSB_INCLUDE_DBG_LOG_DBG_LOG_H_
