/*************************************************************************************************/
/*!
 *  \file   wsf_timer.c
 *
 *  \brief  Timer service.
 *
 *  Copyright (c) 2009-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_timer.h"

#include "wsf_assert.h"
#include "wsf_cs.h"
#include "wsf_os.h"
#include "wsf_trace.h"
#include "wsf_buf.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"

#include <string.h>

#if ( configUSE_TIMERS != 1 )
#error Enable timers in FreeRTOSConfig.h by definiing configUSE_TIMERS as 1
#endif

typedef struct TimerStruct {
  struct TimerStruct *next;
  struct TimerStruct *prev;
  wsfTimer_t *wsfTimerStruct;
  TimerHandle_t tmr;
} TimerStruct_t;

typedef struct TimerList {
  TimerStruct_t *head;
  TimerStruct_t *tail;
} TimerList_t;

static TimerList_t s_timers = { NULL, NULL };
static QueueHandle_t s_queue = NULL;

void prvTimerCallback(TimerHandle_t xTimer)
{
  WsfCsEnter();
  TimerStruct_t *firedTimer = NULL;
  for (TimerStruct_t *ts = s_timers.head; ts != NULL; ts = ts->next) {
    if (ts->tmr == xTimer) {
      firedTimer = ts;
      break;
    }
  }
  if (firedTimer) {
    /* Get the timer handler */
    wsfHandlerId_t handler = firedTimer->wsfTimerStruct->handlerId;
    if (!xQueueSend(s_queue, &firedTimer->wsfTimerStruct, portMAX_DELAY)) {
      WSF_ASSERT(0);
    }
    WsfTaskSetReady(handler, WSF_TIMER_EVENT);
    firedTimer->wsfTimerStruct->isStarted = FALSE;
  }
  WsfCsExit();
}

void WsfTimerInit(void)
{
  s_queue = xQueueCreate(configTIMER_QUEUE_LENGTH, sizeof(wsfTimer_t*));
}

void WsfTimerStartMs(wsfTimer_t *pTimer, wsfTimerTicks_t ms)
{
  WsfCsEnter();
  /* Look if there is existing timer */
  TimerStruct_t *existingTimer = NULL;
  for (TimerStruct_t *ts = s_timers.head; ts != NULL; ts = ts->next) {
    if (ts->wsfTimerStruct == pTimer) {
      existingTimer = ts;
      break;
    }
  }

  uint32_t ticks = pdMS_TO_TICKS(ms);
  if (existingTimer) {
    if (ticks != pTimer->ticks) {
      /* Update timer interval */
      xTimerChangePeriod(existingTimer->tmr, ticks, 0);
    }
    /* Restart the existing timer */
    pTimer->isStarted = xTimerReset(existingTimer->tmr, 0);
  } else {
    TimerHandle_t tmr = xTimerCreate(NULL, ticks,
                                     pdFALSE,
                                     NULL, prvTimerCallback);
    pTimer->isStarted = xTimerStart(tmr, 0);

    TimerStruct_t *ts = (TimerStruct_t*) WsfBufAlloc(
                          sizeof(TimerStruct_t));
    memset(ts, 0, sizeof(TimerStruct_t));
    ts->wsfTimerStruct = pTimer;
    ts->tmr = tmr;
    ts->wsfTimerStruct->ticks = ticks;
    if (s_timers.tail) {
      /* Append to the list end */
      TimerStruct_t *prev = s_timers.tail;
      prev->next = ts;
      ts->prev = prev;
      s_timers.tail = ts;
    } else {
      /* Create first element */
      s_timers.head = ts;
      s_timers.tail = ts;
    }
  }
  WsfCsExit();
}

void WsfTimerStartSec(wsfTimer_t *pTimer, wsfTimerTicks_t sec)
{
  WsfTimerStartMs(pTimer, sec * 1000);
}

void WsfTimerStop(wsfTimer_t *pTimer)
{
  WsfCsEnter();
  TimerStruct_t *itemToRemove = NULL;
  for (TimerStruct_t *ts = s_timers.head; ts != NULL; ts = ts->next) {
    if (ts->wsfTimerStruct == pTimer) {
      itemToRemove = ts;
      break;
    }
  }
  if (itemToRemove) {
    /* Delete the timer and its list item */
    xTimerStop(itemToRemove->tmr, portMAX_DELAY);
    xTimerDelete(itemToRemove->tmr, portMAX_DELAY);
    TimerStruct_t *prev = itemToRemove->prev;
    TimerStruct_t *next = itemToRemove->next;
    if (prev) {
      prev->next = next;
    } else {
      s_timers.head = NULL;
      s_timers.tail = NULL;
    }
    if (next) {
      next->prev = prev;
    }

    /* Update tail if removing tail */
    if(s_timers.tail == itemToRemove) {
      s_timers.tail = prev;
    }

    itemToRemove->wsfTimerStruct->isStarted = FALSE;
    WsfBufFree(itemToRemove);
  } else {
    /* Timer not found; */
  }
  WsfCsExit();
}

wsfTimer_t* WsfTimerServiceExpired(wsfTaskId_t taskId)
{
  wsfTimer_t *tmr;
  if (xQueueReceive(s_queue, &tmr, 0)) {
    return tmr;
  }
  return NULL;
}
