/*************************************************************************************************/
/*!
 *  \file   wsf_os.c
 *
 *  \brief  Software foundation OS main module.
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

#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif
#include <string.h>
#include "wsf_types.h"
#include "wsf_os.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "wsf_timer.h"
#include "wsf_queue.h"
#include "wsf_buf.h"
#include "wsf_msg.h"
#include "wsf_cs.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/**************************************************************************************************
  Compile time assert checks
**************************************************************************************************/

WSF_CT_ASSERT(sizeof(uint8_t) == 1);
WSF_CT_ASSERT(sizeof(uint16_t) == 2);
WSF_CT_ASSERT(sizeof(uint32_t) == 4);

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* maximum number of event handlers per task */
#define WSF_MAX_HANDLERS 16

#define WSF_DISPATCHER_MSG_STACK_SIZE 4096
#define WSF_DISPATCHER_HND_STACK_SIZE 4096
#define WSF_DISPATCHER_MSG_TASK_PRIORITY configMAX_PRIORITIES - 2
#define WSF_DISPATCHER_HND_TASK_PRIORITY configMAX_PRIORITIES - 3

/*! \brief OS serivice function number */
#define WSF_OS_MAX_SERVICE_FUNCTIONS 3

/* Forward declaration */
static void prvWSFMsgTask(void *pvParameters);
static void prvWSFHndTask(void *pvParameters);

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief  Task structure */
typedef struct {
    wsfEventHandler_t handler[WSF_MAX_HANDLERS];
    wsfEventMask_t handlerEventMask[WSF_MAX_HANDLERS];
    wsfQueue_t msgQueue;
    xTaskHandle msgTaskHandle;
    xTaskHandle hndTaskHandle;
    uint8_t numHandler;
} wsfOsTask_t;

/*! \brief  OS structure */
typedef struct {
    wsfOsTask_t task;
    WsfOsIdleCheckFunc_t sleepCheckFuncs[WSF_OS_MAX_SERVICE_FUNCTIONS];
    uint8_t numFunc;
} wsfOs_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief  OS context. */
wsfOs_t wsfOs;

/*************************************************************************************************/
/*!
 *  \brief  Lock task scheduling.
 */
/*************************************************************************************************/
void WsfTaskLock(void)
{
    WsfCsEnter();
}

/*************************************************************************************************/
/*!
 *  \brief  Unock task scheduling.
 */
/*************************************************************************************************/
void WsfTaskUnlock(void)
{
    WsfCsExit();
}

/*************************************************************************************************/
/*!
 *  \brief  Set an event for an event handler.
 *
 *  \param  handlerId   Handler ID.
 *  \param  event       Event or events to set.
 */
/*************************************************************************************************/
void WsfSetEvent(wsfHandlerId_t handlerId, wsfEventMask_t event)
{
    /* coverity[CONSTANT_EXPRESSION_RESULT] */
    WSF_ASSERT(WSF_HANDLER_FROM_ID(handlerId) < WSF_MAX_HANDLERS);

    WSF_TRACE_INFO2("WsfSetEvent handlerId:%u event:%u", handlerId, event);

    /* OR the event mask with the new event flag */
    __atomic_fetch_or(&wsfOs.task.handlerEventMask[WSF_HANDLER_FROM_ID(handlerId)], event,
                      __ATOMIC_RELEASE);

    /* Notify the dispatcher task */
    if (xPortIsInsideInterrupt()) {
        xTaskNotifyFromISR(wsfOs.task.hndTaskHandle, WSF_HANDLER_EVENT, eSetBits, NULL);
    } else {
        xTaskNotify(wsfOs.task.hndTaskHandle, WSF_HANDLER_EVENT, eSetBits);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the task used by the given handler as ready to run.
 *
 *  \param  handlerId   Event handler ID.
 *  \param  event       Task event mask.
 */
/*************************************************************************************************/
void WsfTaskSetReady(wsfHandlerId_t handlerId, wsfTaskEvent_t event)
{
    /* Unused parameter */
    (void)handlerId;

    /* Notify the dispatcher task */
    if (xPortIsInsideInterrupt()) {
        xTaskNotifyFromISR(wsfOs.task.msgTaskHandle, event, eSetBits, NULL);
    } else {
        xTaskNotify(wsfOs.task.msgTaskHandle, event, eSetBits);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Return the message queue used by the given handler.
 *
 *  \param  handlerId   Event handler ID.
 *
 *  \return Task message queue.
 */
/*************************************************************************************************/
wsfQueue_t *WsfTaskMsgQueue(wsfHandlerId_t handlerId)
{
    /* Unused parameter */
    (void)handlerId;

    return &(wsfOs.task.msgQueue);
}

/*************************************************************************************************/
/*!
 *  \brief  Set the next WSF handler function in the WSF OS handler array.  This function
 *          should only be called as part of the stack initialization procedure.
 *
 *  \param  handler    WSF handler function.
 *
 *  \return WSF handler ID for this handler.
 */
/*************************************************************************************************/
wsfHandlerId_t WsfOsSetNextHandler(wsfEventHandler_t handler)
{
    wsfHandlerId_t handlerId = wsfOs.task.numHandler++;

    WSF_ASSERT(handlerId < WSF_MAX_HANDLERS);

    wsfOs.task.handler[handlerId] = handler;

    return handlerId;
}

/*************************************************************************************************/
/*!
*  \brief  Initialize OS control structure.
*
*  \return None.
*/
/*************************************************************************************************/
void WsfOsInit(void)
{
    memset(&wsfOs, 0, sizeof(wsfOs));
    xTaskCreate(
        prvWSFMsgTask, /* The function that implements the task. */
        "CordioM", /* Text name for the task, just to help debugging. */
        WSF_DISPATCHER_MSG_STACK_SIZE, /* The size (in words) of the stack that should be created for the task. */
        NULL, /* A parameter that can be passed into the task.  Not used. */
        WSF_DISPATCHER_MSG_TASK_PRIORITY, /* The priority to assign to the task.  tskIDLE_PRIORITY (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1 is the highest priority. */
        &wsfOs.task
             .msgTaskHandle); /* Used to obtain a handle to the created task.  Not used, so set to NULL. */
    WSF_ASSERT(wsfOs.task.msgTaskHandle);
    xTaskCreate(
        prvWSFHndTask, /* The function that implements the task. */
        "CordioH", /* Text name for the task, just to help debugging. */
        WSF_DISPATCHER_HND_STACK_SIZE, /* The size (in words) of the stack that should be created for the task. */
        NULL, /* A parameter that can be passed into the task.  Not used. */
        WSF_DISPATCHER_HND_TASK_PRIORITY, /* The priority to assign to the task.  tskIDLE_PRIORITY (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1 is the highest priority. */
        &wsfOs.task
             .hndTaskHandle); /* Used to obtain a handle to the created task.  Not used, so set to NULL. */
    WSF_ASSERT(wsfOs.task.hndTaskHandle);
}

/*************************************************************************************************/
/*!
 *  \brief  The message and timer handler task loop
 *
 *  \param  pvParameters   The task parameters (optional)
 * 	\note   Must never return
 *
 *  \return None.
 */
/*************************************************************************************************/
static void prvWSFMsgTask(void *pvParameters)
{
    wsfOsTask_t *pTask;
    void *pMsg;
    wsfTimer_t *pTimer;
    wsfHandlerId_t handlerId;

    pTask = &wsfOs.task;

    while (1) {
        uint32_t taskEventMask = 0;

        if (!xTaskNotifyWait(0, 0xFFFFFFFF, &taskEventMask, portMAX_DELAY))
            continue; /* No notifications, restart waiting */

        if (taskEventMask & WSF_MSG_QUEUE_EVENT) {
            /* handle msg queue */
            while ((pMsg = WsfMsgDeq(&pTask->msgQueue, &handlerId)) != NULL) {
                WSF_ASSERT(handlerId < WSF_MAX_HANDLERS);
                (*pTask->handler[handlerId])(0, pMsg);
                WsfMsgFree(pMsg);
            }
        }

        if (taskEventMask & WSF_TIMER_EVENT) {
            /* service timers */
            while ((pTimer = WsfTimerServiceExpired(0)) != NULL) {
                WSF_ASSERT(pTimer->handlerId < WSF_MAX_HANDLERS);
                (*pTask->handler[pTimer->handlerId])(0, &pTimer->msg);
            }
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  The dispatcher task loop
 *
 *  \param  pvParameters   The task parameters (optional)
 * 	\note   Must never return
 *
 *  \return None.
 */
/*************************************************************************************************/
static void prvWSFHndTask(void *pvParameters)
{
    wsfOsTask_t *pTask;
    uint8_t i;

    pTask = &wsfOs.task;

    while (1) {
        uint32_t taskEventMask = 0;

        /* Wait for a FreeRTOS task notification */
        if (!xTaskNotifyWait(0, 0xFFFFFFFF, &taskEventMask, portMAX_DELAY))
            continue; /* No notifications, restart waiting */

        if (taskEventMask & WSF_HANDLER_EVENT) {
            /* service handlers */
            for (i = 0; i < WSF_MAX_HANDLERS; i++) {
                if ((pTask->handlerEventMask[i] != 0) && (pTask->handler[i] != NULL)) {
                    wsfEventMask_t eventMask;
                    wsfEventMask_t zero = 0;
                    /* Read the event mask and clear it atomically */
                    __atomic_exchange(&pTask->handlerEventMask[i], &zero, &eventMask,
                                      __ATOMIC_ACQ_REL);
                    (*pTask->handler[i])(eventMask, NULL);
                }
            }
        }
    }
}
