/*************************************************************************************************/
/*!
 *  \file   wsf_heap.c
 *
 *  \brief  Heap service.
 *
 *  Copyright (c) 2009-2018 Arm Ltd. All Rights Reserved.
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

#if defined ( __GNUC__ )
#include <unistd.h>
#include <malloc.h>
#endif /* __GNUC__ */

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_cs.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_math.h"
#include "wsf_os.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

static void* freeStartAddr = NULL;
static uint32_t heapUsed = 0;

/*************************************************************************************************/
/*!
 *  \brief      Allocate heap memory.
 *
 *  \param      size    Number of bytes of heap memory used.
 */
/*************************************************************************************************/
void WsfHeapAlloc(uint32_t size)
{
    /* Round up to nearest multiple of 4 for word alignment */
    size = (size + 3) & ~3;

    freeStartAddr = sbrk(size);
    heapUsed += size;
}

/*************************************************************************************************/
/*!
 *  \brief      Get next available heap memory.
 *
 *  \return     Address of the start of heap memory.
 */
/*************************************************************************************************/
void *WsfHeapGetFreeStartAddress(void)
{
    if(freeStartAddr == (caddr_t)-1) {
        WSF_ASSERT(0);
        return NULL;
    }

    return freeStartAddr;
}

/*************************************************************************************************/
/*!
 *  \brief      Get heap available.
 *
 *  \return     Number of bytes of heap memory available.
 */
/*************************************************************************************************/
uint32_t WsfHeapCountAvailable(void)
{
    struct mallinfo temp_mallinfo = mallinfo();

    return temp_mallinfo.fordblks;
}

/*************************************************************************************************/
/*!
 *  \brief      Get heap used.
 *
 *  \return     Number of bytes of heap memory used.
 */
/*************************************************************************************************/
uint32_t WsfHeapCountUsed(void)
{
    return heapUsed;
}
