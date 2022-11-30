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
#endif /* __GNUC__ */

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_cs.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_math.h"
#include "wsf_os.h"
#include "pal_sys.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/
#if defined ( __GNUC__ )
extern caddr_t _sbrk(int incr);
#elif defined ( __ICCARM__ )
extern void *_sbrk(int incr);
#endif

#if defined ( __GNUC__ )
static void* freeStartAddr = 0;
extern unsigned int __HeapBase;
extern unsigned int __HeapLimit;
#elif defined ( __ICCARM__ )
static void* freeStartAddr = 0;
extern unsigned char *HeapBase;
extern unsigned char *HeapLimit;
#endif

/*************************************************************************************************/
/*!
 *  \brief      Reserve heap memory.
 *
 *  \param      size    Number of bytes of heap memory used.
 */
/*************************************************************************************************/
void WsfHeapAlloc(uint32_t size)
{
  /* Round up to nearest multiple of 4 for word alignment */
  size = (size + 3) & ~3;

  freeStartAddr = _sbrk(size);
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
  freeStartAddr = _sbrk(0);
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
  freeStartAddr = _sbrk(0);
#if defined ( __GNUC__ )
  return ((uint32_t)&__HeapLimit - (uint32_t)freeStartAddr);
#elif defined ( __ICCARM__ )
  return ((uint32_t)HeapLimit - (uint32_t)freeStartAddr);  
#endif  
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
  freeStartAddr = _sbrk(0);
#if defined ( __GNUC__ ) 
  return((uint32_t)freeStartAddr - (uint32_t)&__HeapBase);
#elif defined ( __ICCARM__ ) 
  return((uint32_t)freeStartAddr - (uint32_t)HeapBase);   
#endif  
  
}
