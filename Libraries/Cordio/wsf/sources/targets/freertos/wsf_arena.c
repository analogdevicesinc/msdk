/******************************************************************************
 *
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
/**
* @file wsf_arena.c
* @brief 
*/


/**************************************************************************************************
Includes
**************************************************************************************************/
#include <string.h>
#include <stddef.h>
#include "wsf_arena.h"
#include "wsf_types.h"
#include "wsf_buf.h"
#include "wsf_cs.h"
#include "wsf_assert.h"

/**************************************************************************************************
Macros
**************************************************************************************************/


/**************************************************************************************************
Constants
**************************************************************************************************/


/**************************************************************************************************
Global Variables
**************************************************************************************************/

struct 
{
    uint8_t numArenas;
}wsfArenaCb;


/**************************************************************************************************
Functions
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief  Create an arena.
 *
 *  \param  arena    Arena.
 *  \param  size    Arena total size.
 * 
 */
/*************************************************************************************************/
bool_t WsfArenaCreate(WsfArena_t *arena, uint32_t arena_size)
{
    arena->start = WsfBufAlloc(arena_size);
    arena->idx = 0;
    arena->size = arena_size;

    if(arena->start != NULL)
    {
      wsfArenaCb.numArenas++;
      return TRUE;
    }
    
    #if WSF_BUF_FREE_CHECK_ASSERT == TRUE
      WSF_ASSERT(arena->start != NULL);
    #endif

    return FALSE;

}
/*************************************************************************************************/
/*!
 *  \brief  Allocat a buffer within an arena
 *
 *  \param  arena    Arena to allocate within.
 *  \param  size     Size to allocate in arena
 */
/*************************************************************************************************/
void *WsfArenaAlloc(WsfArena_t *arena, uint32_t size)
{

    WsfCsEnter();

    if (arena->idx >= arena->size)
    {
      WsfCsExit();
    #if WSF_BUF_FREE_CHECK_ASSERT == TRUE
      WSF_ASSERT(arena->idx < arena->size);
    #endif
      return NULL;
    }

    void *mem = arena->start + arena->idx;

    arena->idx += size;

    WsfCsExit();
    
    return mem;
}
/*************************************************************************************************/
/*!
 *  \brief  Free an arena.
 *
 *  \param  pBuf    Arena to free.
 */
/*************************************************************************************************/
void WsfArenaFree(WsfArena_t *arena)
{

  WsfCsEnter(); 
  
  
  if(arena->start != NULL)
  {
    
    WsfBufFree(arena->start);
    memset(arena, 0, sizeof(WsfArena_t));
    wsfArenaCb.numArenas--;

  }
  WsfCsExit(); 

}

uint8_t WsfArenaGetNumActive(void)
{
    return wsfArenaCb.numArenas;
}

/*************************************************************************************************/
