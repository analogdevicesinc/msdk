/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
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

#if defined __ICCARM__
#pragma diag_suppress = Pe513, Pe042
#endif

#include <stdlib.h>
#pragma section = "CSTACK"
#pragma section = "HEAP"

//#include <sys/types.h>
#include <errno.h>
#include <stddef.h>

/* ------------------------------------------------------------------------- */
/*!Increase program data space
   This is a minimal implementation.  Assuming the heap is growing upwards
   from __HeapBase towards __HeapLimit.
   See linker file for definition.
   @param[in] incr  The number of bytes to increment the stack by.
   @return  A pointer to the start of the new block of memory                */
/* ------------------------------------------------------------------------- */

unsigned char *HeapBase = __section_begin("HEAP");
unsigned char *HeapLimit = __section_end("HEAP");

void *_sbrk(int incr)
{
    static char *heap_end = 0; /* Previous end of heap or 0 if none */
    char *prev_heap_end;

    if (0 == heap_end) {
        heap_end = HeapBase; /* Initialize first time round */
    }

    prev_heap_end = heap_end;
    heap_end += incr;

    //check
    if (heap_end >= HeapLimit) {
        errno = 132; //ENOMEM;
        return (char *)-1;
    }

    return (void *)prev_heap_end;
} /* _sbrk () */
