/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <malloc.h>

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
static char *heap_end = 0;
extern unsigned int __HeapBase;
extern unsigned int __HeapLimit;
caddr_t _sbrk(int incr)
{
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = (caddr_t)&__HeapBase;
    }
    prev_heap_end = heap_end;

    if ((unsigned int)(heap_end + incr) > (unsigned int)&__HeapLimit) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;

    return (caddr_t)prev_heap_end;
}

// struct mallinfo {
//   size_t arena;    /* total space allocated from system */
//   size_t ordblks;  /* number of non-inuse chunks */
//   size_t smblks;   /* unused -- always zero */
//   size_t hblks;    /* number of mmapped regions */
//   size_t hblkhd;   /* total space in mmapped regions */
//   size_t usmblks;  /* unused -- always zero */
//   size_t fsmblks;  /* unused -- always zero */
//   size_t uordblks; /* total allocated space */
//   size_t fordblks; /* total non-inuse space */
//   size_t keepcost; /* top-most, releasable (via malloc_trim) space */
// };

/*
The structure fields contain the following information:

       arena  The total amount of memory allocated by means other than
              mmap(2) (i.e., memory allocated on the heap).  This figure
              includes both in-use blocks and blocks on the free list.

       ordblks
              The number of ordinary (i.e., non-fastbin) free blocks.

       smblks The number of fastbin free blocks (see mallopt(3)).

       hblks  The number of blocks currently allocated using mmap(2).
              (See the discussion of M_MMAP_THRESHOLD in mallopt(3).)

       hblkhd The number of bytes in blocks currently allocated using
              mmap(2).

       usmblks
              This field is unused, and is always 0.  Historically, it
              was the "highwater mark" for allocated space—that is, the
              maximum amount of space that was ever allocated (in
              bytes); this field was maintained only in nonthreading
              environments.

       fsmblks
              The total number of bytes in fastbin free blocks.

       uordblks
              The total number of bytes used by in-use allocations.

       fordblks
              The total number of bytes in free blocks.

       keepcost
              The total amount of releasable free space at the top of
              the heap.  This is the maximum number of bytes that could
              ideally (i.e., ignoring page alignment restrictions, and
              so on) be released by malloc_trim(3).
*/

struct mallinfo mallinfo(void)
{
    struct mallinfo temp_mallinfo;

    if (heap_end == 0) {
        heap_end = (caddr_t)&__HeapBase;
    }

    temp_mallinfo.arena = ((size_t)&__HeapLimit - (size_t)&__HeapBase);
    temp_mallinfo.ordblks = 0; /* Unused */
    temp_mallinfo.smblks = 0; /* Unused */
    temp_mallinfo.hblks = 0; /* Unused */
    temp_mallinfo.hblkhd = 0; /* Unused */
    temp_mallinfo.usmblks = 0; /* Unused */
    temp_mallinfo.fsmblks = 0; /* Unused */
    temp_mallinfo.uordblks = (size_t)heap_end - (size_t)&__HeapBase;
    temp_mallinfo.fordblks = (size_t)&__HeapLimit - (size_t)heap_end;
    temp_mallinfo.keepcost = 0 /* Unused */;

    return temp_mallinfo;
}
