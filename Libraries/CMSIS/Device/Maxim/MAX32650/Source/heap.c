
/* **** Includes **** */
#include <stdint.h>
#include <errno.h>
#include <unistd.h>

/**
 * @brief  sbrk
 * @detail Increase program data space
 * @detail Malloc and related functions depend on this
 */

/* **** declarations **** */
static char *heap_end = 0;
extern unsigned int __HeapBase;
extern unsigned int __HeapLimit;

/* **** functions **** */
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
