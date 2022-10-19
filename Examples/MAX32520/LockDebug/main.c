/******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
******************************************************************************/

/**
 * @file    main.c
 * @brief   Lock Debug
 * @details This example demonstrates locking/unlocking the debug port under program control
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "flc.h"
#include "mxc_delay.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

typedef struct {
    unsigned int locks; /* # of locks left */
    unsigned int unlocks; /* # of unlocks left */
    unsigned int locked; /* 1 if part is locked, 0 if unlocked */
} debug_status_t;

/***** Globals *****/
volatile unsigned int advance;

/***** Functions *****/
unsigned int debug_status(debug_status_t *ptr);

/*
 * Locks the debug port, and returns E_SUCCESS if successful.
 * The parameter permanent, if non-zero, will cause the lock to be permanent,
 * even if unlock/lock cycles have not been exhausted.
 *
 */
int debug_lock(unsigned int permanent)
{
    volatile uint32_t *lock0 = (uint32_t *)0x10800030;
    volatile uint32_t *lock1 = (uint32_t *)0x10800034;
    uint32_t word = 0x0;
    debug_status_t st;
    int result = E_UNKNOWN;

    /* Check for any existing lock */
    if (debug_status(&st)) {
        /* Already locked */
        result = E_SUCCESS;
    } else {
        /* Not locked, but could we? */
        if (st.locks == 0) {
            /* Nope */
            result = E_BAD_STATE;
        } else {
            MXC_FLC_UnlockInfoBlock(0x10800000);

            /* Otherwise, can lock it with one of the words */
            if ((*lock0 & 0x0000ffff) == 0x0000ffff) {
                word = 0xffffa5a5;
                result = MXC_FLC_Write((uint32_t)lock0, 4, &word);

                if (permanent && (result == E_NO_ERROR)) {
                    word = 0x7fffffff;
                    /* Write the 64th bit to a zero */
                    result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                }
            } else {
                if ((*lock0 & 0xffff0000) == 0xffff0000) {
                    word = 0x5a5affff;
                    result = MXC_FLC_Write((uint32_t)lock0, 4, &word);

                    if (permanent && (result == E_NO_ERROR)) {
                        /* Write the 64th bit to a zero */
                        word = 0x7fffffff;
                        result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                    }
                } else {
                    if ((*lock1 & 0x0000ffff) == 0x0000ffff) {
                        if (permanent) {
                            /* Write the 64th bit to a zero */
                            word = 0x7fffa5a5;
                            result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                        } else {
                            word = 0xffffa5a5;
                            result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                        }
                    } else {
                        if ((*lock1 & 0xffff0000) == 0xffff0000) {
                            if (permanent) {
                                /* Write the 64th bit to a zero */
                                word = 0x5a5affff;
                                result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                            } else {
                                word = 0xda5affff;
                                result = MXC_FLC_Write((uint32_t)lock1, 4, &word);
                            }
                        } else {
                            /* Should not get here */
                            result = E_UNKNOWN;
                        }
                    }
                }
            }

            MXC_FLC_LockInfoBlock(0x10800000);
        }
    }

    return result;
}

/*
 * Unlocks the debug port, and returns E_SUCCESS if successful.
 * The parameter permanent, if non-zero, will cause the unlock to be permanent,
 * even if unlock/lock cycles have not been exhausted.
 *
 */
int debug_unlock(unsigned int permanent)
{
    volatile uint32_t *lock0 = (uint32_t *)0x10800030;
    volatile uint32_t *lock1 = (uint32_t *)0x10800034;
    uint32_t tmp;
    debug_status_t st;
    int result;

    /* Check for any existing lock */
    if (!debug_status(&st)) {
        /* Already unlocked */
        result = E_SUCCESS;
    } else {
        /* Are we able to unlock? */
        if (st.unlocks == 0) {
            /* Nope */
            result = E_BAD_STATE;
        } else {
            MXC_FLC_UnlockInfoBlock(0x10800000);
            /* Otherwise, can unlock -- be thorough about finding all lock words */
            result = E_UNKNOWN;

            if ((*lock0 & 0x0000ffff) != 0x0000ffff) {
                tmp = (*lock0) & 0xffff0000;
                result = MXC_FLC_Write((uint32_t)lock0, 4, &tmp);
            }

            if ((*lock0 & 0xffff0000) != 0xffff0000) {
                tmp = (*lock0) & 0x0000ffff;
                result = MXC_FLC_Write((uint32_t)lock0, 4, &tmp);
            }

            if ((*lock1 & 0x0000ffff) != 0x0000ffff) {
                tmp = (*lock1) & 0xffff0000;
                result = MXC_FLC_Write((uint32_t)lock1, 4, &tmp);
            }

            if ((*lock1 & 0xffff0000) != 0xffff0000) {
                tmp = (*lock1) & 0x8000ffff;
                result = MXC_FLC_Write((uint32_t)lock1, 4, &tmp);
            }

            if (permanent) {
                tmp = (*lock1) & 0x7fffffff;
                result = MXC_FLC_Write((uint32_t)lock1, 4, &tmp);
            }

            MXC_FLC_LockInfoBlock(0x10800000);
        }
    }

    return result;
}

/* Returns the current state (locked/unlocked) */
unsigned int debug_status(debug_status_t *ptr)
{
    volatile uint32_t *lock0 = (uint32_t *)0x10800030;
    volatile uint32_t *lock1 = (uint32_t *)0x10800034;
    unsigned int locks, unlocks, locked;

    locked = locks = unlocks = 0;
    MXC_FLC_UnlockInfoBlock(0x10800000);

    printf("[debug] lock1: 0x%08x lock0: 0x%08x\n", *lock1, *lock0);

    /* Check lower half-words */
    switch ((*lock0) & 0xffff) {
    case 0xffff:
        locks++;
        break;

    case 0xa5a5:
        unlocks++;
        break;

    default:
        /* Either used or something else */
        break;
    }

    switch (((*lock0) >> 16) & 0xffff) {
    case 0xffff:
        locks++;
        break;

    case 0x5a5a:
        unlocks++;
        break;

    default:
        /* Either used or something else */
        break;
    }

    /* Check upper half-words */
    switch ((*lock1) & 0xffff) {
    case 0xffff:
        locks++;
        break;

    case 0xa5a5:
        unlocks++;
        break;

    default:
        /* Either used or something else */
        break;
    }

    switch (((*lock1) >> 16) & 0x7fff) {
    case 0x7fff:
        locks++;
        break;

    case 0x5a5a:
        unlocks++;
        break;

    default:
        /* Either used or something else */
        break;
    }

    if (unlocks) {
        /* Part is locked */
        locked = 1;
    }

    /* Locks available also count toward eventual unlocks */
    unlocks += locks;

    /* Check for write protect */
    if (((*lock1) & 0x80000000) == 0) {
        /* Can't do anything more */
        locks = unlocks = 0;
    }

    MXC_FLC_LockInfoBlock(0x10800000);

    if (ptr) {
        ptr->locks = locks;
        ptr->unlocks = unlocks;
        ptr->locked = locked;
    }

    return locked;
}

void pushbutton(void *unused)
{
    /* Trigger main loop to lock/unlock */
    if (!advance) {
        advance++;
    }
}

// *****************************************************************************
int main(void)
{
    debug_status_t x;
    int y;

    PB_RegisterCallback(0, pushbutton);
    __enable_irq();

    printf("\n\n***** " TOSTRING(TARGET) " Debug Lock-out Example *****\n");

    /* Visually display locked (RED) or unlocked (GREEN) */
    y = debug_status(&x);

    if (y) {
        LED_On(0);
        LED_Off(1);
    } else {
        LED_Off(0);
        LED_On(1);
    }

    printf("debug_status = %d\n", y);
    printf("Locks left = %u, Unlocks left = %u, Debug port locked = %u\n", x.locks, x.unlocks,
           x.locked);

    printf("\nPress button (SW2) to lock/unlock part.\n");

    while (1) {
        if (advance) {
            printf("Button pressed.\n");
            printf(" -- BEFORE -- \n");
            y = debug_status(&x);
            printf("Locks left = %u, Unlocks left = %u, Debug port locked = %u\n", x.locks,
                   x.unlocks, x.locked);

            if (y) {
                printf("Debug port LOCKED. Unlocking port.\n");

                if (!x.unlocks) {
                    printf("NOTE: Won't succeed, either because no more unlocks left or permanent "
                           "bit set.\n");
                }

                /* Change the parameter to a non-zero value to make this permanent. */
                debug_unlock(0);
            } else {
                printf("Debug port unlocked. Locking port.\n");

                if (!x.locks) {
                    printf("NOTE: Won't succeed, either because no more locks left or permanent "
                           "bit set.\n");
                }

                /* Change the parameter to a non-zero value to make this permanent. */
                debug_lock(0);
            }

            printf(" -- AFTER -- \n");
            y = debug_status(&x);
            printf("Locks left = %u, Unlocks left = %u, Debug port locked = %u\n", x.locks,
                   x.unlocks, x.locked);

            /* Visually display locked (RED) or unlocked (GREEN) */
            if (y) {
                LED_On(0);
                LED_Off(1);
            } else {
                LED_Off(0);
                LED_On(1);
            }

            printf("This change will take effect after RSTN (SW1) is pressed, or a power cycle of "
                   "the EV Kit.\n");
            printf("\nPress button (SW2) to lock/unlock part.\n");
            /* Debounce */
            MXC_Delay(MXC_DELAY_MSEC(500));
            advance = 0;
        }
    }

    return 0;
}
