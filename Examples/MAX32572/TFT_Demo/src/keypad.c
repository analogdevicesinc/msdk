/*
 * @file keypad.c
 *
 ******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 ******************************************************************************
 */

/* Global includes */
#include <stdio.h>
/* Local includes */
#include "keypad.h"
#include "skbd.h"
#include "max32572.h"

/*********************************      Variables    **************************/
static volatile int is_pressed = 0;

/* keys mapping on the keyboard */
static unsigned char keyboard_map[16] = {KEY_F, KEY_E, KEY_D, KEY_C, KEY_3, KEY_6, KEY_9, KEY_B,
                                         KEY_2, KEY_5, KEY_8, KEY_0, KEY_1, KEY_4, KEY_7, KEY_A};

/********************************* Static Functions **************************/
static void keypadHandler(void)
{
    unsigned int status;

    MXC_SKBD_InterruptStatus(&status);

    if (MXC_F_SKBD_ISR_OVERIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_OVERIS);
    }

    if (MXC_F_SKBD_ISR_PUSHIS & status) {
        is_pressed = 1;
        /* Clear interruption */
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_PUSHIS);
    }

    if (MXC_F_SKBD_ISR_RELEASEIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_RELEASEIS);
    }

    return;
}

/********************************* Public Functions *************************/
int keypad_init(void)
{
    int rv = 0;
    mxc_skbd_config_t skb_cfg;

    skb_cfg.inputs      = MXC_SKBD_KBDIO4 | MXC_SKBD_KBDIO5 | MXC_SKBD_KBDIO6 | MXC_SKBD_KBDIO7;
    skb_cfg.outputs     = MXC_SKBD_KBDIO0 | MXC_SKBD_KBDIO1 | MXC_SKBD_KBDIO2 | MXC_SKBD_KBDIO3;
    skb_cfg.debounce    = MXC_V_SKBD_CR1_DBTM_TIME10MS;
    skb_cfg.ioselect    = 0;
    skb_cfg.irq_handler = (irq_handler_t)keypadHandler;
    skb_cfg.reg_erase   = 1;

    MXC_SKBD_PreInit();

    rv = MXC_SKBD_Init(skb_cfg);

    if (rv) {
        return E_UNINITIALIZED;
    }

    rv = MXC_SKBD_EnableInterruptEvents(MXC_SKBD_INTERRUPT_STATUS_PUSHIS);

    if (rv) {
        return E_UNINITIALIZED;
    }

    return 0;
}

int keypad_getkey(void)
{
    volatile unsigned int in;
    volatile unsigned int out;
    volatile unsigned int i;
    unsigned short* key;
    mxc_skbd_keys_t keys = {0, 0, 0, 0};
    int pressed_key      = 0;

    if (is_pressed == 1) {
        MXC_SKBD_ReadKeys(&keys);
        key = &keys.key0;

        for (i = 0; i < 4; i++) {
            in  = 0x0f & *key;
            out = (0xf0 & *key) >> 4;

            if (*key) {
                pressed_key = keyboard_map[(in - 4) * 4 + out];
            }

            *key = 0;
            key++;
        }

        is_pressed = 0;
    }

    return pressed_key;
}
