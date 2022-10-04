/*
 * @file keypad.c
 *
 ******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#include <string.h>
#include <stdio.h>

#include <MAX32xxx.h>

#include "keypad.h"

// KB stand for Keyboard
#define INPUT_DEVICE_PC_KB 1 // use PC    keyboard as input device
#define INPUT_DEVICE_EvKit_KB 2 // use EvKit keyboard as input device

//
#define INPUT_DEVICE INPUT_DEVICE_PC_KB

#if INPUT_DEVICE == INPUT_DEVICE_EvKit_KB
#define KEY_ENTER 'B'
#define KEY_ENTER2 'C'
#define KEY_CANCEL 'F'
#elif INPUT_DEVICE == INPUT_DEVICE_PC_KB
#define KEY_ENTER '\n'
#define KEY_ENTER2 '\r'
#define KEY_CANCEL 27 //ESC
#endif

#if INPUT_DEVICE == INPUT_DEVICE_EvKit_KB

static mxc_skbd_keys_t g_keys = { 0, 0, 0, 0 };
static volatile int is_pressed = 0;

/* keys mapping on the keyboard */
static unsigned char keyboard_map[16] = { 'F', 'E', 'D', 'C', '3', '6', '9', 'B',
                                          '2', '5', '8', '0', '1', '4', '7', 'A' };

static void keypadHandler(void)
{
    unsigned int status;

    MXC_SKBD_InterruptStatus(&status);

    if (MXC_F_SKBD_ISR_OVERIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_OVERIS);
    }

    if (MXC_F_SKBD_ISR_PUSHIS & status) {
        MXC_SKBD_ReadKeys(&g_keys);
        is_pressed = 1;
        /* Clear interruption */
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_PUSHIS);
    }

    if (MXC_F_SKBD_ISR_RELEASEIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_RELEASEIS);
    }

    return;
}
#endif

#if INPUT_DEVICE == INPUT_DEVICE_EvKit_KB
int kb_get_key(void)
{
    volatile unsigned int in;
    volatile unsigned int out;
    volatile unsigned int i;
    uint16_t *key;

    int pressed_key = 0;

    if (is_pressed == 1) {
        key = &g_keys.key0;

        for (i = 0; i < 4; i++) {
            in = 0x0f & *key;
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
#elif INPUT_DEVICE == INPUT_DEVICE_PC_KB
int kb_get_key(void)
{
    int key;
    key = MXC_UART_ReadCharacter(MXC_UART0);
    return key;
}
#endif

int kb_init(void)
{
    int rv = 0;

#if INPUT_DEVICE == INPUT_DEVICE_EvKit_KB
    mxc_skbd_config_t skb_cfg;

    skb_cfg.inputs = MXC_SKBD_KBDIO4 | MXC_SKBD_KBDIO5 | MXC_SKBD_KBDIO6 | MXC_SKBD_KBDIO7;
    skb_cfg.outputs = MXC_SKBD_KBDIO0 | MXC_SKBD_KBDIO1 | MXC_SKBD_KBDIO2 | MXC_SKBD_KBDIO3;
    skb_cfg.debounce = MXC_V_SKBD_CTRL1_DBTM_TIME10MS;
    skb_cfg.irq_handler = (irq_handler_t)keypadHandler;
    skb_cfg.reg_erase = 1;

    MXC_SKBD_PreInit();

    rv = MXC_SKBD_Init(skb_cfg);

    if (rv) {
        return E_UNINITIALIZED;
    }

    rv = MXC_SKBD_EnableInterruptEvents(MXC_SKBD_INTERRUPT_STATUS_PUSHIS);

    if (rv) {
        return E_UNINITIALIZED;
    }

#else
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);
#endif

    return rv;
}

int kb_read_num(unsigned int timeout)
{
    (void)timeout;
    int key;
    int num = 0;

    while (1) {
        key = kb_get_key();

        if (key > 0) {
            //echo
            MXC_UART_WriteCharacter(MXC_UART0, (unsigned char)key);

            if ((key >= '0') && (key <= '9')) {
                num = num * 10 + (key - '0');
            }

            if ((key == KEY_ENTER) || (key == KEY_ENTER2)) {
                break;
            } else if (key == KEY_CANCEL) {
                num = -1;
                break;
            }
        }
    }

    MXC_UART_ClearRXFIFO(MXC_UART0);

    return num;
}

int kb_select_from_list_xcol(const char *title, const char **items, int nb_items, int nb_col)
{
    int i, k;
    int selected = 0;
    int nb_row;
    char buf[512];
    char item_data[64];
    int index;

    if (title) {
        printf("\n\n%s\n", title);
    }

    printf("---------------------------------------------------------\n");

    if (nb_col == 0) {
        nb_col = 1;
    }

    nb_row = nb_items / nb_col;

    if (nb_items % 2) {
        nb_row++;
    }

    if (nb_items > 1) {
        for (i = 0; i < nb_row; i++) {
            buf[0] = '\0';

            for (k = 0; k < nb_col; k++) {
                index = i + (k * nb_row);

                if (index < nb_items) {
                    snprintf(item_data, sizeof(item_data), "%-3d- %-32s ", index + 1, items[index]);
                    strncat(buf, item_data, 64);
                }
            }

            printf("%s\n", buf);
        }

        printf("\nPlease select:\n");

        selected = kb_read_num(0);

        if (selected == -1) {
            selected = 0;
        }

        if (selected > nb_items) {
            selected = 0;
        }
    } else if (nb_items == 1) {
        selected = 1;
    }

    return selected;
}
