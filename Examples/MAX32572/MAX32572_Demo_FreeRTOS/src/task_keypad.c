/*
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

#include <string.h>

#include "MAX32xxx.h"
#include "keypad.h"
#include "message.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

static unsigned int is_key_pressed;
extern xQueueHandle xQueueMain;
static xSemaphoreHandle xKBDLock;

/* keys mapping on the keyboard */
static unsigned char keyboard_map[16] = {KEY_F, KEY_E, KEY_D, KEY_C, KEY_3, KEY_6, KEY_9, KEY_B,
                                         KEY_2, KEY_5, KEY_8, KEY_0, KEY_1, KEY_4, KEY_7, KEY_A};

void keypad_stop(void)
{
    MXC_SKBD_DisableInterruptEvents(MXC_SKBD_INTERRUPT_STATUS_PUSHIS |
                                    MXC_SKBD_INTERRUPT_STATUS_OVERIS);
}

void keypad_start(void)
{
    MXC_SKBD_EnableInterruptEvents(MXC_SKBD_INTERRUPT_STATUS_PUSHIS |
                                   MXC_SKBD_INTERRUPT_STATUS_OVERIS);
}

void keypad_handler(void)
{
    unsigned int status;

    MXC_SKBD_InterruptStatus(&status);

    if (MXC_F_SKBD_ISR_OVERIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_OVERIS);
    }
    if (MXC_F_SKBD_ISR_PUSHIS & status) {
        is_key_pressed = 1;
        /* Clear interruption */
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_PUSHIS);
    }
    if (MXC_F_SKBD_ISR_RELEASEIS & status) {
        MXC_SKBD_ClearInterruptStatus(MXC_F_SKBD_ISR_RELEASEIS);
    }
    xSemaphoreGiveFromISR(xKBDLock, NULL);

    return;
}

int keypad_init(void)
{
    int rv = 0;
    mxc_skbd_config_t skb_cfg;

    skb_cfg.inputs      = MXC_SKBD_KBDIO4 | MXC_SKBD_KBDIO5 | MXC_SKBD_KBDIO6 | MXC_SKBD_KBDIO7;
    skb_cfg.outputs     = MXC_SKBD_KBDIO0 | MXC_SKBD_KBDIO1 | MXC_SKBD_KBDIO2 | MXC_SKBD_KBDIO3;
    skb_cfg.debounce    = MXC_V_SKBD_CR1_DBTM_TIME10MS;
    skb_cfg.ioselect    = 0;
    skb_cfg.irq_handler = (irq_handler_t)keypad_handler;
    skb_cfg.reg_erase   = 1;

    MXC_SKBD_PreInit();

    rv = MXC_SKBD_Init(skb_cfg);
    if (rv) {
        return E_UNINITIALIZED;
    }

    keypad_start();

    return 0;
}

void vGetKEYTask(void* pvParameters)
{
    (void)pvParameters;

    unsigned short* key;
    mxc_skbd_keys_t keys = {0, 0, 0, 0};
    volatile unsigned int in;
    volatile unsigned int out;
    volatile unsigned int i;
    char key_press[5];
    char* key_ptr = key_press;
    message_t msgKBD;

    keypad_init();

    xKBDLock = xSemaphoreCreateBinary();

    msgKBD.pcType = 'K'; // it is keyboard message

    for (;;) {
        while (xSemaphoreTake(xKBDLock, 0xFFFF) != pdTRUE) {
            ;
        }

        key_press[0] = '\0';

        if (is_key_pressed) {
            MXC_SKBD_ReadKeys(&keys);
            key = &keys.key0;
            for (i = 0; i < 4; i++) {
                in  = 0x0f & *key;
                out = (0xf0 & *key) >> 4;

                if (*key) {
                    *key_ptr         = keyboard_map[(in - 4) * 4 + out];
                    key_press[i]     = *key_ptr;
                    key_press[i + 1] = '\0';
                }
                *key = 0;
                key++;
            }
            /* Clear interruption */
            MXC_SKBD_ClearInterruptStatus(MXC_SKBD_INTERRUPT_STATUS_PUSHIS);

            // Send Key
            msgKBD.pcMessage[0] = key_press[0];
            xQueueSendToFrontFromISR(xQueueMain, &msgKBD, NULL);
        }
    }
}
