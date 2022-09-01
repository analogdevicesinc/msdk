/******************************************************************************
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

#include "MAX32xxx.h"
#include "keypad.h"
#include "task_smartcard.h"
#include "utils.h"

#include "MAX325xx_afe.h"
#include "OSWrapper.h"
#include "sc_errors.h"
#include "sc_types.h"
#include "smartcard_api.h"

#include "demo_config.h"

/********************************* 		DEFINES		 *************************/

/********************************* 	 	TYPE DEF	 *************************/

/********************************* 		VARIABLES	 *************************/
mxc_sc_context_t sc_context = { 0 };

static UartId_t g_uartId = SCI_1;
static MAX325xxSlots_t g_card_slot = SCI_1_BYPASS_SLOT;

ActivationParams_t ActivationParams = {
#if SMARTCARD_EXT_AFE_Voltage == SMARTCARD_EXT_AFE_5V
    .IccVCC = VCC_5V,
#else
    .IccVCC = VCC_3V,
#endif
    .IccResetDuration = 108, /* 108*372 clock cycles*/
    .IccATR_Timeout = 20160, /* 20160 etus*/
    .IccTS_Timeout = 114, /* 114*372 clock cycles*/
    .IccWarmReset = 0,
};

/********************************* Static Functions **************************/
int mxc_sc_init(mxc_sc_id_t id)
{
    /* For the first time initialization */
    if (!sc_context.first_init) {
        /* Initialize the smart card context information to zero's */
        sc_context.sc[MXC_SC_DEV0].reg_sc = MXC_SC0;
        sc_context.sc[MXC_SC_DEV1].reg_sc = MXC_SC1;
        sc_context.first_init = 1;
    }

    /* Check input parameter */
    if (id > MXC_SC_DEV_MAX) {
        return E_INVALID;
    }

    /* Enable the timer clock */
    /* Clear the bit position to enable clock to timer device */
    switch (id) {
    case MXC_SC_DEV0:
        MXC_SYS_Reset_Periph(MXC_SYS_PERIPH_CLOCK_SC0);
        while (MXC_GCR->rst1 & MXC_F_GCR_RST1_SC0) { }

        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SC0);
        break;
    case MXC_SC_DEV1:
        MXC_SYS_Reset_Periph(MXC_SYS_PERIPH_CLOCK_SC1);
        while (MXC_GCR->rst1 & MXC_F_GCR_RST1_SC1) { }

        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SC1);
        break;
    default:
        return E_INVALID;
    }

    return E_NO_ERROR;
}

static void smart_card_inserted_interrupt(CardState_t cardstate)
{
    if (cardstate == ICC_INSERTION) {
        //...
    }
    if (cardstate == ICC_REMOVAL) {
        //...
    }
    NVIC_ClearPendingIRQ(SC1_IRQn);
}

void sc_set_afe_intterrupt(unsigned int status)
{
    if (status == 1) {
        status = bTRUE;
    } else {
        status = bFALSE;
    }
    SCAPI_ioctl(g_card_slot, IOCTL_ENABLE_AFE_INTERRUPT, &status);
}

int sc_read_atr(char* msg, int* msg_len)
{
    unsigned char atr[256];
    unsigned int i;
    IccReturn_t retval;
    uint32_t status = 0;

    // no messages yet
    *msg_len = 0;

    retval = SCAPI_ioctl(g_card_slot, IOCTL_GET_CARD_STATE, &status);

    if (!((IccReturn_t)status == ICC_ERR_REMOVED)) {
        /*power up the card */
        status = POWER_UP;
        ActivationParams.IccWarmReset = bFALSE;
        retval = SCAPI_ioctl(g_card_slot, IOCTL_POWER_CARD, &status);
        if (ICC_OK != retval) {
            goto read_atr_out;
        }

        /*
         * Read the ATR and save into the atr buffer
         * as output, status will contains the exact ATR length
         */
        status = sizeof(atr);
        retval = SCAPI_read(g_card_slot, atr, &status);
        if (retval) {
            memcpy(msg, "Smartcard Error", 15);
            *msg_len = 15;
        } else {
            memset(msg, ' ', 24);
            switch (status) {
            case 1:
                *msg_len = 2;
                utils_hex2char(atr[0], &msg[0]);
                break;
            case 2:
                *msg_len = 5;
                utils_hex2char(atr[0], &msg[0]);
                utils_hex2char(atr[1], &msg[3]);
                break;
            default:
                utils_hex2char(atr[0], &msg[0]);
                utils_hex2char(atr[1], &msg[3]);
                if (status < 6) {
                    for (i = 2; i < status; i++) {
                        utils_hex2char(atr[i], &msg[3 * i]);
                    }
                    *msg_len = 3 * status - 1;
                } else {
                    utils_hex2char(atr[2], &msg[6]);
                    msg[9] = '.';
                    msg[10] = '.';
                    msg[11] = '.';
                    utils_hex2char(atr[status - 2], &msg[13]);
                    utils_hex2char(atr[status - 1], &msg[16]);
                    *msg_len = 18;
                }
                break;
            }
        }
    }

read_atr_out:
    /*power off the card */
    status = POWER_DOWN;
    SCAPI_ioctl(g_card_slot, IOCTL_POWER_CARD, &status);

    return retval;
}

int sc_init(void)
{
    uint32_t status = 0;
    IccReturn_t retval = ICC_OK;

    /* enable interrupts*/
    OSWrapper_Interrupt_enable();

    /* open slot INTERNAL_AFE for SC0 interfaces */
    SCAPI_open(g_uartId, g_card_slot);

    /* Set the card frequency */
    status = 3200000; // 3.2MHz
    retval = SCAPI_ioctl(g_card_slot, IOCTL_SET_CLOCK_FREQ, &status);
    if (ICC_OK != retval) {
        return retval;
    }

    /* as the card has been powered off, we must reset
     * the initparams, emv mode and working buffer
     */

    /* Set the ATR timings  + card voltage */
    retval = SCAPI_ioctl(g_card_slot, IOCTL_SET_INITPARAMS, (void*)&ActivationParams);
    if (ICC_OK != retval && retval != ICC_ERR_REMOVED) {
        return retval;
    }

    /* Attach AFE user interrupt */
    retval = SCAPI_ioctl(g_card_slot, IOCTL_SET_PRESENCE_IRQ, smart_card_inserted_interrupt);
    if (retval) {
        return retval;
    }

    /* Switch the stack to EMV mode*/
    status = 1;
    retval = SCAPI_ioctl(g_card_slot, IOCTL_SET_EMVMODE, &status);
    if (ICC_OK != retval) {
        return retval;
    }

    return 0; // ICC_OK
}

IccReturn_t sc_get_card_status(void)
{
    uint32_t status = ICC_ERR_REMOVED;

    SCAPI_ioctl(g_card_slot, IOCTL_GET_CARD_STATE, &status);

    return (IccReturn_t)status;
}
