/*
 * ST8034.c
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

#include <stdint.h>
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "sc.h"
#include "gpio.h"

#include "MAX325xx_bypass_afe.h"
#include "sc_errors.h"
#include "sc_states.h"
#include "sc_config.h"
#include "sc_regs.h"
#include "iccabstract.h"
#include "OSWrapper.h"
#include "MAX325xx_afe.h"
#include "MAX325xx_afe_private.h"
#include "MAX325xx_uart.h"
#include "MAX325xx_uart_private.h"
#include "slot.h"

#include "demo_config.h"

/**** NOTE: this AFE driver is intented to be connected on bypass mode ****/
extern mxc_sc_context_t sc_context;

#define MAX325XX_EVKIT_EXT_AFE_PORT (MXC_GPIO0) /* Port 0 */

#define MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN (MXC_GPIO_PIN_21) /* CMDVCC pin mask */
#define MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS (21) /* CMDVCC pin pos */
#define MAX325XX_EVKIT_EXT_AFE_OFF_PIN (MXC_GPIO_PIN_23) /* OFF pin mask */
#define MAX325XX_EVKIT_EXT_AFE_OFF_POS (23) /* OFF pin pos */
#define MAX325XX_EVKIT_EXT_AFE_5V3V_PIN (MXC_GPIO_PIN_22) /* 5V3V pin mask */
#define MAX325XX_EVKIT_EXT_AFE_5V3V_POS (22) /* 5V3V pin pos */

#define MAX325XX_EVKIT_EXT_AFE_OFFLINE (1) /* when CMDVCC = 1, SAM is off */
#define MAX325XX_EVKIT_EXT_AFE_CARD_ABSENT (0) /* OFF pin is low when no card */

#define ST8034_NOT_SELECTED (0)
#define ST8034_SELECTED (1)

#if SMARTCARD_EXT_AFE_Voltage == SMARTCARD_EXT_AFE_5V
#define MAX325XX_EVKIT_EXT_AFE_5V3V_VALUE (1) // Means configure sc gpio to 5V
#else
#define MAX325XX_EVKIT_EXT_AFE_5V3V_VALUE (0) // Means configure sc gpio to 3V
#endif

static IccVoltage_t icc_voltage = VCC_5V;

IccReturn_t bypassSelect(SlotContext_t *SlotCtx, boolean_t Selected)
{
    //uint32_t ChipSelect = ST8034_NOT_SELECTED;
    SCControl_t sccr = { .word = 0 }; /* control register value */
    UartState_t *UartState = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if ((NULL == UartState) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    /* check if the requested slot is available on our target */
    if ((SlotCtx->SlotId != SCI_0_BYPASS_SLOT) && (SlotCtx->SlotId != SCI_1_BYPASS_SLOT)) {
        return ICC_ERR_BAD_SLOT;
    }

    sccr.word = sc_context.sc[SlotCtx->UartId].reg_sc->SC_CR;

#if !defined(__MAX32590) && !defined(__MAX32591) && !defined(__MAX32565) && !defined(__MAX32572)
#if defined(__MAX32510)
    /* Always select bypass AFE, since there is no internal.
     * Otherwise, clock is getting lost after RX and Test Suite fails.
     */
    sccr.bits.BYP_PHY = 1;
#else
    if (bTRUE == Selected) {
        //ChipSelect          = ST8034_SELECTED;
        sccr.bits.BYP_PHY = 1;
    } else {
        //ChipSelect          = ST8034_NOT_SELECTED;
        sccr.bits.BYP_PHY = 0;
    }
#endif
#endif
    sc_context.sc[SlotCtx->UartId].reg_sc->SC_CR = sccr.word;
    MXC_Delay(2);

    UartState->ActiveSlot = SlotCtx->SlotId;

    return ICC_OK;
}

IccReturn_t bypassSetVoltage(SlotContext_t *SlotCtx, IccVoltage_t Voltage)
{
    uint32_t CurrentVoltage = 0;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    if ((SlotCtx->SlotId != SCI_0_BYPASS_SLOT) && (SlotCtx->SlotId != SCI_1_BYPASS_SLOT)) {
        return ICC_ERR_BAD_SLOT;
    }

    /* check if the card is already powered */
    CurrentVoltage =
        MXC_GPIO_OutGet(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN) >>
        MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS;

    if (MAX325XX_EVKIT_EXT_AFE_OFFLINE != CurrentVoltage) {
        /* if the card is already powered, we cannot change the voltage ! */
        return ICC_ERR_POWERED;
    }

    switch (Voltage) {
    case VCC_5V:
    case VCC_3V:
    case VCC_1V8:
        icc_voltage = Voltage;
        break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    return ICC_OK;
}

IccReturn_t bypassApplyVoltage(SlotContext_t *SlotCtx)
{
    uint32_t CurrentVoltage = 0;

    if (NULL == SlotCtx)
        return ICC_ERR_NULL_PTR;

    if ((SlotCtx->SlotId != SCI_0_BYPASS_SLOT) && (SlotCtx->SlotId != SCI_1_BYPASS_SLOT))
        return ICC_ERR_BAD_SLOT;

    /* check if the card is already powered */
    CurrentVoltage =
        MXC_GPIO_OutGet(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN) >>
        MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS;

    if (MAX325XX_EVKIT_EXT_AFE_OFFLINE != CurrentVoltage) {
        /* if the card is already powered, we cannot change the voltage ! */
        return ICC_ERR_POWERED;
    }

    switch (icc_voltage) {
    case VCC_5V:
    case VCC_3V:
    case VCC_1V8:
        CurrentVoltage = 0;
        break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN,
                    (CurrentVoltage << MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS));
    //MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_OFF_PIN, (0 << MAX325XX_EVKIT_EXT_AFE_OFF_POS));

    return ICC_OK;
}

IccReturn_t bypassPower(SlotContext_t *SlotCtx, CardPowerState_t PowerUp)
{
    IccReturn_t retval = ICC_OK;
    SCPin_t scpin = { .word = 0 };
    UartData_t *UartData = NULL;
    UartState_t *UartState = NULL;

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    UartData = SlotCtx->UartData;
    if (NULL == UartData) {
        return ICC_ERR_NULL_PTR;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if ((NULL == UartState) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    if ((SlotCtx->SlotId != SCI_0_BYPASS_SLOT) && (SlotCtx->SlotId != SCI_1_BYPASS_SLOT)) {
        return ICC_ERR_BAD_SLOT;
    }

    switch (PowerUp) {
    case POWER_DOWN:
        scpin.word = sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN;
        scpin.bits.CRDRST = RESET_ACTIVE;
        sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN = scpin.word;
        MXC_Delay(2);

        /*
		 * wait CWT to help the test tool
		 * (if we deactivate too fast, the Lab test tool crashes)
		 */
        IccWait(SlotCtx, SlotCtx->IccProtocolConfig.IccCharWaitingTime);

        /* set CMDVCC input to be high to keep card power down */
        MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN,
                        (MAX325XX_EVKIT_EXT_AFE_OFFLINE << MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS));
        break;

    case POWER_UP:
        /*
		 * here we do not set the isPowering flag,
		 * this will be done when we release the RST signal
		 */
        retval = bypassApplyVoltage(SlotCtx);
        if (ICC_OK != retval) {
            return retval;
        }
        break;

    case RESET_DO:
        scpin.word = sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN;
        scpin.bits.CRDRST = RESET_ACTIVE;
        sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN = scpin.word;
        MXC_Delay(2);
        break;

    case RESET_RELEASE:
        /* Set the Powering bit
		 * this will indicates to the UART driver that the next
		 * data are the ATR
		 */
        SlotCtx->isPoweringUp = bTRUE;

        scpin.word = sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN;
        scpin.bits.CRDRST = !RESET_ACTIVE;
        sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN = scpin.word;
        MXC_Delay(2);
        break;

    default:
        return ICC_ERR_BAD_PARAMETER;
    }

    return ICC_OK;
}

/** @fn     teridian_73S8009rInterrupt_Handler
 *  @brief  Manage PHY Interrupts
 *
 */
void bypassInterrupt_Handler(void *param)
{
    (void)(param);
    return;
}

/** @fn                     AfeEnable IRQ
 *  @brief                  enable or disable AFE interrupts
 *  @param [in] SlotId      slot number, cf #MAX325xxSlots_t
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 */
IccReturn_t bypassEnableIrq(SlotContext_t *SlotCtx, int32_t enable)
{
    (void)enable;

    UartState_t *UartState = NULL;

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if ((NULL == UartState) || (NULL == (void *)UartState->UartAddress) || (NULL == SlotCtx) ||
        (NULL == SlotCtx->UartData)) {
        return ICC_ERR_NULL_PTR;
    }

    return ICC_OK;
}

/** @fn                     teridian_73S8009rGetCardStatus
 *  @brief                  Return the slot state
 *  @param [in] SlotCtx     AFE slot configuration context pointer (cf #SlotContext_t)
 *
 *  @return                     return an #IccReturn_t error code
 *  @retval    ICC_OK                    if the card is inserted and powered.
 *  @retval    ICC_ERR_REMOVED           if the card is not present.
 *  @retval    ICC_ERR_PRESENT_INACTIVE  if the card is present but not powered.
 *
 */
IccReturn_t bypassGetCardStatus(SlotContext_t *SlotCtx)
{
    uint32_t PinValue = 0;

    if (NULL == SlotCtx)
        return ICC_ERR_NULL_PTR;

    if ((SlotCtx->SlotId != SCI_0_BYPASS_SLOT) && (SlotCtx->SlotId != SCI_1_BYPASS_SLOT)) {
        return ICC_ERR_BAD_SLOT;
    }

    // Read OFF PIN
    PinValue = MXC_GPIO_InGet(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_OFF_PIN) >>
               MAX325XX_EVKIT_EXT_AFE_OFF_POS;

    /* Is a card present in the slot ? */
    if (MAX325XX_EVKIT_EXT_AFE_CARD_ABSENT == PinValue) {
        return ICC_ERR_REMOVED;
    }

    /* check if the card is powered */
    PinValue = MXC_GPIO_OutGet(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN) >>
               MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS;

    /* SAM present but not powered */
    if (MAX325XX_EVKIT_EXT_AFE_OFFLINE == PinValue) {
        return ICC_ERR_PRESENT_INACTIVE;
    }

    /* SAM present and powered */
    return ICC_OK;
}

/** @var    bypass_AfeOps
 *  @brief  Analog Front End supported operations
 */
static const SlotOps_t bypass_AfeOps = {
    .select = bypassSelect,
    .setvoltage = bypassSetVoltage,
    .power = bypassPower,
    .getcardstatus = bypassGetCardStatus,
    .enableinterrupt = bypassEnableIrq,
};

/** @fn                     teridian_73S8009rInit
 *  @brief                  Initialize the Analog Front End for the SAM slot
 *
 *  @return                 return an #IccReturn_t error code
 *  @retval ICC_OK          if the AFE is now selected (enabled)
 *
 *  @note   The AFE init must be done *AFTER* the UART init.
 *  @note   As this driver is only for bare-metal, we can directly access to
 *          the GPIOs to configure/drive/
 */
IccReturn_t bypassInit(UartId_t UartId, MAX325xxSlots_t SlotId)
{
    SlotContext_t *SlotCtx = NULL;
    SCPin_t scpin = { .word = 0 };
    SCControl_t sccr = { .word = 0 };
    mxc_gpio_cfg_t config;
    UartState_t *UartState = NULL;

    if (UartId >= MAX325xx_INTERFACE_NUMBER) {
        return ICC_ERR_BAD_INTERFACE;
    }

    if ((SlotId != SCI_0_BYPASS_SLOT) && (SlotId != SCI_1_BYPASS_SLOT)) {
        return ICC_ERR_BAD_SLOT;
    }

    SlotCtx = IccRegisterAfe(SlotId, UartId, (SlotOps_t *)&bypass_AfeOps, NULL);

    if (NULL == SlotCtx) {
        return ICC_ERR_NULL_PTR;
    }

    UartState = (UartState_t *)(SlotCtx->UartData->PrivateData);
    if ((NULL == UartState) || (NULL == UartState->UartAddress)) {
        return ICC_ERR_NULL_PTR;
    }

    /* set the DUAL MODE bit and select the on-chip PHY*/
    sccr.word = sc_context.sc[SlotCtx->UartId].reg_sc->SC_CR;
#if !defined(__MAX32590) && !defined(__MAX32591) && !defined(__MAX32565) && !defined(__MAX32572)
    sccr.bits.DUAL_MODE = 1;
    sccr.bits.BYP_PHY = 1;
#endif
    sc_context.sc[SlotCtx->UartId].reg_sc->SC_CR = sccr.word;
    MXC_Delay(2);

    scpin.bits.CLKSEL = bTRUE;
    scpin.bits.CRDC4 = bTRUE;
    scpin.bits.CRDC8 = bTRUE;
    sc_context.sc[SlotCtx->UartId].reg_sc->SC_PN = scpin.word;
    MXC_Delay(2);

    /* Initialize CMDVCC */
    config.port = MAX325XX_EVKIT_EXT_AFE_PORT;
    config.mask = MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN;
    config.pad = MXC_GPIO_PAD_NONE;
    config.func = MXC_GPIO_FUNC_OUT;
    config.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&config);

    /* deactivate the card */
    MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_CMDVCC_PIN,
                    (MAX325XX_EVKIT_EXT_AFE_OFFLINE << MAX325XX_EVKIT_EXT_AFE_CMDVCC_POS));

    /* Initialize OFF */
    config.port = MAX325XX_EVKIT_EXT_AFE_PORT;
    config.mask = MAX325XX_EVKIT_EXT_AFE_OFF_PIN;
    config.pad = MXC_GPIO_PAD_PULL_UP;
    config.func = MXC_GPIO_FUNC_IN;
    config.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&config);

    /* Configure GPIO pin corresponding to the correct smart card interfaces(SC0 or SC1) */
    switch (UartId) {
    case SCI_0:
        /* Initialize GPIOs */
        /* Out - SC UART GPIOs ***************************************************/
        /* RST */
        config.port = MAX325xx_SC0_BYP_RST_PORT;
        config.mask = MAX325xx_SC0_BYP_RST;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* CLK */
        config.port = MAX325xx_SC0_BYP_CLK_PORT;
        config.mask = MAX325xx_SC0_BYP_CLK;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* In - SC UART GPIOs ***************************************************/
        /* IO */
        config.port = MAX325xx_SC0_BYP_IO_PORT;
        config.mask = MAX325xx_SC0_BYP_IO;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* 5V3V_PIN */
        config.port = MAX325XX_EVKIT_EXT_AFE_PORT;
        config.mask = MAX325XX_EVKIT_EXT_AFE_5V3V_PIN;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_OUT;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_5V3V_PIN,
                        (MAX325XX_EVKIT_EXT_AFE_5V3V_VALUE << MAX325XX_EVKIT_EXT_AFE_5V3V_POS));
        break;
#if defined(__MAX32552) || defined(__MAX32560) || defined(__MAX32565) || defined(__MAX32572)
    case SCI_1:
        /* Initialize GPIOs */
        /* Out - SC UART GPIOs ***************************************************/
        /* RST */
        config.port = MAX325xx_SC1_BYP_RST_PORT;
        config.mask = MAX325xx_SC1_BYP_RST;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* CLK */
        config.port = MAX325xx_SC1_BYP_CLK_PORT;
        config.mask = MAX325xx_SC1_BYP_CLK;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* In - SC UART GPIOs ***************************************************/
        /* IO */
        config.port = MAX325xx_SC1_BYP_IO_PORT;
        config.mask = MAX325xx_SC1_BYP_IO;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_ALT1;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        /* 5V3V_PIN */
        config.port = MAX325XX_EVKIT_EXT_AFE_PORT;
        config.mask = MAX325XX_EVKIT_EXT_AFE_5V3V_PIN;
        config.pad = MXC_GPIO_PAD_NONE;
        config.func = MXC_GPIO_FUNC_OUT;
        config.vssel = MXC_GPIO_VSSEL_VDDIOH;
        MXC_GPIO_Config(&config);

        MXC_GPIO_OutPut(MAX325XX_EVKIT_EXT_AFE_PORT, MAX325XX_EVKIT_EXT_AFE_5V3V_PIN,
                        (MAX325XX_EVKIT_EXT_AFE_5V3V_VALUE << MAX325XX_EVKIT_EXT_AFE_5V3V_POS));
#endif
    }

    SlotCtx->isCardInserted = bTRUE;

    /* Enable AFE interrupt */
    bypassEnableIrq(SlotCtx, bTRUE);

    return ICC_OK;
}
