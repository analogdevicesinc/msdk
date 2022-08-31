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
 * @brief   FaceID EvKit Demo
 *
 * @details
 *
 */

#define S_MODULE_NAME "MAIN"

/***** Includes *****/
#include "board.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_delay.h"
#include "state.h"
#include <mxc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif
#include "MAXCAM_Debug.h"
#include "faceID.h"
#include "keypad.h"
#include "lp.h"
#include "rtc.h"
#include "sema_regs.h"

/***** Definitions *****/
//----------------------------------------------------------
#define SYS_DIV 1 // 1,2,4,8,16,32,128    clock div
#define CLOCK_SOURCE 0 // 0: IPO,  1: ISO, 2: IBRO
//------------------------------------------------------------

extern volatile void const* __FlashStart_; // Defined in linker file

__attribute__((section(
    ".shared__at__mailbox"))) volatile uint32_t mail_box[ARM_MAILBOX_SIZE + RISCV_MAILBOX_SIZE];
volatile uint32_t* arm_mail_box = &mail_box[0];
volatile uint32_t* riscv_mail_box = &mail_box[ARM_MAILBOX_SIZE];

void WakeISR(void)
{
    MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN & ~MXC_F_SEMA_IRQ0_CM4_IRQ;
}

void WUT_IRQHandler()
{
    MXC_WUT_IntClear();
}

#ifdef LP_MODE_ENABLE
static void ARM_low_power(int lp_mode)
{
    switch (lp_mode) {
    case 0:
        PR_DEBUG("Active\n");
        break;

    case 1:
        PR_DEBUG("Enter SLEEP\n");
        MXC_LP_EnterSleepMode();
        PR_DEBUG("Exit SLEEP\n");
        break;

    case 2:
        PR_DEBUG("Enter LPM\n");
        MXC_LP_EnterLowPowerMode();
        PR_DEBUG("Exit LPM\n");
        break;

    case 3:
        PR_DEBUG("Enter UPM\n");
        MXC_LP_EnterMicroPowerMode();
        PR_DEBUG("Exit UPM\n");
        break;

    case 4:
        PR_DEBUG("Enter STANDBY\n");
        MXC_LP_EnterStandbyMode();
        PR_DEBUG("Exit STANDBY\n");
        break;

    case 5:
        PR_DEBUG("Enter BACKUP\n");
        MXC_LP_EnterBackupMode();
        PR_DEBUG("Exit BACKUP\n");
        break;

    case 6:
        PR_DEBUG("Enter POWERDOWN, disable WUT\n");
        MXC_WUT_Disable();
        MXC_Delay(SEC(2));
        MXC_LP_EnterPowerDownMode();
        PR_DEBUG("Exit SHUTDOWN\n");
        break;

    default:
        PR_DEBUG("Enter SLEEP\n");
        MXC_LP_EnterSleepMode();
        PR_DEBUG("Exit SLEEP\n");
        break;
    }
}
#endif

/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);

/************************************ VARIABLES ******************************/
static void screen_faceID(void);
static int init(void);
static int key_process(int key);

State g_state = { "faceID", init, key_process, NULL, 0 };

#ifdef TFT_ENABLE
static text_t screen_msg[] = {
    // info
    { (char*)"FACEID DEMO", strlen("FACEID DEMO") },
    { (char*)"Process Time:", strlen("Process Time:") },
};
#ifdef BOARD_EVKIT_V1
static int bitmap = logo_white_bg_darkgrey_bmp;
static int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
static int bitmap = (int)&logo_rgb565[0];
static int font = (int)&SansSerif16x16[0];
#endif
#endif

/********************************* Static Functions **************************/
static void screen_faceID(void)
{
#ifdef TFT_ENABLE
    MXC_TFT_SetPalette(bitmap);
    MXC_TFT_SetBackGroundColor(4);
    // MXC_TFT_ShowImage(3, 5, bitmap);
#ifdef BOARD_EVKIT_V1
    MXC_TFT_ShowImage(BACK_X, BACK_Y, left_arrow_bmp); // back button icon
#endif
    MXC_TFT_PrintFont(98, 5, font, &screen_msg[0], NULL); // FACEID DEMO
    MXC_TFT_PrintFont(12, 240, font, &screen_msg[1], NULL); // Process Time:
    // texts
#ifdef TS_ENABLE
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(BACK_X, BACK_Y, BACK_X + 48, BACK_Y + 39, KEY_1); // Back
#endif
#endif // #ifdef TFT_ENABLE
}

static int init(void)
{
    uint8_t* raw;
    uint32_t w, h;
    area_t area1 = { 150, 240, 50, 30 };
    area_t area2 = { 60, 290, 180, 30 };

    screen_faceID();

    while (1) {
#ifdef TS_ENABLE
        int key;
        /* Check pressed touch screen key */
        key = MXC_TS_GetKey();

        if (key > 0) {
            // Start Wakeup Timer in case RISC-V sleeps
            MXC_WUT_Enable();
            arm_mail_box[0] = STOP_FACEID;
            key_process(key);
        }

        if (state_get_current() != &g_state) {
            break;
        }
#endif
        MXC_LP_EnterSleepMode(); // ARM sleeps and waits for semaphore from RISC

        // Get and display image
        if (riscv_mail_box[0] == IMAGE_READY) {
            raw = (uint8_t*)riscv_mail_box[1];
            h = riscv_mail_box[2];
            w = riscv_mail_box[3];
            // Clear mailbox
            riscv_mail_box[0] = 0;

#ifdef TS_ENABLE
            /* Disable TouchScreen to avoid collision with TFT */
            MXC_TS_Stop();
#endif

#ifdef TFT_ENABLE // Display captured image
#ifdef BOARD_EVKIT_V1
            MXC_TFT_ShowImageCameraRGB565(X_IMAGE_START, Y_IMAGE_START, raw, h, w);
#endif
#ifdef BOARD_FTHR_REVA
            MXC_TFT_ShowImageCameraRGB565(X_IMAGE_START, Y_IMAGE_START, raw, w, h);
#endif
#endif // #ifdef TFT_ENABLE

#ifdef TS_ENABLE
            /* Enable TouchScreen */
            MXC_TS_Start();
#endif

        } // Get FaceID result
        else if (riscv_mail_box[0] == RESULT_READY) {
            text_t cnnTime, cnnResult;

            cnnTime.data = (char*)&riscv_mail_box[2];
            cnnTime.len = riscv_mail_box[1];
            cnnResult.data = (char*)&riscv_mail_box[7];
            cnnResult.len = riscv_mail_box[6];

            // Clear mailbox
            riscv_mail_box[0] = 0;

#ifdef TS_ENABLE
            /* Disable TouchScreen to avoid collision with TFT */
            MXC_TS_Stop();
#endif

#ifdef TFT_ENABLE
            // Display CNN result
            MXC_TFT_ClearArea(&area1, 4);
            MXC_TFT_PrintFont(150, 240, font, &cnnTime, NULL);
            MXC_TFT_ClearArea(&area2, 4);
            MXC_TFT_PrintFont(CAPTURE_X, CAPTURE_Y, font, &cnnResult, NULL);
#endif // #ifdef TFT_ENABLE

#ifdef TS_ENABLE
            /* Enable TouchScreen */
            MXC_TS_Start();
#endif

            // Enable WUT
            MXC_WUT_Enable();
            LED_On(0);
#ifdef LP_MODE_ENABLE
            /* Got to low power mode for LP_TIME (msec) */
            ARM_low_power(LP_MODE);
#endif
            LED_Off(0);
        }
    }

    return 0;
}

static int key_process(int key)
{
    switch (key) {
    case KEY_1:
        state_set_current(get_home_state());
        break;

    default:
        break;
    }

    return 0;
}

/********************************* Public Functions **************************/
State* get_faceID_state(void)
{
    return &g_state;
}

int main(void)
{
    int key;
    State* state;
    uint32_t ticks;
    mxc_wut_cfg_t cfg;

    printf("\n\nStart ARM\n");
    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    printf("SYS Clock Div= %d \n", SYS_DIV);

    switch (SYS_DIV) {
    case 1:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1;
        break;

    case 2:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV2;
        break;

    case 4:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV4;
        break;

    case 8:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV8;
        break;

    case 16:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV16;
        break;

    default:
        printf("UNKNOWN DIV \n");

        while (1) { }
    }

    printf("MXC_GCR->clkctrl: SYSCLK_SEL %x \n",
        (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >> MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS);
    printf("MXC_GCR->clkctrl: SYSCLK_DIV %x \n",
        (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV) >> MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS);

    switch (CLOCK_SOURCE) {
    case 0:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_IPO_PD; // enable IPO=100MHz during sleep
        break;

    case 1:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_ISO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_ISO_PD; // enable ISO=60MHz during sleep
        break;

    case 2:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IBRO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_IBRO_PD; // enable IBRO=7.3728MHz during sleep
        break;

    default:
        printf("UNKNOWN CLOCK SOURCE \n");

        while (1) { }
    }

    SystemCoreClockUpdate();

    /* Initialize RTC */
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

#ifdef TFT_ENABLE

#ifdef BOARD_EVKIT_V1
    /* Initialize TFT display */
    MXC_TFT_Init();
    /* Set the screen rotation */
    MXC_TFT_SetRotation(SCREEN_ROTATE);
    /* Change entry mode settings */
    MXC_TFT_WriteReg(0x0011, 0x6858);
#endif

#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_180);
    MXC_TFT_SetBackGroundColor(4);
    MXC_TFT_SetForeGroundColor(WHITE); // set font color to white
#endif

#ifdef TS_ENABLE
    /* Initialize Touch Screen controller */
    MXC_TS_Init();
    MXC_TS_Start();
#endif

    /* Display Home page */
    state_init();
#endif // #ifdef TFT_ENABLE

    // Init ARM mailbox
    memset((void*)arm_mail_box, 0, sizeof(arm_mail_box));
    // Init RISCV mailbox
    memset((void*)riscv_mail_box, 0, sizeof(riscv_mail_box));

    printf("ARM mailbox: %x\n", &arm_mail_box[0]);
    printf("RISC-V mailbox: %x\n", &riscv_mail_box[0]);

    // Let RISC-V run
    MXC_FCR->urvbootaddr = (uint32_t)&__FlashStart_; // Set RISC-V boot address
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SMPHR); // Enable Semaphore clock
    MXC_NVIC_SetVector(RISCV_IRQn, WakeISR); // Set wakeup ISR
    /* set wakeup by risc-v */
    NVIC_EnableIRQ(RISCV_IRQn);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CPU1); // Enable RISC-V clock

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(LP_TIME, MXC_WUT_UNIT_MILLISEC, &ticks);

    // config structure for one shot timer to trigger in a number of ticks
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;

    // Init WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);

    // Config WUT
    MXC_WUT_Config(&cfg);

    MXC_LP_EnableWUTAlarmWakeup();

    NVIC_EnableIRQ(WUT_IRQn);

#ifndef TFT_ENABLE
    int i;

    for (i = 0; i < (1 << 27); i++) { }
    // Let debugger interrupt if needed

    __WFI(); // ARM sleeps
#endif

#ifndef TS_ENABLE
    key = KEY_1;
#endif

    while (1) {
        /* Get current screen state */
        state = state_get_current();
#ifdef TS_ENABLE
        /* Check pressed touch screen key */
        key = MXC_TS_GetKey();
#endif

        if (key > 0) {
            // Start Wakeup Timer in case RISC-V sleeps
            MXC_WUT_Enable();
            arm_mail_box[0] = START_FACEID;
            state->prcss_key(key);
        }
    }

    return 0;
}
