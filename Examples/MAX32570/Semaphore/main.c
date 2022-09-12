
/**
 * @file        main.c
 * @brief       Semaphore example
 * @details     Press button to overwrite a global variable. If someone is already writing to it, deny the right
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

#include "bitmap.h"
#include "tft_ssd2119.h"

/***** Definitions *****/
#define KEY_1 1 //P(3.07)
#define KEY_2 2 //P(3.06)

#define BUTTON_SIZE_X 42 //
#define BUTTON_SIZE_Y 41 //

/***** Globals *****/
volatile int A_active = 0;
volatile int B_active = 0;

int sharedVariable = 0;

/***** Functions *****/
static void PB_AWrites(void)
{
    int retval;

    //First check if A is already writing
    if (!A_active) {
        //Check if B is writing
        retval = MXC_SEMA_CheckSema(0);

        if (retval == E_NO_ERROR) {
            if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
                printf("A acquired semaphore!\n");
                printf("A started writing...\n");
                sharedVariable = 100;
                printf("Shared Variable = %d\n\n", sharedVariable);
                A_active = 1;
            }
        } else if (retval == E_BUSY) {
            printf("A can't write right now.\n");
            printf("Shared Variable = %d\n\n", sharedVariable);
            return;
        }
    } else {
        A_active = !A_active;
        //Semaphore should be busy...
        retval = MXC_SEMA_CheckSema(0);

        if (retval == E_BUSY) {
            printf("A stopped writing.\n");
            MXC_SEMA_FreeSema(0);
            printf("A dropped the semaphore...\n\n");
        } else {
            printf("Something went wrong.\n\n");
            return;
        }
    }

    printf("\n");

    return;
}

static void PB_BWrites(void)
{
    int retval;

    //First check if B is already writing
    if (!B_active) {
        //Check if A is writing
        retval = MXC_SEMA_CheckSema(0);

        if (retval == E_NO_ERROR) {
            if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
                printf("B acquired semaphore!\n");
                printf("B started writing...\n");
                sharedVariable = 200;
                printf("Shared Variable = %d\n\n", sharedVariable);
                B_active = 1;
            }
        } else if (retval == E_BUSY) {
            printf("B can't write right now.\n");
            printf("Shared Variable = %d\n\n", sharedVariable);
            return;
        }
    } else {
        B_active = !B_active;
        //Semaphore should be busy...
        retval = MXC_SEMA_CheckSema(0);

        if (retval == E_BUSY) {
            printf("B stopped writing.\n");
            MXC_SEMA_FreeSema(0);
            printf("B dropped the semaphore...\n\n");
        } else {
            printf("Something went wrong.\n\n");
            return;
        }
    }

    printf("\n");

    return;
}

static int setup_display(void)
{
    int x = 100;
    int y = 150;

    MXC_TFT_Init();
    MXC_TS_Init();
    //
    MXC_TS_Start();

    MXC_TFT_ShowImage(11, 7, logo_white_bg_white_bmp);

    MXC_TFT_ShowImage(x, y, key_1_bg_white_bmp);
    MXC_TS_AddButton(x, y, x + BUTTON_SIZE_X, y + BUTTON_SIZE_Y, KEY_1);

    x += BUTTON_SIZE_X + 40; // add 40pixel space
    MXC_TFT_ShowImage(x, y, key_2_bg_white_bmp);
    MXC_TS_AddButton(x, y, x + BUTTON_SIZE_X, y + BUTTON_SIZE_Y, KEY_2);

    // set up font
    MXC_TFT_SetFont(urw_gothic_13_grey_bg_white);

    return 0;
}

// *****************************************************************************
int main(void)
{
    int key;
    printf("***** Semaphore Example *****\n");
    printf("KEY_1 (P3.07)= A tries to write\n");
    printf("KEY_2 (P3.06)= B tries to write\n");
    printf("\n");

    setup_display();

    MXC_SEMA_Init();

    if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
        printf("Semaphore acquired.\n");
    }

    if ((MXC_SEMA_CheckSema(0)) == E_NO_ERROR) {
        printf("Semaphore free.\n");
    } else {
        printf("Semaphore locked.\n");
    }

    MXC_SEMA_FreeSema(0);

    if ((MXC_SEMA_CheckSema(0)) == E_NO_ERROR) {
        printf("Semaphore free.\n");
    } else {
        printf("Semaphore locked.\n");
    }

    printf("\n\nExample running.\n\n");

    while (1) {
        // check touch screen key
        key = MXC_TS_GetKey();

        if (key > 0) {
            switch (key) {
            case KEY_1:
                PB_AWrites();
                break;

            case KEY_2:
                PB_BWrites();
                break;

            default:
                break;
            }
        }
    }
}
