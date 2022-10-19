/*
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

#include "MAX32xxx.h"
#include "bitmap.h"
#include "utils.h"

#include "task_logo_animation.h"
#include "tft_ssd2119.h"

/*********************************      DEFINES      *************************/
#define NB_SPRITE (32)

#define ANIMATION_PERIOD 30 //as ms

/*********************************      TYPE DEF     ************************/

/*********************************      VARIABLES    *************************/
static const int logo_maxim_data[] = {
    maxim_logo_only_small_000_bmp, maxim_logo_only_small_001_bmp, maxim_logo_only_small_002_bmp,
    maxim_logo_only_small_003_bmp, maxim_logo_only_small_004_bmp, maxim_logo_only_small_005_bmp,
    maxim_logo_only_small_006_bmp, maxim_logo_only_small_007_bmp, maxim_logo_only_small_008_bmp,
    maxim_logo_only_small_009_bmp, maxim_logo_only_small_010_bmp, maxim_logo_only_small_011_bmp,
    maxim_logo_only_small_012_bmp, maxim_logo_only_small_013_bmp, maxim_logo_only_small_014_bmp,
    maxim_logo_only_small_015_bmp, maxim_logo_only_small_016_bmp, maxim_logo_only_small_017_bmp,
    maxim_logo_only_small_018_bmp, maxim_logo_only_small_019_bmp, maxim_logo_only_small_020_bmp,
    maxim_logo_only_small_021_bmp, maxim_logo_only_small_022_bmp, maxim_logo_only_small_023_bmp,
    maxim_logo_only_small_024_bmp, maxim_logo_only_small_025_bmp, maxim_logo_only_small_026_bmp,
    maxim_logo_only_small_027_bmp, maxim_logo_only_small_028_bmp, maxim_logo_only_small_029_bmp,
    maxim_logo_only_small_030_bmp, maxim_logo_only_small_031_bmp
};

static int g_index = 0;
static volatile int g_animation_status = 0;

/******************************   STATIC FUNCTIONS  **************************/

/******************************   PUBLIC FUNCTIONS  **************************/
void logo_animation_start(void)
{
    g_animation_status = 1;
}

void logo_animation_stop(void)
{
    g_animation_status = 0;
}

void logo_animation_tick(void)
{
    static unsigned int last_time = 0;
    unsigned int crr_time;

    if (g_animation_status) {
        crr_time = utils_get_time_ms();
        if (crr_time >= (last_time + ANIMATION_PERIOD)) {
            // update display
            MXC_TFT_ShowImage(4, 10, logo_maxim_data[g_index]);
            g_index = (g_index + 1) % NB_SPRITE;

            // update last time
            last_time = (crr_time + ANIMATION_PERIOD);
        }
    }
}
