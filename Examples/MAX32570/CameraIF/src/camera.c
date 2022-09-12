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
#include <stdlib.h>
#include <string.h>
#include "camera.h"
#include "sccb.h"

#include "mxc_device.h"
#include "board.h"
#include "nvic_table.h"
#include "pt.h"
#include "cameraif.h"
#include "utils.h"
#include "dma.h"
#include "uart.h"
#include "mxc_delay.h"
#include "gpio.h"

/*******************************      DEFINES      ***************************/
#define USE_DMA 0 // set it for dma read mode

#define FIFO_THRES_HOLD 4
#define CAMERAIF_DATA_BUS_WITH MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT

/******************************** Static Functions ***************************/
static unsigned int g_framesize = (unsigned int)FRAMESIZE_VGA;
static pixformat_t g_pixelformat = PIXFORMAT_YUV422;

static uint8_t rx_data[512 * 384 + 2]; // +2 for manage overflow
static volatile uint32_t rx_data_index = 0;
static volatile uint32_t g_is_img_rcv = 0;
static int g_total_img_size = 0;

static camera_t camera;
extern int sensor_register(camera_t *camera);

const int resolution[][2] = {
    /* Special resolutions */
    { 512, 384 }, /* SP       */
    // C/SIF Resolutions
    { 352, 288 }, /* CIF       */
    // VGA Resolutions
    { 320, 240 }, /* QVGA      */
    { 640, 480 }, /* VGA       */
};

//----------------------------------------
// CAMERAIF ISR
//----------------------------------------
void camera_irq_handler(void)
{
#if !USE_DMA

    // check fifo threshold flag
    if (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_THRESH) {
        /*
         * Read sequentially do not use loop
         * because it may cause same delay and you may miss some bytes
         */
        unsigned int data;

        if ((rx_data_index + 8) <= g_total_img_size) {
            // 1
            data = MXC_PCIF->fifo_data;
            rx_data[rx_data_index++] = data;
            rx_data[rx_data_index++] = data >> 16;

            // 2
            data = MXC_PCIF->fifo_data;
            rx_data[rx_data_index++] = data;
            rx_data[rx_data_index++] = data >> 16;

            // 3
            data = MXC_PCIF->fifo_data;
            rx_data[rx_data_index++] = data;
            rx_data[rx_data_index++] = data >> 16;

            // 4
            data = MXC_PCIF->fifo_data;
            rx_data[rx_data_index++] = data;
            rx_data[rx_data_index++] = data >> 16;
        }
    }

#endif

    // check img_done flag
    if (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_IMG_DONE) {
        g_is_img_rcv = 1;
#if USE_DMA

        if (g_pixelformat == PIXFORMAT_GRAYSCALE) {
            int i;
            rx_data_index = g_total_img_size / 2;

            for (i = 0; i < rx_data_index; i++) { rx_data[i] = rx_data[i * 2]; }
        } else {
            rx_data_index = g_total_img_size;
        }

#endif
    }

    // clear flags, tmp flag is used to pass coverity check
    unsigned int flags = MXC_PCIF->int_fl;
    MXC_PCIF->int_fl = flags; // clear flags
}

#if USE_DMA
static void setup_dma(void)
{
    int dma_handle = 0;

    MXC_DMA->cn = 0x01;

    MXC_DMA->ch[dma_handle].dst = (uint32_t)rx_data; // Cast Pointer

    if (CAMERAIF_DATA_BUS_WITH == MXC_V_CAMERAIF_CTRL_DATA_WIDTH_8BIT) {
        MXC_DMA->ch[dma_handle].cnt = g_total_img_size;
    } else {
        MXC_DMA->ch[dma_handle].cnt =
            g_total_img_size * 2; // 10 and 12 bit use 2 bytes per word in the fifo
    }

    MXC_DMA->ch[dma_handle].cfg =
        ((0x1 << MXC_F_DMA_CFG_CTZIEN_POS) + (0x0 << MXC_F_DMA_CFG_CHDIEN_POS) +
         (0x3 << MXC_F_DMA_CFG_BRST_POS) + (0x1 << MXC_F_DMA_CFG_DSTINC_POS) +
         (0x2 << MXC_F_DMA_CFG_DSTWD_POS) + (0x0 << MXC_F_DMA_CFG_SRCINC_POS) +
         (0x2 << MXC_F_DMA_CFG_SRCWD_POS) + (0x0 << MXC_F_DMA_CFG_PSSEL_POS) +
         (0x0 << MXC_F_DMA_CFG_PSSEL_POS) + (0x0 << MXC_F_DMA_CFG_REQWAIT_POS) +
         (0xD << MXC_F_DMA_CFG_REQSEL_POS) + (0x0 << MXC_F_DMA_CFG_PRI_POS) +
         (0x0 << MXC_F_DMA_CFG_RLDEN_POS) + (0x1 << MXC_F_DMA_CFG_CHEN_POS));
}
#endif

/******************************** Public Functions ***************************/
int camera_init(void)
{
    int ret = 0;
    unsigned int bpp; // bytes per pixel
    unsigned int framesize;

    // initialize XCLK for camera
    MXC_PT_Init(MXC_PT_CLK_DIV1);
    MXC_PT_SqrWaveConfig(1, CAMERA_FREQ);
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT1);
    MXC_GPIO_SetVSSEL(gpio_cfg_pt1.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_pt1.mask);
    utils_delay_ms(100);

    // initialize serial camera communication bus
    sccb_init();
    // register functions
    sensor_register(&camera);

    ret = camera.init();

    if (ret == 0) {
        ret |= camera.reset();
        ret |= camera.set_pixformat(g_pixelformat);
        ret |= camera.set_framesize((framesize_t)g_framesize);
        ret |= camera.set_contrast(0);
        ret |= camera.set_hmirror(1);
        ret |= camera.set_vflip(1);

        // calculate frame resolution area
        framesize = resolution[g_framesize][0] * resolution[g_framesize][1];
    }

#if USE_DMA
    bpp = 2; // 2 bytes per pixel
#else

    if (g_pixelformat == PIXFORMAT_GRAYSCALE) {
        bpp = 1; // 1 bytes per pixel
    } else {
        bpp = 2; // 2 bytes per pixel
    }

#endif

    if (ret == 0) {
        g_total_img_size = framesize * bpp;
        /*rx_data = (uint8_t *)malloc( ((g_total_img_size+3)/4) * 4); //
        if (rx_data == NULL) {
            return -1;
        }
        */

        MXC_PCIF_Init();
        MXC_PCIF_SetDatawidth(MXC_PCIF_DATAWIDTH_8_BIT);
        MXC_PCIF_SetTimingSel(MXC_PCIF_TIMINGSEL_HSYNC_and_VSYNC);
        MXC_PCIF_SetThreshold(FIFO_THRES_HOLD);

        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_PCIF_SYS, MXC_F_CAMERAIF_CTRL_PCIF_SYS);

#if USE_DMA
        MXC_DMA_Init();

        setup_dma();

        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH,
                     (0x1 << MXC_F_CAMERAIF_CTRL_RX_DMA_THRSH_POS));
        MXC_SETFIELD(MXC_PCIF->ctrl, MXC_F_CAMERAIF_CTRL_RX_DMA, MXC_F_CAMERAIF_CTRL_RX_DMA);
#endif

        MXC_PCIF_EnableInt(MXC_F_CAMERAIF_INT_EN_IMG_DONE);
#if !USE_DMA
        MXC_PCIF_EnableInt(MXC_F_CAMERAIF_INT_EN_FIFO_THRESH);
#endif

        MXC_NVIC_SetVector(CameraIF_IRQn, camera_irq_handler);
    }

    return ret;
}

int camera_reset(void)
{
    return camera.reset();
}

int camera_get_id(void)
{
    return camera.get_id();
}

int camera_dump_registers(void)
{
    return camera.dump_registers();
}

int camera_start_campture_image(void)
{
    int ret = 0;

#if USE_DMA
    setup_dma();
#endif
    // clear flag
    g_is_img_rcv = 0;
    rx_data_index = 0;
    MXC_PCIF_Start(MXC_PCIF_READMODE_SINGLE_MODE);

    return ret;
}

int camera_is_image_rcv(void)
{
    return (g_is_img_rcv) ? 1 : 0;
}

uint8_t *camera_get_pixel_format(void)
{
    if (g_pixelformat == PIXFORMAT_GRAYSCALE) {
        return (uint8_t *)"GRAYSCALE";
    } else if (g_pixelformat == PIXFORMAT_RGB565) {
        return (uint8_t *)"RGB565";
    } else {
        return (uint8_t *)"YUV422";
    }
}

void camera_get_image(uint8_t **img, uint32_t *imgLen, uint32_t *w, uint32_t *h)
{
    *img = (uint8_t *)rx_data;
    *imgLen = rx_data_index;

    *w = resolution[g_framesize][0];
    *h = resolution[g_framesize][1];
}
