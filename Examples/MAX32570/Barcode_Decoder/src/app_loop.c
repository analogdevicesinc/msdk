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

#include "camera.h"
#include "utils.h"
// Barcode Decoder include
#include "zbar.h"

static zbar_image_scanner_t* image_scanner = NULL;

static int is_barcode_find = 0;

/**************************** Static Functions *******************************/
static void print_exec_time(const char* msg, unsigned int t_end, unsigned int t_start)
{
    printf("%s", msg);

    if (t_end >= t_start) {
        t_end -= t_start;
        printf(" %d sec %d ms\n", (t_end / 1000), (t_end % 1000));
    } else {
        printf("\n");
    }
}

static void process_img(void)
{
    uint8_t* raw;
    uint32_t imgLen;
    uint32_t w, h;
    zbar_image_t* image;
    unsigned int start_time;
    unsigned int end_time;

    is_barcode_find = 0;

    camera_get_image(&raw, &imgLen, &w, &h);

#ifdef BRCD_RDR_IMG_TO_PC
    printf("Sending image to pc...\n");
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
    printf("Send end\n");
#endif

    start_time = utils_get_time_ms();

    /* wrap image data */
    image = zbar_image_create();
    zbar_image_set_format(image, *(int*)"Y800");
    zbar_image_set_size(image, w, h);
    zbar_image_set_data(image, raw, w * h, zbar_image_free_data);

    /* scan the image for barcodes */
    zbar_scan_image(image_scanner, image);

    /* extract results */
    const zbar_symbol_t* symbol = zbar_image_first_symbol(image);

    for (; symbol; symbol = zbar_symbol_next(symbol)) {
        /* do something useful with results */
        zbar_symbol_type_t typ = zbar_symbol_get_type(symbol);
        const char* data       = zbar_symbol_get_data(symbol);
        printf("decoded %s symbol \"%s\"\n", zbar_get_symbol_name(typ), data);
        is_barcode_find = 1;
    }

    /* clean up */
    zbar_image_destroy(image);

    if (is_barcode_find) {
        end_time = utils_get_time_ms();
        print_exec_time("Image Process Time", end_time, start_time);
    }
}

/*****************************************************************************/
int app_loop_init(void)
{
    int ret = 0;

    camera_init();
    camera_dump_registers();

    /* create a reader */
    image_scanner = zbar_image_scanner_create();
    /* configure the reader */
    zbar_image_scanner_set_config(image_scanner, ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    //..

    return ret;
}

void app_loop_endless(void)
{
    unsigned int t_start_img;
    unsigned int t_end_img;

    t_start_img = utils_get_time_ms();

    // start to capture
    camera_start_campture_image();

    while (1) {
        if (camera_is_image_rcv()) {
            t_end_img = utils_get_time_ms();

            // process images, search barcode inside it
            process_img();

            if (is_barcode_find) {
                print_exec_time("Image Read Time", t_end_img, t_start_img);
                printf("------------------------------------------------------\n");
            }

            t_start_img = utils_get_time_ms();

            // restart to capture
            camera_start_campture_image();
        }
    }

    //zbar_image_scanner_destroy(image_scanner);
}
