/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "nms.h"
#include "rtc.h"
#include "tft_utils.h"

#define SQUARE(x) ((x) * (x))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define NUM_ARS 3
#define NUM_SCALES 4
#define NUM_CLASSES 21
#define NUM_OBJ_SCALES 2

#define LOC_DIM 4 //(x, y, w, h) or (x1, y1, x2, y2)

#define NUM_PRIORS_PER_AR_PER_OS 1700 // CHANGE USING dims_x/y!
#define NUM_PRIORS NUM_PRIORS_PER_AR_PER_OS *NUM_ARS *NUM_OBJ_SCALES

#define MAX_PRIORS 50
//#define MIN_CLASS_SCORE 26215 // ~0.4*65536
#define MIN_CLASS_SCORE 120 // ~0.4*256
#define MAX_ALLOWED_OVERLAP 0.3 //170

const int dims_y[NUM_SCALES] = { 32, 16, 8, 4 };
const int dims_x[NUM_SCALES] = { 40, 20, 10, 5 };
const float scales[NUM_SCALES] = { 0.1f, 0.2f, 0.4f, 0.8f };
const float ars[NUM_ARS] = { 0.5f, 1.f, 2.f };
const float obj_scales[NUM_OBJ_SCALES] = { 1.f, 1.4142f };

//Arrays to store model outputs
static int8_t prior_locs[LOC_DIM * NUM_PRIORS]; //(x, y, w, h)

//static uint16_t prior_cls_softmax[NUM_CLASSES * NUM_PRIORS] = {0};
static uint8_t prior_cls_softmax[NUM_CLASSES * NUM_PRIORS] = { 0 };

//NMS related arrays
uint16_t nms_scores[NUM_CLASSES - 1][MAX_PRIORS];
int nms_indices[NUM_CLASSES - 1][MAX_PRIORS];
uint8_t nms_removed[NUM_CLASSES - 1][MAX_PRIORS] = { 0 };
int num_nms_priors[NUM_CLASSES - 1] = { 0 };

// NMS array must be reset in between each inference
void reset_arrays(void)
{
    memset(prior_locs, 0, sizeof(prior_locs));
    memset(prior_cls_softmax, 0, sizeof(prior_cls_softmax));
    memset(num_nms_priors, 0, sizeof(num_nms_priors));
    memset(nms_scores, 0, sizeof(nms_scores));
    memset(nms_indices, 0, sizeof(nms_indices));
    memset(nms_removed, 0, sizeof(nms_removed));
}

uint32_t utils_get_time_ms(void)
{
    uint32_t sec;
    uint32_t subsec;
    uint32_t ms;

    MXC_RTC_GetSubSeconds(&subsec);
    subsec /= 4096;
    MXC_RTC_GetSeconds(&sec);

    ms = (sec * 1000) + (subsec * 1000);

    return ms;
}

static uint8_t signed_to_unsigned(int8_t val)
{
    uint8_t value;
    if (val < 0) {
        value = ~val + 1;
        return (128 - value);
    }
    return val + 128;
}

int get_prior_idx(int os_idx, int ar_idx, int scale_idx, int rel_idx)
{
    int prior_idx = 0;
    for (int s = 0; s < scale_idx; ++s) {
        prior_idx += NUM_ARS * NUM_OBJ_SCALES * dims_x[s] * dims_y[s];
    }

    prior_idx += NUM_ARS * NUM_OBJ_SCALES * rel_idx + NUM_OBJ_SCALES * ar_idx + os_idx;
    return prior_idx;
}

void get_indices(int *os_idx, int *ar_idx, int *scale_idx, int *rel_idx, int prior_idx)
{
    int s;

    int prior_count = 0;
    for (s = 0; s < NUM_SCALES; ++s) {
        prior_count += (NUM_OBJ_SCALES * NUM_ARS * dims_x[s] * dims_y[s]);
        if (prior_idx < prior_count) {
            *scale_idx = s;
            break;
        }
    }

    int in_scale_idx = prior_idx;
    for (s = 0; s < *scale_idx; ++s) {
        in_scale_idx -= (NUM_OBJ_SCALES * NUM_ARS * dims_x[s] * dims_y[s]);
    }

    *rel_idx = in_scale_idx / (NUM_ARS * NUM_OBJ_SCALES);

    int in_pixel_idx = in_scale_idx;
    in_pixel_idx -= (*rel_idx * NUM_OBJ_SCALES * NUM_ARS);

    *ar_idx = in_pixel_idx / NUM_OBJ_SCALES;
    *os_idx = in_pixel_idx % NUM_OBJ_SCALES;
}

int8_t check_for_validity(int32_t *cl_addr)
{
    int ch;
    int8_t validity = 0;
    for (ch = 1; ch < (NUM_CLASSES); ++ch) {
        if (cl_addr[ch] > cl_addr[0]) {
            validity = 1;
            break;
        }
    }

    return validity;
}

void calc_softmax(int32_t *prior_cls_vals, int prior_idx)
{
    int ch;
    double sum = 0.;
    double fp_scale = 16384.; //16384.;//4. * 65536.;//16777216.;//

    for (ch = 0; ch < NUM_CLASSES; ++ch) {
        sum += exp(prior_cls_vals[ch] / fp_scale);
    }

    for (ch = 0; ch < (NUM_CLASSES); ++ch) {
        prior_cls_softmax[prior_idx * NUM_CLASSES + ch] =
            (uint8_t)(256. * exp(prior_cls_vals[ch] / fp_scale) / sum);
    }
}

int8_t *get_next_ch_block(int8_t *mem_addr, int forward)
{
    int8_t *mem_parts_start[4] = { (int8_t *)0x51800000, (int8_t *)0x52800000, (int8_t *)0x53800000,
                                   (int8_t *)0x54800000 };
    int8_t *mem_parts_end[4] = { (int8_t *)0x51880000, (int8_t *)0x52880000, (int8_t *)0x53880000,
                                 (int8_t *)0x54880000 };

    int8_t *res_addr;
    if (!forward) {
        res_addr = mem_addr - 0x20000;
    } else {
        res_addr = mem_addr + 0x20000;
    }

    for (int i = 0; i < 4; ++i) {
        if ((res_addr >= mem_parts_start[i]) && (res_addr < mem_parts_end[i]))
            return res_addr;
    }

    if (!forward) {
        res_addr -= 0xf80000;
    } else {
        res_addr += 0xf80000;
    }

    if (res_addr < mem_parts_start[0] || res_addr >= mem_parts_end[3])
        return (int8_t *)0xdeadbeef;

    return res_addr;
}

void get_prior_cls(void)
{
    int32_t *cl_addr_list[NUM_SCALES] = { (int32_t *)0x51800000, (int32_t *)0x5180A000,
                                          (int32_t *)0x5180C800, (int32_t *)0x5180D200 };
    int num_processors = 64;
    int num_channels = 126;

    int ar_idx, cl_idx, scale_idx, rel_idx, prior_idx, prior_count, os_idx;
    int32_t *pixel_addr, *cl_addr;
    int32_t temp_prior_cls[NUM_CLASSES];

    int next_ch = 0;
    int num_valid_pixels = 0;

    for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
        pixel_addr = cl_addr_list[scale_idx];
        prior_count = dims_x[scale_idx] * dims_y[scale_idx];

        for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
            cl_addr = pixel_addr;

            for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
                for (os_idx = 0; os_idx < NUM_OBJ_SCALES; ++os_idx) {
                    prior_idx = get_prior_idx(os_idx, ar_idx, scale_idx, rel_idx);
                    for (cl_idx = 0; cl_idx < NUM_CLASSES; cl_idx += 1) {
                        memcpy(&temp_prior_cls[cl_idx], cl_addr, 4);

                        next_ch = ar_idx * NUM_OBJ_SCALES * NUM_CLASSES + os_idx * NUM_CLASSES +
                                  cl_idx + 1;
                        cl_addr++;

                        if (next_ch == num_processors) {
                            cl_addr = pixel_addr + 4;
                        } else if (next_ch % 4 == 0 || next_ch == num_channels) {
                            cl_addr = (int32_t *)get_next_ch_block((int8_t *)cl_addr, 1);
                            cl_addr -= 4;
                        }
                    }

                    if (check_for_validity(temp_prior_cls)) {
                        num_valid_pixels++;
                        calc_softmax(temp_prior_cls, prior_idx);
                    }
                }
            }
            //pixel_addr += 4;
            pixel_addr += 8; // 4 ch 2 pass!
        }
    }
}

void get_prior_locs(void)
{
    int8_t *loc_addr = (int8_t *)0x5180D500;

    int ar_idx, scale_idx, rel_idx, prior_idx, prior_count, os_idx;

    for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
        for (os_idx = 0; os_idx < NUM_OBJ_SCALES; ++os_idx) {
            int8_t *loc_addr_temp = loc_addr;

            for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
                prior_count = dims_x[scale_idx] * dims_y[scale_idx];
                for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                    prior_idx = get_prior_idx(os_idx, ar_idx, scale_idx, rel_idx);
                    memcpy(&prior_locs[LOC_DIM * prior_idx], loc_addr_temp, LOC_DIM);
                    loc_addr_temp += LOC_DIM;
                }
            }

            loc_addr = get_next_ch_block(loc_addr, 1);
        }
    }
}

void get_priors(void)
{
    reset_arrays();
    get_prior_locs();
    get_prior_cls();
}

float calculate_IOU(float *box1, float *box2)
{
    float x_left = MAX(box1[0], box2[0]);
    float y_top = MAX(box1[1], box2[1]);
    float x_right = MIN(box1[2], box2[2]);
    float y_bottom = MIN(box1[3], box2[3]);
    float intersection_area;

    if (x_right < x_left || y_bottom < y_top)
        return 0.0;

    intersection_area = (x_right - x_left) * (y_bottom - y_top);

    float box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1]);
    float box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1]);

    float iou = (float)(intersection_area) / (float)(box1_area + box2_area - intersection_area);

    return iou;
}

void get_cxcy(float *cxcy, int prior_idx)
{
    int i, scale_idx, os_idx, ar_idx, rel_idx, cx, cy;

    get_indices(&os_idx, &ar_idx, &scale_idx, &rel_idx, prior_idx);

    cy = rel_idx / dims_x[scale_idx];
    cx = rel_idx % dims_x[scale_idx];
    cxcy[0] = (float)((float)(cx + 0.5) / dims_x[scale_idx]);
    cxcy[1] = (float)((float)(cy + 0.5) / dims_y[scale_idx]);
    cxcy[2] = obj_scales[os_idx] * scales[scale_idx] * sqrt(ars[ar_idx]);
    cxcy[3] = obj_scales[os_idx] * scales[scale_idx] / sqrt(ars[ar_idx]);

    for (i = 0; i < 4; ++i) {
        cxcy[i] = MAX(0.0, cxcy[i]);
        cxcy[i] = MIN(cxcy[i], 1.0);
    }
}

void gcxgcy_to_cxcy(float *cxcy, int prior_idx, float *priors_cxcy)
{
    float gcxgcy[4];
    for (int i = 0; i < 4; i++) {
        gcxgcy[i] = (float)prior_locs[4 * prior_idx + i] / 128.0;
    }

    cxcy[0] = priors_cxcy[0] + gcxgcy[0] * priors_cxcy[2] / 10;
    cxcy[1] = priors_cxcy[1] + gcxgcy[1] * priors_cxcy[3] / 10;
    cxcy[2] = exp(gcxgcy[2] / 5) * priors_cxcy[2];
    cxcy[3] = exp(gcxgcy[3] / 5) * priors_cxcy[3];
}

void cxcy_to_xy(float *xy, float *cxcy)
{
    xy[0] = cxcy[0] - cxcy[2] / 2;
    xy[1] = cxcy[1] - cxcy[3] / 2;
    xy[2] = cxcy[0] + cxcy[2] / 2;
    xy[3] = cxcy[1] + cxcy[3] / 2;
}

void insert_val(uint16_t val, uint16_t *arr, int arr_len, int idx)
{
    if (arr_len < MAX_PRIORS) {
        arr[arr_len] = arr[arr_len - 1];
    }

    for (int j = (arr_len - 1); j > idx; --j) {
        arr[j] = arr[j - 1];
    }
    arr[idx] = val;
}

void insert_idx(int val, int *arr, int arr_len, int idx)
{
    if (arr_len < MAX_PRIORS) {
        arr[arr_len] = arr[arr_len - 1];
    }

    for (int j = (arr_len - 1); j > idx; --j) {
        arr[j] = arr[j - 1];
    }
    arr[idx] = val;
}

void insert_nms_prior(uint16_t val, int idx, uint16_t *val_arr, int *idx_arr, int *arr_len)
{
    if ((*arr_len == 0) || ((val <= val_arr[*arr_len - 1]) && (*arr_len != MAX_PRIORS))) {
        val_arr[*arr_len] = val;
        idx_arr[*arr_len] = idx;
    } else {
        for (int i = 0; i < *arr_len; ++i) {
            if (val > val_arr[i]) {
                insert_val(val, val_arr, *arr_len, i);
                insert_idx(idx, idx_arr, *arr_len, i);
                break;
            }
        }
    }

    *arr_len = MIN((*arr_len + 1), MAX_PRIORS);
}

void nms(void)
{
    int prior_idx, class_idx, nms_idx1, nms_idx2, prior1_idx, prior2_idx;
    //uint16_t cls_prob;
    uint8_t cls_prob;
    float prior_cxcy1[4];
    float prior_cxcy2[4];
    float cxcy1[4];
    float cxcy2[4];
    float xy1[4];
    float xy2[4];

    for (prior_idx = 0; prior_idx < NUM_PRIORS; ++prior_idx) {
        for (class_idx = 0; class_idx < (NUM_CLASSES - 1); ++class_idx) {
            cls_prob = prior_cls_softmax[prior_idx * NUM_CLASSES + class_idx + 1];

            if (cls_prob <= MIN_CLASS_SCORE) {
                continue;
            }

            insert_nms_prior(cls_prob, prior_idx, nms_scores[class_idx], nms_indices[class_idx],
                             &num_nms_priors[class_idx]);
        }
    }

    for (class_idx = 0; class_idx < (NUM_CLASSES - 1); ++class_idx) {
        for (nms_idx1 = 0; nms_idx1 < num_nms_priors[class_idx]; ++nms_idx1) {
            if (nms_removed[class_idx][nms_idx1] != 1 &&
                nms_idx1 != num_nms_priors[class_idx] - 1) {
                for (nms_idx2 = nms_idx1 + 1; nms_idx2 < num_nms_priors[class_idx]; ++nms_idx2) {
                    prior1_idx = nms_indices[class_idx][nms_idx1];
                    prior2_idx = nms_indices[class_idx][nms_idx2];

                    // TODO: Let's implement the box loc finding before this nested loop for 100 priors
                    get_cxcy(prior_cxcy1, prior1_idx);
                    get_cxcy(prior_cxcy2, prior2_idx);

                    gcxgcy_to_cxcy(cxcy1, prior1_idx, prior_cxcy1);
                    gcxgcy_to_cxcy(cxcy2, prior2_idx, prior_cxcy2);

                    cxcy_to_xy(xy1, cxcy1);
                    cxcy_to_xy(xy2, cxcy2);

                    float iou = calculate_IOU(xy1, xy2);

                    if (iou > MAX_ALLOWED_OVERLAP) {
                        nms_removed[class_idx][nms_idx2] = 1;
                    }
                }
            }
        }
    }
}

void nms_localize_objects(void)
{
    get_priors();
    nms();
}

void nms_draw_boxes(void)
{
    float prior_cxcy[4];
    float cxcy[4];
    float xy[4];
    int class_idx, prior_idx, global_prior_idx;

    for (class_idx = 0; class_idx < (NUM_CLASSES - 1); ++class_idx) {
        for (prior_idx = 0; prior_idx < num_nms_priors[class_idx]; ++prior_idx) {
            if (nms_removed[class_idx][prior_idx] != 1) {
                global_prior_idx = nms_indices[class_idx][prior_idx];
                get_cxcy(prior_cxcy, global_prior_idx);
                gcxgcy_to_cxcy(cxcy, global_prior_idx, prior_cxcy);
                cxcy_to_xy(xy, cxcy);
                draw_obj_rect(xy, class_idx + 1);
            }
        }
    }
}
