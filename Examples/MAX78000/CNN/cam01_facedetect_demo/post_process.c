/*******************************************************************************
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
 *******************************************************************************/
#include "post_process.h"
// #include "facedetection.h"
#include "mxc_device.h"
#include "mxc.h"

#define S_MODULE_NAME "post_proc"

uint8_t box[4]; // x1, y1, x2, y2

const int dims[NUM_SCALES][2] = { { 28, 21 }, { 7, 5 } };
// NUM_PRIORS_PER_AR = SQUARE(dims[0]) + SQUARE(dims[1]) + SQUARE(dims[2]) + SQUARE(dims[3])
const float scales[NUM_SCALES] = { 0.35715f, 0.7143f };
const float ars[NUM_ARS] = { 0.9f, 0.75f };

//Arrays to store model outputs
static int8_t prior_locs[LOC_DIM * NUM_PRIORS]; //(x, y, w, h)
static int8_t prior_cls[NUM_CLASSES * NUM_PRIORS];
static uint16_t prior_cls_softmax[NUM_CLASSES * NUM_PRIORS] = { 0 };

//NMS related arrays
static uint16_t nms_scores[NUM_CLASSES - 2][MAX_PRIORS];
static uint16_t nms_indices[NUM_CLASSES - 2][MAX_PRIORS];
static uint8_t nms_removed[NUM_CLASSES - 2][MAX_PRIORS] = { 0 };
static int num_nms_priors[NUM_CLASSES - 2] = { 0 };
volatile uint8_t face_detected = 0;

int get_prior_idx(int ar_idx, int scale_idx, int rel_idx)
{
    int prior_idx = 0;

    for (int s = 0; s < scale_idx; ++s) {
        prior_idx += NUM_ARS * MULT(dims[s][0], dims[s][1]);
    }

    prior_idx += NUM_ARS * rel_idx + ar_idx;
    return prior_idx;
}

void get_indices(int *ar_idx, int *scale_idx, int *rel_idx, int prior_idx)
{
    int s;

    int prior_count = 0;

    for (s = 0; s < NUM_SCALES; ++s) {
        prior_count += (NUM_ARS * MULT(dims[s][0], dims[s][1]));

        if (prior_idx < prior_count) {
            *scale_idx = s;
            break;
        }
    }

    int in_scale_idx = prior_idx;

    for (s = 0; s < *scale_idx; ++s) {
        in_scale_idx -= (NUM_ARS * MULT(dims[s][0], dims[s][1]));
    }

    *ar_idx = in_scale_idx % NUM_ARS;
    *rel_idx = in_scale_idx / NUM_ARS;
}

void softmax(void)
{
    int i, ch, calc_softmax;
    float sum;

    memset(prior_cls_softmax, 0, sizeof(prior_cls_softmax));
    for (i = 0; i < NUM_PRIORS; ++i) {
        sum = 0.;
        calc_softmax = 0;

        for (ch = 1; ch < (NUM_CLASSES - 1); ++ch) {
            if (prior_cls[i * NUM_CLASSES + ch] >= prior_cls[i * NUM_CLASSES]) {
                calc_softmax = 1;
                break;
            }
        }

        if (calc_softmax == 0) {
            continue;
        }

        for (ch = 0; ch < (NUM_CLASSES); ++ch) {
            sum += exp(prior_cls[i * NUM_CLASSES + ch] / 128.);
        }

        for (ch = 0; ch < (NUM_CLASSES); ++ch) {
            prior_cls_softmax[i * NUM_CLASSES + ch] =
                (uint16_t)(65536. * exp(prior_cls[i * NUM_CLASSES + ch] / 128.) / sum);
        }
    }
}

void get_prior_locs(void)
{
    int8_t *loc_addr = (int8_t *)0x50400000;

    int ar_idx, scale_idx, rel_idx, prior_idx, prior_count;

    for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
        int8_t *loc_addr_temp = loc_addr;

        for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
            prior_count = MULT(dims[scale_idx][0], dims[scale_idx][1]);

            for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                prior_idx = get_prior_idx(ar_idx, scale_idx, rel_idx);
                memcpy(&prior_locs[LOC_DIM * prior_idx], loc_addr_temp, LOC_DIM);
                loc_addr_temp += LOC_DIM;
            }
        }

        loc_addr += 0x8000;
    }
}

void get_prior_cls(void)
{
    int8_t *cl_addr = (int8_t *)0x50410000;

    int ar_idx, cl_idx, scale_idx, rel_idx, prior_idx, prior_count;

    for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
        prior_count = MULT(dims[scale_idx][0], dims[scale_idx][1]);

        for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
            int8_t *cl_addr_temp = cl_addr + ar_idx * 2; //AR OFFSET

            for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                for (cl_idx = 0; cl_idx < NUM_CLASSES - 1; cl_idx += 1) {
                    cl_addr_temp += cl_idx;
                    prior_idx = get_prior_idx(ar_idx, scale_idx, rel_idx);
                    memcpy(&prior_cls[NUM_CLASSES * prior_idx + cl_idx], cl_addr_temp, 1);
                }

                cl_addr_temp += 3;
            }
        }

        cl_addr = (int8_t *)0x50410930;
    }

    softmax();
}

void get_priors(void)
{
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

    if (x_right < x_left || y_bottom < y_top) {
        return 0.0;
    }

    intersection_area = (x_right - x_left) * (y_bottom - y_top);

    float box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1]);
    float box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1]);

    float iou = (float)(intersection_area) / (float)(box1_area + box2_area - intersection_area);

    return iou;
}

void get_cxcy(float *cxcy, int prior_idx)
{
    int i, scale_idx, ar_idx, rel_idx, cx, cy;

    get_indices(&ar_idx, &scale_idx, &rel_idx, prior_idx);
    cy = rel_idx / dims[scale_idx][1];
    cx = rel_idx % dims[scale_idx][1];
    cxcy[0] = (float)((float)(cx + 0.5) / dims[scale_idx][1]);
    cxcy[1] = (float)((float)(cy + 0.5) / dims[scale_idx][0]);
    cxcy[2] = scales[scale_idx] * sqrt(ars[ar_idx]);
    cxcy[3] = scales[scale_idx] / sqrt(ars[ar_idx]);

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

void insert_idx(uint16_t val, uint16_t *arr, int arr_len, int idx)
{
    if (arr_len < MAX_PRIORS) {
        arr[arr_len] = arr[arr_len - 1];
    }

    for (int j = (arr_len - 1); j > idx; --j) {
        arr[j] = arr[j - 1];
    }

    arr[idx] = val;
}

void insert_nms_prior(uint16_t val, int idx, uint16_t *val_arr, uint16_t *idx_arr, int *arr_len)
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

void reset_nms(void)
{
    for (int cl = 0; cl < NUM_CLASSES - 2; ++cl) {
        num_nms_priors[cl] = 0;

        for (int p_idx = 0; p_idx < MAX_PRIORS; ++p_idx) {
            nms_scores[cl][p_idx] = 0;
            nms_indices[cl][p_idx] = 0;
            nms_removed[cl][p_idx] = 0;
        }
    }
}

void nms(void)
{
    int prior_idx, class_idx, nms_idx1, nms_idx2, prior1_idx, prior2_idx;
    uint16_t cls_prob;
    float prior_cxcy1[4];
    float prior_cxcy2[4];
    float cxcy1[4];
    float cxcy2[4];
    float xy1[4];
    float xy2[4];

    reset_nms();

    for (prior_idx = 0; prior_idx < NUM_PRIORS; ++prior_idx) {
        for (class_idx = 0; class_idx < (NUM_CLASSES - 2); ++class_idx) {
            cls_prob = prior_cls_softmax[prior_idx * NUM_CLASSES + class_idx + 1];

            if (cls_prob < MIN_CLASS_SCORE) {
                continue;
            }

            //num_nms_priors[class_idx] = insert_nms_prior(cls_prob, prior_idx, nms_scores[class_idx], nms_indices[class_idx], num_nms_priors[class_idx]);
            insert_nms_prior(cls_prob, prior_idx, nms_scores[class_idx], nms_indices[class_idx],
                             &num_nms_priors[class_idx]);
        }
    }

    for (class_idx = 0; class_idx < (NUM_CLASSES - 2); ++class_idx) {
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

#ifdef RETURN_LARGEST
float calculate_area(float *xy)
{
    float area;
    area = (xy[3] - xy[1]) * (xy[2] - xy[0]);
    return area;
}
#endif

void box_sanity_check(float *xy)
{
    int error = 0;

    if (xy[0] < 0) {
        xy[0] = 0;
        error++;
    } else if (xy[0] > 1.0) {
        xy[0] = 1.0;
        error++;
    }

    if (xy[1] < 0) {
        xy[1] = 0;
        error++;
    } else if (xy[1] > 1.0) {
        xy[1] = 1.0;
        error++;
    }

    if (xy[2] < 0) {
        xy[2] = 0;
        error++;
    } else if (xy[2] > 1.0) {
        xy[2] = 1.0;
        error++;
    }

    if (xy[3] < 0) {
        xy[3] = 0;
        error++;
    } else if (xy[3] > 1.0) {
        xy[3] = 1.0;
        error++;
    }

    if (error)
        printf("Corrected: %d\n", error);
}

void localize_objects(void)
{
    float prior_cxcy[4];
    float cxcy[4];
    float xy[4];
    int class_idx, prior_idx, global_prior_idx;
#ifdef RETURN_LARGEST
    float area;
    float max_area = 0;
    float max_xy[4];
#endif

    nms();

    for (class_idx = 0; class_idx < (NUM_CLASSES - 2); ++class_idx) {
        for (prior_idx = 0; prior_idx < num_nms_priors[class_idx]; ++prior_idx) {
            if (nms_removed[class_idx][prior_idx] != 1) {
                global_prior_idx = nms_indices[class_idx][prior_idx];
                get_cxcy(prior_cxcy, global_prior_idx);
                gcxgcy_to_cxcy(cxcy, global_prior_idx, prior_cxcy);
                cxcy_to_xy(xy, cxcy);

#ifdef RETURN_LARGEST
                area = calculate_area(xy);
                if (area > max_area) {
                    max_area = area;
                    for (int i = 0; i < 4; ++i) {
                        max_xy[i] = xy[i];
                    }
                }
#else

                box_sanity_check(&xy[0]);

                box[0] = (uint8_t)(IMAGE_SIZE_X * xy[0]);
                box[1] = (uint8_t)(IMAGE_SIZE_Y * xy[1]);
                box[2] = (uint8_t)(IMAGE_SIZE_X * xy[2]);
                box[3] = (uint8_t)(IMAGE_SIZE_Y * xy[3]);

#if 0
			    printf("class: %d, prior_idx: %d, prior: %d, x1: %.2f, y1: %.2f, x2: %.2f, y2: "
                       "%.2f \n",
                       class_idx + 1, prior_idx, global_prior_idx, xy[0], xy[1], xy[2], xy[3]);
#else
                printf("x1:%d y1:%d x2:%d y2:%d\n", box[0], box[1], box[2], box[3]);
                printf("width:%d heigth:%d\n", box[2] - box[0], box[3] - box[1]);
#endif
                face_detected = 1;
                // draw_obj_rect(xy, IMAGE_SIZE_X, IMAGE_SIZE_Y);
#endif
            }
        }
    }

#ifdef RETURN_LARGEST
    if (max_area != 0) {
        class_idx = 0;

        printf("class: %d, prior_idx: %d, prior: %d, x1: %.2f, y1: %.2f, x2: %.2f, y2: "
               "%.2f \n",
               class_idx + 1, prior_idx, global_prior_idx, max_xy[0], max_xy[1], max_xy[2],
               max_xy[3]);

        box[0] = (uint8_t)(IMAGE_SIZE_X * max_xy[0]);
        box[1] = (uint8_t)(IMAGE_SIZE_Y * max_xy[1]);
        box[2] = (uint8_t)(IMAGE_SIZE_X * max_xy[2]);
        box[3] = (uint8_t)(IMAGE_SIZE_Y * max_xy[3]);

        face_detected = 1;
        draw_obj_rect(max_xy, IMAGE_SIZE_X, IMAGE_SIZE_Y);
    }
#endif
}
