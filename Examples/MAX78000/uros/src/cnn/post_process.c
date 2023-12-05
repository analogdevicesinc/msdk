#include "post_process.h"

//Arrays to Unload Model Outputs
static int8_t prior_locs[LOC_DIM * NUM_PRIORS]; //(x, y, w, h)
static int8_t prior_cls[NUM_CLASSES * NUM_PRIORS] = { 0 };
static uint8_t prior_cls_softmax[NUM_CLASSES * NUM_PRIORS] = { 0 };

//Arrays used in NMS
static uint8_t nms_scores[NUM_CLASSES - 1][MAX_PRIORS];
static int nms_indices[NUM_CLASSES - 1][MAX_PRIORS];
static uint8_t nms_removed[NUM_CLASSES - 1][MAX_PRIORS] = { 0 };
static int num_nms_priors[NUM_CLASSES - 1] = { 0 };

const int dims[NUM_SCALES][2] = {{30,40}, {15,20}, {7,10}, {3,5}};
const float scales[NUM_SCALES] = {0.15f, 0.35f, 0.55f, 0.75f};
const float ars[NUM_ARS]       = {2.0f, 1.2f, 0.8f, 0.5f};


void nms_memory_init(void)
{
    memset(prior_locs, 0, sizeof(prior_locs));
    memset(prior_cls_softmax, 0, sizeof(prior_cls_softmax));
    memset(num_nms_priors, 0, sizeof(num_nms_priors));
    memset(nms_scores, 0, sizeof(nms_scores));
    memset(nms_indices, 0, sizeof(nms_indices));
    memset(nms_removed, 0, sizeof(nms_removed));
}

void check_for_all_zero_output(void){
    printf("Checking for all zero prior locs\n");
    int8_t* loc_addr = (int8_t*)0x50400000;
    int loc_addr_offset_list[NUM_SCALES] = {0x5400, 0x66C0, 0x6B70, 0x6C88};

    int ar_idx, scale_idx, rel_idx, prior_idx, prior_count, loc_idx, cl_idx;
    int all_zeros = 1;
    int num_nonzeros = 0;

    for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
        
        for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
            int8_t* loc_addr_temp = loc_addr + loc_addr_offset_list[scale_idx];
            prior_count = MULT(dims[scale_idx][0], dims[scale_idx][1]);
        
            for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                for (loc_idx = 0; loc_idx < LOC_DIM; loc_idx++){
                    if (*loc_addr_temp != 0){
                        all_zeros = 0;
                        num_nonzeros += 1;
                    }
                    loc_addr_temp++;
                }
            }
        }
        
        loc_addr += 0x8000;
    }
    if (all_zeros){
        printf("ALL ZERO PRIOR LOCS\n");
    } else {
        printf("Num Nonzero prior locs: %d\n", num_nonzeros);
    }

    all_zeros = 1;
    num_nonzeros = 0;

    int8_t* cl_addr_list[NUM_SCALES] = {(int8_t*)0x50805400, (int8_t*)0x508066C0, (int8_t*)0x50806B70, (int8_t*)0x50806C88};

    for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
        int8_t* cl_addr = cl_addr_list[scale_idx];
        prior_count = MULT(dims[scale_idx][0], dims[scale_idx][1]);

        for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
            int8_t* cl_addr_temp = cl_addr + (ar_idx / 2) * (0x8000) + (ar_idx%2)*2; //AR OFFSET

            for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                prior_idx = get_prior_idx(ar_idx, scale_idx, rel_idx);

                for (cl_idx = 0; cl_idx < NUM_CLASSES; cl_idx += 1) {
                    if (*cl_addr_temp != 0){
                        all_zeros = 0;
                        num_nonzeros += 1;
                    }
                    cl_addr_temp++;
                }

                cl_addr_temp += 2;
            }
        }
    }

    if (all_zeros){
        printf("ALL ZERO PRIOR CLASSES\n");
    } else {
        printf("Num Nonzero prior classes: %d\n", num_nonzeros);
    }


}

void get_priors(void)
{
    nms_memory_init();
    get_prior_locs();
    get_prior_cls();
}

void get_prior_locs(void)
{
    int8_t* loc_addr = (int8_t*)0x50400000;
    int loc_addr_offset_list[NUM_SCALES] = {0x5400, 0x66C0, 0x6B70, 0x6C88};

    int ar_idx, scale_idx, rel_idx, prior_idx, prior_count;

    for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
        
        for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
            int8_t* loc_addr_temp = loc_addr + loc_addr_offset_list[scale_idx];
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
    int8_t* cl_addr_list[NUM_SCALES] = {(int8_t*)0x50805400, (int8_t*)0x508066C0, (int8_t*)0x50806B70, (int8_t*)0x50806C88};

    int ar_idx, cl_idx, scale_idx, rel_idx, prior_idx, prior_count;
    int num_valid_priors = 0;

    for (scale_idx = 0; scale_idx < NUM_SCALES; ++scale_idx) {
        int8_t* cl_addr = cl_addr_list[scale_idx];
        prior_count = MULT(dims[scale_idx][0], dims[scale_idx][1]);

        for (ar_idx = 0; ar_idx < NUM_ARS; ++ar_idx) {
            int8_t* cl_addr_temp = cl_addr + (ar_idx / 2) * (0x8000) + (ar_idx%2)*2; //AR OFFSET

            for (rel_idx = 0; rel_idx < prior_count; ++rel_idx) {
                prior_idx = get_prior_idx(ar_idx, scale_idx, rel_idx);

                for (cl_idx = 0; cl_idx < NUM_CLASSES; cl_idx += 1) {
                    memcpy(&prior_cls[NUM_CLASSES * prior_idx + cl_idx], cl_addr_temp, 1);
                    cl_addr_temp++;
                }
                
                if (check_for_validity(&prior_cls[NUM_CLASSES * prior_idx])) {
                    num_valid_priors++;
                    calc_softmax(&prior_cls[NUM_CLASSES * prior_idx], prior_idx);
                }

                cl_addr_temp += 2;
            }
        }
    }

    printf("Valid Priors: %d\n", num_valid_priors);
}

int get_prior_idx(int ar_idx, int scale_idx, int rel_idx)
{
    int prior_idx = 0;

    for (int s = 0; s < scale_idx; ++s) {
        prior_idx += NUM_ARS * MULT(dims[s][0], dims[s][1]);
    }

    prior_idx += NUM_ARS * rel_idx + ar_idx;
    return prior_idx;
}

void get_indices(int* ar_idx, int* scale_idx, int* rel_idx, int prior_idx)
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

    *ar_idx  = in_scale_idx % NUM_ARS;
    *rel_idx = in_scale_idx / NUM_ARS; 
}


int8_t check_for_validity(int8_t *cl_addr)
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

void calc_softmax(int8_t *prior_cls_vals, int prior_idx)
{
    int ch;
    float sum = 0.;
    float fp_scale = 128.;

    for (ch = 0; ch < NUM_CLASSES; ++ch) {
        sum += expf(((float)prior_cls_vals[ch]) / fp_scale);
    }

    for (ch = 0; ch < (NUM_CLASSES); ++ch) {
        prior_cls_softmax[prior_idx * NUM_CLASSES + ch] =
            (uint8_t)(256. * expf(((float)prior_cls_vals[ch]) / fp_scale) / sum);
    }
}

void nms(void)
{
    int prior_idx, class_idx, nms_idx1, nms_idx2, prior1_idx, prior2_idx;
    //uint16_t cls_prob;
    uint16_t cls_prob;
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

void insert_nms_prior(uint8_t val, int idx, uint8_t *val_arr, int *idx_arr, int *arr_len)
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

void insert_val(uint8_t val, uint8_t *arr, int arr_len, int idx)
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

void get_cxcy(float *cxcy, int prior_idx)
{
    int i, scale_idx, ar_idx, rel_idx, cx, cy;

    get_indices(&ar_idx, &scale_idx, &rel_idx, prior_idx);

    cy = rel_idx / dims[scale_idx][1];
    cx = rel_idx % dims[scale_idx][1];
    cxcy[0] = (float)((float)(cx + 0.5) / dims[scale_idx][1]);
    cxcy[1] = (float)((float)(cy + 0.5) / dims[scale_idx][0]);
    cxcy[2] = scales[scale_idx] * sqrtf(ars[ar_idx]);
    cxcy[3] = scales[scale_idx] / sqrtf(ars[ar_idx]);

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
    cxcy[2] = expf(gcxgcy[2] / 5) * priors_cxcy[2];
    cxcy[3] = expf(gcxgcy[3] / 5) * priors_cxcy[3];
}

void cxcy_to_xy(float *xy, float *cxcy)
{
    xy[0] = cxcy[0] - cxcy[2] / 2;
    xy[1] = cxcy[1] - cxcy[3] / 2;
    xy[2] = cxcy[0] + cxcy[2] / 2;
    xy[3] = cxcy[1] + cxcy[3] / 2;
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

// void draw_obj_rect(float *xy, uint32_t w, uint32_t h, uint8_t scale)
// {
//     int r = 255, g = 0, b = 0;
//     uint32_t color;

//     int x1 = w * xy[0];
//     int y1 = h * xy[1];
//     int x2 = w * xy[2];
//     int y2 = h * xy[3];
//     int x, y;

//     // sanity check
//     if (x1 < 1)
//         x1 = 1;
//     if (y1 < 1)
//         y1 = 1;
//     if (x2 < 1)
//         x2 = 1;
//     if (y2 < 1)
//         y2 = 1;

//     if (x1 > w - 1)
//         x1 = w - 1;
//     if (y1 > h - 1)
//         y1 = h - 1;
//     if (x2 > w - 1)
//         x2 = w - 1;
//     if (y2 > h - 1)
//         y2 = h - 1;

//     color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));

//     for (x = x1; x < x2; ++x) {
//         MXC_TFT_WritePixel(x * scale + TFT_X_OFFSET, y1 * scale, scale, scale, color);
//         MXC_TFT_WritePixel(x * scale + TFT_X_OFFSET, y2 * scale, scale, scale, color);
//     }

//     for (y = y1; y < y2; ++y) {
//         MXC_TFT_WritePixel(x1 * scale + TFT_X_OFFSET, y * scale, scale, scale, color);
//         MXC_TFT_WritePixel(x2 * scale + TFT_X_OFFSET, y * scale, scale, scale, color);
//     }
// }


void print_detected_boxes(float *out_x1, float *out_y1, float *out_x2, float *out_y2){
    float prior_cxcy[4];
    float cxcy[4];
    float xy[4];
    int class_idx, prior_idx, global_prior_idx;

    printf("################\n");
    for (class_idx = 0; class_idx < (NUM_CLASSES - 1); ++class_idx) {
        for (prior_idx = 0; prior_idx < num_nms_priors[class_idx]; ++prior_idx) {
            if (nms_removed[class_idx][prior_idx] != 1) {
                global_prior_idx = nms_indices[class_idx][prior_idx];
                get_cxcy(prior_cxcy, global_prior_idx);
                printf("%f, %f, %f, %f\n", prior_cxcy[0], prior_cxcy[1], prior_cxcy[2], prior_cxcy[3]);
                gcxgcy_to_cxcy(cxcy, global_prior_idx, prior_cxcy);
                cxcy_to_xy(xy, cxcy);
                printf("Prior: %d, Box: [%f, %f, %f, %f]\n", global_prior_idx, xy[0], xy[1], xy[2], xy[3]);
                *out_x1 = xy[0];
                *out_y1 = xy[1];
                *out_x2 = xy[2];
                *out_y2 = xy[3];

                // draw_obj_rect(xy, TFT_W, TFT_H, 1);
            }
        }
    }

    printf("################\n\n");
}