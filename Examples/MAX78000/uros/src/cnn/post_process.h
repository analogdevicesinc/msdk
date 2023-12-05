#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#define MULT(x, y) ((x) * (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define NUM_ARS     4
#define NUM_SCALES  4
#define NUM_CLASSES 2

#define LOC_DIM 4
#define NUM_PRIORS_PER_AR 1585
#define NUM_PRIORS        NUM_PRIORS_PER_AR* NUM_ARS

#define MAX_PRIORS  20

#define MIN_CLASS_SCORE     128 // ~0.5*256
#define MAX_ALLOWED_OVERLAP 0.1f

#define TFT_W 320
#define TFT_H 240
#define TFT_X_OFFSET 0

void get_priors(void);
void get_prior_locs(void);
void get_prior_cls(void);
int get_prior_idx(int ar_idx, int scale_idx, int rel_idx);
void get_indices(int* ar_idx, int* scale_idx, int* rel_idx, int prior_idx);
void calc_softmax(int8_t *prior_cls_vals, int prior_idx);
int8_t check_for_validity(int8_t *cl_addr);
void check_for_all_zero_output(void);

void nms_memory_init(void);
void nms(void);
void insert_nms_prior(uint8_t val, int idx, uint8_t *val_arr, int *idx_arr, int *arr_len);
void insert_val(uint8_t val, uint8_t *arr, int arr_len, int idx);
void insert_idx(int val, int *arr, int arr_len, int idx);
float calculate_IOU(float *box1, float *box2);

void get_cxcy(float *cxcy, int prior_idx);
void gcxgcy_to_cxcy(float *cxcy, int prior_idx, float *priors_cxcy);
void cxcy_to_xy(float *xy, float *cxcy);

void print_detected_boxes(float* out_x1, float* out_y1, float* out_x2, float* out_y2);
void draw_obj_rect(float *xy, uint32_t w, uint32_t h, uint8_t scale);