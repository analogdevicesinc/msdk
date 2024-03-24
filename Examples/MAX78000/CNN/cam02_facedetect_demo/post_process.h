/******************************************************************************
 *
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// #define RETURN_LARGEST // If defined returns the largest face detected

#define SQUARE(x) ((x) * (x))
#define MULT(x, y) ((x) * (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define NUM_ARS 2
#define NUM_SCALES 2
#define NUM_CLASSES 3 // TODO: CHECK THIS

#define LOC_DIM 4 //(x, y, w, h) or (x1, y1, x2, y2)

#define IMAGE_SIZE_X 168
#define IMAGE_SIZE_Y 224

#define NUM_PRIORS_PER_AR 623
#define NUM_PRIORS NUM_PRIORS_PER_AR *NUM_ARS

#ifdef RETURN_LARGEST
#define MAX_PRIORS 20
#else
#define MAX_PRIORS 1
#endif

#define MIN_CLASS_SCORE 32768 // ~0.5*65536
#define MAX_ALLOWED_OVERLAP 1 //0.3 //170

void get_priors(void);
void nms(void);
void get_cxcy(float *cxcy, int prior_idx);
void gcxgcy_to_cxcy(float *cxcy, int prior_idx, float *priors_cxcy);
void cxcy_to_xy(float *xy, float *cxcy);
void localize_objects(void);
