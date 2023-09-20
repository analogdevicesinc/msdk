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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

//#define RETURN_LARGEST // If defined returns the largest face detected

#define SQUARE(x) ((x) * (x))
#define MULT(x, y) ((x) * (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#define NUM_ARS 2
#define NUM_SCALES 2
#define NUM_CLASSES 3 // TODO: CHECK THIS

#define LOC_DIM 4 //(x, y, w, h) or (x1, y1, x2, y2)

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
