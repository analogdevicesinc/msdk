/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential to Analog Devices, Inc. and
 * its licensors.
 *
 ******************************************************************************/
/**
* @file ring_buffer.h
* @brief 
*/

/**************************************************************************************************
Includes
**************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define RING_BUFFER_SIZE 256 // Must be a power of 2 for efficiency

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile size_t head; // Index for the producer
    volatile size_t tail; // Index for the consumer
    uint32_t cnt;
} RingBuffer;

/**************************************************************************************************
Macros
**************************************************************************************************/

/**************************************************************************************************
Constants
**************************************************************************************************/

/**************************************************************************************************
Global Variables
**************************************************************************************************/

/**************************************************************************************************
Functions
**************************************************************************************************/
void ring_buffer_init(RingBuffer *rb);
bool ring_buffer_put(RingBuffer *rb, uint8_t data);
bool ring_buffer_get(RingBuffer *rb, uint8_t *data);
bool ring_buffer_is_empty(const RingBuffer *rb);
bool ring_buffer_is_full(const RingBuffer *rb);
uint32_t ring_buffer_get_data_cnt(const RingBuffer *rb);

/*************************************************************************************************/
