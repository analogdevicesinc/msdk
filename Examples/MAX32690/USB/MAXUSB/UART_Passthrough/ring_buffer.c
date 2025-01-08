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
* @file ring_buffer.c
* @brief 
*/

/**************************************************************************************************
Includes
**************************************************************************************************/
#include "ring_buffer.h"
#include "mxc_device.h"

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

/**
 * Initializes the ring buffer.
 */
void ring_buffer_init(RingBuffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

/**
 * Adds a byte to the ring buffer.
 * 
 * @param rb Pointer to the ring buffer.
 * @param data The byte to add.
 * @return True if successful, false if the buffer is full.
 */
bool ring_buffer_put(RingBuffer *rb, uint8_t data)
{
    __disable_irq();
    size_t next_head = (rb->head + 1) & (RING_BUFFER_SIZE - 1);

    // Check if buffer is full
    if (next_head == rb->tail) {
        __enable_irq();
        return false; // Buffer is full
    }

    rb->buffer[rb->head] = data;
    rb->head = next_head;
    rb->cnt++;
    __enable_irq();

    return true;
}

/**
 * Retrieves a byte from the ring buffer.
 * 
 * @param rb Pointer to the ring buffer.
 * @param data Pointer to store the retrieved byte.
 * @return True if successful, false if the buffer is empty.
 */
bool ring_buffer_get(RingBuffer *rb, uint8_t *data)
{
    __disable_irq();
    if (rb->head == rb->tail) {
        return false; // Buffer is empty
        __enable_irq();
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & (RING_BUFFER_SIZE - 1);
    rb->cnt--;

    __enable_irq();

    return true;
}

/**
 * Checks if the ring buffer is empty.
 */
bool ring_buffer_is_empty(const RingBuffer *rb)
{
    __disable_irq();
    bool empty = rb->head == rb->tail;

    __enable_irq();
    return empty;
}

/**
 * Checks if the ring buffer is full.
 */
bool ring_buffer_is_full(const RingBuffer *rb)
{
    __disable_irq();
    bool full = ((rb->head + 1) & (RING_BUFFER_SIZE - 1)) == rb->tail;
    __enable_irq();

    return full;
}
uint32_t ring_buffer_get_data_cnt(const RingBuffer *rb)
{
    return rb->cnt;
}

/*************************************************************************************************/
