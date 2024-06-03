/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

/**
 * Note: Increment counter in Secure context.
 * 
 * Warning: Prevent leaks by checking pointers passed into
 *  Secure functions before dereferencing them, or the Non-Secure
 *  world can access all of Secure memory. Secure code must also
 *  handle non-secure memory as volatile.
 * 
 * Param:   *count    Pointer of counter value to increment.
 * Return:  Error code.
 */
// extern int IncrementCount_S(volatile int *count_ns);
