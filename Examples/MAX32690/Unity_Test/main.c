/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

/**
 * @file    main.c
 * @brief   Simple Unity Testing example
 * @details This example shows how to run a simple test program on a host micro.
 */

/***** Includes *****/
#include "unity.h"

// *****************************************************************************
int main(void)
{
    UnityBegin(__FILE__);

    Unity.NumberOfTests++;
    TEST_ASSERT_EQUAL(1 + 1, 2);

    Unity.NumberOfTests++;
    TEST_ASSERT_NOT_EQUAL(1 + 1, 2);

    return (UnityEnd());
}
