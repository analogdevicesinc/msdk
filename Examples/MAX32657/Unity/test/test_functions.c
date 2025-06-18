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

#include <stdint.h>
#include "unity.h"

extern uint8_t simple_add(uint8_t x, uint8_t y);

void setUp(void) {}
void tearDown(void) {}

void test_simple_add_ok(void)
{
    TEST_ASSERT_EQUAL(7, simple_add(3, 4));
}
void test_simple_add_fail(void)
{
    TEST_ASSERT_EQUAL(-1, simple_add(3, -4));
}
