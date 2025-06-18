/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

void setUp(void) {

}

void tearDown(void) {

}

extern uint8_t simple_add(uint8_t x, uint8_t y);

void test_simple_code_addition_pass(void) {
    // Simple addition case
    TEST_ASSERT_EQUAL_UINT8(simple_add(1, 1), 2);
}

void test_simple_code_signed_params_fail(void) {
    // Signed -> unsigned types case
    TEST_ASSERT_EQUAL_UINT8(-2, simple_add(-1, -1));
}

// *****************************************************************************
int main(void)
{
    // Clear screen
    printf("\x1B[2J");
    printf("\n***** Unity Test Framework Example *****\n\n");

    UnityBegin("simple_code.c");

    RUN_TEST(test_simple_code_addition_pass, __LINE__);

    RUN_TEST(test_simple_code_signed_params_fail, __LINE__);

    return (UnityEnd());
}

