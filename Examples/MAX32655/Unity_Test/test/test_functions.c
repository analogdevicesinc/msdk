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
