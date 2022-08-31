/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
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
 *******************************************************************************
 *
 *	@file		test_main.c
 *	@author		Benjamin VINOT - <benjamin.vinot@maximintegrated.com>
 *   @date		Aug 9, 2018
 *
 */

#include "CUnit/Basic.h"
#include <ecdsa.h>
#include <process.h>
#include <session_build.h>
#include <stdio.h>
#include <string.h>

/* Pointer to the file used by the tests. */
static FILE* temp_file = NULL;

/* The suite initialization function.
 * Opens the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_parsing(void)
{
    if (NULL == (temp_file = fopen("test/session_build.ini", "w+"))) {
        return -1;
    } else {
        return 0;
    }
}

/* The suite cleanup function.
 * Closes the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_parsing(void)
{
    if (0 != fclose(temp_file)) {
        return -1;
    } else {
        temp_file = NULL;
        return 0;
    }
}

/* Simple test of fprintf().
 * Writes test data to the temporary file and checks
 * whether the expected number of bytes were written.
 */
void test_LongHex(void)
{
    unsigned int ptr;
    unsigned int result;

    result = parse_longhex(&ptr, "15745420");
    CU_ASSERT_EQUAL(ptr, 0x15745420);
    CU_ASSERT_EQUAL(result, ERR_OK);
    result = parse_longhex(&ptr, "0x15745420");
    CU_ASSERT_EQUAL(ptr, 0x15745420);
    CU_ASSERT_EQUAL(result, ERR_OK);
    result = parse_longhex(&ptr, "sdfsdfsdfs");
    CU_ASSERT_EQUAL(result, ERR_INVALID_OPTION_FORMAT);
    result = parse_longhex(&ptr, "0x1574542");
    CU_ASSERT_EQUAL(result, ERR_INVALID_OPTION_FORMAT);
    result = parse_longhex(&ptr, "0x1574q542");
    CU_ASSERT_EQUAL(result, ERR_INVALID_OPTION_FORMAT);
    result = parse_longhex(&ptr, "0x");
    CU_ASSERT_EQUAL(result, ERR_INVALID_OPTION_FORMAT);
    result = parse_longhex(&ptr, "0x157405424");
    CU_ASSERT_EQUAL(result, ERR_INVALID_OPTION_FORMAT);
}

void test_Hex(void)
{
    unsigned int ptr;

    parse_hex(&ptr, "15");
    CU_ASSERT_EQUAL(ptr, 0x15);
}

/* The main() function for setting up and running the tests.
 * Returns a CUE_SUCCESS on successful running, another
 * CUnit error code on failure.
 */
int TS_Parsing(void)
{
    CU_pSuite pSuite = NULL;

    /* add a suite to the registry */
    pSuite = CU_add_suite("Parsing", init_suite_parsing, clean_suite_parsing);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }

    /* add the tests to the suite */
    /* NOTE - ORDER IS IMPORTANT - MUST TEST fread() AFTER fprintf() */
    if ((NULL == CU_add_test(pSuite, "test of LongHex", test_LongHex))
        || (NULL == CU_add_test(pSuite, "test of Hex", test_Hex)))

    {
        CU_cleanup_registry();
        return CU_get_error();
    }

    return ERR_OK;
}
