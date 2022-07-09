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
*	@file		test_ecdsa.c
*	@author		Benjamin VINOT - <benjamin.vinot@maximintegrated.com>
*   @date		Sept 3, 2018
*
*/

#include <stdio.h>
#include <string.h>
#include "CUnit/Basic.h"
#include <process.h>
#include <session_build.h>
#include <ecdsa.h>

/* Pointer to the file used by the tests. */
static FILE* temp_file = NULL;

/* The suite initialization function.
 * Opens the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int init_suite1(void)
{
   if (NULL == (temp_file = fopen("test/session_build.ini", "w+"))) {
      return -1;
   }
   else {
      return 0;
   }
}

void setup_suite1(void)
{

      return;

}

/* The suite cleanup function.
 * Closes the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite1(void)
{

      return 0;

}

void teardown_suite1(void)
{

      return ;

}

void test_readECDSA_1(void)
{

	unsigned int result;
	unsigned char privkey[] = {0x7a, 0xc8, 0x8a, 0x77, 0x09, 0x5c, 0xe1,
			0x3e, 0x59, 0x3b, 0x83, 0x90, 0x40, 0x64, 0xf9, 0x83, 0x51, 0xdf, 0x9e,
			0xd4, 0x30, 0xeb, 0x14, 0x3c, 0x4a, 0xbc, 0x55, 0xa9, 0x84, 0xe5, 0x7f, 0x39};
	unsigned char pubkey_X[] = {0x3b, 0xe1, 0x24, 0x61, 0x9c, 0xbb, 0xeb, 0x51, 0xe9,
			0x85, 0x32, 0x8e, 0x8e, 0x33, 0xd3, 0x21, 0xca, 0xde, 0x19, 0x62, 0x8c, 0xc0,
			0xdb, 0x43, 0x30, 0x4a, 0x7b, 0x27, 0xf2, 0xdb, 0x8e, 0xfe};

	unsigned char pubkey_Y[] = {0xa8, 0x23, 0xc8, 0x85, 0x79, 0x48, 0xdc, 0x68, 0x8f, 0x3a,
			0x3e, 0xf3, 0xf6, 0xf2, 0x20, 0xa5, 0x14, 0xf0, 0x5c, 0x2c, 0x6c, 0x1c, 0xef, 0x8c,
			0x9f, 0x2f, 0x8d, 0xf1, 0x1d, 0xcf, 0x01, 0x42};

	ecdsa_key_t key;

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test.key");
	CU_ASSERT_EQUAL(result, ERR_OK);
	CU_ASSERT(memcmp(key.ecdsa_privkey, privkey, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.x, pubkey_X, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.y, pubkey_Y, key.ecdsa_len) == 0);
}


void test_readECDSA_cases(void)
{

	unsigned int result;
	unsigned char privkey[] = {0x7a, 0xc8, 0x8a, 0x77, 0x09, 0x5c, 0xe1,
			0x3e, 0x59, 0x3b, 0x83, 0x90, 0x40, 0x64, 0xf9, 0x83, 0x51, 0xdf, 0x9e,
			0xd4, 0x30, 0xeb, 0x14, 0x3c, 0x4a, 0xbc, 0x55, 0xa9, 0x84, 0xe5, 0x7f, 0x39};
	unsigned char pubkey_X[] = {0x3b, 0xe1, 0x24, 0x61, 0x9c, 0xbb, 0xeb, 0x51, 0xe9,
			0x85, 0x32, 0x8e, 0x8e, 0x33, 0xd3, 0x21, 0xca, 0xde, 0x19, 0x62, 0x8c, 0xc0,
			0xdb, 0x43, 0x30, 0x4a, 0x7b, 0x27, 0xf2, 0xdb, 0x8e, 0xfe};

	unsigned char pubkey_Y[] = {0xa8, 0x23, 0xc8, 0x85, 0x79, 0x48, 0xdc, 0x68, 0x8f, 0x3a,
			0x3e, 0xf3, 0xf6, 0xf2, 0x20, 0xa5, 0x14, 0xf0, 0x5c, 0x2c, 0x6c, 0x1c, 0xef, 0x8c,
			0x9f, 0x2f, 0x8d, 0xf1, 0x1d, 0xcf, 0x01, 0x42};

	ecdsa_key_t key;

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test_lower.key");
	CU_ASSERT_EQUAL(result, ERR_OK);
	CU_ASSERT(memcmp(key.ecdsa_privkey, privkey, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.x, pubkey_X, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.y, pubkey_Y, key.ecdsa_len) == 0);

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test_upper.key");
	CU_ASSERT_EQUAL(result, ERR_OK);
	CU_ASSERT(memcmp(key.ecdsa_privkey, privkey, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.x, pubkey_X, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.y, pubkey_Y, key.ecdsa_len) == 0);

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test_mixed_case.key");
	CU_ASSERT_EQUAL(result, ERR_OK);
	CU_ASSERT(memcmp(key.ecdsa_privkey, privkey, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.x, pubkey_X, key.ecdsa_len) == 0);
	CU_ASSERT(memcmp(key.ecdsa_pubkey.y, pubkey_Y, key.ecdsa_len) == 0);
}

void test_readECDSA_2(void)
{
	unsigned int result;
	ecdsa_key_t key;

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test_bad.key");
	CU_ASSERT_EQUAL(result, ERR_BAD_FORMAT);
}

void test_readECDSA_Empty(void)
{
	unsigned int result;
	ecdsa_key_t key;

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_test_empty.key");
	CU_ASSERT_EQUAL(result, ERR_FILE_ERROR);
}

void test_readECDSA_nofile(void)
{
	unsigned int result;
	ecdsa_key_t key;

	result = read_file_ecdsa_keypair(&key, "test/data/ecdsa_do_not_exist.key");
	CU_ASSERT_EQUAL(result, ENOENT);
}




static CU_TestInfo tests_ECDSA[] = {
  { "test of ECDSA file reading 1", test_readECDSA_1 },
  { "test of ECDSA file reading Bad", test_readECDSA_2 },
  { "test of ECDSA file reading Empty", test_readECDSA_Empty },
  { "test of ECDSA file reading No File", test_readECDSA_nofile },
  { "test of ECDSA file reading Cases", test_readECDSA_cases },
	CU_TEST_INFO_NULL,
};

CU_SuiteInfo suites [] = { { "ECDSA", init_suite1, clean_suite1, setup_suite1, teardown_suite1, tests_ECDSA },
		CU_SUITE_INFO_NULL,
	};


/* The main() function for setting up and running the tests.
 * Returns a CUE_SUCCESS on successful running, another
 * CUnit error code on failure.
 */
int TS_ECDSA(void)
{
   if (0 !=  CU_register_suites(suites)){
	   CU_cleanup_registry();
	   return CU_get_error();
   }

   return ERR_OK;
}
