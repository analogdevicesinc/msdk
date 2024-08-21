/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <ucl_testing_config.h>

#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_sys.h>

#include <ecdsatest.h>
#include <shatest.h>
#include <aestest.h>
#include <ucl/ecdsa_generic_api.h>

#define MAJVER 1
#define MINVER 2
#define ZVER 1
//1.0.0: initial release
//1.0.1: p192 initializers size corrected; hash functions number updated
//1.1.0: sha-3 (sha3-224, sha3-256, sha3-384, sha3-512) has been added
//1.2.0: sha-384, sha-512, ecdsa p384, p521, bp256, bp384, bp512 have been added
//1.2.1: aes ecb has been added
#ifdef ECDSA
void test_ecdsa(void)
{
    PRINTF("ECDSA testing----------\n");

    if (UCL_OK == test_ecdsa_curves()) {
        PRINTF("OK\n");
    } else {
        PRINTF("NOK\n");
    }

    PRINTF("\n");
}
#endif//ECDSA

int ucl_testing(void)
{
    int result = UCL_OK;

    PRINTF("FCL testing\n");
    ucl_init();
    test_sha();
//  test_aes();
#ifdef ECDSA
    test_ecdsa();
#endif
    return result;
}

int main(void)
{
    int result = UCL_OK;
    PRINTF("FCL validation tool %d %d %d\n", MAJVER, MINVER, ZVER);
    ucl_testing();

    PRINTF("this is the end...\n");

    return result;
}
