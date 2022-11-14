/******************************************************************************* 
* Copyright (C) 2017 Maxim Integrated Products, Inc., All rights Reserved.
* * This software is protected by copyright laws of the United States and
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
* ownership rights.*/

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
//1.1.0: sha-3 (sha3-224,sha3-256,sha3-384, sha3-512) has been added
//1.2.0: sha-384, sha-512, ecdsa p384, p521, bp256, bp384, bp512 have been added
//1.2.1: aes ecb has been added
#ifdef ECDSA
void test_ecdsa(void)
{
  PRINTF("ECDSA testing----------\n");
  if(UCL_OK==test_ecdsa_curves())
    PRINTF("OK\n");
  else
    PRINTF("NOK\n");
  PRINTF("\n");
}
#endif//ECDSA

int ucl_testing(void)
{
  int result=UCL_OK;

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
  int result=UCL_OK;
  PRINTF("FCL validation tool %d %d %d\n",MAJVER,MINVER,ZVER);
  ucl_testing();

  PRINTF("this is the end...\n");

  return result;
}
