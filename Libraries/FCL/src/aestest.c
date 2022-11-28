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
 */
#include <ucl_testing_config.h>

#include <ucl/ucl_hash.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_aes.h>
#include <ucl/ucl_aes_ecb.h>
#include <ucl/ucl_aes_cbc_mac.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
#include <string.h>

int test_longer(void)
{
    u8 ct_ecb[4 * 16] = { 0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca,
                          0xf3, 0x24, 0x66, 0xef, 0x97, 0xf5, 0xd3, 0xd5, 0x85, 0x03, 0xb9,
                          0x69, 0x9d, 0xe7, 0x85, 0x89, 0x5a, 0x96, 0xfd, 0xba, 0xaf, 0x43,
                          0xb1, 0xcd, 0x7f, 0x59, 0x8e, 0xce, 0x23, 0x88, 0x1b, 0x00, 0xe3,
                          0xed, 0x03, 0x06, 0x88, 0x7b, 0x0c, 0x78, 0x5e, 0x27, 0xe8, 0xad,
                          0x3f, 0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5d, 0xd4 };

    u8 pt[4 * 16] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73,
                      0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7,
                      0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4,
                      0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45,
                      0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10 };

    u8 key0[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };

    u32 result;
    u8 output[4 * 16];
    result = ucl_aes_ecb(output, pt, 4 * 16, key0, 16, UCL_CIPHER_ENCRYPT);
    if (UCL_OK != result)
        printf("ERROR AES 128 ECB ENC\n");
    if (memcmp(output, ct_ecb, (4 * 16)) == 0)
        printf("AES 128 ENC 64 bytes OK\n");
    else {
        printf("ERROR AES 128 ENC 64 bytes\n");
        return (UCL_ERROR);
    }
    return (UCL_OK);
}

int test_aes(void)
{
    // AES 128 ECB tests (from FIPS-197 test vectors, encrypt)
    u8 aeskey128s[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    u8 aesplain128s[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                          0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
    u8 aescipher128s[] = { 0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30,
                           0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A };
    u8 e256[32];
    u8 mac[UCL_AES_BLOCKSIZE];
    u8 cbcmac[UCL_AES_BLOCKSIZE] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                     0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a };
    printf("AES testing --------------------\n");
    ucl_aes(e256, aesplain128s, aeskey128s, UCL_AES_KEYLEN_128, UCL_CIPHER_ENCRYPT);
    if (memcmp(e256, aescipher128s, UCL_AES_BLOCKSIZE) != 0) {
        printf("ERROR AES ENC-128 KAT FIPS\n");
        return (UCL_ERROR);
    } else
        printf("AES ENC-128 KAT FIPS OK\n");

    ucl_aes_ecb(e256, aescipher128s, UCL_AES_BLOCKSIZE, aeskey128s, UCL_AES_KEYLEN_128,
                UCL_CIPHER_DECRYPT);
    if (memcmp(e256, aesplain128s, UCL_AES_BLOCKSIZE) != 0) {
        printf("ERROR AES DEC-128 KAT FIPS\n");
        return (UCL_ERROR);
    } else
        printf("AES DEC-128 KAT FIPS OK\n");

    ucl_aes_cbc_mac(mac, UCL_AES_BLOCKSIZE, aesplain128s, UCL_AES_BLOCKSIZE, aeskey128s,
                    UCL_AES_KEYLEN_128);
    if (memcmp(mac, cbcmac, UCL_AES_BLOCKSIZE) != 0) {
        printf("ERROR AES 128 CBC-MAC\n");
        return (UCL_ERROR);
    } else
        printf("AES 128 CBC-MAC OK\n");
    test_longer();
    return (UCL_OK);
}
