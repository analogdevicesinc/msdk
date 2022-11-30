/******************************************************************************* 
* Copyright (C) 2015-2018 Maxim Integrated Products, Inc., All rights Reserved.
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
* ownership rights.
*     Module Name: ECC
*     Description: ECC definition
*        Filename: ecdsa_generic_api.h
*          Author: LSL
*        Compiler: gcc
*
 *******************************************************************************
 */
/** @file ecdsa_generic_api.h
 * @defgroup UCL_ECDSA ECDSA
 * ECDSA, from FIPS 186-4.
 *
 * @par Header:
 * @link ecdsa_generic_api.h ecdsa_generic_api.h @endlink
 *
 *
 * @ingroup UCL_ECC
 */

/** @brief ECDSA signatures computation and verification
* 
* Long description of the module. This module is in charge of .... 
*  * Elliptic curves have been studied by mathematicians for more than a century.
 * A rich theory have been developed around them, and cryptography have taken
 * the advantage to find a way to use it in practice.
 * Elliptic curve public key cryptosystems were proposed independently by
 * Victor Miller and Neil Koblitz in the mid-eighties. After many attempts to
 * make a cryptographic protocol with an high level of confidence,
 * the first commercial implementation is appearing in the last ten years.
 * The security of the ECC is based on difficulty of the discret logarithm
 * problem.@n
 * @n
 * <b>Principle:</b>@n
 * @n
 * An elliptic curve can be defined as an equation of the form :@n
 * @f$ E ~:~ y^{2} + a_{1}xy + a_{3}y ~=~ x^{3} + a_{2}x^{2} + a_{4}x + a_{6} @f$ @n
 * For cryptographic purpose @a E is defined on GF(p) or GF(2^m).
 * @n
 * If the characteristic of the curve is different from 2 or 3, then an
 * admissible change of variable transforms the equation to :@n
 * @f$ E ~:~ y^{2}~=~x^3+a.x+b @f$ @n
 * If the characteristic of the curve is  2 then an
 * admissible change of variable transforms the equation to :@n
 * @f$ E ~:~ y^{2} + x.y ~=~x^3+ a.x^2 + b @f$ @n
 * @n
 * A point of the curve is a couple @f$ (x,y) @f$  verifying the equation @a E. @n
 * @n
 * It exists an addition on @a E such as @a E is a group.
 * If @a P is point of @a E, then we note @f$ Q = k.P @f$ the result of @p k 
 * successive additions. @n
 * The discrete logarithm problem is the problem which consists to find @p k 
 * from @p Q and @p P.
 * ECDSA is the implementation of the DSA in ECC
*/ 

#ifndef _UCL_ECDSA_GENERIC_API_NEW_H_
#define _UCL_ECDSA_GENERIC_API_NEW_H_
#include "ucl/ucl_config.h"
#include "ucl/ucl_types.h"
#include "ucl/bignum_ecdsa_generic_api.h"

/* ECDSA key lengths */
#define ECDSA_BLOCK_SIZE 32
#ifdef WORD32
#define ECDSA_DIGITS 17
#endif

#define SECP192R1 0
#define SECP224R1 1
#define SECP256R1 2
#define SECP160R1 3
#define SECP384R1 4
#define SECP521R1 5
#define SM2FP192 6
#define SM2FP256 7
#define BP256R1 8
#define BP384R1 9
#define BP512R1 10
//VP for various
#define SM2VP256 11
#define UNKNOWN_CURVE 12
#define MAX_CURVE 12

#define SECP160R1_BYTESIZE 20
#define SECP192R1_BYTESIZE 24
#define SM2FP192_BYTESIZE 24
#define SECP224R1_BYTESIZE 28
#define SECP256R1_BYTESIZE 32
#define BP256R1_BYTESIZE 32
#define SM2FP256_BYTESIZE 32
#define SECP384R1_BYTESIZE 48
#define SECP521R1_BYTESIZE 66
#define BP384R1_BYTESIZE 48
#define BP512R1_BYTESIZE 64


#define SECP160R1_WORDSIZE 8
#define SECP192R1_WORDSIZE 8
#define SM2FP192_WORDSIZE 8
#define SECP224R1_WORDSIZE 8
#define SECP256R1_WORDSIZE 8
#define BP256R1_WORDSIZE 8
#define SM2FP256_WORDSIZE 8
#define SECP384R1_WORDSIZE 12
#define SECP521R1_WORDSIZE 17
#define BP384R1_WORDSIZE 12
#define BP512R1_WORDSIZE 16

//internal defines


#define P192
#define P256
#define P384
#define P521
#define BP256
#define BP384
#define BP512

static const u32 one[ECDSA_DIGITS]={0x00000001,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const u32 two[ECDSA_DIGITS]={0x00000002,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const u32 three[ECDSA_DIGITS]={0x00000003,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
static const u32 four[ECDSA_DIGITS]={0x00000004,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};


/** <b>ECC Curve Structure</b>.
 *
 * @ingroup UCL_ECDSA
 */
typedef struct _t_curve
{
  const u32 *a;/**< curve equation a parameter.                                         */
  const u32 *b;/**< curve equation b parameter.                                         */
  const u32 *p;/**< curve equation p paramter.                                         */
  const u32 *n;/**< curve order.                                         */
  const u32 *xg;/**< curve base point x coordinate.                                         */
  const u32 *yg;/**< curve base point y coordinate.                                         */
  u32 *invp2;/**< curve field inversion of 2.                                         */
  u32 *psquare;/**< curve p parameter square.                                         */
  void *px;/**< curve precomputed x values (may be NULL) .                                         */
  void *py;/**< curve precomputed y values (may be NULL).                                         */
  u32 curve_wsize;/**< curve word size.                                         */
  u32 curve_bsize;/**< curve byte size.                                         */
  int curve;/**< curve identifier.                                         */
} ucl_type_curve;

/** <b>ECC word Jacobian point coordinates</b>.
 * this structure is used within ECC routines
 * @ingroup UCL_ECDSA
 */
typedef struct _t_jacobian_point
{
  u32 *x;/**< jacobian x coordinate.                                         */
  u32 *y;/**< jacobian y coordinate.                                         */
  u32 *z;/**< jacobian z coordinate.                                         */
} ucl_type_ecc_jacobian_point;

/** <b>ECC byte affine point coordinates</b>.
 * this structure is used at application level
 * @ingroup UCL_ECDSA
 */
typedef struct _t_u8_affine_point
{
  u8 *x;/**< affine x coordinate.                                         */
  u8 *y;/**< affine y coordinate.                                         */
} ucl_type_ecc_u8_affine_point;

/** <b>ECC word affine point coordinates</b>.
 * this structure is used within ECC routines
 *
 * @ingroup UCL_ECDSA
 */
typedef struct _t_digit_affine_point
{
  u32 *x;/**< affine x coordinate.                                         */
  u32 *y;/**< affine y coordinate.                                         */
} ucl_type_ecc_digit_affine_point;

/** <b>ECC byte signature structure</b>.
 * this structure is used at application level
 *
 * @ingroup UCL_ECDSA
 */
typedef struct _t_ecdsa_signature
{
  u8 *r;/**<signature r  value */
  u8 *s;/**<signature s value */
} ucl_type_ecdsa_signature;

#ifdef P192
  static const u32 local_inv2_p192r1[SECP192R1_WORDSIZE]={0x00000000,0x80000000,0xffffffff,0xffffffff,0xffffffff,0x7fffffff,0x00000000,0x00000000};
  static const u32 local_xg_p192r1[SECP192R1_WORDSIZE]={0x82ff1012,0xf4ff0afd,0x43a18800,0x7cbf20eb,0xb03090f6,0x188da80e,0x00000000,0x00000000};
  static const u32 local_yg_p192r1[SECP192R1_WORDSIZE]={0x1e794811,0x73f977a1,0x6b24cdd5,0x631011ed,0xffc8da78,0x07192b95,0x00000000,0x00000000};
  static const u32 local_a_p192r1[SECP192R1_WORDSIZE]={0xfffffffc,0xffffffff,0xfffffffe,0xffffffff,0xffffffff,0xffffffff,0x00000000,0x00000000};
static const u32 local_b_p192r1[SECP192R1_WORDSIZE]={0xc146b9b1,0xfeb8deec,0x72243049,0x0fa7e9ab,0xe59c80e7,0x64210519,0x00000000,0x00000000};
  static const u32 local_p_p192r1[SECP192R1_WORDSIZE]={0xffffffff,0xffffffff,0xfffffffe,0xffffffff,0xffffffff,0xffffffff,0x00000000,0x00000000};
  static const u32 local_n_p192r1[SECP192R1_WORDSIZE]={0xb4d22831,0x146bc9b1,0x99def836,0xffffffff,0xffffffff,0xffffffff,0x00000000,0x00000000};
/** <b>ECC Curve structure variable for SEC-P192r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp192r1;
#endif//P192

#ifdef P256
  static const u32 local_inv2_p256r1[SECP256R1_WORDSIZE]={0x00000000,0x00000000,0x80000000,0x00000000,0x00000000,0x80000000,0x80000000,0x7fffffff};
  static const u32 local_xg_p256r1[SECP256R1_WORDSIZE]={0xd898c296,0xf4a13945,0x2deb33a0,0x77037d81,0x63a440f2,0xf8bce6e5,0xe12c4247,0x6b17d1f2};
  static const u32 local_yg_p256r1[SECP256R1_WORDSIZE]={0x37bf51f5,0xcbb64068,0x6b315ece,0x2bce3357,0x7c0f9e16,0x8ee7eb4a,0xfe1a7f9b,0x4fe342e2};
  static const u32 local_a_p256r1[SECP256R1_WORDSIZE]={0xfffffffc,0xffffffff,0xffffffff,0x00000000,0x00000000,0x00000000,0x00000001,0xffffffff};
static const u32 local_b_p256r1[SECP256R1_WORDSIZE]={0x27d2604b,0x3bce3c3e,0xcc53b0f6,0x651d06b0,0x769886bc,0xb3ebbd55,0xaa3a93e7,0x5ac635d8};
  static const u32 local_p_p256r1[SECP256R1_WORDSIZE]={0xffffffff,0xffffffff,0xffffffff,0x00000000,0x00000000,0x00000000,0x00000001,0xffffffff};
  static const u32 local_n_p256r1[SECP256R1_WORDSIZE]={0xfc632551,0xf3b9cac2,0xa7179e84,0xbce6faad,0xffffffff,0xffffffff,0x00000000,0xffffffff};
/** <b>ECC Curve structure variable for SEC-P256r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp256r1;
#endif//P256


#ifdef BP256
   static const u32 local_inv2_bp256r1[BP256R1_WORDSIZE]={0x8fb729bc,0x1009a40e,0xea931014,0x371dfb11,0x4ec1c6b9,0x1f330548,0xd0f754de,0x54fdabed};
  static const u32 local_xg_bp256r1[BP256R1_WORDSIZE]={0x9ace3262,0x3a4453bd,0xe3bd23c2,0xb9de27e1,0xfc81b7af,0x2c4b482f,0xcb7e57cb,0x8bd2aeb9};
  static const u32 local_yg_bp256r1[BP256R1_WORDSIZE]={0x2f046997,0x5c1d54c7,0x2ded8e54,0xc2774513,0x14611dc9,0x97f8461a,0xc3dac4fd,0x547ef835};
  static const u32 local_a_bp256r1[BP256R1_WORDSIZE]={0xf330b5d9,0xe94a4b44,0x26dc5c6c,0xfb8055c1,0x417affe7,0xeef67530,0xfc2c3057,0x7d5a0975};
static const u32 local_b_bp256r1[BP256R1_WORDSIZE]={0xff8c07b6,0x6bccdc18,0x5cf7e1ce,0x95841629,0xbbd77cbf,0xf330b5d9,0xe94a4b44,0x26dc5c6c};
  static const u32 local_p_bp256r1[BP256R1_WORDSIZE]={0x1f6e5377,0x2013481d,0xd5262028,0x6e3bf623,0x9d838d72,0x3e660a90,0xa1eea9bc,0xa9fb57db};
  static const u32 local_n_bp256r1[BP256R1_WORDSIZE]={0x974856a7,0x901e0e82,0xb561a6f7,0x8c397aa3,0x9d838d71,0x3e660a90,0xa1eea9bc,0xa9fb57db};

/** <b>ECC Curve structure variable for Brainpool P256r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp256r1;
#endif//BP256

#ifdef BP384
   static const u32 local_inv2_bp384r1[BP384R1_WORDSIZE]={0x9883f62a,0xc3a38009,0xc80e8d38,0xd669d394,0xbfdb8891,0x0958ed0c,0xf6aa2b5a,0x8a97b884,0x287320ef,0x07aeb7bf,0x519c3694,0x465c8f41};
  static const u32 local_xg_bp384r1[BP384R1_WORDSIZE]={0x47d4af1e,0xef87b2e2,0x36d646aa,0xe826e034,0x0cbd10e8,0xdb7fcafe,0x7ef14fe3,0x8847a3e7,0xb7c13f6b,0xa2a63a81,0x68cf45ff,0x1d1c64f0};
  static const u32 local_yg_bp384r1[BP384R1_WORDSIZE]={0x263c5315,0x42820341,0x77918111,0x0e464621,0xf9912928,0xe19c054f,0xfeec5864,0x62b70b29,0x95cfd552,0x5cb1eb8e,0x20f9c2a4,0x8abe1d75};
  static const u32 local_a_bp384r1[BP384R1_WORDSIZE]={0x22ce2826,0x04a8c7dd,0x503ad4eb,0x8aa5814a,0xba91f90f,0x139165ef,0x4fb22787,0xc2bea28e,0xce05afa0,0x3c72080a,0x3d8c150c,0x7bc382c6};
static const u32 local_b_bp384r1[BP384R1_WORDSIZE]={0xfa504c11,0x3ab78696,0x95dbc994,0x7cb43902,0x3eeb62d5,0x2e880ea5,0x07dcd2a6,0x2fb77de1,0x16f0447c,0x8b39b554,0x22ce2826,0x04a8c7dd};
static const u32 local_p_bp384r1[BP384R1_WORDSIZE]={0x3107ec53,0x87470013,0x901d1a71,0xacd3a729,0x7fb71123,0x12b1da19,0xed5456b4,0x152f7109,0x50e641df,0x0f5d6f7e,0xa3386d28,0x8cb91e82};
  static const u32 local_n_bp384r1[BP384R1_WORDSIZE]={0xe9046565,0x3b883202,0x6b7fc310,0xcf3ab6af,0xac0425a7,0x1f166e6c,0xed5456b3,0x152f7109,0x50e641df,0x0f5d6f7e,0xa3386d28,0x8cb91e82};
/** <b>ECC Curve structure variable for Brainpool P384r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp384r1;
#endif

#ifdef P384
static const u32 local_inv2_p384r1[SECP384R1_WORDSIZE]={0x80000000,0x00000000,0x80000000,0x7fffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0x7fffffff};
static const u32 local_xg_p384r1[SECP384R1_WORDSIZE]={0x72760ab7,0x3a545e38,0xbf55296c,0x5502f25d,0x82542a38,0x59f741e0,0x8ba79b98,0x6e1d3b62,0xf320ad74,0x8eb1c71e,0xbe8b0537,0xaa87ca22};
static const u32 local_yg_p384r1[SECP384R1_WORDSIZE]={0x90ea0e5f,0x7a431d7c,0x1d7e819d,0x0a60b1ce,0xb5f0b8c0,0xe9da3113,0x289a147c,0xf8f41dbd,0x9292dc29,0x5d9e98bf,0x96262c6f,0x3617de4a};
 static const u32 local_a_p384r1[SECP384R1_WORDSIZE]={0xfffffffc,0x00000000,0x00000000,0xffffffff,0xfffffffe,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff};
 static const u32 local_b_p384r1[SECP384R1_WORDSIZE]={0xd3ec2aef,0x2a85c8ed,0x8a2ed19d,0xc656398d,0x5013875a,0x0314088f,0xfe814112,0x181d9c6e,0xe3f82d19,0x988e056b,0xe23ee7e4,0xb3312fa7};
  static const u32 local_p_p384r1[SECP384R1_WORDSIZE]={0xffffffff,0x00000000,0x00000000,0xffffffff,0xfffffffe,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff};
  static const u32 local_n_p384r1[SECP384R1_WORDSIZE]={0xccc52973,0xecec196a,0x48b0a77a,0x581a0db2,0xf4372ddf,0xc7634d81,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff};

/** <b>ECC Curve structure variable for SEC-P384r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp384r1;
#endif//P384

#ifdef P521
  static const u32 local_inv2_p521r1[SECP521R1_WORDSIZE]={0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000100};
  static const u32 local_xg_p521r1[SECP521R1_WORDSIZE]={0xc2e5bd66,0xf97e7e31,0x856a429b,0x3348b3c1,0xa2ffa8de,0xfe1dc127,0xefe75928,0xa14b5e77,0x6b4d3dba,0xf828af60,0x053fb521,0x9c648139,0x2395b442,0x9e3ecb66,0x0404e9cd,0x858e06b7,0x000000c6};
  static const u32 local_yg_p521r1[SECP521R1_WORDSIZE]={0x9fd16650,0x88be9476,0xa272c240,0x353c7086,0x3fad0761,0xc550b901,0x5ef42640,0x97ee7299,0x273e662c,0x17afbd17,0x579b4468,0x98f54449,0x2c7d1bd9,0x5c8a5fb4,0x9a3bc004,0x39296a78,0x00000118};
  static const u32 local_a_p521r1[SECP521R1_WORDSIZE]={0xfffffffc,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0x000001ff};
static const u32 local_b_p521r1[SECP521R1_WORDSIZE]={0x6b503f00,0xef451fd4,0x3d2c34f1,0x3573df88,0x3bb1bf07,0x1652c0bd,0xec7e937b,0x56193951,0x8ef109e1,0xb8b48991,0x99b315f3,0xa2da725b,0xb68540ee,0x929a21a0,0x8e1c9a1f,0x953eb961,0x00000051};
  static const u32 local_p_p521r1[SECP521R1_WORDSIZE]={0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0x000001ff};
  static const u32 local_n_p521r1[SECP521R1_WORDSIZE]={0x91386409,0xbb6fb71e,0x899c47ae,0x3bb5c9b8,0xf709a5d0,0x7fcc0148,0xbf2f966b,0x51868783,0xfffffffa,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0xffffffff,0x000001ff};

/** <b>ECC Curve structure variable for SEC-P521r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp521r1;
#endif//P521

#ifdef BP512
   static const u32 local_inv2_bp512r1[BP512R1_WORDSIZE]={0x2c1d247a,0x9455302b,0x96c16342,0x1440ff97,0x7351c073,0x5766d095,0x4de33421,0xbea6cd80,0x38198438,0x6b31ce65,0xd9e4e907,0xe59846d9,0x19e4fe03,0x9fea7357,0x6df4e245,0x556ecedc};
  static const u32 local_xg_bp512r1[BP512R1_WORDSIZE]={0xbcb9f822,0x8b352209,0x406a5e68,0x7c6d5047,0x93b97d5f,0x50d1687b,0xe2d0d48d,0xff3b1f78,0xf4d0098e,0xb43b62ee,0xb5d916c1,0x85ed9f70,0x9c4c6a93,0x5a21322e,0xd82ed964,0x81aee4bd};
  static const u32 local_yg_bp512r1[BP512R1_WORDSIZE]={0x3ad80892,0x78cd1e0f,0xa8f05406,0xd1ca2b2f,0x8a2763ae,0x5bca4bd8,0x4a5f485e,0xb2dcde49,0x881f8111,0xa000c55b,0x24a57b1a,0xf209f700,0xcf7822fd,0xc0eabfa9,0x566332ec,0x7dde385d};
  static const u32 local_a_bp512r1[BP512R1_WORDSIZE]={0x77fc94ca,0xe7c1ac4d,0x2bf2c7b9,0x7f1117a7,0x8b9ac8b5,0x0a2ef1c9,0xa8253aa1,0x2ded5d5a,0xea9863bc,0xa83441ca,0x3df91610,0x94cbdd8d,0xac234cc5,0xe2327145,0x8b603b89,0x7830a331};
static const u32 local_b_bp512r1[BP512R1_WORDSIZE]={0x8016f723,0x2809bd63,0x5ebae5dd,0x984050b7,0xdc083e67,0x77fc94ca,0xe7c1ac4d,0x2bf2c7b9,0x7f1117a7,0x8b9ac8b5,0x0a2ef1c9,0xa8253aa1,0x2ded5d5a,0xea9863bc,0xa83441ca,0x3df91610};
  static const u32 local_p_bp512r1[BP512R1_WORDSIZE]={0x583a48f3,0x28aa6056,0x2d82c685,0x2881ff2f,0xe6a380e6,0xaecda12a,0x9bc66842,0x7d4d9b00,0x70330871,0xd6639cca,0xb3c9d20e,0xcb308db3,0x33c9fc07,0x3fd4e6ae,0xdbe9c48b,0xaadd9db8};
  static const u32 local_n_bp512r1[BP512R1_WORDSIZE]={0x9ca90069,0xb5879682,0x085ddadd,0x1db1d381,0x7fac1047,0x41866119,0x4ca92619,0x553e5c41,0x70330870,0xd6639cca,0xb3c9d20e,0xcb308db3,0x33c9fc07,0x3fd4e6ae,0xdbe9c48b,0xaadd9db8};
/** <b>ECC Curve structure variable for Brainpool P512r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp512r1;
#endif//BP512

/** <b>ECDSA signature</b>.
 * Compute a ECDSA signature, using curve domain parameters
 *
* @param[out] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] *d: input, the secret key
* @param[in] *ucl_hash: input, the pointer to the hash function (see hash functions documentation for already available ones)
* @param[in] *input: input, the message or the hash digest to be signed,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)
     - Examples are:
       - UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest, r not precomputed
       - UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest, r not precomputed
       - UCL_PRECOMP_R<<UCL_PRECOMP_SHIFT+UCL_NO_INPUT<<UCL_INPUT_SHIFT: pre-compute r 
       - UCL_R_PRECOMP<<UCL_PRECOMP_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT:signing a message, using pre-computed r

 * @return Error code
 *
 * @retval #UCL_OK in case of correct computation
 * @retval #UCL_INVALID_INPUT or #UCL_INVALID OUTPUT in case of wrong parameters configuration (e.g. NULL pointers)
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_signature(ucl_type_ecdsa_signature signature,u8 *d,int(*ucl_hash)(u8*,u8*,u32),u8 *input, u32 inputlength, ucl_type_curve *curve_params,u32 configuration);

/** <b>ECDSA signature verification</b>.
 * Verify a ECDSA signature, using curve domain structure
*
* @param[in] public key: pointer to a ucl_type_ecc_u8_affine_point structure, containing the ECC public key, used for signature verification
* @param[in] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] *ucl_hash: input, the pointer to the hash function (see hash functions documentation for already available ones)
* @param[in] *input: input, the message or the hash digest to be signed,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
* @param[in] configuration (combination of any of these lines) 
* @param[in] 
* @param[in] *ucl_hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verification(ucl_type_ecc_u8_affine_point Q,ucl_type_ecdsa_signature signature,int(*ucl_hash)(u8*,u8*,u32),u8 *input,u32 inputlength,ucl_type_curve *curve_params,u32 configuration);

/** <b>ECC multiplication</b>.
 * multiply a scalar by a ECC point, using affine parameters, computation performed in Jacobian parameters
*
* @param[out] Q: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC result point,
* @param[in] m: pointer to a word array containing the scalar,
* @param[in] X1: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC point, to be multiplied by the scalar,
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
* @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ecc_mult_jacobian(ucl_type_ecc_digit_affine_point Q, u32 *m, ucl_type_ecc_digit_affine_point X1,ucl_type_curve *curve_params);

/** <b>ECC addition</b>.
 * add two ECC points in affine coordinates
*
* @param[out] Q3: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC result point,
* @param[in] Q1: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC point, to be added with,
* @param[in] Q2: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC point, to be added with,
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
* @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ecc_add(ucl_type_ecc_digit_affine_point Q3,ucl_type_ecc_digit_affine_point Q1,ucl_type_ecc_digit_affine_point Q2,ucl_type_curve *curve_params);

/** <b>ECC doubling</b>.
 * doubling one ECC point in affine coordinates
*
* @param[out] Q3: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC result point,
* @param[in] Q1: pointer to a ucl_ecc_digit_affine_point structure, containing the ECC point, to be doubled
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
* @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ecc_double(ucl_type_ecc_digit_affine_point Q3,ucl_type_ecc_digit_affine_point Q1, ucl_type_curve *curve_params);


#endif//ECDSA_GENERIC_API
