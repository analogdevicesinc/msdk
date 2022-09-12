/** @file ecdsa_generic_api.h
 * @defgroup UCL_ECDSA ECDSA
 * Elliptic Curve Digital signature Algorithm.
 *
 * ECDSA is the implementation of DSA using the ECC.
 *
 * @par Header:
 * @link ecdsa_generic_api.h ecdsa_generic_api.h @endlink
 *
 *
 * @ingroup UCL_ECC
 */

#ifndef _UCL_ECDSA_GENERIC_API_NEW_H_
#define _UCL_ECDSA_GENERIC_API_NEW_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
#include "ucl/ucl_config.h"
#include "ucl/ucl_types.h"
#include "ucl/bignum_ecdsa_generic_api.h"

#define MULT_JAC_VERIF_DISABLED 24
#define MULT_JAC_VERIF_ENABLED 17

/* ECDSA key lengths */
//#define ECDSA_BLOCK_SIZE 256/8
//#define ECDSA_BLOCK_SIZE 32
//#define ECDSA_BLOCK_SIZE 192/8
//#define ECDSA_BLOCK_SIZE 32
#define ECDSA_BLOCK_SIZE 32
//sufficient for p192
//#define ECDSA_DIGITS 13
//sufficient for p2
#ifdef WORD32
#define ECDSA_DIGITS 17
#endif
#ifdef WORD16
#define ECDSA_DIGITS 40
#endif

#define SECP192R1 0
#define SECP224R1 1
#define SECP256R1 2
#define SECP256K1 13
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
#define SECP128R1 12
#define UNKNOWN_CURVE 14
#define MAX_CURVE 14

#define SECP128R1_BYTESIZE 16
#define SECP160R1_BYTESIZE 20
#define SECP192R1_BYTESIZE 24
#define SM2FP192_BYTESIZE 24
#define SECP224R1_BYTESIZE 28
#define SECP256R1_BYTESIZE 32
#define SECP256K1_BYTESIZE 32
#define BP256R1_BYTESIZE 32
#define SM2FP256_BYTESIZE 32
#define SECP384R1_BYTESIZE 48
#define SECP521R1_BYTESIZE 66
#define BP384R1_BYTESIZE 48
#define BP512R1_BYTESIZE 64

#define SECP128R1_BITSIZE 128
#define SECP160R1_BITSIZE 160
#define SECP192R1_BITSIZE 192
#define SM2FP192_BITSIZE 192
#define SECP224R1_BITSIZE 224
#define SECP256R1_BITSIZE 256
#define SECP256K1_BITSIZE 256
#define BP256R1_BITSIZE 256
#define SM2FP256_BITSIZE 256
#define SECP384R1_BITSIZE 384
#define SECP521R1_BITSIZE 521
#define BP384R1_BITSIZE 384
#define BP512R1_BITSIZE 512

#ifdef WORD16
#define SECP128R1_WORDSIZE 8
#define SECP160R1_WORDSIZE 16
#define SECP192R1_WORDSIZE 16
#define SM2FP192_WORDSIZE 16
#define SECP224R1_WORDSIZE 16
#define SECP256R1_WORDSIZE 16
#define SECP256K1_WORDSIZE 16
#define BP256R1_WORDSIZE 16
#define SM2FP256_WORDSIZE 16
#define SECP384R1_WORDSIZE 24
#define SECP521R1_WORDSIZE 34
#define BP384R1_WORDSIZE 24
#define BP512R1_WORDSIZE 32
#endif //WORD16

#ifdef WORD32
//8 up to now
#define SECP128R1_WORDSIZE 4
#define SECP160R1_WORDSIZE 8
#define SECP192R1_WORDSIZE 8
#define SM2FP192_WORDSIZE 8
#define SECP224R1_WORDSIZE 8
#define SECP256R1_WORDSIZE 8
#define SECP256K1_WORDSIZE 8
#define BP256R1_WORDSIZE 8
#define SM2FP256_WORDSIZE 8
#define SECP384R1_WORDSIZE 12
#define SECP521R1_WORDSIZE 17
#define BP384R1_WORDSIZE 12
#define BP512R1_WORDSIZE 16
#endif //WORD32

//internal defines
#define P384

#if defined(MAXQ1060)
//#define P224
#define P384
#ifndef __wasp
#define P521
#endif
//#define BP256
//#define BP384
//#define BP512
//#define P192
#define P256

#else //!MAXQ1060

#ifndef PROFILE_2
#ifndef PROFILE_1
//#define SM2P192
#define SM2P256
#define P384
#define P521
#define BP256
#define BP384
#define BP512
#endif //PROFILE_1
#define P224
#define P160
#define P192
#define P128
#endif //PROFILE_2
#define P256
#ifndef ROMCODE_P256
#define K256
#endif //ROMCODE P256
#endif //1060

#ifdef WORD32
static const DIGIT one[ECDSA_DIGITS] = { 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000 };
static const DIGIT two[ECDSA_DIGITS] = { 0x00000002, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                         0x00000000, 0x00000000 };
static const DIGIT three[ECDSA_DIGITS] = { 0x00000003, 0x00000000, 0x00000000, 0x00000000,
                                           0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                           0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                           0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                           0x00000000 };
static const DIGIT four[ECDSA_DIGITS] = { 0x00000004, 0x00000000, 0x00000000, 0x00000000,
                                          0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                          0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                          0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                          0x00000000 };
#endif
#ifndef MAXQ1060
extern void *precompx;
extern void *precompy;
#endif //1060

/** <b>ECC Curve Structure</b>.
 *
 * @ingroup UCL_ECDSA
 */

#define SIGNATURE_COMPUTATION 0xFF
#define SIGNATURE_VERIFICATION 0x00

typedef struct _t_curve {
    const DIGIT *a; /**< curve equation a parameter.                                         */
    const DIGIT *b; /**< curve equation b parameter.                                         */
    const DIGIT *p; /**< curve equation p paramter.                                         */
    const DIGIT *n; /**< curve order.                                         */
    const DIGIT *xg; /**< curve base point x coordinate.                                         */
    const DIGIT *yg; /**< curve base point y coordinate.                                         */
    const DIGIT *invp2; /**< curve field inversion of 2.                                         */
    const DIGIT *psquare; /**< curve p parameter square.                                         */
    void *px; /**< curve precomputed x values (may be NULL) .                                         */
    void *py; /**< curve precomputed y values (may be NULL).                                         */
    DIGIT *p1x; /**<curve precomputed 1|0...0 . P */
    DIGIT *p1y;
    DIGIT curve_wsize; /**< curve word size.                                         */
    DIGIT curve_bsize; /**< curve byte size.                                         */
    int curve; /**< curve identifier.                                         */
} ucl_type_curve;

/** <b>ECC word Jacobian point coordinates</b>.
 * this structure is used within ECC routines
 * @ingroup UCL_ECDSA
 */
typedef struct _t_jacobian_point {
    DIGIT *x; /**< jacobian x coordinate.                                         */
    DIGIT *y; /**< jacobian y coordinate.                                         */
    DIGIT *z; /**< jacobian z coordinate.                                         */
} ucl_type_ecc_jacobian_point;

/** <b>ECC byte affine point coordinates</b>.
 * this structure is used at application level
 * @ingroup UCL_ECDSA
 */
typedef struct _t_u8_affine_point {
    u8 *x; /**< affine x coordinate.                                         */
    u8 *y; /**< affine y coordinate.                                         */
} ucl_type_ecc_u8_affine_point;

/** <b>ECC word affine point coordinates</b>.
 * this structure is used within ECC routines
 *
 * @ingroup UCL_ECDSA
 */
typedef struct _t_digit_affine_point {
    DIGIT *x; /**< affine x coordinate.                                         */
    DIGIT *y; /**< affine y coordinate.                                         */
} ucl_type_ecc_digit_affine_point;

/** <b>ECC byte signature structure</b>.
 * this structure is used at application level
 *
 * @ingroup UCL_ECDSA
 */
typedef struct _t_ecdsa_signature {
    u8 *r; /**<signature r  value */
    u8 *s; /**<signature s value */
} ucl_type_ecdsa_signature;

#ifndef ROMCODE_P256
#ifdef P128
#ifdef WORD32
static const DIGIT local_inv2_p128r1[SECP128R1_WORDSIZE] = { 0xffffffff, 0xffffffff, 0xffffffff,
                                                             0x7fffffff };
static const DIGIT local_xg_p128r1[SECP128R1_WORDSIZE] = { 0xA52C5B86, 0x0C28607C, 0x8B899B2D,
                                                           0x161FF752 };
static const DIGIT local_yg_p128r1[SECP128R1_WORDSIZE] = { 0xDDED7A83, 0xC02DA292, 0x5BAFEB13,
                                                           0xCF5AC839 };
static const DIGIT local_a_p128r1[SECP128R1_WORDSIZE] = { 0xfffffffc, 0xffffffff, 0xffffffff,
                                                          0xfffffffd };
static const DIGIT local_b_p128r1[SECP128R1_WORDSIZE] = { 0x2CEE5ED3, 0xD824993C, 0x1079F43D,
                                                          0xE87579C1 };
static const DIGIT local_p_p128r1[SECP128R1_WORDSIZE] = { 0xffffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff };
static const DIGIT local_n_p128r1[SECP128R1_WORDSIZE] = { 0x9038A115, 0x75A30D1B, 0x00000000,
                                                          0xFFFFFFFE };
#endif //WORD32
extern ucl_type_curve secp128r1;
#endif //P128

#ifdef P160
#ifdef WORD32
#ifdef PRECOMPUT
static const DIGIT local_px160[40][SECP160R1_WORDSIZE] = {
    { 0x13cbfc82, 0x68c38bb9, 0x46646989, 0x8ef57328, 0x4a96b568, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x5f315034, 0xaa6661be, 0x5cac2772, 0x2a82b99b, 0xb32f7dfa, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xaa957a64, 0x803d48b4, 0xa8df4ad2, 0x62ac662f, 0x8aff1b53, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x757bafef, 0x1221d86a, 0x9b534159, 0x93ce3609, 0x5490ee04, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x8420f8e2, 0x31b94cfb, 0xb96713d0, 0x2d40eb34, 0x5871afd9, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xa530cce9, 0x953d81e6, 0xdebd8771, 0x4d04134f, 0xb8268a65, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xcb385caf, 0xc04a34bc, 0x09951d5c, 0x8b6f0e9a, 0x573581a7, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x93c808ee, 0x572369a4, 0x337dce2f, 0x9541777f, 0xd7c44b25, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x6dc91879, 0x3feb69b8, 0xe2cd29f0, 0x16539a59, 0x9c9304c6, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x93bd83dc, 0xcd56effa, 0xc135dbb8, 0xcd4d5fa9, 0x637f3252, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x049f0167, 0xe0594b15, 0x306557a0, 0xb0db743e, 0x8cf07374, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xc9db040a, 0x3dfa5215, 0x513c1a50, 0x929d46c2, 0x319634c4, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xd9202553, 0x5f30c498, 0x7fcbd2eb, 0x685c43c3, 0xf6bda813, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xbe5505e7, 0x1350db98, 0x1ddda9a5, 0x5e765c78, 0xdf259f5d, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xfe922a73, 0x1dce74fd, 0x85624305, 0x609611e9, 0x8f5fc6b1, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x372012f6, 0x3ded2e09, 0xbaa44bf3, 0xf83342c6, 0x0fb8c9c4, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xff06b459, 0x0f899475, 0xbb0d8974, 0xff943332, 0x6e755291, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x10bb2214, 0x0567e30e, 0x7dd72636, 0x77b25a2f, 0x514539b1, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xca4cdbe8, 0x076e67b4, 0xa1da01c6, 0x2bed152d, 0x43d3d0ef, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x512dfae1, 0xe0054ee8, 0xab12a83a, 0xacd52510, 0x7dd903a5, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x89665347, 0xb21844a4, 0xa2961a17, 0xbe4ad4c7, 0xf2e0a32f, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xf279e884, 0x9389b466, 0x4efed9e8, 0xc065cc5b, 0xb65086b9, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xbcb574b6, 0x276dcb1b, 0xfe366098, 0x38277034, 0x639bfdad, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x490dd1ff, 0x1e3cf970, 0xb5ed5433, 0x6da8b18f, 0xe1c3cd3e, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xb431ed7e, 0x8bee4c92, 0x0f5f631f, 0xe9fc708b, 0xaab2bcb2, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x5534eb67, 0xb0642eb6, 0x5604607e, 0xfef61587, 0x0bc6ec2d, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xd04fc329, 0xc7253b5d, 0xc8b50578, 0xb726c368, 0xb56a0944, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xc94887a0, 0xe5fabeea, 0x468c22cd, 0x41d8df82, 0x30b96f4a, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x71484cb9, 0x51fc4a12, 0x43673123, 0xc680af61, 0x226ede9a, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xc580d2b0, 0xcef74a5e, 0x3b1abc99, 0x88cdde8e, 0xa244dd82, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xd408b269, 0x8ec7f46b, 0x644eaf62, 0xb77ae59d, 0xa22fb604, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x803f7d4b, 0xfaad7cdf, 0xf77f0de7, 0x82b9862f, 0xd7579d6d, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x16fc7d8b, 0x1eefb81e, 0x895ccb69, 0x6ce4d0dc, 0x75597b76, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x72051190, 0xaa3f2844, 0x9b696bcd, 0x225eedd4, 0xc8af5a5c, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x75345734, 0x180cf93b, 0x372502d8, 0x37a5f233, 0xedc942e6, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xffaf53a0, 0x413c980c, 0x8660f43b, 0x66163ba5, 0x5ceb5c43, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xb90d3f84, 0x0681933b, 0xb78a401d, 0xb8fb4703, 0xf5ea6709, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xd6319ded, 0xfa2c208e, 0x1c6f3628, 0xbc008f64, 0xc319583b, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xc2302bb2, 0x035761cd, 0xc3f2630f, 0xac47fb29, 0x630eb4e9, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x0398d1a7, 0xfc628121, 0xcbe424ff, 0x59b2c135, 0xcc26b449, 0x00000000, 0x00000000,
      0x00000000 }
};
static const DIGIT local_py160[40][SECP160R1_WORDSIZE] = {
    { 0x7ac5fb32, 0x04235137, 0x59dcc912, 0x3168947d, 0x23a62855, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xeef2b12e, 0x51d31d81, 0x05c344ba, 0x28550bb3, 0x5c9e8a6f, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xd6505ad4, 0x33f307fe, 0xa0284bce, 0xc35dfda5, 0x892bf1eb, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xa51990e0, 0x3a21164c, 0xa63fbce0, 0x04dc533e, 0xaa1801cb, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xec69ff2b, 0xa8dbbd87, 0xba01ba7f, 0x353e8a18, 0xde7de042, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x520264b4, 0xd37d75a6, 0xfb1861be, 0x3cf3fb13, 0x6f664e0c, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x6d46dd3b, 0xd53584d6, 0x07ad6d17, 0x198e5600, 0x96745fad, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x2e6b75df, 0x9f3c8568, 0x5192b7ed, 0x13185a66, 0x8931fced, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x843e60e6, 0x41a16328, 0x6a9ef4e3, 0x5834b46d, 0x56699c77, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xde82ade3, 0x05170bd7, 0xf1e4cc92, 0xc88c2dc4, 0xb5b405c0, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x8e0d9e9f, 0x1972401c, 0xb7b1d22a, 0x24d940cd, 0xa90e1d4f, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x2685dbb9, 0xc65c2aad, 0x8c41552c, 0xa55a155d, 0x5bc1e06a, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x8471b916, 0xa95a1fd9, 0x656d571a, 0xd0eb1405, 0x35fd9067, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xe4af2e23, 0x71972538, 0xadaf7893, 0x8eaa99f9, 0xc11f88b4, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x22c3d1df, 0x7a86cc81, 0x191b2895, 0x36e73a69, 0x110f2581, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xc6bebd78, 0xf182058f, 0x822cbba0, 0xd9adc903, 0xef26a187, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xb350699f, 0x73b3b224, 0xcf6a0328, 0x39df089b, 0x29b6b49b, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x376ed8f8, 0xab33b198, 0x4013c7bb, 0x4cf96eb7, 0x6fd83ed5, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x6a9083c4, 0xc393aae2, 0xe8cc521d, 0x53f43d6e, 0x9976f321, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x19731bd1, 0xb54a555c, 0x67873acc, 0x513d72de, 0x978823c4, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x31b980ca, 0x6519e3de, 0x92dea640, 0xcaa2f378, 0x46b7032f, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x27fb61d5, 0x4dc9c060, 0xc1c25cd1, 0x9c115298, 0x9af559ef, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x4348b1be, 0x4b972ff6, 0xbb66a044, 0xc495a96a, 0xe400df68, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x70350b22, 0x5b34149a, 0x042e7ea8, 0x96b42630, 0x9e72bb9d, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x3034b192, 0xe1ae9718, 0xd2236b20, 0xc296bf9f, 0xa51c4dc9, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x3140c4c9, 0x6382a22f, 0xb847573b, 0x9f3f2d6e, 0xd0af24d4, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xf89e6cfc, 0xf63aee66, 0xdc330fae, 0x2d306ff1, 0xcb7a71cc, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x8ef297e0, 0xd1fe7cec, 0x5545cf2f, 0xffc64abf, 0xfd8c54da, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xac8a7258, 0xd42d5fab, 0x9a4695b7, 0xf8466c36, 0x7e23f921, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x20c7f1c2, 0x4ba405d6, 0x05350ec4, 0x50785b18, 0x8d80cf9b, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x6d4d9e84, 0xafea8882, 0x3442194a, 0x44317378, 0x842f7872, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x87c82564, 0x173db52d, 0x8f2d897f, 0xd9d04f25, 0x68c4f8cd, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xea91e0da, 0xfc6a7926, 0x95f9fb25, 0xf1d7428f, 0x76c04ef0, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xdfdf6c59, 0xe6d5419f, 0x69dd054c, 0xf852f040, 0x5dae6094, 0x00000000, 0x00000000,
      0x00000000 },
    { 0xbbb17d13, 0xdcf07989, 0x1aece58e, 0x5bc94c0f, 0xb52d6737, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x2327ff4c, 0x61319887, 0x653ef8aa, 0x91fb3743, 0x17ed74ac, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x9706e5a1, 0x1736faef, 0x2ecb86ee, 0x2fe498ab, 0x205abe70, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x410f490c, 0x524e5f3a, 0xfaf81111, 0x02e23556, 0xa7769d4b, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x68e00b2c, 0x0b7558e7, 0x48fbd849, 0xf2364df5, 0x5b6d1a0f, 0x00000000, 0x00000000,
      0x00000000 },
    { 0x506250d5, 0x84b3cca9, 0x3769d024, 0x76528440, 0x783b41b6, 0x00000000, 0x00000000,
      0x00000000 }
};
#endif //PRECOMPUT
static const DIGIT local_inv2_p160r1[SECP160R1_WORDSIZE] = { 0xc0000000, 0xffffffff, 0xffffffff,
                                                             0xffffffff, 0x7fffffff, 0x00000000,
                                                             0x00000000, 0x00000000 };
static const DIGIT local_xg_p160r1[SECP160R1_WORDSIZE] = { 0x13cbfc82, 0x68c38bb9, 0x46646989,
                                                           0x8ef57328, 0x4a96b568, 0x00000000,
                                                           0x00000000, 0x00000000 };
static const DIGIT local_yg_p160r1[SECP160R1_WORDSIZE] = { 0x7ac5fb32, 0x04235137, 0x59dcc912,
                                                           0x3168947d, 0x23a62855, 0x00000000,
                                                           0x00000000, 0x00000000 };
static const DIGIT local_a_p160r1[SECP160R1_WORDSIZE] = { 0x7ffffffc, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0x00000000,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_b_p160r1[SECP160R1_WORDSIZE] = { 0xc565fa45, 0x81d4d4ad, 0x65acf89f,
                                                          0x54bd7a8b, 0x1c97befc, 0x00000000,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_p_p160r1[SECP160R1_WORDSIZE] = { 0x7fffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0x00000000,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_n_p160r1[SECP160R1_WORDSIZE] = { 0xca752257, 0xf927aed3, 0x0001f4c8,
                                                          0x00000000, 0x00000000, 0x00000001,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_p1x_p160r1[SECP160R1_WORDSIZE] = { 0x5be78f90, 0x20ebda7b, 0x0f40d239,
                                                            0xe86da218, 0x12efb435, 0x00000000,
                                                            0x00000000, 0x00000000 };
static const DIGIT local_p1y_p160r1[SECP160R1_WORDSIZE] = { 0x1849ebf3, 0x2de58ff4, 0x5808adcd,
                                                            0xe98f76ae, 0xe08ba9b4, 0x00000000,
                                                            0x00000000, 0x00000000 };
#endif //WORD32
#ifdef WORD16
static const DIGIT local_inv2_p160r1[SECP160R1_WORDSIZE] = { 0x0000, 0xc000, 0xffff, 0xffff,
                                                             0xffff, 0xffff, 0xffff, 0xffff,
                                                             0xffff, 0x7fff, 0x0000, 0x0000,
                                                             0x0000, 0x0000, 0x0000, 0x0000 };
#endif //WORD16

/** <b>ECC Curve structure variable for SEC-P160r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp160r1;
#endif //P160

#ifdef P192
#ifdef WORD32
#ifdef PRECOMPUT
static const DIGIT local_px192[48][SECP192R1_WORDSIZE] = {
    { 0x82ff1012, 0xf4ff0afd, 0x43a18800, 0x7cbf20eb, 0xb03090f6, 0x188da80e, 0x00000000,
      0x00000000 },
    { 0x3048afdf, 0xd9947373, 0xa5355bfc, 0x29005092, 0x48fbfdbd, 0xb7310b45, 0x00000000,
      0x00000000 },
    { 0x0dcb2eaa, 0xd812efff, 0x22355286, 0x1fa5302b, 0x3b61edd2, 0x9ff5229d, 0x00000000,
      0x00000000 },
    { 0xce1d0db6, 0xf1c803e9, 0xe187c91a, 0xfad7710d, 0x6a392603, 0xee355e9c, 0x00000000,
      0x00000000 },
    { 0x131406ad, 0xa6ac7b9f, 0x1fa93ae2, 0x2cba22f7, 0x8c0b10cd, 0x7d23ed57, 0x00000000,
      0x00000000 },
    { 0x20ac893b, 0xbf6cd8b8, 0xa3e36a89, 0x47789236, 0xaffee063, 0xc7580802, 0x00000000,
      0x00000000 },
    { 0xca732037, 0xf60bd792, 0x18b038fd, 0x6b80ec5b, 0x0bd263b5, 0x09b95f94, 0x00000000,
      0x00000000 },
    { 0x44093861, 0x9683b9e0, 0x1a536843, 0x65574a0c, 0x14cc1c3d, 0xf1998a35, 0x00000000,
      0x00000000 },
    { 0x84ce428f, 0xdeca4990, 0x3c134d75, 0xaa4e5b31, 0x99175374, 0xe9d58363, 0x00000000,
      0x00000000 },
    { 0x49d986be, 0x782134c3, 0xa90bd1ba, 0xe4146387, 0x26164abf, 0xd9b3e2f5, 0x00000000,
      0x00000000 },
    { 0x0d9979a9, 0xc7881e3b, 0x042cb2e2, 0x96a582ba, 0xf78a7f07, 0x4191fb96, 0x00000000,
      0x00000000 },
    { 0xba5d1b22, 0xe5da35a0, 0x74e7f4b6, 0x1b716c5b, 0x2a358eda, 0xcf81def6, 0x00000000,
      0x00000000 },
    { 0x5d7c48d8, 0xc39649c5, 0x5a927c35, 0xeb2cdfae, 0xcba671fb, 0x67e30cbd, 0x00000000,
      0x00000000 },
    { 0xca59bd50, 0xcfaea451, 0xbab866b5, 0x0d024d62, 0x6fdd31bd, 0x7fe01aa6, 0x00000000,
      0x00000000 },
    { 0x0e32b059, 0xfe75c0ea, 0x74101902, 0xe51a10db, 0xfe3784c4, 0x6e02e109, 0x00000000,
      0x00000000 },
    { 0x4228c31b, 0x7ebbdebd, 0xdba105b8, 0xdddf4fd3, 0x00673ad2, 0xf994d7cb, 0x00000000,
      0x00000000 },
    { 0x0388e2d9, 0xbae1bf02, 0x3ae1e678, 0x1e7a93b5, 0xedb5d6e9, 0x5e542b93, 0x00000000,
      0x00000000 },
    { 0xa386a9c6, 0x74930470, 0x953a4c5b, 0x92e85f81, 0xbe975f8a, 0x73cadca6, 0x00000000,
      0x00000000 },
    { 0xb2b87be3, 0xe9a045f1, 0x00914a56, 0x8bee2c1d, 0x2ae54d50, 0xbd12ebd1, 0x00000000,
      0x00000000 },
    { 0xe27e65d0, 0x32b8b3d1, 0x9fc34c40, 0x6dcf6c82, 0xaccc7076, 0xe0e10bb5, 0x00000000,
      0x00000000 },
    { 0xc5e3f559, 0x69c05c3a, 0xae78cfa1, 0xd5416d4b, 0x500dcc19, 0xf4110d1d, 0x00000000,
      0x00000000 },
    { 0xbd4ffff4, 0x5f99cb8d, 0xf34a3da6, 0x49b33356, 0x93fa78a4, 0x893db51f, 0x00000000,
      0x00000000 },
    { 0xbe00f5ae, 0x5ed4985c, 0xb9b4155f, 0x30b40d03, 0xce95aeef, 0x372fe994, 0x00000000,
      0x00000000 },
    { 0x1bec5470, 0x254e2c93, 0x66098f5d, 0x75706c67, 0xfb457f76, 0x53789c99, 0x00000000,
      0x00000000 },
    { 0xc0a1e340, 0xb19963d8, 0x80d1090b, 0x4730d4f4, 0x184ac737, 0x51a581d9, 0x00000000,
      0x00000000 },
    { 0x6e1bb384, 0x5dab3070, 0x11d1deba, 0x587757c3, 0x75e8a4ea, 0xeb139afa, 0x00000000,
      0x00000000 },
    { 0x7d5102e2, 0xe1ce7aa3, 0x0d703e65, 0x958325c1, 0xf441a8b9, 0x85cb370f, 0x00000000,
      0x00000000 },
    { 0xad26617c, 0x51c34e5c, 0xd08aac4c, 0xd4f8ae6d, 0x0bd955f1, 0x195ff941, 0x00000000,
      0x00000000 },
    { 0x2fb8ff2a, 0xd5627e77, 0x43382fcf, 0x557d20bb, 0xef8eaa48, 0xf9e1f291, 0x00000000,
      0x00000000 },
    { 0xf057bbab, 0xf3e2eaac, 0x99c99026, 0x144989d3, 0x04a9a6e9, 0x643e1845, 0x00000000,
      0x00000000 },
    { 0x7e09a7bf, 0x792fb6b7, 0xe2035aee, 0xebedd123, 0x9ceac819, 0x4af48c15, 0x00000000,
      0x00000000 },
    { 0x23c6424b, 0x05b21823, 0xb858fe90, 0xd165b0ca, 0x44ce3b9a, 0x4258dc2e, 0x00000000,
      0x00000000 },
    { 0x4b81c09c, 0x1d8601cf, 0x62a2ed79, 0xe8a735bf, 0x649bcbca, 0xf5e0f31b, 0x00000000,
      0x00000000 },
    { 0xdac10960, 0x6100c532, 0x35a44cc1, 0x1d72bbba, 0xfaf379d7, 0xf1e3d414, 0x00000000,
      0x00000000 },
    { 0x043b4a90, 0xd8d659f4, 0x80e90a17, 0x4c273ce3, 0x26d1d34e, 0xc4e5af03, 0x00000000,
      0x00000000 },
    { 0xdecbec87, 0xee4ed5d5, 0x2adf763f, 0x3f7f0447, 0x0affa5dd, 0x21c40e40, 0x00000000,
      0x00000000 },
    { 0x0c5d8f50, 0xb6eb7193, 0xb904b596, 0x1c245c02, 0x951f7513, 0x04bc1f71, 0x00000000,
      0x00000000 },
    { 0xf8049d70, 0xadbef439, 0x7fcd617d, 0xd259bca0, 0xbb4b9f7e, 0x115095a0, 0x00000000,
      0x00000000 },
    { 0x9d36c05b, 0x14319850, 0x72ac06d7, 0x3c2072c0, 0x94ab8df9, 0x8f55220b, 0x00000000,
      0x00000000 },
    { 0xd8e6f7f7, 0x333294bf, 0x2d4d87c2, 0x8d572c7c, 0xe9fab3a5, 0x044a87d6, 0x00000000,
      0x00000000 },
    { 0x6470ab57, 0xab8eac9f, 0xdb4bc046, 0x575da7c2, 0xc3bf805f, 0x1cd4eca7, 0x00000000,
      0x00000000 },
    { 0xb325c960, 0x4c809f70, 0x9b56331e, 0x5f66e407, 0xac45469f, 0x6bf88b97, 0x00000000,
      0x00000000 },
    { 0x0dd81268, 0x72056699, 0xa1a0d1ce, 0xf7a98167, 0x42165379, 0x8eb0322c, 0x00000000,
      0x00000000 },
    { 0xcbdf060c, 0x15b007ff, 0x6541f6f4, 0x3b20e563, 0x0c56d3c6, 0x920f9a3c, 0x00000000,
      0x00000000 },
    { 0xd5d13bb1, 0x793d4469, 0x4bc45596, 0x88ebc216, 0xfa9c78ee, 0x9a77925c, 0x00000000,
      0x00000000 },
    { 0xddf64a97, 0xb17b83f2, 0xb7bef219, 0x620ffeed, 0x9b58df7b, 0xeb7c1032, 0x00000000,
      0x00000000 },
    { 0x966fa680, 0xce01eb42, 0x2eacbd34, 0xda359fa0, 0x05f6511c, 0xd93af9b2, 0x00000000,
      0x00000000 },
    { 0xfc1d3a93, 0x3b76ac4b, 0xb49c1631, 0x2b9caa3b, 0x4a62a75f, 0x33626d7d, 0x00000000,
      0x00000000 }
};
static const DIGIT local_py192[48][SECP192R1_WORDSIZE] = {
    { 0x1e794811, 0x73f977a1, 0x6b24cdd5, 0x631011ed, 0xffc8da78, 0x07192b95 },
    { 0x29cf2290, 0x1845c566, 0x4546d949, 0x42d8585c, 0xdcd27c1e, 0xff9eae9e },
    { 0x026164f6, 0xffdfccfc, 0xd13b7b8c, 0x6e8bd096, 0xe844b4d1, 0xa8a8b331 },
    { 0x21d2cfda, 0xb1fa4123, 0xe93fbf85, 0x862f7135, 0xa0299f7b, 0x4f71b1c6 },
    { 0xf2525d64, 0x262a3b0b, 0xec8bec0d, 0x3e989364, 0xbca5df18, 0x8b8311dc },
    { 0xc1857578, 0xefaf2d9e, 0xa122527a, 0x1bf320f4, 0x8c317375, 0x1b14a006 },
    { 0x0aec1acd, 0x19ec9370, 0x043f87f3, 0xfbc3c982, 0x355ed561, 0x7d80d527 },
    { 0xd06c577a, 0xae654068, 0x91337355, 0x45a6b4e9, 0x31b044b1, 0xc69a45f5 },
    { 0xde99f1f0, 0x875ae41f, 0x4cfbb12f, 0x4a2eaee1, 0x327702c8, 0x2f288d46 },
    { 0xc8d1d414, 0x7be54fb9, 0x79639fe4, 0xf9b74f7d, 0xd5dd0d07, 0x3e7ae645 },
    { 0x4beda193, 0xf5306875, 0x030ebd69, 0x94e8b959, 0xccc4c67f, 0xee884d47 },
    { 0xba572d36, 0xbac71b3d, 0xf3b9cbd2, 0xd97bf620, 0xbdfb598d, 0xc720af12 },
    { 0xecbfbe7d, 0x7a83cee1, 0x06301577, 0xce32d03c, 0x5810f5c3, 0xa93549c4 },
    { 0xf51b2da1, 0x5ffbf3d8, 0xcaf3195e, 0xe4aeaa59, 0x42523cf4, 0x6217317f },
    { 0x9eb4b13b, 0xad73921c, 0x5a0b4169, 0x87913c97, 0x76d18a29, 0xcb805642 },
    { 0x282eff65, 0x8336b43e, 0x044815a8, 0x2c4bae88, 0x46800c66, 0x0212e052 },
    { 0xbf1c2b2b, 0x6d3239f4, 0x6c2b0a12, 0x34647992, 0xeeec6409, 0xf9ebb891 },
    { 0xeec70948, 0x008240a9, 0x2320739d, 0x9cef856c, 0xc671b987, 0x08d99bca },
    { 0x15349181, 0xb283a793, 0xe7ce8f40, 0xecc58faf, 0x9e7d57b3, 0xac2bd78d },
    { 0xd10dcb2b, 0xfd0ff0e6, 0xa66edee3, 0xc2df611d, 0x1279845a, 0xde1f9546 },
    { 0xe0710102, 0x33b45adc, 0x3f6038c4, 0xc75e9ca7, 0x165acab1, 0x3aaaa4f1 },
    { 0x8ddc5c18, 0x243f0927, 0xf3f1f4c9, 0x98e7f7a2, 0x76d6696d, 0x9931dd24 },
    { 0x4ebd40a7, 0xe08a724d, 0x78d5fb20, 0x04ff3443, 0x0a9971ed, 0x517c12ca },
    { 0xae620385, 0x502b9382, 0x30319e38, 0x32ee805b, 0xb9abf308, 0x4da9b7c8 },
    { 0xe69912a5, 0xecc56731, 0x2f683f16, 0x7cdfcea0, 0xe0bb9f6e, 0x5bd81ee2 },
    { 0x4a7e64bd, 0x1304f0b4, 0x6348c2e9, 0x733b4c08, 0xaa65f542, 0x24026e2a },
    { 0x4b59b603, 0xb495824f, 0x73c25219, 0xbb46e9e2, 0xc86b03f5, 0xfc068662 },
    { 0xfe0ec672, 0x2dbef9cc, 0xf087647d, 0xef51700a, 0x6e61dd41, 0xa4c9d112 },
    { 0xc85ab986, 0x10a9c62d, 0x808e1e33, 0xcb26beee, 0xd4adacf7, 0x0ae91fcb },
    { 0x8ffbb15b, 0x2b6c349e, 0x6cb3bbc8, 0x29a0d0d2, 0xf2de8241, 0xed7df7a2 },
    { 0xa287bce4, 0x1b22e942, 0xb3fcec63, 0xd0be4cdb, 0x752be821, 0xb0d449d8 },
    { 0xef38bea4, 0x90bdb734, 0x223ec757, 0xc3752d96, 0xef06f371, 0x49ebe894 },
    { 0x52279a24, 0x1139908b, 0xeafac9af, 0xff64549d, 0xfd8fab05, 0x0aa9f937 },
    { 0xbfbc0308, 0xca8a6acb, 0x40041000, 0x522e41e3, 0x6eff3c49, 0x6feda53e },
    { 0x7b4295a9, 0x41d29841, 0x500a6acb, 0x83f184dd, 0xcd2d9846, 0x3e82f0fc },
    { 0x4b2691c9, 0xd5405c0b, 0xc7c22324, 0xde0160bc, 0x9c6bf00d, 0xe6932644 },
    { 0xbe34803d, 0xa4d0916e, 0x8c21962a, 0x8bec948a, 0xfd69f8d0, 0x150096e7 },
    { 0xc37ec132, 0xe809a84f, 0xda2d243b, 0x3dcd955d, 0x7b94e440, 0xc63bfece },
    { 0xa6977ddc, 0x99f2bafd, 0xbb93ca10, 0x719d3aa1, 0x64a648f2, 0x0b3813b5 },
    { 0xca6efaa2, 0xf163bd16, 0xfc02933e, 0x01d5af79, 0x3fc6f293, 0xe7183dee },
    { 0x77b7090d, 0x9473b641, 0xab2d8df1, 0x033af8cb, 0x68b3ebdb, 0x6b21afa0 },
    { 0x5cb03cf8, 0x3c1790aa, 0x327c6224, 0xaa7f2ae9, 0xd6146193, 0x83d52c5d },
    { 0x7aa1c0a5, 0xb58f57dd, 0xb830d8d3, 0x0b7e8cc2, 0xeecccca6, 0x89f9b030 },
    { 0x0d0028d3, 0xdd7b77d0, 0x5192d915, 0xcee00dda, 0x81f6b3b6, 0xcd7902f7 },
    { 0x236e3720, 0x073b6812, 0x21ce9fae, 0x0dde6d5f, 0x4d7a237d, 0xc450746d },
    { 0x2989f0b5, 0x0d8e8c4e, 0x3b5ed862, 0x67dece59, 0xcce6e4b6, 0xfff6bcd5 },
    { 0xbb419bf9, 0x34e7a555, 0x40bfb040, 0xcd51acc8, 0x111a4548, 0xfffe73b0 },
    { 0xc20604cd, 0x2b2f5693, 0xd3023824, 0x66d49d6f, 0x9014319b, 0x7d8796e3 }
};
#endif //PRECOMPUT
static const DIGIT local_inv2_p192r1[SECP192R1_WORDSIZE] = { 0x00000000, 0x80000000, 0xffffffff,
                                                             0xffffffff, 0xffffffff, 0x7fffffff,
                                                             0x00000000, 0x00000000 };
static const DIGIT local_psquare_p192r1[] = { 0x00000001, 0x00000000, 0x00000002, 0x00000000,
                                              0x00000001, 0x00000000, 0xfffffffe, 0xffffffff,
                                              0xfffffffd, 0xffffffff, 0xffffffff, 0x0fffffff,
                                              0x00000000, 0x00000000 };
static const DIGIT local_xg_p192r1[SECP192R1_WORDSIZE] = { 0x82ff1012, 0xf4ff0afd, 0x43a18800,
                                                           0x7cbf20eb, 0xb03090f6, 0x188da80e,
                                                           0x00000000, 0x00000000 };
static const DIGIT local_yg_p192r1[SECP192R1_WORDSIZE] = { 0x1e794811, 0x73f977a1, 0x6b24cdd5,
                                                           0x631011ed, 0xffc8da78, 0x07192b95,
                                                           0x00000000, 0x00000000 };
static const DIGIT local_a_p192r1[SECP192R1_WORDSIZE] = { 0xfffffffc, 0xffffffff, 0xfffffffe,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_b_p192r1[SECP192R1_WORDSIZE] = { 0xc146b9b1, 0xfeb8deec, 0x72243049,
                                                          0x0fa7e9ab, 0xe59c80e7, 0x64210519,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_p_p192r1[SECP192R1_WORDSIZE] = { 0xffffffff, 0xffffffff, 0xfffffffe,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_n_p192r1[SECP192R1_WORDSIZE] = { 0xb4d22831, 0x146bc9b1, 0x99def836,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_p1x_p192r1[SECP192R1_WORDSIZE] = { 0xca9d10c7, 0x21264bec, 0xec063368,
                                                            0xf95e1b37, 0x1e7ab415, 0x05e228e2,
                                                            0x00000000, 0x00000000 };
static const DIGIT local_p1y_p192r1[SECP192R1_WORDSIZE] = { 0x1e4c1e3a, 0x85bb1c70, 0xd7c66b13,
                                                            0xbbc56839, 0x333fa2c0, 0x2d4d66cb,
                                                            0x00000000, 0x00000000 };
#endif //WORD32
#ifdef WORD16
static const DIGIT local_inv2_p192r1[SECP192R1_WORDSIZE] = { 0x0000, 0x0000, 0x0000, 0x8000,
                                                             0xffff, 0xffff, 0xffff, 0xffff,
                                                             0xffff, 0xffff, 0xffff, 0x7fff,
                                                             0x0000, 0x0000, 0x0000, 0x0000 };
static const DIGIT local_psquare_p192r1[] = {
    0x0001, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000,
    0x0000, 0xfffe, 0xffff, 0xffff, 0xffff, 0xfffd, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
    0x0fff, 0xffff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x000,  0x00000
};
#endif //WORD16
/** <b>ECC Curve structure variable for SEC-P192r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp192r1;
#endif //P192

#ifdef P224
#ifdef WORD16
//incorrect value
static const DIGIT local_inv2_p224r1[SECP224R1_WORDSIZE];
#endif //WORD16
#ifdef WORD32
#ifdef PRECOMPUT
static const DIGIT local_px224[56][SECP224R1_WORDSIZE] = {
    { 0x115c1d21, 0x343280d6, 0x56c21122, 0x4a03c1d3, 0x321390b9, 0x6bb4bf7f, 0xb70e0cbd,
      0x00000000 },
    { 0x90c4c66d, 0x349259f5, 0xc5cd8e85, 0x97ba8d1c, 0x04ef6799, 0x17773824, 0x0b6ec4fe,
      0x00000000 },
    { 0xe648e979, 0x48b498da, 0x6ee6853f, 0x6ec31e02, 0x4e42f35c, 0x011d4146, 0xd84dd535,
      0x00000000 },
    { 0x39c2389c, 0xf46a9672, 0xdc90f43b, 0xd55cdbc1, 0xf80e5836, 0x5d449bc4, 0xec36d9c2,
      0x00000000 },
    { 0x3b05f824, 0x88b95561, 0xda0f02a0, 0x4db1cfed, 0x3792d39a, 0x757ffb73, 0x5756dac0,
      0x00000000 },
    { 0x70795de7, 0xd38d962e, 0x2bbed764, 0x84fd36a2, 0x803833d7, 0x0073b656, 0x8d71e438,
      0x00000000 },
    { 0x266ec9e4, 0x299886df, 0x4e3c35a0, 0x9a30b93b, 0xf409d4c2, 0xa5db84d9, 0x3fc88280,
      0x00000000 },
    { 0x6dddf554, 0x2d966526, 0xd78b60ef, 0xa4179613, 0x27a34cdb, 0x6afc31ce, 0xd35ab74d,
      0x00000000 },
    { 0xc244d210, 0x337f6efd, 0x2f1eb463, 0x529c1474, 0x6a1400e8, 0x269b98c1, 0x8fe85e22,
      0x00000000 },
    { 0x0b742716, 0xe7d42ceb, 0xb08cc94f, 0x885d3ada, 0x543f2288, 0x252106aa, 0xc423e8f4,
      0x00000000 },
    { 0x11183679, 0xa27022a0, 0xdf2c1708, 0xdae38235, 0xd97bc9e3, 0x6e3ffded, 0x9b274709,
      0x00000000 },
    { 0xe262ac00, 0xcf6cafc6, 0x49759a9c, 0x3046a5ee, 0x37bca928, 0x68de0525, 0x910c7c4d,
      0x00000000 },
    { 0x05cba313, 0xf486a25f, 0xee043800, 0xb5527b72, 0x5db4e12a, 0x8677fc1f, 0x6d90d394,
      0x00000000 },
    { 0xa13f84fa, 0xfc0f2cc4, 0x19247fe9, 0x1a934c7c, 0xd4a3f61c, 0x4066bd3b, 0x1f9ebac0,
      0x00000000 },
    { 0x666ebbe9, 0x5efd9675, 0x664d40ce, 0x2a43bca7, 0x42df8d8a, 0xf99bc522, 0x1f49bbb0,
      0x00000000 },
    { 0x002d4a1a, 0x9e3677bd, 0x37576013, 0x01357689, 0x02272829, 0xea89991f, 0x582873dd,
      0x00000000 },
    { 0xff48dbc5, 0xc8ae4d94, 0x32c87a44, 0xb5c88b66, 0x2b8d081c, 0x66c72787, 0x5be5de8b,
      0x00000000 },
    { 0x13efc568, 0x0cf71648, 0xcd153bb4, 0xda5ff49c, 0x3d34bc47, 0xc590e2a3, 0xe7fef102,
      0x00000000 },
    { 0x0c6cbda2, 0x3074a622, 0x0c256b32, 0x0e42f583, 0x13ebb1c0, 0x9840d244, 0xb74596f3,
      0x00000000 },
    { 0xb5f3821f, 0x2e3336bf, 0x4de316a2, 0x3a65da51, 0xa6729fb0, 0x4b59c371, 0x98399a2d,
      0x00000000 },
    { 0x61e6767e, 0x761a0715, 0x16b97f7a, 0x766255c6, 0xb25a8c32, 0x44e9959b, 0xbecf156d,
      0x00000000 },
    { 0xadaade65, 0x3c1a494e, 0x4da77fe5, 0xec86d6da, 0x992996ab, 0x6090e3e7, 0x65c3553c,
      0x00000000 },
    { 0xf2d242b5, 0x716300e3, 0x354b54a8, 0x0c08811d, 0xb06f96f7, 0x5c2c8815, 0x0bfd9cd7,
      0x00000000 },
    { 0x006b8831, 0xe2cfa9a4, 0xa097303f, 0xdd850136, 0x8abda714, 0x8237b7f4, 0x655a4ca1,
      0x00000000 },
    { 0x8b10f373, 0xcc432ec5, 0xacbaee45, 0xce60fa44, 0xa250b93f, 0x12622e45, 0xd7936585,
      0x00000000 },
    { 0x1dc87357, 0x8b6099fd, 0x01805f91, 0xbc5c6423, 0xdbe2890a, 0xb85a8acb, 0x3c1d8608,
      0x00000000 },
    { 0x2a84ddae, 0xc2cab591, 0xf4bc1843, 0xba95e5fe, 0xf0a2c46c, 0x322a7b91, 0xe98b4738,
      0x00000000 },
    { 0xac9d8d16, 0xf48536dd, 0x75728193, 0xcd614ec3, 0x7b4ae90d, 0x90354bad, 0x78121dbf,
      0x00000000 },
    { 0x6cab26e3, 0xa0064196, 0x2991fab0, 0x3a0b91fb, 0xec27a4e1, 0x5f8ebeef, 0x0499aa8a,
      0x00000000 },
    { 0x9747992d, 0x303f5fb9, 0xe0f16cda, 0x23bba2a9, 0xe1090026, 0xa90da939, 0x6a67a1d5,
      0x00000000 },
    { 0x01416d01, 0x5efd0c73, 0xd21f7134, 0x538ba9b6, 0xd927193a, 0xf14119f9, 0xa3639db8,
      0x00000000 },
    { 0xc8816ee7, 0x5e9c7df6, 0x9842df69, 0x77624d23, 0x7b651a9d, 0x628ba6df, 0x1ad705fc,
      0x00000000 },
    { 0x0e4e0958, 0x26739928, 0xa8eb9c0b, 0x79abae5c, 0x740ea335, 0xe9a8acf0, 0x0fb6ce42,
      0x00000000 },
    { 0xb213e815, 0x3b3839a7, 0x046d14cc, 0x579dd6bc, 0x9e5cd7de, 0x9a32ea3d, 0x5ffb92d2,
      0x00000000 },
    { 0x3575a99b, 0x81efdf8d, 0xc3e002e1, 0x3e661252, 0x1a7d2e1c, 0xe761924e, 0xb4d93c2f,
      0x00000000 },
    { 0xeb2efafd, 0x90924857, 0xce412231, 0x53fcac2b, 0xdaa14455, 0x3562d58e, 0x825800fd,
      0x00000000 },
    { 0xa0f43cf7, 0x7026e333, 0x6f5ef42b, 0x99623659, 0xa5eae280, 0x0e37fe85, 0xe518694a,
      0x00000000 },
    { 0xdebb19bf, 0x0ed18744, 0xe95399a3, 0xbda0d859, 0x4cc312c6, 0x89bf80ef, 0xa8739e3a,
      0x00000000 },
    { 0xecbd84cc, 0x29848d4f, 0xa5f73fc9, 0x29ac9c0c, 0x1472888e, 0x86f02de3, 0x71cab188,
      0x00000000 },
    { 0x5b4a167d, 0x0908382d, 0x1527f975, 0x9f347664, 0x617bb599, 0xd2e02004, 0x95463d7a,
      0x00000000 },
    { 0xb0015d18, 0x76e9f54d, 0x883a988e, 0x6ae8828b, 0xd16f8e7a, 0x50fd5712, 0xcaed9fbd,
      0x00000000 },
    { 0x581605b2, 0x0ba9f0b1, 0xa3e821c2, 0xde04055a, 0xf1d424a6, 0x65ac2252, 0x06c75b24,
      0x00000000 },
    { 0x2df5312d, 0x6ee28cae, 0x61d16f5c, 0x7c4cc71b, 0xb7619a3e, 0x899b4779, 0x05c73240,
      0x00000000 },
    { 0xe77c8dc4, 0x964656b4, 0xcc090c94, 0x82b4bd9d, 0x8ade54b7, 0xb70dc531, 0x766d0696,
      0x00000000 },
    { 0x197a33bf, 0x557faaca, 0xdaae4499, 0x13a8c26c, 0x8bbdf010, 0x5020b243, 0xb2d38912,
      0x00000000 },
    { 0x54a326ea, 0x4fe9831e, 0x2068eddf, 0xbfb2caf7, 0x95061a44, 0xb6cf1f93, 0x64c62dfb,
      0x00000000 },
    { 0xe8fe8e66, 0x3ece5310, 0x8f7d81eb, 0xafd5df19, 0x33b52ad3, 0xa6a2678c, 0xbc27d01f,
      0x00000000 },
    { 0x5e378545, 0x55942a6c, 0x04057921, 0x2cb09d0a, 0x94a7f12d, 0x0d4f3d37, 0xf4cbc578,
      0x00000000 },
    { 0x4e1bfb82, 0xe8b73797, 0x55b632b8, 0x16715839, 0x347a0d7b, 0xf6752fdd, 0xff24b567,
      0x00000000 },
    { 0xf7817cb9, 0x784a7084, 0x0738ee9a, 0xc326bcab, 0xc11e11d9, 0x0f1aae3e, 0xdc0fe90e,
      0x00000000 },
    { 0x59d957d5, 0xe92e1b44, 0xefb82e8e, 0x286a1014, 0x047bd1b9, 0x220433a1, 0x05f10b73,
      0x00000000 },
    { 0xad72df31, 0x2060d920, 0x061ff2e1, 0x84f03283, 0xd9d47a17, 0x8f858379, 0xf25d4718,
      0x00000000 },
    { 0x8f7deaaf, 0x11b665a1, 0xe67a6941, 0xa0684f63, 0xe0e1edfe, 0xf18eb5a5, 0x7becb6ac,
      0x00000000 },
    { 0x93c2221b, 0xe0ca2718, 0xff4aa164, 0x35d2b3c6, 0xb1ff4eb7, 0xaca92b31, 0x8a5e2ba3,
      0x00000000 },
    { 0xa9028a1b, 0xd9140da5, 0x2e1460a9, 0x9dee40a0, 0xb83c50d0, 0xe7e3e336, 0xc169089f,
      0x00000000 },
    { 0xb1e357d2, 0x8a9026c3, 0x021e02ec, 0xd71cd7d7, 0x8cff3ea9, 0xba9c0481, 0x4c3433c1,
      0x00000000 }
};
static const DIGIT local_py224[56][SECP224R1_WORDSIZE] = {
    { 0x85007e34, 0x44d58199, 0x5a074764, 0xcd4375a0, 0x4c22dfe6, 0xb5f723fb, 0xbd376388,
      0x00000000 },
    { 0xead5b482, 0x2f665d7d, 0xf2ec720d, 0xef429221, 0x1b00e363, 0x345906b1, 0x3399d464,
      0x00000000 },
    { 0xe56a0c7f, 0xc4551099, 0x64147b5e, 0xc7882134, 0x2f9d42dd, 0x5f869bc0, 0x29f372a0,
      0x00000000 },
    { 0x5c1abdba, 0xb708fd11, 0x2edbd224, 0x35204fc6, 0x4f1fcc4c, 0x4b202342, 0xe6d06b8c,
      0x00000000 },
    { 0x8ad5ca3e, 0x9d567d67, 0x8c4a6b1c, 0x60b114fc, 0xdedbbff1, 0xa92e7587, 0xe5436b88,
      0x00000000 },
    { 0x289418ff, 0x47b800da, 0xe1865046, 0xcb7479fa, 0xc937ef44, 0x19ef4070, 0xc87c4db5,
      0x00000000 },
    { 0x772c32c5, 0x6dcc3b18, 0xbd2c1b9e, 0x60e1caa1, 0xf530bd7e, 0xc50469a5, 0xb2d5c61d,
      0x00000000 },
    { 0x22deb15e, 0xab85ccdd, 0xe5783a6a, 0x93c62137, 0x41cffd8c, 0xe90f2da1, 0x355a1830,
      0x00000000 },
    { 0x08bfe30c, 0xfe17cbae, 0x80906b35, 0xe337bc23, 0x0790028b, 0x612cf37f, 0x2461ef27,
      0x00000000 },
    { 0x4e5c3e51, 0x78c55037, 0x250e1559, 0xf2929337, 0x6e9adba6, 0x5a9d44d3, 0x5e59a8cd,
      0x00000000 },
    { 0xbe03cedb, 0x2aa7278d, 0x0a6b1aed, 0x6b83db83, 0xccf5fc9f, 0xcbd1ab87, 0x07f81b58,
      0x00000000 },
    { 0xe3ce9df6, 0x366e2a1f, 0x8d81d75b, 0xd2a5ea08, 0x57aeafd8, 0xc2693b1f, 0xfa70f7cf,
      0x00000000 },
    { 0x8275e0f8, 0xcfd9ed33, 0x31e6295c, 0xc2754ff8, 0x263eff65, 0x08d5cce9, 0xa29c529d,
      0x00000000 },
    { 0x4dc92932, 0x2e72d3d8, 0xe021799d, 0xee518f36, 0x36022f89, 0x3d86fe18, 0xc15314a9,
      0x00000000 },
    { 0x92dc9c43, 0x6229e0b8, 0x608436e6, 0x10d0ece8, 0x858f1853, 0xb8d321dc, 0x9812dd4e,
      0x00000000 },
    { 0x808af9ee, 0x6820a531, 0x8223cb6b, 0x9f43062a, 0x1f74c61b, 0x3cf39ab5, 0x256cf11c,
      0x00000000 },
    { 0x47fb64db, 0xc6f81e08, 0x9d5a5831, 0xc8e3753e, 0x658a02ae, 0xa213388c, 0xde526c0d,
      0x00000000 },
    { 0x254a1ef7, 0xbcfe7608, 0x9ef39906, 0x3892c738, 0x1fa738bc, 0xb70b99ef, 0x3582edd4,
      0x00000000 },
    { 0x4c62ca24, 0xb3e5f45f, 0xb0fc452e, 0x80604f0c, 0x85b11d23, 0x059b7946, 0xc2739304,
      0x00000000 },
    { 0x1d2c4ca4, 0xcdc0cdeb, 0x95ad3a40, 0x62b80515, 0x3d33a171, 0x9585f1d3, 0xbc86f366,
      0x00000000 },
    { 0xf854c0c3, 0xd139be06, 0xe28ceb45, 0xae5b4b81, 0x0f94449e, 0x3eef3ce4, 0x22c59db0,
      0x00000000 },
    { 0x1fb09346, 0xaffa610b, 0x540b8a4a, 0xcbabf1c6, 0x1a13ccd3, 0x18c28ac5, 0x02995b1b,
      0x00000000 },
    { 0x55fb7a6e, 0x5e6927d8, 0x2ad242d9, 0x765e0020, 0xcc90817a, 0x7b9a1040, 0x70bec414,
      0x00000000 },
    { 0xd9fb233a, 0xee14debc, 0x1b2b940d, 0xf53c79ae, 0x13250018, 0x93e0f48f, 0x9841c9ae,
      0x00000000 },
    { 0xbbcaf302, 0x69591994, 0xf9907d07, 0xd63dc82e, 0xf7dfd16c, 0x9e3ac288, 0xeaf1ecd1,
      0x00000000 },
    { 0x6c54338d, 0xf1db057c, 0xfad613eb, 0xec0b6c06, 0xd581ff44, 0x20eb009a, 0x043fda53,
      0x00000000 },
    { 0xbcf6a4a9, 0x19757862, 0x1e0357b1, 0x07df9837, 0x8dc936b1, 0x8f711b78, 0x2c9f5cf1,
      0x00000000 },
    { 0x76459803, 0xc81f34c1, 0x46b70715, 0x840c279f, 0xc1505093, 0x2b8e8383, 0xc1289a49,
      0x00000000 },
    { 0x7766af5d, 0x50751040, 0x29610d54, 0xf70684d9, 0xd77aae82, 0x338c5b81, 0x6916f6d4,
      0x00000000 },
    { 0x56e50c5b, 0x05581b68, 0xcd1626a0, 0x8263bae8, 0xadef496f, 0x7ac901b2, 0x364ab4fb,
      0x00000000 },
    { 0x14413e1c, 0xdfc495a4, 0x82e829df, 0x859dda71, 0xa5aa7762, 0xc75bf6c0, 0x1e308b96,
      0x00000000 },
    { 0xeff87242, 0xf9b4ce09, 0x642bc3d9, 0x280e8c45, 0xfbaf9108, 0x2d83a9d0, 0x6b49fa03,
      0x00000000 },
    { 0x40857c64, 0x9209dcfe, 0x50b42587, 0xa23fe434, 0xb6fe44b1, 0x2a6f3862, 0x9ed13b1b,
      0x00000000 },
    { 0x7051d25a, 0x94de69bb, 0x32f84b5d, 0x66569b4f, 0x6397aaea, 0x6a6ab49c, 0x760dcbd2,
      0x00000000 },
    { 0xe830ae92, 0xd7e4fce0, 0xe026d6a9, 0xe55e6d67, 0x031a055b, 0x2044365e, 0x5aaf9016,
      0x00000000 },
    { 0x8ea96621, 0x8d8d7914, 0x1c3dd9ed, 0x16b523a0, 0x8b219f94, 0x77daeaaf, 0xd8db0cc2,
      0x00000000 },
    { 0x7e54b39c, 0x0c95591a, 0xd11da6c4, 0xdfafa514, 0xd813a819, 0x8b85f259, 0xc923585d,
      0x00000000 },
    { 0x0cf87023, 0x0940230c, 0xa05496c0, 0xbb78d5db, 0x5937d53b, 0xb2b5976d, 0x442b29e7,
      0x00000000 },
    { 0xf9c8e420, 0x9873e393, 0xeec5214d, 0xcaefb0ea, 0x48d91b92, 0x75600057, 0x77f6c0e5,
      0x00000000 },
    { 0x416d1fe2, 0x7d0e1b00, 0x6d75f091, 0xfbb4b074, 0x83e0565c, 0x94a9374f, 0xbcd2ad6d,
      0x00000000 },
    { 0xa493f000, 0x59e0880a, 0x180b8946, 0x4591bfe0, 0x04256ba8, 0xf9294aa3, 0xde461cf8,
      0x00000000 },
    { 0x94d4ac30, 0x7d967f3d, 0x4c94ce46, 0x47dc0cd4, 0xa75922d0, 0x98a41dc5, 0xdf3d2c79,
      0x00000000 },
    { 0x82c73e3a, 0xda9f7f63, 0x5165c56b, 0xfd561861, 0x1fab2116, 0xb0839464, 0x72855882,
      0x00000000 },
    { 0x39d3f652, 0x3bd4b444, 0x4de3ba4f, 0xe5772a25, 0x503a4a3f, 0xc79bc97e, 0xabf9e8e5,
      0x00000000 },
    { 0xa45a81e0, 0x083ee96f, 0x8682ddbc, 0x58a0ef62, 0x5fc4abbc, 0x530d7521, 0xb15a11b9,
      0x00000000 },
    { 0x01586916, 0x3100304a, 0xdb306832, 0xf88d14ae, 0x118de7ec, 0xce03ed87, 0xf179611a,
      0x00000000 },
    { 0x7ac4ac28, 0xe2f1f6b5, 0xb0fc8efc, 0x088fbbfe, 0xf119b2ed, 0x0ac2e7a0, 0xa6f3a5ab,
      0x00000000 },
    { 0x24458450, 0x05574631, 0x2f7ae581, 0x1e28609c, 0x7ac93598, 0x13d648ee, 0x6bf9e3ea,
      0x00000000 },
    { 0x00e2e016, 0xeace1f14, 0x608d2097, 0x71e59b8c, 0x89375d09, 0x2b07ec89, 0xc3e28cdc,
      0x00000000 },
    { 0xa5f98390, 0x74cf639e, 0x0aa22ffb, 0x47b75c35, 0xfae98a40, 0x17fc459a, 0x956ec2d6,
      0x00000000 },
    { 0x0a0552ed, 0xfc73970b, 0xb758f17a, 0x9a485994, 0xae567be9, 0xd70b8914, 0x69e353d3,
      0x00000000 },
    { 0xd5fcd107, 0x9b073404, 0x0122910a, 0xdc6e7312, 0x751b6929, 0x06220562, 0x6fd61fb9,
      0x00000000 },
    { 0x433b367e, 0xaee9f3b6, 0x870a6d61, 0x232ccba5, 0xbcb19b33, 0x19f4645f, 0xcbe539c1,
      0x00000000 },
    { 0xefc3c545, 0x525df154, 0x3d682caf, 0x1b9883a4, 0x68f96d4c, 0x5907bee0, 0xd21d1159,
      0x00000000 },
    { 0x45f72594, 0xc5bfb3a6, 0x4f72dbc1, 0x6539c0e7, 0x5dd1edb9, 0xc479c84f, 0x3cf8b900,
      0x00000000 },
    { 0xa25052d2, 0x0a6a0689, 0x7b35a9a0, 0x34162fdd, 0xc74dece8, 0x7ef9d92b, 0x5320270b,
      0x00000000 }
};
#endif //PRECOMPUT
static const DIGIT local_inv2_p224r1[SECP224R1_WORDSIZE] = { 0x00000001, 0x00000000, 0x80000000,
                                                             0xffffffff, 0xffffffff, 0xffffffff,
                                                             0x7fffffff, 0x00000000 };
static const DIGIT local_xg_p224r1[SECP224R1_WORDSIZE] = { 0x115c1d21, 0x343280d6, 0x56c21122,
                                                           0x4a03c1d3, 0x321390b9, 0x6bb4bf7f,
                                                           0xb70e0cbd, 0x00000000 };
static const DIGIT local_yg_p224r1[SECP224R1_WORDSIZE] = { 0x85007e34, 0x44d58199, 0x5a074764,
                                                           0xcd4375a0, 0x4c22dfe6, 0xb5f723fb,
                                                           0xbd376388, 0x00000000 };
static const DIGIT local_a_p224r1[SECP224R1_WORDSIZE] = { 0xfffffffe, 0xffffffff, 0xffffffff,
                                                          0xfffffffe, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0x00000000 };
static const DIGIT local_b_p224r1[SECP224R1_WORDSIZE] = { 0x2355ffb4, 0x270b3943, 0xd7bfd8ba,
                                                          0x5044b0b7, 0xf5413256, 0x0c04b3ab,
                                                          0xb4050a85, 0x00000000 };
static const DIGIT local_p_p224r1[SECP224R1_WORDSIZE] = { 0x00000001, 0x00000000, 0x00000000,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0x00000000 };
static const DIGIT local_p1x_p224r1[SECP224R1_WORDSIZE] = { 0x93aaae5e, 0x184088f6, 0xabf313ee,
                                                            0x8740af17, 0xd8c4e89c, 0xdc801936,
                                                            0x36687d26, 0x00000000 };
static const DIGIT local_p1y_p224r1[SECP224R1_WORDSIZE] = { 0xdd41238e, 0x95fae462, 0x1ea77b80,
                                                            0x4cacb1ae, 0x2a3abc85, 0xcc570f6b,
                                                            0x2b64f550, 0x00000000 };
static const DIGIT local_n_p224r1[SECP224R1_WORDSIZE] = { 0x5c5c2a3d, 0x13dd2945, 0xe0b8f03e,
                                                          0xffff16a2, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0x00000000 };
#endif //WORD32

/** <b>ECC Curve structure variable for SEC-P224r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp224r1;
#endif //P224
#endif //ROMCODE P256

#ifdef P256
#ifdef WORD32
#ifndef ROMCODE_P256
static const DIGIT local_px25664[64][SECP256R1_WORDSIZE] = {
    { 0xd898c296, 0xf4a13945, 0x2deb33a0, 0x77037d81, 0x63a440f2, 0xf8bce6e5, 0xe12c4247,
      0x6b17d1f2 },
    { 0xe1277c6e, 0xa5eb4787, 0xff6ca038, 0xcd28392e, 0x9836315f, 0x8b821c62, 0x8a6b4185,
      0x76a94d13 },
    { 0x12d0441b, 0x2db2e35f, 0x0d6a564c, 0x190d0b71, 0x8603ed61, 0x87ffd152, 0xb0091659,
      0x34a2d4a3 },
    { 0xa245573f, 0xf077e8da, 0xf4fd0a64, 0x508676f2, 0x2e6d6bd2, 0x99778967, 0xcf069e4d,
      0xe716aed2 },
    { 0x6eade3c4, 0x03e8465c, 0xc9052a05, 0x714ab749, 0x40e586b4, 0x8e5c6433, 0x4e91e90d,
      0xa018366f },
    { 0x7d96dff1, 0x8f7dcecb, 0xaad5fcfe, 0x30e431c1, 0x70cd98d5, 0xfef6a0b5, 0x141fe54f,
      0x0ec73885 },
    { 0xe392d805, 0x3725c5dc, 0x875405d4, 0xa87a8069, 0xfaa3449a, 0xa4a54daa, 0x4c6a93d7,
      0xf8f5dccf },
    { 0x6f922dbd, 0x984a2317, 0x1f99218f, 0x1ca63116, 0x8e4d71b9, 0x3f85eaad, 0xfd4daf31,
      0x6d28b6bf },
    { 0x185a5943, 0x3a5a9e22, 0x5c65dfb6, 0x1ab91936, 0x262c71da, 0x21656b32, 0xaf22af89,
      0x7fe36b40 },
    { 0xfdde3445, 0x7aff4fe1, 0xa2aea781, 0x76bba9dd, 0xff26519e, 0x85371fe7, 0x4d7061e6,
      0x6965b638 },
    { 0x4d9aefbd, 0x258fca6c, 0x73b95775, 0x1efe47b2, 0x41bc43dc, 0x32ca9f0d, 0x8c669d76,
      0x0fbc341c },
    { 0xa3fe67b2, 0xc6e3197a, 0x0a2e3338, 0x34a8bfe7, 0xa97323b2, 0xbc666b9b, 0x773c85dc,
      0x6608c243 },
    { 0x17e55104, 0xc2ebaf80, 0xbb8e9c71, 0xf73a835f, 0x4d8b561c, 0x63de93c3, 0x27b78737,
      0xd8de7652 },
    { 0x071e5c83, 0xeea6bc92, 0x8542a0be, 0x8bd27f19, 0x2a58e5b1, 0x20a845b7, 0x5026d73f,
      0x54ccc941 },
    { 0x22d32936, 0xc30fb771, 0x32847f01, 0xe40cd02e, 0x513d7f38, 0x9f6cc7d1, 0x7814a47d,
      0xc5440c59 },
    { 0xb066fd48, 0xbf9f4172, 0xb8971583, 0x1c37101c, 0xa6badca6, 0x06c79b97, 0x4227f1c5,
      0x241c567a },
    { 0x8e14db63, 0x90e75cb4, 0xad651f7e, 0x29493baa, 0x326e25de, 0x8492592e, 0x2811aaa5,
      0x0fa822bc },
    { 0xb76a6987, 0x68f41305, 0x43da43ff, 0x2b481ab4, 0xd7ee1b3f, 0x7ddc6988, 0xa9989954,
      0x54bc18d7 },
    { 0x70aae231, 0x60aafad1, 0x79f96b7c, 0x5194b852, 0x9267756f, 0x85f24823, 0x9761e3f2,
      0x1d35c969 },
    { 0x750e4f5f, 0x5572aea8, 0xa567e0e6, 0xcbf5ca9a, 0xaa02f29a, 0x388ff0f7, 0x844b5aef,
      0x55d9a959 },
    { 0x4351964c, 0xd4d3d2de, 0x6f5412c1, 0x34692437, 0x85755c08, 0xae5abca1, 0xbe28c47f,
      0x6e29f959 },
    { 0x0cb71280, 0x28a7d411, 0xa8456311, 0xc141c259, 0x796f458e, 0xc00f2ef0, 0xb2bfeed9,
      0xff046a9e },
    { 0xc5852e50, 0xc7d0b24c, 0xcf570cc5, 0x8b2c7e63, 0x49ee7eff, 0xc1f9aa23, 0xfeabb058,
      0xe486c7df },
    { 0x2b519178, 0x84b6b8ec, 0xa43d1fc5, 0xb63b9998, 0xb5e2ee0a, 0xcfe08cf8, 0xb5e50430,
      0xf41d7f4b },
    { 0x7512218e, 0xa84aa939, 0x74ca0141, 0xe9a521b0, 0x18a2e902, 0x57880b3a, 0x12a677a6,
      0x4a5b5066 },
    { 0x008e2df0, 0x6628d837, 0x4328e5f7, 0x32c3b257, 0x8bbe0f50, 0x012c29df, 0xde2ab995,
      0x2eb3910b },
    { 0x3ecca7e0, 0xc739a5ea, 0x6743333e, 0xa7d2c98f, 0x224d9428, 0x0fef6335, 0x5c792a0c,
      0x7ef2ee3c },
    { 0xec49e853, 0x6701f090, 0x9975e04a, 0xa9cb5352, 0xeb5fa77a, 0x57802554, 0x21640aeb,
      0x0e514164 },
    { 0xe2f2b734, 0xf0699bf9, 0x5501d267, 0x79c3bb5b, 0xf1164457, 0x0634a786, 0x9eecc99a,
      0x224a0229 },
    { 0x7789b84d, 0xea6065ad, 0xd1041ce2, 0x167d5ee4, 0xb56493fe, 0xcdb5d111, 0x57374b4c,
      0x4a89a614 },
    { 0x72bd05a0, 0x3438c84a, 0x0b1fc80d, 0xba82337d, 0xf36d1f90, 0xedc2f995, 0x3a31885c,
      0xe5e89236 },
    { 0xfab5c2cf, 0x21579992, 0x1fb084de, 0x5312f0c2, 0x04fa90a4, 0x71e74741, 0x1e221f50,
      0xe4107e43 },
    { 0xd789bd85, 0x57c84fc9, 0xc297eac3, 0xfc35ff7d, 0x88c6766e, 0xfb982fd5, 0xeedb5e67,
      0x447d739b },
    { 0x137de736, 0x1ebd8bac, 0x8688ce24, 0xa2f8d561, 0x8cdec18e, 0x8937542b, 0x949ccf3e,
      0x9022e314 },
    { 0x44cd3397, 0x741145c1, 0x19515eb5, 0xdc34c106, 0xb5156ded, 0x89386cf2, 0x19eda723,
      0x73baff04 },
    { 0x1fa4e33c, 0x0e2e5fb3, 0x8b098cb3, 0x2683bec7, 0x2976fb98, 0xc974446c, 0x1a4c25bb,
      0x9cf646b9 },
    { 0x523b716d, 0x826fadc0, 0xf74e1a6b, 0x0d238966, 0x8d18df9e, 0xe8a5c793, 0x8b8ca534,
      0xf81f5be3 },
    { 0x87354b7a, 0x8ed9c7e7, 0xc3915c97, 0x022eca56, 0xe53cde1d, 0x2397f463, 0xd77e0848,
      0x85685474 },
    { 0x12378c16, 0xddc6dc1b, 0x4615622a, 0x9e360755, 0x6bf9597d, 0xb319e52d, 0xc12b3b11,
      0x1136b759 },
    { 0x943e832a, 0x9c762ef1, 0x1786df70, 0x07e50ab0, 0x2589f18e, 0x90f573a8, 0xa7c2a51a,
      0x0d2bf28b },
    { 0xf7f82f2a, 0xaee9c75d, 0x4afdf43a, 0x9e4c3587, 0x37371326, 0xf5622df4, 0x6ec73617,
      0x8a535f56 },
    { 0xe476c81d, 0x9c837f4f, 0x99e53d5e, 0xb7a3030e, 0x3da8a366, 0x9af7cd27, 0xdcb41a01,
      0xad6090df },
    { 0x93c4a205, 0xa52f2c36, 0xf510a848, 0xde1adf89, 0x75b929fc, 0xe4665570, 0x6305bbc4,
      0xb6e06c51 },
    { 0xa9aa52df, 0x3cd5f4e4, 0xb42a627f, 0x18c452b1, 0xd991ece6, 0x6dbc4189, 0x7f608bf7,
      0x45a511c9 },
    { 0x9076f57b, 0x28cf1ab9, 0xcecac607, 0x030b86e3, 0x1cf2a53f, 0xb927e350, 0x4880c79c,
      0x20e11856 },
    { 0x99273a8f, 0xdd1452e8, 0x694fd54a, 0x3fdd478e, 0xdfd7dac9, 0x0318ec7f, 0x723a027b,
      0x5c9cc4f8 },
    { 0x28523eb3, 0xe7dd9ab3, 0x556c6c4a, 0x351a3a75, 0x4f3dcde3, 0x46ac1844, 0xf8a8b5ab,
      0xbaffd4eb },
    { 0xba111101, 0x835208d8, 0x76db3c5f, 0x479f0b1a, 0x257e4c96, 0xed22a34d, 0x8fced766,
      0x06ad2a09 },
    { 0x313728be, 0x6cf20ffb, 0xa3c6b94a, 0x96439591, 0x44315fc5, 0x2736ff83, 0xa7849276,
      0xa6d39677 },
    { 0x4514c6fd, 0x6eef62ff, 0x0dab013e, 0xfd683b4d, 0x10ff5936, 0x33d4db57, 0x089cf4e0,
      0xac25da80 },
    { 0x2fd20bae, 0x55d4e37a, 0xd5b4a811, 0x26035672, 0x176d343a, 0x027bba12, 0xc3bcfcd9,
      0x285250ed },
    { 0x3e07e4f1, 0x171f3a2a, 0xd0dc267c, 0x1fc5147d, 0x7ef8890e, 0x818debc0, 0x2ed0366f,
      0xc472c106 },
    { 0x839bb85f, 0x320f09c3, 0xa050e62c, 0x0101fb06, 0x9ad53458, 0x557582c9, 0x1666432b,
      0x55d5398d },
    { 0x32513926, 0xc62dbc9c, 0xefec85ec, 0xcc268940, 0x4bfc3383, 0x94febab2, 0x3ca08726,
      0x88428646 },
    { 0xd4441fbb, 0x3e0b5d5a, 0xa5ed115e, 0x9eaa8520, 0xedb92555, 0x7fb60c55, 0x21de21d4,
      0xb4958c4e },
    { 0x6e12e1df, 0xac24e643, 0x7d761d04, 0xd8a7fd0b, 0xb346027c, 0x4134bf13, 0x88e932b6,
      0xd0e09196 },
    { 0xe895df07, 0x6a703f10, 0x01876bd8, 0xfd75f3fa, 0x0ce08ffe, 0xeb5b06e7, 0x2783dfee,
      0x68f6b854 },
    { 0x67ea1f34, 0xd34dab17, 0x04739e0d, 0x646bd354, 0x93b353d6, 0xc3b41d17, 0x3d474b66,
      0x7f9460a2 },
    { 0x8e6d8483, 0xcec9b4d7, 0x46967f0d, 0x82f2d7a8, 0x1b6ee945, 0x2958f49f, 0xa33d563d,
      0x9c39cb60 },
    { 0x5c38d4e4, 0x7dda97c5, 0xb1b17374, 0xe2fd65f1, 0x30181e25, 0x97566e74, 0x11229938,
      0x3d928526 },
    { 0xa44e8de3, 0x606304b1, 0x2ecc1e07, 0x5c08966a, 0x08bd1791, 0x3a5a7dcf, 0x468810b7,
      0xf50b99b7 },
    { 0x6037cfb4, 0x9e199405, 0xc3cdaa85, 0x7400f246, 0x0da0b0e0, 0x0d77f488, 0x3ff5ed0b,
      0x04253182 },
    { 0x885fbd44, 0xc0747129, 0x9f26267a, 0x89c2caa3, 0x16d5222c, 0xe05471ce, 0xd9ab5905,
      0x9bbf06da },
    { 0x444c0448, 0x1ead3233, 0xc10319e3, 0x96357686, 0x3b7893e7, 0x4943fccb, 0x2943dfa5,
      0xb12fadf5 }
};
static const DIGIT local_py25664[64][SECP256R1_WORDSIZE] = {
    { 0x37bf51f5, 0xcbb64068, 0x6b315ece, 0x2bce3357, 0x7c0f9e16, 0x8ee7eb4a, 0xfe1a7f9b,
      0x4fe342e2 },
    { 0x4b8c5110, 0x0e9ddd72, 0x0fc78baa, 0x8599a004, 0xe11e8720, 0x6cb0a1b5, 0x341f260e,
      0xa985fe61 },
    { 0x5e93b146, 0xb424e784, 0xdeddd81d, 0xe7d766b9, 0x6e777fc0, 0x2bca7104, 0x53a1e3c2,
      0xbeaaed6a },
    { 0x469218d1, 0x505fc26b, 0xfa07c9b3, 0xd13d0df2, 0x87b9a851, 0x5912b066, 0x94fc72ab,
      0x353663e6 },
    { 0x6b26e8d0, 0xf45c4202, 0x44614f37, 0xd5f7284e, 0x349d8369, 0x7c6ce578, 0x14110b16,
      0xe2bbec17 },
    { 0x7c7b552c, 0x9ec841ac, 0x87ed78aa, 0x1d5264ce, 0x58065251, 0x91acc4ef, 0x87ae875d,
      0xd6224f4e },
    { 0x6cda02fa, 0x02661951, 0xa52143ba, 0x51f3da64, 0x64613273, 0x389d3e33, 0x66d63054,
      0xe58176cf },
    { 0xa0ef3cff, 0x44abff02, 0xb84da299, 0x4c24e9f0, 0x1ccb4a1c, 0x40fbe1a6, 0x141dd2fa,
      0xaf39d905 },
    { 0x699ca101, 0xd50d152c, 0x7b8af212, 0x74b3d586, 0x07dca6f1, 0x9f09f404, 0x25b63624,
      0xe697d458 },
    { 0x18855113, 0x7cc4389e, 0x1a5f99d9, 0xf47ce605, 0x38732dbe, 0x325e4966, 0x9c482511,
      0xd1bcdae3 },
    { 0xd3e71ca0, 0xed3b6ea9, 0x200a9ff1, 0x950b069e, 0x4957d3d3, 0xe4014b1d, 0x2f360e3f,
      0xbd802263 },
    { 0xe5b47a28, 0x1923fe5c, 0x40c54308, 0xc22b159f, 0xa746e03f, 0x846f640d, 0xc521c168,
      0xa1a916be },
    { 0xe52e08cd, 0x2a02ef80, 0x1940db1b, 0xc2f73fce, 0xd1dcf924, 0x5c4c628a, 0xbe13f2d1,
      0x2fd29465 },
    { 0x140916a1, 0xcfd08ef7, 0x5d8ee496, 0x929e0bcc, 0xdad2bf22, 0x3a8f8715, 0xb4514532,
      0x1c433f45 },
    { 0x47cabfd4, 0x9a42fa37, 0x31e41e5d, 0xd8a6d286, 0x23ecf4ac, 0x2edc1e1d, 0x383e1fa7,
      0xd27ee9ba },
    { 0x92857b08, 0x7f7d2c67, 0xaea36605, 0x8ea87138, 0x043203a4, 0x9363817c, 0xd4302d4b,
      0x40a62d93 },
    { 0x5f462ee7, 0xe4112454, 0x50fe82f5, 0x34b1a650, 0xb3df188b, 0x6f4ad4bc, 0xf5dba80d,
      0xbff44ae8 },
    { 0x5b9aae49, 0xfe457cd0, 0xbb73b119, 0x52eb8ed4, 0xba73e2fd, 0x7391b851, 0x11e6eaf3,
      0x4b2c8c12 },
    { 0xccd6ac71, 0x5867063a, 0x729a66f1, 0xedf58aa4, 0x22ebf810, 0x1d660ab6, 0x2df60823,
      0xc7226cb6 },
    { 0xbca97db0, 0xc858eb76, 0x33ad1b15, 0x4c9810c6, 0xbcc8ec52, 0x766d574f, 0xa5dad203,
      0x69cb7f9a },
    { 0x563fd88f, 0x118824bd, 0x7a0bfb63, 0xef640c52, 0xc184246d, 0x5052ec6c, 0x500f32f6,
      0x34565d9f },
    { 0x5ec33919, 0x7ec3271f, 0xbd28487a, 0x012b6e6e, 0xdd4cfcc9, 0x0a01eddb, 0xc0953a17,
      0x432f55ac },
    { 0xaecf107d, 0x66ba3cad, 0x3392ebd8, 0x694463d6, 0x9f51e05a, 0x9ee88a5c, 0x5606a12e,
      0x51fd75ed },
    { 0xca6a3551, 0xa7a1665d, 0x6919e1f9, 0xab49acc3, 0xfaa82347, 0xf1046de3, 0xbd9af8d6,
      0xe6a669be },
    { 0x4c4f3840, 0x0beada7a, 0x19e26d9d, 0x626db154, 0xe1627d40, 0xc42604fb, 0xeac089f1,
      0xeb13461c },
    { 0xd274eaae, 0x0910cc4e, 0xecf65f49, 0x9540d120, 0x8da4999f, 0x77d37d34, 0x37474b3a,
      0x3f29c023 },
    { 0x552ac094, 0x302b22dd, 0xdfbd3d20, 0x81b21450, 0xd5e609db, 0xa4f67f51, 0x30acc011,
      0xafb68627 },
    { 0x76405cb2, 0x336e3d13, 0x90c9ee36, 0xf5683941, 0x3a91030d, 0x278eb4a5, 0x65905469,
      0xcf331cea },
    { 0x91ec7fdf, 0x840f5854, 0x73c7afd0, 0x07b704b6, 0x871d7fff, 0x149a08ad, 0x9b6d22b4,
      0xfa41a8d2 },
    { 0x018e3ea8, 0xef9b7d7f, 0x34711999, 0xa72e2806, 0x9fe677e6, 0x08c2ea97, 0xed480d5c,
      0x45b04e87 },
    { 0xe1a69f5d, 0xc936de2b, 0x9cd099ae, 0xca73995b, 0xe79b5c3e, 0x1301eb01, 0xda1b87d2,
      0x77439de4 },
    { 0x2b955c2b, 0xd028403f, 0xff7b2410, 0x3c7892dc, 0x741c2b32, 0x3948c668, 0xcf701c9a,
      0x1e5f11e6 },
    { 0x72e25b32, 0x0c7e33c9, 0xa7fae500, 0x3d349b95, 0x3a4aaff7, 0xe12e9d95, 0x834131ee,
      0x2d4825ab },
    { 0x44c42ecc, 0xf4ef5c08, 0x44ea7657, 0x7f49366c, 0x5226ce08, 0x66d0bb04, 0x2904a394,
      0x2fae5e4f },
    { 0x9adcb8e4, 0x41ef2a13, 0xf8261349, 0x779ef92c, 0xc18f825e, 0x6a00d1e8, 0x4977ac5f,
      0x1e97de63 },
    { 0x0e0d4563, 0x19c45e06, 0xebc16032, 0x9e205827, 0x8b821f31, 0x1e408e25, 0xc1f65a89,
      0x37b0624d },
    { 0x12632401, 0x464002f5, 0x3a878330, 0x66075850, 0x380e0328, 0x1d56d29d, 0x9c1f06df,
      0xdc7f4932 },
    { 0xd16e04c6, 0x8954402b, 0x1fd6fdbd, 0x06a891dc, 0x2a6f7287, 0xeebc913b, 0x0bf587b6,
      0x20b50eb5 },
    { 0xb3488127, 0x79f59bd0, 0xe751cce3, 0x9614aa28, 0x0fe9ecb2, 0x09f0abc6, 0x45168fdd,
      0x7dec0fcf },
    { 0x5b20d37c, 0x48263af1, 0x60551446, 0x27ec9db9, 0x94b4e7ed, 0x7087a10a, 0x13bd00ac,
      0x0cac3f43 },
    { 0x223094b7, 0xc5f9a0ac, 0x4c8c7669, 0xcde53386, 0x085a92bf, 0x37e02819, 0x68b08bd7,
      0x0455c084 },
    { 0xee6705dd, 0x818abc84, 0x4f3048df, 0x393fdc36, 0xcc5bd8a8, 0xaccd75c2, 0x48f4c91e,
      0x77b5d1dd },
    { 0x5db36d05, 0x537f4d86, 0xa82a9f87, 0x63f99419, 0xe79ed73b, 0xc5e1f126, 0x21b323f5,
      0xbdb4277c },
    { 0x125ec16c, 0x7b52bd12, 0xd22955ce, 0x5a919b27, 0xcb625ad2, 0x3fe3337f, 0x73ea9b6d,
      0x73be0ec7 },
    { 0xada7afe6, 0x8583bedb, 0x40e1b71e, 0x9fe0dc9b, 0xfb6de997, 0x31bdc3e3, 0xac437ef7,
      0xff67b352 },
    { 0x5ce80d6e, 0xaddaca69, 0xa11acab5, 0x4f003675, 0x3708af96, 0xa827589d, 0x1dfed259,
      0x84efe07a },
    { 0xd5acf387, 0x1978300c, 0xbf48b6ee, 0x24511ccf, 0x795ac09c, 0x003a8eb3, 0xdbc95e83,
      0x0fa93dc5 },
    { 0x5fb37ef4, 0xc1a7ba9d, 0xb70e89ae, 0xde429d82, 0xb9eb62a0, 0x59a4c01b, 0x58f85e35,
      0x2d5b8b1c },
    { 0xc357f5f4, 0xf2bab833, 0x2284059b, 0x824a920c, 0x2d27ecdf, 0x66b8babd, 0x9b0b8816,
      0x674f8474 },
    { 0x05830541, 0x60b53977, 0x243bf37a, 0x810983e8, 0x2a1b338c, 0x883da931, 0x5cb44c7b,
      0xebc69d98 },
    { 0x54e0ab97, 0xf9de455a, 0xf1eff323, 0x6e9b1a2c, 0x1046c171, 0x3f7284ad, 0xf4ad7ea8,
      0x7866c086 },
    { 0x86b0daa9, 0x571c683d, 0xe22b5d8a, 0x85f3b7b1, 0x84d667d8, 0x6432ab5a, 0x945f70e9,
      0x41820e01 },
    { 0x4fed936f, 0xf7f63118, 0x1833d9e1, 0xd90d6a7f, 0x8ebaa72a, 0x059c6a9e, 0x49ff8e2d,
      0x576e2290 },
    { 0xafb8a54d, 0x0ee4282a, 0xebdbc15a, 0xce0eb122, 0x65d81207, 0x69660db3, 0x01c2dd5a,
      0xf778429c },
    { 0x4add7118, 0x4c7e3c9f, 0x2d463841, 0x2db9f613, 0xc38cb7e1, 0xe5c041fb, 0x996640ef,
      0xfcd6e7bf },
    { 0x9fd28fbe, 0x6a6544db, 0x9846b8df, 0x544cc4bd, 0x91dbf867, 0xb4eb60b6, 0xfe7d1452,
      0x069c9a59 },
    { 0x78712655, 0x90c76f8a, 0xf310bf7f, 0xcf5293d2, 0xfda45028, 0xfbc8044d, 0x92e40ce6,
      0xcbe1feba },
    { 0xcd9063ec, 0x58bec2ca, 0x24e34ffc, 0xc6ff8151, 0x8f076661, 0x69f1f2bf, 0x8fb06187,
      0xd0d51677 },
    { 0x6b2c50fb, 0x94b3356f, 0x3dbe4368, 0xbbade52f, 0xbca16546, 0x03d196b7, 0xe493e510,
      0xf097bf1e },
    { 0xc0ccf0d3, 0x6faaef00, 0x197abc7f, 0xcc0ab138, 0x5c2869ea, 0x3ff05703, 0xd468eab5,
      0xd32ca0a5 },
    { 0xdb7f3588, 0x4a3f3ba6, 0x21721e85, 0xe975f18d, 0x2dedcebb, 0x8789973a, 0x55f18f0c,
      0xe2b5061e },
    { 0xd8775d23, 0x00b7325e, 0x34ec5dd4, 0xeea3e4f5, 0xc8661fac, 0x628ca4d0, 0xfae00f7e,
      0xc90e8bcc },
    { 0x7a1d3e88, 0xa3447dfa, 0x15036ed2, 0xc0d4bddc, 0x0a6f47e8, 0x6755daf3, 0x4de120a3,
      0x1bcc7fa8 },
    { 0xb369de57, 0x96575e76, 0x32fc4df2, 0x1d05e703, 0x77b2a6a7, 0xe96f8f66, 0x99b0f4ac,
      0x02e24595 }
};
#endif //ROMCODE P256

static const DIGIT local_inv2_p256r1[SECP256R1_WORDSIZE] = { 0x00000000, 0x00000000, 0x80000000,
                                                             0x00000000, 0x00000000, 0x80000000,
                                                             0x80000000, 0x7fffffff };
static const DIGIT local_psquare_p256r1[] = { 0x00000001, 0x00000000, 0x00000000, 0xfffffffe,
                                              0xffffffff, 0xffffffff, 0xfffffffe, 0x00000001,
                                              0xfffffffe, 0x00000001, 0xfffffffe, 0x00000001,
                                              0x00000001, 0xfffffffe, 0x00000002, 0xfffffffe };
static const DIGIT local_xg_p256r1[SECP256R1_WORDSIZE] = { 0xd898c296, 0xf4a13945, 0x2deb33a0,
                                                           0x77037d81, 0x63a440f2, 0xf8bce6e5,
                                                           0xe12c4247, 0x6b17d1f2 };
static const DIGIT local_yg_p256r1[SECP256R1_WORDSIZE] = { 0x37bf51f5, 0xcbb64068, 0x6b315ece,
                                                           0x2bce3357, 0x7c0f9e16, 0x8ee7eb4a,
                                                           0xfe1a7f9b, 0x4fe342e2 };
static const DIGIT local_a_p256r1[SECP256R1_WORDSIZE] = { 0xfffffffc, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0x00000000, 0x00000000,
                                                          0x00000001, 0xffffffff };
static const DIGIT local_b_p256r1[SECP256R1_WORDSIZE] = { 0x27d2604b, 0x3bce3c3e, 0xcc53b0f6,
                                                          0x651d06b0, 0x769886bc, 0xb3ebbd55,
                                                          0xaa3a93e7, 0x5ac635d8 };
static const DIGIT local_p_p256r1[SECP256R1_WORDSIZE] = { 0xffffffff, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0x00000000, 0x00000000,
                                                          0x00000001, 0xffffffff };
static const DIGIT local_n_p256r1[SECP256R1_WORDSIZE] = { 0xfc632551, 0xf3b9cac2, 0xa7179e84,
                                                          0xbce6faad, 0xffffffff, 0xffffffff,
                                                          0x00000000, 0xffffffff };
static const DIGIT local_p1x_p256r1[SECP256R1_WORDSIZE] = { 0xc420924a, 0x39912513, 0x487cab57,
                                                            0x00b60867, 0x48adde64, 0x5afb62de,
                                                            0x1e67a44b, 0x0b197a2e };
static const DIGIT local_p1y_p256r1[SECP256R1_WORDSIZE] = { 0x2efba5a0, 0x461ac4c7, 0x47404cbf,
                                                            0xf0a0ab11, 0x9839be03, 0xa990c7a2,
                                                            0x0c6bac1e, 0x5b5fc4ce };
#endif //WORD32
#ifdef WORD16
static const DIGIT local_inv2_p256r1[SECP256R1_WORDSIZE] = { 0x0000, 0x0000, 0x0000, 0x0000,
                                                             0x0000, 0x8000, 0x0000, 0x0000,
                                                             0x0000, 0x0000, 0x0000, 0x8000,
                                                             0x0000, 0x8000, 0xffff, 0x7fff };
static const DIGIT local_psquare_p256r1[] = {
    0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xfffe, 0xffff, 0xffff, 0xffff, 0xffff,
    0xffff, 0xfffe, 0xffff, 0x0001, 0x0000, 0xfffe, 0xffff, 0x0001, 0x0000, 0xfffe, 0xffff,
    0x0001, 0x0000, 0x0001, 0x0000, 0xfffe, 0xffff, 0x0002, 0x0000, 0xfffe, 0xffff
};
#endif //WORD16
/** <b>ECC Curve structure variable for SEC-P256r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp256r1;
#endif //P256

#ifndef ROMCODE_P256
#ifdef K256
#ifdef WORD32

static const DIGIT local_inv2_p256k1[SECP256K1_WORDSIZE] = { 0x7ffffe18, 0xffffffff, 0xffffffff,
                                                             0xffffffff, 0xffffffff, 0xffffffff,
                                                             0xffffffff, 0x7fffffff };
static const DIGIT local_psquare_p256k1[SECP256K1_WORDSIZE * 2] = {
    0x000e90a1, 0x000007a2, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfffff85e, 0xfffffffd, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};
static const DIGIT local_xg_p256k1[SECP256K1_WORDSIZE] = { 0x16F81798, 0x59F2815B, 0x2DCE28D9,
                                                           0x029BFCDB, 0xCE870B07, 0x55A06295,
                                                           0xF9DCBBAC, 0x79BE667E };
static const DIGIT local_yg_p256k1[SECP256K1_WORDSIZE] = { 0xFB10D4B8, 0x9C47D08F, 0xA6855419,
                                                           0xFD17B448, 0x0E1108A8, 0x5DA4FBFC,
                                                           0x26A3C465, 0x483ADA77 };
static const DIGIT local_a_p256k1[SECP256K1_WORDSIZE] = { 0x00000000, 0x00000000, 0x00000000,
                                                          0x00000000, 0x00000000, 0x00000000,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_b_p256k1[SECP256K1_WORDSIZE] = { 0x00000007, 0x00000000, 0x00000000,
                                                          0x00000000, 0x00000000, 0x00000000,
                                                          0x00000000, 0x00000000 };
static const DIGIT local_p_p256k1[SECP256K1_WORDSIZE] = { 0xFFFFFC2F, 0xFFFFFFFE, 0xFFFFFFFF,
                                                          0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                                          0xFFFFFFFF, 0xFFFFFFFF };
static const DIGIT local_n_p256k1[SECP256K1_WORDSIZE] = { 0xD0364141, 0xBFD25E8C, 0xAF48A03B,
                                                          0xBAAEDCE6, 0xFFFFFFFE, 0xFFFFFFFF,
                                                          0xFFFFFFFF, 0xFFFFFFFF };
static const DIGIT local_p1x_p256k1[SECP256R1_WORDSIZE] = { 0xeb9a9787, 0x92f76cc4, 0x59599680,
                                                            0x89bdde81, 0xbbd3788d, 0x74669716,
                                                            0xef5ba060, 0xdd3625fa };
static const DIGIT local_p1y_p256k1[SECP256R1_WORDSIZE] = { 0x39bb56bc, 0xc80972fe, 0xd77cc6a6,
                                                            0x6beb9e67, 0xfba8ce35, 0x9e25dafe,
                                                            0xadf1cf2b, 0x85e7705c };
#endif //WORD32
#ifdef WORD16
static const DIGIT local_inv2_p256k1[SECP256K1_WORDSIZE] = {};
static const DIGIT local_psquare_p256k1[] = {};
#endif //WORD16
/** <b>ECC Curve structure variable for SEC-P256k1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp256k1;
#endif //K256

#ifdef SM2P192
static const DIGIT local_xg_sm2fp192[8] = { 0xadd50bdc, 0x32220b3b, 0xc3cc315e, 0x746434eb,
                                            0x1b62eab6, 0x421debd6, 0x00000000, 0x00000000 };
static const DIGIT local_yg_sm2fp192[8] = { 0xbfa36ea1, 0xe5d7fdfc, 0x153b70c4, 0xd47349d2,
                                            0xcbb42c07, 0x0680512b, 0x00000000, 0x00000000 };
static const DIGIT local_a_sm2fp192[8] = { 0x6831d7e0, 0x2f3c848b, 0x73bbfeff, 0x2417842e,
                                           0xfa32c3fd, 0x787968b4, 0x00000000, 0x00000000 };
static const DIGIT local_p_sm2fp192[8] = { 0x5c45517d, 0x45728391, 0xbf6ff7de, 0xe8b92435,
                                           0x4c044f18, 0x8542d69e, 0x00000000, 0x00000000 };
static const DIGIT local_n_sm2fp192[8] = { 0x0485628d, 0x29772063, 0xbf6ff7dd, 0xe8b92435,
                                           0x4c044f18, 0x8542d69e, 0x00000000, 0x00000000 };

/** <b>ECC Curve structure variable for SM2-FP192</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve sm2fp192;
#endif
#ifdef SM2P256
#ifdef WORD32
static const DIGIT local_pxsm2256[64][SM2FP256_WORDSIZE] = {
    { 0x7fedd43d, 0x4c4e6c14, 0xadd50bdc, 0x32220b3b, 0xc3cc315e, 0x746434eb, 0x1b62eab6,
      0x421debd6 },
    { 0x1990f4f2, 0x841622a3, 0xf3618078, 0xed4cf0c7, 0x9dab74b6, 0xb72f3ec5, 0x7ff603f8,
      0x5dfe5c35 },
    { 0xfe67a0ef, 0x64c11313, 0xcf69962f, 0x7c3076d8, 0x837fe4a4, 0x72877284, 0x7913e42a,
      0x633ac47a },
    { 0xe701f177, 0x4d2de06b, 0x406cb282, 0x792f05a5, 0x227c0ae9, 0xc719a552, 0xc07f80f2,
      0x5a959164 },
    { 0x6f9e559c, 0x41375f83, 0x231ae664, 0xd0c73da9, 0x883fd503, 0x8e1aa6b6, 0x2ca0502b,
      0x5f7540d0 },
    { 0xf20ba5bc, 0xcd0451f1, 0xfeac2941, 0x795b3f79, 0x875f38dd, 0x89141a56, 0xb8d532bd,
      0x49830ad6 },
    { 0x453a7259, 0x4c53e90b, 0x6ae07d43, 0xfc2609be, 0xe67c4543, 0xd52abe44, 0xc22d427b,
      0x7f21b333 },
    { 0xb1354e3c, 0xab553b2a, 0x4f5a49d9, 0xf6c31147, 0x615f8fa7, 0xd5ad9b48, 0x24097b87,
      0x218209cc },
    { 0x8241cff0, 0x5fb78193, 0xefb13a14, 0x36fcf14a, 0x09616e4b, 0x777ad992, 0xe436333f,
      0x4fc73375 },
    { 0xffabef26, 0x9b0ecbda, 0xb45ef69d, 0x6e3f1fdd, 0x769d14a5, 0x938f8725, 0xffb84f34,
      0x29ca0bf7 },
    { 0xb688d50c, 0x967f0c67, 0xe8040a50, 0x30ba2426, 0x02e63f04, 0xc0938f5b, 0x6a372e67,
      0x72a87e91 },
    { 0xedd5036d, 0x16a5754f, 0xbe89be9f, 0xda337eb1, 0xb53dc42b, 0xa918671f, 0xa4d75953,
      0x1dcf5ab8 },
    { 0x22bc800d, 0xa83035d3, 0x280ce5e3, 0xceaa9d69, 0xd562b9ea, 0x7e94cf3b, 0xe5ffbdac,
      0x73e88840 },
    { 0xb8d50594, 0xe2a92e86, 0x2504a490, 0x115899f5, 0x05dbfd54, 0xa25de3ad, 0xe801260b,
      0x7952bc82 },
    { 0x43952036, 0x5468bde1, 0xd7d07da1, 0x9c7f145b, 0x33b82a02, 0xbfb5f6de, 0x69be265f,
      0x21c8128b },
    { 0x826d136b, 0xfb9fb1fe, 0x17a1e9d6, 0x6cc2e94f, 0x5f0dd4f0, 0x33c656fb, 0xcac1f09b,
      0x5a2a2d62 },
    { 0xfdf82709, 0x8af94460, 0x0d76bf24, 0x6eaa07c9, 0x962018e0, 0x6f7e221c, 0x594ff462,
      0x18b05dc4 },
    { 0x3010248b, 0x1418a3b2, 0x03aa4d76, 0x30f685c9, 0xdbc0b417, 0xa905d985, 0xc0c0441b,
      0x55d2cb76 },
    { 0x24b10bc2, 0xa8b76c2a, 0x5435c699, 0xdc376ff1, 0x1c5238e4, 0x4b44dd87, 0xe3d9515d,
      0x71800353 },
    { 0xd8c66ef7, 0x5aeb4f2a, 0x4d217cbb, 0x1d568c68, 0xebb395bc, 0x78d3e0e6, 0x3d907a5f,
      0x05dce0bb },
    { 0xa044f872, 0xae5fede2, 0x5f3fda7f, 0x34075941, 0xf3b87391, 0x6a7b8213, 0x6a0b0732,
      0x25a584c5 },
    { 0x33ab9325, 0x883de84a, 0x10fe9fd7, 0x10ea550b, 0xc2667a46, 0xc33e0eab, 0xff9b3594,
      0x80c06bf2 },
    { 0x89420ecb, 0xdcf79d5e, 0x1ff79e02, 0xa282713f, 0x4e002a48, 0x7a2f5987, 0x7a2ef889,
      0x26cc5cb2 },
    { 0xc62a73a2, 0xa14dc34c, 0x63ec8bb9, 0x75608980, 0x8df55e5e, 0x944390f9, 0x5abe215c,
      0x649aba68 },
    { 0xc5362b25, 0x191e1bff, 0xc6f5d571, 0x547e7597, 0xf067ff3c, 0x1dcbf655, 0x46cc31e3,
      0x5e8c0295 },
    { 0xf85a5ce5, 0xb68793ff, 0x68599166, 0xe7cd5721, 0x10e945e3, 0xd2f4677d, 0x60661dc4,
      0x802fbb61 },
    { 0x491fac79, 0x4e3c4b15, 0x5f909a9c, 0xdd2d15f9, 0x61ce45ee, 0x21210b82, 0x8d384176,
      0x1abafe6a },
    { 0x3cf35623, 0x7606b395, 0x9d0d74a1, 0xad1159db, 0xf8eb5662, 0xfe47b4ac, 0x51303070,
      0x17a7b880 },
    { 0x1e6db0dd, 0x3605edda, 0xde4b2954, 0x433703dc, 0x819cc09f, 0xb6a0ad95, 0x1b919e9d,
      0x7bd1f16b },
    { 0xe18fa5be, 0x85ad8602, 0x3e0ace7e, 0xc88d632c, 0xc5467e45, 0x2dc0fa8e, 0x76f79fc9,
      0x12cfbffc },
    { 0xb51d4026, 0x49b0b9a6, 0x2e163284, 0x115e1115, 0xf2d12975, 0x1b4b5555, 0xeeed4053,
      0x4131f2ae },
    { 0x06074aec, 0x22f02d8b, 0x6fcb3e78, 0x73a6408c, 0x5fa34853, 0x20677afd, 0x35a689cc,
      0x322f765e },
    { 0x517ce4a4, 0xcae1bfdb, 0xe2c0096e, 0xf5c80ba9, 0xb51e0502, 0x9915090f, 0xc1a05566,
      0x6b8a24d9 },
    { 0x5b25615a, 0x074db2d1, 0x87726d3a, 0x767b8e1d, 0x650360d8, 0x6bc3b44d, 0x447fcf5c,
      0x22aa97ee },
    { 0x05b7675b, 0x3422855c, 0x9482afaf, 0x65efe8d9, 0x58e8ab62, 0xb2180b13, 0xa3fbe986,
      0x5fec9691 },
    { 0x64cf6e38, 0x88256378, 0xb4ebc4c5, 0xf5b2bf00, 0x7dfdcadc, 0x69e2607d, 0x255da80a,
      0x23ac91db },
    { 0x6857c0c7, 0x12db0cb8, 0xa3f0ddee, 0xc2cc991f, 0xe299eeb0, 0xe024e774, 0x84e91d11,
      0x47d42cb5 },
    { 0xedef3320, 0x2b107239, 0x19334659, 0x74463cdb, 0x5b462094, 0x2ccc2385, 0x5bc818ca,
      0x5c318376 },
    { 0x73a5af81, 0x368f7112, 0x27e18605, 0x434e9ff6, 0x979b4d70, 0xa5318ec1, 0xb989cae3,
      0x2e59712a },
    { 0x41e55aa3, 0x43c59e93, 0x94c671cf, 0x2ed0178a, 0xb696e240, 0xa06a102e, 0x68cc9c83,
      0x56cebdcb },
    { 0xff499406, 0x497c866c, 0x8bb031ba, 0xbd9d0a04, 0x6a4d8c78, 0x148f754a, 0xafd9f094,
      0x7639212c },
    { 0x7dfb68b8, 0xc83aa002, 0xbf9d7107, 0xd6a2e5df, 0x2cc8879c, 0x00de16a8, 0x7d928497,
      0x6a25f2cb },
    { 0xc90dffd3, 0xd7c446ff, 0x8982afb2, 0xcb649259, 0xc253f296, 0x369b211f, 0x02c1ace3,
      0x70cb583c },
    { 0xf675b0dc, 0x11ae9c7e, 0x5d95691c, 0x5d6fe259, 0x6d6dcf72, 0x78e715d1, 0x813b01db,
      0x19770b15 },
    { 0xe3685fa1, 0x3be4b7f5, 0x431079a9, 0x26d25ef6, 0x35c732a7, 0x37f080c8, 0xc7aad48d,
      0x67ec1a7d },
    { 0x041dd1a1, 0x46a59275, 0x7197e4a6, 0x410040b4, 0xbe5c13f7, 0x0e49bd4e, 0xde90ed3a,
      0x81ab5149 },
    { 0xdd3219b1, 0x5cbcd3ae, 0xcdbaf779, 0xea33f8e1, 0x9ac614bd, 0xf0f6b6f4, 0xe1cb7533,
      0x4d7e262b },
    { 0x4d2791d5, 0x89be1f94, 0x41a456a9, 0x3f7e929e, 0x05f7688a, 0x8d321f53, 0x8d63d4b0,
      0x24952ef6 },
    { 0xfd76a98c, 0x65bea336, 0xfb8da5a6, 0x3669dd67, 0xa8d92f14, 0x9d75602d, 0x5fd8c621,
      0x3b8263c5 },
    { 0xf348e3c3, 0xb6067bc3, 0x00262cc4, 0xdf027f92, 0x514d588b, 0xba84de40, 0x6e98bcc4,
      0x48490057 },
    { 0xd6d3e93d, 0xee2e488b, 0x8b54f171, 0xcbaee79f, 0x661ca601, 0x04e51f3b, 0x9c5fdd83,
      0x1599dd90 },
    { 0x452471c3, 0x62afb1c9, 0xab40994e, 0x6f7cdad4, 0x07eaca20, 0x640ce9b2, 0x8c4d106c,
      0x2f8af39f },
    { 0xe49a2f46, 0x6627cee9, 0x5cbdbfa2, 0x0e16b730, 0x53a1ff86, 0x1d7e4c0b, 0x58de1254,
      0x10d34aee },
    { 0x76d16b4f, 0x02e18b21, 0x100ea3e5, 0xdfe81fd0, 0xda97ea6e, 0x17f029cb, 0x63e1ad7e,
      0x19d4d64c },
    { 0x430953e9, 0x8dd5815e, 0x554fdc84, 0x3da3c8c3, 0xe68ea74f, 0x9af203ad, 0x8dfd1b57,
      0x5b85af9d },
    { 0x366ee3c4, 0x8b0468f8, 0x0fb59d78, 0x10ec7f33, 0x86961b02, 0xfdcdcc48, 0xd6aa8696,
      0x6a003306 },
    { 0x24f8ab61, 0x6c471de5, 0x55b87443, 0xe80e7d2f, 0x43ea32ef, 0x3894da16, 0x321fafcb,
      0x24a69b0d },
    { 0xb857f5d3, 0xd438b997, 0xf791d4a5, 0xfb96661f, 0x0b0b70bb, 0xe07243cc, 0x9701a10b,
      0x7b5e5462 },
    { 0x5670bb57, 0xecc4131f, 0x8a1d560e, 0xd062d7a3, 0xb719117a, 0x4047f558, 0xec0a1a5b,
      0x1e6d7348 },
    { 0x739b6bf3, 0xd1e7822f, 0x184cc0b9, 0x47b21d2a, 0x67db1b89, 0xbbe9a5df, 0x491094b9,
      0x626e79bc },
    { 0x6f3b0dd8, 0x72278801, 0xf81c8ffc, 0xf0f79dc5, 0xd3a8be1b, 0x2aa353ca, 0xae66a0b4,
      0x61c7036d },
    { 0x874c1c19, 0x0f344bc8, 0xfec2efcf, 0xbd9b0cd3, 0xd4bddcf9, 0x432bea98, 0x9290bca1,
      0x4c501a37 },
    { 0x10ce4d33, 0xa03728b5, 0x43d58369, 0x0f415be3, 0x1d67eb0d, 0x9db6bc70, 0x01dbf8b0,
      0x1d1a521f },
    { 0x91e27aee, 0xb92add1a, 0xd239402c, 0x7f9496b9, 0x0ded1232, 0x036e18cc, 0x93dc5255,
      0x1d8aba5b }
};
static const DIGIT local_pysm2256[64][SM2FP256_WORDSIZE] = {
    { 0xe46e09a2, 0xa85841b9, 0xbfa36ea1, 0xe5d7fdfc, 0x153b70c4, 0xd47349d2, 0xcbb42c07,
      0x0680512b },
    { 0x44faf956, 0x168f6450, 0xb37297d2, 0x57a14faf, 0x1d9dd425, 0x848f9df6, 0xb1fa7c3e,
      0x5cb838bc },
    { 0xe8c61583, 0x4755eb8c, 0xb1bdc2e9, 0x1b97bd1b, 0xc8f30c7b, 0xb070f8d3, 0x4fb54f4a,
      0x2c1427e3 },
    { 0xd9b81093, 0x686525ae, 0x416b824f, 0x72b3d44a, 0x5368878f, 0x20f2a25d, 0xc59817b3,
      0x749e7eda },
    { 0xbf675eef, 0x036af4d5, 0x4f09e2c6, 0x74164a32, 0xf66d7094, 0x03766005, 0xb6d2dcbf,
      0x752478d0 },
    { 0x593a5986, 0xdf80f262, 0x914986e2, 0x8b437050, 0xab9acc1b, 0x24788785, 0x1fa585ad,
      0x80a227c0 },
    { 0x8871d238, 0x1629f102, 0xada14518, 0x3732cf96, 0xfeacfd7e, 0x1ee6ffa4, 0xed5f406c,
      0x67bbbec4 },
    { 0x84703a76, 0x9a6d3d7a, 0x362fcc46, 0x94bf9216, 0x452e1865, 0xa86edb59, 0x740ac01f,
      0x40786351 },
    { 0xa8fa494e, 0xc7e905ff, 0x7d6c6828, 0x58888793, 0x47200463, 0x4ea50849, 0x747b4b46,
      0x746883b5 },
    { 0x5c6537be, 0xe9486d3d, 0xd7eff0b4, 0xf3285f6d, 0x03056e9a, 0x57f25e27, 0x9f540a2c,
      0x2ef38ca6 },
    { 0xa654f75c, 0xbbead76e, 0xe9fecab0, 0x32b2f681, 0xe54dc716, 0x35a6ddc3, 0xc1336b2d,
      0x7218d2cb },
    { 0x674ef91a, 0x653c0ce7, 0xc56d0a2a, 0x5e4b2fcd, 0x69de7d9e, 0x847b93a6, 0x72a052ac,
      0x836b5720 },
    { 0x84dd4e8f, 0x2a5f7a89, 0x74c73a14, 0x9803bf64, 0x82fc920d, 0x7dd8ae01, 0xdb1dfcdd,
      0x722fa8cf },
    { 0x9125d0d9, 0x1d01d127, 0x6042ed4a, 0xa23e500e, 0x8c8d2f9e, 0x8bdbb69c, 0x310e3f25,
      0x5fa6953a },
    { 0xd3a44835, 0x6fdc4dc2, 0x80871cea, 0x763b636b, 0x44280271, 0x4c338ea6, 0x8456f80b,
      0x34539419 },
    { 0x20f7181c, 0x71677860, 0xd5532614, 0x00f45b33, 0x5621ee7c, 0xf082290c, 0x4d66fa79,
      0x7b490968 },
    { 0x4d98c1d2, 0x5823c30a, 0x0ca27048, 0xb2bcba77, 0x7fd06b9c, 0x601fb24f, 0xfe8ab3cd,
      0x58761f15 },
    { 0x68a874de, 0xa218b466, 0xf7edb156, 0x3ffbc8bc, 0xaa235d71, 0x119c67ef, 0xb44837f5,
      0x30591789 },
    { 0x5b587d68, 0x2e8f4591, 0xc5787bd0, 0x58e1c7a7, 0x28ee6739, 0xd2bb86a4, 0x1def64fc,
      0x6f59ac9d },
    { 0x20fe46c5, 0x848d3688, 0xdac1764c, 0x74ea2c0e, 0xcf3bfefe, 0xdaf76cf2, 0x0f2284a9,
      0x7ada6b4e },
    { 0xab81b12b, 0x9b29d9f9, 0x36d832ca, 0x405e22c3, 0x56e8be76, 0x53152be1, 0x8367137d,
      0x069a55f6 },
    { 0x206229db, 0xaefea46a, 0xa288b955, 0xe740c9d2, 0xe875ef70, 0xf1ce3522, 0x249478d4,
      0x22773060 },
    { 0xd79ff785, 0xc6839e9e, 0x6358ee4f, 0x87772865, 0xe3676317, 0x910ad40e, 0x2ca10a3a,
      0x26678984 },
    { 0x40dc6f10, 0x67b536c3, 0xce318f3a, 0xa47fc152, 0xefdae671, 0x02e68779, 0xbf2b118f,
      0x553d88d9 },
    { 0x58bb298e, 0x3d231928, 0x367c6f1c, 0x7f4088b0, 0x8f1cd788, 0x640fc533, 0x2485674d,
      0x249c072a },
    { 0x84931b28, 0x445893f4, 0xe6b4e176, 0x7a833901, 0xc6bd998d, 0xb7835806, 0x03fc71b5,
      0x5ef79014 },
    { 0x40c70c82, 0x2ab900d3, 0xc98f98c0, 0xe5105529, 0xa56bdf47, 0x2676be5c, 0x8394d619,
      0x49bbe0c7 },
    { 0xc42ad8de, 0x71449d1a, 0x83211cd4, 0x45abab64, 0xf1b23b6f, 0x0c2ed9c2, 0x2101e784,
      0x25b9b29a },
    { 0x133a1386, 0x0aa05013, 0x82a8fae5, 0x2aff9616, 0xbdc09fdb, 0xb22364ed, 0x0cfcfe9d,
      0x1840d4a0 },
    { 0x79326c35, 0x76c3ebf8, 0xc69b927a, 0xdc7e6383, 0xdf0e07ee, 0xe25f006c, 0xcb5d0312,
      0x494f5a5d },
    { 0xa317ccfa, 0x5d48b204, 0x087c8819, 0x599b9ecb, 0xb47c3347, 0x0f22a70a, 0x9599da33,
      0x7382f1a2 },
    { 0xecea99fd, 0x2eb26ab2, 0xbd9cf6b1, 0xc07669a4, 0xa3dfd717, 0xf54faae7, 0x2050bb31,
      0x1bfc5787 },
    { 0xed6910f8, 0xe7a0c3c1, 0x7dbcaf32, 0x8a77f685, 0x083d3d53, 0xfb9457c5, 0xff8b0ec5,
      0x4494d90d },
    { 0x753d75da, 0x7384cd09, 0x1750efe6, 0xdd7f69e6, 0x84545083, 0x1622765e, 0x13dc6c97,
      0x5f94a59e },
    { 0x6a09fcad, 0x888b4fe4, 0x4bc5353c, 0x390409c2, 0x8635c837, 0x365fac4c, 0x45c925e1,
      0x4d64150f },
    { 0x1d5b831a, 0x70413054, 0x15e2059b, 0x91cc0d99, 0xaa05c052, 0xc669498c, 0x941e5c03,
      0x516c6301 },
    { 0x4472d8e4, 0xc4ff8f7e, 0x7f6ee66c, 0xfc38eca0, 0xa2ea16a1, 0xcd1d3617, 0x6902dccc,
      0x1458edf7 },
    { 0x484c8b14, 0xc1b7f27a, 0xa010430d, 0x148be613, 0xa5f3ef06, 0x2635cefa, 0x51a9b788,
      0x3ef70133 },
    { 0x0d398be4, 0x33c0e7ce, 0x7be311cf, 0x5a704069, 0x3a855bae, 0x8d2ce70d, 0x0c1d98c2,
      0x47a3234b },
    { 0xd618d166, 0x2dd6d94d, 0xddcc11f9, 0x67ef16ac, 0x01467c6a, 0x12da711e, 0x0748cb3e,
      0x1da66a6a },
    { 0x0b6d4717, 0x33b70ab5, 0x46483b36, 0xf4b05c1b, 0xba85b72f, 0xb9571547, 0x4f1d9d18,
      0x45c57546 },
    { 0xff3112fb, 0x04c083fe, 0x5c0671c8, 0x6ee677e6, 0x990aa7ee, 0x9260863b, 0xdf7b68dc,
      0x5d536777 },
    { 0x42ce8a50, 0x8693ec12, 0x8663406f, 0xe0a05ff7, 0x86e669b3, 0x096f0229, 0x8021d614,
      0x7292e693 },
    { 0x46452487, 0x703cc39c, 0xd6965a9c, 0xac93e000, 0xd50fbc15, 0x170f099a, 0x7c72af78,
      0x4b851e06 },
    { 0xd88f6ebe, 0x9a70430a, 0x47b5b891, 0x4d394050, 0x3defad66, 0x71861da9, 0x9eb5cc42,
      0x1b04cdcf },
    { 0xe0316173, 0xeffa3d73, 0xa14a370d, 0x48232332, 0xff569df8, 0x1112634d, 0xe9b98b1f,
      0x5d2ace43 },
    { 0xacea0cf0, 0x3b20a76e, 0x4aa81a3b, 0x0afa435c, 0x5c2da121, 0xe3ca9ae3, 0x3ea60495,
      0x3521b13b },
    { 0x83f1541b, 0xeb1fa48d, 0x62c15f4e, 0x6a362902, 0x343d4654, 0x4973db5d, 0x7e54e8a6,
      0x06e38200 },
    { 0xf4662c9b, 0xebdc1ed5, 0x822679c8, 0xbb2fca39, 0x22bae083, 0x8b3fa42d, 0x2347b4db,
      0x43254f8f },
    { 0x8e6e57ec, 0xfcd9b0fb, 0xd0cda3eb, 0x41ca0e42, 0x0825ee26, 0xa19fd816, 0x8d66763b,
      0x4cefef06 },
    { 0x0f4dcc7f, 0x6d593489, 0xcd0be69e, 0x0fdbe422, 0x413bdf1e, 0xde8292fb, 0x6f89eb40,
      0x761c644f },
    { 0x4dda3914, 0x4f988a8c, 0x8a0de099, 0x3800d152, 0xfe6ff3d7, 0xb5b8b5a4, 0x3bc606e9,
      0x3d0d5f54 },
    { 0x6caff1a4, 0x2aed552f, 0x563be2a5, 0xdcaac4ea, 0x5bcd4eab, 0x61da75c9, 0x6f444e06,
      0x515659aa },
    { 0x922edc4d, 0x7fd5cb9c, 0xa6f98930, 0x70c03d4d, 0x465d35fc, 0x76e0ae4a, 0xac5576d0,
      0x24d5e793 },
    { 0xb004bb75, 0xaf6faabe, 0xcce7016b, 0xfff0a4a5, 0x512abd80, 0x5a99de46, 0x46e3ac00,
      0x170087c5 },
    { 0x47d1eb03, 0xd823ea97, 0x4cb6923d, 0xd4dec682, 0x3d326c18, 0xbd1f7ad8, 0x287708ed,
      0x201a63e5 },
    { 0xfdcdc6f7, 0x38e770d3, 0x7829c115, 0xca2d93cf, 0x7c1be1ec, 0x6d0f0de0, 0x5e7a6e90,
      0x7580ae32 },
    { 0x532e29bb, 0xb33a6701, 0xc109eda5, 0x87d6c5a4, 0xf56a7339, 0xb265c921, 0x9cb0337d,
      0x1bb36612 },
    { 0xd34aaca3, 0xdce9cacf, 0xf2027744, 0x41349adb, 0xafae1034, 0x7093ca6f, 0x552ddaee,
      0x099e620b },
    { 0x1d2a7386, 0xa7d36f8d, 0x468d439c, 0x38eae28c, 0xacc6435a, 0x3ba7bdaf, 0xd3d7ea81,
      0x62e46d63 },
    { 0x5b8edb28, 0x4d79b7cd, 0x18bb00ae, 0x0c5ac3c4, 0x469bdc08, 0x24c30ac9, 0x5fc6267c,
      0x2b4c7fb7 },
    { 0x321d3a74, 0x74bcc09d, 0x86af8334, 0x786af22e, 0xb2d7e7f6, 0xc8cc21d3, 0x5b97f896,
      0x6aac8007 },
    { 0xd4d827f1, 0x7e5afc8c, 0x4127ef11, 0x3f33f617, 0xc4a5891c, 0x70e90c42, 0x7aa5f281,
      0x3807b9c7 },
    { 0xfd94fcdf, 0x412baea2, 0x0e9e0e2c, 0x739d0731, 0xf0e7c97c, 0x68ce609c, 0x09f866a4,
      0x44a6dcc8 }
};
static const DIGIT local_inv2_sm2p256[SM2FP256_WORDSIZE] = { 0x8478efe2, 0xb9176dc5, 0xae22a8be,
                                                             0x22b941c8, 0xdfb7fbef, 0x745c921a,
                                                             0x2602278c, 0x42a16b4f };
static const DIGIT local_xg_sm2fp256[SM2FP256_WORDSIZE] = { 0x7fedd43d, 0x4c4e6c14, 0xadd50bdc,
                                                            0x32220b3b, 0xc3cc315e, 0x746434eb,
                                                            0x1b62eab6, 0x421debd6 };
static const DIGIT local_yg_sm2fp256[SM2FP256_WORDSIZE] = { 0xe46e09a2, 0xa85841b9, 0xbfa36ea1,
                                                            0xe5d7fdfc, 0x153b70c4, 0xd47349d2,
                                                            0xcbb42c07, 0x0680512b };
static const DIGIT local_a_sm2fp256[SM2FP256_WORDSIZE] = { 0x3937e498, 0xec65228b, 0x6831d7e0,
                                                           0x2f3c848b, 0x73bbfeff, 0x2417842e,
                                                           0xfa32c3fd, 0x787968b4 };
static const DIGIT local_b_sm2fp256[SM2FP256_WORDSIZE] = { 0x27c5249a, 0x6e12d1da, 0xb16ba06e,
                                                           0xf61d59a5, 0x484bfe48, 0x9cf84241,
                                                           0xb23b0c84, 0x63e4c6d3 };
static const DIGIT local_p_sm2fp256[SM2FP256_WORDSIZE] = { 0x08f1dfc3, 0x722edb8b, 0x5c45517d,
                                                           0x45728391, 0xbf6ff7de, 0xe8b92435,
                                                           0x4c044f18, 0x8542d69e };
static const DIGIT local_n_sm2fp256[SM2FP256_WORDSIZE] = { 0xc32e79b7, 0x5ae74ee7, 0x0485628d,
                                                           0x29772063, 0xbf6ff7dd, 0xe8b92435,
                                                           0x4c044f18, 0x8542d69e };
#endif //WORD32
/** <b>ECC Curve structure variable for SM2-FP256</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve sm2fp256;
#endif

//1.6.0
#ifdef BP256
#ifdef WORD32
static const DIGIT local_inv2_bp256r1[BP256R1_WORDSIZE] = { 0x8fb729bc, 0x1009a40e, 0xea931014,
                                                            0x371dfb11, 0x4ec1c6b9, 0x1f330548,
                                                            0xd0f754de, 0x54fdabed };
static const DIGIT local_xg_bp256r1[BP256R1_WORDSIZE] = { 0x9ace3262, 0x3a4453bd, 0xe3bd23c2,
                                                          0xb9de27e1, 0xfc81b7af, 0x2c4b482f,
                                                          0xcb7e57cb, 0x8bd2aeb9 };
static const DIGIT local_yg_bp256r1[BP256R1_WORDSIZE] = { 0x2f046997, 0x5c1d54c7, 0x2ded8e54,
                                                          0xc2774513, 0x14611dc9, 0x97f8461a,
                                                          0xc3dac4fd, 0x547ef835 };
static const DIGIT local_a_bp256r1[BP256R1_WORDSIZE] = { 0xf330b5d9, 0xe94a4b44, 0x26dc5c6c,
                                                         0xfb8055c1, 0x417affe7, 0xeef67530,
                                                         0xfc2c3057, 0x7d5a0975 };
static const DIGIT local_b_bp256r1[BP256R1_WORDSIZE] = { 0xff8c07b6, 0x6bccdc18, 0x5cf7e1ce,
                                                         0x95841629, 0xbbd77cbf, 0xf330b5d9,
                                                         0xe94a4b44, 0x26dc5c6c };
static const DIGIT local_p_bp256r1[BP256R1_WORDSIZE] = { 0x1f6e5377, 0x2013481d, 0xd5262028,
                                                         0x6e3bf623, 0x9d838d72, 0x3e660a90,
                                                         0xa1eea9bc, 0xa9fb57db };
static const DIGIT local_n_bp256r1[BP256R1_WORDSIZE] = { 0x974856a7, 0x901e0e82, 0xb561a6f7,
                                                         0x8c397aa3, 0x9d838d71, 0x3e660a90,
                                                         0xa1eea9bc, 0xa9fb57db };
static const DIGIT local_p1x_bp256r1[] = { 0x523cfc52, 0x5178578b, 0xfa34a8f6, 0xb54cd654,
                                           0xc405daad, 0x0e5330cb, 0x94d027d2, 0x916a3b81 };
static const DIGIT local_p1y_bp256r1[] = { 0x35d50ea9, 0xa16de4ca, 0xbd97e50c, 0x30112330,
                                           0x76a61643, 0x02898f7e, 0xa1e2aaf6, 0x058dd741 };

#endif //WORD32

/** <b>ECC Curve structure variable for Brainpool P256r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp256r1;
#endif //BP256

#ifdef BP384
#ifdef WORD32
static const DIGIT local_inv2_bp384r1[BP384R1_WORDSIZE] = { 0x9883f62a, 0xc3a38009, 0xc80e8d38,
                                                            0xd669d394, 0xbfdb8891, 0x0958ed0c,
                                                            0xf6aa2b5a, 0x8a97b884, 0x287320ef,
                                                            0x07aeb7bf, 0x519c3694, 0x465c8f41 };
static const DIGIT local_xg_bp384r1[BP384R1_WORDSIZE] = { 0x47d4af1e, 0xef87b2e2, 0x36d646aa,
                                                          0xe826e034, 0x0cbd10e8, 0xdb7fcafe,
                                                          0x7ef14fe3, 0x8847a3e7, 0xb7c13f6b,
                                                          0xa2a63a81, 0x68cf45ff, 0x1d1c64f0 };
static const DIGIT local_yg_bp384r1[BP384R1_WORDSIZE] = { 0x263c5315, 0x42820341, 0x77918111,
                                                          0x0e464621, 0xf9912928, 0xe19c054f,
                                                          0xfeec5864, 0x62b70b29, 0x95cfd552,
                                                          0x5cb1eb8e, 0x20f9c2a4, 0x8abe1d75 };
static const DIGIT local_a_bp384r1[BP384R1_WORDSIZE] = { 0x22ce2826, 0x04a8c7dd, 0x503ad4eb,
                                                         0x8aa5814a, 0xba91f90f, 0x139165ef,
                                                         0x4fb22787, 0xc2bea28e, 0xce05afa0,
                                                         0x3c72080a, 0x3d8c150c, 0x7bc382c6 };
static const DIGIT local_b_bp384r1[BP384R1_WORDSIZE] = { 0xfa504c11, 0x3ab78696, 0x95dbc994,
                                                         0x7cb43902, 0x3eeb62d5, 0x2e880ea5,
                                                         0x07dcd2a6, 0x2fb77de1, 0x16f0447c,
                                                         0x8b39b554, 0x22ce2826, 0x04a8c7dd };
static const DIGIT local_p_bp384r1[BP384R1_WORDSIZE] = { 0x3107ec53, 0x87470013, 0x901d1a71,
                                                         0xacd3a729, 0x7fb71123, 0x12b1da19,
                                                         0xed5456b4, 0x152f7109, 0x50e641df,
                                                         0x0f5d6f7e, 0xa3386d28, 0x8cb91e82 };
static const DIGIT local_n_bp384r1[BP384R1_WORDSIZE] = { 0xe9046565, 0x3b883202, 0x6b7fc310,
                                                         0xcf3ab6af, 0xac0425a7, 0x1f166e6c,
                                                         0xed5456b3, 0x152f7109, 0x50e641df,
                                                         0x0f5d6f7e, 0xa3386d28, 0x8cb91e82 };
static const DIGIT local_p1x_bp384r1[] = { 0x75a63e99, 0xac10f79e, 0xb09b811a, 0x2d3695fe,
                                           0x0aa7c2a9, 0xf1d81e69, 0x4f8bc8c4, 0x6f583fe1,
                                           0xd194fab9, 0x1c630948, 0xda6be62a, 0x5ad00e2f };
static const DIGIT local_p1y_bp384r1[] = { 0x6bd7f655, 0xf919af00, 0x9983e49e, 0x7b9d90e3,
                                           0xfec72afc, 0x9f2601b2, 0x7592b7dd, 0x5e3a861a,
                                           0x94137b47, 0xec0f4f90, 0x1c5182a2, 0x33b63ca5 };
#endif //WORD32

/** <b>ECC Curve structure variable for Brainpool P384r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp384r1;
#endif

#ifdef P384
#ifdef WORD32
#ifdef PRECOMPUT
static const DIGIT local_px384[96][SECP384R1_WORDSIZE] = {
    { 0x72760ab7, 0x3a545e38, 0xbf55296c, 0x5502f25d, 0x82542a38, 0x59f741e0, 0x8ba79b98,
      0x6e1d3b62, 0xf320ad74, 0x8eb1c71e, 0xbe8b0537, 0xaa87ca22 },
    { 0x401dafcf, 0xa462b892, 0x3d157811, 0xac8ebc77, 0x28a8583b, 0xa464fa94, 0xe5152cf6,
      0x6b80dff0, 0xb231511a, 0x5fbd88e2, 0x5282369c, 0xd5d89c3b },
    { 0x76ead2e4, 0xbd5ea787, 0xead4d0dc, 0xb8e0f03c, 0xa7dffded, 0xa2193f92, 0x351bc35e,
      0xb3f2467a, 0x36c5018a, 0xdeda6f06, 0x69269255, 0x21a4f5d3 },
    { 0x25adc9ec, 0x00e81058, 0x1873b2bb, 0x8d8e5a4c, 0x5d1f591a, 0xa7a882f9, 0x6c27f447,
      0x5cc7168a, 0x3461d347, 0x679662c0, 0x91dc62be, 0xb2518139 },
    { 0x450dcda4, 0x7b5f7103, 0x210005fb, 0x991daeaf, 0xe7e9f447, 0x399186af, 0x60a1e9a4,
      0x80bf2bf9, 0x2bf6a679, 0x51735619, 0x97b96d3f, 0x13ee2f3c },
    { 0x69feeee8, 0xb0658763, 0x8d4a71dd, 0xb1fdf7de, 0x2fb21fd4, 0x69c95186, 0x99cee80c,
      0xccd9fa55, 0xc1c26343, 0x3fd704b7, 0xb4089351, 0xf910ebfb },
    { 0xbb5ef8a6, 0x25043ffb, 0xccd47590, 0xe2193c62, 0x60f5afd2, 0xc96307a5, 0xf776ee76,
      0x36a26654, 0xaa19b096, 0xaaaa4f9f, 0x7f00128d, 0x3e9c2dd2 },
    { 0x5712ffa2, 0x18a993cc, 0xd6cff1e3, 0xf6785deb, 0x2bc686a7, 0x3048566f, 0x830a0d97,
      0x5f8dfd4a, 0x82199d69, 0x98dc64ea, 0x319c8898, 0x995b5b2a },
    { 0x75ef7ece, 0x1ac3b9d6, 0x84439ac8, 0x45346c2d, 0xee89fb8a, 0x61a16216, 0x0e29eed1,
      0xa5be804e, 0x51d258a2, 0x6272beeb, 0xad62b895, 0xba2213c7 },
    { 0xe2d987d4, 0x3f4afca6, 0x9bcf5278, 0xd92988f9, 0x2ae8f15e, 0x0b81f5fb, 0x2ef0ef36,
      0xed0f1774, 0xb58df192, 0xde81c101, 0xebba96c8, 0x81edb994 },
    { 0xf72584e0, 0xee588514, 0x019aca87, 0x7d3bd20f, 0xea7a5c79, 0xc78f7b96, 0xd2a5cf08,
      0x941d8442, 0x60aad0f1, 0x8d84f3be, 0x20ad8c82, 0xf94dfea6 },
    { 0x3c8769ed, 0x14e2f301, 0x6cde423d, 0x3774d66a, 0xf9b585bc, 0xaa513701, 0xc19636f5,
      0x893c8008, 0xa9209aad, 0x40964ab2, 0xf8e1e90a, 0x038bf01e },
    { 0xf47168ac, 0x9112e340, 0x90080705, 0x3917d3a0, 0xcc7fdedf, 0xb6971da1, 0x8512e48a,
      0xcbd8aa25, 0xa7f6297e, 0x804100ca, 0x433eb6a3, 0xf19c3f9b },
    { 0xced7e5ce, 0xb5004b5d, 0x3706e5ac, 0x6073bb16, 0xc733aa38, 0x4fa63884, 0xecb75383,
      0x72c8dab9, 0x874cd0d4, 0xeb3468e9, 0xbee33dfc, 0xdae976e4 },
    { 0x9c901612, 0x788075a9, 0xeb3b8adb, 0x9e1f87fc, 0x7ffa2531, 0xa13dbaf4, 0xe67355a1,
      0xf55bf77d, 0x587090cd, 0x5ff9b4a3, 0x5fc581a0, 0x1c968081 },
    { 0xcd485d1c, 0x77f32039, 0xe77ef3f9, 0x0af10bfa, 0xb5c0a09f, 0x881ed562, 0x2316a602,
      0x38c088ac, 0xc0919df5, 0xb697e0ff, 0x6bac8b1f, 0x56a80dae },
    { 0xd74d9642, 0x06ba5918, 0xdfa0f56d, 0x92dd9cde, 0x8cbae3ef, 0x74c1ac3d, 0x5f6f39bf,
      0x2fce93bf, 0xe2cfe67c, 0xff348797, 0x510cc718, 0x079dc566 },
    { 0x1d845828, 0x7a605cbf, 0x6b74dd32, 0x9d338308, 0x250b7a72, 0xb00cd47a, 0x964452bd,
      0xa5a5f852, 0x7a9d39ed, 0x1e39d6df, 0x3a976ac7, 0x89ac606a },
    { 0x9211fa73, 0xac45111a, 0x06a39bd4, 0xce690ad3, 0xdaf57c5e, 0x1cf6877d, 0x5831213c,
      0x7707a815, 0xb658221b, 0x85c3003b, 0xe0344c2a, 0x59de5a56 },
    { 0xbbc7e72a, 0xeba8e784, 0xefe97e5b, 0xec1d62bc, 0x417fb221, 0xbcc4322e, 0x54de88af,
      0xaba96295, 0xc903dc6a, 0xbb4003bd, 0xd084b7cb, 0x4ec12d78 },
    { 0x08d7e82f, 0x0762fe2f, 0x8591e9ec, 0x950f4481, 0x1cc0d4b1, 0xb4ef28cf, 0x3e1774a6,
      0x7a9336d4, 0x87ec952d, 0x7aa7ae12, 0xf8794748, 0xb31975d3 },
    { 0x9a87bddb, 0xe7658a42, 0x749c85ed, 0xddbeef12, 0x8cc5b531, 0xf94c76cd, 0x195b04d3,
      0xf1f8e726, 0xbec4a862, 0x7430740e, 0x7bcc96f5, 0xf84e292b },
    { 0x40a566e4, 0x1ec88356, 0x52f856bc, 0x471c2fc8, 0x263796fd, 0xa8c8b378, 0xab0f109f,
      0x8e3ba3e5, 0x70f1fa00, 0x57bf0a23, 0xedeeb698, 0xf4095b6f },
    { 0x843354fa, 0x4a022566, 0x6d03b7b6, 0x19b11f29, 0xcd1c65ed, 0xe6d86f21, 0x39bfb9cc,
      0x846a727b, 0x69420b94, 0x79cb7bf1, 0x234edf6d, 0x9d755d13 },
    { 0xd8ee21c9, 0x39c1b328, 0x558717db, 0x2c3e0c91, 0x3f8686a9, 0x4b58808b, 0x18141b1a,
      0x43603909, 0x37ca7abc, 0xd6e98b0d, 0x060cbd1b, 0xf532389a },
    { 0x364f3d9b, 0x9703b2f6, 0x72dcbd76, 0xffe30759, 0x54a63592, 0x0a8ce940, 0xda220c9b,
      0x422454f0, 0xc1bd4c63, 0x7dbe997e, 0x5d6bd8af, 0xa3f97376 },
    { 0xde151060, 0x22fed8dd, 0x0add81d5, 0xb68661b3, 0xa86ca648, 0xc867147e, 0x6d9fa8d0,
      0x8ea6e63c, 0x71fdb423, 0x3caec845, 0x2971b4d6, 0x1b860787 },
    { 0xf5324c87, 0xc9b75bfd, 0x3eeed861, 0x0a3e3a18, 0x43b8def6, 0x9c12afd2, 0x05243e7e,
      0xc4450f94, 0x59f0736d, 0x74c69688, 0x4ec7704a, 0x2e4e9ade },
    { 0x268106ab, 0xe54c3c1c, 0x5251d5e5, 0xd0557c60, 0xe9236f3f, 0x17ff8f7c, 0xa07bed43,
      0x6a163adf, 0xa3ea9008, 0x3bdfaa52, 0x72bff246, 0xa7fc27f2 },
    { 0x5ca8fad8, 0x8d12fe42, 0xd10970f6, 0x0fbdf6b9, 0xc7d54f8e, 0x3efac0da, 0x081d4201,
      0x78d97625, 0xd3724ac0, 0xadbe354f, 0xa1c855e6, 0xc7706509 },
    { 0xeb02d061, 0xa4f057ae, 0x2f3bc02e, 0x05b67f0b, 0xf86e089a, 0x56aead32, 0x3031e720,
      0x36ee2eea, 0xb1b8e237, 0x5b51b855, 0x9b8bb539, 0x87389331 },
    { 0x0d71d442, 0x2505f322, 0x71076f00, 0x7164fd9b, 0x553e946b, 0xc057820f, 0x60518230,
      0xf2ecc083, 0xbf4ea967, 0xd0bdcca9, 0x608db07d, 0xd4e59198 },
    { 0x0020362e, 0x84a2ae35, 0x19bf5f49, 0xea2b1df4, 0x287c9b53, 0x2bb64e18, 0x2b7b03e9,
      0x11c2df65, 0x85d5a524, 0x23a42ea4, 0x11cd506e, 0xe10cb36d },
    { 0x37c3ccaa, 0x6a0642f8, 0x7bc1a00d, 0x8f8d46cd, 0xefc36d76, 0x5f3af201, 0x5677cfe5,
      0xce059f5b, 0x95d268aa, 0xd71ed2a6, 0xeeb92e0c, 0x1a60ec14 },
    { 0x256eb547, 0xc066e749, 0xd20bc008, 0xcb207dd3, 0x91b8b552, 0x7dacd841, 0x16c50475,
      0x00088cc1, 0x369d064b, 0x5fe835db, 0x927b1863, 0x03650873 },
    { 0x1f336928, 0xdfb863bb, 0xdae4edcc, 0xa4678f28, 0xf1095811, 0x4460b514, 0x8ba4a18e,
      0x220d6528, 0x15f458ff, 0xeb5c7502, 0x9420e572, 0x7d1b4733 },
    { 0x2805e596, 0x6f5e0f5d, 0x26281050, 0x89bba01d, 0xee0d124b, 0x712e1253, 0xbadc49fb,
      0xd214b583, 0xaaf0c000, 0x049f9294, 0x6ac2c3ef, 0x4e5a9dfe },
    { 0x37d6c1b7, 0x26063118, 0xf99591e8, 0x6176a0ef, 0x6ada2064, 0x8c6750b9, 0x3eba3308,
      0xf5035e9a, 0x921335ed, 0x22046f04, 0x06508f45, 0x4f769c49 },
    { 0x9a0b4196, 0x686fba63, 0xb4a737ec, 0x07b4a582, 0x25efe643, 0x6530679b, 0x5e188d49,
      0x4df54bca, 0x0936e379, 0x8c78145d, 0x58d44250, 0x8b31d353 },
    { 0x91f22d45, 0x98163d17, 0xc56e9558, 0xdb56b796, 0x6d0f4d11, 0x84d52975, 0xd2000d91,
      0x0e7c0d8f, 0xe0e16028, 0xdd83eadf, 0xeccf825f, 0xd15fdfb4 },
    { 0x8456b4da, 0x8568960c, 0x67fec6ab, 0x511af0d3, 0x9caf93b8, 0x054cd79b, 0x77d8d02f,
      0x3d84e21e, 0x846e6f59, 0x5f9507f0, 0x1d798ddd, 0x778bfe30 },
    { 0x4ec91657, 0x7aa29366, 0x09dd1ca2, 0x59bbaa87, 0x8172145c, 0xa02d4113, 0xf2ddd3e9,
      0xb56aa4a4, 0x9d07fddd, 0x356556fe, 0x86fea1b0, 0x137d0872 },
    { 0x057a5682, 0xdb22f743, 0x51f721ae, 0x212dcf2c, 0xc951c5d9, 0x66b6274c, 0x182a49cc,
      0xc05cfe04, 0xffd6582c, 0xb031da3e, 0x7ebd5f3c, 0xb3d99da9 },
    { 0xdcf5b44d, 0xf5e6c882, 0x36c2daf9, 0x0ad397a7, 0xc4cc9ae9, 0x1f1652fe, 0x97f2913e,
      0xe60fda50, 0xb36de182, 0xb735f7b4, 0x401f5436, 0x5916fdca },
    { 0x839484bc, 0x63416ddf, 0x4917a76e, 0xf8e673e6, 0x687bb208, 0x3692169f, 0x2a8a454a,
      0x262fd932, 0x7991a2b7, 0xd91c12c5, 0x2b9046b5, 0xad0cad66 },
    { 0xad3ab043, 0x064b0f77, 0x5742664b, 0xe002ff5a, 0x087fecfd, 0xf83f8cb0, 0x8b1122c6,
      0xe1cb78ca, 0x2fd30fee, 0x8d7d7600, 0xf3cdbdd4, 0xdd22a800 },
    { 0x7c84ad05, 0x64809cc7, 0xfa12106d, 0x98ade2d9, 0x370da1ba, 0x2dc25204, 0x35be0e7c,
      0xca3aa1c9, 0x1df710de, 0x7bd4d112, 0x87db5c07, 0x4fa9b824 },
    { 0xf055ffea, 0x64fac04c, 0x1bfefcc4, 0x205fb93a, 0xb4ba5e68, 0x8dbefaee, 0xf8f9b6df,
      0x933a8bea, 0x575d2139, 0x35ff277c, 0x1d5dc732, 0x851ea5d6 },
    { 0xaa03bd53, 0xa628b09a, 0xa4f52d78, 0xba065458, 0x4d10ddea, 0xdb298789, 0x8a3e297d,
      0xb42a31af, 0x06421279, 0x40f7f9e7, 0x800119c4, 0xc19e0b4c },
    { 0xbc7c0795, 0xcc794d81, 0xbb66b55d, 0x3ef70872, 0x04ff152b, 0xd7b9e340, 0x87164fe9,
      0x16543ced, 0x871a332f, 0x96880dfc, 0xa9e60062, 0x96b2eecb },
    { 0x6c91ad26, 0x794a459a, 0x371da833, 0x6b258ca2, 0x3933f3e9, 0x9ef81528, 0x0df0fafe,
      0x49baf296, 0x01f2318d, 0xf9bc786c, 0x127f8c4f, 0x3dc56a32 },
    { 0x1b635020, 0xf68a33e6, 0x1c6e73da, 0x66e39e7b, 0x2d8d8378, 0x18b1c2e0, 0xd51abd7c,
      0x14b81ed8, 0x400435fa, 0x1940113c, 0x462109a0, 0xc57f7195 },
    { 0xb6ca9da5, 0x173d9554, 0x3b066f62, 0xa2be12ab, 0x26a92fef, 0x2746e101, 0xeb642cb1,
      0xf85098a6, 0xec50ac0d, 0x07951ffe, 0x40e8d694, 0x821c76bb },
    { 0xa22fd201, 0x0ee2798d, 0x496a4e52, 0xcf4e3a5f, 0x01887409, 0xcb78360e, 0x90f1c750,
      0xd72e7d27, 0xfc01b123, 0x6d9135fa, 0x04727dc6, 0xd7a0aafa },
    { 0xea22e778, 0x0139babf, 0xdf3cf2e1, 0xec6284ac, 0x04c9e381, 0x802e7ed8, 0x9e1e75ca,
      0xefb3591a, 0xcd470675, 0xd2f9c128, 0x5bc0b5b5, 0xf59328c8 },
    { 0xd81e43b0, 0x3f8229a8, 0x558bd431, 0xf9c58abd, 0xa1ea026a, 0x318c2bdf, 0x425e7020,
      0x058ea1e6, 0xd4ca4ce0, 0xc6e7b862, 0x16e36b9d, 0x95b1fa9e },
    { 0xc690ccc6, 0x61f040be, 0x51f9e5b6, 0xaa51cd7b, 0x81156ecf, 0x29242bde, 0x8c3782b8,
      0x31e3d90a, 0xe589ff8b, 0x2f1f10d1, 0x848b8e4e, 0x64247c77 },
    { 0xae10aa6c, 0xfcdc8ff8, 0x6390eb18, 0xb2088fe4, 0x0a65d44f, 0xcd2a1459, 0x4c430ced,
      0xcc04e2b4, 0xd2958a59, 0x94133abe, 0x262a66c5, 0x3c23dad9 },
    { 0xce6020d3, 0xe2117b3c, 0x4a2defc3, 0x5ea3d73e, 0x0fb7e135, 0x03ab6c8a, 0x353d9628,
      0x0ad21bb2, 0xc19e2e37, 0x53ae49c0, 0x991289ef, 0x347aaead },
    { 0x2e2cc4ab, 0xe816e48c, 0x09d866e4, 0xdea35767, 0x66dfa43e, 0xcd03908f, 0x167d467d,
      0xa05c151a, 0x5416b1c5, 0xe29eca36, 0x6a893ee6, 0xe9a70d3a },
    { 0xa213e83b, 0xb3de52c7, 0x55db2392, 0x464a92d0, 0xa76f70c9, 0x0f54907f, 0x455c1c82,
      0xf03811a5, 0xa71bfebc, 0xdbc082ab, 0x8edffa7f, 0xc6b40528 },
    { 0xa8492b0e, 0x0aa8984a, 0x3281b1ab, 0xa8d57ab9, 0xa774654e, 0x0880535a, 0x978aea8a,
      0xcb15d86f, 0xdaf4eed1, 0x926e0999, 0xc68c9e4f, 0x88a2cd73 },
    { 0xde68b4d8, 0x8c56d73b, 0xf933e263, 0x2ae421bf, 0xc419b34e, 0x448c5f77, 0xfede4d58,
      0x97b15d0e, 0x78b64510, 0x11a125ac, 0x030b7226, 0xb2e694d4 },
    { 0x9d1fe745, 0x393890cc, 0xa65c0ae6, 0xe803284a, 0x1eedd379, 0x2832404a, 0x24557409,
      0xa5aa0a42, 0x8428733f, 0x748e6589, 0x46399bb0, 0x47154818 },
    { 0x1a9bb0aa, 0x31efc132, 0xbb0f6ca7, 0xda74bdf2, 0xcd3d0497, 0x1af000f8, 0x2aaa3451,
      0x39acb8f2, 0xedfb898d, 0x23eaf4cd, 0xcad3d109, 0xe7b3f42d },
    { 0xaa827b07, 0x7d9a06f4, 0x7cd6596c, 0xcc3c82e4, 0x14cbf8b8, 0x26d7ffdc, 0xeb8257aa,
      0xb7b13317, 0xb78cb337, 0x8ea3df11, 0xe55ffea3, 0x131f3845 },
    { 0x5992b108, 0x0708a95b, 0x2834f982, 0x000a18d6, 0xdfae696e, 0x8b42b3bb, 0x4351358e,
      0x630527b7, 0x3721adae, 0x0f74aa5d, 0x3f5ef596, 0x5b96e58d },
    { 0x82291675, 0x9eddce1a, 0x1fbd66c3, 0xfde42b6c, 0xcc9157ff, 0xf7829803, 0x448572b6,
      0x2f260b66, 0x0f6ba10f, 0xf477b7ca, 0x671610df, 0xfab8664a },
    { 0x47f8e6e8, 0xddca4b69, 0x124c3655, 0xcfaee2ef, 0xfd59d024, 0x63239e5d, 0xbf716a7e,
      0x1272d9fd, 0x5a3f4d14, 0xe434bc3b, 0x88138c55, 0xe8b3261e },
    { 0x9ed1436f, 0x66f2906d, 0xa02cce5b, 0xbd93b0f9, 0x998709c1, 0x668531fa, 0x1eb74c1c,
      0x94f85bda, 0xae51c7b8, 0x79728904, 0xde5427a8, 0x302e9b41 },
    { 0x7742bff0, 0x3fcd7e30, 0xf1f19fd8, 0xd81be3fe, 0x7722f9a6, 0x50708b65, 0x810cbef3,
      0x8d509a8d, 0xc413c682, 0xff08f667, 0x7c111c0c, 0x1e12df49 },
    { 0x526b2679, 0x23ec0027, 0x1c91d6d0, 0xa4214685, 0x3f4783ff, 0x16c1df9e, 0xeb51fab6,
      0x8ecfdc22, 0x1fce1183, 0xdeee0f97, 0x035e6988, 0xf1d06b58 },
    { 0x4905ca71, 0xe4bfc2c0, 0xd156f761, 0xf33a450a, 0xd08848c2, 0x3d8b29db, 0xa2309686,
      0x097da395, 0x5f4972d7, 0x21190503, 0x17cbaa12, 0xb2d10558 },
    { 0x3f943745, 0xf9f5d75b, 0xc3f49b85, 0x4392035f, 0x31f666fc, 0xd4d06a53, 0x923b3473,
      0xf8b09ef0, 0xece2e50d, 0x56978ebb, 0x1ddd0f09, 0x83838b51 },
    { 0x2e0d7596, 0x91db566d, 0x84aa1e72, 0xbd5b4980, 0x83f2e4d5, 0x43c7147a, 0x7a172ca4,
      0x57d9880c, 0xb974258b, 0xde7e71e6, 0xfed337f6, 0x228c9e51 },
    { 0xf3cddf3f, 0x3cedb737, 0x43c7549c, 0x2a4b4119, 0x29199be6, 0x8d31786b, 0x29f24c09,
      0xaaac218b, 0xd9210950, 0x11fac541, 0x30cc3eda, 0x1bfd76a0 },
    { 0x044acc65, 0xbc988641, 0x48ac42b2, 0xb52dbe4d, 0xd8be8b31, 0xd23337a1, 0x02c00b31,
      0x875a82b4, 0x0e4d57ac, 0xb81682fa, 0xb31bcb79, 0xa79e73cd },
    { 0x31e76220, 0x3992d2a1, 0xff7003c6, 0x7ad2a60b, 0x8f1546be, 0xdb6a4d39, 0xe8a1a2a3,
      0xb13228b1, 0x993d3a02, 0x1470159c, 0x9e9bf4d9, 0x5b4e8b1d },
    { 0xd9a0d716, 0x09da59b0, 0x3ba58128, 0xe90f2266, 0xf30671a0, 0xe867df85, 0xaba22d8c,
      0x4bf10cfb, 0x18406ee8, 0x017faf54, 0xb97c0053, 0x4cbf6414 },
    { 0x22c317df, 0x54c51297, 0x93f78a50, 0x1b709e8f, 0x48a8029b, 0xee5c4ca0, 0x9ca255af,
      0xb55cfe22, 0xfbce159b, 0x6e9ff806, 0x01213dd7, 0xc948a002 },
    { 0x968b4a5f, 0x110162f2, 0xba0f8b3d, 0x1323feff, 0xb1666eaa, 0x89acc4c0, 0x3007a482,
      0x85dfb383, 0x5c7a9ba1, 0x5f5449e7, 0x6bfcf21e, 0x38174a00 },
    { 0xe3667565, 0x8fad0c67, 0xcaeb77c5, 0xb6809115, 0x2244896f, 0x8f1ca009, 0x7726e572,
      0x760aca36, 0x05216d7a, 0xa367d788, 0x44cff24c, 0xbe0e98a0 },
    { 0x95ddd724, 0x53dca6a6, 0x8e32f846, 0xda3cb33b, 0x33f0c368, 0xeceef531, 0xc12e5467,
      0x568c2f8b, 0x9e53d17d, 0x6feb14eb, 0xb6403d8f, 0xefc23d8b },
    { 0x752660ab, 0x65b36f1e, 0xb5600f5f, 0xae6588c1, 0x3ad0287f, 0xfdebb815, 0x5d4bf0f8,
      0xd255a09c, 0x1c1935f4, 0xa447eced, 0xd68d92e4, 0x6d76746d },
    { 0x3f385d2b, 0x35b2e438, 0x932b516d, 0x6e668ac4, 0x51a3599f, 0x8a1646fe, 0xd12c0f3a,
      0xaee309ab, 0xc46da385, 0x0231423b, 0x637ee943, 0xf23c10b4 },
    { 0xaa019503, 0x10e2488d, 0x0b19a816, 0x080c0858, 0x9d12c29b, 0xa9d44412, 0xa0dc0cf8,
      0xf1fa6864, 0x45628ecb, 0xe85aedf0, 0xabda7cd6, 0x04ddb732 },
    { 0xcc3e621c, 0x60defb1f, 0xb291f8c6, 0x3c551eaa, 0xb5053054, 0x6aa4e445, 0x2d7b0a90,
      0xdd36b95e, 0x163b4ea9, 0xf8e13784, 0xbb8f63f2, 0xe1724c60 },
    { 0xbc6510ad, 0x08aff4b2, 0x74375758, 0xe061870d, 0x8aae6004, 0x5b02734b, 0xce9622d2,
      0xe00b1374, 0x72430767, 0xbae3cd27, 0xfab48cdf, 0xab468adb },
    { 0xc3965d6c, 0x7655aeee, 0x15754e8d, 0xa205d8ff, 0xf98f3f6b, 0x2c547f20, 0x31fa2ac8,
      0x80f7e381, 0x34e62f59, 0xe1a0ae17, 0x98286ca2, 0x121ddf8c },
    { 0xa19b7f44, 0x49c0c09d, 0x44b47593, 0x21cf10eb, 0x21b7abd1, 0xeead581f, 0x2c8e0b36,
      0x221fc0f6, 0x49455f17, 0x97e3d3b4, 0x3112a8ef, 0x30b1817a },
    { 0x550e2f47, 0x129eb4f3, 0x712138f7, 0x6337dc9d, 0x13e54a9c, 0x3e2a1f7f, 0xfd9784aa,
      0xddcc51e7, 0xea743f41, 0x8c33d591, 0x01697ae2, 0xeea16f9d },
    { 0x4cca56ab, 0xf21adecc, 0xa3122b0e, 0x59777afe, 0xe5deb68a, 0x7e0d55e7, 0xb01753e0,
      0x2fb272e9, 0x8b664954, 0x921cb62a, 0x04b92393, 0xd42a4bd2 },
    { 0x32a41354, 0x305bb9a8, 0x5929c1da, 0xb50d25f2, 0xc1ad2df7, 0x0526e29c, 0xd3ad2521,
      0xe75ec303, 0x5af9c584, 0x37e078f4, 0x57fb4021, 0x75bd3593 },
    { 0x65fccf36, 0x84662bf6, 0xa828e18d, 0xc53f2a9e, 0xaddb7d44, 0x342a2176, 0x204bf00b,
      0xb8fec0e2, 0x6cc5ffcd, 0xc6bd5e9d, 0x8026401c, 0x47acf18f },
    { 0xb41f372e, 0x4d15eb3d, 0x5413894a, 0xa8a8acce, 0x75665b49, 0xf1f33fca, 0xfd23529e,
      0xf0711a2f, 0xd87a4a75, 0x57ffcaca, 0xff113d37, 0x2841b4da },
    { 0xfdb89d39, 0x712ae60e, 0x23f83b51, 0x3abf3c50, 0x1e1c7829, 0xc5986922, 0xb24cadd3,
      0x35d95b85, 0x40aa05fe, 0xf5a82b37, 0xe7f5d8f7, 0x13beee45 }
};
static const DIGIT local_py384[96][SECP384R1_WORDSIZE] = {
    { 0x90ea0e5f, 0x7a431d7c, 0x1d7e819d, 0x0a60b1ce, 0xb5f0b8c0, 0xe9da3113, 0x289a147c,
      0xf8f41dbd, 0x9292dc29, 0x5d9e98bf, 0x96262c6f, 0x3617de4a },
    { 0xebab3673, 0x6822b755, 0xf49bf223, 0xd1210cb8, 0x440a7aba, 0x9caba57e, 0x0e2a3bdb,
      0xd41d4fc4, 0x9a9448f1, 0x41816d5e, 0xe12906d2, 0xd815229d },
    { 0x09ea07cb, 0x4a0ab4d4, 0xbc926b83, 0xdd574f9c, 0x967f587e, 0x6a6a72aa, 0x0372df0c,
      0xd78d4d86, 0x828d35e0, 0xe9a595ae, 0x50514591, 0x38849236 },
    { 0xe701e4d0, 0x7958b510, 0x1708eedf, 0x6e5f2be8, 0x4f79204a, 0xb0b25e1f, 0xabedb90e,
      0x1a479632, 0xb01bee25, 0x242dc35a, 0xf910c2d5, 0xa89fbdb7 },
    { 0x220244b8, 0x3cb0f721, 0xe6860a4b, 0xa1ce495f, 0x4d1751ad, 0xc0a1437b, 0xf55e10ee,
      0x5b0af9b4, 0x9cbedc04, 0x70f25a5e, 0x5c11ea07, 0x4c1548bc },
    { 0x78a34642, 0x67787c6e, 0xa14c33ef, 0xe5ee4625, 0x09f4a12e, 0x2a10ae07, 0x568a2562,
      0x10aff206, 0xb5e65097, 0x7648b908, 0x47190a73, 0xc62e2af7 },
    { 0xa912e417, 0xb9b681be, 0x205d7508, 0x2df33bfa, 0xaba26b12, 0x685d8cc3, 0x87023f04,
      0xbc005209, 0xb02de357, 0xad380091, 0x5d674f6b, 0x8e3337f5 },
    { 0x6d293338, 0x1f67ebd3, 0x518a9a55, 0x9893a729, 0x9b8e1e80, 0x89f3b063, 0x78f712f2,
      0x95b29110, 0x27223053, 0xdf966b7d, 0x35396542, 0x29e5ae4f },
    { 0x0328049b, 0x02234625, 0x8e055df0, 0xdab5a0c0, 0x26deab8d, 0x4759e138, 0xdf5f1ec1,
      0x9132448e, 0x1056b382, 0x1c7edccc, 0x446d869e, 0xa8fdb93c },
    { 0x2fd6596f, 0x445201c4, 0x366014c3, 0xd29c151f, 0x10e4fd21, 0x6e489e1b, 0x14996f8c,
      0x54c04482, 0xcddbbbac, 0x8bb04837, 0x1b6e5b43, 0xc4c6090f },
    { 0xebf46b90, 0xe8748e64, 0x02e421d6, 0x0c86f427, 0xeaddfd7f, 0x4f18508a, 0xb65e4ead,
      0xbe0387e0, 0x8ca85c8c, 0x30115744, 0xa14b1c47, 0x18bafdae },
    { 0x8fda1cec, 0x72b187e2, 0x36f76260, 0xe56836a4, 0x2a957afb, 0x02830845, 0xf2b6ac21,
      0x4059981a, 0xa3e33627, 0x92573f3d, 0x70d98f8f, 0x8b230822 },
    { 0x2d8b523f, 0x45e0b176, 0x39d2cf6c, 0x1b7078cb, 0xee4928be, 0xaf40b68b, 0x5a620149,
      0x65d24d48, 0x5d389045, 0x4a1e9b51, 0xf610090c, 0xae61a171 },
    { 0x28fd84dd, 0xf61f4f0c, 0xe323431b, 0xf32b946a, 0x5da6e7e8, 0xe8c51797, 0x5e9f0ecc,
      0xa121a936, 0xff8a86b5, 0xed5b9f08, 0x54d9705b, 0x7453fe30 },
    { 0xf06dca9e, 0x0c5bb6c2, 0x4fd22a60, 0xd376123e, 0x2325c589, 0xed4ac3f7, 0x30bb885e,
      0xedce0302, 0xb608a6ca, 0x8aaf499b, 0xa9b81ebb, 0x779876a7 },
    { 0xfac5bc04, 0x17ec580c, 0xece4d5a4, 0xc76909fd, 0xa9a07232, 0x2801020a, 0x4384667e,
      0xff06b5d1, 0xb1e4be6c, 0xbc65e264, 0x1259802b, 0x4fabc2bf },
    { 0xc16bb523, 0xa0ecd5d0, 0xcc51c42f, 0x29f49d50, 0x892459a9, 0x36275538, 0x8484b5f1,
      0x62637770, 0x5c806f88, 0x89a8437e, 0x5d9087bc, 0xa64da09a },
    { 0xdefb5093, 0x09df5dd5, 0x26a847c5, 0x916f53c3, 0xe1c388f2, 0x7fe24da6, 0xeed7cec9,
      0x4d38da8e, 0xe37bbbe8, 0x9efe7f18, 0x1ec6ccb4, 0x310ab22e },
    { 0xa45d9e35, 0x468ac4c7, 0xf49c0bee, 0x02e3059c, 0x8ef2acbb, 0x36089f51, 0x8199fac3,
      0xa41ca1e0, 0xa08f5c25, 0xfeb4cd34, 0xd916513a, 0x411d2298 },
    { 0xc3d9d39a, 0x9093107b, 0x6af9e4de, 0x02878403, 0x06936041, 0x6e0920c2, 0x2491c59f,
      0x01993809, 0x49f4c8cf, 0xe716ccfd, 0x6f80fa8b, 0x5d74a6c9 },
    { 0x4a220ca4, 0xb96a9042, 0x30560a02, 0x99002b95, 0x6186f30e, 0x3de7b18b, 0x9ca19d83,
      0x63628bc1, 0xe7dcbd8e, 0x185b2584, 0x7c371327, 0xdf11408c },
    { 0x05afad92, 0xadb05a6f, 0xdccd2a07, 0x6f60fb80, 0x16ebe27e, 0xe5f7a486, 0x29ab1ca6,
      0xd94d431d, 0xacc19cdb, 0x19c9c477, 0x6c088656, 0x3948f91f },
    { 0xb3ee6deb, 0x99472dff, 0x31ea0776, 0x637af70d, 0x8a9437e5, 0x202b5905, 0x8651b370,
      0x896e8f4e, 0x8ad736ce, 0xab946c50, 0x90a13cb3, 0xbf6ec454 },
    { 0x67903be8, 0xa17f33fb, 0x4078f9ed, 0xfa15ea93, 0x52e6f955, 0x0f85c2af, 0x321df41e,
      0x7ce2a463, 0xf5eab794, 0x43b47a5a, 0x2bdbeabb, 0xc490547b },
    { 0x23d86ecd, 0x7a7e1839, 0x085a4e9a, 0x31ea31b1, 0xbe643603, 0xbc40ce5a, 0xa2124163,
      0xbd22cfb2, 0xde3a82ba, 0x6f04caa2, 0xc3b38e69, 0xb9d2852c },
    { 0x1fb62abf, 0xacbdc014, 0xcaba0102, 0x9fe70e8c, 0x076eec6c, 0x19425828, 0x3251dfef,
      0xf72bf2a5, 0x4e84c4a3, 0x061b2bce, 0x33435953, 0x228c2228 },
    { 0xde1aa18f, 0x2c6e8711, 0x5b5d3e24, 0x630c727a, 0xa4e95e69, 0x1dbef0a1, 0x38b8d116,
      0x3750825e, 0x3f6059b7, 0xcbefa593, 0x0f597307, 0xbd60a040 },
    { 0xc4a0143d, 0xc29f735c, 0x4d7ccd32, 0x4ecc34d2, 0x9c623b32, 0x03073083, 0x3f872163,
      0x47d08954, 0xd82dcd7d, 0xeca60f71, 0xbbc36994, 0xec1ef93e },
    { 0x39df6db2, 0x997c6627, 0x7c8791a2, 0x35b365a4, 0xe8e859da, 0x5c5a7fd8, 0xa9a55def,
      0x939226dd, 0x81b96729, 0x167c9adf, 0x1c82f217, 0x344ad7d9 },
    { 0xddee3fa8, 0x3e610d70, 0x828a967f, 0xbf575954, 0xb53a955e, 0x2ae095f6, 0x7b90d88c,
      0x56033472, 0x8dc0a40a, 0x3c112381, 0xd5c067d3, 0x4cfec7ce },
    { 0x205e62e9, 0x5841ee49, 0xc40ea262, 0x96ed905a, 0x506a5c36, 0x2c97dab6, 0xa4aef7d8,
      0x0f317f1b, 0x952c8acc, 0xaec182bc, 0x600a1b20, 0x509e270d },
    { 0xf5b834cf, 0xe4b80367, 0xfcd18ce5, 0x618379a5, 0x214592a9, 0xbd10607b, 0xede301ef,
      0xc1e82061, 0x048e4b41, 0x98c302a2, 0x147750f1, 0x96c4edbd },
    { 0x2adaefc2, 0xe5f50bfe, 0x36adb53e, 0x64666b55, 0xe90ed2a4, 0x7768fb7f, 0x06cc84c0,
      0x425d7817, 0x31d6ab0b, 0x52ec806a, 0x4697d363, 0xd7e67972 },
    { 0x40111ed6, 0x9097f043, 0x962a83fb, 0xba955625, 0x130860da, 0x56181333, 0x752a51f0,
      0x9614a3ec, 0x27f4b8e0, 0x9d89fa34, 0xddb96fd5, 0x8859ef7e },
    { 0x6ee6c982, 0x53948b38, 0x7b9e974e, 0xa578322e, 0x6bb2d5d8, 0x38610155, 0x66527a63,
      0x3ed290b7, 0x7e91a190, 0x388b7a9f, 0x226ce3b0, 0x811e25b1 },
    { 0x6f39d742, 0x56b34044, 0xa0f2b519, 0x9e2e938d, 0x77f54092, 0x732bf98f, 0x04211c4e,
      0x0f6b57f6, 0x837a6b85, 0x106561e9, 0x6402b5c6, 0xf75eff06 },
    { 0x91013e25, 0x965622c6, 0xced6e714, 0x4b8f321b, 0x0051e057, 0xb5a0603d, 0x7a5e5a25,
      0x81a2b658, 0x306b5712, 0x7bdd7fa6, 0xf6acf171, 0x526f1b07 },
    { 0x2c18581e, 0x9bde2f07, 0x44b0f2e1, 0xa8a4a413, 0xca599ce5, 0xefbec9a6, 0x8c756857,
      0x994d283d, 0x68e2eda5, 0x41937029, 0x849988cb, 0x7e35d102 },
    { 0xdbf1b2b6, 0x58830d04, 0xfe9f2877, 0x257cb87a, 0x2da9ca1e, 0xc37b19d9, 0x663ee951,
      0xc0414185, 0x7f22ee03, 0x1c8c3fcf, 0x6e84fe25, 0xbc3a4e5b },
    { 0x92b01ea6, 0x183b44e1, 0x01ae050c, 0x7414770c, 0xd705d880, 0xf339b53d, 0xfaa8f031,
      0xab2344eb, 0x65ab7f33, 0x04ae4be6, 0x8ab95c34, 0x1bbea6c3 },
    { 0x1b8935ff, 0xfe97a16b, 0xfa735802, 0xfca8c5be, 0xf76431b9, 0xf7f6ea02, 0x72f26ec8,
      0x37869c2f, 0x1ec658a7, 0x150686bf, 0xa3ae9e76, 0xbe41e479 },
    { 0x2f711ecf, 0x42e9a286, 0xc2472c15, 0x0db3df0e, 0x41df85f8, 0x1283110d, 0x6e41c686,
      0xb603544f, 0x54041a11, 0x732f05a0, 0x7820e29e, 0x9586a28c },
    { 0x7a3d1084, 0x4d7e446d, 0xc720a95e, 0x8d37cb72, 0x96fb5822, 0xf411e7ca, 0x6301c209,
      0xf5bee88d, 0xf5203595, 0x5337672b, 0xd554d1d3, 0x75b71e1b },
    { 0xcf414a55, 0x80453099, 0xb6a87be3, 0xaea86e9f, 0xe55b3b1c, 0xed647cea, 0xf3197f7f,
      0x32927302, 0x423aa340, 0x501d12e8, 0xc9c9273d, 0xc00c4d4e },
    { 0x4ce184ac, 0xb9e4584d, 0x95de57dc, 0xd36aab4f, 0xf5a0cf25, 0x99c48720, 0xabfc71c7,
      0xf8ad16ef, 0x4ebf2d1c, 0x603bb39f, 0x2b8e1ad3, 0x806fbd10 },
    { 0x287ec7aa, 0x8b5ff455, 0xfc7e03ed, 0x517adad9, 0x8e765f15, 0x25e7253e, 0x2759b7ac,
      0x24758758, 0x10275f27, 0x86d0cea2, 0xee1842f9, 0x9926862d },
    { 0x9c821b7d, 0x5ba62b46, 0x27d1e9ff, 0xd0afb3ca, 0x28aa18e7, 0xe2765b05, 0xb6296d2c,
      0xed1fec99, 0x16a3589d, 0x2013139e, 0xc2737a29, 0x333edd5c },
    { 0x2f5fe954, 0xa6c32ab6, 0x885ea15f, 0x7c46012d, 0x61b9d626, 0xf5307fb6, 0x30b8384f,
      0x32f82b92, 0x0516a17f, 0x785c7b87, 0x821aecce, 0x5dc6ee8f },
    { 0xe6c88c41, 0x822d0fc5, 0xe639d858, 0xaf68aa6d, 0x35f6ebf2, 0xc1c7cad1, 0xe3567af9,
      0x577a30ea, 0x1f5b77f6, 0xe5a0191d, 0x0356b301, 0x16f3fdbf },
    { 0xd3eb1cca, 0x8f80a433, 0x0bbac091, 0x27a8e3fd, 0xa8ec664e, 0x738590ee, 0xb2cb7d0e,
      0xaa3dd92d, 0x37ea27c4, 0x76c846cf, 0x87b431ed, 0xb9fb524a },
    { 0x50c23814, 0x329b33e9, 0x468e6a05, 0xa42f9dde, 0xd1b32a37, 0x28c3a32c, 0xc8acaf24,
      0x2103c6a0, 0x43b2cbdc, 0x9a7eaf32, 0xcfc82f75, 0xea9ec23d },
    { 0x279cac7c, 0x7ec34da2, 0x1b8ec400, 0x1abea785, 0xeeeeba47, 0x25f6a9b4, 0xd21eda2a,
      0x7379eac9, 0x9af2e50d, 0x08402d5a, 0xc0b5abd9, 0xd1137ea3 },
    { 0x11836d15, 0x9529388e, 0x756b176c, 0x316e074c, 0x0984ca6d, 0x2ba847f8, 0xe78c0cf1,
      0xb1e2d2ce, 0x3a4e10db, 0x51d03996, 0x9e40a86c, 0x4dbf6d73 },
    { 0x28997d31, 0x3eaf4dc3, 0xb212cd3a, 0x5d3bfca7, 0xa1f86b49, 0xa41158d2, 0x98ed293f,
      0x9aa1ad64, 0xf1808db6, 0x76940e40, 0xa1dd0752, 0xe3bfdeba },
    { 0x18aac609, 0x70d17b7a, 0x050839cf, 0xe4c16b59, 0x7438e95e, 0xd51d3e0c, 0x8dcdf8e3,
      0x682cd98a, 0x2592ab4e, 0xca1499c8, 0x825613aa, 0x850a3c8a },
    { 0xe2cbfa44, 0x4b5115e0, 0x5914f4c2, 0x11d4efaa, 0x94c0cfde, 0x2be2acf4, 0x97015da2,
      0xf8032336, 0x0abccb66, 0x3d0bcb89, 0x75d1fd36, 0x2a257889 },
    { 0x87e2519f, 0x4004d2f2, 0x331a0e13, 0xb1cdeaa6, 0x6e22db4b, 0x05dfc199, 0xbf734568,
      0xe05e0653, 0xfbc4cd34, 0x93664f3b, 0xe179a680, 0xc3b58cde },
    { 0xa56600f5, 0xf24b93eb, 0xe48ef3d4, 0xd26fe4e3, 0x00a42cd6, 0x1da58eb7, 0x0b708b29,
      0xbc181fa7, 0x46cfc593, 0x2f2510e8, 0x23484b36, 0xacac3695 },
    { 0xd090e5aa, 0x37d7d0bd, 0xa27598a0, 0x377b89f9, 0x512e854e, 0xf9015e85, 0x341befdc,
      0x4f2f1054, 0x095dec26, 0x6e0a0084, 0xf22dd9cc, 0x82f3242f },
    { 0xb620e1e6, 0x45983eba, 0x1e5f9b53, 0xba626771, 0x3fbb588f, 0xc38d3473, 0x7f32556b,
      0x8256b5b4, 0x4fc5d5ba, 0x08ea4e87, 0x5f3c4411, 0xfd6d07b0 },
    { 0xe3636016, 0xc07bb07d, 0x4c00333a, 0x12b29e9a, 0x53eec121, 0x888e1907, 0x640707c9,
      0x64acc0d1, 0x03519fa1, 0xb78ca0ff, 0xc6c4eafb, 0x5c88fb72 },
    { 0xfcf69d9e, 0x17e0ac0b, 0xd99a80f5, 0xf8fb7dda, 0x2bd8bcd4, 0x3cbc116c, 0x9732c221,
      0x40c015d9, 0x27e929e3, 0x2e0ebe2e, 0x22a20838, 0x93cf5583 },
    { 0xb25b8465, 0x56d6e47d, 0x3ea4a11a, 0x4af74b9d, 0x3dbb7eb6, 0xa898f066, 0x001f8107,
      0x06cb5dd7, 0x603b0189, 0x933664fc, 0x337228e6, 0x1bd1a457 },
    { 0x90374e84, 0x5fbed33d, 0x3ca2df4e, 0x0b51a3ef, 0x7fdfab4d, 0xb990a3e3, 0x0024108c,
      0xc1bcfaa8, 0x8647fc14, 0x678fd285, 0x2b13dde8, 0x0a925462 },
    { 0x5e695394, 0x98ca61b5, 0x0e70a633, 0x6f591102, 0x216b71d5, 0x80a93ec7, 0xc463e3b8,
      0xbbb25c8f, 0x8615e7a5, 0xb18e3fba, 0xfc212920, 0x5a5d5c5f },
    { 0x89f3c69a, 0x1228f051, 0xdcf0208c, 0xd9e13081, 0x8d0223a7, 0x951cb192, 0xd759ce54,
      0xe53f2beb, 0x60edbac6, 0x7392cd05, 0x1e6f179d, 0xa36e4c38 },
    { 0xaa644979, 0xa3cd5292, 0xbf5f18ac, 0x67028246, 0x79b30f09, 0x855ff5a7, 0x6809871a,
      0x94468255, 0xf0fc05b1, 0x0816d773, 0xc424ced7, 0xf5132490 },
    { 0x74987483, 0x207d9a72, 0xa03e1e95, 0x7a2c1137, 0xc7f641e0, 0x5439a675, 0xd038d0f7,
      0x3ce1b3d4, 0xd267716e, 0x37514e5f, 0x9a01be2f, 0xf874bd31 },
    { 0x0454b468, 0x3e51527c, 0x1ae5a919, 0xe42fce6c, 0x96e0b000, 0x87f04548, 0x2a1d5efe,
      0xe9c7ad4c, 0xc445565d, 0xe83efae1, 0x3d904f20, 0xbff63109 },
    { 0x8b8e632a, 0x9da5eb75, 0x2a67bb20, 0x04f9430a, 0x337b7d77, 0xdbec6713, 0x8bc3c1dd,
      0x9366b0c8, 0xff98c3d3, 0xbfe1079a, 0x4e0463e5, 0x17b3545e },
    { 0xba51b083, 0x27a8b1d6, 0xb41a3fe4, 0xe0b4ede3, 0xa0a03c29, 0x93edb60c, 0xbc442f42,
      0xe95d089d, 0x36e05f20, 0xdce08baf, 0xa652f7b0, 0x016c3f70 },
    { 0xaaeed382, 0x13e475d2, 0x222a1acb, 0xa7591818, 0x181bfeaa, 0x69c8c2a1, 0xda76f82a,
      0xf1890009, 0xe9c9bb60, 0x129162d3, 0x68517103, 0x8cb496fd },
    { 0x753ee324, 0xddcebb55, 0x6924666f, 0xe87ab07c, 0x4ecf1a68, 0x9b475d74, 0x2e6236c0,
      0xf82be8f5, 0x3cfd056b, 0x237c0dba, 0xc3c6cbd2, 0x354cd872 },
    { 0x3944b6d6, 0xce66a52d, 0x20f79db5, 0x988451ef, 0xd582bdfc, 0x4300b001, 0x147bb6e8,
      0xb22f9897, 0x10685251, 0xf8b944ee, 0xc5a20177, 0x4c34f592 },
    { 0x267be621, 0x06a9c51e, 0x6885406a, 0x850869d3, 0x35d53004, 0x787532ff, 0x3a05a393,
      0xef212ca1, 0xc967cf9a, 0xd692a2d8, 0xaa79c4ed, 0xc24e75e6 },
    { 0x00f30197, 0x07abd762, 0xd4e9c563, 0xe364a04f, 0xd34fde05, 0x0a18d7a8, 0x4def5a2f,
      0x796343d6, 0xcdfb8100, 0x8c49bb4a, 0x6fef27c9, 0x4bf2143e },
    { 0x14f02364, 0x1ee8af19, 0x91556534, 0x363211f5, 0xe5b5a54d, 0x70b3c1a1, 0xc1c64b56,
      0x213f21ac, 0x7c98448b, 0xd53ff1b6, 0x1801ee82, 0xf968aa66 },
    { 0x0cd001d1, 0x6fcb85e9, 0x3df1dfe2, 0xe0781a9d, 0xc8a2265b, 0xf650285a, 0x37a9b579,
      0x8727088a, 0x22c74609, 0xbb844df2, 0xd5476d33, 0xc3742094 },
    { 0xb0228bd2, 0x516090d9, 0x80745095, 0x2599843f, 0x28a501be, 0x2fe05207, 0xfea4fa16,
      0xbffe106a, 0xe03baebb, 0xf83a3767, 0x8ec66837, 0x71b00e3d },
    { 0x8032ae22, 0x79c6ee57, 0x837263de, 0x34c95370, 0x6c9e06d9, 0xf7c55301, 0xedbbc83b,
      0x58f7d1d1, 0xe81c6ded, 0x478a56ad, 0x588cf0a0, 0xb1415ce2 },
    { 0x4f0474fc, 0x9f7dc254, 0x40cb718b, 0x13f390d7, 0x3a45ad17, 0xe6091c18, 0x1605976d,
      0x9b795d80, 0x2055ee90, 0xe0bcd7af, 0x92ee7c2e, 0xd637fbf6 },
    { 0x8aef73c9, 0x82891358, 0x8fdcf0e2, 0x0278f9c8, 0x736ba12c, 0xda5422de, 0x8a7187e0,
      0x8eda5949, 0x56515f40, 0x9b9c9923, 0x61f862be, 0x59b93f29 },
    { 0x589533ed, 0x37a92437, 0xafb7b2b7, 0xc8d03cf8, 0xe1c86c06, 0x5a09b405, 0xbc4d1a9d,
      0x52f38699, 0xbf018632, 0x3f5d0995, 0x380a0d13, 0x65e4ee35 },
    { 0x66ec7670, 0x2395ecdf, 0x89c27dfd, 0x0f87c763, 0x2e602e17, 0x6ad8f575, 0x9d6a90ae,
      0x7652bc9a, 0xf8781fe2, 0x4ed54599, 0xeece0e40, 0xc1179ca4 },
    { 0x22248e00, 0xc0bc215d, 0xd45f6553, 0x9ed447e3, 0xdaad7d1d, 0xaf084771, 0x1f2d8179,
      0xee94bf70, 0x267bb3fd, 0x1e8c72ad, 0x8de7ac78, 0xc425de16 },
    { 0x9ef190ff, 0x09bb6d5b, 0x34e4a376, 0x5a186f76, 0xcd22a980, 0x96d53577, 0x66709488,
      0x907a086a, 0x7d8e3f1e, 0x7e567e0a, 0x486076ea, 0x36653375 },
    { 0x35b12e30, 0x50abffb5, 0x5ef6e7fb, 0x310c2622, 0x62b950aa, 0x769b07ce, 0xef7126c1,
      0x1e0f686c, 0x36b4d522, 0xc4601e6b, 0xc89617dd, 0xcf4d4f4d },
    { 0xf681a609, 0x57874d65, 0x283ff85a, 0xd8d8c262, 0xbc42a3cd, 0xcf13ae9b, 0x8ba86c86,
      0x6063ecf7, 0xd1ae018e, 0x09bd4976, 0xdceaf81a, 0x10b976f0 },
    { 0x0205f0d8, 0x62ec2e88, 0x5329d563, 0x8e49391c, 0x251ddb37, 0x71fcdbd2, 0xe9cee913,
      0x92a91a35, 0xd84aee90, 0x99c6e5cb, 0xf1a79358, 0x5459524f },
    { 0xdf5745ca, 0xee6e05b6, 0x9f304afa, 0x8373dbbe, 0x432202c8, 0xab7bf494, 0x6707a413,
      0x575dfbd0, 0x2ac7c5c0, 0x6fbb50d9, 0xc042e8eb, 0x8042b7d4 },
    { 0xa97eeec2, 0x779e13c5, 0x34631a1b, 0x4c8c03b2, 0x375f6406, 0x6dc880f9, 0x243c87bf,
      0x5deb5178, 0x1ffe48a5, 0x8ba028f0, 0xb253a99f, 0x7990c82e },
    { 0xda194383, 0xc6239833, 0xad55b1fd, 0xb4d2bd31, 0xfa999a4e, 0xb05888a0, 0x1a7db127,
      0x5c4e46be, 0xea3734bc, 0xa970057a, 0x1b34ca68, 0x6b51ba9b },
    { 0x6eafd08a, 0x9400a529, 0x2f0f7c45, 0x9d2dad27, 0x5abaee00, 0xa8f595fd, 0x2b34e6f6,
      0x3fcbab7a, 0x351939a0, 0x4454ccf4, 0x3b90a779, 0xd4fafd9c },
    { 0x90c532ef, 0x6bf12b44, 0x3f791143, 0x8644b4c8, 0xd9ec40fa, 0xa3eb5ee6, 0x10c6ba12,
      0x30d8c204, 0xeb0feb7b, 0x0c23f41f, 0x1f6d5b1f, 0x1cd46eb6 },
    { 0xc9fab249, 0xc2181590, 0xdb3e1b77, 0xc2582db4, 0x9cfc4d7e, 0x1c2d3890, 0x15550e58,
      0x96ad8a95, 0x529fa3ef, 0x7184f9ff, 0x1a5318d1, 0x7c4525e9 },
    { 0xdf741a6a, 0xa6aa9ec1, 0x7eecd684, 0x514d1c37, 0x91c00276, 0x147d8aef, 0x13214ea1,
      0xbddd1aaa, 0xba8a0868, 0x15bad99a, 0x4f01f331, 0x434837ea }
};
#endif //PRECOMPUT
static const DIGIT local_inv2_p384r1[SECP384R1_WORDSIZE] = { 0x80000000, 0x00000000, 0x80000000,
                                                             0x7fffffff, 0xffffffff, 0xffffffff,
                                                             0xffffffff, 0xffffffff, 0xffffffff,
                                                             0xffffffff, 0xffffffff, 0x7fffffff };
static const DIGIT local_xg_p384r1[SECP384R1_WORDSIZE] = { 0x72760ab7, 0x3a545e38, 0xbf55296c,
                                                           0x5502f25d, 0x82542a38, 0x59f741e0,
                                                           0x8ba79b98, 0x6e1d3b62, 0xf320ad74,
                                                           0x8eb1c71e, 0xbe8b0537, 0xaa87ca22 };
static const DIGIT local_yg_p384r1[SECP384R1_WORDSIZE] = { 0x90ea0e5f, 0x7a431d7c, 0x1d7e819d,
                                                           0x0a60b1ce, 0xb5f0b8c0, 0xe9da3113,
                                                           0x289a147c, 0xf8f41dbd, 0x9292dc29,
                                                           0x5d9e98bf, 0x96262c6f, 0x3617de4a };
static const DIGIT local_a_p384r1[SECP384R1_WORDSIZE] = { 0xfffffffc, 0x00000000, 0x00000000,
                                                          0xffffffff, 0xfffffffe, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0xffffffff };
static const DIGIT local_b_p384r1[SECP384R1_WORDSIZE] = { 0xd3ec2aef, 0x2a85c8ed, 0x8a2ed19d,
                                                          0xc656398d, 0x5013875a, 0x0314088f,
                                                          0xfe814112, 0x181d9c6e, 0xe3f82d19,
                                                          0x988e056b, 0xe23ee7e4, 0xb3312fa7 };
static const DIGIT local_p_p384r1[SECP384R1_WORDSIZE] = { 0xffffffff, 0x00000000, 0x00000000,
                                                          0xffffffff, 0xfffffffe, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0xffffffff };
static const DIGIT local_n_p384r1[SECP384R1_WORDSIZE] = { 0xccc52973, 0xecec196a, 0x48b0a77a,
                                                          0x581a0db2, 0xf4372ddf, 0xc7634d81,
                                                          0xffffffff, 0xffffffff, 0xffffffff,
                                                          0xffffffff, 0xffffffff, 0xffffffff };
static const DIGIT local_p1x_p384r1[SECP384R1_WORDSIZE] = { 0xa7dc885c, 0xd8ed2ff2, 0x12f8e1fa,
                                                            0xb499e34b, 0x37d205ce, 0x7eb2ff39,
                                                            0xa5127bf6, 0x9f1383e8, 0xd6c96f1b,
                                                            0x09e7ad61, 0xf514dae5, 0x7fbbe67c };
static const DIGIT local_p1y_p384r1[SECP384R1_WORDSIZE] = { 0xb0437ac0, 0x840182e0, 0xde9d6cba,
                                                            0xdf52f874, 0x6fee5e30, 0xc8efc7be,
                                                            0xd33df9bd, 0xd59af65a, 0x87f4a1b5,
                                                            0xd5141a5d, 0xf5ca9b9d, 0xf38aabe5 };
#endif //WORD32

/** <b>ECC Curve structure variable for SEC-P384r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp384r1;
#endif

#ifdef P521
#ifdef WORD32
#ifdef PRECOMPUT
static const DIGIT local_px521[133][SECP521R1_WORDSIZE] = {
    { 0xc2e5bd66, 0xf97e7e31, 0x856a429b, 0x3348b3c1, 0xa2ffa8de, 0xfe1dc127, 0xefe75928,
      0xa14b5e77, 0x6b4d3dba, 0xf828af60, 0x053fb521, 0x9c648139, 0x2395b442, 0x9e3ecb66,
      0x0404e9cd, 0x858e06b7, 0x000000c6 },
    { 0x81a8402f, 0x3a6c508f, 0x07dcdc53, 0x0dbf5c80, 0x985630df, 0xe5965da7, 0x61ca00a0,
      0x943ff188, 0x0b188a52, 0xd632f8d2, 0x92fd3f3d, 0x2a87f661, 0xac743b0b, 0xda97da26,
      0x9c8ad05d, 0x7d10d8a8, 0x000001d1 },
    { 0x69be00ac, 0xb38e8b90, 0x95e50b81, 0xe46b33cb, 0x1f5d4723, 0x9c9152c4, 0x8c2a465a,
      0x7b8f4f6e, 0xf269cb33, 0xb85fa9e8, 0xcb5b47a9, 0x03c7389e, 0x0f312e99, 0x458afc78,
      0xb5ac20c9, 0xf40a0f02, 0x000001d5 },
    { 0xe4ffac9b, 0xe070a145, 0xaa280fff, 0xf8295455, 0x7010805f, 0xa2d6cf89, 0x3220ff7c,
      0x04e13089, 0xad11681e, 0x5935e6e2, 0xb9cef2bc, 0xd91efb7b, 0x14bb8c4d, 0x3c260b99,
      0x3a265599, 0x24bf88d5, 0x000000f0 },
    { 0xcaa4bec3, 0xa3480d58, 0x5989e4a0, 0x38948993, 0x6f7ba4ca, 0x3f4d7b6c, 0xd39cb2ae,
      0xe2913a55, 0xd7be6499, 0x615043ea, 0xf504fec3, 0x9d4c41cb, 0x8d44893b, 0x4f93da96,
      0x9a7928e0, 0x33a7acdc, 0x00000013 },
    { 0xa2302041, 0x4f0e025f, 0x48d262b5, 0x4d50d989, 0xd622c7d4, 0x01720a0a, 0x58d1c97c,
      0xe3b5a941, 0x38aed2a5, 0xfbc5b183, 0x823b5d8d, 0x27f02177, 0x775f38c5, 0xadf74074,
      0x4f1a6ea0, 0x95bf91f4, 0x00000104 },
    { 0x46d39a88, 0xb5f53851, 0x9cc76deb, 0x58a99a0f, 0xf3825b82, 0xcfdd909e, 0xf4694014,
      0x8f49dbdc, 0x9cfec7b7, 0xefd505fe, 0x56925281, 0x66a7f2a1, 0x2a604fb8, 0x6f478d9b,
      0x60f7c436, 0xcbc72e9e, 0x00000021 },
    { 0x3ca98249, 0x93d61c07, 0x907aa44a, 0x04c4f43d, 0x51d5b1d6, 0xee3d2c31, 0xf859df2a,
      0xe74d6892, 0x922f8fa9, 0x16ab4838, 0x757b0010, 0xb0b6f7d1, 0xa918f320, 0x7ca940d8,
      0x97b8c9b5, 0x45e9325b, 0x000001e1 },
    { 0x008c7a44, 0xabf17feb, 0x7b12def4, 0xade7917a, 0x3786b033, 0xbfba65b8, 0xf0d9f9f1,
      0x8ef4af44, 0xab136cae, 0x8f47615c, 0xbcbd5d3e, 0xe42f100d, 0xf05f9b27, 0x8f3dc658,
      0xd6f2dbb2, 0xd95a6b2d, 0x0000008e },
    { 0xc11b2a0b, 0xd23357cb, 0xb179a631, 0x70de0d3f, 0x122b12b6, 0xb57a0fad, 0x78489416,
      0x3203600f, 0xd26d2267, 0x99fe0b86, 0x3b9deed0, 0x36f3262f, 0x24a32dcd, 0xb6e61160,
      0xaa745047, 0xeabd7c06, 0x0000009c },
    { 0xe93ef527, 0x1683448d, 0x0e5b6f98, 0x158fd922, 0xe23f2fa6, 0xcd0bcea6, 0x15982d3b,
      0x455d35de, 0xbda98d71, 0x2fc788d7, 0xf139b889, 0x65d44c08, 0x4105fd65, 0x219749fd,
      0xe02f1fd1, 0xa9ff816a, 0x000000f0 },
    { 0xb6813f72, 0x6f7f2fe8, 0x940210f0, 0x95f979fe, 0x1a409657, 0x42cbd8d7, 0x474c141b,
      0x2e2d2f08, 0x52a6a362, 0x63e5eff0, 0xb1c5a27d, 0xcd24503e, 0xdb08b256, 0x30335d77,
      0x9724bebf, 0x827057cf, 0x00000003 },
    { 0x1c3be7ef, 0x62b63169, 0x8074207d, 0xdab254bf, 0x235a4c77, 0xf6f1466b, 0xd3b01424,
      0x66f8f60f, 0x3bae9f80, 0x9da54069, 0x500f8a6c, 0x1a840763, 0xb25994b1, 0x542e2dd9,
      0x183b3cee, 0x996307e5, 0x0000006e },
    { 0x5c346588, 0xbb59f267, 0x76f373a1, 0xac37b368, 0xa4c6d860, 0x4826e015, 0x61bdf224,
      0x4ef341b0, 0xbecac30b, 0x70b01fb4, 0xf0bb0c51, 0x5f518cfc, 0x9d163117, 0xbbec7a91,
      0xde2da58e, 0x8a65bee8, 0x000000ba },
    { 0x5a714194, 0x9aa53b47, 0x6957d5eb, 0x01163dd8, 0x908f5929, 0x5bd2c24f, 0xdbe42b0f,
      0x87f519f4, 0xfb6b5ad4, 0xff767379, 0x9a193f28, 0x4d0f8bba, 0xee549d79, 0xf3c42bd0,
      0x825244fa, 0xbf851d3f, 0x0000009b },
    { 0x414d9815, 0xbf2097b9, 0xcda66dfc, 0xfa0673b3, 0x193b17b0, 0x8c775677, 0xc5387e9a,
      0x07c8dbf6, 0x4c5da984, 0x141de69d, 0x06a8b133, 0xf0650ac8, 0xd2b258c4, 0xadd766ee,
      0x96303678, 0xa6f42763, 0x000001da },
    { 0x689dbe3c, 0x94a182e2, 0x02610256, 0x2d8ee166, 0xe6294291, 0x4d0cb259, 0x0a7fcf2d,
      0x244db120, 0x8f58f3af, 0xb50d34b0, 0x13275075, 0x2ad4b9ec, 0xc96bfcfa, 0xa19beccf,
      0x86f5baf0, 0x1a5509da, 0x00000185 },
    { 0xe96a67e2, 0xd8720c8a, 0xd54e0f6a, 0xa646eced, 0xcbd31552, 0x3936ce2d, 0xb35268c2,
      0x6d44c225, 0x6168f13d, 0x56dc0d08, 0xfb2c207f, 0x2b9bbca1, 0xa136c5a4, 0xb4f0da3a,
      0xf8216f89, 0xed2cf79b, 0x00000141 },
    { 0x5c6f42b0, 0x519a595a, 0xa675acb4, 0xe39552a9, 0x609641a9, 0xc993a489, 0xcb9d6301,
      0x1427eb18, 0xfe02d86e, 0xeb102dd9, 0xa4c15ddc, 0x86dfc2d5, 0xb76f6b2b, 0x0dbfd177,
      0xb285289a, 0xec31ea91, 0x000000e2 },
    { 0xbdf87653, 0xc6999604, 0xb3f0b8c1, 0x5383b379, 0x0bf0435c, 0x98c6ac3d, 0x433c527b,
      0x2b04984e, 0x02073e7f, 0x5308793c, 0xe1ef3fa7, 0x4897091d, 0xbb6b4777, 0x4fea6f39,
      0x472023e1, 0x5ad4c5cc, 0x000000e2 },
    { 0x81128a65, 0x620aff9a, 0xd6a9d2c1, 0xa463ffe7, 0xdd4810ef, 0x2206277a, 0xdd0687ca,
      0x413edd73, 0x249cef0a, 0xf4b12ec9, 0xc4b165cf, 0x14dd6665, 0xc5c070c3, 0x0cf86a23,
      0x84755f2c, 0x42354fe9, 0x00000131 },
    { 0x5338e99e, 0x541bdd79, 0x2656702d, 0x2753b15c, 0xa7f6af6e, 0x360263f1, 0x02adc827,
      0x42c97787, 0x27abff88, 0xb4b4a665, 0x4a9a1ce9, 0x1f9dfe51, 0x48ceb12e, 0xf1fa56ac,
      0x6d0b967d, 0xf0e55a36, 0x00000116 },
    { 0x7b602085, 0xf762eaf7, 0x2d470147, 0x3b6e6233, 0xd3cca3e7, 0xe1028d76, 0xccf39be7,
      0xf3616dd8, 0x08e8f10c, 0xa35c4cc1, 0x7cb4b321, 0xc413db10, 0xc3249f4b, 0xeab81774,
      0xbe83bc78, 0x3e7bdea7, 0x00000134 },
    { 0xa2abb00e, 0x2d2f9e08, 0x17648c6a, 0x4c8c679d, 0xb8520197, 0x52e2e9ba, 0xe0898e48,
      0x7470a3cf, 0x90617e18, 0xbfeb2a91, 0xf44de1a6, 0x247ea702, 0x7b1604fb, 0x7baf95b2,
      0x020c5856, 0xe9e81b78, 0x00000159 },
    { 0x2feda8ac, 0x336b0622, 0x0c0b391c, 0x4658ae22, 0x1f2119d0, 0xe12f058f, 0x495525a1,
      0xa1f96c94, 0xcd7d9f17, 0x32efc7ae, 0xbea7a5eb, 0x34e4fe2a, 0x7135c5f5, 0x0efd2492,
      0xb83241ff, 0xd7a04eb2, 0x00000036 },
    { 0xc7734385, 0x067f879f, 0xb4a51794, 0x787c7f18, 0x72d9af69, 0xa269c446, 0x87d71897,
      0xa4498e73, 0xa91814ac, 0x28e3d84a, 0xf62fede0, 0x8caf2f61, 0x5dc202bf, 0xad36da31,
      0xa4689189, 0xfed8bc0f, 0x00000068 },
    { 0xbecff63c, 0x5d3f00ec, 0x671a2835, 0x280fc963, 0x849ad2f5, 0x27cf9990, 0xb27c7f2d,
      0xa1cfa523, 0xb8278b74, 0x9d9a6870, 0x9ee793b2, 0xd6176cdf, 0xb31e9501, 0x9a98521a,
      0x7436e438, 0x4422e43e, 0x000000d9 },
    { 0x2a7f01fc, 0xbee09423, 0x998af9a2, 0x75176a16, 0xfbe89a7a, 0x13273068, 0x13c42b6a,
      0x3e74aa6f, 0x579c61b0, 0x7da022be, 0xff23e10a, 0xaa759216, 0xfd852641, 0xcf7f7d1a,
      0x1090b6c7, 0x0e9aaf1c, 0x000000db },
    { 0xc6884ce1, 0xc2d7eaa1, 0x9faf6322, 0xd7b35c4f, 0x337ea884, 0x8d1a1d85, 0x0f946ebe,
      0xbcc6a809, 0x754fb8d2, 0x45eab172, 0x91dbd926, 0x1e0b251e, 0xfdce8503, 0x4b61112a,
      0x15fa4759, 0x80737c49, 0x0000000a },
    { 0x3dca9409, 0x8002b6d6, 0x163b8124, 0xd31714c1, 0x720a124a, 0xca655b42, 0x9596636b,
      0x88388dbe, 0xc0b7dc61, 0xa321822d, 0x5824e60f, 0xcdd952c8, 0x6cb4916a, 0x79ff4436,
      0x5f88dab8, 0xa115847e, 0x00000009 },
    { 0xe09d583b, 0x9ebd0a79, 0x3f312038, 0x8890ceb3, 0xfe1c7d97, 0x254ff069, 0x3825dcb4,
      0xcd82a50a, 0x4c37cde0, 0x065bf2f9, 0xa7b458e7, 0x59b7c4e2, 0x7d105c29, 0xace6716e,
      0x9f26170e, 0x84d99934, 0x00000107 },
    { 0xa6a08edb, 0x0bc2fc66, 0x81d46942, 0x4f83dbe4, 0x4e44b819, 0xdd379bc1, 0xb5785cc1,
      0x6f409f1a, 0x40232da3, 0x3fac6dda, 0x11b9bc84, 0x83b4bfdc, 0xc91e3874, 0x4563149a,
      0xf562966f, 0x210062c6, 0x00000126 },
    { 0x6a3cf15f, 0x9aecae2d, 0x9c959376, 0xd9d0b49a, 0x57cc7e25, 0x4aad9454, 0x3876fe58,
      0x68361832, 0x2bd9dc24, 0xccc6f2d8, 0x605bafbe, 0x3970525c, 0xa2c5499d, 0xf17524ed,
      0xb0c7bf4f, 0xe30b69d5, 0x0000005f },
    { 0xdf9314e0, 0x904f2d4b, 0xe7a00aac, 0xdaae850d, 0x582efb03, 0x79231083, 0xec7fe6d2,
      0x80f1c283, 0x199d74a8, 0x2d5b3996, 0x395007e7, 0x5f120b9b, 0x4773f03e, 0x30d23773,
      0x3b78b686, 0xf4c19273, 0x00000121 },
    { 0x6e4a010a, 0x714c9d4d, 0x98bbdd05, 0x9e8621eb, 0xefc60f48, 0xf3f33305, 0x0706c352,
      0x4e392a4c, 0x94dcfc6c, 0x5d0f7b3a, 0x2434d058, 0x28c34306, 0x41885900, 0xdba377bb,
      0xad73df88, 0x0ed4082d, 0x0000012a },
    { 0xd19906d0, 0x7facf165, 0xad92d455, 0xd8e059af, 0x6ebec576, 0x7356696f, 0x1d6d311d,
      0x67044e55, 0x3a2e50cf, 0xe2bc81dc, 0xc00ea37f, 0x860036de, 0xff32d69d, 0x71dfbe61,
      0xf502baff, 0x298b766b, 0x0000019f },
    { 0xc4d6ab08, 0x69cad3cc, 0x3b8990b8, 0x3adb5777, 0x8d958801, 0xd0cad8ce, 0x6d728f9e,
      0xcb572e66, 0xcd5131fc, 0xe3d9e7c4, 0x6145dc06, 0xafceb6b0, 0xe213043a, 0x12ecd392,
      0x3a64c87c, 0xbd599294, 0x0000000b },
    { 0xa1da3c44, 0x5e94c2a2, 0x6c05a3e2, 0x90d95091, 0x74871eb1, 0xb29b791e, 0x50c2d855,
      0xe82a41fa, 0x1e689f42, 0xd2d24df9, 0x0cf6ed2f, 0xe9e93545, 0xc6ed3e7e, 0x5fc9d992,
      0x3606713d, 0xb4eb7df0, 0x0000004d },
    { 0x3fdd638d, 0x16bc3341, 0x8c518c30, 0x8c37c58a, 0x79df75d1, 0x9dddb8ee, 0xc91a620a,
      0x0bc82320, 0x9126a200, 0x6e924a6c, 0xd20c8c89, 0x744594a9, 0x4cc73cae, 0x0e9b59ac,
      0x0b276b4c, 0xce872f0a, 0x0000008b },
    { 0x9f004528, 0x8756b00a, 0xe9c2b2e2, 0x20374136, 0xcca54915, 0x9e6c6a22, 0xf6629736,
      0xc5dc6015, 0xc4510c8b, 0x5ef3efa8, 0xdcc2ce9a, 0x1fe7836c, 0x0b55a453, 0xac80124a,
      0xdbb7a64f, 0xe430cd86, 0x00000104 },
    { 0x378cab85, 0x11446efe, 0xbc48bf6d, 0x4c3c68cf, 0x64c19baf, 0x5e514661, 0xd5cbb135,
      0xc0f00735, 0xa693b1a9, 0x5d4844a7, 0x9579fdaf, 0x517a4871, 0x532c1e6e, 0x5ead064b,
      0x4135ecb1, 0x60207c2d, 0x00000172 },
    { 0x42df612f, 0x7b9409bb, 0x1bc66a3f, 0x32f47dd2, 0x23f375fe, 0x213f80e0, 0x06aa6c21,
      0xaa71c57f, 0x84298947, 0x5bf7b3a7, 0x5b608fa3, 0x001c2c96, 0x173c7f4f, 0x417619c6,
      0xd4a20677, 0xbcff66ac, 0x000000b3 },
    { 0xd4cd8948, 0xd8faa3bc, 0x9d2888d8, 0xddc43783, 0x581abba5, 0x1e951bd5, 0x2947d11f,
      0xa52dfab2, 0x728a40ea, 0x8fbb0ed8, 0x31d769cc, 0xd4f4d06f, 0xfac908d2, 0xd93b1d93,
      0xf5a3b0cd, 0x8852b438, 0x000000bb },
    { 0x3e206f1d, 0x327eec48, 0xfd9e0124, 0x20bae09c, 0x12fe91f1, 0x90e15a50, 0x359590d6,
      0x0dd6de71, 0x7346265f, 0xb47ff787, 0xd72b31fc, 0xf68c7538, 0x853e2029, 0x1210aa5e,
      0xf09c0c81, 0x22955a4d, 0x00000040 },
    { 0xda44ff51, 0xb342d85d, 0xd08596ff, 0x54ba0522, 0xb0ac99c2, 0x3da8e8ce, 0x91f37dcb,
      0xcebe6a96, 0xc24d7d1c, 0x8ebad3d9, 0xc75945ef, 0xf11a4319, 0xed1ac001, 0xd4dfe534,
      0x1a521e4d, 0x745e2772, 0x000000c4 },
    { 0x851368ee, 0xc03c182d, 0x57c919f2, 0xbb94a33d, 0xe333ace3, 0x9cf0c767, 0xd974de7f,
      0x037c7d47, 0x8b18fa5c, 0xe96e02cf, 0x26cf35f7, 0x6f1a3e91, 0x33486c59, 0x47affdc0,
      0xec8c2a2b, 0x2c370bfe, 0x000000e1 },
    { 0x31ff6e80, 0x4585d3aa, 0xc19ae65b, 0x8521ac35, 0xf9f78334, 0x983e0c6a, 0x435db0f0,
      0xc4131279, 0x51bdb67f, 0x0f2d350b, 0xd65d265f, 0x1fea5391, 0x798e80aa, 0x7f0b55f7,
      0xd549eaa9, 0xf322493a, 0x000000ce },
    { 0xcfd387ad, 0xdbeeb421, 0x5bb4d523, 0xd208fe08, 0xed24edf7, 0xff2282db, 0xc806b492,
      0x30556ff9, 0xd5b5bf57, 0xe7716c3c, 0x153fb3b2, 0x847e3a43, 0x391809c5, 0xe9a8fb3c,
      0x92596244, 0x52b4a729, 0x000001c0 },
    { 0x6debbcf6, 0xfdf7681d, 0x600ea3c3, 0xe21aef25, 0xbb30b75b, 0x181fb048, 0x8d8b7b4f,
      0xf9d8c43d, 0xca68ec32, 0x3d3bc6de, 0xaf1641eb, 0x6d469252, 0x10252291, 0x1be3c9ba,
      0x8fa4c10b, 0xb57f2c32, 0x00000155 },
    { 0x8f1ef579, 0x1e226a93, 0x138fedd2, 0x8aeebec9, 0x1ca4c4e2, 0xa7e39d7d, 0x49724f86,
      0x5e53f86d, 0xeb759db4, 0x9815987d, 0x0e9cd47a, 0x538c9f70, 0x1a7fac87, 0xfffc98c7,
      0x7ce980e1, 0x3195776e, 0x0000019f },
    { 0x7468e327, 0xfd026128, 0xee755906, 0x789e8dc4, 0x80218c94, 0xea0453e4, 0x088d7c56,
      0xe263fe27, 0x30277200, 0x9219cee6, 0x8ce5ad85, 0x293401d2, 0xb960ebba, 0x627036d3,
      0xf92da0ea, 0x8be032fe, 0x00000030 },
    { 0xbdec6efb, 0xe7bfc99f, 0x67326c7b, 0x2bd3d26b, 0x594c614e, 0x23f041ff, 0xec9b4420,
      0x5a778c67, 0x30eaa444, 0x75b5618b, 0xae6d50a4, 0xf8e2bb92, 0x1fdeb6f0, 0x7278c60b,
      0x97c654cc, 0x5b8471dd, 0x00000139 },
    { 0xc4c762bc, 0x0fc6c549, 0x4e5eb96f, 0xc43915e3, 0xd1bb0b5b, 0xad88ee68, 0x3f1f26a3,
      0x484a9fad, 0x2bc15e9d, 0x0d93055c, 0x06d26bb7, 0xa745abbb, 0x464f0731, 0xf60ed3b8,
      0xea9058bf, 0x963d8d94, 0x00000039 },
    { 0xd90119ae, 0x2bad7686, 0x464b230c, 0x8dcd64f0, 0xdad19cb1, 0x1d1eeb94, 0x86f0eb48,
      0x3b470389, 0x2c50f380, 0x42abb37d, 0xa7a99eb1, 0xfabbcebb, 0x33fb0b08, 0xfeed69dd,
      0x31dc2b1e, 0x8de8b50e, 0x00000057 },
    { 0xf6d20eca, 0xec9a8956, 0x684e0169, 0xeda0a24d, 0xe583c40a, 0xa55ae258, 0x455d3ce1,
      0xbe8c6286, 0x2096ae62, 0x05bf6313, 0x20b48bde, 0xf68e0249, 0x4d99cd14, 0x1b6595bc,
      0x774958be, 0x0e740830, 0x00000030 },
    { 0x24acb1fa, 0x36cdc3e6, 0x6c6d43ad, 0xf593d32d, 0x373c8e18, 0x766c2fcd, 0x3d12b7f3,
      0x0a151836, 0xb4234a24, 0x924427f4, 0x2335ff44, 0xaab87785, 0x7c362011, 0x9e581189,
      0x5ee724db, 0xd86f8fba, 0x00000184 },
    { 0x1733b3c3, 0x7ddf4705, 0xf4a25d85, 0x7c7878a3, 0x29ad09a6, 0x4d86f57c, 0xb26d5c69,
      0xa97d4ab6, 0x04b07913, 0x71cc9250, 0xb4c0b7b2, 0x08a84131, 0x5119f83e, 0x388fc8aa,
      0x6bfda0cb, 0x23a3bd0b, 0x0000007e },
    { 0x4175e7e0, 0xbce0f107, 0x1965464d, 0x51ac63d7, 0x52aaa9eb, 0x429f6b9f, 0x2a1846d7,
      0x6ed3c53b, 0x6dcda98a, 0xfa87d587, 0xc72d6d0a, 0x9858c801, 0x67681451, 0x94a5bcf3,
      0xd3de0285, 0xb7c46ecd, 0x00000183 },
    { 0x578b7a3f, 0x27068f87, 0x423a5346, 0x506446f1, 0x58dac5e4, 0xe9d9c75b, 0x61b8f695,
      0x8ebc86e5, 0x611c51e3, 0x272ac14e, 0x4e24fb32, 0x67863a1d, 0x02e5b539, 0x270d4fdc,
      0xa4186d8d, 0x96d7b7a9, 0x000000f4 },
    { 0xfcfefdb2, 0xe1f0c1ae, 0x44930c59, 0xb5bc33d3, 0x64d2c6e0, 0x253d5f49, 0xe316ba1f,
      0x25efdded, 0xe30c3a34, 0xeb3ab21f, 0xcfa52a9d, 0x797a1ad6, 0xa81ba831, 0x6a3ca2e0,
      0xe112dd07, 0x27bb664c, 0x0000007e },
    { 0x21dd6e7a, 0x0f8341fe, 0xea93e5c1, 0x5c106f8e, 0xad0114bc, 0x8e2ad8e9, 0xf059d291,
      0x277ef823, 0xdc50fd78, 0xed4a28e5, 0x89cdf20d, 0xa0719651, 0x1fde162c, 0xe393b264,
      0x30a5b16d, 0xd383b86d, 0x00000041 },
    { 0x2d3d7641, 0xdd916628, 0x39c02cee, 0x99029e9b, 0xb6de7881, 0xce03fac9, 0x4f66ebbc,
      0xc2cd0f78, 0x64c83e37, 0xe3919483, 0xfd7bb155, 0xfc853cb8, 0x432314cc, 0xa5a99b24,
      0x133b2709, 0xef0b7ed0, 0x000001d0 },
    { 0xb6bf5ec4, 0x1ebc5309, 0x3a92ba16, 0x29ca5ba7, 0x70839c14, 0xbdb1bce4, 0xde1f966c,
      0x963fd369, 0x4faba5b7, 0x208fdc75, 0x678fc163, 0x2f1ea65a, 0xa773bbc5, 0x792fa255,
      0xb22a653b, 0x752a08fa, 0x0000008f },
    { 0xdbbd171f, 0xa6d6b095, 0x44f06181, 0xf0d3b957, 0x78fab381, 0x46b97865, 0xae356e29,
      0x7da97845, 0xd01b3c09, 0x4a0c3e00, 0xe106d5be, 0xaed18677, 0x1affdb67, 0xd7309c76,
      0x25f2b8c2, 0x97158754, 0x00000193 },
    { 0xeaffd713, 0xa87650d1, 0xa797ad44, 0xec781c97, 0xb4bcac17, 0x1e94accd, 0xb3981cc9,
      0x2284f96f, 0x091634eb, 0xb8b2ef57, 0xcad053d0, 0x4accf5e8, 0x5d1ef480, 0x0eca4f9b,
      0xd6e76957, 0x1598ef27, 0x00000131 },
    { 0x9185544d, 0x6d9b0c3c, 0x8df2765f, 0xad21890e, 0xcbe030a2, 0x47836ee3, 0xf7651aed,
      0x606b9133, 0x71c00932, 0xb1a31586, 0xcfe05f47, 0x9806a369, 0xf57f3700, 0xc2ebc613,
      0xf065f07c, 0x1022d6d2, 0x00000109 },
    { 0x286492ad, 0xee31e71a, 0x65f86ac4, 0x08f3de44, 0xda713cb4, 0xe89700d4, 0xa86b7104,
      0x7ad0f5e9, 0x2572c161, 0xd9a62e4f, 0x25cc1c99, 0x77d223ef, 0x3b962e0c, 0xedff6961,
      0x81d8b205, 0x818d28f3, 0x0000008e },
    { 0x85edc827, 0x56f74e69, 0x6a731640, 0xafe2ef0a, 0xd51647d7, 0x61f8bcfd, 0x41e33fa7,
      0xbeabbb8f, 0xec83c547, 0x3e4c3aae, 0x2cc8dd97, 0x48a9d1e9, 0xe57d3822, 0x7b4d53b5,
      0xa45c0584, 0x92387ac8, 0x000000d5 },
    { 0xcecdf0dc, 0x0e13c1dd, 0xeeb9aa6c, 0xacd530b6, 0x0af8d3a8, 0xbe3c0fc8, 0xce44ae8b,
      0xee67f380, 0x3854b528, 0x9adc5636, 0x130f4497, 0x527f7f90, 0xe80c35ce, 0xee085713,
      0x190fae23, 0x29bc4b3e, 0x00000064 },
    { 0x07207165, 0x51db2d9d, 0xc3d4c59c, 0xc7d56a41, 0x4e278b0e, 0xa47fadda, 0x04ffc588,
      0x5c3964e2, 0x9719ecfd, 0x52bc9f38, 0x35a04049, 0x2c9292c0, 0xfae7580e, 0x885ff9ba,
      0xd5cc866e, 0x2319a2dc, 0x0000015c },
    { 0x0ac4cb66, 0x4c740ae1, 0x59c635ff, 0xc8a66d97, 0x2569dac8, 0x328b65af, 0x8838dbc9,
      0xfdb6599b, 0x7fd83c9f, 0x702473f8, 0x128683f6, 0x0d1f2ad8, 0x5030b76b, 0x00ae5b50,
      0x6d15b5ed, 0xb1375d70, 0x00000159 },
    { 0xdf5ee186, 0x3878c70a, 0x6fffef9f, 0x62f4f54a, 0x80400fa0, 0x0a7664c3, 0xd0ea21b9,
      0x99be58c2, 0xe311aa6d, 0xfa70b9ad, 0xabf43a4d, 0xafadd452, 0x0f7b5de5, 0xe6291c61,
      0x78d61e62, 0x388a4215, 0x00000024 },
    { 0x6e640a8d, 0xbf2ffcb7, 0xa0b5a07e, 0x63479501, 0x5f36f2e7, 0x55bb3001, 0x058a676a,
      0x9890f999, 0x455c0d27, 0xd9ee341b, 0x18e08fc8, 0x6c95780c, 0x442a075d, 0xeb63a675,
      0xfb69da00, 0xce46a1a5, 0x0000006b },
    { 0x05487074, 0x5ec6427b, 0x80c426ad, 0x0b59fa1f, 0xc9946594, 0x0207d510, 0xd38f83d0,
      0xba1fe985, 0xc96eb133, 0x113aec98, 0x82515193, 0x3424ad01, 0xfb6f9b10, 0x3eb15aef,
      0xf15d8be1, 0xccb719e8, 0x0000013c },
    { 0x448996e2, 0xe0828a5b, 0x516e440d, 0xdd3f4d9b, 0xab10e109, 0x223b4f87, 0x4ce7241c,
      0x766bde38, 0xdcaf88a7, 0x0b1cc320, 0xd75440d5, 0xb0470e7d, 0xba76b7e8, 0xe3894e41,
      0xa762bb9e, 0xa0341305, 0x0000013f },
    { 0x817221d9, 0xcdb8a92c, 0x780ed8a2, 0x7ff2643e, 0x56fc9caa, 0x5efcceda, 0xf823d2fc,
      0x59b5aa12, 0x9dbfeb54, 0xfe97818b, 0xa13dde72, 0x2becceaf, 0xe8692b1b, 0x5d5b53ce,
      0xcae9ddc6, 0xf5e5ac69, 0x0000000f },
    { 0xeec6e312, 0x163a9e62, 0x459162d8, 0xd11432c1, 0xa25d4934, 0xdde7d941, 0x8d868aad,
      0x9067028e, 0x5b6e3d21, 0xb71b1708, 0xe95d3c7f, 0xaed73637, 0x1dce0d78, 0x51bcc93c,
      0x504a9d0b, 0x2f45d06a, 0x000000be },
    { 0x67db4570, 0x358eb536, 0x685fa5ba, 0xaed3f999, 0xcb571fca, 0x4593c950, 0x0411dc5d,
      0xdff7278c, 0x3bfe7e51, 0xd62a4464, 0x49f6f05a, 0x53b87ca2, 0xf2c3d2dc, 0x72327899,
      0x9df4da5e, 0xbaa194aa, 0x000000c7 },
    { 0x853572e7, 0x92401ab5, 0xeddea020, 0x957d2ca6, 0x3f9fbd6f, 0x7d54dd65, 0x2e05d05d,
      0xad0c7d09, 0x04183b30, 0x6656fbab, 0x0b716d5b, 0x543b5017, 0x9857d796, 0x2481449d,
      0xe3a39722, 0x86173b6d, 0x0000005d },
    { 0xa03efd8c, 0x18cadddd, 0xaeb3a762, 0x9d59bb52, 0xd1fd3f91, 0x571d86be, 0xd2f26d6d,
      0x81642aa1, 0x99877251, 0x7d1c3bc9, 0xd7c2fd25, 0x0518eff8, 0x37b6c641, 0x6a54f170,
      0x0d33a426, 0x8ffedfc0, 0x000000c8 },
    { 0xab9923d7, 0x5f47ddb9, 0x8e3fac52, 0x839865d4, 0x647424b0, 0x0214e93f, 0x31d65c6b,
      0x266c909c, 0x51457fac, 0x956deca4, 0xa85ce8a5, 0x8e8d240e, 0xa4d9b281, 0x736652c4,
      0xb77a526d, 0x131a4191, 0x0000008a },
    { 0x569dfcd1, 0x5637ceab, 0xb2c13802, 0x0d1cf091, 0x731087b6, 0x09115a1f, 0xbdb45b2b,
      0xeda479fe, 0x8e6e5507, 0xed883a7d, 0x30bd8162, 0xbc9206ba, 0xc6294b24, 0xff2af28e,
      0xc8947bc9, 0x2058521a, 0x0000003c },
    { 0xc000e4f4, 0x4d1ba365, 0x7b2caa8f, 0xcf34a1a8, 0x783e3c39, 0x0e4a58d5, 0x2c1fcbd9,
      0x1ad4b5ce, 0xe8b9556c, 0x6b30bfb9, 0x6301b3a4, 0x304ad236, 0xc8d571f1, 0xdacf31b8,
      0xec93e088, 0x25b7a0fd, 0x0000012e },
    { 0xd34b138f, 0xbf0b5ea3, 0x3ac7cad4, 0xc4951381, 0xcab6a319, 0x82ca5536, 0x88bb4ea8,
      0xff569136, 0x4761d3e9, 0xcee2512a, 0x55ef9647, 0x4695e521, 0x9378f132, 0xbb83316b,
      0x8d43bb4c, 0x1d447f04, 0x0000002b },
    { 0xa79cb073, 0xadf5e06e, 0x8bf180e6, 0xcb46aca9, 0x835f8e22, 0x5bf0d9eb, 0x7713f3ad,
      0xd692cd20, 0xf0d87f74, 0xa33e9d7f, 0xf602a362, 0x9f2b6950, 0xdfa4f77c, 0x27a4c70b,
      0x3c56080a, 0x99c699dd, 0x00000105 },
    { 0x81fe2fbd, 0xd17bc059, 0xcdfc711d, 0x8247a0c7, 0x63ce74ee, 0x973eab2f, 0x4191b5f1,
      0x270a6bdc, 0x637f5917, 0xe21afb51, 0x50326c7b, 0xa84e71c5, 0xf30c35cf, 0x14cef332,
      0xb7407d5e, 0xff0e5f89, 0x00000011 },
    { 0x796de182, 0x9a579dee, 0xf6fd67c5, 0x6f780aa7, 0x6c566337, 0x8e2e0d6d, 0xe5314714,
      0xd8d685b5, 0x8099f308, 0xbc0bcb3a, 0x585ce610, 0x7127691f, 0xfe15ed8a, 0xb1dd4f51,
      0x217bd3c7, 0xab542e06, 0x00000183 },
    { 0x35210df1, 0x1e2df236, 0x9aace566, 0x387b5c56, 0xee401323, 0xdd8152c9, 0x9cac0076,
      0x56fe9ec9, 0xe1a9f782, 0x88ed7fc1, 0x681f0428, 0x6551487d, 0x97689006, 0xeaacc101,
      0x3b7e7fe6, 0x626bcad1, 0x0000015e },
    { 0x0261ada4, 0x674e392f, 0x5bb103a8, 0x5af762f5, 0xc9e8bf69, 0x281ad6a5, 0x5c57af63,
      0xa88ad2e4, 0x3b909ce5, 0x7fbce1f2, 0x4a8c86da, 0x37883da8, 0x8fc8357a, 0x94f7c570,
      0xc7c21ea8, 0xa1f2fb59, 0x000001ac },
    { 0xea2e4075, 0x58e8ca2c, 0xf271d545, 0x5307ba37, 0x7ae7ad07, 0x14de3b6e, 0x169efeb3,
      0x6bf3dac6, 0xf0996ac4, 0x7be665b6, 0x60a81ea5, 0x3cbd0766, 0xb0876044, 0x93369d84,
      0x8a489412, 0x33cd68dc, 0x00000133 },
    { 0xf8ece594, 0x9dae6d7a, 0xcce0b041, 0x7ad12f48, 0xe7374970, 0x9d97efc7, 0x311f5e2e,
      0x7b8b1668, 0xbb741640, 0x0d68a7b6, 0x9be65aa4, 0xf9ea3dea, 0xa11c78a5, 0xaa3e6811,
      0x0ac6e65e, 0x155ad4e5, 0x00000062 },
    { 0x91609a61, 0x1d40359e, 0x6890616d, 0x9adf4e08, 0x8b14a62e, 0x89dd6635, 0x49adeb20,
      0xfaa34c0a, 0x105a9bf0, 0x174689f7, 0xa83a1154, 0xbf727a3d, 0x0d6d5d50, 0x4afe90c1,
      0x1f2fe4ff, 0x9954d9d0, 0x00000126 },
    { 0x75804da9, 0x2a7e8ac1, 0x8d50bdc6, 0x0b41f611, 0xd8fafb1c, 0xcfa0a757, 0xd231a6b0,
      0x75540d94, 0x0c4fe03d, 0x524440fb, 0xce9738dd, 0xb8665fbf, 0x8f1bd64f, 0xbb74f6a8,
      0xbb8d6c67, 0x063afa3c, 0x00000189 },
    { 0x76bcd267, 0xa8c68b9d, 0x96362c64, 0x95f78ca8, 0x03603f09, 0xbb8302f3, 0x272622b9,
      0xf15b0d54, 0x97692826, 0x4cebfcd1, 0x19c758fc, 0x4d29d2b1, 0xe11ac4fe, 0x4703bb56,
      0x36b53bdb, 0x7152abd5, 0x00000060 },
    { 0xa2fda60f, 0x0079a845, 0xda8db63a, 0x6dfdf102, 0x0148a559, 0x7d275960, 0x684b9737,
      0xe089c346, 0x820bdb1c, 0x09f865b2, 0x891ae57e, 0x000daef6, 0xa4f64f15, 0x384f5eee,
      0x3da7e79c, 0xdd68e490, 0x0000013a },
    { 0xefbf6b21, 0x18caff50, 0x51f2ba1d, 0x0eeb65ec, 0x296c458b, 0x2cd8a43f, 0x9747ceab,
      0x04d795b2, 0xdc306c57, 0x52b0bca6, 0x17ec9c28, 0x39fe8ad5, 0x81a0cb57, 0xa4a9e3fb,
      0xa638d6f5, 0xdd6933a9, 0x0000017d },
    { 0x0bcf3b88, 0x442ebf30, 0xdc5bfd2b, 0x4b863857, 0xc50960c2, 0x632a1056, 0xc9d74020,
      0xdbf49444, 0x78287d19, 0x335f464b, 0x5c490a63, 0x0cf8d5c7, 0xd7d2d2c6, 0x4f9d9bd7,
      0xad52d8a9, 0x1490fb61, 0x000000eb },
    { 0x74ba879a, 0x51318ac9, 0xa68b584c, 0x23723b6d, 0x02ac441e, 0xa7872bf0, 0x65c7b9f4,
      0x1e0c8fd6, 0xb893449d, 0xe8e3bc24, 0xf06e02a6, 0x2e83c6b8, 0x78d50ef5, 0xba754c01,
      0x2c547c38, 0x7c6c06ef, 0x00000132 },
    { 0xa308e123, 0x5663f565, 0x157c2290, 0x91a48983, 0xc054408a, 0x222f7f93, 0xb839a520,
      0xd8694963, 0xd314da50, 0x864b491d, 0xd675cc09, 0xa315d96e, 0x390531b6, 0x32964930,
      0x21e11f6e, 0x93876719, 0x000001d0 },
    { 0x5a53c22b, 0x1bd8ce7b, 0x7cab446a, 0x78733fcd, 0x48acb394, 0xc44ca4e2, 0xa38c790f,
      0xa9888b1e, 0x15c34237, 0x36afb6eb, 0xfb702063, 0xb913b8a8, 0x917508fa, 0x34b77cc5,
      0xf9e4732b, 0xa931d7a7, 0x00000050 },
    { 0x90185363, 0xf5c1a4c6, 0x20c04ed5, 0x15f58fd9, 0xe913a1b2, 0xa7b46e0c, 0x36f1d8c5,
      0x1e7167c6, 0xbf5fcfb1, 0xf4fe6bfe, 0xa6c2027b, 0xc46faf10, 0x4e1f12f4, 0x45d59300,
      0x12185485, 0x46bcc873, 0x000000ca },
    { 0xe2eca03c, 0xfed9b067, 0xeb3230b2, 0x34ff6f2a, 0x31bc82be, 0xda83b96a, 0xcc89c862,
      0x3b138d2c, 0xebd59505, 0xf4a27aee, 0x8bb49dda, 0x29df153a, 0x50a555ba, 0x2dffec46,
      0x0fec5d80, 0xc899108d, 0x000000bf },
    { 0x512e743f, 0xbca656eb, 0xba4f85ec, 0xcd94fc24, 0x048bdf6f, 0xf4383ad0, 0x587f47a6,
      0xdb10e381, 0xda3ec391, 0x47f244e7, 0x0c33624b, 0x5f337e38, 0x82a3c375, 0x9259425f,
      0xf276bf73, 0x509e56c3, 0x00000014 },
    { 0xe63885d6, 0xf7593760, 0x86e546ca, 0xce5054bb, 0x6e9567b2, 0x5910ad52, 0xe420aff3,
      0xe812cf7b, 0x904e9676, 0x266049fd, 0x2c8b34ad, 0x42d0e2ab, 0xa4382f29, 0xf79f8f91,
      0x41d531de, 0xf8cde3ed, 0x00000035 },
    { 0x8fe6f511, 0x141838bf, 0xee745e2e, 0x8e9ff343, 0x02325306, 0x71da9fd5, 0xf0d0d68e,
      0xa527b9ba, 0x31144649, 0xce19b22c, 0x0bfaa46c, 0xca9b778b, 0x6ce31145, 0x66e689a7,
      0xd8d1da21, 0xed43460d, 0x000000bf },
    { 0x10a8c4fb, 0x73a6ba38, 0xecc93e5d, 0x5153d959, 0xb59e9871, 0x7ca58012, 0xafd442f1,
      0xedc0dbef, 0xb9cf7691, 0xb9050a22, 0x464d017d, 0x3d1e96fe, 0x82074dca, 0x541781a4,
      0x8b355413, 0xedce0db3, 0x0000006b },
    { 0x4e1080a4, 0x7d6e4240, 0xe1adb078, 0x5cf6c427, 0x3d594e28, 0xe74ec603, 0xbdf853f2,
      0xc0185404, 0xc3335717, 0x287d94de, 0xc3179807, 0xf735656a, 0x604e93f7, 0xd356f2bf,
      0xcbe27815, 0xb1fdc2f2, 0x00000151 },
    { 0x7d874400, 0x8af346c5, 0x71098f45, 0xd9e98cb5, 0x79565740, 0xdb400810, 0xed9a7dcf,
      0x0f7625eb, 0xc93d2542, 0x5d1415d0, 0x688d0692, 0xf87f74b4, 0x9f648523, 0x9fa2e31b,
      0xcb387129, 0x4e505024, 0x00000006 },
    { 0xd6e1bd72, 0xa66cccc8, 0x4ae2fa5c, 0x1ea4e921, 0x2d021fa7, 0x66775af2, 0x2a4c4677,
      0x6d711dfb, 0x5ae380b3, 0xddaae8bb, 0x600f11fc, 0x53158994, 0xda94d443, 0xfeb122db,
      0xdffc21dd, 0x7cbf1afa, 0x00000006 },
    { 0xee32e2ff, 0x6985bbd6, 0x3c35a00b, 0xe69f0f22, 0x0ede3c1b, 0xb1abb4ae, 0x93d96532,
      0xa3d7ecde, 0x4f693cab, 0x69eae259, 0xfe3665f9, 0x52e38b65, 0xf2fa7a85, 0x17664e93,
      0xa5300907, 0xe1a2febf, 0x00000000 },
    { 0x4c7f31f1, 0xa0f946b3, 0x004b810d, 0x122b97ec, 0x831a9cb2, 0x98f876d1, 0xaebd922b,
      0x6690030a, 0xa21b0412, 0x12a235d1, 0xe097208f, 0x5ced8104, 0x49d32fb6, 0x049b33ed,
      0x87b187eb, 0x2e88d762, 0x0000007f },
    { 0x8114885d, 0xbf07e90d, 0x9f39a562, 0x76d513b2, 0x7e5446a1, 0xd98ae4f7, 0x6264b6c0,
      0x7e3314e9, 0x6f3a7560, 0xe634dee7, 0xa08d539e, 0x1dfa2392, 0xbdae3c53, 0x45764812,
      0xd824e9ac, 0x858b836f, 0x0000003e },
    { 0x8f69079e, 0x6d7eedc4, 0x4f03b92f, 0xf02e8e40, 0xcb76b181, 0xdf751116, 0x234c1925,
      0xc255eb2c, 0xa94af9b2, 0xaf997d5e, 0xd1374a41, 0x797f3df0, 0x23986cda, 0x357fd20b,
      0x1540adb8, 0xf1caccda, 0x0000007d },
    { 0x6ea09a93, 0xd7c0efa3, 0xfb01c2b3, 0x95f69e09, 0xd43150e2, 0x021b1721, 0x5879bc51,
      0x39ff07e0, 0x6cf50ce3, 0x0dd73c32, 0x5758870f, 0x9d69d32a, 0x1c0e91d9, 0x5df4cc8c,
      0x15df54ab, 0x00335088, 0x00000182 },
    { 0xd6a37bab, 0x4331cdcd, 0x85c741c8, 0x67b89137, 0x4fc72389, 0xaeffcbdb, 0x1c7ae6dd,
      0xcc12e59a, 0xee1e838b, 0x4a35540b, 0xecd3e84d, 0x6148fb60, 0xb2290730, 0x7319e868,
      0x4de5208b, 0x4321baae, 0x00000160 },
    { 0xc4dd900c, 0x6ad03106, 0x7ca08923, 0x8227fb5d, 0x863ba5d9, 0x2e3c1e7f, 0xb5be20d0,
      0x5f76d6bf, 0x9492569a, 0x841dc1d2, 0x47e0dc0d, 0xfda5b20e, 0x2d720dc5, 0x65aba106,
      0x03e6765e, 0x9eab8ffb, 0x00000046 },
    { 0xcd5f918f, 0xe12d8922, 0x49f73f1b, 0xec7c1763, 0xc744eb3d, 0xcc5a3a14, 0x782dbda1,
      0x2b96f342, 0xab68dde4, 0x87284905, 0xe720668a, 0x647354b9, 0x739c4393, 0xe07da694,
      0x2cc145f9, 0xbae499bf, 0x000001ae },
    { 0xab7a841f, 0x2bf35f94, 0x78e3fb0d, 0xdcbda338, 0xa0195f98, 0xb913947a, 0x4370e51d,
      0xc8e30a9c, 0xf0e558ee, 0x8a403e1f, 0x606db1dc, 0x7dd167c4, 0x5827ef91, 0xaa343cbe,
      0xaace0f43, 0x1404f2d6, 0x000001ac },
    { 0x5f7abaea, 0x02fb84fd, 0xe991e445, 0xa238ad04, 0x7c972add, 0x0d9b2d41, 0x478c558a,
      0xac2439f9, 0x1ec86ed8, 0xc3c78a2c, 0x2d3b2f29, 0x3735839b, 0x50317454, 0x259728d4,
      0xe2377aba, 0x7b53a9cb, 0x00000179 },
    { 0x09b5947b, 0xc7cc50fb, 0xd5eae02f, 0xe166a9af, 0xda349c90, 0x47c7953f, 0x21c0984d,
      0xc3d4bcae, 0x45ff2419, 0xcee13c2f, 0x47780ddc, 0x334cc1f1, 0xf1157180, 0x21381fb6,
      0xeaede87b, 0x28ff4f86, 0x0000017d },
    { 0x3fbadc57, 0x62a1f73e, 0xb1ba2d35, 0xeabf4b68, 0x76f3bde7, 0x711d7f63, 0xb31c3fb3,
      0x162f5183, 0xd7d716f7, 0x0dcd66fe, 0xe46816d9, 0x6e1dbb78, 0xf833a6f7, 0x532831b8,
      0x4cfed0cf, 0x64191ccb, 0x00000190 },
    { 0x13b42e63, 0x651b47c8, 0x16bee2b6, 0xe7054413, 0x86a71ac0, 0xdcf60ca3, 0xe3baf787,
      0xabddb357, 0x81a2db2d, 0x99d631d1, 0xc8c9b834, 0x3f1e3c7f, 0x61adf4d7, 0x3b30fa18,
      0x14b9f46b, 0xd1d8d980, 0x00000139 },
    { 0x66bc31a5, 0x8dd33d1c, 0x9d19c1c0, 0x80a07495, 0xe03a1ee5, 0xda13c485, 0x68acfd38,
      0x316d85f9, 0x0d9d6273, 0x8617c80d, 0xf94d5f22, 0x6ddebb71, 0x8efd0a44, 0x1c345203,
      0x7b3746e3, 0x7ca0b406, 0x0000007b },
    { 0x72e2518a, 0x61064029, 0x7b5fbfc1, 0xfacd33fa, 0x7632ad67, 0xd50f7771, 0x1f0d2282,
      0xba868807, 0x04d90803, 0xacacd911, 0x51496ee8, 0xf94b3e0f, 0xece3c365, 0xfb8a5406,
      0x620400f2, 0x96f5c5e7, 0x00000109 },
    { 0x945039ac, 0x0ce1f2d1, 0xfdf94d41, 0x1d1e6504, 0x8752d630, 0xf36eafd5, 0xefbf0132,
      0x24477f7f, 0xc02bf85b, 0xae8fba2c, 0xeb510e61, 0x53a24ca8, 0x9c2c453d, 0xeadbc7f7,
      0x310eda46, 0xcfdbc9dc, 0x000001a9 },
    { 0x3c80f820, 0x4270a797, 0xd8e3db55, 0xe1455e1c, 0xc31eea36, 0xfc7d04b5, 0x8237619e,
      0x86d4b43d, 0x5696ac8b, 0xca90fead, 0xbe0a5d49, 0xd1df350d, 0xaea06270, 0x844e818f,
      0xd89cc1dc, 0xfa19b70b, 0x000000fb },
    { 0x9da7bb54, 0xfc5ba8d5, 0xa7c7fbcb, 0x29f3e817, 0xa40ad3fc, 0x31a34412, 0x2d1711b0,
      0xe17b9369, 0x16ee6505, 0x41c7bc17, 0x3d1db8ca, 0x5feecffd, 0x02f9e04e, 0xcbb93378,
      0x091db41e, 0xaeb67b8b, 0x00000187 },
    { 0xa3a2be76, 0x312dc896, 0x61da109b, 0x9e77c0c3, 0xbf0993be, 0xb1c5379d, 0xb7bcc4dd,
      0x9272b9e8, 0x09337d54, 0x5437f625, 0xcf735302, 0x5c897a61, 0x002db288, 0x368e21ca,
      0x184aee4a, 0x33197cc1, 0x000001af },
    { 0x56b092a2, 0x22b8eccb, 0x736ea9aa, 0x0a47d70e, 0xd64a7d08, 0x3eefc772, 0xbb2e0f14,
      0x85408cec, 0x285cb70e, 0x73047afd, 0x63297a2a, 0x1732f016, 0x4cf37eac, 0x4dfe656a,
      0xc0ec357b, 0x147d6d66, 0x000000c7 },
    { 0xdd04afce, 0x1d45c5c8, 0xbbd9c392, 0xce141a50, 0xc9e0fc05, 0x44ca5fa5, 0x60ccfb3c,
      0x4b9496ff, 0xdb5552a6, 0xbe260bc6, 0x7d3a7a09, 0xa0eabfdd, 0x1c77c1f9, 0xe9cc06ae,
      0x2aabee9e, 0xfa792d04, 0x000001db },
    { 0xce8f9a17, 0x61829972, 0xe0fda47e, 0x2d21913c, 0x52466c05, 0x84cb9da2, 0x9fd85710,
      0xfcc4eaab, 0x2f56fb90, 0xab1ec541, 0x6a231d4a, 0xbb435bec, 0xe16d3c47, 0xe1fffeae,
      0x0f023a36, 0xa7db0284, 0x0000002d },
    { 0x0e6a4ece, 0xcd6151c2, 0xa2840752, 0x884e5e75, 0xc41b64b7, 0xa6752672, 0xd5cd2b79,
      0x7585f7cb, 0x892fb84d, 0xac8f7a1c, 0x32a80f6f, 0xfdb20c74, 0x39c7ecfe, 0xaec25313,
      0x93b1e75b, 0xc84d7c5c, 0x0000011b },
    { 0xb7ad6f5e, 0xe5f78ee5, 0x8e726f3e, 0x3302bb5a, 0xa4363476, 0x6a3cdd24, 0x5bf5dab4,
      0x88c67b31, 0xa8d14f9a, 0x65922f93, 0x0d4bb945, 0xd9a180d2, 0x3d5c0170, 0x17806ba2,
      0x6252fe3c, 0x3e7b3c10, 0x0000015f }
};
static const DIGIT local_py521[133][SECP521R1_WORDSIZE] = {
    { 0x9fd16650, 0x88be9476, 0xa272c240, 0x353c7086, 0x3fad0761, 0xc550b901, 0x5ef42640,
      0x97ee7299, 0x273e662c, 0x17afbd17, 0x579b4468, 0x98f54449, 0x2c7d1bd9, 0x5c8a5fb4,
      0x9a3bc004, 0x39296a78, 0x00000118 },
    { 0x9bc73abe, 0x52e272f8, 0x8d186333, 0x9e3004d2, 0x9d998852, 0x0b3075ac, 0x8db547aa,
      0xc182e139, 0x4b6b82c1, 0xc7ca30c9, 0xb800d50e, 0x1335e4c3, 0x3d3e6972, 0xc714b18d,
      0x2d77001f, 0x37343c58, 0x0000007a },
    { 0x486bde7f, 0x8fac1245, 0x4d4a7784, 0x1430cd50, 0xbd29a67b, 0x58d2b99d, 0x38e3bceb,
      0x965cf657, 0x57354f17, 0x062dc3aa, 0xe5b45635, 0x40ed2a81, 0x103b69c5, 0x981bba93,
      0x413c42fc, 0x3503aa73, 0x0000008e },
    { 0xcc9c279a, 0x1ff2439c, 0x90bab4cf, 0xdfcd0b6e, 0xc3acbfda, 0xfa08a7ba, 0xcdb22a56,
      0xc91b8f40, 0x672be103, 0x9624ac18, 0xa7cb0f83, 0x1e59bac0, 0xed5f94c6, 0xae0aa78e,
      0xb43029e1, 0x9655b8c3, 0x00000052 },
    { 0x48219a09, 0xc865844f, 0x37426bdc, 0xa6d0b179, 0x284ae0e6, 0x7386a0a6, 0xc2dc16fc,
      0xddd6cba0, 0x2940d262, 0x77d2934d, 0x741b88ab, 0x3af6ed35, 0xa6dae43f, 0x96f43a65,
      0x1414083b, 0xc7143535, 0x00000057 },
    { 0x8e2d6ab2, 0xf9cd8299, 0x6c27631d, 0xa75b296b, 0x1e00bac9, 0x07700049, 0xebd66975,
      0xdc99e546, 0x75e54993, 0x44fa5282, 0xd50e44d1, 0x25fbca4b, 0x81057b1a, 0x46631393,
      0xbdcdf45a, 0xc22eff26, 0x000001cf },
    { 0xd9c31e98, 0x5bf75682, 0xc8bce143, 0x74d5cd73, 0xc700e94a, 0x778bd5f1, 0x3a66b233,
      0xb01480c7, 0x688df0cc, 0xc667f078, 0x16bbcf57, 0x780c73e4, 0x485890e0, 0xb77f5ba9,
      0xb9ddacb3, 0xd62addf5, 0x0000017d },
    { 0x37316056, 0xdb18d7bf, 0x8fc0eb89, 0xc3453204, 0x060e2db1, 0x3929db79, 0xb1e67403,
      0x2ccbf668, 0x9b05f538, 0xc7ce019a, 0x4dca0ffe, 0x9377f446, 0x8a570683, 0x42947eb6,
      0xe0a1bf92, 0x6eb25e34, 0x0000009a },
    { 0x46bbc412, 0x23bb06a5, 0xd0135eb6, 0x15718699, 0xb4ed9e10, 0x8f1639c4, 0xd7f644a0,
      0x767fee42, 0xfe8ec420, 0x668267b0, 0x64e90133, 0xec4f5482, 0x536d132b, 0x679e614e,
      0x174df6c5, 0x0d72fbaf, 0x000000a5 },
    { 0xcdc4cff1, 0x3f277e18, 0x6eb7045f, 0xcad81ada, 0xbb804212, 0x3f82d6da, 0x03b6a618,
      0x7eca8375, 0x95f2970d, 0xda887969, 0xeb3a7834, 0x21fc350f, 0x55f9ac1c, 0xfc332b6e,
      0x18d4111a, 0x0d85b434, 0x0000003a },
    { 0xe55948cd, 0xa2a53148, 0x0cc0a0b5, 0x2d952f50, 0x61839836, 0x26f77e71, 0xad87bb45,
      0xa8bf589a, 0x6a9296e4, 0xaf21e0fc, 0x7d394e96, 0x91788120, 0x68622361, 0x47f3d389,
      0x090a8cb2, 0xf17141da, 0x000001b6 },
    { 0xdc5feef6, 0x6b372ca7, 0xd06696b6, 0x7557c331, 0x4a42090f, 0x4b10d90b, 0x6f10352b,
      0x2c93e984, 0x2b8ded01, 0x72ac24ff, 0xd8c833a6, 0xfa53f5da, 0x7c9354ca, 0xc166ed4b,
      0x9e3b4cf5, 0x81df59a0, 0x000001fa },
    { 0xa4b301ed, 0x45e2ae85, 0xe1fb6112, 0x0640d46d, 0x83fc8f4b, 0x7f102d19, 0x4f30038e,
      0x6eeece3a, 0x912f39c3, 0x74d062f5, 0x16906c7b, 0xa802a553, 0xe6d6e8c1, 0xa8d8c58f,
      0xa57b4387, 0x77cfd9bc, 0x000001f0 },
    { 0x566098b5, 0x965caf8e, 0xafbd8436, 0xe8ba0e81, 0x7299fbc1, 0x77f4e27d, 0x367e2155,
      0xf5169f5c, 0x01a49a6f, 0xc88fa561, 0x557b66de, 0x8a6a0228, 0x8e1bee40, 0xc867f1fa,
      0x6de8fced, 0xbf4b5f80, 0x00000042 },
    { 0x9f78b57f, 0x825ffb60, 0xe0f74b73, 0xd1913668, 0x5468f89d, 0xf42a5b03, 0xaacae46c,
      0xa2347499, 0xc85272e8, 0xc771e34c, 0xd427d3a1, 0x295684c1, 0x433451dc, 0xed749903,
      0x0cbabbd4, 0xb872072c, 0x00000049 },
    { 0x39c8824b, 0xcdbf1daa, 0x5638cb0c, 0xf8f2cd8c, 0xb6d33286, 0xd966c8e9, 0xc1331e1f,
      0xe4b63ef8, 0xfb5f843e, 0xac72d8e3, 0x8ff6b4b6, 0x110d0d4a, 0xacd5cf3b, 0x0067fe1e,
      0x21c1e826, 0x5eaae5b3, 0x00000068 },
    { 0x5ac043e6, 0xefe1469c, 0x7d273130, 0xc86b7721, 0x2606809a, 0x2324af0c, 0x21800fcb,
      0x33a3324e, 0xa6b0d999, 0x8a38f30f, 0xd038b182, 0x12db51d0, 0xb3490ff5, 0x53ca86cc,
      0x7a935163, 0x2ecd7ecf, 0x0000016f },
    { 0x7ec6bcea, 0x1eb0707d, 0xebe7eed0, 0x341b0f22, 0x37333b3e, 0x99c3c50c, 0xde463e09,
      0xdcd5faeb, 0xe481cdd0, 0xe688ed5e, 0x0fead808, 0x6fc12bd1, 0x29746a50, 0x7b8a4983,
      0x829e8d3c, 0x2be1b92f, 0x000000a8 },
    { 0xb85445eb, 0x46ed4b2a, 0xc8124af3, 0xd272162c, 0x675589f1, 0x1b7e446d, 0x9e56e455,
      0x605b2806, 0xbd0d72ef, 0x39f9b1ab, 0x793f00e5, 0x52d79f51, 0x7978077c, 0x28d3e23b,
      0x4866c9d8, 0x9759e0e6, 0x0000010c },
    { 0xe6cd3ba8, 0x0db5541a, 0x8c0bac0e, 0x5504d521, 0xcfac5e4c, 0x32f61c5c, 0x8538bd21,
      0xb8e644b5, 0x51fc9949, 0x8517c72e, 0xad6f4e8d, 0x8d47a70c, 0x17ab479e, 0xb158e063,
      0x74f01418, 0x8c565f7b, 0x0000002f },
    { 0x2e9c881e, 0x6f4741df, 0x43f26343, 0x2656911d, 0x1c82e8e5, 0xebae8ae4, 0xf3900776,
      0xceb1ca37, 0x16721332, 0x07f2f8fe, 0xaad4911b, 0x59265673, 0xfc3bda3f, 0xe8799eab,
      0xfd073355, 0x55805723, 0x00000088 },
    { 0x49642f41, 0x7ed35bc3, 0x22ee3354, 0x69c8bc46, 0x1911836f, 0x52846623, 0xbf4b87e2,
      0xdb193baf, 0x4209f1e3, 0x427ff5f4, 0xdae27044, 0x4152fd13, 0xa1f0ae7d, 0x3b73f059,
      0x6f1503e1, 0x621ace50, 0x00000196 },
    { 0x9d38d508, 0x86999f15, 0xfb204d42, 0x0c9a990a, 0xcb616f5b, 0x27c4421d, 0x46ae31ec,
      0x2cb6590d, 0x309acf22, 0x1a33f6ab, 0x80320210, 0x13d99e2a, 0x341d1667, 0xa13731b3,
      0x4bfcc116, 0x7e0baeba, 0x000001f0 },
    { 0xda533799, 0xdeb81330, 0x3996b5be, 0xdc4de9f1, 0xd33f04b5, 0x6a8b7630, 0x2bccad83,
      0xa4554acb, 0x2a7f5d61, 0x7974bd47, 0xb01696d5, 0xc481a752, 0x8ca48da9, 0x6dfbefde,
      0xe7db0c27, 0x2fda492f, 0x00000053 },
    { 0x9ca40933, 0xf11b8e76, 0xd06857c8, 0x1951cd54, 0x29f13914, 0x0e9124f6, 0x3d67d0cd,
      0xe6ebbbe3, 0xbf16f9c2, 0x5483a2bc, 0x255ea8ca, 0xd4e4ea52, 0x734eef50, 0xee0db0b5,
      0x16207775, 0x54434b51, 0x00000017 },
    { 0x96aa1400, 0x84a08542, 0xcd72b4a4, 0x871885f1, 0xe9d3027e, 0x00954202, 0x5c86a302,
      0x19b35069, 0x6cfc52a2, 0xbe420b63, 0x87175b19, 0x8b752282, 0x9a22611d, 0xd1811b8c,
      0x3e468601, 0xf9c23d80, 0x000001d2 },
    { 0xdaa8dc76, 0xda4f7464, 0x919e1298, 0xa5b83150, 0x81c82c13, 0x00d3948d, 0x63724cde,
      0x95def324, 0x7d5a1038, 0x9b57597b, 0x57432b3a, 0x42d9d64e, 0x936e411c, 0x9807d998,
      0xf1e57507, 0xe68eefac, 0x0000003e },
    { 0x6d9ebcad, 0x483b1f8c, 0x987b2c42, 0x25f3aadc, 0xf70950d0, 0x83a7bd03, 0x14f250ee,
      0x67aefd6a, 0x90070d75, 0x6e90c4f9, 0x035fdd1e, 0xb9d65432, 0x2ab23f7e, 0x190281bb,
      0xb69015da, 0x36008cb1, 0x00000137 },
    { 0x40260e22, 0x1973c88b, 0xe48dca69, 0x83078cb7, 0xb199e3eb, 0x4815d67f, 0x7b4de5d3,
      0xf57bde1f, 0xe139dc10, 0x88a1ebb5, 0x6689dd21, 0x7c20e7c6, 0x80884875, 0x40cba346,
      0x39171e05, 0x5c821402, 0x00000062 },
    { 0x0aed0c43, 0x1eaf3d40, 0xea2a20a4, 0x7a7196f7, 0x97781948, 0x43123414, 0xe88d93fc,
      0x856126d0, 0xd5710224, 0x96dac31f, 0x243e6395, 0xc23548f5, 0x9613a79e, 0xf351df8b,
      0x626fb1fd, 0x0dae5d50, 0x000000e2 },
    { 0xfaada7a3, 0xdca7480b, 0x0d4fb693, 0x203303d3, 0xb9ef7822, 0x0ec1c4d2, 0x594552ea,
      0x2b97216d, 0xb59c5490, 0xb93e533e, 0x0d6c3787, 0x50d31ad6, 0x5cb84476, 0x0f1014d6,
      0x02b816ad, 0x0523796c, 0x000001fa },
    { 0x9548d9c4, 0x6a00191e, 0xd9716285, 0x800f66bc, 0x15dd9859, 0x3a8c05a3, 0xac571627,
      0xaf6dcdf5, 0x006a2a90, 0xeae07417, 0xfcb7c955, 0x11b368fb, 0xc4abe2b7, 0x5e47618f,
      0x5ab9dd68, 0x328f44c8, 0x0000015d },
    { 0xb5d073b0, 0xa71532ad, 0xb4fb97b3, 0x37159fb3, 0x7a1fff6d, 0x15b655b2, 0x9e08a89c,
      0x55ac89ee, 0xbc027776, 0x13cfab6b, 0x9dece9a9, 0xa57a7f28, 0xc03a1d45, 0x65a3d815,
      0x95515dba, 0xe5cd61ea, 0x00000100 },
    { 0xfa8b51f0, 0xf103ff6d, 0x40e2bdf0, 0xae7afb51, 0x83254171, 0x1130380e, 0xcda10d95,
      0xe83501b8, 0x4f3a8c01, 0x1057771e, 0xac807069, 0x8f52196a, 0xa5623821, 0x3609b0aa,
      0x94a0a7f1, 0x8c257906, 0x000001db },
    { 0xfcdadfeb, 0xc0899758, 0x54319374, 0x1638c314, 0xc8ae469c, 0x1831028e, 0x3a0e36a7,
      0x41093ffe, 0x2860701a, 0x75c6667b, 0x64de0a06, 0x38c843a2, 0x949389d1, 0xc6e2c8a0,
      0x3a1cc818, 0x754f22b7, 0x000001fe },
    { 0x01bb03f8, 0x5e5e1239, 0x5304ba94, 0xd36c3583, 0xb49f6a95, 0x5025f960, 0x75e5b6e3,
      0x72dd554f, 0x869c9d17, 0x85bfb398, 0x9cace4a7, 0xd71cf71f, 0x7291dd9d, 0x0f6fd3ad,
      0xd1b5758f, 0x5da8a689, 0x0000018b },
    { 0xf6ea9cf1, 0x8623bdbb, 0x547aa650, 0x3aad9495, 0xbeb27159, 0xd3d853fc, 0x30b40833,
      0x3d25a648, 0xdec51bd1, 0x12d188e8, 0xb348c3fa, 0x836330d2, 0x73c2ea59, 0x9df50cfe,
      0x61ded0b8, 0xb5dffb20, 0x0000018a },
    { 0x2099171a, 0x248fd9d7, 0x1d16d66c, 0x908d539e, 0x171bfd3b, 0xab47b9eb, 0xecdd8e37,
      0x70e5de8c, 0x51eed557, 0xedd44ae0, 0xad95aeb2, 0x4983c6fa, 0xcdc8214f, 0xdedbfea1,
      0x131549b3, 0xc6ae2030, 0x000000c9 },
    { 0x2c35933c, 0x651f7422, 0xe025609e, 0x95238355, 0x6c14d96e, 0xe7891b5b, 0x0304d219,
      0x89c0fa37, 0x61b597ef, 0xdf7da33d, 0x4120905c, 0x43df0357, 0x718df73d, 0xacf01355,
      0x631ada96, 0x7ca32d0e, 0x000000b8 },
    { 0xa385741d, 0x939b5896, 0x3a6c90e9, 0xa80c7d7e, 0xe4c02621, 0x8df754d6, 0x72958d91,
      0xccc73ff1, 0xe911d8d9, 0xa80cd923, 0x8c0171e8, 0xee0dca49, 0xfecadde6, 0x850d8439,
      0x4f714086, 0x37550fc2, 0x000000c8 },
    { 0x843dc902, 0x4b033a37, 0xc27a9863, 0xb2d3664c, 0x18280e54, 0xf4ebdc35, 0xae02b757,
      0xac7789a6, 0x10dd96f8, 0x50084ba4, 0x1f5f6349, 0xcc345937, 0xfdee6f84, 0x5f65aca8,
      0x02d841cf, 0x4aac5f59, 0x000001b9 },
    { 0x0603bc79, 0x81b9b564, 0xf875e591, 0x8f39155e, 0xdfd32c0d, 0xf6a508fd, 0x6b0dac71,
      0x7eace6f9, 0x9e4c1eb3, 0xe40163d8, 0xff10fa6d, 0x265226c5, 0xda76d272, 0x1e1af5cb,
      0xa10dc788, 0x954bc056, 0x0000012c },
    { 0xc8c88de9, 0x84ea0e03, 0x63cdb21f, 0x621a8088, 0xcd6ec216, 0x332c292e, 0xc4cd0dca,
      0xfd01ee09, 0xfdfce95c, 0x5354ec9e, 0xa6b8db30, 0x05c307b2, 0xb44784ae, 0xf7b254ea,
      0x7e61408a, 0xd3848125, 0x00000128 },
    { 0x7335f4a6, 0xe194dba0, 0x4bde2f58, 0x9d6532fb, 0x9016930a, 0xfac563c5, 0xaf9e2095,
      0x626eaeef, 0xb7d9fde1, 0x7af29b71, 0x36360bbf, 0x809b7fe4, 0xe3cd1c8e, 0x57b5e52d,
      0x37b01251, 0x52b90409, 0x00000032 },
    { 0xdf65e0bb, 0x745e5b41, 0x3b84ed58, 0xf8887b53, 0x9c0684ea, 0x1a54bc47, 0xce9c7672,
      0xfd4f1d36, 0xce5730fc, 0x490bc265, 0x8430adb9, 0x2fa1ad6d, 0xb7db19b4, 0x0f117bdc,
      0x9c246e44, 0xbd339a78, 0x0000005b },
    { 0x18a3f84b, 0xdce5de2c, 0x2ad42ca2, 0xc4bb672e, 0xdd0731b2, 0xb6581d7e, 0x2809ac7e,
      0xb6ca46ca, 0x189b0ff8, 0xde8edba2, 0x1ff4631d, 0xa72ee3e8, 0xf1004bcb, 0xf44a8fc6,
      0xc8878531, 0x103dd1b9, 0x000001ca },
    { 0xfe9e5dab, 0xdf7b80aa, 0x1861e785, 0x5585f93a, 0xbbb5764c, 0xb70fa9ab, 0x3a39ef40,
      0x23ce8aed, 0x14920ff9, 0xa719329a, 0x4a00e5b9, 0xa7451112, 0xff5c3588, 0x48b0001b,
      0x0e0551ce, 0x8dcb5fe1, 0x000000f1 },
    { 0x59d9ae12, 0xe49b344e, 0xc925b6da, 0x5a5e32a1, 0xf054a1c6, 0xe81d38d1, 0xf15fb6d7,
      0x4ae4fe0c, 0x5cf03296, 0x6ef46dcd, 0x12da300d, 0xbf46b976, 0x35fa7fdd, 0xfb373fe8,
      0x62bcaf82, 0x0f78e5ee, 0x0000003b },
    { 0x9ea96c94, 0x599e10e5, 0x40593ce0, 0x4426deb7, 0x2d48d29f, 0x948d9286, 0xc03f5cc5,
      0xd3ad8e54, 0x219cd0d4, 0x53fe15e4, 0x0c8fc479, 0x03b91421, 0xe5aa96e6, 0xa023c978,
      0xedb69f81, 0xdee00502, 0x000000b7 },
    { 0xe33cdcbd, 0xccab7b37, 0x855619d0, 0x561d18f2, 0x818f21d0, 0x56aa5963, 0x571c3c5e,
      0x1413193b, 0xfd748c72, 0x9b30214c, 0x8eadf936, 0xef0bd8d4, 0xd1964b92, 0xeb6d831f,
      0x9bc42beb, 0x4b37d5e0, 0x000001ef },
    { 0x9daf8d6d, 0x7ab52e73, 0x3f789f6c, 0xd6d75d4c, 0xbfc9a31e, 0xc3b7560e, 0xd7db7ad5,
      0x87b7f183, 0x9c213425, 0x0f60a2db, 0x52c58981, 0xc2e67c94, 0x1b6d0b43, 0x001cf3e8,
      0x78235b3f, 0x31efa39b, 0x000000ce },
    { 0x3e959ca8, 0x115a8112, 0x8bed7eee, 0x13dc4aa3, 0xef94c5fd, 0x02340f86, 0x99a5d5cc,
      0x519293a4, 0x294349a0, 0x09573381, 0xda33b39f, 0xdefe063f, 0xa382a806, 0x6a18640a,
      0xd227a86f, 0xb3e25fdb, 0x000000d3 },
    { 0x7b67b298, 0x4608ac08, 0xc85cf812, 0x3baf2ae6, 0x0d25bedf, 0xb30aec55, 0x7ce48459,
      0x58149350, 0xd3c41e2d, 0xfcb39f45, 0x126e11a7, 0x48d241a0, 0x909a1621, 0x7b425421,
      0xe746bf4b, 0x9bb349dd, 0x000000ab },
    { 0x054ed7a9, 0xe10dc8a6, 0xcf39bec8, 0x7094fc36, 0x25848527, 0x487789a8, 0x39974124,
      0x8786eaa3, 0xa6e3e5ab, 0x47a0d6b0, 0xe981e517, 0x54ed0b21, 0xe768123b, 0x85cf17ea,
      0x636e78c6, 0xd9a933b0, 0x00000127 },
    { 0x9fd73274, 0x3241b0df, 0x8fe5a5dd, 0x7b653239, 0x38412a8d, 0x2d4dd56e, 0xcc58a62a,
      0xe2925f33, 0xf5cee12b, 0xb859ea32, 0x7c25d1d2, 0x5af7c402, 0x6f196e55, 0xe172cec1,
      0xde00641c, 0x34815491, 0x0000010b },
    { 0xc9558569, 0x25f04ed9, 0xef9562c5, 0x0a315324, 0x9ccefb92, 0x65a02a28, 0x88115608,
      0xb2429dd4, 0x641d646b, 0xc1714c93, 0x48bb144a, 0xba4d3691, 0x9efa3a6f, 0x48ec2d9d,
      0xf1a93ae3, 0x5ff6a4fb, 0x000001ed },
    { 0x8e549373, 0xdfc7c844, 0xb52ad215, 0x439dc785, 0xfb9de6dc, 0x9e157e18, 0x8793d562,
      0x9283c331, 0x33ddc16c, 0x79f9ec90, 0xceff5086, 0xed0dbb7f, 0x8765145e, 0xfa466807,
      0xa3b387b7, 0xba9411a9, 0x00000176 },
    { 0xd2dfd89c, 0xfd301b76, 0x0f38e6c1, 0xac9c2b0d, 0x2da8e6d4, 0xcf9d3acf, 0xeb9024c5,
      0x99db13dc, 0x9ee659a9, 0x4ebfc5ba, 0x9b2fa547, 0xbded8094, 0xc08d98c5, 0xfcf035f9,
      0x56b8b217, 0xb50dd136, 0x0000014d },
    { 0x26627252, 0x1a60ef05, 0x9bdc0f5f, 0x31682a83, 0xa112be82, 0x52a055ad, 0x4ecf7367,
      0x9c3a8848, 0x49e70680, 0x52350872, 0xb881b9cf, 0x61b6b478, 0x887d3c1e, 0xbec16c5a,
      0xd598a671, 0x8a8b2b82, 0x000000ef },
    { 0x1961aa16, 0x91fa1e12, 0x67dd7513, 0x381b2d71, 0x44db8a51, 0xbae86351, 0xbc31c1cd,
      0x4cc1915f, 0x8ec6e9e1, 0xf79f4059, 0xa8929a72, 0xb142b6ec, 0x8ed71fb2, 0x4629c648,
      0xbd52faa7, 0x9f79bccf, 0x00000029 },
    { 0xad6dbeaf, 0x04b8115b, 0xa996519b, 0x7d7f0bb9, 0x9c397c91, 0xab54d4c8, 0x77a6a3e3,
      0x1750b97d, 0x14554321, 0x1637909b, 0x7a5aeea3, 0x7311da68, 0x52130453, 0x1f91a0fd,
      0x86753f28, 0x4338b3d6, 0x0000014c },
    { 0x54de3ecb, 0xe4c4d8a0, 0x5f8cb5ce, 0xe2f88ac2, 0x68eb11a2, 0x17a4e60d, 0x3b79619a,
      0x6d0e05fb, 0xd5da75ef, 0xa54874b7, 0xc77f6ac1, 0x18c2b1b3, 0x829736c3, 0x5a2a6fdd,
      0xcd44843b, 0x9ce62f22, 0x00000059 },
    { 0x2e58f405, 0x096aff66, 0xd6658910, 0xee92b17d, 0xd33b5b5a, 0x65792cb3, 0x5678f269,
      0xfa503f21, 0xefdffc67, 0x4e2757c8, 0x665347f5, 0xbe46a6f1, 0x6d9773e7, 0xd049a158,
      0xb0a28ac5, 0x9555b002, 0x00000145 },
    { 0x24785079, 0x65d3657b, 0x5e064334, 0xb1d2b5f7, 0x2b7b61e1, 0x0736bfda, 0x284e4fa6,
      0xb39c4db1, 0xbd934998, 0x66cedf43, 0x324d2de1, 0x8f9f6243, 0x4c193171, 0x92f2524e,
      0x964a8383, 0x75705acd, 0x00000031 },
    { 0xb72d8840, 0x7cbc9061, 0x86a5ba36, 0xe611924d, 0xe066d292, 0xe02e9ca8, 0xa68a0466,
      0xdb092f52, 0x0c0b18f8, 0xf5d97f5d, 0x5c99b95c, 0xb5e17146, 0x9d3d6acf, 0x1bafbe74,
      0xf370bd5f, 0x606103fe, 0x0000013b },
    { 0x514c45ed, 0xb292c583, 0x947e68a1, 0x89ac5bf2, 0xaf507c14, 0x633c4300, 0x7da4020a,
      0x943d7ba5, 0xc0ed8274, 0xd1e90c7a, 0xe59426e6, 0x9634868c, 0xc26bc9de, 0x24a6fff2,
      0x152416cd, 0x1a012168, 0x0000000c },
    { 0x8cdf1f60, 0x721231cf, 0x6717760f, 0x8b640f2b, 0xe045a403, 0xbe726f8c, 0x0370689f,
      0x422285dc, 0x72ea0dcb, 0x7196bf8f, 0xc8086623, 0xa16f7855, 0xc326fe48, 0xd4e19fc7,
      0x8f68bf44, 0xfdbc856e, 0x0000013e },
    { 0xfacdb1cb, 0x802ece3d, 0x9e537236, 0x9e4f5d20, 0x386a5610, 0xd10acc25, 0x4c6c0247,
      0x53f55d46, 0x8c419fa4, 0x390fe980, 0xbed2bd92, 0x8b0a8aa6, 0xf854424f, 0x90e7f479,
      0x9e7f4392, 0xd714bfb5, 0x00000074 },
    { 0xdbb3838a, 0x17ea6fb5, 0xb425af89, 0x192eca92, 0x85a94659, 0x5fd3bfcf, 0xdaa4b4cc,
      0x8d75b3f5, 0x9bee144f, 0xbacc18b8, 0xff60ec49, 0x57591774, 0x2363ac43, 0x319cba95,
      0x472ee36c, 0x6c0d079e, 0x000000f0 },
    { 0x0ae9af6e, 0x8b77ddb7, 0xdd7fd36c, 0x0e21b050, 0xdc8b9c19, 0x35d968ec, 0xa624ad4f,
      0x6f3f0785, 0x1d42c97e, 0x97778a68, 0xa2a4342c, 0x86103bf0, 0x3ababc35, 0x0aef36fd,
      0x2fe10364, 0x46c8fb21, 0x000001e1 },
    { 0x1f1e1f7f, 0x7a772a90, 0xc669aa61, 0xa790dcb7, 0xc5e4f185, 0xdad905ae, 0x116ad6ce,
      0x3bd91c29, 0xc329d8a7, 0xf87c96ea, 0xb5b14581, 0x399c929c, 0x3a8253c0, 0x6fd2983c,
      0xda988b39, 0x616a8b61, 0x00000181 },
    { 0xee1bcdaf, 0xf188f980, 0xc991f497, 0x9ccc1a6b, 0x21ec6f5d, 0x6f733d12, 0xda9f5bdb,
      0x59ce56b9, 0x40e88d50, 0xe03a9a1f, 0xc0190a2a, 0xa5e60328, 0x9ff9d482, 0x382bdbe6,
      0xbb792de6, 0x26895085, 0x00000013 },
    { 0xf3ed6253, 0x8c618934, 0x6e9433b4, 0xe0034244, 0x4260b2c5, 0x989b9921, 0xd9df4747,
      0x11699804, 0x763a7f63, 0x125fe61f, 0xc6ec395a, 0xa972786d, 0xb7b7b8c6, 0x662f9fe7,
      0x5456556e, 0xfb644a61, 0x0000008a },
    { 0x4c67518d, 0x63f89883, 0xf85eb1d9, 0xb5ca3508, 0xaf652b25, 0x1ea1c74f, 0xbddedc4d,
      0x4fbc5476, 0xf4b33c74, 0x22daa81a, 0x3f36ae29, 0x9f7941a0, 0xbca5ca40, 0x90c2b8ce,
      0xb587b3d6, 0x53e736a1, 0x000001b3 },
    { 0x0eadf722, 0x4f5581d3, 0x50d8999b, 0x6e4e31e0, 0x02f03368, 0xc14bb148, 0x26a263b5,
      0x41f19643, 0x3b5f480e, 0xc0642e4c, 0x63191881, 0x0a3280df, 0x78fa9069, 0x4ee1959a,
      0x557a8dc0, 0x7376078a, 0x00000181 },
    { 0xc692d9cc, 0x6ee75741, 0x62a91d4d, 0xd97eb6f2, 0xe2924e75, 0x8858ba5d, 0x517030b1,
      0xb8e1b22a, 0x2af83dfb, 0x651a6838, 0x1c21be14, 0xc1fb14d2, 0xc9eba60e, 0x87689d46,
      0x0da1a29d, 0xe16cfc1a, 0x00000107 },
    { 0x300b2478, 0xec12ea4b, 0xf2a48388, 0x91ac6eed, 0x7a7631ad, 0xfbdc1bba, 0x79d5f4ce,
      0x5b2669cb, 0x48c8f025, 0x219d64f9, 0xaadb5873, 0xadcaab29, 0x671bd673, 0x3b07af1c,
      0xb7f2d8f5, 0xe66e7c67, 0x000000c9 },
    { 0xb50a3e06, 0x3e73accf, 0x0754f2fd, 0x0d8ebee3, 0x4b3f3fd1, 0x3cd0ae95, 0xc08bb2db,
      0xc4a4eee0, 0x19d71e5d, 0x79552963, 0x872e35c9, 0xf91e1ab7, 0xf6c102af, 0xc85090f6,
      0x1e42a1ba, 0x8edf8c5d, 0x00000179 },
    { 0xe71e511d, 0x7130d0ca, 0x35663592, 0x168ef9d9, 0xadbf816f, 0x63a9936e, 0xd4f7a44a,
      0xaf63ee57, 0xcd0ea152, 0x59f21032, 0xa881056f, 0x2f552046, 0x21ba6bf5, 0x26eda9a6,
      0xd0a0caf9, 0xe75991c7, 0x0000004c },
    { 0xbb54847b, 0xacf67416, 0xf2b54b28, 0xa293a8d7, 0x1f7f6b47, 0x32f2b11a, 0xc6d67a9d,
      0xc11948bc, 0x47623783, 0xc8b86b1f, 0x5c2362e6, 0x357326aa, 0x6cf79126, 0xc298be1b,
      0x84c5a79c, 0xf07b3ea7, 0x0000006c },
    { 0x32d8542e, 0xa8483195, 0xc3f38387, 0x1f7a8e67, 0xd6797f2e, 0x162ac248, 0x0a6a1118,
      0xd26469e1, 0xf203102d, 0xb33aff1f, 0xaadf0ae9, 0x63a52154, 0x4bb313a7, 0xf0431dbb,
      0xca9da156, 0xf93c18fb, 0x0000015c },
    { 0x5a82fbaa, 0xfcfa87a5, 0x1569f45b, 0x988a9a6c, 0x5d3d6aa5, 0xe65849f6, 0x05446b48,
      0xa3d0879f, 0x76f480d1, 0x45571ccf, 0x0efd5f79, 0xacd9a323, 0xae956132, 0xf30e5ec4,
      0xea93f245, 0x325306d0, 0x000000a7 },
    { 0x3b6cc6dd, 0x2863bcbf, 0xe9430f99, 0x70760671, 0x7586cdde, 0x46cce87b, 0x161a045f,
      0xc1f58398, 0x291636ae, 0xa84ca0cf, 0x26892bd1, 0x11242d27, 0x8c971e9f, 0xb1d85da7,
      0x61aa5039, 0xc66e0e1d, 0x000001b1 },
    { 0xce5889f5, 0x777ab201, 0x4c6f0545, 0x635d968f, 0x872777ea, 0xbb48a0cf, 0xc616138f,
      0x33f4de62, 0xbc8d65ce, 0x93e74192, 0x5ebae1f0, 0x532ca9bd, 0xfbaba8d0, 0xd783e104,
      0xbdf2d850, 0xb32af569, 0x000001ba },
    { 0x95338fa8, 0xdfb1b255, 0xd59e754d, 0xabadba3a, 0x436e70fc, 0x2338452b, 0xe07a21b6,
      0x34cfb5b2, 0xf81ea5a2, 0x25ddb684, 0xaa325fc8, 0x98f463c0, 0x94aeb02e, 0x963bbc80,
      0x434ab9fa, 0xa98fe976, 0x00000188 },
    { 0xf8b6b176, 0x7b23a55d, 0xc679c536, 0xbc660c66, 0x897fde1a, 0xad82144a, 0x545baf12,
      0x5082037d, 0x4448059f, 0x4bf2b117, 0xf1e8c52d, 0x59c25c1b, 0x8e030e69, 0xc01a4b8d,
      0x229951c1, 0x45077803, 0x00000184 },
    { 0x5f29dbd2, 0x67e18983, 0x74d67113, 0xf0092c9f, 0x847dc935, 0x3faa747c, 0x7f5be1b1,
      0x798604f2, 0x8c76c26a, 0xf766db9a, 0x5c205ca3, 0xa38d86f0, 0x8c6e65ad, 0xfec09777,
      0x156b3b92, 0x1c2b356c, 0x000000d1 },
    { 0x7acb23ca, 0xe1b3574a, 0x3636a1c2, 0xadda7c9e, 0x9f3d947c, 0xfc998cf8, 0x1ba0511b,
      0x38ee4df4, 0x03e4694b, 0x1f40cc14, 0x800fb6d9, 0xccecf4e0, 0xe1665d06, 0x021f708a,
      0xc492d329, 0x2bcd7975, 0x00000063 },
    { 0x1cba57c5, 0x30c79e7b, 0x6062fa39, 0xf71d84be, 0xe2b9c131, 0x8fb70b03, 0xcd862b12,
      0xbb0f27e7, 0x4ffd6a0a, 0x2fea1087, 0x2a7a7b08, 0x17787158, 0x8653ce49, 0xf79606cc,
      0x629a5ff9, 0x9514c960, 0x000001a0 },
    { 0xdca5b802, 0x44912d16, 0x808db8c1, 0x3c30f445, 0x4d2b7d5c, 0xd59e9290, 0x2697a600,
      0xc1e20a6b, 0x3a028772, 0xfbbe023c, 0xc6e3b099, 0x8873d0bb, 0x050828ad, 0x5fe76ac9,
      0xc0ab0ab0, 0xcf2ac286, 0x0000003e },
    { 0xfebbfad7, 0x5936d4fe, 0xa30ed6c6, 0xc94f1262, 0xb2081a7d, 0x6013faf4, 0x85a4e883,
      0x684ffdbf, 0x19dbdad1, 0xaaff2438, 0x1f91e40f, 0x8f348444, 0x1ecea707, 0x145d06f6,
      0xecbade82, 0xeb7a4980, 0x0000004d },
    { 0x5d2ac4b2, 0x027df255, 0x83078809, 0xa61a5557, 0x0237c9bd, 0x6f5d2cd4, 0xcef7c441,
      0x7cd9f911, 0xabc19578, 0x0d5256ef, 0xf338db58, 0xd86047af, 0x14d7c98e, 0xaa2a1b50,
      0xf5e65a3a, 0xbe9edffa, 0x000001bd },
    { 0xa864385f, 0x259b9da4, 0x17c2597b, 0xdbafc55c, 0x56bf5d23, 0xde890acb, 0xf8455b59,
      0xe51182c0, 0xc456e1c7, 0x75d51a03, 0xb318e747, 0x9c7929c6, 0x39b3ed84, 0xaf23a7f1,
      0x910f4ab2, 0xee136a2b, 0x000001ac },
    { 0x27bede63, 0x68ac9834, 0xe54a999b, 0xd5c6c405, 0x231e8eb0, 0x3d763e32, 0x47c7edb7,
      0xe36ca2ff, 0x8d2b6016, 0xd9f18505, 0x60d55105, 0x4e576eb0, 0x76e3f591, 0xe3e9f249,
      0x83ea6a2c, 0xd5bf8f00, 0x00000157 },
    { 0x514ac5db, 0x20b7457d, 0x954721fd, 0xa03028fb, 0x5c77cb17, 0x6f20d518, 0xad74495a,
      0x28f92697, 0x6b327dc8, 0x630e0156, 0x65832ca9, 0x6c5067d6, 0xd3f6db89, 0x9c9305fb,
      0x905fffdf, 0x681013fa, 0x00000188 },
    { 0xd39ac021, 0x9893eb50, 0x39bd296b, 0x3bf70873, 0x792dec05, 0xa10440a7, 0xa67ad18f,
      0x96be97e7, 0x17ae5f6d, 0xcf2175ae, 0x0b6f0503, 0x70fbb8bf, 0xf7a755ad, 0xbb565fad,
      0x93ccfd37, 0x8904f9a0, 0x000000f8 },
    { 0x34a088db, 0x42aabe74, 0x9e5b69ee, 0x871248c8, 0x55a87edb, 0x7d624d6e, 0xbaee4473,
      0x81bc0bcf, 0x9559a1b0, 0xf450c5ad, 0xdf4c738c, 0xa604379c, 0x10ec0574, 0x36b304e2,
      0x8d8cf3d9, 0x948a50d3, 0x0000018e },
    { 0x6dee6431, 0xabbfe426, 0x12fd88ea, 0x2d51aac6, 0x1c83591e, 0xc6f21525, 0x9ee42180,
      0x85c550e5, 0xa8cd1c50, 0xe1a4297d, 0xb55723cf, 0x28f07cf6, 0x4113977b, 0xbede788e,
      0xbedaeeab, 0x8c7572fc, 0x000001ab },
    { 0x0c3477fa, 0x20f4598e, 0xccf7f114, 0x14c758b4, 0x051b875e, 0x0c83f642, 0x1b290e7d,
      0x14ab1b50, 0xf1b18fa8, 0x99f812c7, 0x88ca8e75, 0xaa60f399, 0xb9c6c5f7, 0x00de2bd0,
      0x28703a2a, 0x765da973, 0x00000015 },
    { 0x56d21d18, 0xa90a4290, 0x55b410a1, 0x82666307, 0x894a6b05, 0xb4684a8b, 0x828cf75c,
      0x8a1ade63, 0x127702a3, 0x4fb2f85a, 0xadf7b709, 0x83ff7d05, 0xa68d1db6, 0x1d3f5a92,
      0xc093cd5c, 0x243ce1db, 0x000000f5 },
    { 0x6432c12b, 0x7c518071, 0x90ae7556, 0xb7e004ad, 0xd8532693, 0xdf847160, 0xa18b3802,
      0x4d8c7b8a, 0x96872af8, 0x90e25045, 0xaecb6fad, 0xe69d4894, 0x17d0fe85, 0x6ddaa06c,
      0x340c3528, 0x49f1a466, 0x000000d9 },
    { 0x88da1a0f, 0xc834e659, 0xeebe47a8, 0xb9bfba08, 0x7b3a2b73, 0xf59d3393, 0x8decf4e4,
      0xffc7cb5d, 0x83dc5f4e, 0x72477dd5, 0x43ac64eb, 0x0a59e11e, 0xcb10a6ab, 0xda8aa16c,
      0x3759c37c, 0xe571ec8c, 0x00000047 },
    { 0xd94cb899, 0x2d65148d, 0x3b28185e, 0xcac38630, 0xe102adb5, 0xa36bb7bd, 0xff3fffb6,
      0xa1e7e0b5, 0xe628e1af, 0xc842ceac, 0x8647613e, 0x8e6dfa7c, 0x84ade38b, 0xeafd7585,
      0x00188d5e, 0x7c4c0b0a, 0x00000151 },
    { 0xb16cc1f3, 0xe514497f, 0x627db2f6, 0xbc044f39, 0xa1fa2bdc, 0x76fa5148, 0x5fe3fd7a,
      0xcf7eb72e, 0xee3d45e6, 0x83236c11, 0xc4eddb54, 0xa02d7e92, 0x4d4982a5, 0xcfb5bb89,
      0x7c217ea5, 0xee6ac2a2, 0x000000bd },
    { 0xc1669168, 0x241323f7, 0x79b94b5a, 0x9934b3de, 0xd9f9a406, 0xd332c1a2, 0xe4a49eac,
      0xbe3ebe21, 0x678be3ed, 0x4a00648e, 0xff09db31, 0xc9f534ea, 0x5efdd859, 0x2de77406,
      0x00f5c7a5, 0x2c218cd8, 0x00000152 },
    { 0xae2b39c2, 0x1a13e3ee, 0x3c218179, 0xc431081d, 0xae68b7c6, 0x5cbc14c1, 0x9005a304,
      0xcf2559bb, 0x2ec7aed5, 0x14d7c1e9, 0x1e2e2f0d, 0x5c379bfe, 0x886f0cf9, 0xfc33e4d2,
      0xac4e1d17, 0x2f14e7d0, 0x00000071 },
    { 0xda1d3ea1, 0x7b17e356, 0xa7af9610, 0x0ed57d7e, 0x3ae89c0e, 0x4a6ac3e5, 0x5f82b4a3,
      0xc8f722b8, 0x7f0850fa, 0x453d5a4a, 0x7b3ac4a3, 0xd6f79d23, 0x0dbae800, 0xe1c9dfc2,
      0xc4b9258e, 0x56e4f0f5, 0x000001ca },
    { 0x446e7206, 0xe2087f9c, 0xb48bb585, 0x60f69447, 0x2020bbf2, 0xda365862, 0x3cd51256,
      0xc8a26367, 0xc62f7c5e, 0x8cc663b1, 0xe72bbd20, 0xc86d4165, 0xaac68be3, 0xffce9039,
      0xc3c360a8, 0x6c6c747c, 0x000001d4 },
    { 0x834bf891, 0x03cfa731, 0x630b8032, 0x4e016085, 0x49db4d96, 0x90cf9b59, 0x335c9ddd,
      0x96fd2614, 0x8e3b98fb, 0xc0214510, 0x1da2ea2f, 0x6ded0633, 0x9d0ead76, 0x2bda6d76,
      0xf3e4fbf2, 0x4ba00e99, 0x00000169 },
    { 0xa538774a, 0xbb35ef0c, 0x2b28e6fb, 0xac9aa3ce, 0x0a801edb, 0x57e02d06, 0x94847324,
      0x69775443, 0x29dd0a3b, 0xb526bb19, 0xc468815f, 0xaed93900, 0x221df814, 0x3eb1b9b2,
      0xd9472daf, 0xe421e27a, 0x000000b9 },
    { 0xaeaaa0d2, 0x08801d4e, 0x6fc2830a, 0x3b052b0a, 0x6fef40c1, 0x4e02318f, 0x06f38abb,
      0x5a24478a, 0x6dedd4f3, 0xc90c5392, 0xff07397b, 0x402e4a9e, 0xf1bdec37, 0xde13d7bf,
      0x727eb837, 0x8a363ccb, 0x0000014d },
    { 0xe324ea11, 0x9bdd0365, 0x1117cdc2, 0x9625d01b, 0x7f6c259c, 0x491ea642, 0x125826f0,
      0x61b79486, 0x18a56486, 0x573debc8, 0x0d31132a, 0xe224dd3b, 0x80be8263, 0x075c3bf7,
      0xb84a07fa, 0x0eced67a, 0x0000010e },
    { 0xc01605a9, 0x2a04363c, 0x7d7d1e54, 0x0d7cc0d8, 0xa0f9fd2b, 0x0515eb70, 0xbb4ec5e8,
      0x725d12a9, 0x3a6eb09a, 0xd1b5b8a3, 0x87027d7b, 0xb6c836b1, 0xaaf9cd03, 0xf4773322,
      0x2500c647, 0x2fb3354e, 0x00000081 },
    { 0xf2dc12f5, 0xbb656a89, 0x5c46fcf4, 0x6c4a32bb, 0x70e7a152, 0xf324462c, 0x1f7eaad2,
      0x4b1d509a, 0x76e51583, 0x3da87fa7, 0x19fc86cc, 0x9a435e20, 0x9eab8de3, 0xb97d3f44,
      0x8f92304b, 0xc56c7c98, 0x000000a8 },
    { 0x87717bb9, 0x10c813e9, 0xcfb9252a, 0xd9e1f5a8, 0xd7bde47c, 0x0983c1f7, 0x5474e52d,
      0x7446023b, 0x1941b518, 0x1f189de8, 0xb425810a, 0x1997c69b, 0x58bca332, 0xe15b5f1e,
      0x49092985, 0x715da4c9, 0x00000076 },
    { 0x497da0c6, 0x32d92252, 0xbeb7b1f1, 0xa2d4c8b1, 0xf01c7409, 0xede29968, 0xda06607f,
      0xad902eb5, 0x5c1e7d9e, 0x2c60236c, 0x17a3b1f0, 0x51f51ddd, 0x40ab7cb8, 0xb71b393f,
      0xa16ac6bd, 0x710fc9f9, 0x000001cc },
    { 0x77c16db0, 0x74eeeaa4, 0xeb22ce92, 0x297624e8, 0x45098c91, 0x9d3944bc, 0xfa4dc50c,
      0x6ff31fa0, 0xcc40c211, 0x3c7fb6da, 0x96b4cd1c, 0x74d95dec, 0xa6388941, 0x829423db,
      0xfc3123b5, 0xaa2b7d45, 0x00000119 },
    { 0x3db1c500, 0xdfe19ea5, 0x48ef4a56, 0xdd047171, 0xb9a5ec76, 0x38ab939e, 0x5648f852,
      0x8f2d5685, 0x2726f67e, 0x7f81313f, 0xf68f54bb, 0x41cf3794, 0xde14d6ec, 0x9d400e52,
      0x4c9eb1ba, 0xc046fda1, 0x0000008a },
    { 0x5e4846ce, 0x95f44add, 0x7211225e, 0x85d2a8e8, 0x091a6646, 0xcbf0abd8, 0x102deeaa,
      0x9d8b3475, 0x602ba91c, 0xe2a0a4ab, 0x131de0d4, 0xa6276d7f, 0x5f215c70, 0xbb90d117,
      0x07e3aa82, 0x49a60e3e, 0x00000019 },
    { 0x60e096a3, 0x10c72b82, 0x5ff796ba, 0xc73306ec, 0xfecc17ec, 0xb3f60666, 0x46f82c8d,
      0xb9e0d51f, 0x296da50c, 0x6cbe5c7e, 0x4fa6c1ce, 0x87cb1104, 0x491f9b8b, 0x5d5e0f6d,
      0x70eb3d18, 0x187baf69, 0x00000103 },
    { 0x6d7824ad, 0xdb376339, 0xb16a391b, 0x6b948aef, 0x4ae9013e, 0x2d174fd6, 0x16a57de2,
      0x9b5c22ee, 0x9e857a25, 0x3bfed1da, 0x4d644bbd, 0x635187a6, 0x6fe288ac, 0x94786e3a,
      0xece25962, 0x827c8b03, 0x00000092 },
    { 0xf6af1252, 0xae6f6b72, 0x4ff8a00b, 0x0fc58398, 0xa81fd641, 0x1b2c0cc5, 0xe68b6fba,
      0xbdcaabe6, 0x70b14e44, 0xd1af82fe, 0xe55ce735, 0xe9c35419, 0x01dcefb9, 0x3c83d9dc,
      0xfb5824c9, 0x42604200, 0x0000011f },
    { 0x87436d56, 0xbec7b150, 0xa01f6c04, 0x07cf1192, 0xf73971b0, 0x047b37f6, 0x5ae1f9d4,
      0x7e7d8b9b, 0xca1a9900, 0x0e311b4f, 0x6a81fb38, 0x5dc55f3d, 0x2956af04, 0xe257401a,
      0xf66ae95f, 0x90c2ad09, 0x0000016b },
    { 0x36988cba, 0x7389f3dd, 0x762ca5f4, 0x4ab5c460, 0xb5cf8989, 0x9b96cf82, 0x158a4c79,
      0x2ffa0db5, 0x3a3d24f4, 0xb5821b1a, 0xf8c464d0, 0x17771cb7, 0x62c59a09, 0x9ffd7646,
      0xdc162385, 0x0eefa35f, 0x000001e1 },
    { 0xd8f82b51, 0xfbf287c8, 0x7f0ecada, 0xb7ca6096, 0x15242666, 0x1dc411b7, 0x3dc6ed96,
      0xebf82d6c, 0xd0a92b49, 0x8f82229d, 0x9dac585a, 0x75a6a4a7, 0xc839248a, 0x443ae75e,
      0x1ba7c9c3, 0x4defb89f, 0x000001b5 },
    { 0x52fe816f, 0xbae72ff5, 0x9a5a3747, 0x7b5d9e52, 0xa689a68a, 0x7a7eadcc, 0x330ba649,
      0x18dec239, 0xeb30bd8f, 0xafc49e4c, 0x80c1a715, 0x25042716, 0xbf6e92d9, 0x4d5aafff,
      0x7577cc4f, 0x94af51fb, 0x000001b9 },
    { 0x4c695615, 0x611fe09d, 0x323b6dcc, 0x087334f6, 0xd53bcfdc, 0xf7bd51bb, 0x5a8bcdb4,
      0x98ab5c92, 0xcccd6f6a, 0xe7e9d273, 0x24b4a3ea, 0x2582b219, 0xf22c4405, 0xf8fea0e3,
      0xd3e5a701, 0x42c0f700, 0x0000006d },
    { 0xe5c35f69, 0x76ec7312, 0xc07874d4, 0x6832a65a, 0xc121e8c9, 0x398dab5a, 0x97b47c38,
      0xf88aa2c8, 0x07678cf3, 0xfc77e1c0, 0x345a9b89, 0x7327a90f, 0x49240b3e, 0x267417a0,
      0xe3912921, 0x3aa3ea97, 0x00000116 },
    { 0x3c115125, 0x1f3e5910, 0xbfb07241, 0x0d2d18fc, 0xba4b009d, 0x5d6aa860, 0x806ff8ed,
      0x8d0d9368, 0x1a05c049, 0xcc5ca370, 0xe9ab4c41, 0x7bf21cae, 0x001d5cca, 0x6632c3e0,
      0xb60e5593, 0x3a4f99b6, 0x00000008 },
    { 0x72765171, 0xf2224e6f, 0x11b89e04, 0x3f9ea548, 0x138f1a43, 0x1e07c6eb, 0x621f1f7e,
      0xd9b61817, 0x2f69af1a, 0x08e851aa, 0xc0d63f32, 0x3e75da6f, 0xf0f0d74d, 0x49a72d31,
      0xc516a10c, 0xfccfc561, 0x0000009f },
    { 0x38d77a9e, 0x7259e719, 0xa3661199, 0xcd2278b1, 0xbf6dfc59, 0xcf7fdf77, 0x96a2b632,
      0x6265f9cb, 0x4eb09c8a, 0x1365e44a, 0x2b23bf87, 0xc1ac3054, 0x5aaec208, 0x3161a2c0,
      0x55db8ae9, 0x1af3a614, 0x00000098 },
    { 0xbc363066, 0x30ab00a3, 0xd371d11c, 0xd9a0ab44, 0xab047490, 0x13697c6a, 0xa57574cd,
      0x1adffb54, 0xe548b99b, 0x753eafa0, 0xa7f0df39, 0x51abf774, 0x903eaee9, 0xe8fab02c,
      0xf7542020, 0xdf2f5e8a, 0x000000fa },
    { 0xdef8f3c5, 0x73dd99cc, 0x7ee6942e, 0x46a2f1c8, 0x47e43d8b, 0xae8bb505, 0x5b912753,
      0xa24b5b63, 0x26120607, 0x26a615ca, 0xa661fe70, 0xf8837a16, 0x49fac25d, 0xd716a6d6,
      0x51898a22, 0x6cc2464c, 0x0000003d }
};
#endif //PRECOMPUT
static const DIGIT local_inv2_p521r1[SECP521R1_WORDSIZE] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000100
};
static const DIGIT local_xg_p521r1[SECP521R1_WORDSIZE] = {
    0xc2e5bd66, 0xf97e7e31, 0x856a429b, 0x3348b3c1, 0xa2ffa8de, 0xfe1dc127,
    0xefe75928, 0xa14b5e77, 0x6b4d3dba, 0xf828af60, 0x053fb521, 0x9c648139,
    0x2395b442, 0x9e3ecb66, 0x0404e9cd, 0x858e06b7, 0x000000c6
};
static const DIGIT local_yg_p521r1[SECP521R1_WORDSIZE] = {
    0x9fd16650, 0x88be9476, 0xa272c240, 0x353c7086, 0x3fad0761, 0xc550b901,
    0x5ef42640, 0x97ee7299, 0x273e662c, 0x17afbd17, 0x579b4468, 0x98f54449,
    0x2c7d1bd9, 0x5c8a5fb4, 0x9a3bc004, 0x39296a78, 0x00000118
};
static const DIGIT local_a_p521r1[SECP521R1_WORDSIZE] = {
    0xfffffffc, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x000001ff
};
static const DIGIT local_b_p521r1[SECP521R1_WORDSIZE] = {
    0x6b503f00, 0xef451fd4, 0x3d2c34f1, 0x3573df88, 0x3bb1bf07, 0x1652c0bd,
    0xec7e937b, 0x56193951, 0x8ef109e1, 0xb8b48991, 0x99b315f3, 0xa2da725b,
    0xb68540ee, 0x929a21a0, 0x8e1c9a1f, 0x953eb961, 0x00000051
};
static const DIGIT local_p_p521r1[SECP521R1_WORDSIZE] = {
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x000001ff
};
static const DIGIT local_n_p521r1[SECP521R1_WORDSIZE] = {
    0x91386409, 0xbb6fb71e, 0x899c47ae, 0x3bb5c9b8, 0xf709a5d0, 0x7fcc0148,
    0xbf2f966b, 0x51868783, 0xfffffffa, 0xffffffff, 0xffffffff, 0xffffffff,
    0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x000001ff
};
static const DIGIT local_p1x_p521r1[] = { 0x6b4c3f67, 0x82e05142, 0x3fc34315, 0x83049259,
                                          0x972d1c60, 0x2b17027d, 0x06941699, 0x650bd0df,
                                          0xbf06dea4, 0xc960bca9, 0xf6bf6453, 0xc9b131ee,
                                          0x6e2a0bd0, 0xc7865c90, 0x5d5f6799, 0xffb964e0,
                                          0x00000033 };
static const DIGIT local_p1y_p521r1[] = { 0x16056e76, 0xcd247a09, 0x4aabbfce, 0xc0f214f1,
                                          0x8fb1cf42, 0x7b7ba942, 0x79dfcd33, 0x521e44f1,
                                          0x030cfa52, 0x72151cc5, 0x3f763269, 0x6e731597,
                                          0xfa5b5eb9, 0x15eea047, 0xc9cc275d, 0x6870c5d0,
                                          0x0000017c };
#endif //WORD32
/** <b>ECC Curve structure variable for SEC-P521r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve secp521r1;
#endif

#ifdef BP512
#ifdef WORD32
static const DIGIT local_inv2_bp512r1[BP512R1_WORDSIZE] = {
    0x2c1d247a, 0x9455302b, 0x96c16342, 0x1440ff97, 0x7351c073, 0x5766d095, 0x4de33421, 0xbea6cd80,
    0x38198438, 0x6b31ce65, 0xd9e4e907, 0xe59846d9, 0x19e4fe03, 0x9fea7357, 0x6df4e245, 0x556ecedc
};
static const DIGIT local_xg_bp512r1[BP512R1_WORDSIZE] = {
    0xbcb9f822, 0x8b352209, 0x406a5e68, 0x7c6d5047, 0x93b97d5f, 0x50d1687b, 0xe2d0d48d, 0xff3b1f78,
    0xf4d0098e, 0xb43b62ee, 0xb5d916c1, 0x85ed9f70, 0x9c4c6a93, 0x5a21322e, 0xd82ed964, 0x81aee4bd
};
static const DIGIT local_yg_bp512r1[BP512R1_WORDSIZE] = {
    0x3ad80892, 0x78cd1e0f, 0xa8f05406, 0xd1ca2b2f, 0x8a2763ae, 0x5bca4bd8, 0x4a5f485e, 0xb2dcde49,
    0x881f8111, 0xa000c55b, 0x24a57b1a, 0xf209f700, 0xcf7822fd, 0xc0eabfa9, 0x566332ec, 0x7dde385d
};
static const DIGIT local_a_bp512r1[BP512R1_WORDSIZE] = {
    0x77fc94ca, 0xe7c1ac4d, 0x2bf2c7b9, 0x7f1117a7, 0x8b9ac8b5, 0x0a2ef1c9, 0xa8253aa1, 0x2ded5d5a,
    0xea9863bc, 0xa83441ca, 0x3df91610, 0x94cbdd8d, 0xac234cc5, 0xe2327145, 0x8b603b89, 0x7830a331
};
static const DIGIT local_b_bp512r1[BP512R1_WORDSIZE] = {
    0x8016f723, 0x2809bd63, 0x5ebae5dd, 0x984050b7, 0xdc083e67, 0x77fc94ca, 0xe7c1ac4d, 0x2bf2c7b9,
    0x7f1117a7, 0x8b9ac8b5, 0x0a2ef1c9, 0xa8253aa1, 0x2ded5d5a, 0xea9863bc, 0xa83441ca, 0x3df91610
};
static const DIGIT local_p_bp512r1[BP512R1_WORDSIZE] = {
    0x583a48f3, 0x28aa6056, 0x2d82c685, 0x2881ff2f, 0xe6a380e6, 0xaecda12a, 0x9bc66842, 0x7d4d9b00,
    0x70330871, 0xd6639cca, 0xb3c9d20e, 0xcb308db3, 0x33c9fc07, 0x3fd4e6ae, 0xdbe9c48b, 0xaadd9db8
};
static const DIGIT local_n_bp512r1[BP512R1_WORDSIZE] = {
    0x9ca90069, 0xb5879682, 0x085ddadd, 0x1db1d381, 0x7fac1047, 0x41866119, 0x4ca92619, 0x553e5c41,
    0x70330870, 0xd6639cca, 0xb3c9d20e, 0xcb308db3, 0x33c9fc07, 0x3fd4e6ae, 0xdbe9c48b, 0xaadd9db8
};
static const DIGIT local_p1x_bp512r1[] = { 0xd417e2e3, 0xc5cff507, 0x2b548eaa, 0x066cc526,
                                           0x5e1c93fb, 0x705ff956, 0x92e44ac2, 0x2f0c64e8,
                                           0xe9f2d580, 0xe5068698, 0x1834d6b0, 0x4917a380,
                                           0x21e9c0f8, 0x92c7b458, 0x9f30b3fa, 0x22893afe };
static const DIGIT local_p1y_bp512r1[] = { 0xefe89ad4, 0x77d67f46, 0x90b9ccc0, 0xe9983393,
                                           0xe74c256c, 0x50098aa6, 0x5141bf13, 0x0844cde6,
                                           0x0bd3d553, 0xe0c07ef6, 0x354bb2d8, 0xeb15faa0,
                                           0x3587baa4, 0x35835ac1, 0xf274fdab, 0x7f5e3b03 };

#endif //WORD32

/** <b>ECC Curve structure variable for Brainpool P512r1</b>.
 *
 * @ingroup UCL_ECDSA
 */
extern ucl_type_curve bp512r1;
#endif
#endif //ROMCODE P256

/** <b>ECDSA signature</b>.
 * Compute a ECDSA signature, using curve domain parameters, introduced in 2.4.9
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
 * @retval #UCL_INVALID_INPUT or #UCL_INVALID OUTPUT in case of wrong parameters configuration (e.g. NULL pointers, secret key greater than curve order)
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_signature(ucl_type_ecdsa_signature signature, u8 *d, int (*ucl_hash)(u8 *, u8 *, u32),
                        u8 *input, u32 inputlength, ucl_type_curve *curve_params,
                        u32 configuration);
int ucl_ecdsa_signature_without_protection(ucl_type_ecdsa_signature signature, u8 *d,
                                           int (*ucl_hash)(u8 *, u8 *, u32), u8 *input,
                                           u32 inputlength, ucl_type_curve *curve_params,
                                           u32 configuration);

/** <b>ECDSA signature</b>.
 * Verify a ECDSA signature, using curve domain structure, introduced in 2.4.9
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

int ucl_ecdsa_verification(ucl_type_ecc_u8_affine_point Q, ucl_type_ecdsa_signature signature,
                           int (*ucl_hash)(u8 *, u8 *, u32), u8 *input, u32 inputlength,
                           ucl_type_curve *curve_params, u32 configuration);

/** <b>SM2 DSA signature</b>.
 * compute a SM2 P256 DSA signature, using the SM2 curve as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00, with SM3 hash function defined in draft-shen-sm3-hash-00
*
* @param[out] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] *d: input, the secret key
* @param[in] *A: input, a pointer to a ucl_type_ecc_u8_affine_point structure, representing the public key
* @param[in] *input: input, the input message or hash which signature to be checked
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)

 * @return Error code
 *
 * @retval #UCL_OK in case of correct computation
 * @retval #UCL_INVALID_INPUT or #UCL_INVALID OUTPUT in case of wrong parameters configuration (e.g. NULL pointers)
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_signature(ucl_type_ecdsa_signature signature, u8 *d, ucl_type_ecc_u8_affine_point A,
                         u8 *input, u32 inputlength, u8 *ida, u8 *entla, u32 configuration);

/** <b>SM2 DSA signature</b>.
 * compute a SM2 DSA signature, using the curve domain parameters provided in the curve domain structure, with SM3 hash function
 * it is equivalent to the ucl_sm2dsa_signature function, if the sm2fp256 variable is used
*
* @param[out] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] *d: input, the secret key
* @param[in] *A: input, a pointer to a ucl_type_ecc_u8_affine_point structure, representing the public key
* @param[in] *input: input, the input message or hash which signature to be checked
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing one is sm2fp256, described in the documentation) 
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)

 * @return Error code
 *
 * @retval #UCL_OK in case of correct computation
 * @retval #UCL_INVALID_INPUT or #UCL_INVALID OUTPUT in case of wrong parameters configuration (e.g. NULL pointers)
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_signature_curve(ucl_type_ecdsa_signature signature, u8 *d,
                               ucl_type_ecc_u8_affine_point A, u8 *input, u32 inputlength, u8 *ida,
                               u8 *entla, ucl_type_curve *curve_params, u32 configuration);

/** <b>ECC SM2 DSA signature</b>.
 * Verify a SM2 P256 DSA signature, using the SM2 curve as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00, with SM3 hash function defined in draft-shen-sm3-hash-00
*
* @param[in] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] public key: pointer to a ucl_type_ecc_u8_affine_point structure, containing the ECC public key, used for signature verification
* @param[in] *input: input, the message or the hash digest to be signed,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
* @param[in] configuration (combination of any of these lines)
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)

* @return Error code

* @retval #UCL_OK if the signature is verified
* @retval #UCL_ERROR if the signature is not verified
* @retval #UCL_INVALID_INPUT in case of wrong parameters 
*
* @ingroup UCL_ECDSA */

int ucl_sm2dsa_verification(ucl_type_ecdsa_signature signature, ucl_type_ecc_u8_affine_point A,
                            u8 *input, u32 inputlength, u8 *ida, u8 *entla, u32 configuration);

/** <b>ECC SM2 DSA signature</b>.
 * Verify a SM2 DSA signature, using the SM2 curve given as a parameter, with the SM3 hash function defined in draft-shen-sm3-hash-00
*
* @param[in] signature: pointer to a ucl_type_ecdsa_signature structure, containing the signature (r,s) values
* @param[in] public key: pointer to a ucl_type_ecc_u8_affine_point structure, containing the ECC public key, used for signature verification
* @param[in] *input: input, the message or the hash digest to be signed,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing one, sm2fp256, is described in the documentation) 
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)

* @return Error code

* @retval #UCL_OK if the signature is verified
* @retval #UCL_ERROR if the signature is not verified
* @retval #UCL_INVALID_INPUT in case of wrong parameters 
*
* @ingroup UCL_ECDSA */
int ucl_sm2dsa_verification_curve(ucl_type_ecdsa_signature signature,
                                  ucl_type_ecc_u8_affine_point A, u8 *input, u32 inputlength,
                                  u8 *ida, u8 *entla, ucl_type_curve *curve_params,
                                  u32 configuration);

/** <b>SM2 public key encryption</b>.
 * encrypt data, using the SM2 public key encryption scheme, as defined in the draf-shen-ecdsa-01 section 7, using a KDF based on SM3 hash function as defined in draft-shen-sm3-hash-00
*
* @param[out] *c: output, pointer to a u8 array containing the produced ciphertext
* @param[in] *A: input, the ECC SM2 public key, to be used for encryption
* @param[in] *m: input, the message to be encrypted,
* @param[in] klen: input, the len of the ciphertext, in bits
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing one, sm2fp256, is described in the documentation) 
*
* limitation: the cryptotext length is limited to 500 bytes
*
* @return Error code

* @retval #UCL_OK if the encryption process is OK
* @retval #UCL_ERROR if the encryption process is not OK
* @retval #UCL_INVALID_OUPUT if the output pointer parameters are NULL or if the cryptotext length is too long
* @retval #UCL_INVALID_INPUT in case of NULL input parameters pointers
* @retval #UCL_STACK_ERROR in case of too small available stack space
*
* @ingroup UCL_ECDSA */
int ucl_sm2_encryption_curve(u8 *c, ucl_type_ecc_u8_affine_point A, u8 *m, u32 klen,
                             ucl_type_curve *curve_params);

/** <b>SM2 public key encryption</b>.
 * decrypt data, using the SM2 public key encryption scheme, as defined in the draf-shen-ecdsa-01 section 7, using a KDF based on SM3 hash function as defined in draft-shen-sm3-hash-00
*
* @param[out] *m: output, pointer to a u8 array containing the produced plaintext
* @param[in] *d: input, the ECC SM2 private key, to be used for decryption
* @param[in] *A: input, the ECC SM2 public key, to be used for decryption
* @param[in] *c: input, the ciphertext to be decrypted
* @param[in] klen: input, the len of the ciphertext, in bits
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing one, sm2fp256, is described in the documentation) 
*
* limitation: the cryptotext length is limited to 500 bytes
*
* @return Error code

* @retval #UCL_OK if the decryption process is OK
* @retval #UCL_ERROR if the decryption process is not OK
* @retval #UCL_INVALID_OUPUT if the output pointer parameters are NULL
* @retval #UCL_INVALID_INPUT in case of NULL input parameters pointers or if the cryptotext length is too long
* @retval #UCL_STACK_ERROR in case of too small available stack space
*
* @ingroup UCL_ECDSA */
int ucl_sm2_decryption_curve(u8 *m, u8 *d, ucl_type_ecc_u8_affine_point A, u8 *c, u32 klen,
                             ucl_type_curve *curve_params);

/** <b>ECDSA signature</b>.
 * Compute a ECDSA signature
 *
* @param[out] *r_and_others: input/output, the r value for the computed signature (may have been pre-computed, with UCL_R_PRECOMP configuration) if UCL_R_PRECOMP or UCL_PRECOMP_R are used, the array is a double curve array as containing the r and the k-1.
* @param[out] *s: output, the s value for the computed signature
* @param[in] *d: input, the secret key
* @param[in] *input: input, the message or the hash digest to be signed,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - SECP160R1 or SECP192R1  or SECP224R1 or  SECP256R1 or SECP384R1 or SECP521R1 or BP256R1 or BP384R1 or BP512R1
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)
     - UCL_SHA256 or UCL_SHA224 or UCL_SHA384 or UCL_SHA512 or UCL_SHA1: hash function, if UCL_MSG_INPUT is used.
     - Examples are:
       - SECP192R1<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest on P192 curve, r not precomputed
       - SECP256R1<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest on P256 curve, r not precomputed
       - SECP256R1<<UCL_CURVE_SHIFT+UCL_PRECOMP_R<<UCL_PRECOMP_SHIFT+UCL_NO_INPUT<<UCL_INPUT_SHIFT: pre-compute r on P-256.
       - UCL_R_PRECOMP<<UCL_PRECOMP_SHIFT+SECP192R1<<UCL_CURVE_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT+ UCL_SHA256<<UCL_HASH_SHIFT:signing a message, to be hashed with SHA-256, using pre-computed r

 * @return Error code
 *
 * @retval #UCL_OK in case of correct computation
 * @retval #UCL_INVALID_INPUT in case of wrong parameters configuration
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_sign(u8 *r_and_others, u8 *s, u8 *d, u8 *input, u32 inputlength, u32 configuration);

#ifdef P160
int ucl_ecdsa_sign_p160r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p160r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p160r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecdsa_sign_p160r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p160r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_p160_Q(DIGIT qx[40][8], DIGIT qy[40][8], DIGIT *x, DIGIT *y, DIGIT *p,
                           unsigned int digits);
int ucl_ecdsa_verify_p160r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p160r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp160r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP160R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp160r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp160r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp160r1 curve
* @param[in] *n: input, the n parameter of the secp160r1 curve
* @param[in] *p: input, the p parameter of the secp160r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p160r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef P192
int ucl_ecdsa_sign_p192r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p192r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p192r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecies_decrypt_p192r1_aes_hmac_sha256(u8 *m, u32 keylength, u8 *xG, u8 *yG, u8 *a, u8 *n,
                                             u8 *p, u8 *d, u8 *rx, u8 *ry, u8 *crypto,
                                             int crypto_len, u8 *t);
int ucl_ecies_encrypt_p192r1_aes_hmac_sha256(u8 *rx, u8 *ry, u8 *crypto, u8 *t, u32 keylength,
                                             u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *a, u8 *n, u8 *p,
                                             u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p192r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p192r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p192r1_sia256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_p192_Q(DIGIT qx[48][8], DIGIT qy[48][8], DIGIT *x, DIGIT *y, DIGIT *p,
                           unsigned int digits);
int ucl_ecdsa_verify_p192r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p192r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p192r1_sia256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp192r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP192R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp192r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp192r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp192r1 curve
* @param[in] *n: input, the n parameter of the secp192r1 curve
* @param[in] *p: input, the p parameter of the secp192r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p192r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef P224
int ucl_ecdsa_sign_p224r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p224r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p224r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecdsa_sign_p224r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p224r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p224r1_sha224(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_p224_Q(DIGIT qx[56][8], DIGIT qy[56][8], DIGIT *x, DIGIT *y, DIGIT *p,
                           unsigned int digits);
int ucl_ecdsa_verify_p224r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p224r1_sha224(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp224r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP224R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp224r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp224r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp224r1 curve
* @param[in] *n: input, the n parameter of the secp224r1 curve
* @param[in] *p: input, the p parameter of the secp224r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p224r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef P256
int ucl_ecdsa_sign_p256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p256r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecdsa_sign_p256r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p256r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_p256_Q(DIGIT qx[64][SECP256R1_WORDSIZE], DIGIT qy[64][SECP256R1_WORDSIZE],
                           DIGIT *x, DIGIT *y, DIGIT *p, unsigned int digits);
int ucl_ecdsa_verify_p256r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p256r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp256r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP256R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp256r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp256r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp256r1 curve
* @param[in] *n: input, the n parameter of the secp256r1 curve
* @param[in] *p: input, the p parameter of the secp256r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
int ucl_ecdsa_verify_p256r1_sha256_with_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ,
                                                u8 *r, u8 *s, u8 *a, u8 *n, u8 *p, u8 *m,
                                                u32 MsgLng);
#endif

#ifdef BP256
int ucl_ecdsa_sign_bp256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                           u8 *n, u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_bp256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_bp256r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                     u32 MsgLng);
int ucl_ecdsa_sign_bp256r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp256r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_bp256_Q(DIGIT qx[64][8], DIGIT qy[64][8], DIGIT *x, DIGIT *y, DIGIT *p,
                            DIGIT digits);
int ucl_ecdsa_verify_bp256r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp256r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a bp256r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #BP256R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the bp256r1 base point
* @param[in] *yG: input, the y affine coordinate of the bp256r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the bp256r1 curve
* @param[in] *n: input, the n parameter of the bp256r1 curve
* @param[in] *p: input, the p parameter of the bp256r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_bp256r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                             u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef P384
int ucl_ecdsa_sign_p384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p384r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecdsa_sign_p384r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p384r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p384r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
void precomputation_p384_Q(DIGIT qx[96][8], DIGIT qy[96][8], DIGIT *x, DIGIT *y, DIGIT *p,
                           unsigned int digits);
int ucl_ecdsa_verify_p384r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p384r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p384r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp384r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP384R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp384r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp384r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp384r1 curve
* @param[in] *n: input, the n parameter of the secp384r1 curve
* @param[in] *p: input, the p parameter of the secp384r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef BP384
int ucl_ecdsa_sign_bp384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                           u8 *n, u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_bp384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_bp384r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                     u32 MsgLng);
int ucl_ecdsa_sign_bp384r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp384r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp384r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp384r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp384r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp384r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a bp384r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #BP384R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the bp384r1 base point
* @param[in] *yG: input, the y affine coordinate of the bp384r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the bp384r1 curve
* @param[in] *n: input, the n parameter of the bp384r1 curve
* @param[in] *p: input, the p parameter of the bp384r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_bp384r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                             u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef P521
int ucl_ecdsa_sign_p521r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a, u8 *n,
                          u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_p521r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_p521r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                    u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                    u32 MsgLng);
int ucl_ecdsa_sign_p521r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                               u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p521r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_p521r1_sha512(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p521r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                 u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p521r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_p521r1_sha512(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                   u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a secp521r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #SECP521R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the secp521r1 base point
* @param[in] *yG: input, the y affine coordinate of the secp521r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the secp521r1 curve
* @param[in] *n: input, the n parameter of the secp521r1 curve
* @param[in] *p: input, the p parameter of the secp521r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_p521r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                            u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

#ifdef BP512
int ucl_ecdsa_sign_bp512r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                           u8 *n, u8 *p, u8 *d, u8 *hash, u32 hashlength);
int ucl_ecdsa_r_precomp_sign_bp512r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *a, u8 *n, u8 *p, u8 *d);
int ucl_ecdsa_sign_bp512r1_r_precomp(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r,
                                     u8 *kminus1, u8 *s, u8 *a, u8 *n, u8 *p, u8 *d, u8 *m,
                                     u32 MsgLng);
int ucl_ecdsa_sign_bp512r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                                u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp512r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp512r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_sign_bp512r1_sha512(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *d, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp512r1_sha1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                  u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp512r1_sha256(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp512r1_sha384(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
int ucl_ecdsa_verify_bp512r1_sha512(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s,
                                    u8 *a, u8 *n, u8 *p, u8 *m, u32 MsgLng);
/** <b>ECDSA signature</b>.
 * Verify a bp512r1 ECDSA signature, directly from the message hash
*
* @param[in] keylength: input, the bytes size of the curve, i.e. #BP512R1_BYTESIZE
* @param[in] *xG: input, the x affine coordinate of the bp512r1 base point
* @param[in] *yG: input, the y affine coordinate of the bp512r1 base point
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *a: input, the a parameter of the bp512r1 curve
* @param[in] *n: input, the n parameter of the bp512r1 curve
* @param[in] *p: input, the p parameter of the bp512r1 curve
* @param[in] *hash: input, the already hashed digest of the message,
* @param[in] hashlength: input, the hash length, in bytes
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_ecdsa_verify_bp512r1(u32 keylength, u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *a,
                             u8 *n, u8 *p, u8 *hash, u32 hashlength);
#endif

/** <b>ECDSA signature</b>.
 * Verify a ECDSA signature
 *
* @param[in] *xQ: input, the x affine coordinate of the public key
* @param[in] *yQ: input, the y affine coordinate of the public key
* @param[in] *r: input, the r value for the computed signature,
* @param[in] *s: input, the s value for the computed signature
* @param[in] *input: input, the signed message or signed hashed digest of the message,
* @param[in] inputlength: input, the input length, in bytes
* @param[in] configuration (combination of any of these lines) 
     SECP160R1 or SECP192R1  or SECP224R1 or  SECP256R1 or SECP384R1 or SECP521R1 or BP256R1 or BP384R1 or BP512R1
     - 	UCL_MSG_INPUT or UCL_HASH_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_SHA256 or UCL_SHA224 or UCL_SHA384 or UCL_SHA512 or UCL_SHA1: hash function, if UCL_MSG_INPUT is used.
     - UCL_PRECOMP_TRICK: informs about use of precomputed values, to speed up the verification
     -	Examples are:
       -	SECP192R1<<UCL_CURVE_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT+ UCL_SHA256<<UCL_HASH_INPUT:verifying a signed message, hashed with SHA-256, 
       -	SECP256R1<<UCL_CURVE_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT+ UCL_SHA256<<UCL_HASH_INPUT:verifying a signed message, hashed with SHA-256, 
       -	UCL_PRECOMP_TRICK<<UCL_PRECOMP_TRICK_SHIFT:uses the precompx and precompy global variables as pointing to the customer tables containing the precomputed values
       -	SECP192R1<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: verifying a signed hash digest on P192 curve,
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters configuration
 *
 * @ingroup UCL_ECDSA */

int ucl_ecdsa_verify(u8 *xQ, u8 *yQ, u8 *r, u8 *s, u8 *input, u32 inputlength, u32 configuration);
int ucl_ecdsa_verify_new(ucl_type_ecc_u8_affine_point Q, ucl_type_ecdsa_signature signature,
                         int (*hash)(u8 *, u8 *, u32), u8 *input, u32 inputlength,
                         ucl_type_curve *curve_params, u32 configuration);

void ecc_ellipticmult(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x, DIGIT *y, DIGIT *p,
                      unsigned digits, int curve);
void ecc_mod192(DIGIT *b, DIGIT *c, DIGIT cDigits, DIGIT *p, DIGIT pDigits);
int ecc_modsub(DIGIT *a, DIGIT *b, DIGIT *c, DIGIT *p, DIGIT digits);
int __API__ ecc_modsub_new(DIGIT *p_result, DIGIT *p_left, DIGIT *p_right,
                           ucl_type_curve *curve_params);
int ecc_modleftshift(DIGIT *a, DIGIT *b, DIGIT c, DIGIT digits, DIGIT *p, DIGIT pDigits, int curve);
int ecc_modleftshift_new(DIGIT *a, DIGIT *b, DIGIT c, DIGIT digits, ucl_type_curve *curve_params);
int ecc_modmult(DIGIT *r, DIGIT *a, DIGIT *b, DIGIT *m, DIGIT k, int curve);
int ecc_modmult_new(DIGIT *r, DIGIT *a, DIGIT *b, ucl_type_curve *curve_params);
void ecc_modmultscalar(DIGIT *r, DIGIT a, DIGIT *b, DIGIT *m, DIGIT k, int curve);
int ecc_modsquare(DIGIT *r, DIGIT *a, DIGIT *m, DIGIT k, int curve);
int ecc_modsquare_new(DIGIT *r, DIGIT *a, ucl_type_curve *curve_params);
void ecc_mod256(DIGIT *b, DIGIT *c, DIGIT cDigits, DIGIT *p, DIGIT pDigits);
void ecc_mod224(DIGIT *b, DIGIT *c, DIGIT cDigits, DIGIT *p, DIGIT pDigits);
int ecc_infinite_affine(DIGIT *x, DIGIT *y, int digits, int curve);
int ecc_mult_affine(DIGIT *xq, DIGIT *yq, DIGIT *m, DIGIT *a, DIGIT *x, DIGIT *y, DIGIT *p,
                    unsigned digits, int curve);
int ecc_mult_affine_new(ucl_type_ecc_digit_affine_point Q, DIGIT *m, DIGIT *x, DIGIT *y,
                        ucl_type_curve *curve_params);
int ecc_infinite_jacobian(DIGIT *x, DIGIT *y, DIGIT *z, int digits, int curve);
int ecc_infinite_jacobian_new(ucl_type_ecc_jacobian_point Q, ucl_type_curve *curve_params);
int ecc_double_jacobian(DIGIT *x3, DIGIT *y3, DIGIT *z3, DIGIT *x1, DIGIT *y1, DIGIT *z1, DIGIT *p,
                        DIGIT digits, int curve);
int ecc_double_jacobian_new(ucl_type_ecc_jacobian_point Q3, ucl_type_ecc_jacobian_point Q1,
                            ucl_type_curve *curve_params);
int ecc_add_jacobian_affine(DIGIT *x3, DIGIT *y3, DIGIT *z3, DIGIT *x1, DIGIT *y1, DIGIT *z1,
                            DIGIT *x2, DIGIT *y2, DIGIT *p, DIGIT digits, int curve);

int ecc_add_jacobian_affine_new(ucl_type_ecc_jacobian_point Q3, ucl_type_ecc_jacobian_point Q1,
                                ucl_type_ecc_digit_affine_point Q2, ucl_type_curve *curve_params);
int ecc_add_jacobian_jacobian(DIGIT *tx3, DIGIT *ty3, DIGIT *tz3, DIGIT *x1, DIGIT *y1, DIGIT *z1,
                              DIGIT *x2, DIGIT *y2, DIGIT *z2, DIGIT *p, DIGIT digits, int curve);
int ecc_add_jacobian_jacobian_new(ucl_type_ecc_jacobian_point T3, ucl_type_ecc_jacobian_point X1,
                                  ucl_type_ecc_jacobian_point X2, ucl_type_curve *curve_params);
int ecc_convert_affine_to_jacobian(DIGIT *xq, DIGIT *yq, DIGIT *zq, DIGIT *x, DIGIT *y, int digits,
                                   int curve);
int ecc_convert_jacobian_to_affine_new(DIGIT *x, DIGIT *y, DIGIT *xq, DIGIT *yq, DIGIT *zq,
                                       ucl_type_curve *curve_params);
int ecc_convert_affine_to_jacobian_new(ucl_type_ecc_jacobian_point Q,
                                       ucl_type_ecc_digit_affine_point X1,
                                       ucl_type_curve *curve_params);
int ecc_convert_jacobian_to_affine(DIGIT *x, DIGIT *y, DIGIT *xq, DIGIT *yq, DIGIT *zq, DIGIT *p,
                                   int digits, int curve);
int ecc_mult_coz(DIGIT *xq, DIGIT *yq, DIGIT *k, DIGIT *a, DIGIT *x, DIGIT *y, DIGIT *p,
                 DIGIT digits, int curve);
int ecc_mult_coz_new(ucl_type_ecc_digit_affine_point *Q, DIGIT *k, DIGIT digits_ext,
                     ucl_type_ecc_digit_affine_point point, ucl_type_curve *curve_params);
int ecc_mult_jacobian(DIGIT *xq, DIGIT *yq, DIGIT *m, DIGIT *a, DIGIT *x, DIGIT *y, DIGIT *p,
                      DIGIT digits, int curve);
int ecc_mult_jacobian_new(ucl_type_ecc_digit_affine_point Q, DIGIT *m,
                          ucl_type_ecc_digit_affine_point X1, ucl_type_curve *curve_params);
int ecc_equal_jacobian_new(ucl_type_ecc_jacobian_point Q1, ucl_type_ecc_jacobian_point Q2,
                           ucl_type_curve *curve_params);
int ecc_add_new(ucl_type_ecc_digit_affine_point Q3, ucl_type_ecc_digit_affine_point Q1,
                ucl_type_ecc_digit_affine_point Q2, ucl_type_curve *curve_params);
int ecc_double_new(ucl_type_ecc_digit_affine_point Q3, ucl_type_ecc_digit_affine_point Q1,
                   ucl_type_curve *curve_params);

void ecc_mult_jacobian_window4(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x, DIGIT *y,
                               DIGIT *p, unsigned int digits, int curve);
int ecc_mult_jacobian_window4_new(ucl_type_ecc_digit_affine_point Q, DIGIT *m,
                                  ucl_type_curve *curve_params);

int precomputation_Q_generic(u32 rsize, u32 digits, DIGIT qx[96][8], DIGIT qy[96][8], DIGIT *x,
                             DIGIT *y, DIGIT *p, u32 curve);

#ifdef P160
void ecc_mult_jacobian_window4_p160(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P160
#ifdef P192
void ecc_mult_jacobian_window4_p192(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P192
#ifdef P256
void ecc_mult_jacobian_window4_p256(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P256
#ifdef P384
void ecc_mult_jacobian_window4_p384(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P384
#ifdef P224
void ecc_mult_jacobian_window4_p224(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P224
#ifdef P521
void ecc_mult_jacobian_window4_p521(DIGIT *xres, DIGIT *yres, DIGIT *m, DIGIT *a, DIGIT *x,
                                    DIGIT *y, DIGIT *p, DIGIT digits, int curve);
#endif //P521

int ithbit(DIGIT *x, int size, int ith);

/** <b>compute bits size and word size of a curve domain parameter p</b>.
* @param[out] np: pointer to the real number of bits of p, i.e. the largest position with a bit set to 1
* @param[out] digits_tmp: pointer to the real number of words (DIGIT) of p, i.e. the largest position with a word non null
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
 *
 * @ingroup UCL_ECC */
void msbit_and_size(int *np, int *digits_tmp, ucl_type_curve *curve_params);

/** <b>extend the msbit of a sclar for a curve domain parameter</b>.
* @param[in,out] KEXT: the pointer to the extended scalar array, with a bit equal to 1 appended after the msb equal to 1
* @param[out] digits_ext: the pointer to the new size of the new scalar
* @param[in] np: the real number of bits of p, i.e. the largest position with a bit set to 1
* @param[int] digits_tmp: the real number of words (DIGIT) of p, i.e. the largest position with a word non null
* @param[in] *curve_params: the pointer to a ucl_type_curve structure, containing the curve domain parameters (already existing ones are described in the documentation) 
 *
 * @ingroup UCL_ECC */
void set_msbit_curve(DIGIT *KEXT, DIGIT *digits_ext, int np, int digits_tmp,
                     ucl_type_curve *curve_params);

#ifdef SM2P256
/** <b>SM2 DSA signature</b>.
 * compute a SM2 P256 DSA signature, with curves parameters in input
*
* @param[out] *r: output, the r value of the computed signature
* @param[out] *s: output, the s value of the computed signature
* @param[in] *d: input, the private key value
* @param[in] *xA: input, the x affine coordinate of the public key
* @param[in] *yA: input, the y affine coordinate of the public key
* @param[in] *input: input, the message or the hash digest to be signed
* @param[in] inputlength: input, the input length, in bytes
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - SM2FP256 (as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00) or SM2VP256
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)
     - UCL_SM3: hash function, if UCL_MSG_INPUT is used.
     - Examples are:
       - SM2FP256<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest on SM2 P256 curve, r not precomputed
       - SM2FP256<<UCL_CURVE_SHIFT+UCL_PRECOMP_R<<UCL_PRECOMP_SHIFT+UCL_NO_INPUT<<UCL_INPUT_SHIFT: pre-compute r on SM2 P256.
       - UCL_R_PRECOMP<<UCL_PRECOMP_SHIFT+SM2FP256<<UCL_CURVE_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT+ UCL_SM3<<UCL_HASH_SHIFT:signing a message, to be hashed with SM3, using pre-computed r
* @param[in] *a: input, the a parameter of the curve
* @param[in] *b: input, the b parameter of the curve
* @param[in] *p: input, the p parameter of the curve
* @param[in] *xG: input, the x affine coordinate of the base point
* @param[in] *yG: input, the y affine coordinate of the base point
* @param[in] *n: input, the n parameter of the curve

 * @return Error code
 * @return computed signature

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not correctly computed
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_sign_curve(u8 *r, u8 *s, u8 *d, u8 *xA, u8 *yA, u8 *input, u32 inputlength, u8 *ida,
                          u8 *entla, u32 configuration, u8 *a, u8 *b, u8 *p, u8 *xG, u8 *yG, u8 *n);

/** <b>SM2 DSA signature</b>.
 * compute a SM2 P256 DSA signature, for SM2 P256 only (as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00)
*
* @param[out] *r: output, the r value of the computed signature
* @param[out] *s: output, the s value of the computed signature
* @param[in] *d: input, the private key value
* @param[in] *xA: input, the x affine coordinate of the public key
* @param[in] *yA: input, the y affine coordinate of the public key
* @param[in] *input: input, the message or the hash digest to be signed
* @param[in] inputlength: input, the input length, in bytes
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - SM2FP256, as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)
     - UCL_SM3: hash function, if UCL_MSG_INPUT is used.
     - Examples are:
       - SM2FP256<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest on SM2 P256 curve, r not precomputed
       - SM2FP256<<UCL_CURVE_SHIFT+UCL_PRECOMP_R<<UCL_PRECOMP_SHIFT+UCL_NO_INPUT<<UCL_INPUT_SHIFT: pre-compute r on SM2 P256.
       - UCL_R_PRECOMP<<UCL_PRECOMP_SHIFT+SM2FP256<<UCL_CURVE_SHIFT+ UCL_MSG_INPUT<<UCL_INPUT_SHIFT+ UCL_SM3<<UCL_HASH_SHIFT:signing a message, to be hashed with SM3, using pre-computed r
 * @return Error code
 * @return computed signature

 * @retval #UCL_OK if the computation is ok
 * @retval #UCL_ERROR if the computation is not ok
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_sign(u8 *r, u8 *s, u8 *d, u8 *xA, u8 *yA, u8 *input, u32 inputlength, u8 *ida,
                    u8 *entla, u32 configuration);
/** <b>ECDSA signature</b>.
 * verify a SM2 P256 DSA signature, with curve parameters in input
*
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *xA: input, the x affine coordinate of the public key
* @param[in] *yA: input, the y affine coordinate of the public key
* @param[in] *input: input, the input message or hash which signature to be checked
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
* @param[in] configuration (combination of any of these lines) 
     - UCL_R_PRECOMP or UCL_PRECOMP_R:
     - UCL_R_PRECOMP: using precomputed r to finish the signature computation,
     - UCL_PRECOMP_R: to only pre-compute the r value,
     - SM2FP256 (as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00) or SM2VP256
     - UCL_MSG_INPUT or UCL_HASH_INPUT or UCL_NO_INPUT
     - UCL_MSG_INPUT: the message will be hashed first,
     - UCL_HASH_INPUT: the message is already the hash digest,
     - UCL_NO_INPUT: for r precomputation (UCL_PRECOMP_R)
     - UCL_SM3: hash function, if UCL_MSG_INPUT is used.
     - Examples are:
       - SM2FP256<<UCL_CURVE_SHIFT+UCL_HASH_INPUT<<UCL_INPUT_SHIFT: signing a hash digest on a SM2 P256 curve, using the given curve parameters
       - SM2VP256<<UCL_CURVE_SHIFT)^(UCL_MSG_INPUT<<UCL_INPUT_SHIFT)^(UCL_SM3<<UCL_HASH_SHIFT)^(UCL_NO_PRECOMP_TRICK<<UCL_PRECOMP_TRICK_SHIFT): signing a message on a SM2 P256 curve, using the given curve parameters
* @param[in] *a: input, the a parameter of the curve
* @param[in] *b: input, the b parameter of the curve
* @param[in] *p: input, the p parameter of the curve
* @param[in] *xG: input, the x affine coordinate of the base point
* @param[in] *yG: input, the y affine coordinate of the base point
* @param[in] *n: input, the n parameter of the curve

 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_verify_curve(u8 *r, u8 *s, u8 *xA, u8 *yA, u8 *input, u32 inputlength, u8 *ida,
                            u8 *entla, u32 configuration, u8 *a, u8 *b, u8 *p, u8 *xG, u8 *yG,
                            u8 *n);

/** <b>ECDSA signature</b>.
 * Verify a SM2 P256 DSA signature, directly from the message hash, using the SM2 curve as defined in SM2 Digital Signature Algorithm draft-shen-sm2-ecdsa-00
*
* @param[in] *r: input, the r value of the signature to be verified
* @param[in] *s: input, the s value of the signature to be verified
* @param[in] *xA: input, the x affine coordinate of the public key
* @param[in] *yA: input, the y affine coordinate of the public key
* @param[in] *input: input, the input message or hash which signature to be checked
* @param[in] inputlength: input, the input length, in bytes
* @param[in] *ida: input, the IDA identifier
* @param[in] *entla: input, the ENTLA (entlenA bits) parameter
 * @return Error code

 * @retval #UCL_OK if the signature is verified
 * @retval #UCL_ERROR if the signature is not verified
 * @retval #UCL_INVALID_INPUT in case of wrong parameters 
 *
 * @ingroup UCL_ECDSA */
int ucl_sm2dsa_verify(u8 *r, u8 *s, u8 *xA, u8 *yA, u8 *input, u32 inputlength, u8 *ida, u8 *entla,
                      u32 configuration);
int ucl_sm2dsa_sign_sm2fp256_sm3(u8 *r, u8 *s, u8 *d, u8 *xQ, u8 *yQ, u8 *m, u32 MsgLng, u8 *ida,
                                 u8 *entla);
int ucl_sm2dsa_verify_sm2fp256_sm3(u8 *r, u8 *s, u8 *xQ, u8 *yQ, u8 *m, u32 MsgLng, u8 *ida,
                                   u8 *entla);

#endif
#ifdef __cplusplus
}
#endif /* __cplusplus  */
#endif //ECDSA_GENERIC_API
