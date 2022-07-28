#ifndef _UCL_ALGORITHMS_VERSIONS_DEF
#define _UCL_ALGORITHMS_VERSIONS_DEF

#define UCL_AES_VERSION 0x010101
#define UCL_DES_VERSION 0x010101
#define UCL_3DES_VERSION 0x010101
#define UCL_RSA_VERSION 0x010101
#define UCL_DH_VERSION 0x010101
#define UCL_ECDSA_VERSION 0x010101
#define UCL_SHA256_VERSION 0x010101
#define UCL_SHA384_VERSION 0x010101
#define UCL_SHA512_VERSION 0x010101
#define UCL_SHA3_VERSION 0x010101
#define UCL_SHA224_VERSION 0x010101
#define UCL_ECDH_VERSION 0x010101
#define UCL_RNG_VERSION 0x010101

u32 ucl_get_3des_version(void);
u32 ucl_get_aes_version(void);
u32 ucl_get_des_version(void);
u32 ucl_get_rsa_version(void);
u32 ucl_get_dh_version(void);
u32 ucl_get_ecdsa_version(void);
u32 ucl_get_ecdh_version(void);
u32 ucl_get_rng_version(void);
u32 ucl_get_sha256_version(void);
u32 ucl_get_sha384_version(void);
u32 ucl_get_sha512_version(void);
u32 ucl_get_sha224_version(void);
u32 ucl_get_sha3_version(void);
#endif
