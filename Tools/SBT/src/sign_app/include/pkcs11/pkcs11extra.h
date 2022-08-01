/*
 * (C) Copyright nCipher Corporation Limited 2001.
 * All rights reserved.
 */

#ifndef _PKCS11EXTRA_H_
#define _PKCS11EXTRA_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

/* Use a range within CK*_VENDOR_DEFINED unlikely to clash with anyone
   else's, cf. http://lxr.mozilla.org/mozilla/ident?i=CKO_NETSCAPE */
#define NFCK_VENDOR_NCIPHER 0xde436972UL
#define CKA_NCIPHER         (CKA_VENDOR_DEFINED | NFCK_VENDOR_NCIPHER)
#define CKM_NCIPHER         (CKM_VENDOR_DEFINED | NFCK_VENDOR_NCIPHER)
#define CKK_NCIPHER         (CKK_VENDOR_DEFINED | NFCK_VENDOR_NCIPHER)

/* Some things can't be supported specifically because of FIPS mode
   restrictions. We want the error codes to be recognizable if
   possible, so don't use NFCK_VENDOR_NCIPHER here. */
/* These ones should no longer be necessary, but leave them
   defined for existing applications and test programs. */
#define CKR_FIPS_TEMPLATE_INCONSISTENT (CKR_VENDOR_DEFINED | CKR_TEMPLATE_INCONSISTENT)
#define CKR_FIPS_MECHANISM_INVALID     (CKR_VENDOR_DEFINED | CKR_MECHANISM_INVALID)

/* We need a FIPS authorization token from a cardset,
   even if generating module protected keys on a module slot */
#define CKR_FIPS_TOKEN_NOT_PRESENT (CKR_VENDOR_DEFINED | CKR_TOKEN_NOT_PRESENT)

#define CKR_FIPS_FUNCTION_NOT_SUPPORTED (CKR_VENDOR_DEFINED | CKR_FUNCTION_NOT_SUPPORTED)

/* Our vendor-defined key types. */
#define CKK_KCDSA (CKK_NCIPHER + 0x7UL)
#define CKK_SEED  (CKK_NCIPHER + 0x33UL)

/* These were vendor defined, but are added to v2.30 spec */
/* The old values will be kept as CKK_NC_XXX_HMAC to allow */
/* the best possible interop with old code */
/* We define CKK_XXX_HMAC to the values in the 2.30 spec */
/* if they are not already defined (by a 2.30 header) */
#define CKK_NC_SHA_1_HMAC (CKK_NCIPHER + 1) /*RFC2104*/
#ifndef CKK_SHA_1_HMAC
#define CKK_SHA_1_HMAC 0x00000028
#endif
#define CKK_NC_MD5_HMAC (CKK_NCIPHER + 2)
#ifndef CKK_MD5_HMAC
#define CKK_MD5_HMAC 0x00000027
#endif

/* New key types and associated mechanisms for v2.20 */
/* Protected in an ifndef to allow them to be used with v2.01 headers */
#ifndef CKK_SHA256_HMAC
#define CKK_SHA256_HMAC 0x0000002B
#define CKK_SHA384_HMAC 0x0000002C
#define CKK_SHA512_HMAC 0x0000002D

#define CKM_SHA256_HMAC         0x00000251
#define CKM_SHA256_HMAC_GENERAL 0x00000252
#define CKM_SHA384_HMAC         0x00000261
#define CKM_SHA384_HMAC_GENERAL 0x00000262
#define CKM_SHA512_HMAC         0x00000271
#define CKM_SHA512_HMAC_GENERAL 0x00000272
#endif

/* These are in v2.20 ammendment 3 only so in seperate ifndef */
/* just in case people are using v2.20 original headers */
#ifndef CKK_SHA224_HMAC
#define CKK_SHA224_HMAC 0x0000002E

#define CKM_SHA224_HMAC         0x00000256
#define CKM_SHA224_HMAC_GENERAL 0x00000257
#endif

/* Our vendor-defined mechanisms. */
/* Added for customer special requirements */
#define CKM_WRAP_RSA_CRT_COMPONENTS (CKM_NCIPHER + 0x1UL)
#define CKM_CAC_TK_DERIVATION       (CKM_NCIPHER + 0x2UL)

#define CKM_MD5_HMAC_KEY_GEN      (CKM_NCIPHER + 0x6UL) /* no params */
#define CKM_NC_MD5_HMAC_KEY_GEN   (CKM_NCIPHER + 0x6UL) /* no params */
#define CKM_SHA_1_HMAC_KEY_GEN    (CKM_NCIPHER + 0x3UL) /* no params */
#define CKM_NC_SHA_1_HMAC_KEY_GEN (CKM_NCIPHER + 0x3UL) /* no params */
/* CKM_SHA_1_HMAC_KEY_GEN should have had an NC in its name to show it is */
/* a vendor defined mech. The old name is still defined to allow old code */
/* to still work. Ditto for CKM_MD5_HMAC_KEY_GEN */

#define CKM_NFKM_WRAP_BLOB (CKM_NCIPHER | CKM_DES3_CBC_PAD) /* 0x00000136 */

/* New vendor-defined mechanisms for v2.20 */
/* Spec inconsistently allows HMAC with generic secret keys; we don't. */
#define CKM_NC_SHA224_HMAC_KEY_GEN (CKM_NCIPHER + 0x24UL) /* no params */
#define CKM_NC_SHA256_HMAC_KEY_GEN (CKM_NCIPHER + 0x25UL) /* no params */
#define CKM_NC_SHA384_HMAC_KEY_GEN (CKM_NCIPHER + 0x26UL) /* no params */
#define CKM_NC_SHA512_HMAC_KEY_GEN (CKM_NCIPHER + 0x27UL) /* no params */

/* Adding KCDSA key genration and usage */
#define CKM_KCDSA              (CKM_NCIPHER + 0x18UL)
#define CKM_KCDSA_KEY_PAIR_GEN (CKM_NCIPHER + CKM_DSA_KEY_PAIR_GEN) /* 0x00000010 */

#define CKM_KCDSA_SHA1          (CKM_NCIPHER + 0x4UL)
#define CKM_KCDSA_HAS160        (CKM_NCIPHER + 0x5UL)
#define CKM_KCDSA_RIPEMD160     (CKM_NCIPHER + 0x11UL)
#define CKM_KCDSA_PARAMETER_GEN (CKM_NCIPHER + CKM_DSA_PARAMETER_GEN) /* 0x00002000 */

/* New mechanism type CKM_KCDSA. When used
in a CK_MECHANISM structure passed to C_SignInit or C_VerifyInit
the pParameter field must point to a mechanism parameter of a new
type, CK_KCDSA_PARAMS. */

typedef struct CK_KCDSA_PARAMS {
    CK_MECHANISM_PTR digestMechanism;
    CK_BBOOL dataIsHashed;
} CK_KCDSA_PARAMS;

typedef CK_KCDSA_PARAMS CK_PTR CK_KCDSA_PARAMS_PTR;

/* Adding SEED mech key genration and usage */
#define CKM_SEED_KEY_GEN     (CKM_NCIPHER + 0x12UL)
#define CKM_SEED_ECB         (CKM_NCIPHER + 0x13UL)
#define CKM_SEED_CBC         (CKM_NCIPHER + 0x14UL)
#define CKM_SEED_CBC_PAD     (CKM_NCIPHER + 0x15UL)
#define CKM_SEED_MAC         (CKM_NCIPHER + 0x16UL)
#define CKM_SEED_MAC_GENERAL (CKM_NCIPHER + 0x17UL)

#ifndef CKM_SEED_ECB_ENCRYPT_DATA
#define CKM_SEED_ECB_ENCRYPT_DATA (CKM_NCIPHER + 0x20UL)
#endif

#ifndef CKM_SEED_CBC_ENCRYPT_DATA
#define CKM_SEED_CBC_ENCRYPT_DATA (CKM_NCIPHER + 0x19UL)
typedef CK_AES_CBC_ENCRYPT_DATA_PARAMS CK_SEED_CBC_ENCRYPT_DATA_PARAMS;
typedef CK_AES_CBC_ENCRYPT_DATA_PARAMS CK_PTR CK_SEED_CBC_ENCRYPT_DATA_PARAMS_PTR;
#endif

/* Hash mechanism asscociated with KCDSA*/
#define CKM_HAS160 (CKM_NCIPHER + 0x8UL)

/* Extend to support C_DeriveKey */
/* We used to have these as synonyms for CKM_DES_ECB and CKM_DES3_ECB,
   but v2.20 has now added them as standard. If we aren't using v2.20
   headers, use the same values.
*/
#ifndef CKM_DES_ECB_ENCRYPT_DATA
#define CKM_DES_ECB_ENCRYPT_DATA  0x00001100
#define CKM_DES3_ECB_ENCRYPT_DATA 0x00001102
#endif

/* New mechanims which nCore provide but the standard spec does not up to v2.20 */

/* Used with C_DeriveKey to create a public half form a loan private key. */
#ifndef CKM_PUBLIC_FROM_PRIVATE
#define CKM_PUBLIC_FROM_PRIVATE (CKM_NCIPHER + 0x2100UL)
#endif

/* Used with C_Sign and AES_CMAC may be in offical spec post v2.20 */

/* Our original implementation did not match that given in the v2.30 */
/* draft specifications. We had a CKM_AES_CMAC that was actually a */
/* general CMAC, whereas the v2.30 has 2 mechanisms with CKM_AES_CMAC_GENERAL */
/* being what we had called CKM_AES_CMAC. */
/* The value we gave the old CKM_AES_CMAC is going to become CKM_NC_AES_CMAC. */
/* The other mechanisms will be defined here for if v2.20 or earlier headers are used */
#ifndef CKM_AES_CMAC
#define CKM_AES_CMAC         (0x00001089UL)
#define CKM_AES_CMAC_GENERAL (0x0000108AUL)
/* These are the values given by the draft PKCS#11 v.2.30 specification */
#endif
#define CKM_NC_AES_CMAC (CKM_NCIPHER + 0x1087UL)
/* This is the value we used to give to our version of CKM_AES_CMAC */

/* Our vendor-defined attributes. */

/* Map PKCS#11 keys to NFKM keys. This is specific to us. */
#define CKA_NFKM_ID      (CKA_NCIPHER + CKA_ID)    /* ID = 0x00000102 */
#define CKA_NFKM_APPNAME (CKA_NCIPHER + CKA_LABEL) /* LABEL = 0x00000003 */

/* Support for CKM_CAC_TK_DERIVATION (customer special requirement) */
#define CKA_TKC1 (CKA_NCIPHER + 0x11UL)
#define CKA_TKC2 (CKA_NCIPHER + 0x12UL)
#define CKA_TKC3 (CKA_NCIPHER + 0x13UL)

/* for use with KCDSA needed x and y values in private key */
#define CKA_PRIVATE_VALUE (CKA_NCIPHER + 0x14UL)
#define CKA_PUBLIC_VALUE  (CKA_NCIPHER + 0x15UL)
/* Attributes for KCDSAcomm key to produce an Initvalues (iv). */
#define CKA_SEED    (CKA_NCIPHER + 0x16UL)
#define CKA_COUNTER (CKA_NCIPHER + 0x17UL)

#define CKA_NFKM_HASH (CKA_NCIPHER + 0x18UL)

#define CKA_SAM_REAL           (CKA_NCIPHER + 0x30UL)
#define CKA_SAM_REAL_CHECKED   (CKA_NCIPHER + 0x31UL)
#define CKA_SAM_REAL_INSECURE  (CKA_NCIPHER + 0x32UL)
#define CKA_SAM_REAL_EXPLICIT  (CKA_NCIPHER + 0x33UL)
#define CKA_SAM_REAL_UNWRAPKEK (CKA_NCIPHER + 0x34UL)
#define CKA_SAM_REAL_IMPORT    (CKA_NCIPHER + 0x35UL)
#define CKA_SAM_REAL_WEAK      (CKA_NCIPHER + 0x36UL)
#define CKA_SAM_REAL_SHORT     (CKA_NCIPHER + 0x37UL)

#define CKA_SAM_CONSIDERED            (CKA_NCIPHER + 0x40UL)
#define CKA_SAM_CONSIDERED_CHECKED    (CKA_NCIPHER + 0x41UL)
#define CKA_SAM_CONSIDERED_INSECURE   (CKA_NCIPHER + 0x42UL)
#define CKA_SAM_CONSIDERED_EXPLICIT   (CKA_NCIPHER + 0x43UL)
#define CKA_SAM_CONSIDERED_UNWRAPKEK  (CKA_NCIPHER + 0x44UL)
#define CKA_SAM_CONSIDERED_IMPORT     (CKA_NCIPHER + 0x45UL)
#define CKA_SAM_CONSIDERED_WEAK       (CKA_NCIPHER + 0x46UL)
#define CKA_SAM_CONSIDERED_SHORT      (CKA_NCIPHER + 0x47UL)
#define CKA_SAM_CONSIDERED_UNWRAPMECH (CKA_NCIPHER + 0x48UL)
#define CKA_SAM_CONSIDERED_TOKEN      (CKA_NCIPHER + 0x49UL)

/* Specific to the keyAuthority build.  Is this key currently
   the active key in its keyset?*/
#define CKA_KA_ACTIVE (CKA_NCIPHER + 0x50UL)

/* This struct and CKD definitions are specified as per spec v2.11 but 
   not declared in the v2.11 headers.
   (v2.11 headers were later corrected to include the missing typedefs
   and defines, so we now check with #ifndef so this will work with later
   versions.)
*/
#ifndef CKD_SHA1_KDF
typedef CK_ULONG CK_EC_KDF_TYPE;

typedef struct CK_ECDH1_DERIVE_PARAMS {
    CK_EC_KDF_TYPE kdf;
    CK_ULONG ulSharedDataLen;
    CK_BYTE_PTR pSharedData;
    CK_ULONG ulPublicDataLen;
    CK_BYTE_PTR pPublicData;
} CK_ECDH1_DERIVE_PARAMS;

typedef CK_ECDH1_DERIVE_PARAMS CK_PTR CK_ECDH1_DERIVE_PARAMS_PTR;

#define CKD_NULL     0x00000001
#define CKD_SHA1_KDF 0x00000002
#endif

/* These CKD definitions are defined in the proposed v2.30 spec but
   the v2.30 headers are not yet released, so they are included here
   to work with the v2.20 headers. #ifndef is used so this will still
   work with later versions of the headers
*/

#ifndef CKD_SHA224_KDF

#define CKD_SHA224_KDF 0x00000005
#define CKD_SHA256_KDF 0x00000006
#define CKD_SHA384_KDF 0x00000007
#define CKD_SHA512_KDF 0x00000008

#endif

/*
   Adding a new vendor defined mechanism CK_NEC_AES_CMAC_KEY_DERIVATION
   based on SP800-108 standards.
*/

#ifndef CKM_NC_AES_CMAC_KEY_DERIVATION

#define CKM_NC_AES_CMAC_KEY_DERIVATION       (CKM_NCIPHER + 0x300UL)
#define CKM_NC_AES_CMAC_KEY_DERIVATION_SCP03 (CKM_NCIPHER + 0x301UL)

typedef struct CK_NC_AES_CMAC_KEY_DERIVATION_PARAMS {
    CK_ULONG ulContextLen;
    CK_BYTE_PTR pContext; /* optional context data */
    CK_ULONG ulLabelLen;
    CK_BYTE_PTR pLabel; /* optional label data */
} CK_NC_AES_CMAC_KEY_DERIVATION_PARAMS;

typedef CK_NC_AES_CMAC_KEY_DERIVATION_PARAMS CK_PTR CK_NC_AES_CMAC_KEY_DERIVATION_PARAMS_PTR;

#endif

#include "pkcs11tkofn.h"

#define __PASTE(x, y) x##y

/* ==============================================================
 * Define the "extern" form of our extra entry points.
 * ==============================================================
 */

#define CK_NEED_ARG_LIST              1
#define CK_PKCS11_FUNCTION_INFO(name) extern CK_DECLARE_FUNCTION(CK_RV, name)
#include "pkcs11fkofn.h"
#undef CK_NEED_ARG_LIST
#undef CK_PKCS11_FUNCTION_INFO

/* ==============================================================
 * Define the typedef form of the extra entry points.
 * ==============================================================
 */

#define CK_NEED_ARG_LIST              1
#define CK_PKCS11_FUNCTION_INFO(name) typedef CK_DECLARE_FUNCTION_POINTER(CK_RV, __PASTE(CK_, name))
#include "pkcs11fkofn.h"
#undef CK_NEED_ARG_LIST
#undef CK_PKCS11_FUNCTION_INFO

/* ==============================================================
 * Define structed vector of entry points, like a CK_FUNCTION_LIST
 * but also including the extra ones. This was declared but not
 * defined in pkcs11tkofn.h
 * ==============================================================
 */
#define CK_PKCS11_FUNCTION_INFO(name) __PASTE(CK_, name) name;

struct CK_FUNCTION_LIST_EX {
    CK_VERSION version; /* Cryptoki version */

/* Pile all the function pointers into the CK_FUNCTION_LIST. */
/* pkcs11f.h has all the information about the Cryptoki
 * function prototypes. */
#include "pkcs11f.h"
#include "pkcs11fkofn.h"
};

#undef CK_PKCS11_FUNCTION_INFO
#undef __PASTE

#ifdef __cplusplus
}
#endif

#endif /* _PKCS11EXTRA_H_ */
