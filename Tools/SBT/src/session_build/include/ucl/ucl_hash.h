//these defines are used to select or not hash functions
//useful on platforms with limited resources

#ifndef _UCL_HASH_H
#define _UCL_HASH_H

#define HASH_SHA256
#ifndef ROMCODE_P256
#define HASH_SHA1
#define HASH_SHA384
#define HASH_SHA224
#define HASH_SHA512
#define HASH_SIA256
#define HASH_MD5
#define HASH_RIPEMD160
#define HASH_SM3
#define HASH_SHA3
#define HASH_SHA3_224
#define HASH_SHA3_256
#define HASH_SHA3_384
#define HASH_SHA3_512
#endif
#define MAX_HASH_FUNCTIONS 14
#define UCL_UNDEFINED_HASH -1
//extern int hash_size[MAX_HASH_FUNCTIONS];


#endif//UCL_HASH
