/*============================================================================
 *
 * userland_crypto.c - connects to the HW drivers using a socket
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009-2014 Maxim Integrated Products, Inc.
 * Copyright © 2009 Innova Card. All rights reserved. Do not disclose.
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : connection to the HW crypto drivers
 *
 * Revisions +                    changes
 *  2.4.0    |  Initial version without any sanity check
 *
 *==========================================================================*/

#ifndef USERLAND_CRYPTO_H
#define USERLAND_CRYPTO_H
#include "ucl/ucl_config.h"



#if defined (JIBE_LINUX_HW)
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/types.h>
#include <ucl/ucl_des.h>
#include <ucl/ucl_aes.h>

#define AF_ALG 38
#define SOL_ALG 279

#define SPLICE_F_GIFT	(0x08)	/* pages passed in are a gift */
#define AES_BLOCKLENGTH	(UCL_AES_BLOCKSIZE)
#define DES3_KEYLENGTH	(UCL_3DES_KEYSIZE)
#define DES_BLOCKLENGTH	(UCL_DES_BLOCKSIZE)
#define DES_KEYLENGTH	(UCL_DES_KEYSIZE)


#ifndef TRUE
#define TRUE	(!!1)
#define FALSE	(!TRUE)
#endif

#define VOID_PARAM	((unsigned int) -1)

struct sockaddr_alg {
	__u16	salg_family;
	__u8	salg_type[14];
	__u32	salg_feat;
	__u32	salg_mask;
	__u8	salg_name[64];
};

struct af_alg_iv {
	__u32	ivlen;
	__u8	iv[0];
};

/* Socket options */
#define ALG_SET_KEY			1
#define ALG_SET_IV			2
#define ALG_SET_OP			3

/* Operations */
#define ALG_OP_DECRYPT			0
#define ALG_OP_ENCRYPT			1

int crypto_hw_init(void);
int crypto_hw_exit(void);
int crypto_hw_set_key(const unsigned char *algo, const unsigned char *key, unsigned int keylen);
int crypto_hw_set_3des_key(const unsigned char *key, unsigned int start, unsigned int end);
int crypto_hw_get_key(unsigned char *key);
void crypto_hw_3des_flush_key( void );
int crypto_hw_set_mode(int mode);
int crypto_hw_get_mode( void );
int crypto_hw(unsigned int op,
	     const unsigned char *bufferIn,
	     unsigned char *bufferOut,
	     unsigned int buflen,
	     unsigned char *oiv,
         unsigned int blocklen);


#endif /*#if defined (JIBE_LINUX_HW)*/

#endif /*USERLAND_CRYPTO_H*/
