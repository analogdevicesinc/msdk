/*============================================================================
 *
 * ucl_sys.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
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
 * Purpose :
 *
 *==========================================================================*/
#ifndef _UCL_SYS_H_
#define _UCL_SYS_H_

#include "ucl/ucl_config.h"

//MAX32550 A versions definitions
//the function ucl_get_chip_version returns one of those values for a rev A chip
#define MAX32550_A1_CHIP  1
#define MAX32550_A2_CHIP  2
#define MAX32550_A3_CHIP  3
#define MAX32550_B1_CHIP  4
#define MAX32550_B2_CHIP  5
#define X86_PLATFORM      6
#define USIP_PLATFORM     7
#define JIBE_PLATFORM     8
#define MAX32555_PLATFORM 9
#define MAX32666_PLATFORM 10
#define MAX32652_PLATFORM 11
#define MAX32620_PLATFORM 12
#define MAX32621_PLATFORM 13
#define MAX32630_PLATFORM 14
#define MAX32631_PLATFORM 15
#define MAX32565_PLATFORM 16
#define MAX32600_PLATFORM 17
#define ARM_M3_PLATFORM   18

#define UNDEFINED_CHIP 0

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_sys.h
 * @defgroup UCL_SYSTEM UCL System
 * UCL System Functions.
 *
 * @par Header:
 * @link ucl_sys.h ucl_sys.h @endlink
 *
 */

/** <b>UCL Init</b>.
 * Cryptographic Library Initialisation.
 * Initialisation of stack and hardware interfaces (if available).
 *
 * @param[in] buffer Work buffer for pkc
 * @param[in] size   Size of the buffer (number of unsigned ints)
 *
 * @pre @p size must greater than or equal to 1024.
 *
 * @note The UCL Stack error does not matter if you don't use #UCL_PKC functions.
 * @note The error "USIP(R) TRNG not available" occurs for USIP Linux platform
 * if the file '/dev/hwrandom' does not exist.
 * @return #UCL_OK or Error vector
 *
 * @retval The error vector is a combination of:
 *     @li 0x001: UCL Stack error (Buffer NULL or invalid size)
 *     @li 0x010: USIP AES not available
 *     @li 0x020: USIP AES Corrrupted
 *     @li 0x100: USIP TRNG not available
 *     @li 0x200: USIP TRNG Corrupted
 *
 * @warning After the version 2.1.0 the @p buffer is mandatory
 *
 * @ingroup UCL_SYSTEM
 */
int ucl_init(u32* buffer, u32 size);

/** <b>Chip identification</b>.
 * chip and chip version identification
 *
 * @return error code
 * @retval MAX32550_A1_CHIP
 * @retval MAX32550_A2_CHIP
 * @retval MAX32550_A3_CHIP
 * @retval MAX32550_B1_CHIP
 * @retval UNDEFINED_CHIP
 *
 * @ingroup UCL_SYSTEM
 */
int ucl_get_chip_version(void);

/** <b>Crypto Controller reset</b>.
 * crypto controller FSM and registers clearing
 * no returned value
 *
 * @ingroup UCL_SYSTEM
 */

void ucl_crypto_block_reset(void);

#if defined(JIBE_LINUX_HW)
int ucl_exit(void);
#endif /*#if defined(__jibe) && defined(JIBE_USERLAND_CRYPTO) && defined(__linux) */
#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_SYS_H_ */
