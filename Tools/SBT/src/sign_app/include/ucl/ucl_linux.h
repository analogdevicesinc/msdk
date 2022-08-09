/*============================================================================
 *
 * ucl_linux.h
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
#ifndef _UCL_LINUX_H_
#define _UCL_LINUX_H_

/** @defgroup UCL_LINUX UCL Linux Configuration
 * List of configuration parameters.
 */

/** <b>Dev to access USIP TRNG</b>.
 * '/dev/hwrandom'.
 *
 * @note To create the device: mknode /dev/hwrandom c 10 183
 *
 * @ingroup UCL_LINUX */
#if __usip == 1
#define UCL_UTRNG_DEV "/dev/hwrandom"
#elif __jibe == 1
#define UCL_UTRNG_DEV "/dev/hw_random"
#else
#define UCL_UTRNG_DEV "/dev/urandom"
#endif /* __usip */

#endif /*_UCL_LINUX_H_*/
