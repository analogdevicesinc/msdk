/*============================================================================
 *
 *  ucl_info.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright (C) 2004-2010 Innova Card.
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
 * Purpose : Library information
 *
 *==========================================================================*/
#ifndef ucl_INFO_H_
#define ucl_INFO_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_info.h
 * @defgroup INFO Information
 * General Library Information.
 *
 * @par Header:
 * @link ucl_info.h ucl_info.h @endlink
 *
 */

/** <b>Get Version</b>.
 * Get the library version number.
 *
 * @return string
 *
 * @ingroup INFO */
const char* ucl_get_version(void);

/** <b>Get Copyright</b>.
 * Get the library copyright.
 *
 * @return string
 *
 * @ingroup INFO */
const char* ucl_get_copyright(void);

/** <b>Get Build Date</b>.
 * Get the library build date.
 *
 * @return string
 *
 * @ingroup INFO */
const char* ucl_get_build_date(void);

/** <b>Get Options</b>.
 * Get the library options.
 *
 * @return string
 *
 * @ingroup INFO */
const char* ucl_get_options(void);
#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /* ucl_INFO_H_ */
