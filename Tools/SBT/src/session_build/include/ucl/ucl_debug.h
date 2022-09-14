
/*============================================================================
 *
 * ucl_debug.h - debug/trace helpers
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

#ifndef UCL_DEBUG_H
#define UCL_DEBUG_H
#include "ucl/ucl_config.h"
#include <stdio.h>

#define __userland_format(fmt) "%s:%d " fmt "\n"
#define __userland_format_source __func__, __LINE__

#define _userland_debug(fmt) __userland_format(fmt), __userland_format_source

#ifdef DEBUG /* defined using the kernel config */
#warning "DEBUG traces are enabled"
#define dbg_print(fmt, arg...) printf(_userland_debug(fmt), ##arg)
#else
#define dbg_print(fmt, arg...)
#endif

#endif /*UCL_DEBUG_H*/
