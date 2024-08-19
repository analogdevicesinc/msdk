/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      Version driver implementation.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 *
 * Copyright (c) 2022-2023 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*************************************************************************************************/

#ifndef PAL_VERSION_H
#define PAL_VERSION_H

#include "pal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup PAL_VERSION
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief      PAL Version structure. */
typedef struct
{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  uint32_t commit;
  char *buildDate;
} PalVersion_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Get versioning information.
 *
 *  \return     PAL Version structure.
 */
/*************************************************************************************************/
const PalVersion_t *PalGetVersion(void);

/*! \} */    /* PAL_VERSION */

#ifdef __cplusplus
};
#endif

#endif /* PAL_VERSION_H */
