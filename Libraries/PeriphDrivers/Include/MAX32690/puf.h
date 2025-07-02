/**
 * @file
 * @brief Physically Unclonable Function (PUF), definitions and function prototypes.
 */

/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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
 *
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PUF_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PUF_H_

/* **** Includes **** */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup puf Physically Unclonable Function
 * @ingroup periphlibs
 * @brief This is the high level API for the PUF.
 * @{
 */

/***** Definitions *****/

/**
 * @brief   Brief of any definitions, enumes, structures, etc
 */
typedef struct {
    uint32_t template;
} mxc_puf_struct_template_t;

/***** Function Prototypes *****/

/**
 * @brief   Template Brief.
 * @param   param1   First Parameter.
 * 
 * @return  #E_NO_ERROR if everything is successful, @ref MXC_Error_Codes
 *             "error" if unsuccessful.
 */
int MXC_PUF_TemplateFunction(uint32_t param1);

/**@} end of group pulsetrains*/

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_PT_H_
