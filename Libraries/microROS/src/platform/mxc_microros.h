/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_H
#define _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_H

#include <time.h>
#include "mxc_transports.h"
#include "mxc_microros_options.h"

/* 
This function is the application code's "main" function.  It will be run as
an RTOS task by the MSDK platform layer after initializing micro-ROS.  
Conventionally, this is defined in an "app.c" file for each project. 
*/
extern void appMain(void *argument);

#endif // _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_H
