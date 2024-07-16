/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
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
#ifndef EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_SDHC_H_
#define EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_SDHC_H_

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gpio.h"
#include "uart.h"
#include "ff.h"

/***** Definitions *****/

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define MAXLEN 256

/***** FUNCTION PROTOTYPES *****/

void generateMessage(unsigned length);

int mount();

int umount();

int formatSDHC();

int getSize();

int ls();

int createFile(char *file_name, unsigned int length);

int appendFile(char *file_name, unsigned int length);

int mkdir(char *dir_name);

int cd(char *dir_name);

int deleteFile(char *file_name);

int example();

void waitCardInserted();

#endif // EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_SDHC_H_
