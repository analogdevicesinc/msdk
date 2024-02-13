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

#ifndef EXAMPLES_MAX32680_WEARLEVELING_INCLUDE_CLI_H_
#define EXAMPLES_MAX32680_WEARLEVELING_INCLUDE_CLI_H_

#include "lfs.h"

/*
 * @brief Function to receive next command from the command line.
 *
 * @param cmd 	Buffer to store command into.
 * @param size 	Size of the command buffer.
 *
 * @return The size of the command if successful, otherwise an error code.
 */
int cmd_get(char *cmd, size_t size);

/*
 * @brief Function to process command and call appropriate command handler.
 *
 * @param lfs 	Pointer to mounted filesystem instance
 * @param cmd 	Buffer containing characters read from the command line.
 * @param size 	Number of characters in the command buffer.
 *
 * @return E_NO_ERROR if command processed successfully, otherwise an error code.
 */
int cmd_process(lfs_t *lfs, char *cmd, size_t size);

#endif // EXAMPLES_MAX32680_WEARLEVELING_INCLUDE_CLI_H_
