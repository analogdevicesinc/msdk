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
#ifndef EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_USER_CLI_H_
#define EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_USER_CLI_H_

/* -------------------------------------------------- */
//                GLOBAL VARIABLE
/* -------------------------------------------------- */
extern const command_t user_commands[];
extern const unsigned int num_user_commands;

/* -------------------------------------------------- */
//             FUNCTION PROTOTYPES
/* -------------------------------------------------- */
int handle_size(int argc, char *argv[]);

int handle_format(int argc, char *argv[]);

int handle_mount(int argc, char *argv[]);

int handle_ls(int argc, char *argv[]);

int handle_mkdir(int argc, char *argv[]);

int handle_createfile(int argc, char *argv[]);

int handle_cd(int argc, char *argv[]);

int handle_add_data(int argc, char *argv[]);

int handle_del(int argc, char *argv[]);

int handle_fatfs(int argc, char *argv[]);

int handle_unmount(int argc, char *argv[]);

#endif // EXAMPLES_MAX32655_SDHC_FTHR_INCLUDE_USER_CLI_H_
