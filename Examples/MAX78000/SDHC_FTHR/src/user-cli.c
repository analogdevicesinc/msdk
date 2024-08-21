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

#include <stdlib.h>
#include "cli.h"
#include "sdhc.h"
#include "user-cli.h"

const command_t user_commands[] = {
    { "size", "size", "Find the Size of the SD Card and Free Space", handle_size },
    { "format", "format", "Format the Card", handle_format },
    { "mount", "mount", "Manually Mount Card", handle_mount },
    { "ls", "ls", "list the contents of the current directory", handle_ls },
    { "mkdir", "mkdir <directory name>", "Create a directory", handle_mkdir },
    { "file_create", "file_create <file name> <number of bytes to add>",
      "Create a file of random data", handle_createfile },
    { "cd", "cd <directory name>", "Move into a directory", handle_cd },
    { "add_data", "add_data <file name> <number of bytes to add>",
      "Add random Data to an Existing File", handle_add_data },
    { "del", "del <file name>", "Delete a file", handle_del },
    { "fatfs", "fatfs", "Format Card and Run Example of FatFS Operations", handle_fatfs },
    { "unmount", "unmount", "Unmount card", handle_unmount },
};

const unsigned int num_user_commands = sizeof(user_commands) / sizeof(command_t);

int handle_size(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return getSize();
}

int handle_format(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return formatSDHC();
}

int handle_mount(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return mount();
}

int handle_ls(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return ls();
}

int handle_mkdir(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Incorrect usage. Please provide directory name.\n");
        return E_INVALID;
    }

    return mkdir(argv[1]);
}

int handle_createfile(int argc, char *argv[])
{
    if (argc != 3) {
        printf("Incorrect usage. Please provide filename and length.\n");
        return E_INVALID;
    }

    unsigned int length = atoi(argv[2]);
    return createFile(argv[1], length);
}

int handle_cd(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Incorrect usage. Please provide directory name.\n");
        return E_INVALID;
    }

    return cd(argv[1]);
}

int handle_add_data(int argc, char *argv[])
{
    if (argc != 3) {
        printf("Incorrect usage. Please provide filename and length.\n");
        return E_INVALID;
    }

    unsigned int length = atoi(argv[2]);
    return appendFile(argv[1], length);
}

int handle_del(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Incorrect usage. Please provide filename.\n");
        return E_INVALID;
    }

    return deleteFile(argv[1]);
}

int handle_fatfs(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return example();
}

int handle_unmount(int argc, char *argv[])
{
    if (argc != 1) {
        printf("Incorrect usage. Too many parameters.\n");
        return E_INVALID;
    }

    return umount();
}
