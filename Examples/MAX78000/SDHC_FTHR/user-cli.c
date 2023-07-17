/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/


#include "user-cli.h"


const command_table_t commands[] = {{"Size", handle_size, "Find the Size of the SD Card and Free Space\n\r"},
											{"Format", handle_format, "Format the Card\n\r"},
                                            {"Mount", hande_mount, "Manually Mount Card\n\r"},
											{"ls", handle_ls, "list the contents of the current directory\n\r"},
											{"mkdir", handle_mkdir, "Create a directory\n\r"},
                                            {"file_create", handle_createfile, "Create a file of random data\n\r"},
                                            {"cd", handle_cd, "Move into a directory\n\r"},
                                            {"add_data", handle_add_data, "Add random Data to an Existing File\n\r"},
                                            {"Del", handle_del, "Delete a file\n\r"},
                                            {"FatFs", handle_fatfs, "Format Card and Run Example of FatFS Operations\n\r"},
                                            {"Unmount", handle_unmount, "Unmount card\n\r"},
											{"Help", handle_help, "Prints a help message with info about all of the supported commands.\n\r"}};

const int num_commands = sizeof(commands) / sizeof(command_table_t);
/*
 * @name handle_size
 *
 * @brief Finds the Size of the SD Card and Free Space
 *
 * @param argc and *argv[]
 *
 *
 * @return
 * 		   void
 */
void handle_size(int argc, char *argv[]){
    getSize();
}

/*
 * @name handle_format
 *
 * @brief han
 *
 * @param argc and *argv[]
 *
 *
 * @return
 * 		   void
 */
void handle_format(int argc, char *argv[]){
    formatSDHC();
}

void hande_mount(int argc, char *argv[]){
    mount();
}

void handle_ls(int argc, char *argv[]){
    ls();
}

void handle_mkdir(int argc, char *argv[]){
    mkdir(argv[1]);
}

void handle_createfile(int argc, char *argv[]){
    unsigned int length = atoi(argv[2]);
    createFile(argv[1],length);
}

void handle_cd(int argc, char *argv[]){
    cd(argv[1]);
}

void handle_add_data(int argc, char *argv[]){
    unsigned int length = atoi(argv[2]);
    appendFile(argv[1],length);
}

void handle_del(int argc, char *argv[]){
    delete(argv[1]);
}

void handle_fatfs(int argc, char *argv[]){
    example();
}

void handle_unmount(int argc, char *argv[]){
    umount();

}
/* @name handle_help
 *
 * @brief: Prints a help message with info about all of the supported commands.
 *
 */
void handle_help(int argc, char *argv[])
{
	printf("\n\r");
	for (int i = 0; i < num_commands;i++)
		printf("%s --> %s", commands[i].name, commands[i].help_string);
}