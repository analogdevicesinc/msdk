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

#ifndef EXAMPLES_MAX32665_WEARLEVELING_INCLUDE_CLI_H_
#define EXAMPLES_MAX32665_WEARLEVELING_INCLUDE_CLI_H_

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

#endif // EXAMPLES_MAX32665_WEARLEVELING_INCLUDE_CLI_H_
