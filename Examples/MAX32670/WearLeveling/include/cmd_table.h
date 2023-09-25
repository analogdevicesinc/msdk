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

#ifndef EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_
#define EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_

#define CMD_TABLE { { "stop", "stop", "Ends the example", handle_stop }, \
	                { "read", "read <filename> <number of bytes> <location>", "Reads data from a specific location within a file.", handle_read }, \
	                { "write", "write (--create) <filename> <character string> <location>", "Writes a character string to a specific location within a file.\n    If the create flag is included and the file does not exist, the file will\n    be created.", handle_write }, \
	                { "swl", "swl <number of writes>", "Stands for \"show wear leveling.\" This command writes to a file\n    the specified number of times. Once all writes have completed, the number\n    of times each flash page (filesystem block) was written to is printed to\n    the terminal. (Writes should be distributed somewhat evenly across many\n    filesystem blocks.) This command may take a while to complete. LED0 is\n    used as a heartbeat while the command is executing.", handle_swl } };



#endif /* EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_ */
