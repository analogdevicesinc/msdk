/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#ifndef EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_
#define EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_

#define CMD_TABLE                                                                                 \
    { { "stop", "stop", "Ends the example", handle_stop },                                        \
      { "read", "read <filename> <number of bytes> <location>",                                   \
        "Reads data from a specific location within a file.", handle_read },                      \
      { "write", "write <filename> <character string> <location>",                                \
        "Writes a character string to a specific location within a file.\n    If the create flag" \
        " is included and the file does not exist, the file will\n    be created.",               \
        handle_write },                                                                           \
      { "swl", "swl <number of writes>",                                                          \
        "Stands for \"show wear leveling.\" This command writes to a file\n    the specified "    \
        "number of times. Once all writes have completed, the number\n    of times each flash "   \
        "page (filesystem block) was written to is printed to\n    the terminal. (Writes should " \
        "be distributed somewhat evenly across many\n    filesystem blocks.) This command may "   \
        "take a while to complete. LED0 is\n    used as a heartbeat while the command is "        \
        "executing.",                                                                             \
        handle_swl } };

#define FILENAME_POS 1
#define NUM_WRITES_POS 1
#define NUM_BYTES_POS 2
#define DATA_POS 2
#define LOCATION_POS 3

#endif /* EXAMPLES_MAX32670_WEARLEVELING_INCLUDE_CMD_TABLE_H_ */
