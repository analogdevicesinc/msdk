/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
#include "CM4_Utils.h"

/*
    This algorithm comes from Jack W. Crenshaw's 1998 article in Embedded:
    http://www.embedded.com/electronics-blogs/programmer-s-toolbox/4219659/Integer-Square-Roots

    And book 'Math Toolkit for Realtime Programming'
        Jack Crenshaw (2000) Math Toolkit for Real-Time Programming. Lawrence, Kan: CRC Press.
*/
uint16_t sqrt32(uint32_t a)
{
    uint32_t rem = 0, root = 0;

    for (int i = 32 / 2; i > 0; i--) {
        root <<= 1;
        rem = (rem << 2) | (a >> (32 - 2));
        a <<= 2;
        if (root < rem) {
            rem -= root | 1;
            root += 2;
        }
    }
    return root >> 1;
}
