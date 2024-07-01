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

.section .nonsecure_flash

/* Include the non-secure binary file, include global symbols we can use at link time */
_binary_nonsecure_bin_start:
.incbin "build/build_ns/nonsecure.bin"
_binary_nonsecure_bin_end:

.globl _binary_nonsecure_bin_start
.globl _binary_nonsecure_bin_end
