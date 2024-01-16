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
 
    .syntax unified
    .arch armv7-m
    .text   
    .thumb
    .thumb_func
    .align 2


    .globl   Boot_Lower
    .type    Boot_Lower %function
/* Boot from the lower flash array */
Boot_Lower:

    ldr     r0,=0x10004000      /* Address for main flash image */
    ldr     r1,=0xE000ED08      /* Address for SCB_VTOR_REG */

    /* First 32-bit word in image is initial stack pointer */
    /* Second 32-bit word in image is Address of the Reset Handler */
    
    /* Set VTOR with vector table base address */
    str     r0,[r1]
    /* Load initial SP */
    ldr     sp,[r0]
    /* Load LR with address of the Reset Handler */
    ldr     lr,[r0,#4]
    /* Jump to Reset Handler. */
    bx      lr
