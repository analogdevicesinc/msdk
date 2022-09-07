/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef _RAM_INSTANCE_ADDRS_H_
#define _RAM_INSTANCE_ADDRS_H_

/* ================================================================================ */
/* ========================       System RAM Section       ======================== */
/* ================================================================================ */

#define MXC_SRAM0_MEM_BASE 0x20000000UL
#define MXC_SRAM0_MEM_SIZE 0x00008000UL

#define MXC_SRAM1_MEM_BASE 0x20008000UL
#define MXC_SRAM1_MEM_SIZE 0x00010000UL

#define MXC_SRAM2_MEM_BASE 0x20018000UL
#define MXC_SRAM2_MEM_SIZE 0x00008000UL

#define MXC_SRAM3_MEM_BASE 0x20020000UL
#define MXC_SRAM3_MEM_SIZE 0x00020000UL

#define MXC_SRAM4_MEM_BASE 0x20040000UL
#define MXC_SRAM4_MEM_SIZE 0x00040000UL

#define MXC_SRAM5_MEM_BASE 0x20080000UL
#define MXC_SRAM5_MEM_SIZE 0x00040000UL

#define MXC_SRAM6_MEM_BASE 0x200C0000UL
#define MXC_SRAM6_MEM_SIZE 0x00040000UL

#define MXC_SRAM0_INST0_MEM_BASE 0x20000000UL
#define MXC_SRAM0_INST0_MEM_SIZE 0x00008000UL

#define MXC_SRAM1_INST0_MEM_BASE 0x20008000UL
#define MXC_SRAM1_INST0_MEM_SIZE 0x00008000UL
#define MXC_SRAM1_INST1_MEM_BASE 0x20010000UL
#define MXC_SRAM1_INST1_MEM_SIZE 0x00008000UL

#define MXC_SRAM2_INST0_MEM_BASE 0x20018000UL
#define MXC_SRAM2_INST0_MEM_SIZE 0x00008000UL

#define MXC_SRAM3_INST0_MEM_BASE 0x20020000UL
#define MXC_SRAM3_INST0_MEM_SIZE 0x00010000UL
#define MXC_SRAM3_INST1_MEM_BASE 0x20030000UL
#define MXC_SRAM3_INST1_MEM_SIZE 0x00010000UL

#define MXC_SRAM4_INST0_MEM_BASE 0x20040000UL
#define MXC_SRAM4_INST0_MEM_SIZE 0x00010000UL
#define MXC_SRAM4_INST1_MEM_BASE 0x20050000UL
#define MXC_SRAM4_INST1_MEM_SIZE 0x00010000UL
#define MXC_SRAM4_INST2_MEM_BASE 0x20060000UL
#define MXC_SRAM4_INST2_MEM_SIZE 0x00010000UL
#define MXC_SRAM4_INST3_MEM_BASE 0x20070000UL
#define MXC_SRAM4_INST3_MEM_SIZE 0x00010000UL

#define MXC_SRAM5_INST0_MEM_BASE 0x20080000UL
#define MXC_SRAM5_INST0_MEM_SIZE 0x00010000UL
#define MXC_SRAM5_INST1_MEM_BASE 0x20090000UL
#define MXC_SRAM5_INST1_MEM_SIZE 0x00010000UL
#define MXC_SRAM5_INST2_MEM_BASE 0x200A0000UL
#define MXC_SRAM5_INST2_MEM_SIZE 0x00010000UL
#define MXC_SRAM5_INST3_MEM_BASE 0x200B0000UL
#define MXC_SRAM5_INST3_MEM_SIZE 0x00010000UL

#define MXC_SRAM6_INST0_MEM_BASE 0x200C0000UL
#define MXC_SRAM6_INST0_MEM_SIZE 0x00010000UL
#define MXC_SRAM6_INST1_MEM_BASE 0x200D0000UL
#define MXC_SRAM6_INST1_MEM_SIZE 0x00010000UL
#define MXC_SRAM6_INST2_MEM_BASE 0x200E0000UL
#define MXC_SRAM6_INST2_MEM_SIZE 0x00010000UL
#define MXC_SRAM6_INST3_MEM_BASE 0x200F0000UL
#define MXC_SRAM6_INST3_MEM_SIZE 0x00010000UL

/* ================================================================================ */
/* =========================       MAA RAM Section       ========================== */
/* ================================================================================ */

#define MXC_MAA_MEM0_BASE 0x40001100UL
#define MXC_MAA_MEM0_SIZE 0x00000100UL

#define MXC_MAA_MEM1_BASE 0x40001200UL
#define MXC_MAA_MEM1_SIZE 0x00000100UL

#define MXC_MAA_MEM2_BASE 0x40001300UL
#define MXC_MAA_MEM2_SIZE 0x00000100UL

#define MXC_MAA_MEM3_BASE 0x40001400UL
#define MXC_MAA_MEM3_SIZE 0x00000100UL

#define MXC_MAA_MEM4_BASE 0x40001500UL
#define MXC_MAA_MEM4_SIZE 0x00000100UL

#define MXC_MAA_MEM5_BASE 0x40001600UL
#define MXC_MAA_MEM5_SIZE 0x00000100UL

#endif /* _RAM_INSTANCE_ADDRS_H_ */
