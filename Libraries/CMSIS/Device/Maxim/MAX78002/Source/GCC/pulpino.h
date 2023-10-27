// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

/**
 * @file
 * @brief Register mapping for PULPino peripherals.
 *
 * Contains event register mappings for the PULPino SOC as
 * well as some general definitions for the overall system.
 *
 * @author Florian Zaruba
 *
 * @version 1.0
 *
 * @date 2/10/2015
 *
 */

/******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
 ******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
#define SOC_PERIPHERALS_BASE_ADDR (PULPINO_BASE_ADDR + 0xA100000)

#define MBU_PERIPHERALS_BASE_ADDR (PULPINO_BASE_ADDR + 0xA100000)

#ifdef MXC_CHIP

#include "chip.h"

#else // #ifdef MXC_CHIP

#define UART_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x0000)

#endif // #ifdef MXC_CHIP

#define GPIO_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x1000)
#define SPI_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x2000)
#define TIMER_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x3000)
#define EVENT_UNIT_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x4000)
#define I2C_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x5000)
#define FLL_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x6000)
#define SOC_CTRL_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x7000)

/** STDOUT */
#define STDOUT_BASE_ADDR (SOC_PERIPHERALS_BASE_ADDR + 0x10000)
#define FPUTCHAR_BASE_ADDR (STDOUT_BASE_ADDR + 0x1000)
#define FILE_CMD_BASE_ADDR (STDOUT_BASE_ADDR + 0x2000)
#define STREAM_BASE_ADDR (STDOUT_BASE_ADDR + 0x3000)

/** Instruction RAM */
#define INSTR_RAM_BASE_ADDR (0x00)
#define INSTR_RAM_START_ADDR (0x80)

/** ROM */
#define ROM_BASE_ADDR (0x8000)

/** Data RAM */
#define DATA_RAM_BASE_ADDR (0x00100000)

/** Registers and pointers */
#define REGP(x) ((volatile unsigned int *)(x))
#define REG(x) (*((volatile unsigned int *)(x)))
#define REGP_8(x) (((volatile uint8_t *)(x)))

/* pointer to mem of apb pulpino unit - PointerSocCtrl */
#define __PSC__(a) *(unsigned volatile int *)(SOC_CTRL_BASE_ADDR + a)

/** Peripheral Clock gating */
#define CGREG __PSC__(0x04)

/** Clock gate SPI */
#define CGSPI 0x00
/** Clock gate UART */
#define CGUART 0x01
/** Clock gate GPIO */
#define CGGPIO 0x02
/** Clock gate SPI Master */
#define CGGSPIM 0x03
/** Clock gate Timer */
#define CGTIM 0x04
/** Clock gate Event Unit */
#define CGEVENT 0x05
/** Clock gate I2C */
#define CGGI2C 0x06
/** Clock gate FLL */
#define CGFLL 0x07

/** Boot address register */
#define BOOTREG __PSC__(0x08)

#define RES_STATUS __PSC__(0x14)

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_SOURCE_GCC_PULPINO_H_
