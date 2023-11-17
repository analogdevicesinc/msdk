/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
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
 ******************************************************************************/

/**
 * @file    riscv_rom.h
 * @brief   RISCV MSR ROM defines
 *
 * @details This file specidies RISCV ROM MSR functions and data
 */

#ifndef RISCV_ROM_H_
#define RISCV_ROM_H_

#include "main_msr.h"


/* Define RISCV starting addresses */
#define RV_MSR_DATA 	(0x20062000)
#define RV_START_SRAM 	(0x20068000)
#define RV_START_ROM 	(0x2006c000)


/* Define data used by RISC-V ROM */
#define msr_version (*(volatile uint32_t *) (RV_MSR_DATA + 0x00))
#define msr_version_lo (*(volatile uint16_t *) (RV_MSR_DATA + 0x00))
#define msr_version_hi (*(volatile uint16_t *) (RV_MSR_DATA + 0x02))
#define msr_ctrl (*(volatile uint32_t *) (RV_MSR_DATA + 0x04))
#define msr_ctrl_exit_code (*(volatile uint16_t *) (RV_MSR_DATA + 0x04))
#define msr_ctrl_err_status (*(volatile uint16_t *) (RV_MSR_DATA + 0x06))
#define msr_cfg  (*(volatile uint16_t *) (RV_MSR_DATA + 0x08))
#define msr_irq_trap  (*(volatile uint16_t *) (RV_MSR_DATA + 0x0a))
#define msr_decoded_track ((volatile msr_decoded_track_t *) (RV_MSR_DATA + 0x100))


/* Define RISCV-ROM functions */
#define pReset_Handler ((void (*)(uint32_t)) (RV_START_ROM + 0x00))
#define pReset_Handler_Call ((void (*)(uint32_t)) (RV_START_ROM + 0x04))
#define pset_default_parameters ((void (*)(void)) (RV_START_ROM + 0x08))
#define pmsr_init ((void (*)(void)) (RV_START_ROM + 0x0c))

#define pGetSwipe ((void (*)(void)) (RV_START_ROM + 0x10))
#define pProcessSwipe ((void (*)(void)) (RV_START_ROM + 0x14))
#define pmsr_enable ((void (*)(void)) (RV_START_ROM + 0x18))
#define pmsr_disable ((void (*)(void)) (RV_START_ROM + 0x1c))

#define pADC9_IRQHandler ((void (*)(void)) (RV_START_ROM + 0x20))
#define pTMR5_IRQHandler ((void (*)(void)) (RV_START_ROM + 0x24))
#define pCM4_IRQHandler ((void (*)(void)) (RV_START_ROM + 0x28))
#define pUNUSED_IRQHandler ((void (*)(void)) (RV_START_ROM + 0x2c))

#define pset_periph_clock_hz ((void (*)(uint32_t)) (RV_START_ROM + 0x40))
#define pget_periph_clock_hz ((uint32_t (*)(void)) (RV_START_ROM + 0x44))
#define pset_pclk2aclk_div ((void (*)(uint32_t)) (RV_START_ROM + 0x48))
#define pget_pclk2aclk_div ((uint32_t (*)(void)) (RV_START_ROM + 0x4c))

#define pset_aclk2smpl_div ((void (*)(uint32_t)) (RV_START_ROM + 0x50))
#define pget_aclk2smpl_div ((uint32_t (*)(void)) (RV_START_ROM + 0x54))
#define pset_pclk2tmr_div ((void (*)(uint32_t)) (RV_START_ROM + 0x58))
#define pget_pclk2tmr_div ((uint32_t (*)(void)) (RV_START_ROM + 0x5c))

#define pset_adc_frame_len ((void (*)(uint32_t)) (RV_START_ROM + 0x60))
#define pget_adc_frame_len ((uint32_t (*)(void)) (RV_START_ROM + 0x64))
#define pset_adc_frame_seq ((void (*)(uint8_t *)) (RV_START_ROM + 0x68))
#define pget_adc_frame_seq ((void (*)(uint8_t *)) (RV_START_ROM + 0x6c))

#define pset_getswipe_timout ((void (*)(uint32_t)) (RV_START_ROM + 0x78))
#define pget_getswipe_timout ((uint32_t (*)(void)) (RV_START_ROM + 0x7c))

#define pset_thr_scale ((void (*)(uint32_t, uint32_t)) (RV_START_ROM + 0x80))
#define pget_thr_scale ((void (*)(uint32_t *, uint32_t *)) (RV_START_ROM + 0x84))
#define pset_maxnoise ((void (*)(uint32_t)) (RV_START_ROM + 0x88))
#define pget_maxnoise ((uint32_t (*)(void)) (RV_START_ROM + 0x8c))

#define pset_pream_fct ((void (*)(uint32_t, uint32_t)) (RV_START_ROM + 0x90))
#define pget_pream_fct ((void (*)(uint32_t *, uint32_t *)) (RV_START_ROM + 0x94))
#define pset_adc_cm_setup ((void (*)(hw_register_setup_t *)) (RV_START_ROM + 0x98))
#define pget_adc_cm_setup ((void (*)(hw_register_setup_t *)) (RV_START_ROM + 0x9c))

#define pset_trk_timo ((void (*)(uint32_t, uint32_t, uint32_t)) (RV_START_ROM + 0xa0))
#define pget_trk_timo ((void (*)(uint32_t, uint32_t *, uint32_t *)) (RV_START_ROM + 0xa4))
#define pset_trk_tcnt ((void (*)(uint32_t, uint32_t, uint32_t)) (RV_START_ROM + 0xa8))
#define pget_trk_tcnt ((void (*)(uint32_t, uint32_t *, uint32_t *)) (RV_START_ROM + 0xac))

#define pset_trk_trh_setup ((void (*)(uint32_t, uint32_t, uint32_t)) (RV_START_ROM + 0xb0))
#define pget_trk_trh_setup ((void (*)(uint32_t, uint32_t *, uint32_t *)) (RV_START_ROM + 0xb4))
#define pset_trk_flt_setup ((void (*)(uint32_t, track_flt_setup_t *)) (RV_START_ROM + 0xb8))
#define pget_trk_flt_setup ((void (*)(uint32_t, track_flt_setup_t *)) (RV_START_ROM + 0xbc))

#define pset_trk_arrays ((void (*)(uint32_t, uint32_t, uint32_t, uint32_t)) (RV_START_ROM + 0xc0))
#define pget_trk_arrays ((void (*)(uint32_t, uint32_t *, uint32_t *, uint32_t *)) (RV_START_ROM + 0xc4))
#define pset_trk_params ((void (*)(uint32_t, parse_bits_t *)) (RV_START_ROM + 0xc8))
#define pget_trk_params ((void (*)(uint32_t, parse_bits_t *)) (RV_START_ROM + 0xcc))


/* Define RISCV-ROM vector table origin */
#define MSR_ISR_VECTOR (RV_START_ROM + 0x100)


/* Fields and Bits */

/* Swipe direction */
#define MSR_FORWARD     (0)
#define MSR_REVERSE     (1)

/* Parse bits error codes */
#define MSR_ERR_OK          (0x00)    /* No error */
#define MSR_ERR_BAD_LEN     (0x01)    /* invalid length parameter */
#define MSR_ERR_START_SEN   (0x02)    /* start sentinel was not found */
#define MSR_ERR_END_SEN     (0x04)    /* end sentinel was not found */
#define MSR_ERR_OUTLIER     (0x08)    /* invalid sample value */
#define MSR_ERR_PARAM       (0x10)    /* invalid parameter */
#define MSR_ERR_LRC         (0x40)    /* invalid LRC (LRC != 0) */
#define MSR_ERR_PARITY      (0x80)    /* parity error */

/* error codes */
#define  ADCERR_OVERRUN     (1 << 0)
#define  ADCERR_SHIFT       (1 << 1)
#define  ADCERR_INCOMPLETE  (1 << 2)
#define  HIGH_NOISE         (1 << 3)

/* Swipe termination (exit) codes */
#define  GETSWIPE_BUSY    (0x00)
#define  GETSWIPE_OK      (0x01)
#define  GETSWIPE_ERROR   (0x02)
#define  GETSWIPE_TIMO    (0x03)

/* MSR_CFG */
#define  MSR_CFG_CM4IRQ_ENABLE     (1 << 0)


#endif /* RISCV_ROM_H_ */
