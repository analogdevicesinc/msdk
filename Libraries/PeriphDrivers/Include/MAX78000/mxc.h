/*
 *
 ******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 ******************************************************************************
 */

#ifndef __MAX32xxx_H_
#define __MAX32xxx_H_

//
#include "mxc_device.h"

//
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "nvic_table.h"

//
#include "board.h"
#include "led.h"
#include "pb.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#endif
#include "tsc2046.h"

/*
 *  Peripheral Driver Includes
 */
#include "adc.h"
#include "aes.h"
#include "cameraif.h"
#include "crc.h"
#include "dma.h"
#include "flc.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "icc.h"
#include "lp.h"
#include "owm.h"
#include "pt.h"
#include "rtc.h"
#include "sema.h"
#include "spi.h"
#include "tmr.h"
#include "trng.h"
#include "uart.h"
#include "wdt.h"
#include "wut.h"

#endif /* __MAX32xxx_H_ */
