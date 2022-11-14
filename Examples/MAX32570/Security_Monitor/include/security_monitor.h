/*
 * security_monitor.h
 *
 ******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef EXAMPLES_MAX32570_SECURITY_MONITOR_INCLUDE_SECURITY_MONITOR_H_
#define EXAMPLES_MAX32570_SECURITY_MONITOR_INCLUDE_SECURITY_MONITOR_H_

/* AES */
#define AES_BASE 0x40005000

int smon_init(void);
int smon_check_aes_key(void);
int smon_rtc_status(void);
int smon_start_rtc(void);
int smon_clear_tamper_time(void);
int smon_check_tamper(void);
int smon_check_tamper_time(void);
int smon_clear_tamper(void);
int smon_load_aes_key(void);
int smon_set_int_sensors(void);
int smon_set_ext_sensors(void);
int smon_secalm_keywipe(void);
int smon_create_DRS(void);

#endif // EXAMPLES_MAX32570_SECURITY_MONITOR_INCLUDE_SECURITY_MONITOR_H_

/* EOF */
