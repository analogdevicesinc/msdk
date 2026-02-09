/******************************************************************************
 *
 * Copyright (C) 2023-2026 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SYS_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SYS_H_

/***** Includes *****/
#include <mxc_sys.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  Apply preinit configuration for the target
 */
void max32xx_system_init(void);

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || defined(CONFIG_SOC_MAX32650)

#define ADI_MAX32_CLK_IPO MXC_SYS_CLOCK_HIRC96
#define ADI_MAX32_CLK_IBRO MXC_SYS_CLOCK_HIRC8
#if !defined(CONFIG_SOC_MAX32650)
#define ADI_MAX32_CLK_ERFO MXC_SYS_CLOCK_XTAL32M
#define ADI_MAX32_CLK_ISO MXC_SYS_CLOCK_HIRC
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_LIRC8K
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_XTAL32K
#else
#define ADI_MAX32_CLK_ISO MXC_SYS_CLOCK_CRYPTO
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_NANORING
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_X32K

#define MXC_SYS_RESET0_USB MXC_SYS_RESET_USB
#endif

#define z_sysclk_prescaler(v) MXC_SYS_SYSTEM_DIV_##v
#define sysclk_prescaler(v) z_sysclk_prescaler(v)

static inline void Wrap_MXC_SYS_SetClockDiv(int div)
{
    MXC_SYS_Clock_Div((mxc_sys_system_div_t)div);
}

static inline int Wrap_MXC_SYS_Select32KClockSource(int clock)
{
    return 0;
}

static inline int Wrap_MXC_SYS_GetUSN(uint8_t *usn)
{
#if defined(CONFIG_SOC_MAX32650)
    return MXC_SYS_GetUSN(usn, MXC_SYS_USN_LEN);
#else
    return MXC_SYS_GetUSN(usn, NULL);
#endif
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX32655) || \
    defined(CONFIG_SOC_MAX32670) || defined(CONFIG_SOC_MAX32672) ||   \
    defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32675) ||   \
    defined(CONFIG_SOC_MAX32680) || defined(CONFIG_SOC_MAX32657) ||   \
    defined(CONFIG_SOC_MAX78002) || defined(CONFIG_SOC_MAX78000) || defined(CONFIG_SOC_MAX32660)

#if !defined(CONFIG_SOC_MAX32660)
#define ADI_MAX32_CLK_IPO MXC_SYS_CLOCK_IPO
#if defined(CONFIG_SOC_MAX78002)
#define ADI_MAX32_CLK_IPLL MXC_SYS_CLOCK_IPLL
#define ADI_MAX32_CLK_EBO MXC_SYS_CLOCK_EBO
#elif !defined(CONFIG_SOC_MAX78000)
#define ADI_MAX32_CLK_ERFO MXC_SYS_CLOCK_ERFO
#endif
#define ADI_MAX32_CLK_IBRO MXC_SYS_CLOCK_IBRO
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_INRO
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_ERTCO
#define ADI_MAX32_CLK_EXTCLK MXC_SYS_CLOCK_EXTCLK
#if !(defined(CONFIG_SOC_MAX32670) || defined(CONFIG_SOC_MAX32672) || \
      defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32675))
#define ADI_MAX32_CLK_ISO MXC_SYS_CLOCK_ISO
#endif
#else
#define ADI_MAX32_CLK_IPO MXC_SYS_CLOCK_HIRC
#define ADI_MAX32_CLK_INRO MXC_SYS_CLOCK_NANORING
#define ADI_MAX32_CLK_ERTCO MXC_SYS_CLOCK_HFXIN
#endif

#define z_sysclk_prescaler(v) MXC_SYS_CLOCK_DIV_##v
#define sysclk_prescaler(v) z_sysclk_prescaler(v)

static inline void Wrap_MXC_SYS_SetClockDiv(int div)
{
    MXC_SYS_SetClockDiv((mxc_sys_system_clock_div_t)div);
}

#if defined(CONFIG_SOC_MAX32657)
static inline mxc_sys_32k_clock_t wrap_get_32k_clock_source_instance(uint8_t clock_source)
{
    mxc_sys_32k_clock_t clk_src;

    switch (clock_source) {
    case 4: // ADI_MAX32_PRPH_CLK_SRC_ERTCO
        clk_src = MXC_SYS_32K_CLOCK_ERTCO;
        break;
    case 5: // ADI_MAX32_PRPH_CLK_SRC_INRO
        clk_src = MXC_SYS_32K_CLOCK_INRO;
        break;
    default:
        return -1;
    }

    return clk_src;
}

static inline int Wrap_MXC_SYS_Select32KClockSource(int clock)
{
    return MXC_SYS_Select32KClockSource(wrap_get_32k_clock_source_instance(clock));
}
#else
static inline int Wrap_MXC_SYS_Select32KClockSource(int clock)
{
    return 0;
}
#endif

static inline int Wrap_MXC_SYS_GetUSN(uint8_t *usn)
{
#if defined(CONFIG_SOC_MAX32660)
    return MXC_SYS_GetUSN(usn, MXC_SYS_USN_LEN, 0);
#else
    return MXC_SYS_GetUSN(usn, NULL);
#endif
}

#endif // part number

static inline int Wrap_MXC_SYS_ClockCalibrate(mxc_sys_system_clock_t clock)
{
#if defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32690)
    return MXC_SYS_ClockCalibrate(clock);
#else
    return E_NOT_SUPPORTED;
#endif
}

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SYS_H_
