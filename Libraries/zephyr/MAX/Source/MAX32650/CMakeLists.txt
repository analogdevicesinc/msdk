##############################################################################
#
# Copyright (C) 2025 Analog Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
##############################################################################


if(NOT TARGET_REV)
  # Default version A1, not actively use in driver but requires to be defined.
  zephyr_compile_definitions(-DTARGET_REV=0x4131)
endif()

zephyr_include_directories(
    ${MSDK_PERIPH_SRC_DIR}/SYS
    ${MSDK_PERIPH_SRC_DIR}/ADC
    ${MSDK_PERIPH_SRC_DIR}/DMA
    ${MSDK_PERIPH_SRC_DIR}/FLC
    ${MSDK_PERIPH_SRC_DIR}/GPIO
    ${MSDK_PERIPH_SRC_DIR}/I2C
    ${MSDK_PERIPH_SRC_DIR}/SPIMSS
    ${MSDK_PERIPH_SRC_DIR}/ICC
    ${MSDK_PERIPH_SRC_DIR}/LP
    ${MSDK_PERIPH_SRC_DIR}/OWM 
    ${MSDK_PERIPH_SRC_DIR}/PT
    ${MSDK_PERIPH_SRC_DIR}/RTC
    ${MSDK_PERIPH_SRC_DIR}/SEMA
    ${MSDK_PERIPH_SRC_DIR}/SPI
    ${MSDK_PERIPH_SRC_DIR}/TRNG
    ${MSDK_PERIPH_SRC_DIR}/TMR
    ${MSDK_PERIPH_SRC_DIR}/UART
    ${MSDK_PERIPH_SRC_DIR}/WDT
)

if(CONFIG_ARM)
  zephyr_library_sources(${MSDK_CMSIS_DIR}/Source/system_max32650.c)
  zephyr_library_sources(${MSDK_PERIPH_SRC_DIR}/SYS/mxc_lock.c)
elseif(CONFIG_RISCV)
  zephyr_library_sources(${MSDK_CMSIS_DIR}/Source/system_riscv_max32650.c)
endif()

zephyr_library_sources(
    ./max32xxx_system.c

    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_assert.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_delay.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/pins_me10.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/sys_me10.c
     
    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_me10.c
    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_reva.c
    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_common.c

    ${MSDK_PERIPH_SRC_DIR}/LP/lp_me10.c
    
    ${MSDK_PERIPH_SRC_DIR}/PT/pt_me10.c
    ${MSDK_PERIPH_SRC_DIR}/PT/pt_reva.c
        
    ${MSDK_PERIPH_SRC_DIR}/SEMA/sema_me10.c
    ${MSDK_PERIPH_SRC_DIR}/SEMA/sema_reva.c

    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_me10.c
    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_reva.c
)

if (CONFIG_UART_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_common.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_me10.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_reva.c
)
endif()

if (CONFIG_GPIO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_common.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_me10.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_reva.c
)
endif()

if (CONFIG_SPI_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_me10.c
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_reva1.c
)
endif()

if (CONFIG_I2C_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/I2C/i2c_me10.c
    ${MSDK_PERIPH_SRC_DIR}/I2C/i2c_reva.c
)
endif()

if (CONFIG_WDT_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_common.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_me10.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_reva.c
)
endif()

if (CONFIG_RTC_MAX32 OR CONFIG_COUNTER_RTC_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_me10.c
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_reva.c
)
endif()

if (CONFIG_SOC_FLASH_MAX32 OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_common.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_me10.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_reva.c
)
endif()

if (CONFIG_ADC_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/ADC/adc_me10.c
    ${MSDK_PERIPH_SRC_DIR}/ADC/adc_reva.c
)
endif()

if (CONFIG_W1_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/OWM/owm_me10.c
    ${MSDK_PERIPH_SRC_DIR}/OWM/owm_reva.c
)
endif()

if (CONFIG_PWM_MAX32 OR CONFIG_TIMER_MAX32 OR CONFIG_COUNTER_TIMER_MAX32) 
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_common.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_me10.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_reva.c
)
endif()

if (CONFIG_I2S_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/SPIMSS/i2s_me10.c
    ${MSDK_PERIPH_SRC_DIR}/SPIMSS/i2s_reva.c
)
endif()

if (CONFIG_ENTROPY_MAX32_TRNG)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_me10.c
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_reva.c
)
endif()

