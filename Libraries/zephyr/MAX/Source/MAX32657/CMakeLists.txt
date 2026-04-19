##############################################################################
#
# Copyright (C) 2024 Analog Devices, Inc.
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

math(EXPR __MXC_FLASH_MEM_SIZE "${CONFIG_FLASH_SIZE} * 1024")
math(EXPR __MXC_SRAM_MEM_SIZE "${CONFIG_SRAM_SIZE} * 1024")

zephyr_compile_definitions(
    -D__MXC_FLASH_MEM_BASE=${CONFIG_FLASH_BASE_ADDRESS}
    -D__MXC_FLASH_MEM_SIZE=${__MXC_FLASH_MEM_SIZE}
    -D__MXC_SRAM_MEM_BASE=${CONFIG_SRAM_BASE_ADDRESS}
    -D__MXC_SRAM_MEM_SIZE=${__MXC_SRAM_MEM_SIZE}
)

zephyr_include_directories(
    ${MSDK_PERIPH_SRC_DIR}/SYS
    ${MSDK_PERIPH_SRC_DIR}/AES
    ${MSDK_PERIPH_SRC_DIR}/CRC
    ${MSDK_PERIPH_SRC_DIR}/DMA
    ${MSDK_PERIPH_SRC_DIR}/FLC
    ${MSDK_PERIPH_SRC_DIR}/GPIO
    ${MSDK_PERIPH_SRC_DIR}/I3C
    ${MSDK_PERIPH_SRC_DIR}/ICC
    ${MSDK_PERIPH_SRC_DIR}/LP
    ${MSDK_PERIPH_SRC_DIR}/RTC
    ${MSDK_PERIPH_SRC_DIR}/SPI
    ${MSDK_PERIPH_SRC_DIR}/TRNG
    ${MSDK_PERIPH_SRC_DIR}/TMR
    ${MSDK_PERIPH_SRC_DIR}/UART
    ${MSDK_PERIPH_SRC_DIR}/WDT
    ${MSDK_PERIPH_SRC_DIR}/WUT
)

zephyr_library_sources(
    ./max32xxx_system.c

    ${MSDK_CMSIS_DIR}/Source/system_max32657.c

    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_assert.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_delay.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_lock.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/pins_me30.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/sys_me30.c

    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_me30.c
    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_reva.c

    ${MSDK_PERIPH_SRC_DIR}/LP/lp_me30.c

    ${MSDK_PERIPH_SRC_DIR}/WUT/wut_me30.c
    ${MSDK_PERIPH_SRC_DIR}/WUT/wut_reva.c

    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_me30.c
    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_reva.c
)

if (CONFIG_UART_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_common.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_me30.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_revb.c
)
endif()

if (CONFIG_GPIO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_common.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_me30.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_reva.c
)
endif()

if (CONFIG_SPI_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_me30.c
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_reva1.c
)
endif()

if (CONFIG_I3C_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/I3C/i3c_me30.c
    ${MSDK_PERIPH_SRC_DIR}/I3C/i3c_reva.c
)
endif()

if (CONFIG_WDT_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_common.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_me30.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_revb.c
)
endif()

if (CONFIG_RTC_MAX32 OR CONFIG_COUNTER_RTC_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_me30.c
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_reva.c
)
endif()

if (CONFIG_SOC_FLASH_MAX32 OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_common.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_me30.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_reva.c
)
endif()

if (CONFIG_PWM_MAX32 OR CONFIG_TIMER_MAX32 OR CONFIG_COUNTER_TIMER_MAX32) 
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_common.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_me30.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_revb.c
)
endif()

if (CONFIG_ENTROPY_MAX32_TRNG)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_me30.c
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_revb.c
)
endif()

if (CONFIG_CRYPTO_MAX32 OR CONFIG_BT OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/AES/aes_me30.c
    ${MSDK_PERIPH_SRC_DIR}/AES/aes_revb.c
)
endif()

if (CONFIG_CRYPTO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/CRC/crc_me30.c
    ${MSDK_PERIPH_SRC_DIR}/CRC/crc_reva.c
)
endif()
