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
    ${MSDK_PERIPH_SRC_DIR}/ADC
    ${MSDK_PERIPH_SRC_DIR}/AES
    ${MSDK_PERIPH_SRC_DIR}/CAMERAIF
    ${MSDK_PERIPH_SRC_DIR}/CRC
    ${MSDK_PERIPH_SRC_DIR}/DMA
    ${MSDK_PERIPH_SRC_DIR}/FLC
    ${MSDK_PERIPH_SRC_DIR}/GPIO
    ${MSDK_PERIPH_SRC_DIR}/I2C
    ${MSDK_PERIPH_SRC_DIR}/I2S
    ${MSDK_PERIPH_SRC_DIR}/ICC
    ${MSDK_PERIPH_SRC_DIR}/LP
    ${MSDK_PERIPH_SRC_DIR}/LPCMP
    ${MSDK_PERIPH_SRC_DIR}/OWM
    ${MSDK_PERIPH_SRC_DIR}/PT
    ${MSDK_PERIPH_SRC_DIR}/RTC
    ${MSDK_PERIPH_SRC_DIR}/SEMA
    ${MSDK_PERIPH_SRC_DIR}/SPI
    ${MSDK_PERIPH_SRC_DIR}/SYS
    ${MSDK_PERIPH_SRC_DIR}/TMR
    ${MSDK_PERIPH_SRC_DIR}/TRNG
    ${MSDK_PERIPH_SRC_DIR}/UART
    ${MSDK_PERIPH_SRC_DIR}/WDT
    ${MSDK_PERIPH_SRC_DIR}/WUT
)

if(CONFIG_ARM)
  zephyr_library_sources(${MSDK_CMSIS_DIR}/Source/system_max78000.c)
  zephyr_library_sources(${MSDK_PERIPH_SRC_DIR}/SYS/mxc_lock.c)
elseif(CONFIG_RISCV)
  zephyr_library_sources(${MSDK_CMSIS_DIR}/Source/system_riscv_max78000.c)
endif()

zephyr_library_sources(
    ./max78xxx_system.c

    ${MSDK_PERIPH_SRC_DIR}/CAMERAIF/cameraif_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/CAMERAIF/cameraif_reva.c

    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_me17.c
    ${MSDK_PERIPH_SRC_DIR}/ICC/icc_reva.c

    ${MSDK_PERIPH_SRC_DIR}/LP/lp_ai85.c

    ${MSDK_PERIPH_SRC_DIR}/LPCMP/lpcmp_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/LPCMP/lpcmp_reva.c

    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_assert.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/mxc_delay.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/pins_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/SYS/sys_ai85.c

    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_me17.c
    ${MSDK_PERIPH_SRC_DIR}/DMA/dma_reva.c

    ${MSDK_PERIPH_SRC_DIR}/SEMA/sema_me17.c
    ${MSDK_PERIPH_SRC_DIR}/SEMA/sema_reva.c

    ${MSDK_PERIPH_SRC_DIR}/PT/pt_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/PT/pt_reva.c

    ${MSDK_PERIPH_SRC_DIR}/WUT/wut_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/WUT/wut_reva.c
)

if (CONFIG_ADC_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/ADC/adc_me17.c
    ${MSDK_PERIPH_SRC_DIR}/ADC/adc_reva.c
)
endif()

if (CONFIG_UART_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_common.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/UART/uart_revb.c
)
endif()

if (CONFIG_GPIO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_common.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/GPIO/gpio_reva.c
)
endif()

if (CONFIG_SPI_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_me17.c
    ${MSDK_PERIPH_SRC_DIR}/SPI/spi_reva1.c
)
endif()

if (CONFIG_I2C_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/I2C/i2c_me17.c
    ${MSDK_PERIPH_SRC_DIR}/I2C/i2c_reva.c
)
endif()

if (CONFIG_I2S_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/I2S/i2s_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/I2S/i2s_reva.c
)
endif()

if (CONFIG_WDT_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_common.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_me17.c
    ${MSDK_PERIPH_SRC_DIR}/WDT/wdt_revb.c
)
endif()

if (CONFIG_RTC_MAX32 OR CONFIG_COUNTER_RTC_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_me17.c
    ${MSDK_PERIPH_SRC_DIR}/RTC/rtc_reva.c
)
endif()

if (CONFIG_SOC_FLASH_MAX32 OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_common.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_me17.c
    ${MSDK_PERIPH_SRC_DIR}/FLC/flc_reva.c
)
endif()

if (CONFIG_PWM_MAX32 OR CONFIG_TIMER_MAX32 OR CONFIG_COUNTER_TIMER_MAX32) 
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_common.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/TMR/tmr_revb.c
)
endif()

if (CONFIG_ENTROPY_MAX32_TRNG OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_ai85.c
    ${MSDK_PERIPH_SRC_DIR}/TRNG/trng_revb.c
)
endif()

if (CONFIG_CRYPTO_MAX32 OR CONFIG_HWINFO_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/AES/aes_me17.c
    ${MSDK_PERIPH_SRC_DIR}/AES/aes_revb.c

    ${MSDK_PERIPH_SRC_DIR}/CRC/crc_me17.c
    ${MSDK_PERIPH_SRC_DIR}/CRC/crc_reva.c
)
endif()

if (CONFIG_W1_MAX32)
zephyr_library_sources(
    ${MSDK_PERIPH_SRC_DIR}/OWM/owm_me17.c
    ${MSDK_PERIPH_SRC_DIR}/OWM/owm_reva.c
)
endif()
