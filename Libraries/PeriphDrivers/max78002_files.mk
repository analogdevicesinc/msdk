###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
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

# This is the name of the build output file

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC ?= $(subst m,M,$(subst a,A,$(subst x,X,$(TARGET))))
TARGET_LC ?= $(subst M,m,$(subst A,a,$(subst X,x,$(TARGET))))

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../CMSIS
endif
ifeq "$(LIBS_DIR)" ""
LIBS_DIR = $(CMSIS_ROOT)/..
endif

PERIPH_DIR := $(LIBS_DIR)/PeriphDrivers
SOURCE_DIR := $(PERIPH_DIR)/Source
INCLUDE_DIR := $(PERIPH_DIR)/Include

PERIPH_DRIVER_INCLUDE_DIR += $(INCLUDE_DIR)/$(TARGET_UC)/

# Expose a "PINS_FILE" option for easily overriding the pin definitions
PINS_FILE ?= $(SOURCE_DIR)/SYS/pins_ai87.c

# Source files



PERIPH_DRIVER_C_FILES += $(PINS_FILE)
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/sys_ai87.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ADC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/AES
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AES/aes_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AES/aes_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CAMERAIF
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CAMERAIF/cameraif_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CAMERAIF/cameraif_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CRC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CRC/crc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CRC/crc_reva.c

ifneq "$(RISCV_CORE)" "1"
ifneq "$(RISCV_CORE)" "RV32"
# The RISC-V core does not have access to the CSI2 peripheral.
PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CSI2
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CSI2/csi2_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CSI2/csi2_reva.c
endif
endif

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/DMA
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/FLC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/GPIO
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/I2C
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_reva.c
 
PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/I2S
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2S/i2s_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2S/i2s_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ICC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/LP
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/LP/lp_ai87.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/LPCMP
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/LPCMP/lpcmp_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/LPCMP/lpcmp_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/OWM
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/PT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/RTC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SDHC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SDHC/sdhc_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SDHC/sdhc_reva.c
USE_NATIVE_SDHC = yes

MXC_SPI_VERSION ?= v1
# Selects the SPI drivers to build with.  Acceptable values are:
# - v1
# - v2
PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SPI
export MXC_SPI_VERSION
ifeq ($(MXC_SPI_VERSION),v1)
# SPI v1 (Legacy) Implementation
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_reva1.c
PROJ_CFLAGS+=-DMXC_SPI_V1
else
ifeq ($(MXC_SPI_VERSION),v2)
# SPI v2 Implementation
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_ai87_v2.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_reva2.c
else
$(error Invalid value for MXC_SPI_VERSION = "$(MXC_SPI_VERSION)"  Acceptable values are "v1" or "v2")
endif
endif

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/TRNG
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TRNG/trng_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TRNG/trng_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/TMR
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/UART
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/WDT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/WUT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WUT/wut_ai87.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WUT/wut_reva.c
