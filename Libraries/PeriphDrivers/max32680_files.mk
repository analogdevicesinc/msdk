################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ###############################################################################

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

PERIPH_DRIVER_INCLUDE_DIR  += $(INCLUDE_DIR)/$(TARGET_UC)/

# Source files
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_assert.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_delay.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_lock.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/nvic_table.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/pins_me20.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/sys_me17.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ADC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/AES
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AES/aes_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AES/aes_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/AFE
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AFE/afe.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AFE/afe_gpio.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AFE/hart_uart.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/AFE/infoblock.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CRC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CRC/crc_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CRC/crc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/DMA
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/FLC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/GPIO
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/I2C
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_reva.c
 
PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/I2S
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2S/i2s_me20.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2S/i2s_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ICC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_reva.c

ifeq "$(RISCV_CORE)" ""
PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/LP
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/LP/lp_me17.c
endif

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/OWM
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/PT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_me20.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/RTC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SEMA
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SEMA/sema_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SEMA/sema_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SIMO
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SIMO/simo_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SIMO/simo_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SPI
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/TRNG
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TRNG/trng_me20.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TRNG/trng_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/TMR
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/UART
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/WDT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_revb.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/WUT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WUT/wut_me17.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WUT/wut_reva.c

# Where to find header files for this project
PERIPH_DRIVER_H_FILES +=  $(shell find $(PERIPH_DRIVER_INCLUDE_DIR) -name '*.h')
