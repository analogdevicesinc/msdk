################################################################################
 # Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 # 
 # This software is protected by copyright laws of the United States and
 # of foreign countries. This material may also be protected by patent laws
 # and technology transfer regulations of the United States and of foreign
 # countries. This software is furnished under a license agreement and/or a
 # nondisclosure agreement and may only be used or reproduced in accordance
 # with the terms of those agreements. Dissemination of this information to
 # any party or parties not specified in the license agreement and/or
 # nondisclosure agreement is expressly prohibited.
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
# Source files)
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_assert.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_delay.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/mxc_lock.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/pins_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/sys_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SYS/nvic_table.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ADC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ADC/adc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CAMERAIF
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CAMERAIF/cameraif_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CAMERAIF/cameraif_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/CTB
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CTB/ctb_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CTB/ctb_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/CTB/ctb_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/DMA
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/DMA/dma_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/EMAC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/EMAC/emac_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/EMAC/emac_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/FLC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/FLC/flc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/GPIO
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/GPIO/gpio_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/HTMR
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/HTMR/htmr_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/HTMR/htmr_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/I2C
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/I2C/i2c_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/ICC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/ICC/icc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/LP
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/LP/lp_me13.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/OWM
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/OWM/owm_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/PT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/PT/pt_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/RTC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/RTC/rtc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SDHC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SDHC/sdhc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SDHC/sdhc_reva.c
USE_NATIVE_SDHC = yes

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SEMA
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SEMA/sema_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SEMA/sema_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SFCC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SFCC/sfcc_me13.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SKBD
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SKBD/skbd_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SKBD/skbd_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SMON
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SMON/smon_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SMON/smon_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SPI
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPI/spi_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SPIXF
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPIXF/spixf_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPIXF/spixf_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SPIXR
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPIXR/spixr_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SPIXR/spixr_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/SRCC
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SRCC/srcc_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/SRCC/srcc_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/TMR
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/TMR/tmr_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/UART
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/UART/uart_reva.c

PERIPH_DRIVER_INCLUDE_DIR += $(SOURCE_DIR)/WDT
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_common.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_me13.c
PERIPH_DRIVER_C_FILES += $(SOURCE_DIR)/WDT/wdt_reva.c

# Where to find header files for this project
PERIPH_DRIVER_H_FILES +=  $(shell find $(PERIPH_DRIVER_INCLUDE_DIR) -name '*.h')
