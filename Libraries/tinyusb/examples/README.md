## Overview

This folder provides a verbatim copy of the TinyUSB device examples folder to
be used as a reference for additional USB applications for the supported MCUs.

While not all TinyUSB examples have been ported to an MSDK project, minimal
effort is required to create a new project from these examples.

## Porting Guide

 + Create a new MSDK project, or leverage an existing MSDK project as a template
 + Copy the .c and .h files from the src/ folder of the desired TinyUSB example into the MSDK project
 + Modify the project.mk file to include the following lines to build TinyUSB library and point to the local tusb_config.h file
     ```
     # Enable TINYUSB library
     LIB_TINYUSB=1
     TINYUSB_CONFIG_DIR = ./
     ```
 + The TinyUSB build system and the MSDK each have their own independent Board Support Packages (BSP). To leverage the
   example code as-is, wrappers functions must be created to bridge the BSPs.  Add the following functions
   to main.c for the project.  The board_init function must be modified for the correct USB startup per the selected MCU.
     ```
    //------------------------------------------------------------------------------
    // Wrapper functions to bridge TinyUSB BSP with MSDK BSP
    //------------------------------------------------------------------------------
    void board_init(void)
    {
        // 1ms tick timer
        SysTick_Config(SystemCoreClock / 1000);

        //USB Startup - This is MCU specific
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
        MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_0P9EN;
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_USB);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_USB);
    }

    void board_led_write(bool state)
    {
        if (state) {
            LED_On(0);
        } else {
            LED_Off(0);
        }
    }

    uint32_t board_button_read(void)
    {
        return PB_Get(0);
    }

    void USB_IRQHandler(void)
    {
        tud_int_handler(0);
    }

    volatile uint32_t system_ticks = 0;

    void SysTick_Handler(void)
    {
        system_ticks++;
    }

    uint32_t board_millis(void)
    {
        return system_ticks;
    }
     ```
