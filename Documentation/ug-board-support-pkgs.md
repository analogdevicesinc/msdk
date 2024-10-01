# Board Support Packages

The MSDK supports multiple parts and evaluation platforms (see [supported parts](#supported-parts)) through **"Board Support Packages" (BSPs)**. For microcontrollers with multiple evaluation platforms, multiple BSPs will be available.

The role of a _BSP_ is to provide a hardware abstraction layer for the initialization and management of board-level hardware such as serial interfaces, pushbuttons, LEDs, external peripheral devices, TFT displays, etc. which will vary between evaluation platforms. The BSP abstraction layer also improves code portability to custom devices.

???+ note "ℹ️ **Note**"
    The first task when opening or creating any project is to ensure the BSP is set correctly.

## How to Set the BSP

To set the BSP for a project:

- In **VS Code**:  [How to Set the BSP (VS Code)](#how-to-set-the-bsp-vs-code)
- In **Eclipse**:  [How to Set the BSP (Eclipse)](#how-to-set-the-bsp-eclipse)
- **Command-Line** Development:  [How to Set the BSP (Command-Line)](#how-to-set-the-bsp-command-line)

## BSP Table

Available BSPs are located in the `Libraries/Boards` folder for each _Target Microcontroller_.

![Figure 34](res/Fig34.jpg)

The name of a BSP's folder is used with the `BOARD` [build configuration variable](#build-configuration-variables) to build a project for a specific BSP. The table below matches the correct `BOARD` values to _external part numbers_.

| External Part Number                         | `TARGET`       | `BOARD`        |
| ---------------------------------------------|--------------- | -------------- |
| [MAX32520-KIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32520-kit.html)      | `MAX32520`     | `EvKit_V1`     |
| [MAX32520FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32520fthr.html)      | `MAX32520`     | `MAX32520FTHR` |
| [MAX32650-EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32650-evkit.html)    | `MAX32650`     | `EvKit_V1`     |
| [MAX32650FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32650fthr.html)      | `MAX32650`     | `FTHR_APPS_A`  |
| [MAX32655EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32655evkit.html)     | `MAX32655`     | `EvKit_V1`     |
| [MAX32655FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32655fthr.html)      | `MAX32655`     | `FTHR_Apps_P1` |
| [MAX32660-EVSYS](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32660-evsys.html)    | `MAX32660`     | `EvKit_V1`     |
| [MAX32662EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/MAX32662EVKIT.html)                                | `MAX32662`     | `EvKit_V1`     |
| [MAX32666EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32666evkit.html)     | `MAX32665`     | `EvKit_V1`     |
| [MAX32666FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32666fthr.html)      | `MAX32665`     | `FTHR`         |
| MAX32666FTHR2                                | `MAX32665`     | `FTHR2`        |
| [MAX32670EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32670evkit.html)     | `MAX32670`     | `EvKit_V1`     |
| [MAX32672EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32672evkit.html)     | `MAX32672`     | `EvKit_V1`     |
| [MAX32672FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32672fthr.html)      | `MAX32672`     | `FTHR`         |
| [MAX32675EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32675evkit.html)     | `MAX32675`     | `EvKit_V1`     |
| MAX32675FTHR                                 | `MAX32675`     | `FTHR_Apps_B`  |
| [MAX32680EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32680evkit.html)     | `MAX32680`     | `EvKit_V1`     |
| [MAX32690EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/MAX32690EVKIT.html)                                | `MAX32690`     | `EvKit_V1`     |
| [AD-APARD32690-SL](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/ad-apard32690-sl.html)     | `MAX32690`    | `APARD`   |
| [MAX78000EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html)     | `MAX78000`     | `EvKit_V1`     |
| [MAX78000FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000fthr.html)      | `MAX78000`     | `FTHR_RevA`    |
| [MAXREFDES178](https://www.analog.com/en/design-center/reference-designs/maxrefdes178.html)                                          | `MAX78000`     |  `MAXREFDES178` |
| MAX78000CAM01 (Engineering samples only)      | `MAX78000`     | `CAM01_RevA`    |
| MAX78000CAM02 (Engineering samples only)      | `MAX78000`     | `CAM02_RevA`    |
| [MAX78002EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78002evkit.html)     | `MAX78002`     | `EvKit_V1`     |

## Custom BSPs

For custom boards, additional BSPs can be easily created and added to the MSDK. Inspecting the `Libraries/CMSIS/Device/Maxim/TARGET/Source/system_TARGET.c` for a target microcontroller shows how the BSP is integrated into the microcontroller's startup code.

For example, the MAX78000's `system_max78000.c` startup file shows that `Board_Init` is a weak function that can be overridden. `Board_Init` is called from the default `SystemInit` implementation, which can also be overridden.

    :::C
    /* This function is called before C runtime initialization and can be
    * implemented by the application for early initializations. If a value other
    * than '0' is returned, the C runtime initialization will be skipped.
    *
    * You may over-ride this function in your program by defining a custom
    *  PreInit(), but care should be taken to reproduce the initialization steps
    *  or a non-functional system may result.
    */
    __weak int PreInit(void)
    {
        /* Do nothing */
        return 0;
    }

    /* This function can be implemented by the application to initialize the board */
    __weak int Board_Init(void)
    {
        /* Do nothing */
        return 0;
    }

    /* This function is called just before control is transferred to main().
    *
    * You may over-ride this function in your program by defining a custom
    *  SystemInit(), but care should be taken to reproduce the initialization
    *  steps or a non-functional system may result.
    */
    __weak void SystemInit(void)
    {
        /* Configure the interrupt controller to use the application vector table in */
        /* the application space */
    #if defined(__CC_ARM) || defined(__GNUC__)
        /* IAR sets the VTOR pointer incorrectly and causes stack corruption */
        SCB->VTOR = (uint32_t)__isr_vector;
    #endif /* __CC_ARM || __GNUC__ */

        /* Enable instruction cache */
        MXC_ICC_Enable(MXC_ICC0);

        /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 and 11 */
        /* Grant full access, per "Table B3-24 CPACR bit assignments". */
        /* DDI0403D "ARMv7-M Architecture Reference Manual" */
        SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
        __DSB();
        __ISB();

        SystemCoreClockUpdate();

        Board_Init();
    }

A custom BSP can implement one or all of the weak functions. The file structure for a typical BSP can be found below.  **The board.mk file is required**, while the rest of the project structure is a recommendation.

    :::bash
        CustomBSP (defines BOARD value)
         ├─ board.mk (required file!)
         ├─ Include
         |  └─ board.h
         └─ Source
            └─ board.c

The name of the BSP's root folder will be the string used with the `BOARD` [build configuration variable](#build-configuration-variables) to select it for a project.  In the example above, one would use `BOARD = CustomBSP` to select it as the active BSP.

### BSP Search Directory

By default, the MSDK searches for BSPs in the `Libraries/Boards` folder for each microcontroller.  This can be changed using the `BSP_SEARCH_DIR` [build configuration variable](#build-configuration-variables), which allows users to load a BSP from a directory outside of the MSDK.  The MSDK also uses the `BOARD` variable in its search path.

For example, the configuration...

    :::Makefile
    # project.mk

    BSP_SEARCH_DIR = /home/username/mybsps
    # ^ "root" of the BSP search path
    BOARD = CustomBSP
    # "stem" of the BSP search path

... will attempt to load the `/home/username/msbsps/CustomBSP/board.mk` file.

### Custom BSP Template

The following contents can be used as a bare-bones starter template for a custom BSP.

* _board.h_

        :::C
        // board.h

        #define BOARD_CUSTOM
        // ^ This type of compiler definition is
        // sometimes useful. It allows application code
        // to check if a specific BSP is being used.
        // Ex: #ifdef BOARD_CUSTOM
        //     ...
        //     #endif

        /**
        * \brief   Initialize the BSP and board interfaces.
        * \returns #E_NO_ERROR if everything is successful
        */
        int Board_Init(void);

* _board.c_

        :::C
        //board.c
        #include "board.h"
        #include "mxc_error.h"

        int Board_Init(void)
        {
            // Implement me!
            return E_NO_ERROR;
        }

* _board.mk_

        :::Makefile
        # board.mk

        ifeq "$(BOARD_DIR)" ""
        # This Makefile will self-locate if BOARD_DIR is not specified.
        BOARD_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
        endif

        SRCS += board.c
        VPATH += $(BOARD_DIR)/Source
        IPATH += $(BOARD_DIR)/Include

## Disabling BSPs

It should also be noted that BSP integration can be disabled entirely by setting the `LIB_BOARD` [build configuration variable](#build-configuration-variables) to 0. This will skip the inclusion of the BSP's `board.mk` file entirely, and the default system initialization functions will be used.

This option can also be used to implement a custom BSP inside of a project's application code.  For example, a user could implement `Board_Init` inside of a project's `main.c` file without having to create a separate BSP folder with `LIB_BOARD = 0`.

* _project.mk_

        :::Makefile
        # project.mk

        LIB_BOARD = 0

* _main.c_

        :::C
        // main.c
        int Board_Init(void)
        {
            // Implement me!
            return E_NO_ERROR;
        }

        int main(void)
        {
            Board_Init();
            // ...
        }
