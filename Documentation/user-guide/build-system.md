# Build System

## Build System Overview

The **Build System** manages the compilation of source code into program binaries and offers a **Command-Line Interface (CLI)** for setting **Build Configuration Variables**. All IDEs interface with this system.

The Build System is managed by two files found in a project's root directory, one called **Makefile** and one called **project.mk**. These files are used by the [GNU Make](https://www.gnu.org/software/make/) program (which is a part of the MSDK toolchain) to locate and build a project's source code.

* **Makefile** is the "core" file and should not be edited directly. Instead, it exposes the **CLI** that can be accessed in the _project.mk_ file, on the command line, in your system's environment, or through your IDE. It also comes with a default configuration that is suitable for most projects.
* **project.mk** offers a convenient and stable access point for advanced build configuration, and this is the file that should be edited if necessary.

When the command

    make

is run from inside of a project folder, the program `make` will resolve any project-specific settings and then build the project's source code.

## Default Build Behavior

By default, the build system will **auto-search** the **root** project directory for _source code_ (**`*.c`**) and _header files_ (**`*.h`**) to compile into a program binary. The _optional_ **include** and **src** directories are also searched if they exist.

    :::bash
    Root Project Directory
    ├─ project.mk
    ├─ Makefile
    ├─ *.h
    ├─ *.c
    ├─include  # <-- Optional
    └─ *.h
    ├─src      # <-- Optional
    └─ *.c

Additionally, a project's build system will come pre-configured for a specific _Target Microcontroller_ and its primary _BSP_.

The default configuration is suitable for most use cases, but a system of _Build Configuration Variables_ is available if additional configuration is needed.

## Build Configuration Variables

A **Build Configuration Variable** is a [Makefile variable](https://www.gnu.org/software/make/manual/make.html#Using-Variables) and therefore follows the same rules. However, they have been streamlined to be made much easier to use, so most of the [official GNU Make documentation](https://www.gnu.org/software/make/manual/make.html) is only needed for advanced use cases.

### How to Set a Build Configuration Variable

To set a **standard** configuration variable, **use the `=` syntax**...

    VARIABLE=VALUE

The **`=`** operator is used for _most_ configuration variables with a few exceptions (documented in the [reference table](#build-tables)) when a variable should contain a **_list_ of values**. In such cases, **use `+=` the syntax** to _add_ values to the list.

    VARIABLE+=VALUE1
    VARIABLE+=VALUE2

### Where to Set a Build Configuration Variable

For most variables, you should set them in the **project.mk** file (exceptions are documented in the [reference table](#build-tables) and IDE-specific sections).

For example, to enable hardware floating-point acceleration for a project, the **`MFLOAT_ABI`** configuration variable can be used with a value of **`hard`**. The contents of **project.mk** might then look as follows:

(_Inside project.mk_)

    :::Make
    # This file can be used to set build configuration
    # variables. These variables are defined in a file called
    # "Makefile" that is located next to this one.

    # For instructions on how to use this system, see
    # https://analogdevicesinc.github.io/msdk/Documentation/user-guide/index.md

    # **********************************************************

    MFLOAT_ABI=hard # Enable hardware floating point acceleration

It should also be noted that configuration variables can be set on the **command line** as well. For example

    make MFLOAT_ABI=hard

will have the same effect.

Additionally, **environment variables** can be used. For example (on Linux)

    export MFLOAT_ABI=hard

will set the hardware floating point acceleration as the default for all projects with an environment variable.

However, there is a _precedence hierarchy_ that should be taken into consideration.

### Precedence Hierarchy

The precedence hierarchy for the value of a configuration variable is:

* **IDE/command-line > project.mk > environment variable > default value**

If a value is set in an IDE _and_ project.mk, the IDE's value will take precedence. However, the ["override" directive](https://www.gnu.org/software/make/manual/make.html#Override-Directive) can be used in project.mk to give it max precedence.

## Build Tables

The following sections present the available [Build Configuration Variables](#build-configuration-variables).

### Primary Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MAXIM_PATH`           | (Optional) Specify the location of the MSDK                | This optional variable can be used to change where the Makefile looks for the MSDK installation. By default, the build system will attempt to locate the MSDK with a relative path. If a project is moved _outside_ of the SDK, this variable must be set to the absolute path of the MSDK installation. |
| `TARGET`               | Set the _Target Microcontroller_                           | **If you are using an IDE, set this variable in the IDE's settings instead of project.mk** |
| `BOARD`                | Set the _Board Support Package (BSP)_                      | **If you are using an IDE, set this variable in the IDE's settings instead of project.mk.**  See [Board Support Packages](board-support-pkgs.md) for more details.  When you change this option, it's usually a good idea to fully clean your project, then rebuild. |
| `BSP_SEARCH_DIR`       | Set the directory to search for the _Board Support Package (BSP)_                      | By default, the `Libraries/Boards` folder of the MSDK is searched for the `TARGET` microcontroller.  This setting is useful for loading custom BSPs from outside of the MSDK.  When `LIB_BOARD=1`, the build system looks for the file path at `$(BSP_SEARCH_DIR)/$(BOARD)/board.mk`. See [BSP Search Directory](board-support-pkgs.md/bsp-search-directory) for more details. |

### Project Build Variables

The following variables deal with fundamental project tasks such as adding source code, include paths, changing the output filename, etc.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `VPATH`                | Where to search for source (.c/.cpp) files                      | **Use the `+=` operator with this variable**.  This controls where the Makefile will look for **source code** files. If `AUTOSEARCH` is enabled (which it is by default), all source code files in the directories specified by this option will be automatically added to the build. If `AUTOSEARCH` is disabled, this tells the Makefile where to look for the files specified by `SRCS`. |
| `IPATH`                | Where to search for header (.h) files                      | **Use the `+=` operator with this variable**.  This controls where the Makefile will look for **header** files. _Unlike_ the `VPATH` option, this is not related to `AUTOSEARCH`. Individual header files are _not_ ever manually added to the build. Instead, you only need to specify the _location_ of your header files. |
| `SRCS`                 | List of source (.c/.cpp) files to add to the build              | **Use the `+=` operator with this variable**. All of the files in this list will be added to the build. If `AUTOSEARCH` is enabled, this is most useful for adding the full absolute path to a singular source file to selectively add to the build. If `AUTOSEARCH` is disabled, _all_ of the source files for the project must be added to `SRCS`, and they must also all be located on an entry in `VPATH`. Otherwise, a full path relative to the Makefile must be used. |
| `AUTOSEARCH`           | Automatically search for source (.c/.cpp) files                 | Enable or disable the automatic detection of .c files on `VPATH` (enabled by default). Set to `0` to disable or `1` to enable. If auto-search is disabled, source files must be manually added to `SRCS`. |
| `PROJECT`              | Set the output filename                                    | This controls the output filename of the build.  File extensions should _not_ be included in the filename.  **For VS Code, you should use the [project_name](visual-studio-code.md/project_name) advanced config option instead of project.mk.** |
| `PROJ_LIBS`            | Add a static library file (.a) to the project              | **Use the `+=` operator with this variable**.  Additional static libraries to link against can be added with this option. It should be noted that static library files are named with the `lib<libraryname>.a` convention.  Only add `<libraryname>` to this variable. Example: Give a file called `libEXAMPLE.a`, write `PROJ_LIBS += EXAMPLE` Additionally, ensure that the location of the library is added to `PROJ_LDFLAGS`. Example: `PROJ_LDFLAGS += -Lsome/library/search/directory` |

### Build Variables for the Compiler

The following variables can be used to interface with the compiler to perform common tasks such as changing the optimization level, adding compiler definitions to the build, and changing floating point acceleration.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MXC_OPTIMIZE_CFLAGS`  | Set the optimization level                                 | See [Optimize Options](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html) for more details.  Normal builds will default to `-Og`, which is good for debugging, while release builds will default to `-O2`. |
| `PROJ_CFLAGS`          | Add compiler flags to the build                            | **Use the `+=` operator with this variable**.  Compiler flags can be added with this option, including compiler definitions. For each value, the same syntax should be used as if the compiler flag was passed in over the command line. These can include standard [GCC options](https://gcc.gnu.org/onlinedocs/gcc-10.4.0/gcc/Option-Summary.html#Option-Summary) and/or [ARM-specific](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) options. |
| `PROJ_AFLAGS`          | Add assemblers flag to the build                           | **Use the `+=` operator with this variable**.  Assembler flags can be added with this option. |
| `PROJ_OBJS`            | Add object files to the build                              | **Use the `+=` operator with this variable**.  If needed, object files (.o) can be added to the build with this option. |
| `DEBUG`                | Toggle extra debug information  | Set this to `1` to enable extra debug information at compile time.  This generally improves the reliability of debugging at some increase in code size.  Set to `0` to disable. |

### Build Variables for the Linker

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `LINKERFILE`           | Set the linkerfile to use                                  | A linkerfile is responsible for specifying the available memory banks, their layout, and the organization of program binaries memory.  The file should exist in `Libraries/CMSIS/Device/Maxim/TARGET/Source/GCC` in the MSDK, or it should be placed inside the root directory of the project. |
| `PROJ_LDFLAGS`         | Add a linker flag to the build                             | **Use the `+=` operator with this variable**.  Flags can be passed to the linker with this option. See [GCC Options for Linking](https://gcc.gnu.org/onlinedocs/gcc/Link-Options.html#Link-Options) |

### Build Variables for Arm Cores

The following build variables are used to control options specific to the Arm Cortex-M4 core available.  They are available on all microcontrollers, and for all projects unless that project is built for a RISC-V core.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MFLOAT_ABI`           | Set the floating point acceleration level                  | Sets the floating-point acceleration level.  Permitted values are `hard`, `soft`, and `softfp` (default). To enable full hardware acceleration instructions, use `hard`, but keep in mind that _all_ libraries your source code uses must also be compiled with `hard`. If there is any conflict, you'll get a linker error. For more details, see `-mfloat-abi` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html). |
| `DEFAULT_OPTIMIZE_FLAGS` | Override the default extra optimization flags | Extra compiler optimization flags are added to the build. They are defined in `Libraries/CMSIS/Device/Maxim/GCC/gcc.mk`.  These can be disabled entirely by setting this variable to empty (`DEFAULT_OPTIMIZE_FLAGS=`). |
| `DEFAULT_WARNING_FLAGS` | Override the default warning flags | Default flags controlling warning output are added in `Libraries/CMSIS/Device/Maxim/GCC/gcc.mk`.  These can be disabled entirely by setting this variable to empty (`DEFAULT_OPTIMIZE_FLAGS=`). |
| `MCPU`           | Set the processor type                  | Set the target ARM processor.  Directly maps to `-mcpu` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html).  This flag is handled by the MSDK and not typically changed manually. |
| `MFPU`           | Set the FPU architecture                  | Set the floating point unit (FPU) architecture.  Directly maps to `-mfpu` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html). This flag is handled by the MSDK and not typically changed manually. |

### Build Variables for RISC-V Cores

The following build variables are used for RISC-V development.  They are only available on microcontrollers with RISC-V cores.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `RISCV_CORE`           | Build a project for the RISC-V core                        | Set to `1` to convert an entire project to use the RISC-V toolchain.  Only available on microcontrollers with a RISC-V core. |
| `RISCV_LOAD`           | Compile and load project for the RISC-V core               | **Only available on the MAX32655, MAX32680, and MAX32690**.  Set to `1` compile the project specified by `RISCV_APP` for the RISC-V core and link it into the same binary as the current project.  Useful for dual-core projects. |
| `RISCV_APP`            | Project folder to compile for the `RISCV_LOAD` option      | **Only available on the MAX32655, MAX32680, and MAX32690**.  This option specifies the project to build for the RISC-V core when `RISCV_LOAD` is enabled.  Must be a path relative to the project that enables `RISCV_LOAD`, or an absolute path. |
| `RISCV_PREFIX`         | Change the toolchain prefix                                | This option can be used to override the GCC toolchain prefix if needed.  For example, to use the legacy RISC-V toolchain `RISCV_PREFIX = riscv-none-embed` will attempt to compile with `riscv-none-embed-gcc`. |

### Build Variables for Toggling Libraries

The following variables can be used to enable the [available libraries](libraries.md) in the MSDK.  Each library may also offer its own build configuration variables when enabled, which are documented in the [libraries](libraries.md) section.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `LIB_BOARD`            | Include the BSP library (enabled by default)               | Inclusion of the Board-Support Package (BSP) library, which is enabled by default, can be toggled with this variable. Set to `0` to disable or `1` to enable. |
| `LIB_PERIPHDRIVERS`    | Include the peripheral driver library (enabled by default) | The peripheral driver library can be toggled with this option. If disabled, you'll lose access to the higher-level driver functions but still have access to the register-level files. Set to `0` to disable or `1` to enable. |
| `LIB_CMSIS_DSP`        | Include the CMSIS-DSP library                              | The [CMSIS-DSP library](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html) can be enabled with this option.  Set to `0` to disable or `1` to enable. |
| `LIB_CORDIO`           | Include the Cordio library                                 | The Cordio BLE library can be included with this option. This is only applicable for microcontrollers with an integrated BLE controller. |
| `LIB_FCL`              | Include the Free Cryptographic Library (FCL)               | This option toggles the Free Cryptographic Library (FCL), which is a collection of software-implemented common cryptographic functions that can be included with this option. Set to `0` to disable or `1` to enable. |
| `LIB_FREERTOS`         | Include the FreeRTOS library                               | The [FreeRTOS](https://freertos.org/) library can be enabled with this option, which is an open-source Real-Time Operating System (RTOS). Set to `0` to disable or `1` to enable. |
| `LIB_LC3`              | Include the LC3 codec library                              | This option enables the inclusion of the Low Complexity Communication Codec (LC3), which is an efficient low latency audio codec. Set to `0` to disable or `1` to enable. |
| `LIB_LITTLEFS`         | Include the littleFS library                               | This option toggles the ["Little File System"](https://github.com/littlefs-project/littlefs) library - a small filesystem library designed for microcontrollers.  Set to `0` to disable or `1` to enable. |
| `LIB_LWIP`             | Include the lwIP library                                   |                                                              |
| `LIB_MAXUSB`           | Include the MaxUSB library                                 | This option toggles the inclusion of the MAXUSB library, which facilitates the use of the native USB peripherals on some microcontrollers. Set to `0` to disable or `1` to enable. |
| `LIB_TINY_USB`         | Include the TinyUSB library                                | This option toggles the inclusion of the TinyUSB library, which facilitates the use of the native USB peripherals on some microcontrollers. Set to `0` to disable or `1` to enable. |
| `LIB_SDHC`             | Include the SDHC library                                   | This option toggles the Secure Digital High Capacity (SDHC) library, which can be used to interface with SD cards. Additionally, it enables the [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) library, which implements a generic FAT filesystem. |
| `LIB_CLI`             | Include the MSDK's built-in CLI library                     | This option toggles the MSDK's built-in CLI library, which can be used to process received commands over UART. |
| `LIB_USS`             | Include the USS Library                                     | This option toggles the Unified Security Software library.  It is only available via NDA. |

### Build Variables for the PeriphDrivers Library

The following variables are specific to the PeriphDrivers library.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MXC_SPI_VERSION`            | Set the SPI drivers to use (default is `v1`) | The PeriphDrivers offer two versions of the SPI API in order to maintain backwards compatibility.  Acceptable values are `v1` (legacy) or `v2`.  See [The SPI V2 Developer Note](developer-notes.md#spi-v2-library) for more details. |

### Build Variables for Secure Boot Tools (SBTs)

For microcontrollers with a secure bootloader, the following build configuration variables can be used to enable integration with the Secure Boot Tools.  These are a suite of applications designed for use with microcontrollers that have secure bootloaders.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `SBT`                  | Toggle SBT integration                                     | Toggles integration with the [Secure Boot Tools (SBTs)](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download.html?swpart=SFW0015360C). These are a suite of applications designed for use with microcontrollers that have secure bootloaders. When this is enabled, some additional rules become available such as `make sla` and `make scpa`. Set to `0` to disable or `1` to enable. |
| `MAXIM_SBT_DIR`        | Where to find the SBTs                                     | This option can be used to manually specify the location of the SBTs. Usually, this is not necessary. By default, the `Tools/SBT` directory of the MaximSDK will be searched. If the [SBT installer](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download.html?swpart=SFW0015360C) is used, it will set the `MAXIM_SBT_DIR` environment variable to point to itself automatically. |
| `TARGET_SEC`           | Secure part number to use                                  | Some secure microcontrollers have multiple secure variants, and this option can be used to specify the variant to use with the SBTs.  Defaults are intelligently selected and can be found in `$(MAXIM_SBT_DIR)/SBT-config.mk` |
| `SCP_PACKETS`          | Where to build the scp_packets folder                      | Defaults to `build/scp_packets`                              |
| `TEST_KEY`             | Which test key to sign applications with                   | Defaults to `$(MAXIM_SBT_DIR)/devices/$(TARGET_SEC)/keys/maximtestcrk.key`, which is the Maxim test key that can be used for development. |

### Build Variables Controlling the Output

The following build variables can be used to control how to build output is formatted.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `VERBOSE`              | Toggle verbose builds             | Set to `1` to enable a verbose build that prints exactly what the compiler is doing for each step.  This is useful for troubleshooting. |
| `FORCE_COLOR`          | Force colorized compiler output   | By default, GCC will attempt to autodetect whether colorized output is supported or not.  Set to `1` to force color (equivalent to `PROJ_CFLAGS += -fdiagnostics-color=always`).  This is useful for forcing color in CI systems. |
