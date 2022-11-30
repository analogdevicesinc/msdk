# VSCode-Maxim

_(If you're viewing this document from within Visual Studio Code you can press `CTRL+SHIFT+V` to open a Markdown preview window.)_

## Quick Links

* [VSCode-Maxim Github](https://github.com/Analog-Devices-MSDK/VSCode-Maxim)
* [Wiki](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/wiki)
  * If it's not in the readme, check the wiki.
  * If it's not in the wiki, open a ticket!

## Introduction

VSCode-Maxim is a set of [Visual Studio Code](https://code.visualstudio.com/) project configurations and utilities for enabling embedded development for [Analog Device's MSDK](https://github.com/Analog-Devices-MSDK/msdk) and the [MAX-series](https://www.maximintegrated.com/en/products/microcontrollers.html) microcontrollers.

The following features are supported:

* Code editing with intellisense down to the register level
* Code compilation with the ability to easily re-target a project for different microcontrollers and boards
* Flashing programs
* GUI and command-line debugging

## Dependencies

* [Visual Studio Code](https://code.visualstudio.com/)
* [C/C++ VSCode Extension](https://github.com/microsoft/vscode-cpptools)
* [Maxim Micros SDK](https://www.maximintegrated.com/content/maximintegrated/en/design/software-description.html/swpart=SFW0010820A)

## Installation

The steps below are also available in video form in "Understanding Artificial Intelligence Episode 8.5 - Visual Studio Code" [here](https://www.maximintegrated.com/en/products/microcontrollers/artificial-intelligence.html/tab4/vd_1_2eaktism#.YyDxHaE8U_Y.mailto).

1. Download & install the Maxim Microcontrollers SDK for your OS from the links below.
    * [Windows](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0010820A)
    * [Linux (Ubuntu)](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018720A)
    * [MacOS](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018610A)

2. Run the installer executable, and ensure that "Visual Studio Code Support" is enabled for your installation.

    ![Selected Components](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/installer_components.JPG)

3. Finish the MaximSDK installation, taking note of where the MaximSDK was installed.

4. Download & install Visual Studio Code for your OS [here](https://code.visualstudio.com/Download).

5. Launch Visual Studio Code.

6. Install the Microsoft [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).

7. Use `CTRL + SHIFT + P` (or `COMMAND + SHIFT + P` on MacOS) to open the developer prompt.

8. Type "open settings json" and select the "Preferences: Open Settings (JSON)" option (_not_ the "Preferences: Open _Default_ Settings (JSON)").  This will open your user settings.json file in VS Code's editor.

    ![Open Settings JSON Command](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/open_settings_json.jpg)

9. Add the entries below into your user settings.json file.

    ```json
    {
        // There may be other settings up here...
        
        "MAXIM_PATH":"C:/MaximSDK", // Set this to the installed location of the MaximSDK.  Only use forward slashes '/' when setting this path!
        "update.mode": "manual",
        "extensions.autoUpdate": false,
        
        // There may be other settings down here...
    }
    ```

10. Save your changes to the file with `CTRL + S` and restart VS Code.

11. That's it!  You're ready to start using Visual Studio Code to develop with Maxim's Microcontrollers.  The MaximSDK examples come pre-populated with .vscode project folders, and the `Tools/VSCode-Maxim` folder of the SDK contains documentation and templates.  See [Usage](#usage) below for more details.

## Usage

This section covers basic usage of the VSCode-Maxim project files.  For documentation on Visual Studio Code itself, please refer to the official docs [here](https://code.visualstudio.com/Docs).

### Opening Projects

Visual Studio Code is built around a "working directory" paradigm.  The editor is always rooted in a working directory, and the main mechanism for changing that directory is `File -> Open Folder...`.

![File -> Open Folder](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/file_openfolder.JPG)

As a result, you'll notice that there is no "New Project" mechanism.  A "project" in VS Code is simply a folder.  It will look inside of the opened folder for a `.vscode` _sub_-folder to load project-specific settings from.

A project that is configured for VS Code will have, at minimum, a .vscode sub-folder and a Makefile in its directory _(Note:  You may need to enable viewing of hidden items in your file explorer to see the .vscode sub-folder)_.  

Ex:

![Example Directory Contents](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/opening_projects_2.jpg)

### Where to Find Projects

The [Examples](https://github.com/Analog-Devices-MSDK/msdk/tree/main/Examples) in the MSDK come with with pre-configured .vscode project folders.  These projects can be opened "out of the box", but it's good practice to copy example folders _outside_ of the MSDK so that the original copies are kept as clean references.  The examples can be freely moved to any location _without a space in its path_.

Additionally, empty project templates and a drag-and-drop folder for "injecting" a VSCode-Maxim project can be found under `Tools/VSCode-Maxim` in the MaximSDK installation.

### Build Tasks

Once a project is opened 4 available build tasks will become available via `Terminal > Run Build task...` or the shortcut `Ctrl+Shift+B`.  These tasks are configured by the `.vscode/task.json` file.

![Build Tasks Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/buildtasks.JPG)

#### Build

* Compiles the code with a `make all` command.
* Additional options are passed into Make on the command-line based on the project's settings.json file.
* The `./build` directory will be created and will contain the output binary, as well as all intermediary object files.

#### Clean

* Cleans the build output, removing the `./build` directory and all of its contents.

#### Clean-Periph

* This task is the same as 'clean', but it also removes the build output for Maxim's peripheral drivers.
* Use this if you would like to recompile the peripheral drivers from source on the next build.

#### Flash

* Launching this task automatically runs the `Build` task first.  Then, it flashes the output binary to the microcontroller.
* It uses the GDB `load` and `compare-sections` commands, and handles launching an OpenOCD internally via a pipe connection.
* The flashed program will be halted until the microcontroller is reset, power cycled, or a debugger is connected.
* A debugger must be connected correctly to use this task.  Refer to the datasheet of your microcontroller's evaluation board for instructions.
  
#### Flash & Run

* This is the same as the `Flash` task, but it also will launch execution of the program once flashing is complete.
  
#### Erase Flash

* Completely erases all of the application code in the flash memory bank.
* Once complete, the target microcontroller will be effectively "blank".
* This can be useful for recovering from Low-Power (LP) lockouts, bad firmware, etc.

### Debugging

![Debug Window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger.JPG)

Debugging is enabled by Visual Studio Code's integrated debugger.  Launch configurations can be found in the `.vscode/launch.json` file.

* Note: **Flashing does not happen automatically when launching the debugger.**  Run the "Flash" [build task](#build-tasks) for your program _before_ debugging.

#### Debugger Limitations

In general, Maxim's microcontrollers have the following debugger limitations at the hardware level:

* The debugger can not be connected _while_ the device is in reset.

* The device can not be debugged while the device is in Sleep, Low Power Mode, Micro Power Mode, Standby, Backup, or Shutdown mode.  These modes shut down the SWD clock.

* These limitations can sometimes make the device difficult or impossible to connect to if firmware has locked out the debugger.  In such cases, the ["Erase Flash"](#erase-flash) task can be used to recover the part.

#### Launching the Debugger

1. Attach your debugger to the SWD port on the target microcontroller.  (Refer to the datasheet of your evaluation board for instructions on connecting a debugger)

2. Flash the program to the microcontroller with the "Flash" [Build Task](#build-tasks).  **Flashing does not happen automatically when launching the debugger.**

3. Launch the debugger with `Run > Start Debugging`, with the shortcut `F5`, or via the `Run and Debug` window (Ctrl + Shift + D) and the green "launch" arrow.  

    ![Debug Tab](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger_window.JPG)

4. The debugger will launch a GDB client & OpenOCD server, reset the microcontroller, and should break on entry into `main`.

    ![Debugger Break on Main](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger_breakmain.JPG)

#### Using the Debugger

* For full usage details, please refer to the [official VS Code debugger documentation](https://code.visualstudio.com/docs/editor/debugging).

The main interface for the debugger is the debugger control bar:

![Debugger Control Bar Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger_bar.JPG)

`Continue | Step Over | Step Into | Step Out | Restart | Stop`

Breakpoints can be set by clicking in the space next to the line number in a source code file.  A red dot indicates a line to break on.  Breakpoints can be removed by clicking on them again.  Ex:

![Breakpoint](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/breakpoint.JPG)

## Project Configuration

### Project Settings

`.vscode/settings.json` is the main project configuration file.  Values set here are parsed into the other .json config files.  

**When a change is made to this file, VS Code should be reloaded with CTRL+SHIFT+P -> Reload Window (or alternatively restarted completely) to force a re-parse.**

![Reload Window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/reload_window.JPG)

The default project configuration should work for most use cases as long as `"target"` and `"board"` are set correctly.

Any field from `settings.json` can be referenced from any other config file (including itself) with `"${config:[fieldname]}"`

The following configuration options are available:

### Basic Config Options

#### `"target"`

* This sets the target microcontroller for the project.
* It sets the `TARGET` [Build Configuration](#build-configuration) variable.
* Supported values:
  * `"MAX32520"`
  * `"MAX32570"`
  * `"MAX32650"`
  * `"MAX32655"`
  * `"MAX32660"`
  * `"MAX32662"`
  * `"MAX32665"` (for MAX32665-MAX32668)
  * `"MAX32670"`
  * `"MAX32672"`
  * `"MAX32675"`
  * `"MAX32680"`
  * `"MAX32690"`
  * `"MAX78000"`
  * `"MAX78002"`

#### `"board"`

* This sets the target board for the project (ie. Evaluation Kit, Feather board, etc.)
* Supported values:
  * ... can be found in the `Libraries/Boards` folder of the MaximSDK
  * For example, the supported options for the MAX78000 are `"EvKit_V1"`, `"FTHR_RevA"`, and `"MAXREFDES178"`.

  ![MAX78000 Boards](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/78000_boards.JPG)

### Advanced Config Options

#### `"MAXIM_PATH"`

* This option must point to the root installation directory of the MaximSDK.  
* It should be placed in the _global_ user settings.json file during first-time VSCode-Maxim setup.  See [Installation](#installation).

#### `"terminal.integrated.env.[platform]:Path"`

* This prepends the location of toolchain binaries to the system `Path` used by VSCode's integrated terminal.
* The Path is not sanitized by default, which means that the terminal inherits the system path.
* Don't touch unless you know what you're doing :)

#### `"project_name"`

* Sets the name of project.  This is used in other config options such as `program_file`.
* Default value: `"${workspaceFolderBasename}"`

#### `"program_file"`

* Sets the name of the file to flash and debug.  This is provided in case it's needed, but for most use cases should be left at its default.  
* File extension must be included.
* Default value: `"${config:project_name}.elf"`

#### `"symbol_file"`

* Sets the name of the file that GDB will load debug symbols from.
* File extension must be included.
* Default value: `"${config:program_file}"`

#### `"M4_OCD_interface_file"`

* Sets the OpenOCD interface file to use to connect to the Arm M4 core.  This should match the debugger being used for the M4 core.
* The `MaximSDK/Tools/OpenOCD/scripts/interface` folder is searched for the file specified by this setting.
* `.cfg` file extension must be included.
* Default value: `"cmsis-dap.cfg"`

#### `"M4_OCD_target_file"`

* Sets the OpenOCD target file to use for the Arm M4 core.  This should match the target microcontroller.
* `.cfg` file extension must be included.
* The `MaximSDK/Tools/OpenOCD/scripts/target` folder is searched for the file specified by this setting.
* Default value: `"${config:target}.cfg"`

#### `"RV_OCD_interface_file"`

* Sets the OpenOCD interface file to use to connect to the RISC-V core.  This should match the debugger being used for the RISC-V core.
* The `MaximSDK/Tools/OpenOCD/scripts/interface` folder is searched for the file specified by this setting.
* `.cfg` file extension must be included.
* Default value: `"ftdi/olimex-arm-usb-ocd-h.cfg"`

#### `"RV_OCD_target_file"`

* Sets the OpenOCD target file to use for the RISC-V core.
* The `MaximSDK/Tools/OpenOCD/scripts/target` folder is searched for the file specified by this setting.
* `.cfg` file extension must be included.
* Default value: `"${config:target}_riscv.cfg"`

#### `"v_Arm_GCC"`

* Sets the version of the Arm Embedded GCC to use, including toolchain binaries and the standard library version.
* This gets parsed into `ARM_GCC_path`.
* Default value:  `"10.3"`

#### `"v_xPack_GCC"`

* Sets the version of the xPack RISC-V GCC to use.
* This gets parsed into `xPack_GCC_path`.
* Default value: `"10.2.0-1.2"`

#### `"OCD_path"`

* Where to find the OpenOCD.
* Default value: `"${config:MAXIM_PATH}/Tools/OpenOCD"`

#### `"ARM_GCC_path"`

* Where to find the Arm Embedded GCC Toolchain.
* Default value: `"${config:MAXIM_PATH}/Tools/GNUTools/${config:v_Arm_GCC}"`

#### `"xPack_GCC_path"`

* Where to find the RISC-V GCC Toolchain.
* Default value: `"${config:MAXIM_PATH}/Tools/xPack/riscv-none-embed-gcc/${config:v_xPack_GCC}"`

#### `"Make_path"`

* Where to find Make binaries (only used on Windows)
* Default value: `"${config:MAXIM_PATH}/Tools/MSYS2/usr/bin"`

#### `"C_Cpp.default.includePath"`

* Which paths to search to find header (.h) files.
* Does not recursively search by default.  To recursively search, use `/**`.

#### `"C_Cpp.default.browse.path"`

* Which paths to search to find source (.c) files.
* Does not recursively search by default.  To recursively search, use `/**`.

#### `"C_Cpp.default.defines"`

* Sets the compiler definitions to use for the intellisense engine.
* Most definitions should be defined in header files, but if a definition is missing it can be entered here to get the intellisense engine to recognize it.

### Setting Search Paths for Intellisense

VS Code's intellisense engine must be told where to find the header files for your source code.  By default, Maxim's perpiheral drivers, the C standard libraries, and all of the sub-directories of the workspace will be searched for header files to use with Intellisense.  If VS Code throws an error on an `#include` statement (and the file exists), then a search path is most likely missing.

To add additional search paths :

1. Open the `.vscode/settings.json` file.  

2. Add the include path(s) to the `C_Cpp.default.includePath` list.  The paths set here should contain header files, and will be searched by the Intellisense engine and when using "Go to Declaration" in the editor.

3. Add the path(s) to any relevant implementation files to the `C_Cpp.default.browse.path` list.  This list contains the paths that will be searched when using "Go to Definition".

## Build Configuration

A project's build system is managed by two files found in the project's root directory.  These files are used alongside the [GNU Make](https://www.gnu.org/software/make/) program (which is a part of the MaximSDK toolchain) to locate and build a project's source code for the correct microcontroller.

* `Makefile`
* `project.mk`

![Files are located in the root directory](img/projectmk.JPG)

When the command...

```shell
make
```

... is run, the program `make` will load settings from these two files.  Then, it will use them to build the project's source code.  VSCode-Maxim is a "wrapper" around this Makefile system.

The file named `Makefile` is the "core" file for the project.  It should not be edited directly.  Instead, it offers a number of configuration variables that can be overridden in the `project.mk` file, on the command-line, in your system's environment, or via your IDE.  It also comes with a default configuration that is suitable for most projects.

### Default Build Behavior

By default, the build system will auto-search the root project directory source code (`*.c`) and header files (`*.h`).  The optional "include" and "src" directories are also searched if they exist.

```shell
Root Project Directory
├─ project.mk
├─ Makefile
├─ *.h
├─ *.c
├─include  # <-- Optional
  └─ *.h
├─src      # <-- Optional
  └─ *.c
```

Additionally, the "core" `Makefile` will come pre-configured for a specific target microcontroller and Board Support Package (BSP).  The default BSP will match the main EVKIT for the device.  In VSCode-Maxim, the two [Basic Config Options](#basic-config-options) can be used to easily override the target microcontroller and BSP.  These options are passed to `make` on the command-line when the ["Build" task](#build-tasks) is run.

For more advanced build configuration, configuration variables should be used.

### How to Set a Configuration Variable

A configuration variable is a [Makefile variable](https://www.gnu.org/software/make/manual/make.html#Using-Variables), and therefore follows the same rules.  However, they have been streamlined to be made much easier to use, so most of the official GNU Make documentation is only needed for advanced use-cases.

To set a configuration variable, use the syntax...

```Makefile
VARIABLE=VALUE
```

The `=` operater is used for _most_ configuration variables with a few exceptions (that are clearly documented) when a variable should contain a _list_ of values.  In such cases, use the syntax...

```Makefile
VARIABLE+=VALUE1
VARIABLE+=VALUE2
```

... to _add_ values to the list.

In most cases, you should do this from inside of **project.mk**.  

For example, if I wanted to enable hardware floating-point acceleration for my project, I would use the `MFLOAT_ABI` configuration variable to set its value to `hard`.  The contents of **project.mk** might then look as follows:

(_Inside project.mk_)

```Makefile
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

#BOARD=FTHR_RevA
# ^ For example, you can uncomment this line to make the 
# project build for the "FTHR_RevA" board.

# **********************************************************

MFLOAT_ABI=hard # Enable hardware floating point acceleration
```

It should also be noted that configuration variables can be set on the **command-line** as well.  For example...

```shell
make MFLOAT_ABI=hard
```

... will have the same effect.

Additionally, **environment variables** can be used.  For example (on linux)...

```shell
export TARGET=MAX78000
```

... will set all projects to build for the MAX78000.

However, there is a precedence hierarchy that should be taken into consideration.

### Precedence Hierarchy

The precedence hierarchy for the value of a configuration variable is:

* **IDE/command-line > project.mk > environment variable > default value**

...meaning if a value is set on the command-line _and_ project.mk, the command-line value will take precedence.  However, the ["override" directive](https://www.gnu.org/software/make/manual/make.html#Override-Directive) can be used in project.mk to give it max precedence.

### Configuration Variables Table

The following configuration variables are available.

| Variable | Description | Example | Details |
|--- | --- | --- | ---|
**Target**
| `TARGET` | Set the target microcontroller | `TARGET=MAX78000` |
| `BOARD` | Set the Board Support Package (BSP) | `BOARD=FTHR_RevA` | Every microcontroller has a number of BSPs available for it that can be found in the `Libraries/Boards/TARGET` folder of the MaximSDK.  When you change this option, it's usually a good idea to fully clean your project, then re-build.
**SDK**
| `MAXIM_PATH` | (Optional) Specify the location of the MaximSDK | `MAXIM_PATH=/path/to/MSDK` | This optional variable can be used to change where the Makefile looks for the MaximSDK.  By default, the Makefile will attempt to locate the MaximSDK with a relative path moving "up" from its original location.  This option is most useful when a project is moved _outside_ of the SDK and you're developing on the command-line, since VS Code and Eclipse will set this via an environment variable.  It's also useful for re-targeting a project to point to the development repository.
| `CAMERA` | (Optional) Set the Camera drivers to use | `CAMERA=HM0360_MONO` | This option is only useful for the MAX78000 and MAX78002, and sets the camera drivers to use for the project.  Permitted values are `HM01B0`, `HM0360_MONO`, `HM0360_COLOR`, `OV5642`, `OV7692` (default), or `PAG7920`.  Camera drivers can be found in the `Libraries/MiscDrivers/Camera` folder of the MaximSDK.  Depending on the selected camera, a compiler definition may be added to the build. See the `board.mk` Makefile in the active BSP for more details.
**Source Code**
| `VPATH` | Where to search for source (.c) files | `VPATH+=your/source/path` | **Use the `+=` operator with this option**.  This controls where the Makefile will look for **source code** files.  If `AUTOSEARCH` is enabled (which it is by default) this controls which paths will be searched.  If `AUTOSEARCH` is disabled, this tells the Makefile where to look for the files specified by `SRCS`.
| `IPATH` | Where to search for header (.h) files | `IPATH+=your/include/path` | **Use the `+=` operator with this option**.  This controls where the Makefile will look for **header** files.  _Unlike_ the `VPATH` option, this is not related to `AUTOSEARCH`.  Individual header files are _not_ ever manually added into the build.  Instead, you only need to specify the _location_ of your header files.
| `AUTOSEARCH` | Automatically search for source (.c) files | `AUTOSEARCH=0` | Enable or disable the automatic detection of .c files on `VPATH` (enabled by default).  Set to `0` to disable, or `1` to enable.  If autosearch is disabled, source files must be manually added to `SRCS`.
| `SRCS` | List of source (.c) files to add to the build | `SRCS+=./my/other/source.c` | **Use the `+=` operator with this option**.  All of the files in this list will be added to the build.  If `AUTOSEARCH` is enabled, this is most useful for adding the full absolute path to a singular source file to selectively add to the build.  If `AUTOSEARCH` is disabled, _all_ of the source files for the project must be added to `SRCS`, and they must also all be located on an entry in `VPATH`.  Otherwise, a full path relative to the Makefile must be used.
| `PROJECT` | Set the output filename | `PROJECT=MyProject` | This controls the output filename of the build.  File extensions should _not_ be set here since the output file format may vary depending on the build recipe.  For VSCode-Maxim, you should use the [project_name](#project_name) advanced config option instead, which sets `PROJECT` on the command-line [Build Tasks](#build-tasks).
**Compiler**
| `MXC_OPTIMIZE_CFLAGS` | Set the optimization level | `MXC_OPTIMIZE_CFLAGS=-O2` | See [Optimize Options](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html) for more details.  Normal builds will default to `-Og`, which is good for debugging, while release builds will default to `-O2`.
| `PROJ_CFLAGS` | Add a compiler flag to the build | `PROJ_CFLAGS+=-Wextra`, `PROJ_CFLAGS+=-DMYDEFINE` | Compiler flags can be added with this option, including compiler definitions.  For each value, the same syntax should be used as if the compiler flag was passed in via the command-line.  These can include standard [GCC options](https://gcc.gnu.org/onlinedocs/gcc-10.4.0/gcc/Option-Summary.html#Option-Summary) and/or [ARM-specific](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) options.
| `MFLOAT_ABI` | Set the floating point acceleration level | `MFLOAT_ABI=hard` | Sets the floating-point acceleration level.  Permitted values are `hard`, `soft`, `softfp` (default).  To enable full hardware acceleration instructions use `hard`, but keep in mind that _all_ libraries your source code uses must also be compiled with `hard`.  If there is any conflict, you'll get a linker error.  For more details, see `-mfloat-abi` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html).
**Linker**
| `LINKERFILE` | Set the linkerfile to use | `LINKERFILE=newlinker.ld` | You can use a different linkerfile with this option.  The file should exists in `Libraries/CMSIS/Device/Maxim/TARGET/Source/GCC` in the MaximSDK, or it should be placed inside the root directory of the project.
**Libraries**
| `LIB_BOARD` | Include the BSP library (enabled by default) | `LIB_BOARD=0` | Inclusion of the Board-Support Package (BSP) library, which is enabled by default, can be toggled with this variable.  This library contains important startup code specific to a microcontroller's evaluation platform, such as serial port initialization, power sequencing, external peripheral initalization, etc.  Set to `0` to disable, or `1` to enable.
| `LIB_PERIPHDRIVERS` | Include the peripheral driver library (enabled by default) | `LIB_PERIPHDRIVERS=0` | The peripheral driver library can be toggled with this option.  If disabled, you'll lose access to the higher-level driver functions but still have access to the register-level files.  Set to `0` to disable, or `1` to enable.
| `LIB_CMSIS_DSP` | Include the CMSIS-DSP library | `LIB_CMSIS_DSP=1` | The [CMSIS-DSP library](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html) can be enabled with this option.  Set to `0` to disable, or `1` to enable.
| `LIB_CORDIO` | Include the Cordio library | `LIB_CORDIO=1` | The Cordio BLE library can be included with this option.  This is only applicable towards microcontrollers with an integrated BLE controller.
| `LIB_FCL` | Include the Free Cryptographic Library (FCL) | `LIB_FCL=1` | This option toggles the Free Cryptographic Library (FCL), which is a collection of software-implemented common cryptographic functions can be included with this option.  Set to `0` to disable, or `1` to enable.
| `LIB_FREERTOS` | Include the FreeRTOS library | `LIB_FREERTOS=1` | The [FreeRTOS](https://freertos.org/) library can be enabled with this option, which is an open-source Real-Time Operating System (RTOS).  Set to `0` to disable, or `1` to enable.
| `LIB_LC3` | Include the LC3 codec library | `LIB_LC3=1` | This option enables the inclusion of the Low Complexity Communication Codec (LC3), which is an efficient low latency audio codec.  Set to `0` to disable, or `1` to enable.
| `LIB_LITTLEFS` | Include the littleFS library | `LIB_LITTLEFS=1` | This option toggles the ["Little File System"](https://github.com/littlefs-project/littlefs) library - a small filesystem library designed for microcontrollers.  Set to `0` to disable, or `1` to enable.
| `LIB_LWIP` | Include the lwIP library | `LIB_LWIP=1` | |
| `LIB_MAXUSB` | Include the MaxUSB library | `LIB_MAXUSB=1` | This option toggles the inclusion of the MAXUSB library, which facilitates the use of the native USB peripherals on some microcontrollers.  Set to `0` to disable, or `1` to enable.
| `LIB_SDHC` | Include the SDHC library | `LIB_SDHC=1` | This options toggles the Secure Digital High Capacity (SDHC) library, which can be used to interface with SD cards.  Additionally, it enables the [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) library, which implements a generic FAT filesystem.
**Secure Boot Tools (SBT)**
| `SBT` | Toggle SBT integration | `SBT=1` | Toggles integration with the [Secure Boot Tools (SBTs)](https://www.maximintegrated.com/en/design/technical-documents/userguides-and-manuals/7/7637.html).  These are a suite of applications designed for use with microcontrollers that have secure bootloaders.  When this is enabled, some additional rules become available such as `make sla` and `make scpa`.  Set to `0` to disable or `1` to enable.
| `MAXIM_SBT_DIR` | Where to find the SBTs | `MAXIM_SBT_DIR=C:/MaximSBT` | This option can be used to manually specify the location of the SBTs.  Usually, this is not necessary.  By default, the `Tools/SBT` directory of the MaximSDK will be searched.  If the [SBT installer](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0015360C) is used, it will set the `MAXIM_SBT_DIR` environment variable to point to itself automatically.
| `TARGET_SEC` | Secure part number to use | `TARGET_SEC=MAX32651` | Some secure microcontrollers have multiple secure variants, and this option can be used to specify the variant to use with the SBTs.  Defaults are intelligently selected, and can be found in `$(MAXIM_SBT_DIR)/SBT-config.mk`
| `SCP_PACKETS` | Where to build the scp_packets folder | | Defaults to `build/scp_packets` |
| `TEST_KEY` | Which test key to sign applications with | | Defaults to `$(MAXIM_SBT_DIR)/devices/$(TARGET_SEC)/keys/maximtestcrk.key`, which is the Maxim test key that can be used for development.

## Project Creation

### Option 1.  Copying a Pre-Made Project

Copying a pre-made example project is a great way to get rolling quickly, and is currently the recommended method for creating new projects.  

The release package for this project (Located at Tools/VSCode-Maxim in the MaximSDK) contains a `New_Project` folder designed for such purposes.  Additionally, any of the VS Code-enabled Example projects can be copied from the SDK.

1. Copy the existing project folder to an accessible location.  This will be the location of your new project.

2. (Optional) Rename the folder.  For example, I might rename the folder to `MyProject`.

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your target microcontroller and board correctly.  See [Basic Config Options](#basic-config-options)

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. That's it!  The existing project is ready to build, debug, and modify.

### Option 2 - Creating a Project from Scratch

If you want to start from scratch, take this option.

1. Create your project folder.  For example, I might create a new project in a workspace folder with the path: `C:\Users\Jake.Carter\workspace\MyNewProject`.

2. Copy the **contents** of the `Inject` folder into the project folder created in step 2.  This includes a `.vscode` folder and a `Makefile`.  In the example above, the contents of the 'MyProject' folder would be the following :

    ```shell
    C:\Users\Jake.Carter\workspace\MyNewProject
    +-- \.vscode
    +-- Makefile
    ```

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your target microcontroller correctly.  See [Basic Config Options](#basic-config-options).

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. Fundamentally, that's it.  Your new empty project can now be opened with `File > Open Folder` from within VS Code.

## Issue Tracker

Bug reports, feature requests, and contributions are welcome via the [issues](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim/issues) tracker on Github.

New issues should contain _at minimum_ the following information:

* Visual Studio Code version #s (see `Help -> About`)
* C/C++ Extension version #
* Target microcontroller and evaluation platform
* The projects `.vscode` folder and `Makefile` (where applicable).  Standard compression formats such as `.zip`, `.rar`, `.tar.gz`, etc. are all acceptable.
