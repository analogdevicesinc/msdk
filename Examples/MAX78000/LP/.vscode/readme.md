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

* Build
  * Compiles the code with a `make all` command.
  * Additional options are passed into Make on the command-line based on the project's settings.json file.
  * The `./build` directory will be created and will contain the output binary, as well as all intermediary object files.

* Clean
  * Cleans the build output, removing the `./build` directory and all of its contents.

* Clean-Periph
  * This task is the same as 'clean', but it also removes the build output for Maxim's peripheral drivers.
  * Use this if you would like to recompile the peripheral drivers from source on the next build.

* Flash
  * Launching this task automatically runs the `Build` task first.  Then, it flashes the output binary to the microcontroller.
  * It uses the GDB `load` and `compare-sections` commands, and handles launching an OpenOCD internally via a pipe connection.
  * The flashed program will be halted until the microcontroller is reset, power cycled, or a debugger is connected.
  * A debugger must be connected correctly to use this task.  Refer to the datasheet of your microcontroller's evaluation board for instructions.
  
* Flash & Run
  * This is the same as the `Flash` task, but it also will launch execution of the program once flashing is complete.
  
* Erase Flash
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

* These limitations can sometimes make the device difficult or impossible to connect to if firmware has locked out the debugger.  In such cases, the "Erase Flash" task can be used to recover the part.

#### Launching the Debugger

1. Attach your debugger to the SWD port on the target microcontroller.  (Refer to the datasheet of your evaluation board for instructions on connecting a debugger)

2. Flash the program to the microcontroller with the "Flash" [Build Task](#Build-Tasks).  **Flashing does not happen automatically when launching the debugger.**

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

## Configuration

### Project Settings

`.vscode/settings.json` is the main project configuration file.  Values set here are parsed into the other .json config files.  

**When a change is made to this file, VS Code should be reloaded with CTRL+SHIFT+P -> Reload Window (or alternatively restarted completely) to force a re-parse.**

![Reload Window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/reload_window.JPG)

The default project configuration should work for most use cases as long as `"target"` and `"board"` are set correctly.

Any field from `settings.json` can be referenced from any other config file (including itself) with `"${config:[fieldname]}"`

The following configuration options are available:

### Basic Config Options

* `"target"`
  * This sets the target microcontroller for the project.
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

* `"board"`
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

### Editing the Makefile

At the heart of every project is its `Makefile`.  Build Tasks are essentially a wrapper around the Makefile.  Adding source code files to the build, setting compiler flags, linking libraries, etc. must be done by directly editing this file.

The usage guidelines below are specific to Maxim's Makefiles.  The [GNU Make Manual](https://www.gnu.org/software/make/manual/html_node/index.html) is a good one to have on hand for documentation regarding Makefiles in general.

#### Adding Source Code Files

* VS Code's editor can create and add new files to a project, but they won't be added to the build automatically.  The Makefile must be told which source code files to build, and where to find them.
* Add a source file to the build with `SRCS += yourfile.c`
* The Makefile looks for project source files in the `/src` directory by default.  Add additional directories to search with `VPATH += yoursourcedirectory`
* The Makefile looks for project header files in the `/src` directory by default.  Add additional directories to search with `IPATH += yourincludedirectory`

#### Compiler Flags

* Compiler flags can be added/changed via the `PROJ_CFLAGS` variable.
* Add a new flag to be passed to the compiler with `PROJ_CFLAGS += -yourflag`.  Flags are passed in the order that they are added to the `PROJ_CFLAGS` variable.

#### Linking Libraries

* Additional libraries can be linked via the `PROJ_LIBS` variable.  Add a new library to the build with `PROJ_LIBS += yourlibraryname`.
  * Note : Do not include the 'lib' part of the library name, or the file extension.  For example, to link `libarm_cortexM4lf_math.a` set `PROJ_LIBS += arm_cortexM4lf_math`.
* Tell the linker where to find the library with the '-L' linker flag.  Set `PROJ_LDFLAGS += -Lpathtoyourlibrary`.  For example, set `PROJ_LDFLAGS += -L./lib` to search a 'lib' directory inside of the project for libraries.

#### Optimization Level

* The optimization level that the compiler uses can be set by changing the `MXC_OPTIMIZE_CFLAGS` variable.  
* See [GCC Optimization Options](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html) for more details on available optimization levels.  For example, disable optimization with `MXC_OPTIMIZE_CFLAGS = -O0`

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

6. Fundamentally, that's it.  Your new empty project can now be opened with `File > Open Folder` from within VS Code.  However, you'll probably want to add some source code.  See [Configuring the Makefile](#configuring-the-makefile).

## Issue Tracker

Bug reports, feature requests, and contributions are welcome via the [issues](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim/issues) tracker on Github.

New issues should contain _at minimum_ the following information:

* Visual Studio Code version #s (see `Help -> About`)
* C/C++ Extension version #
* Target microcontroller and evaluation platform
* The projects `.vscode` folder and `Makefile` (where applicable).  Standard compression formats such as `.zip`, `.rar`, `.tar.gz`, etc. are all acceptable.
