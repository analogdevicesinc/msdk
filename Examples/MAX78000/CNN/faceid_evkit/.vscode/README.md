# VSCode-Maxim

_(If you're viewing this document from within Visual Studio Code you can press `CTRL+SHIFT+V` to open a Markdown preview window.)_

## Quick Links

* [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)
  * If it's not in the README, check the UG.
  * If it's not in the UG, open a ticket!
* [VSCode-Maxim Github](https://github.com/Analog-Devices-MSDK/VSCode-Maxim)

## Introduction

VSCode-Maxim is a set of [Visual Studio Code](https://code.visualstudio.com/) project configurations and utilities for enabling embedded development for [Analog Device's MSDK](https://github.com/Analog-Devices-MSDK/msdk) and the [MAX32xxx/MAX78xxx microcontrollers](https://www.analog.com/en/product-category/microcontrollers.html).

The following features are supported:

* Code editing with intellisense down to the register level
* Code compilation with the ability to easily re-target a project for different microcontrollers and boards
* Flashing programs
* GUI and command-line debugging

## Dependencies

* [Visual Studio Code](https://code.visualstudio.com/)
  * [C/C++ VSCode Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
  * [Cortex-Debug Extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)
* [Analog Devices MSDK](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0010820A)

## Installation

The steps below are also available in video form in "Understanding Artificial Intelligence Episode 8.5 - Visual Studio Code" [here](https://www.analog.com/en/education/education-library/videos/6313212752112.html).

1. Download and install the Analog Devices MSDK for your OS from the links below.  For more detailed instructions on installing the MSDK, see the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)
    * [Windows](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0010820A)
    * [Linux (Ubuntu)](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018720A)
    * [MacOS](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018610A)

2. Run the installer executable, and ensure that "Visual Studio Code Support" is enabled for your installation.

    ![Selected Components](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/installer_components.JPG)

3. Finish the MSDK installation, taking note of where the MSDK was installed.

4. Download & install Visual Studio Code for your OS [here](https://code.visualstudio.com/Download).

5. Launch Visual Studio Code.

6. Install the Microsoft [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).

7. Install the [Cortex-Debug Extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

8. Use `CTRL + SHIFT + P` (or `COMMAND + SHIFT + P` on MacOS) to open the developer prompt.

9. Type "open settings json" and select the "Preferences: Open Settings (JSON)" option (_not_ the "Preferences: Open _Default_ Settings (JSON)").  This will open your user settings.json file in VS Code's editor.

    ![Open Settings JSON Command](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/open_settings_json.jpg)

10. Add the entries below into your user settings.json file.

    ```json
    {
        // There may be other settings up here...
        
        "MAXIM_PATH":"C:/MaximSDK", // Set this to the installed location of the Analog Devices MSDK. Only use forward slashes '/' when setting this path!
        "update.mode": "manual",
        "extensions.autoUpdate": false,
        
        // There may be other settings down here...
    }
    ```

11. Save your changes to the file with `CTRL + S` and restart VS Code.

12. That's it! You're ready to start using Visual Studio Code to develop with Analog Devices MAX-series Microcontrollers. The MSDK examples come pre-populated with .vscode project folders, and the `Tools/VSCode-Maxim` folder of the MSDK contains documentation and templates.  See [Usage](#usage) below for more details.

## Usage

This section covers basic usage of the VSCode-Maxim project files.  For documentation on Visual Studio Code itself, please refer to the official docs [here](https://code.visualstudio.com/Docs).

### Opening Projects

Visual Studio Code is built around a "working directory" paradigm.  The editor is always rooted in a working directory, and the main mechanism for changing that directory is `File -> Open Folder...`.

![File -> Open Folder](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/file_openfolder.JPG)

As a result, you'll notice that there is no "New Project" mechanism.  A "project" in VS Code is simply a folder.  It will look inside of the opened folder for a `.vscode` _sub_-folder to load project-specific settings from.

A project that is configured for VS Code will have, at minimum, a .vscode sub-folder and a Makefile in its directory _(Note: You may need to enable viewing of hidden items in your file explorer to see the .vscode sub-folder)_.  

Ex:

![Example Directory Contents](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/opening_projects_2.jpg)

### Where to Find Projects

The [Examples](https://github.com/Analog-Devices-MSDK/msdk/tree/main/Examples) in the MSDK come with with pre-configured .vscode project folders.  These projects can be opened "out of the box", but it's good practice to copy example folders _outside_ of the MSDK so that the original copies are kept as clean references.  The examples can be freely moved to any location _without a space in its path_.

Additionally, empty project templates and a drag-and-drop folder for "injecting" a VSCode-Maxim project can be found under `Tools/VSCode-Maxim` in the MSDK installation.

### Build Tasks

Once a project is opened 4 available build tasks will become available via `Terminal > Run Build task...` or the shortcut `Ctrl+Shift+B`.  These tasks are configured by the `.vscode/task.json` file.

![Build Tasks Image](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/buildtasks.JPG)

#### Build

* Compiles the code with a `make all` command.
* Additional options are passed into Make on the command-line based on the project's settings.json file.
* The `./build` directory will be created and will contain the output binary, as well as all intermediary object files.

#### Clean

* Cleans the build output, removing the `./build` directory and all of its contents.

#### Clean-Periph

* This task is the same as 'clean', but it also removes the build output for the MSDK's peripheral drivers.
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

![Debug Window](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/debugger.JPG)

Debugging is enabled by Visual Studio Code's integrated debugger.  Launch configurations can be found in the `.vscode/launch.json` file.

* Note: **Flashing does not happen automatically when launching the debugger.**  Run the "Flash" [build task](#build-tasks) for your program _before_ debugging.

#### Debugger Limitations

In general, the MAX-series microcontrollers have the following debugger limitations at the hardware level:

* The debugger can not be connected _while_ the device is in reset.

* The device can not be debugged while the device is in Sleep, Low Power Mode, Micro Power Mode, Standby, Backup, or Shutdown mode.  These modes shut down the SWD clock.

* These limitations can sometimes make the device difficult or impossible to connect to if firmware has locked out the debugger.  In such cases, the ["Erase Flash"](#erase-flash) task can be used to recover the part.

#### Launching the Debugger

1. Attach your debugger to the SWD port on the target microcontroller.  (Refer to the datasheet of your evaluation board for instructions on connecting a debugger)

2. Flash the program to the microcontroller with the "Flash" [Build Task](#build-tasks).  **Flashing does not happen automatically when launching the debugger.**

3. Launch the debugger with `Run > Start Debugging`, with the shortcut `F5`, or via the `Run and Debug` window (Ctrl + Shift + D) and the green "launch" arrow.  

    ![Debug Tab](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/debugger_window.JPG)

4. The debugger will launch a GDB client & OpenOCD server, reset the microcontroller, and should break on entry into `main`.

    ![Debugger Break on Main](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/debugger_breakmain.JPG)

#### Using the Debugger

* For full usage details, please refer to the [official VS Code debugger documentation](https://code.visualstudio.com/docs/editor/debugging).

The main interface for the debugger is the debugger control bar:

![Debugger Control Bar Image](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/debugger_bar.JPG)

`Continue | Step Over | Step Into | Step Out | Restart | Stop`

Breakpoints can be set by clicking in the space next to the line number in a source code file.  A red dot indicates a line to break on.  Breakpoints can be removed by clicking on them again.  Ex:

![Breakpoint](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/breakpoint.JPG)

## Project Configuration

### Project Settings

`.vscode/settings.json` is the main project configuration file.  Values set here are parsed into the other .json config files.  

**When a change is made to this file, VS Code should be reloaded with CTRL+SHIFT+P -> Reload Window (or alternatively restarted completely) to force a re-parse.**

![Reload Window](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/reload_window.JPG)

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
  * ... can be found in the `Libraries/Boards` folder of the MSDK
  * For example, the supported options for the MAX78000 are `"EvKit_V1"`, `"FTHR_RevA"`, and `"MAXREFDES178"`.

  ![MAX78000 Boards](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/main/img/78000_boards.JPG)

### Advanced Config Options

#### `"MAXIM_PATH"`

* This option must point to the root installation directory of the MSDK.  
* It should be placed in the _global_ user settings.json file during first-time VSCode-Maxim setup.  See [Installation](#installation).

#### `"terminal.integrated.env.[platform]:Path"`

* This prepends the location of the MSDK toolchain binaries to the system `Path` used by VSCode's integrated terminal.
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

VS Code's intellisense engine must be told where to find the header files for your source code.  By default, the MSDK's peripheral drivers, the C standard libraries, and all of the sub-directories of the workspace will be searched for header files to use with Intellisense.  If VS Code throws an error on an `#include` statement (and the file exists), then a search path is most likely missing.

To add additional search paths :

1. Open the `.vscode/settings.json` file.  

2. Add the include path(s) to the `C_Cpp.default.includePath` list.  The paths set here should contain header files, and will be searched by the Intellisense engine and when using "Go to Declaration" in the editor.

3. Add the path(s) to any relevant implementation files to the `C_Cpp.default.browse.path` list.  This list contains the paths that will be searched when using "Go to Definition".

## Build Configuration

A project's build system is managed by two files found in the project's root directory.  These files are used alongside the [GNU Make](https://www.gnu.org/software/make/) program (which is a part of the MSDK toolchain) to locate and build a project's source code for the correct microcontroller.

* `Makefile`
* `project.mk`

![Files are located in the root directory](https://raw.githubusercontent.com/Analog-Devices-MSDK/VSCode-Maxim/65af7c61800c7039956f3c1971ffd7915008668d/img/projectmk.JPG)

When the command...

```shell
make
```

... is run, the program `make` will load settings from these two files.  Then, it will use them to build the project's source code.  VSCode-Maxim is a "wrapper" around this Makefile system.

**See the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system) for full documentation on how to configure the build system.**

## Project Creation

### Option 1.  Copying a Pre-Made Project

Copying a pre-made example project is a great way to get rolling quickly, and is currently the recommended method for creating new projects.  

The release package for this project (Located at Tools/VSCode-Maxim in the Analog Devices MSDK) contains a `New_Project` folder designed for such purposes. Additionally, any of the VS Code-enabled Example projects can be copied from the MSDK.

1. Copy the existing project folder to an accessible location.  This will be the location of your new project.

2. (Optional) Rename the folder.  For example, I might rename the folder to `MyProject`.

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your target microcontroller and board correctly.  See [Basic Config Options](#basic-config-options)

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. That's it!  The existing project is ready to build, debug, and modify.

### Option 2 - Injecting

VSCode-Maxim releases provide the `Inject` folder for "injecting" into an existing folder.  If you want to start from scratch or use the project files with existing source code, take this option.

1. Create your project folder if necessary.  For example, I might create a new project in a workspace folder with the path: `C:\Users\Jake.Carter\workspace\MyNewProject`.

2. Copy the **contents** of the `Inject` folder into the project folder from step 1.  The contents to copy include a `.vscode` folder, a `Makefile`, and a `project.mk` file.  For this example, the contents of the 'MyProject' folder would be the following:

    ```shell
    C:\Users\Jake.Carter\workspace\MyNewProject
    |- .vscode
    |- Makefile
    |- project.mk
    ```

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your target microcontroller correctly.  See [Basic Config Options](#basic-config-options).

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. Configure the [build system](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system) for use with any pre-existing source code.

7. That's it!  Your new empty project can now be opened with `File > Open Folder` from within VS Code.

## Issue Tracker

Bug reports, feature requests, and contributions are welcome via the [issues](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/issues) tracker on Github.

New issues should contain _at minimum_ the following information:

* Visual Studio Code version #s (see `Help -> About`)
* C/C++ Extension version #
* Target microcontroller and evaluation platform
* The projects `.vscode` folder and `Makefile` (where applicable).  Standard compression formats such as `.zip`, `.rar`, `.tar.gz`, etc. are all acceptable.
