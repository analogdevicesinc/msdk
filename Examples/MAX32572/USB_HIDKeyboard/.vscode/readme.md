# VSCode-Maxim

[VSCode-Maxim on Github](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim)

If you are viewing this document from within Visual Studio Code, press `CTRL+SHIFT+V` to open a Markdown preview window.

## Introduction

This repository is dedicated to maintaining [Visual Studio Code](https://code.visualstudio.com/) project files that integrate with [Maxim Integrated's](https://www.maximintegrated.com/en/products/microcontrollers.html) Microcontroller SDK.  The following features are enabled by the project files:

* Code editing with intellisense and definition look-ups down to the register level
* Code compilation with the ability to easily re-target a project for different microcontrollers and boards
* Flashing program binaries
* GUI and command-line debugging

## Dependencies

The project folders in this repo have the following dependencies:

* [Visual Studio Code](https://code.visualstudio.com/)
* [C/C++ VSCode Extension](https://github.com/microsoft/vscode-cpptools)
* [Maxim Micros SDK](https://www.maximintegrated.com/content/maximintegrated/en/design/software-description.html/swpart=SFW0010820A)

## Installation

1. Download & install the Maxim Microcontrollers SDK for your OS from the links below.
    * [Windows](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0010820A)
    * [Linux (Ubuntu)](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018720A)
    * [MacOS](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018610A)

2. Run the installer executable.

3. During component selection, ensure that "Visual Studio Code Support" is selected.

    ![Selected Components](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/installer_components.JPG)

4. Finish the MaximSDK installation, and proceed to step 5 below to set up Visual Studio Code.

5. Download & install Visual Studio Code for your OS [here](https://code.visualstudio.com/Download).

6. Launch Visual Studio Code.

7. Install the Microsoft [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).

8. Use `CTRL + SHIFT + P` (or `COMMAND + SHIFT + P` on MacOS) to open the developer prompt.

9. Type "open settings json" and select the "Preferences: Open Settings (JSON)" option (_not_ the "Preferences: Open _Default_ Settings (JSON)").  This will open your user settings.json file in VS Code's editor.

    ![Open Settings JSON Command](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/open_settings_json.jpg)

10. Add the entries below into your user settings.json file.

    Note:  **If you installed the MaximSDK to a non-default location change the value of `"MAXIM_PATH"`.  Only use forward slashes `/` when setting `"MAXIM_PATH"`.**

    ```json
    {
        // There may be other settings up here...
        
        "MAXIM_PATH":"C:/MaximSDK", // Only use forward slahes '/' when setting this path!
        "update.mode": "manual",
        "extensions.autoUpdate": false,
        
        // and/or other settings down here...
    }
    ```

11. Save your changes to the file with `CTRL + S` and restart VS Code.

12. That's it!  You're ready to start using Visual Studio Code to develop with Maxim's Microcontrollers.  The MaximSDK examples come pre-populated with .vscode project folders, and the `Tools/VSCode-Maxim` folder of the SDK contains documentation and templates.  See [Usage](#usage) below for more details.

## Usage

This section covers basic usage of the VSCode-Maxim project files.  For documentation on Visual Studio Code itself, please refer to the official docs [here](https://code.visualstudio.com/Docs).  

Prior experience with Visual Studio Code is not required to understand this section or use the project files, but some basic familiarity is helpful.  For new users, this initial familiarity can be gained by working through the full [User Guide](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim/blob/main/userguide.md).

### Opening Projects

Visual Studio Code is built around a "working directory" paradigm.  VS Code's editor is always running from inside of a working directory, and the main mechanism for changing that directory is `File -> Open Folder...`  

![File -> Open Folder](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/file_openfolder.JPG)

VS Code will look in the opened folder for a `.vscode` _sub_-folder to load project-specific settings from.

Opening an existing project is as simple as `File -> Open Folder...`.  A project that is configured for VS Code will have, at minimum, a .vscode sub-folder and a Makefile in its directory.  Ex:

![Example Directory Contents](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/opening_projects_2.jpg)

Note:  You may need to enable viewing of hidden items in your file explorer to see the .vscode sub-folder.

### Build Tasks

Once a project is opened 4 available build tasks will become available via `Terminal > Run Build task...` or the shortcut `Ctrl+Shift+B`.  These tasks are configured by the `.vscode/task.json` file.

![Build Tasks Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/buildtasks.JPG)

* Build
  * Compiles the code.
  * The `./build` directory will be created and will contain the output binary, as well as all intermediary object files.

* Clean
  * This task cleans the build output, removing the `./build` directory and all of its contents.

* Clean-Periph
  * This task is the same as 'clean', but it also removes the build output for Maxim's peripheral drivers.
  * Use this if you would like to recompile the peripheral drivers from source on the next build.

* Flash
  * This task runs the Build task, and then flashes the output binary to the microcontroller.
  * A debugger must be connected to the correct debugger port on the target microcontroller.  Refer to the datasheet of your microcontrollers evaluation board for instructions on connecting a debugger.

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

### Debugging

Debugging is enabled by Visual Studio Code's integrated debugger.  Launch configurations are provided by the `.vscode/launch.json` file.

* Note: **Flashing does not happen automatically when launching the debugger.**  Run the "Flash" [build task](#build-tasks) for your program before debugging.

![Debug Window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger.JPG)

#### Debugger Limitations

In general, Maxim's microcontrollers have the following debugger limitations at the hardware level:

* The debugger can not be connected _while_ the device is in reset.

* The device can not be debugged while the device is in Sleep, Low Power Mode, Micro Power Mode, Standby, Backup, or Shutdown mode.  These modes shut down the SWD clock.

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

#### RISC-V Debugging

For microcontrollers with both an Arm M4 and a RISC-V core, the "GDB (RISC-V)" launch profile is provided to enable RISC-V debugging.  The RISC-V core requires setup and handoff from the Arm M4 core.  As a result, this is an advanced configuration requiring a unique combination of the project's source code, Makefiles, and VSCode-Maxim project settings.  Such projects are appended with the `"-riscv"` suffix in the project's folder name.

To debug a RISC-V project:

1. Connect your Arm (SWD) and RISC-V (JTAG) debuggers.  The RISC-V projects come pre-configured for the [ARM-USB-OCD-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-OCD-H/) + [ARM-JTAG-20-10](https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-20-10/) adapter.

2. Make sure your Olimex debugger drivers are installed correctly.  Sometimes they need to be updated using the "zadig" tool.  See [this](https://www.olimex.com/Products/ARM/JTAG/_resources/ARM-USB-OCD_and_OCD_H_manual.pdf) Olimex User Manual for more details.

3. Run the "Flash" task.  Ex:

    ![image](https://user-images.githubusercontent.com/38844790/168398354-2ac2961b-6d45-4f84-8805-0ab5339a4b98.png)

4. Launch the debugger using the GDB (Arm M4) profile first:

    ![image](https://user-images.githubusercontent.com/38844790/168398415-147a3a96-1a7d-4057-8a32-0dfaf2d378c1.png)

    ... which should hit the breakpoint in `main.c`...
    ![image](https://user-images.githubusercontent.com/38844790/168398503-0f2ae9c1-f535-4d41-aed9-9d9e19b16303.png)

5. Continue the debugger.  The code in `main.c` will boot up the RISC-V core.  You can optionally set a breakpoint on `WakeISR` to see when the RISC-V core has signaled it's ready.

    ![image](https://user-images.githubusercontent.com/38844790/168398665-9486e1b6-73bd-481e-a4b5-15dd44c7d7b9.png)

6. Now, launch another debugger window with the GDB (RISC-V) profile.

    ![image](https://user-images.githubusercontent.com/38844790/168398707-b6771bf3-b6bf-47a2-b963-b0b9fc003ca4.png)

    ... which should hit the breakpoint on main.  

    Notice the "Signal 0" exception below...  This is a known issue caused by a reset hardware bug on the RISC-V core that can be safely ignored.  The exception message is harmless, but annoying...  It will present itself every time the debugger is paused.

    ![image](https://user-images.githubusercontent.com/38844790/168399130-95fe7539-fb46-4c06-a268-6b720403b539.png)

7. From here, the debugger should be fully functional.  Ex, stepping through loading CNN weights on the MAX78000 RISC-V core:

    ![image](https://user-images.githubusercontent.com/38844790/168399419-d0488a0e-2068-4cc7-9108-0a296fdc04b4.png)

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
    * `"MAX32665"` (for MAX32665-MAX32668)
    * `"MAX32670"`
    * `"MAX32672"`
    * `"MAX32675"`
    * `"MAX78000"`

* `"board"`
  * This sets the target board for the project (ie. Evaluation Kit, Feather board, etc.)
  * Supported values:
    * ... can be found in the `Libraries/Boards` folder of the MaximSDK
    * For example, the supported options for the MAX78000 are `"EvKit_V1"`, `"FTHR_RevA"`, and `"MAXREFDES178"`.

    ![MAX78000 Boards](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/78000_boards.JPG)

### Advanced Config Options

* `"MAXIM_PATH"`
  * This option must point to the root installation directory of the MaximSDK.  
  * It should be placed in the _global_ user settings.json file during first-time VSCode-Maxim setup.  See [Installation](#installation).

* `"terminal.integrated.env.[platform]:Path"`
  * This prepends the location of toolchain binaries to the system `Path` used by VSCode's integrated terminal.
  * The Path is not sanitized by default, which means that the terminal inherits the system path.
  * Don't touch unless you know what you're doing :)

* `"project_name"`
  * Sets the name of project.  This is used in other config options such as `program_file`.
  * Default value: `"${workspaceFolderBasename}"`

* `"program_file"`
  * Sets the name of the file to flash and debug.  This is provided in case it's needed, but for most use cases should be left at its default.  
  * File extension must be included.
  * Default value: `"${config:project_name}.elf"`

* `"symbol_file"`
  * Sets the name of the file that GDB will load debug symbols from.
  * File extension must be included.
  * Default value: `"${config:program_file}"`

* `"M4_OCD_interface_file"`
  * Sets the OpenOCD interface file to use to connect to the Arm M4 core.  This should match the debugger being used for the M4 core.
  * The `MaximSDK/Tools/OpenOCD/scripts/interface` folder is searched for the file specified by this setting.
  * `.cfg` file extension must be included.
  * Default value: `"cmsis-dap.cfg"`

* `"M4_OCD_target_file"`
  * Sets the OpenOCD target file to use for the Arm M4 core.  This should match the target microcontroller.
  * `.cfg` file extension must be included.
  * The `MaximSDK/Tools/OpenOCD/scripts/target` folder is searched for the file specified by this setting.
  * Default value: `"${config:target}.cfg"`

* `"RV_OCD_interface_file"`
  * Sets the OpenOCD interface file to use to connect to the RISC-V core.  This should match the debugger being used for the RISC-V core.
  * The `MaximSDK/Tools/OpenOCD/scripts/interface` folder is searched for the file specified by this setting.
  * `.cfg` file extension must be included.
  * Default value: `"ftdi/olimex-arm-usb-ocd-h.cfg"`

* `"RV_OCD_target_file"`
  * Sets the OpenOCD target file to use for the RISC-V core.
  * The `MaximSDK/Tools/OpenOCD/scripts/target` folder is searched for the file specified by this setting.
  * `.cfg` file extension must be included.
  * Default value: `"${config:target}_riscv.cfg"`

* `"v_Arm_GCC"`
  * Sets the version of the Arm Embedded GCC to use, including toolchain binaries and the standard library version.
  * This gets parsed into `ARM_GCC_path`.
  * Default value:  `"10.3"`

* `"v_xPack_GCC"`
  * Sets the version of the xPack RISC-V GCC to use.
  * This gets parsed into `xPack_GCC_path`.
  * Default value: `"10.2.0-1.2"`

* `"OCD_path"`
  * Where to find the OpenOCD.
  * Default value: `"${config:MAXIM_PATH}/Tools/OpenOCD"`

* `"ARM_GCC_path"`
  * Where to find the Arm Embedded GCC Toolchain.
  * Default value: `"${config:MAXIM_PATH}/Tools/GNUTools/${config:v_Arm_GCC}"`

* `"xPack_GCC_path"`
  * Where to find the RISC-V GCC Toolchain.
  * Default value: `"${config:MAXIM_PATH}/Tools/xPack/riscv-none-embed-gcc/${config:v_xPack_GCC}"`

* `"Make_path"`
  * Where to find Make binaries (only used on Windows)
  * Default value: `"${config:MAXIM_PATH}/Tools/MSYS2/usr/bin"`

### Setting Search Paths for Intellisense

VS Code's intellisense engine must be told where to find the header files for your source code.  By default, Maxim's perpiheral drivers, the C standard libraries, and all of the sub-directories of the workspace will be searched for header files to use with Intellisense.  If VS Code throws an error on an `#include` statement (and the file exists), then a search path is most likely missing.

To add additional search paths :

1. Open the `.vscode/settings.json` file.  

2. Add the include path(s) to the `C_Cpp.default.includePath` list.  The paths set here should contain header files, and will be searched by the Intellisense engine and when using "Go to Declaration" in the editor.

3. Add the path(s) to any relevant implementation files to the `C_Cpp.default.browse.path` list.  This list contains the paths that will be searched when using "Go to Definition".

## Die Types to Part Numbers

The MaximSDK's peripheral driver filenames are written using die types instead of external part numbers.  This table shows which part numbers correspond to each die type, which is useful when browsing through source file definitions in Maxim's peripheral drivers.

| Die Type | Part Number |
| -------- | ----------- |
| ES17 | MAX32520 |
| ME10 | MAX32650 |
| ME11 | MAX32660 |
| ME13 | MAX32570 |
| ME14 | MAX32665 |
| ME15 | MAX32670 |
| ME16 | MAX32675 |
| ME17 | MAX32655 |
| ME18 | MAX32690 |
| ME21 | MAX32672 |
| AI85 | MAX78000 |
| AI87 | MAX78002 |

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

## Troubleshooting

Before troubleshooting, ensure that you are using the project files from the latest VSCode-Maxim version, and that the version number of Visual Studio Code and the C/C++ extension match the release notes.  Sometimes, issues are caused by VS Code auto-updates.

Additionally, ensure that your MaximSDK is updated to the latest version. You can use the "MaintenanceTool" application in the root directory of the SDK installation.

### Testing the Setup

Opening a VSCode-Maxim project with `File > Open Folder` should make Maxim's toolchain accessible from the integrated terminal.  To test that everything is working properly :

1. Navigate to the open `TERMINAL` tab on the bottom of the VS Code application.  If a terminal is not open, you can open a new terminal with `Terminal > New Terminal` or (Ctrl+Shift+`).  

   ![Terminal image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/Terminal.JPG)

2. The following commands to retrieve version numbers should be able to be run successfully from within the terminal :

    * `make -v`
    * `openocd -v`
    * `arm-none-eabi-gcc -v`
    * `arm-none-eabi-gdb -v`

   For example, the `make -v` command should similar to the following:

   ![Make -v example output](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/make_test.JPG)

If the tools are not accessible from the terminal, then the system settings and/or project settings must be examined further.  (Troubleshooting guide is in progress)

### Common Issues Caused by a Bad MAXIM_PATH

* Large 'Problem' count when opening VS Code

    ![Problems Screenshot](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/issue_includeerrors.jpg)

* "Unable to resolve configuration with compilerPath..."

    ![Compiler Path Issue](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/issue_compilerpath.jpg)

The issues above are usually caused by a missing or improperly set "MAXIM_PATH" global settings.json variable.  

If you see the issues below ensure that you have set "MAXIM_PATH" in your _global_ user settings.json file, and that this path matches the location of the MaximSDK installation on your system.

This can be resolved by double checking that the [Installation](#installation) procedure has been followed exactly.

You can check the MAXIM_PATH on the VS Code terminal with the following commands...

(Windows cmd)

```shell
echo %MAXIM_PATH%
```

(Windows powershell)

```shell
echo $env:MAXIM_PATH
```

(Linux/MacOS)

```shell
printenv | grep MAXIM_PATH
```

... which should print the exact location of the MaximSDK installation.

### Strange Debugger Behavior

If you debugger is behaving strangely (which might consist of skipping code, throwing hard faults, failing to find function definitions, etc.) ensure that the program you're debugging matches the latest build output of the project.  These symptoms are usually caused by a mismatch between the build output file and the contents of the micro's flash.

This issue is can usually be fixed by running the "Flash" build task.  Remember - flashing does not happen automatically when launching the debugger.

## Issue Tracker

Bug reports, feature requests, and contributions are welcome via the [issues](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim/issues) tracker on Github.

New issues should contain _at minimum_ the following information:

* Visual Studio Code version #s (see `Help -> About`)
* C/C++ Extension version #
* Target microcontroller and evaluation platform
* The projects `.vscode` folder and `Makefile` (where applicable).  Standard compression formats such as `.zip`, `.rar`, `.tar.gz`, etc. are all acceptable.
