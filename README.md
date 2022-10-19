# Analog Devices MSDK

## License Agreement

**© Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.**

This software is protected by copyright laws of the United States and
of foreign countries. This material may also be protected by patent laws
and technology transfer regulations of the United States and of foreign
countries. This software is furnished under a license agreement and/or a
nondisclosure agreement and may only be used or reproduced in accordance
with the terms of those agreements. Dissemination of this information to
any party or parties not specified in the license agreement and/or 
nondisclosure agreement is expressly prohibited.

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.**

Except as contained in this notice, the name of **Maxim Integrated Products, Inc.**
shall not be used except as stated in the **Maxim Integrated Products, Inc. Branding Policy.**

The mere transfer of this software does not imply any licenses
of trade secrets, proprietary technology, copyrights, patents,
trademarks, maskwork rights, or any other form of intellectual
property whatsoever. **Maxim Integrated Products, Inc.** retains all
ownership rights.

[Final Copyright](https://www.maximintegrated.com/en/aboutus/legal/copyrights/default-copyright.html "Final Copyright")

[Final Software License Agreement](https://www.maximintegrated.com/en/aboutus/legal/sla/no-distribute.html "Final SLA")

## Introduction

The Maxim Microcontrollers SDK (MSDK), now a part of Analog Devices, contains tools and resources to develop firmware for the [MAX-series of microcontrollers](https://www.maximintegrated.com/en/products/microcontrollers.html).  This includes register files, peripheral drivers, system startup files, documentation, various utilities, third-party libraries, IDE support files, and a toolchain.

The following development environments are supported:

* Command-line development
* Eclipse
* Visual Studio Code
* IAR
* Keil

This repository contains the latest source code of the MSDK and is used for development.  It does _not_ contain the MSDK _toolchain_, which is a separate collection of programs used to build, program, and debug the contents of this repo on hardware.

The full MSDK is available via an [Automatic Installer](#automatic-installer), which pulls from the latest release tag of this repo and packages it alongside the toolchain.

Users who would like to retrieve the bleeding-edge development copies of the MSDK resources can pull them from this repository.  See [Developing from the Repo](#developing-from-the-repo).

## Installation

### Automatic Installer

The MSDK is available via an automatic installer for the platforms below.  The automatic installer will retrieve the latest _release_ version of this repository and the latest toolchain for your OS.

* [Windows 10](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0010820A)

* [Ubuntu Linux](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018720A)

  * This file must be made executable before it can be run. Use the command `chmod +x MaximMicrosSDK_linux.run`. Alternatively, set “Allow executing as program” in the Ubuntu GUI.

* [MacOS](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018610A)

  * Some additional steps are required to complete the automatic installation on MacOS.  See the [Completing the Installation on MacOS](#completing-the-installation-on-macos) below.

### Cloning the Github Repo

This repo can be cloned to obtain the latest development copies of the MSDK source code.

1. First, you'll need to set up an SSH key for your Github account.  See the [Github Docs on SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) for instructions.

2. Once you've set up an SSH key for your account, you can clone this repository with the command:

    ```shell
    git clone --recurse git@github.com:Analog-Devices-MSDK/msdk.git
    ```

The source code in the repo can now be copied elsewhere, but if you'd like to set it up for "in-place" development you'll also need to perform a few manual steps to link the toolchain.  See [Developing from the Repo](#developing-from-the-repo).

### Developing from the Repo

The Github repo contains source code only.  In order to develop on it directly the toolchain must be made available at the same file-paths as the full MSDK installation.  The easiest way to do this is to retrieve the toolchain with the automatic installer and then create symbolic links.  This section walks through the process.

1. Install the toolchain via the [Automatic Installer](#automatic-installer) for your OS if you haven't already.

    At minimum, install the components:
    * GNU RISC-V Embedded GCC
    * GNU Tools for ARM Embedded Processors
    * Open On-Chip Debugger
    * MSYS2 (if you're on Windows 10)

    If you've already installed the MSDK, run the `MaintenanceTool` program to ensure these components are selected and updated to the latest version.

2. Clone the Github repository to an accessible location without any spaces in its filepath.  See [Cloning the Github Repo](#cloning-the-github-repo).

3. Create symbolic directory links to link the toolchain at the expected locations.

    On **Windows**:
    * Open a **command prompt** _as administrator_.

    * `cd` into the cloned location of the Github repo from step 2.

    * Run the following commands.  If you installed the MSDK to a non-default location in step 1, change `C:\MaximSDK` to point to the location you chose.

        ```cmd
        mklink /D Tools\GNUTools C:\MaximSDK\Tools\GNUTools
        ```

        ```cmd
        mklink /D Tools\OpenOCD C:\MaximSDK\Tools\OpenOCD
        ```

        ```cmd
        mklink /D Tools\MSYS2 C:\MaximSDK\Tools\MSYS2
        ```

        ```cmd
        mklink /D Tools\xPack C:\MaximSDK\Tools\xPack
        ```

        Example output:

        ```cmd
        C:\Users\Username\repos\msdk>mklink /D Tools\GNUTools C:\MaximSDK\Tools\GNUTools
        symbolic link created for Tools\GNUTools <<===>> C:\MaximSDK\Tools\GNUTools

        C:\Users\Username\repos\msdk>mklink /D Tools\OpenOCD C:\MaximSDK\Tools\OpenOCD
        symbolic link created for Tools\OpenOCD <<===>> C:\MaximSDK\Tools\OpenOCD

        C:\Users\Username\repos\msdk>mklink /D Tools\MSYS2 C:\MaximSDK\Tools\MSYS2
        symbolic link created for Tools\MSYS2 <<===>> C:\MaximSDK\Tools\MSYS2

        C:\Users\Username\repos\msdk>mklink /D Tools\xPack C:\MaximSDK\Tools\xPack
        symbolic link created for Tools\xPack <<===>> C:\MaximSDK\Tools\xPack

        ```

    On **Linux/MacOS**:
    * Open a terminal

    * `cd` into the cloned location of the Github repo from step 2.

    * Run the following commands.  If you installed the MSDK to a non-default location in step 1, change `~\MaximSDK` to point to the location you chose.

        ```shell
        ln -s ~/MaximSDK/Tools/GNUTools Tools/GNUTools
        ```

        ```cmd
        ln -s ~/MaximSDK/Tools/OpenOCD Tools/OpenOCD
        ```

        ```cmd
        ln -s ~/MaximSDK/Tools/xPack Tools/xPack
        ```

        You can use `ls -la Tools` to verify the links have been created successfully.

        Example output:

        ```shell
        username@machine:~/repos/msdk$ ln -s ~/MaximSDK/Tools/GNUTools Tools/GNUTools
        username@machine:~/repos/msdk$ ln -s ~/MaximSDK/Tools/OpenOCD Tools/OpenOCD
        username@machine:~/repos/msdk$ ln -s ~/MaximSDK/Tools/xPack Tools/xPack
        username@machine:~/repos/msdk$ ls -la Tools
        total 20
        drwxr-xr-x 5 username username 4096 Oct  4 16:32 .
        drwxr-xr-x 9 username username 4096 Oct  4 16:29 ..
        drwxr-xr-x 2 username username 4096 Oct  4 16:29 BitmapConverter
        drwxr-xr-x 2 username username 4096 Oct  4 16:29 Bluetooth
        lrwxrwxrwx 1 username username   40 Oct  4 16:32 GNUTools -> /home/username/MaximSDK/Tools/GNUTools
        lrwxrwxrwx 1 username username   39 Oct  4 16:32 OpenOCD -> /home/username/MaximSDK/Tools/OpenOCD
        drwxr-xr-x 6 username username 4096 Oct  4 16:29 SBT
        lrwxrwxrwx 1 username username   37 Oct  4 16:32 xPack -> /home/username/MaximSDK/Tools/xPack
        ```

4. The MSDK now contains a virtual copy of the toolchain at the correct locations.  Some additional setup may be required to use it depending on your chosen development environment.  See [Setup](#setup) below.

### Completing the Installation on MacOS

On MacOS, some additional missing packages must be manually installed via [Homebrew](https://brew.sh/).

**For non-M1 platforms:**

1. Follow the instructions on the [Homebrew home page](https://brew.sh/) to install Homebrew on your system.

2. Then, open a terminal and run the command...

    ```shell
    brew install libusb-compat libftdi hidapi libusb
    ```

**For M1 platforms**:

You must use a Rosetta terminal to install Homebrew:

1. Open a terminal and update Rosetta.

    ```shell
    softwareupdate --install-rosetta --agree-to-license
    ```

2. Close the terminal.

3. Run a new terminal in Rosetta:

    1. Go to Finder > Applications and find your Terminal

    2. Right-Click Terminal and Duplicate it.  Rename it to "Terminal i386".

    3. Rich-Click "Terminal i386" > Get Info > Enable "Open using Rosetta"

    4. Launch the new "Terminal i386" and type `arch` to verify that it says `i386` now.

4. From your Rosetta terminal follow the instructions on the [Homebrew home page](https://brew.sh/) to install Homebrew on your system.

5. Run the command...

    ```shell
    brew install libusb-compat libftdi hidapi libusb
    ```

## Setup

The setup guides below demonstrate how to configure supported development environments for use with this repository.  It assumes a successful [installation](#installation).

### Visual Studio Code

All example projects in the MSDK come pre-configured with [Visual Studio Code](https://code.visualstudio.com/) project folders that are managed by the [VSCode-Maxim](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop) repository.  They can be found in the `.vscode` sub-folder of each example, and include local copies of the VSCode-Maxim documentation.

The setup procedure is exactly the same as a standard VSCode-Maxim installation (instructions [here](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#installation)), except on step 9 `"MAXIM_PATH"` can be set to the location of this repository instead of the release version of the MSDK.

See [Usage](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#usage) from the VSCode-Maxim readme for more details on using the projects once setup is complete.

### Eclipse

Eclipse itself should be launched with the `Tools/Eclipse/cdt/eclipse(.bat/.sh)` script, which will set `MAXIM_PATH` and a few other environment variables for use with the _release_ version of the MSDK by default.  Therefore, there are two options for using Eclipse with the MSDK development repository:

#### Option 1 - Project Settings

1. Import or open a project in your Eclipse workspace.

2. Right click on the project and select `Properties`.

3. Navigate to `C/C++ Build -> Environment`.

4. Add a _new_ environment variable.  Set:

    * Name: `MAXIM_PATH`

    * Value: Installed location of this repository.  Ex: `C:\Users\JCarter3\repos\msdk`.

    * Select **"Add to all configurations"**

5. Ensure the **"Replace native environment with the specified one"** option is selected.

6. Hit "Apply" -> "Apply and Close".

7. Clean the project.

The project is now configured for use with the MSDK development repo, and subsequent builds will load the latest development resources.

#### Option 2 - Edit setenv

1. Locate the `setenv.bat` script that can be found in the root directory of the _release_ MSDK installation.

2. (Optional, recommended) Copy the `setenv.bat` file to a backup called `setenv-release.bat`.  This backup file can be used to revert back to the original if needed.

3. Open the `setenv.bat` script in a text editor.

4. Change the line

    ```bat
    set MAXIM_PATH=%CD%
    ```

    to set `MAXIM_PATH` to the installed location of this repository.  For example:

    ```bat
    set MAXIM_PATH=C:\Users\Username\repos\msdk
    ```

**Note:** This will configure _all_ Eclipse sessions to load resources from the development repo.

### Command-line Setup

This section assumes some familiarity with basic terminal concepts such as your system's Path and environment variables.  Since there are such wide variety of terminal applications and Operating Systems, these instructions are intentionally left somewhat generic.  

However, an example `.bashrc` file is provided below that can be referenced for most Unix systems.  Simply copy and paste the entries below into your shell's startup script and edit `MAXIM_PATH` to the installed location of this repository.

```bash
# ~/.bashrc

# Set MAXIM_PATH environment variable
export MAXIM_PATH=#changeme!

# Add Arm Embedded GCC to path (v10.3)
export ARM_GCC_ROOT=$MAXIM_PATH/Tools/GNUTools/10.3
export PATH=$ARM_GCC_ROOT/bin:$PATH

# Add xPack RISC-V GCC to path (v10.2)
export XPACK_GCC_ROOT=$MAXIM_PATH/Tools/xPack/riscv-none-embed-gcc/10.2.0-1.2
export PATH=$XPACK_GCC_ROOT/bin:$PATH

# Add OpenOCD to path
export OPENOCD_ROOT=$MAXIM_PATH/Tools/OpenOCD
export PATH=$OPENOCD_ROOT:$PATH
```

More generic instructions can be found below.  These instructions will use Unix syntax unless a step is targeted specifically at Windows.

1. Create a new environment variable on your system called `MAXIM_PATH`.  Set its value to the installed location of this repository.

2. Add the following entries to your system's path:

    * `$MAXIM_PATH/Tools/GNUTools/10.3/bin`
    * `$MAXIM_PATH/Tools/xPack/riscv-none-embed-gcc/10.2.0-1.2/bin`
    * `$MAXIM_PATH/Tools/OpenOCD`

3. (Windows only) Add the following entry to your system's path:
    * `%MAXIM_PATH%/Tools/MSYS2/usr/bin`

4. Restart your shell, and verify the tools are accessible with the following commands:

    1. Arm Embedded GCC:

        ```shell
        $ arm-none-eabi-gcc -v

        Using built-in specs.
        COLLECT_GCC=arm-none-eabi-gcc

        # ... Lots of other info will be printed here...
        
        Supported LTO compression algorithms: zlib
        gcc version 10.3.1 20210824 (release) (GNU Arm Embedded Toolchain 10.3-2021.10)
        ```

    2. xPack RISC-V GCC:

        ```shell
        $ riscv-none-embed-gcc -v

        Using built-in specs.
        COLLECT_GCC=riscv-none-embed-gcc
        
        # ... Lots of other info will be printed here...

        Supported LTO compression algorithms: zlib
        gcc version 10.2.0 (xPack GNU RISC-V Embedded GCC x86_64)
        ```

    3. OpenOCD

        ```shell
        $ openocd -v

        Open On-Chip Debugger 0.11.0+dev-g56a818e4c (2022-07-19-19:00)
        Licensed under GNU GPL v2
        For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
        ```

        **Note:** The automatic installer should have installed OpenOCD dependencies.  Should any missing package errors get thrown on this command they can be resolved by installing the missing packages manually.

        **On linux:**

        ```shell
        sudo apt-get install libusb-1.0 libusb-0.1 libhidapi-libusb0 libhidapi-hidraw0
        ```

        **On MacOS with [Homebrew](https://brew.sh/)**

        ```shell
        brew install libusb-compat libftdi hidapi libusb
        ```

    4. GNU Make

        ```shell
        $ make -v

        GNU Make 4.3
        Built for x86_64-pc-linux-gnu
        Copyright (C) 1988-2020 Free Software Foundation, Inc.
        License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
        This is free software: you are free to change and redistribute it.
        There is NO WARRANTY, to the extent permitted by law.
        ```

        Make should also have been installed by the automatic installer.   If Make is not available, install it with your package manager.

        **On linux:**

        ```shell
        sudo apt-get install make
        ```

        **On MacOS with [Homebrew](https://brew.sh/)**

        ```shell
        brew install make
        ```

## Usage Quick-Start

Following a successful [installation](#installation) and [setup](#setup), additional documentation on usage can be found below:

* [Visual Studio Code](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#usage)
* [Eclipse](https://www.maximintegrated.com/en/design/technical-documents/userguides-and-manuals/6/6245.html)
* [IAR](https://www.youtube.com/playlist?list=PLQ4i891m2efIwwd4ScApoPv7RaYLwhAjF)
* [Keil](https://www.youtube.com/watch?v=d_O2tu5CMbQ)

## Contributing

Contributions to the MSDK are welcome.  See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.
