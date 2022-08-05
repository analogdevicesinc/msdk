License Agreement
=====================

**© Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.**

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

The Maxim Microcontrollers SDK (MaximSDK), now a part of Analog Devices, contains tools and resources to develop firmware for the [MAX-series of microcontrollers](https://www.maximintegrated.com/en/products/microcontrollers.html).  This includes register files, peripheral drivers, system startup files, documentation, various utilities, third-party libraries, IDE support files, and a toolchain.

This repository contains the latest source code of the MaximSDK and is used for development.  It does _not_ contain the MaximSDK _toolchain_, which is a separate collection of programs used to build, program, and debug the contents of this repo on hardware.  

Currently, the MaximSDK toolchain is currently built around [GNU Make](https://www.gnu.org/software/make/), a [custom fork](https://github.com/Analog-Devices-MSDK/openocd) of [OpenOCD](https://openocd.org/), and [GNU GCC](https://www.bing.com/ck/a?!&&p=e191484d4651f3b3JmltdHM9MTY1ODUyNzg2NiZpZ3VpZD04ODEwYTM3NS1iM2FlLTQ3MGQtYTVjYS0zMjNmMmZlNWVkZWEmaW5zaWQ9NTE4Mg&ptn=3&hsh=3&fclid=2e9aac62-0a0b-11ed-99a6-78b7c6a38599&u=a1aHR0cHM6Ly9nY2MuZ251Lm9yZy8&ntb=1) ([Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain#:~:text=The%20GNU%20Arm%20Embedded%20toolchain%20contains%20integrated%20and,on%2032-bit%20Arm%20Cortex-A%2C%20Cortex-R%20and%20Cortex-M%20processors.) for Cortex-M development and [xPack RISCV GCC](https://github.com/xpack-dev-tools/riscv-none-embed-gcc-xpack) for RISC-V development)

The full MaximSDK (including its toolchain) is available via an [Automatic Installer](#automatic-installer), which pulls from the latest release tag and packages it alongside the toolchain.  

Users who would like to retrieve the bleeding-edge development copies of the MaximSDK resources can also pull them from this repo.  See [Installing from the Github Repo](#installing-from-the-github-repo)

## Installation

### Automatic Installer

The MaximSDK is available via an automatic installer for the platforms below.  The automatic installer will retrieve the latest _release_ version of this repository _and_ the latest toolchain for your OS.

* [Windows 10](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0010820A)

* [Ubuntu Linux](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018720A)

    * This file must be made executable before it can be run. Use the command `chmod +x MaximMicrosSDK_linux.run`. Alternatively, set “Allow executing as program” in the Ubuntu GUI.

* [MacOS](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018610A)

    * Some additional steps are required to complete the automatic installation on MacOS.  See the [Completing the Installation on MacOS](#completing-the-installation-on-macos) below.


### Cloning the Github Repo

This repo can be cloned to obtain the latest development copies of the MaximSDK source code.

1. First, you'll need to set up an SSH key for your Github account.  See the [Github Docs on SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) for instructions.

2. Once you've set up an SSH key for your account, you can clone this repository with the command:

    ```shell
    git clone --recurse git@github.com:Analog-Devices-MSDK/msdk.git
    ```

The source code in the repo can now be copied elsewhere, but if you'd like to set up the repo for "in-place" development you'll also need to perform a few manual steps to link the toolchain.  See [Developing from the Repo](#developing-from-the-repo)


### Developing from the Repo

The Github repo does not contain a copy of the toolchain.  In order to develop directly on the repo the toolchain must be linked into it.

1. Install the toolchain via the [Automatic Installer](#automatic-installer) for your OS if you haven't already.

    At minimum, install the components:
    * GNU RISC-V Embedded GCC
    * GNU Tools for ARM Embedded Processors
    * Open On-Chip Debugger
    * MSYS2 (if you're on Windows 10)

    If you've already installed the MaximSDK, run the `MaintenanceTool` program to ensure these components are selected and updated to the latest version.

2. Create symbolic directory links to link the toolchain at the expected locations.

    On **Windows**, open a terminal as administrator and `cd` into the cloned repo.  Then, use the `mklink` command.  
    
    Ex (with the MaximSDK installed to the default location):

    ```shell
    mklink /D Tools\GNUTools C:\MaximSDK\Tools\GNUTools && mklink /D Tools\OpenOCD C:\MaximSDK\Tools\OpenOCD && mklink /D Tools\MSYS2 C:\MaximSDK\Tools\MSYS2 && mklink /D Tools\xPack C:\MaximSDK\Tools\xPack
    ```

    On **Linux/MacOS**, open a terminal and `cd` into the cloned repo.  Then, use the `ln` command.

    Ex (with the MaximSDK installed to the default location):

    ```shell
    ln -s ~/MaximSDK/Tools/GNUTools Tools/GNUTools && ln -s ~/MaximSDK/Tools/OpenOCD Tools/OpenOCD && ln -s ~/MaximSDK/Tools/xPack Tools/xPack
    ```

    The Github repo now has a virtual copy of the toolchain at the expected locations.

3. For **command-line** development add the following paths system's Path, replacing `repo` with the actual cloned location of the repository.

    * `repo/Tools/GNUTools/10.3/bin`
    * `repo/Tools/OpenOCD`
    * `repo/Tools/xPack/riscv-none-embed-gcc/10.2.0-1.2/bin`
    * `repo/Tools/MSYS2/usr/bin` (On Windows only)

4. For development with **[Visual Studio Code](https://github.com/Analog-Devices-MSDK/VSCode-Maxim)**, simply set `MAXIM_PATH` in your global settings.json file to the location of the Github repo.  See step 10 of the [VSCode-Maxim installation](https://github.com/Analog-Devices-MSDK/VSCode-Maxim#installation) instructions for more details.  VS Code will now load source code from the Github repo and the example projects should behave as expected.


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
