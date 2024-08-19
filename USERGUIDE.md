# MSDK User Guide

## Overview

The Maxim Microcontrollers SDK (MSDK), now a part of [Analog Devices](https://www.analog.com/en/index.html), contains the necessary software and tools to develop firmware for the [MAX32xxx and MAX78xxx Microcontrollers](https://www.analog.com/en/parametricsearch/10984). That includes register and system startup files to enable low-level development for its [supported parts](#supported-parts). It also provides higher-level peripheral driver APIs (written in C) alongside various utilities, third-party libraries, Board Support Packages (BSPs), and a set of example programs for each microcontroller.

Additionally, the MSDK includes a GCC-based toolchain, and builds are managed by a system of Makefiles (See [GNU Make](https://www.gnu.org/software/make/manual/)). A [custom fork of OpenOCD](https://github.com/analogdevicesinc/openocd) enables flashing and debugging. The MSDK's toolchain and build system offers a Command Line Interface (CLI), and project files for [supported development environments](#supported-development-environments) are maintained that build on top of that CLI.

This document describes the MSDK's installation, setup, and usage.

### Supported Operating Systems

- Windows (Windows 10 only)

- Linux (Ubuntu only)

- MacOS

### Supported Parts

The MSDK officially supports the following microcontrollers and evaluation platforms.

* [**MAX32520**](https://www.analog.com/en/products/max32520.html): ChipDNA Secure Microcontroller with Secure Boot for IoT Applications

    - [MAX32520-KIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32520-kit.html)

    - [MAX32520FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32520fthr.html)


---

* [**MAX32570**](https://www.analog.com/en/products/max32570.html):  Low-Power Arm Cortex-M4 Microcontroller with Contactless Radio for Secure Applications **(Available by NDA only**)

    - [MAX32570-QNKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32570-qnkit.html)

    - [MAX32570-MNKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32570-mnkit.html)


---

* **MAX32572** **(Not Yet Publicly Available)**
    - MAX32572EVKIT **(Not Yet Publicly Available)**


---

* [**MAX32650**](https://www.analog.com/en/products/max32650.html):  Ultra-Low-Power Arm Cortex-M4 with FPU-Based Microcontroller (MCU) with 3MB Flash and 1MB SRAM

    - [MAX32650-EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32650-evkit.html)

    - [MAX32650FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32650fthr.html)


---

* [**MAX32651**](https://www.analog.com/en/products/max32651.html):  Ultra-Low-Power Arm Cortex-M4 with FPU-Based Microcontroller (MCU) with 3MB Flash and 1MB SRAM

    - [MAX32651-EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32651-evkit.html)


---

* [**MAX32655**](https://www.analog.com/en/products/max32655.html): Low-Power, Arm Cortex-M4 Processor with FPU-Based Microcontroller and Bluetooth 5.2

    - [MAX32655EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32655evkit.html)

    - [MAX32655FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32655fthr.html)


---

* [**MAX32660**](https://www.analog.com/en/products/max32660.html):  Tiny, Ultra-Low-Power Arm Cortex-M4 Processor with FPU-Based Microcontroller (MCU) with 256KB Flash and 96KB SRAM

    - [MAX32660-EVSYS](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32660-evsys.html)


---

* [**MAX32662**](https://www.analog.com/en/products/max32662.html): Arm Cortex-M4 Processor with FPU-Based Microcontroller (MCU) with 256KB Flash and 80KB SRAM

    - [MAX32662EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/MAX32662EVKIT.html)


---

* [**MAX32665-MAX32666 Family**](https://www.analog.com/en/products/max32665.html):  Low-Power Arm Cortex-M4 with FPU-Based Microcontroller with Bluetooth 5 for Wearables

    - [MAX32666EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32666evkit.html)

    - [MAX32666FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32666fthr.html)

    - MAX32666FTHR2 (Product Page Not Yet Available)


---

* [**MAX32670**](https://www.analog.com/en/products/max32670.html):  High-Reliability, Ultra-Low-Power Microcontroller Powered by Arm Cortex-M4 Processor with FPU for Industrial and IoT

    - [MAX32670EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32670evkit.html)


---

* [**MAX32672**](https://www.analog.com/en/products/max32672.html): High-Reliability, Tiny, Ultra-Low-Power Arm Cortex-M4F Microcontroller with 12-Bit 1MSPS ADC

    - [MAX32672EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32672evkit.html)


---

* [**MAX32675**](https://www.analog.com/en/products/max32675.html):  Ultra-Low-Power Arm Cortex-M4F with Precision Analog Front-End for Industrial and Medical Sensors

    - [MAX32675EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32675evkit.html)

    - MAX32675FTHR (Product Page Not Yet Available)


---

* [**MAX32680**](https://www.analog.com/en/products/max32680.html):  Ultra-Low-Power Arm Cortex-M4F with Precision Analog Front-End and Bluetooth LE 5.2

    - [MAX32680EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32680evkit.html)


---

* [**MAX32690**](https://www.analog.com/en/products/max32690.html):  Arm Cortex-M4 with FPU Microcontroller and Bluetooth LE 5 for Industrial and Wearables

    - [MAX32690EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/MAX32690EVKIT.html)
    - [AD-APARD32690-SL](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/ad-apard32690-sl.html)

---

* [**MAX78000**](https://www.analog.com/en/products/max78000.html):  Artificial Intelligence Microcontroller with Ultra-Low-Power Convolutional Neural Network Accelerator

    - [MAX78000EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html)

    - [MAX78000FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000fthr.html)

    - [MAXREFDES178](https://www.analog.com/en/design-center/reference-designs/maxrefdes178.html)


---

* [**MAX78002**](https://www.analog.com/en/products/max78002.html):  Artificial Intelligence Microcontroller with Low-Power Convolutional Neural Network Accelerator

    - [MAX78002EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78002evkit.html)


---

### Supported Development Environments

- [Visual Studio Code](https://code.visualstudio.com/)
- [Eclipse IDE](https://www.eclipseide.org/)
- [IAR Embedded Workbench](https://www.iar.com/ewarm)
- [Keil MDK](https://www2.keil.com/mdk5/)
- Command-line Development

    - Supported shells (Windows)
        - [MSYS2](https://www.msys2.org/)

    - Supported shells (Ubuntu)
        - [Bash](https://tiswww.case.edu/php/chet/bash/bashtop.html)
        - [Zsh](https://www.zsh.org/)

    - Supported shells (MacOS)
        - [Zsh](https://www.zsh.org/)

### Supported Languages

- C
- C++
- Assembly (Arm and/or RISC-V instruction set depending on the microcontroller)

## Installation

### Prerequisites

- **Elevated/Administrator rights**

- ???+ warning "**⚠️ MacOS**"
    On MacOS, please also download and install [Homebrew](https://brew.sh/).  It will be used in [Completing the Installation on MacOS](#completing-the-installation-on-macos) later on.

- ???+ warning "**⚠️ Ubuntu**"
    Several GUI packages are required by the QT installer framework _even on headless systems_.  Run the following command _before_ running the installer to retrieve them.

            :::shell
            sudo apt update && sudo apt install libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-shm0 libxcb-util1 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-render0 libxcb-shape0 libxcb-sync1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1 libxcb1 libxkbcommon-x11-0 libxkbcommon0 libgl1 libusb-0.1-4 libhidapi-libusb0 libhidapi-hidraw0

### Download

The MSDK installer is available for supported Operating Systems from the download links below.

- [**Windows 10**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0010820B)

- [**Linux (Ubuntu)**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018720B)

    - ???+ note "ℹ️ **Note**"
        This file must be made executable before running (`chmod +x MaximMicrosSDK_linux.run`). Alternatively, set `Allow executing as program" in the Ubuntu GUI.
        ![Figure 1](res/Fig1.jpg)

- [**MacOS**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018610B)

    - ???+ note "ℹ️ **Note**"
        On MacOS, the installer is distributed inside a .dmg disk image file. Double-click the downloaded file to mount it. Afterward, the installer executable will be made available inside the mounted drive.

### Setup

The MSDK installer can be run through a [**GUI Installation**](#gui-installation) or a [**Command-Line Installation**](#command-line-installation)

#### GUI Installation

1. [**Download**](#download) the installer executable to an accessible location and launch it.

2. Click **Next** to proceed from the Welcome screen.

3. Choose the installation location.

    ![Figure 2](res/Fig2.jpg)

4. Select the components to install. It's recommended to install all components.

    ![Figure 3](res/Fig3.jpg)

5. Continue to the installation page, and click **install** to begin. Installation can be safely canceled at any time.

    ![Figure 6](res/Fig6.jpg)

    ![Figure 7](res/Fig7.jpg)

6. Click **Finish** to complete the installation.

    ![Figure 8](res/Fig8.jpg)

7. You should now see the contents of the installation directory populated with the MSDK.

    ![Figure 10](res/Fig10.jpg)

    ???+ warning "⚠️ **Warning**"
        On MacOS, some [_additional steps_](#completing-the-installation-on-macos) are required.


#### Command-Line Installation

The MSDK installer features a command-line interface that can be used as an alternative to its GUI.  This is useful for installations on "headless" systems and scripting.

??? note "ℹ️ **Note:**" The `--help` Command"
    The available commands can be retrieved by running the MSDK installer executable with the `--help` option on the command line.  For example:

        :::shell
        $ ./MaximMicrosSDK_linux.run --help

        Usage: ./MaximMicrosSDK_linux.run [options] command <args> <key=value>

        Qt Installer Framework supports both GUI and headless mode. The installation operations can be invoked with the following commands and options. Note that the options marked with "CLI" are available in the headless mode only.

        Commands:
        in, install - install default or selected packages - <pkg ...>
        ch, check-updates - show available updates information on maintenance tool
        up, update - update all or selected packages - <pkg ...>
        rm, remove - uninstall packages and their child components - <pkg ...>
        li, list - list currently installed packages - <regexp>
        se, search - search available packages - <regexp>
            Note: The --filter-packages option can be used to specify
            additional filters for the search operation
        co, create-offline - create offline installer from selected packages - <pkg ...>
        pr, purge - uninstall all packages and remove entire program directory

        Options:
        -h, --help                                     Displays help on commandline
                                                        options.
        # ...

To run a _headless_ installation:

1. [**Download**](#download) the installer executable to an accessible location.

2. Ensure that you are able to run the installer with **_elevated permissions_**.

    ???+ note "Windows"
        Open a Command Prompt or PowerShell **_as administrator_**.

    ???+ note "Ubuntu and MacOS"
        Ensure you have `sudo` rights.

3. Run the installer with the arguments `in --root <install location>`

    ???+ note "Windows"
            :::shell
            .\MaximMicrosSDK_win.exe in --root C:/MaximSDK

    ???+ note "Ubuntu and MacOS"
            :::shell
            sudo ./MaximMicrosSDK_linux.run in --root ~/MaximSDK

4. Follow the installer's command-line instructions to accept licenses and confirm installation size.

    ??? note "ℹ️ **Note: Unattended Installations**"
        You can run the installer without any user input by auto-accepting all licenses, messages, and input.

            :::shell
            sudo ./MaximMicrosSDK_linux.run in --root ~/MaximSDK --accept-licenses --accept-messages --confirm-command

5. (Ubuntu and MacOS) Change ownership of the installation folder with:

        :::shell
        sudo chown -R $(whoami):$(whoami) <MSDK installation folder>

    ??? note "ℹ️ **Note: Folder Ownership**"
        Usually, running the installation with `sudo` results in an installation owned by the `root` user.

        You can verify this with the `ls -la` command.

            :::shell
            ls -la ~/MaximSDK
            total 29656
            drwxr-xr-x   8 root     root         4096 Jul 13 20:41 .
            drwxr-x---  17 username username     4096 Jul 13 20:41 ..
            drwxr-xr-x   2 root     root         4096 Jul 13 20:41 Documentation
            drwxr-xr-x  15 root     root         4096 Jul 13 20:41 Examples
            -rw-r--r--   1 root     root       171189 Jul 13 20:41 InstallationLog.txt
            drwxr-xr-x  17 root     root         4096 Jun 28 23:42 Libraries
            drwxr-xr-x   2 root     root         4096 Jul 13 20:41 Licenses
            -rwxr-xr-x   1 root     root     28287992 Jul 13 20:41 MaintenanceTool
            -rw-r--r--   1 root     root      1719694 Jul 13 20:41 MaintenanceTool.dat
            -rw-r--r--   1 root     root         9770 Jul 13 20:41 MaintenanceTool.ini
            drwxr-xr-x  11 root     root         4096 Jun 28 23:42 Tools
            -rw-r--r--   1 root     root        13123 Jun 28 23:48 changelog.txt
            -rw-r--r--   1 root     root        67664 Jul 13 20:41 components.xml
            -rw-r--r--   1 root     root           48 Jul 13 20:41 installer.dat
            drwxr-xr-x 112 root     root        12288 Jul 13 20:41 installerResources
            -rw-r--r--   1 root     root        25214 Jun 29 00:47 maxim.ico
            -rw-r--r--   1 root     root          362 Jul 13 20:41 network.xml
            -rwxrwxrwx   1 root     root         1129 Jun 29 00:47 setenv.sh
            -rwxrwxrwx   1 root     root          300 Jun 29 00:47 updates.sh

        The owner of the MSDK installation should be changed back to the normal user with the command above.  Otherwise, the toolchain may behave inconsistently against file permission issues.

        Once complete, `ls -la` should look similar to below (where `username` is your username).

            :::shell
            ls -la ~/MaximSDK
            total 29656
            drwxr-xr-x   8 username     username         4096 Jul 13 20:41 .
            drwxr-x---  17 username     username         4096 Jul 13 20:41 ..
            drwxr-xr-x   2 username     username         4096 Jul 13 20:41 Documentation
            drwxr-xr-x  15 username     username         4096 Jul 13 20:41 Examples
            -rw-r--r--   1 username     username       171189 Jul 13 20:41 InstallationLog.txt
            drwxr-xr-x  17 username     username         4096 Jun 28 23:42 Libraries
            drwxr-xr-x   2 username     username         4096 Jul 13 20:41 Licenses
            -rwxr-xr-x   1 username     username     28287992 Jul 13 20:41 MaintenanceTool
            -rw-r--r--   1 username     username      1719694 Jul 13 20:41 MaintenanceTool.dat
            -rw-r--r--   1 username     username         9770 Jul 13 20:41 MaintenanceTool.ini
            drwxr-xr-x  11 username     username         4096 Jun 28 23:42 Tools
            -rw-r--r--   1 username     username        13123 Jun 28 23:48 changelog.txt
            -rw-r--r--   1 username     username        67664 Jul 13 20:41 components.xml
            -rw-r--r--   1 username     username           48 Jul 13 20:41 installer.dat
            drwxr-xr-x 112 username     username        12288 Jul 13 20:41 installerResources
            -rw-r--r--   1 username     username        25214 Jun 29 00:47 maxim.ico
            -rw-r--r--   1 username     username          362 Jul 13 20:41 network.xml
            -rwxrwxrwx   1 username     username         1129 Jun 29 00:47 setenv.sh
            -rwxrwxrwx   1 username     username          300 Jun 29 00:47 updates.sh



#### Completing the Installation on MacOS

???+ warning "⚠️ **Warning**"
    On MacOS, some additional missing packages must be manually installed with [Homebrew](https://brew.sh/).  There are also some manual setup steps required to retrieve `make` version 4.  The instructions in this section are critical.

1. Install [Homebrew](https://brew.sh/).

2. Run the command below to install dependencies for OpenOCD.

        :::shell
        brew install libusb-compat libftdi hidapi libusb

### Maintenance

An MSDK installation contains a `MaintenanceTool` executable program in its root directory. Use the Maintenance Tool to retrieve updates, manage components, and uninstall the MSDK.

![Figure 11](res/Fig11.jpg)

#### Updates

The MSDK releases updates quarterly, and the Maintenance Tool will retrieve the latest release when **Update components** is run.

#### Older Versions and Offline Installer

Older versions of the MSDK are available as an **_offline installer_** for each release tag. They are available on the [Releases page](https://github.com/analogdevicesinc/msdk/releases) of the MSDK GitHub and can be used to roll back to a specific MSDK release.

#### Development Resources

Users can obtain development copies of the MSDK resources from [Github](https://github.com/analogdevicesinc/msdk).  Setup instructions can be found in the repository's [README](https://github.com/analogdevicesinc/msdk/blob/main/README.md).

## Getting Started

The MSDK is designed for both evaluation and end-application development. The typical **evaluation** cycle usually involves setting up the development environment, running demos, and exercising the peripheral driver API on an _evaluation platform_. The typical **development** cycle typically involves building a prototype application on an _evaluation platform_ first, then porting the application to a custom board. This section describes how to get started with the MSDK focusing on the _evaluation_ cycle.

**First**, review the [**Key Concepts**](#key-concepts) below.  Then, proceed to the section for your preferred IDE. Each sub-section is written as a self-contained quick-start with links to additional documentation on important topics.

- [Getting Started with Visual Studio Code](#getting-started-with-visual-studio-code)
- [Getting Started with Eclipse](#getting-started-with-eclipse)
- [Getting Started with Command-Line Development](#getting-started-with-command-line-development)

### Key Concepts

The MSDK supports multiple development environments with different features that may tailor to the user's preferences. There are a few key concepts to remember that are universal to MSDK development.

- **Target Microcontroller**:  The _target microcontroller_ refers to the base part number of the microcontroller used for development. The MSDK contains register-level support and startup files for each of its [supported parts](#supported-parts), and it's important to note that support files for a target microcontroller and its _Board Support Packages_ are distinct from each other.

    For example, if the [MAX78000EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html) _or_ [MAX78000FTHR](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78000fthr.html) is being used, the _Target Microcontroller_ is the MAX78000.

---

- **Board Support Package (BSP)**:  The MSDK supports evaluation platforms for target microcontrollers with _Board Support Packages_.  For microcontrollers with multiple evaluation platforms, multiple BSPs are available. These can be found in the `Libraries/Boards` folder of the MSDK installation.

    By default, most projects in the MSDK come pre-configured for the "EVKIT"-type BSP, which is generally the largest evaluation platform for that device with most (or all) pins broken out. It's important to note that the active BSP may need to be reconfigured for a project, which is done slightly differently for each development environment.

---

- **System Environment**:  Your system's _environment_ is a broad term that encapsulates the programs and variables available to your system's shell on the command line. The user is expected to have some basic familiarity with this concept.

---

- **System Path**:  Your system's _Path_ is a unique environment variable that tells it where to search for program binaries. The user is expected to be familiar with this concept and how to modify the system Path if necessary.

---

- **Integrated Development Environment (IDE)**:  An IDE offers a higher-level user interface (typically with a GUI) that manages the tools for **editing** source code, **building** source code, **flashing** program binaries, and **debugging**. The abbreviation is frequently used in this document, and the MSDK supports _multiple_ IDEs that can be used depending on preference. (See ["Supported Development Environments"](#supported-development-environments))

---

- **Build Configuration vs. Project Configuration**: An MSDK project is comprised of two complementary systems:  The _Build System_ and the _Integrated Development Environment (IDE)_. These systems each offer their own configuration interfaces, and it's important to note what each is used for.

    The **Build System** manages source code compilation into program binaries and offers a **Command-Line Interface (CLI)** for setting **Build Configuration Variables**.

    The **IDE** offers a higher-level user interface (typically with a GUI) for managing a project and sits _on top_ of the build system's _CLI_. Each IDE offers its own settings for managing fundamental aspects of the build, such as:

    - Setting the _Target Microcontroller_
    - Setting the _Board Support Package_
    - Configuring the _Environment_ and _System Path_ for use with the MSDK toolchain

### Getting Started with Visual Studio Code

The MSDK includes Visual Studio Code ("VS Code") support through the [VSCode-Maxim](https://github.com/MaximIntegratedTechSupport/VSCode-Maxim) project.

This section walks through setup, opening, and running an example project with VS Code. This material is also available in video form targeting the MAX78000 in ["Understanding Artificial Intelligence Episode 8.5 - Visual Studio Code"](https://www.analog.com/en/education/education-library/videos/6313212752112.html).

For complete documentation, see the [Visual Studio Code](#visual-studio-code) section of this User Guide.

#### Setup (VS Code)

The setup below only needs to be done once per MSDK [installation](#installation).

1. Download and install Visual Studio Code for your OS [here](https://code.visualstudio.com/Download).

2. Launch Visual Studio Code.

3. Install the Microsoft [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools).

4. Install the [Cortex-Debug extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

5. Use **`CTRL + SHIFT + P`** (or **`COMMAND + SHIFT + P`** on MacOS) to open the developer prompt.

6. Type "open user settings" and select the **"Preferences: Open User Settings (JSON)"** option.

    ![Open Settings JSON Command](res/Fig42.jpg)

7. Add the entries below to your user settings.json file.

        :::json
        // There may be other settings up here...

        "MAXIM_PATH": "Change me!  Only use forward slashes (/) for this path",
        "update.mode": "manual",
        "extensions.autoUpdate": false,

        // There may be other settings down here...

    ???+ warning "⚠️ **Setting MAXIM_PATH**"
        Set the `MAXIM_PATH` option to the _absolute path_ of the MSDK installation.
        For example, you might set `"MAXIM_PATH":"C:/MaximSDK"` on Windows and `"MAXIM_PATH":"/home/username/MaximSDK"` on Ubuntu/MacOS.

    ???+ note "ℹ️ **Note: Automatic Updates**"
        `"update.mode: "manual"` and `"extensions.autoUpdate": false` _disable_ automatic updates of VS Code and its extensions, respectively.  This is an _optional_ (but recommended) addition left over from the early days of VS Code development when there was lots of feature churn.  Things have stabilized more as of version 1.70+, but updates remain frequent.  For the VSCode-Maxim project files, the exact version numbers tested with each release can be found on the [VSCode-Maxim Releases](https://github.com/analogdevicesinc/VSCode-Maxim/releases) page.

8. Save your changes to the file with **`CTRL + S`** and restart VS Code.

#### Building and Running a Project (VS Code)

1. Launch Visual Studio Code.

2. Select **File -> Open Folder...**

    ![File -> Open Folder](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/file_openfolder.JPG)

3. Navigate to an example project for the target microcontroller in the MSDK's `Examples` folder.

    ![Figure 43](res/Fig43.jpg)

    ???+ warning "**⚠️ Copying Examples**"
        It's strongly recommended to copy example projects to an _outside_ folder before modifying them.  This keeps the MSDK's "source" copy preserved for reference.  Project folders must be copied to a location _without_ any spaces in its filepath.

4. VS Code will prompt for trust the first time. Select _Trust folder and enable all features_

    ![Trust Prompt](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/workspaceTrustPrompt.JPG)

5. The opened project should look something like this.

    ![Figure 16](res/Fig16.jpg)

6. Set the **Board Support Package** to match your evaluation platform. In VS Code, this is done by editing the `.vscode/settings.json` file and setting the `"board"`  project configuration option.

    ???+ note "ℹ️ **Note**"
        See [Board Support Packages](#board-support-packages) for more details and a table of values.

    ![Figure 17](res/Fig17.jpg)

7. Save your changes to `settings.json` with `CTRL+S`.

8. Reload the VS Code window. After changing any options in `settings.json`, a reload is necessary to force it to re-index VS Code's Intellisense engine.

    VS Code can be conveniently reloaded with the **Reload Window** developer command accessed with **`CTRL + SHIFT + P`** (or **`COMMAND + SHIFT + P`** on MacOS).

    ![Reload window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/reload_window.JPG)

9. Press the shortcut **`Ctrl+Shift+B`** to open the available **Build Tasks** (alternatively navigate to _Terminal -> Run Build task..._).

    ![Build Tasks Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/buildtasks.JPG)

10. Run the **"build"** task to compile the project for the configured _Target Microcontroller_ and _BSP_.  Notice that the `TARGET` and `BOARD` Build Configuration Variables are set on the command line. The program binary is successfully compiled into the `.elf` program binary in the **build** sub-folder of the project.

    ![Figure 18](res/Fig18.jpg)

11. Connect a debug adapter between the host PC and the evaluation platform. Detailed instructions on this hardware setup can be found in the evaluation platform's Datasheet and Quick-Start Guide, which are available on its [analog.com](https://analog.com) product page.

12. Run the **`flash`**  build task.  Running this task will automatically build the project if needed, flash the program binary, and halt the program execution to await a debugger connection.

    ![Figure 19](res/Fig19.jpg)

13. Open the **Run and Debug** window (**`CTRL+SHIFT+D`**) and launch the debugger (**`F5`**).

    ![Figure 20](res/Fig20.jpg)

14. Verify the program counter enters `main` successfully.

    ![Figure 21](res/Fig21.jpg)

15. Press **Continue** (**`F5`**) to run the program.

    ![Debugger Control Bar Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger_bar.JPG)

        Continue | Step Over | Step Into | Step Out | Restart | Stop

16. Exercise the debugger and press stop to disconnect when finished.

    ???+ note "ℹ️ **Note**"
        See [Visual Studio Code](#visual-studio-code) for additional more detailed documentation.

---

### Getting Started with Eclipse

#### Setup (Eclipse)

The only setup required to use Eclipse is to ensure that the "Eclipse" component has been selected during the [MSDK installation](#installation). If the MSDK is already installed, Eclipse can be retrieved using the [Maintenance Tool](#maintenance).

This section is an Eclipse "quick-start" that walks through creating, building, and running a project. For complete documentation, see the [Eclipse](#eclipse) section of this User Guide.

#### Building and Running a Project (Eclipse)

1. Launch Eclipse with its start menu shortcut.

    ![Figure 22](res/Fig22.jpg)

2. Ensure Eclipse is set to the **C/C++ perspective** in the top right corner.  Otherwise, the new project wizard will not show up.

3. Navigate to **File -> New -> Maxim Microcontrollers**.

    ![Figure 31](res/Fig31.jpg)

4. Enter the project name and hit **Next**.

    ![Figure 32](res/Fig32.jpg)

5. Follow the new project wizard.

    - Chip type selects the _Target Microcontroller_
    - Board type selects the [_Board Support Package (BSP)_](#board-support-packages)
    - Example type selects the example project to be copied as the template for the new project.
    - Adapter type selects the debug adapter to use.

    ![Figure 33](res/Fig33.jpg)

6. Select **Finish** to create the new project.

7. Build the project using the **Build** hammer button (top left).

    ![Figure 27](res/Fig27.jpg)

8.  Select the correct project in the **Launch Configuration** dropdown and set it to **Debug** mode.

9. Use the **Debug** button (top left) to flash the program binary and connect the debugger.

    ![Figure 28](res/Fig28.jpg)

10. The Eclipse view will switch to debug mode, and the debugger will break on entry into `main`.

    ![Figure 29](res/Fig29.jpg)

11. **Resume** the program (**`F8`**) using the top control bar and exercise the debugger.

    ![Figure 30](res/Fig30.jpg)

12. **Terminate** the debugger (**`CTRL+F2`**) when finished.

    ???+ note "ℹ️ **Note**"
        See [Eclipse](#eclipse) for additional more detailed documentation.

---

### Getting Started with Command-Line Development

This section demonstrates how to build MSDK example projects on the command line. It also shows how to flash and debug over the command line. The [MAX78002EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max78002evkit.html) will be used as an example, but the same concepts apply to all parts.

For more detailed documentation, see the [Command-Line Development](#command-line-development) section of this User Guide.

#### Setup (Command-Line)

##### Windows

On Windows, use the MinGW shortcut to launch an MSYS2/MinGW terminal. This shortcut points to `Tools/MSYS2/msys.bat` in an MSDK installation and correctly configures the user's environment for development.

![Figure 22](res/Fig22.jpg)

##### Linux/MacOS

###### Sourcing `setenv`

A `setenv.sh` script is available in the root directory of an MSDK installation.  This file can be sourced to facilitate the setup of an environment for MSDK development.

```bash
source ~/MaximSDK/setenv.sh
```

This command can also be added to shell startup scripts (`~/.bashrc`, `~/.zshrc`, etc.) to automate the environment setup.

???+ note "ℹ️ **Note: Automatic Updates**"
    `setenv.sh` will automatically check for available updates to the MSDK.  This can be permanently disabled by following its prompt on startup, or by deleting/moving the `updates.sh` script in the root directory of the MSDK installation.

    ![Figure 51](res/Fig51.jpg)

###### Manual Setup

1. On Linux and MacOS, copy the following contents into your shell's terminal profile/startup script to manually configure your environment for MSDK development. Depending on your system and shell, this could be `~/.profile`, `~/.zprofile`, `~/.bashrc`, `~/.zshrc`, etc. Command-line Linux/MacOS users are expected to know which file to edit for their particular system and preferences.

        # Set MAXIM_PATH to point to the MSDK
        export MAXIM_PATH=#changeme!

        # Add Arm Embedded GCC to path (v10.3)
        export ARM_GCC_ROOT=$MAXIM_PATH/Tools/GNUTools/10.3
        export PATH=$ARM_GCC_ROOT/bin:$PATH

        # Add xPack RISC-V GCC to path (v12.2)
        export XPACK_GCC_ROOT=$MAXIM_PATH/Tools/xPack/riscv-none-elf-gcc/12.2.0-3.1
        export PATH=$XPACK_GCC_ROOT/bin:$PATH

        # Add OpenOCD to path
        export OPENOCD_ROOT=$MAXIM_PATH/Tools/OpenOCD
        export PATH=$OPENOCD_ROOT:$PATH

2. Change `export MAXIM_PATH=#changeme!` to the installation location of the MSDK. This will make the toolchain accessible from the command line by adding it to your *system's path*.

        # Set MAXIM_PATH environment variable
        export MAXIM_PATH=$HOME/MaximSDK

##### Verification

Run the following commands to verify that the toolchain is accessible. They should display version numbers successfully.

- `arm-none-eabi-gcc -v`
- `arm-none-eabi-gdb -v`
- `make -v`
- `openocd -v`

Any "file not found" errors indicate that `MAXIM_PATH` has not been set correctly or the system's Path has not been configured correctly.

#### Building and Running an Example (Command-Line)

1. First, copy an [example project](https://github.com/analogdevicesinc/msdk/tree/main/Examples) to an accessible directory outside of the SDK. The `Hello_World` project is a good one to start with.

    ???+ warning "**⚠️ Copying Examples**"
        It's strongly recommended to copy example projects to an _outside_ folder before modifying them.  This keeps the MSDK's "source" copy preserved for reference.  Project folders must be copied to a location _without_ any spaces in its filepath.

2. Launch your terminal. On Windows, use the MinGW shortcut or `Tools/MSYS2/msys.bat` file to launch the MSYS2 terminal.

3. `cd` into the location of the copied example project.

4. Run the following command to build the example:

        make

    ???+ note "ℹ️ **Note: Improving Build Speed**"
        The following command can be used to enable parallel builds and drastically improve build speed:

            :::shell
            make -r -j --output-sync=target --no-print-directory

        - `-r` is an option that ignores some of Make's implicit rules to improve build speed.
        - `-j` enables parallel execution of the build in the maximum number of threads.

            ???+ warning "**⚠️ Parallel Builds**"
                Parallel builds can mangle the console output.  To deal with this, the `--output-sync=target` option can be used.  However, _this is only available in Make version 4 or higher_.  When this option is used, `--no-print-directory` is also used to declutter the build output.


    Expected output:

        :::bash
        Loaded project.mk
        CC    main.c
        CC   /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/Source/board.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/stdio.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/LED/led.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/PushButton/pb.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Display/adafruit_3315_tft.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Touchscreen/adafruit_3315_touch.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Camera/camera.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Camera/mipi_camera.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Camera/ov7692.c
        CC    /home/msdk/Libraries/Boards/MAX78002/EvKit_V1/../../../MiscDrivers/Camera/sccb.c
        AS    /home/msdk/Libraries/CMSIS/Device/Maxim/MAX78002/Source/GCC/startup_max78002.S
        CC    /home/msdk/Libraries/CMSIS/Device/Maxim/MAX78002/Source/heap.c
        CC    /home/msdk/Libraries/CMSIS/Device/Maxim/MAX78002/Source/system_max78002.c
        LD    /home/msdk/Examples/MAX78002/Hello_World/build/max78002.elf
        arm-none-eabi-size --format=berkeley /home/msdk/Examples/MAX78002/Hello_World/build/max78002.elf
        text    data     bss     dec     hex filename
        35708    2504    1156   39368    99c8 /home/msdk/Examples/MAX78002/Hello_World/build/max78002.elf

5. Connect a debug adapter between the host PC and the evaluation platform. Detailed instructions on this hardware setup can be found in the evaluation platform's Datasheet and Quick-Start Guide, which are available on its [analog.com](https://analog.com) product page.

6. Flash and run the program with OpenOCD.

        :::shell
        make flash.openocd

    ???+ note "ℹ️ **Note: Flashing with Make**"
        The command `make flash.openocd` is a build target added to the MSDK as of the [June 2023 Release](https://github.com/analogdevicesinc/msdk/releases/tag/v2023_06) to make flashing over the command-line easier.  It launches and drives an OpenOCD server behind the scenes to flash the project's binary.  See the `Tools/Flash/flash.mk` file for implementation details, and [Flashing on the Command-Line](#flashing-on-the-command-line) for more details on launching debug server/clients manually.

    Expected output:

        :::bash
        Open On-Chip Debugger 0.11.0+dev-g4cdaa275b (2022-03-02-09:57)
        Licensed under GNU GPL v2
        For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
        DEPRECATED! use 'adapter driver' not 'interface'
        Info : CMSIS-DAP: SWD supported
        Info : CMSIS-DAP: Atomic commands supported
        Info : CMSIS-DAP: Test domain timer supported
        Info : CMSIS-DAP: FW Version = 0256
        Info : CMSIS-DAP: Serial# = 044417016af50c6500000000000000000000000097969906
        Info : CMSIS-DAP: Interface Initialised (SWD)
        Info : SWCLK/TCK = 1 SWDIO/TMS = 1 TDI = 0 TDO = 0 nTRST = 0 nRESET = 1
        Info : CMSIS-DAP: Interface ready
        Info : clock speed 2000 kHz
        Info : SWD DPIDR 0x2ba01477
        Info : max32xxx.cpu: Cortex-M4 r0p1 processor detected
        Info : max32xxx.cpu: target has 6 breakpoints, 4 watchpoints
        Info : starting gdb server for max32xxx.cpu on 3333
        Info : Listening on port 3333 for gdb connections
        Info : SWD DPIDR 0x2ba01477
        target halted due to debug-request, current mode: Thread
        xPSR: 0x81000000 pc: 0x0000fff4 msp: 0x20003ff0
        ** Programming Started **
        ** Programming Finished **
        ** Verify Started **
        ** Verified OK **
        ** Resetting Target **
        Info : SWD DPIDR 0x2ba01477
        shutdown command invoked

7. The program has been flashed and the target micro has been reset.  The flashed program should now be running.  For the `Hello_World` example, an LED on the board should be blinking.

    ???+ note "ℹ️ **Note**"
        See [Command-Line Development](#command-line-development) for additional more detailed documentation.

## Visual Studio Code

Support for [Visual Studio Code](https://code.visualstudio.com/) is maintained for the MSDK and developed on the [VSCode-Maxim](https://github.com/analogdevicesinc/VSCode-Maxim) GitHub repository.

For setup/quick-start instructions, see ["Getting Started with Visual Studio Code"](#getting-started-with-visual-studio-code) first.  This section offers detailed usage info focusing on the typical development cycle.

### Opening Example Projects

Visual Studio Code is built around a "working directory" paradigm. The editor is always rooted in a working directory, and the main mechanism for changing that directory is **File -> Open Folder...**

![File -> Open Folder](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/file_openfolder.JPG)

As a result, you'll notice that there is no "New Project" mechanism. A "project" in VS Code is simply a folder. It will look inside the opened folder for a `.vscode` _sub_-folder to load project-specific settings from.

![Example Directory Contents](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/opening_projects_2.jpg)

_(Note:  You may need to enable viewing of hidden items in your file explorer to see the .vscode sub-folder)._

To open a project:

1. Launch Visual Studio Code.

2. Select **File -> Open Folder...**

    ![File -> Open Folder](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/file_openfolder.JPG)

3. Navigate to an example project for the target microcontroller in the MSDK's `Examples` folder.

    ![Figure 43](res/Fig43.jpg)

    ???+ warning "**⚠️ Copying Examples**"
        It's strongly recommended to copy example projects to an _outside_ folder before modifying them.  This keeps the MSDK's "source" copy preserved for reference.  Project folders must be copied to a location _without_ any spaces in its filepath.

4. VS Code will prompt for trust the first time. Select _Trust folder and enable all features_

    ![Trust Prompt](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/workspaceTrustPrompt.JPG)

5. The opened project should look something like this.

    ![Figure 16](res/Fig16.jpg)

6. Verify the **_Board Support Package_** for the project is set correctly.

### How to Set the BSP (VS Code)

To set the BSP for an open project:

1. Set the `"board"` [project configuration](https://github.com/analogdevicesinc/VSCode-Maxim/tree/main#project-configuration) option in `.vscode/settings.json`, which maps to the `BOARD` _[Build Configuration Variable](#build-tables)_.

    See [Board Support Packages](#board-support-packages) for a table of possible values.

    ![Figure 17](res/Fig17.jpg)

2. **Reload the VS Code window** to re-index its Intellisense engine.

    VS Code can be conveniently reloaded with the **Reload Window** developer command accessed with **`CTRL + SHIFT + P`** (or **`COMMAND + SHIFT + P`** on MacOS).

    ![Reload window](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/reload_window.JPG)

### Build Tasks

Once a project is opened 4 available build tasks will become available via `Terminal > Run Build task...` or the shortcut `Ctrl+Shift+B`.  These tasks are configured by the `.vscode/task.json` file.

![Build Tasks Image](https://raw.githubusercontent.com/analogdevicesinc/VSCode-Maxim/main/img/buildtasks.JPG)

#### Build

* Compiles the code with a `make all` command.
* Additional options are passed into Make on the command-line based on the project's settings.json file.
* The `./build` directory will be created and will contain the output binary, as well as all intermediary object files.
* Notice the **`TARGET`**, **`BOARD`** , and **`PROJECT`** Build Configuration Variables being set on the command line, and the program binary successfully compiled into the `.elf` program binary in the **build** sub-folder of the project.

    ![Figure 18](res/Fig18.jpg)

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

### Flashing and Debugging

This section assumes a debugger is connected between the host PC and the evaluation platform. For more detailed instructions on this hardware setup, refer to the evaluation platform's Datasheet and Quick-Start Guide, which are available on its [analog.com](https://analog.com) product page.

#### Arm Core Debugging

1. Run the **`flash`**  [build task](#build-tasks).  Running this task will automatically build the project if needed, flash the program binary, and halt the program execution to await a debugger connection.

    **Flashing does not happen automatically when launching the debugger**. This is an intentional design choice for VS Code to allow the debugger to quickly restart the program under debug without a lengthy re-flash procedure.

    ![Figure 19](res/Fig19.jpg)

2. Open the **Run and Debug** window (**`CTRL+SHIFT+D`**) and select the `Debug Arm (Cortex-debug)` profile.

    ![Figure 20](res/Fig20.jpg)

    ![Figure 46](res/Fig46.jpg)

3. Verify the program counter enters `main` successfully.

    ![Figure 21](res/Fig21.jpg)

4. Press **Continue** (**`F5`**) to run the program.  The debugger control bar can be used to exercise the debugger further.

    ![Debugger Control Bar Image](https://raw.githubusercontent.com/MaximIntegratedTechSupport/VSCode-Maxim/main/img/debugger_bar.JPG)

        Continue | Step Over | Step Into | Step Out | Restart | Stop

#### Breakpoints

Breakpoints can be set by clicking next to a line number in VS Code's editor. They are removed by clicking on them again.

![Figure 35](res/Fig35.jpg)

Additionally _conditional_ breakpoints can be added by _right-clicking_ on a line.

![Figure 36](res/Fig36.jpg)

The condition and condition type can be modified with the dropdown. This is useful for setting a breakpoint on a certain value in a `for` loop iterator or when a specific bit in a register is set, for example.

![Figure 37](res/Fig37.jpg)

#### Peripheral Browsing

A peripheral browser lets you quickly view the formatted register-level contents of the peripheral blocks on a target microcontroller under debug.

As of the [v1.6.0](https://github.com/analogdevicesinc/VSCode-Maxim/releases/tag/v1.6.0) VSCode-Maxim project files, pre-made [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) launch profiles are included in each project.  These profiles enable peripheral browsing via an embedded "Cortex Peripherals"window.

![Figure 47](res/Fig47.jpg)

Alternatively, *watch expressions* can be used.  These can be set for registers and variables. (For example, the `sysctrl` register below).

![image](https://user-images.githubusercontent.com/38844790/177819247-ccf90782-e1e6-44f2-8605-c39fc15c09a6.png)

- Adding **`,b`** lets you print out the value in **binary**
- Adding **`,x`** prints the value in **hex**.
- Standard **logical** and **bitwise** operations are supported inside the watch expression.
- **Register** and **variable** values can be **modified** through these same watch-points.  _(Right click -> Set Value)_

It should be noted that the debugger's watch points are *contextual*, meaning that its symbol look-ups will depend on the active point in your program.

#### Disassembly View

Stepping through disassembly is supported and enabled by the [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) launch profile.

![Figure 48](res/Fig48.jpg)

To open the disassembly view:

1. [Launch a debug session](#arm-core-debugging) with the `Debug Arm (Cortex-Debug)` profile.

2. Open the developer command prompt with `CTRL + SHIFT + P`.

3. Run the "Open Disassembly View" developer command.

    ![Figure 49](res/Fig49.jpg)

    Alternatively, right click -> "Open Disassembly View"

    ![Figure 50](res/Fig50.jpg)

4. The debugger will step through whichever window has the active focus.  Set the focus to the disassembly window to step through the assembly code.

See the [Cortex-Debug Wiki](https://github.com/Marus/cortex-debug/wiki/Disassembly-Debugging) for more details.

#### Dual Core Debugging

For microcontrollers with _both_ an Arm M4 and a RISC-V core, the _GDB (RISC-V)_ launch profile is provided to enable RISC-V debugging.

???+ note "ℹ️ **Note**"
    The RISC-V core requires setup and handoff from the Arm M4 core. As a result, this is an advanced configuration requiring a unique combination of the project's source code, Makefiles, and VSCode-Maxim project settings. Such projects are appended with the `-riscv` suffix in the project's folder name.

This section demonstrates how to debug `-riscv` projects in VS Code using the [mnist-riscv](Examples/MAX78000/CNN/mnist-riscv) project for the MAX78000 as an example.

1. Connect _both_ your Arm (SWD) and RISC-V (JTAG) debuggers. VSCode-Maxim projects come pre-configured to use the [ARM-USB-OCD-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-OCD-H/) + [ARM-JTAG-20-10](https://www.olimex.com/Products/ARM/JTAG/ARM-JTAG-20-10/) adapters for the RISC-V JTAG port. Ex:

    ![Debugger-EvKIT Connections](https://user-images.githubusercontent.com/38844790/190749647-f4ef066a-afcc-4749-bb9c-53c7e33e2cf9.jpg)

    ???+ warning "**⚠️ Olimex Drivers**"
        If connection issues occur with the Olimex adapter, verify that the drivers are installed correctly.  See Section 3.3.3 of the [Olimex User Manual](https://www.olimex.com/Products/ARM/JTAG/_resources/ARM-USB-OCD_and_OCD_H_manual.pdf).  The [Zadig](https://zadig.akeo.ie/) tool to install WinUSB drivers.

2. [Open](#opening-example-projects) the project in VS Code.

3. Run the "Flash" task.

    ![image](https://user-images.githubusercontent.com/38844790/168398354-2ac2961b-6d45-4f84-8805-0ab5339a4b98.png)

4. Launch the debugger using the **Debug Arm (Cortex-Debug)** or **GDB (Arm M4)** profile **first**:

    ![Figure 46](res/Fig46.jpg)

    ... which should hit the breakpoint in `main.c`...
    ![image](https://user-images.githubusercontent.com/38844790/168398503-0f2ae9c1-f535-4d41-aed9-9d9e19b16303.png)

5. **Continue** the debugger.  The code in `main.c` will boot up the RISC-V core. You can optionally set a breakpoint on `WakeISR` to see when the RISC-V core has signaled it's ready.

    ![image](https://user-images.githubusercontent.com/38844790/168398665-9486e1b6-73bd-481e-a4b5-15dd44c7d7b9.png)

6. Now, switch the debugger profile to the **GDB (RISC-V) profile** and launch it. This will launch an additional instance on a separate port and connect to the Olimex adapter.

    ![image](https://user-images.githubusercontent.com/38844790/168398707-b6771bf3-b6bf-47a2-b963-b0b9fc003ca4.png)

    ???+ note "ℹ️ **Note: Signal 0 Exception**"
        The "Signal 0" exception below is a known issue caused by a signaling mismatch between the RISC-V core and VS Code's debugger engine. The exception message is harmless and can be safely ignored.

        ![image](https://user-images.githubusercontent.com/38844790/168399130-95fe7539-fb46-4c06-a268-6b720403b539.png)

7. From here, the debugger should be fully functional. The Arm vs. RISC-V debugger instance can be selected with the dropdown on the debugger control bar.

    ![image](https://user-images.githubusercontent.com/38844790/168399419-d0488a0e-2068-4cc7-9108-0a296fdc04b4.png)

### Project Settings

`.vscode/settings.json` is the main project configuration file.  Values set here are parsed into the other .json config files.

**When a change is made to this file, VS Code should be reloaded with CTRL+SHIFT+P -> Reload Window (or alternatively restarted completely) to force a re-parse.**

![Reload Window](https://raw.githubusercontent.com/analogdevicesinc/VSCode-Maxim/main/img/reload_window.JPG)

The default project configuration should work for most use cases as long as [`"target"`](#target) and [`"board"`](#board) are set correctly.

???+ note "ℹ️ **Note**"
    Any field from `settings.json` can be referenced from any other VS Code config file (including itself) with `"${config:[fieldname]}"`

The following configuration options are available:

#### `"MAXIM_PATH"`

* This option must point to the root installation directory of the MSDK.
* It should be placed in the _global_ user settings.json file during first-time VSCode-Maxim setup.  See [Getting Started with Visual Studio Code](#getting-started-with-visual-studio-code).

#### `"target"`

* This sets the target microcontroller for the project.
* It sets the `TARGET` [Build Configuration](#build-configuration-variables) variable.

#### `"board"`

* This sets the target board for the project (ie. Evaluation Kit, Feather board, etc.)
* See [How to Set the BSP (VS Code)](#how-to-set-the-bsp-vs-code)

#### `"terminal.integrated.env.[platform]:Path"`

* This prepends the location of the MSDK toolchain binaries to the system `Path` used by VSCode's integrated terminal.
* The Path is not sanitized by default, which means that the terminal inherits the system path.

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
* Default value: `"12.2.0-3.1"`

#### `"OCD_path"`

* Where to find the OpenOCD.
* Default value: `"${config:MAXIM_PATH}/Tools/OpenOCD"`

#### `"ARM_GCC_path"`

* Where to find the Arm Embedded GCC Toolchain.
* Default value: `"${config:MAXIM_PATH}/Tools/GNUTools/${config:v_Arm_GCC}"`

#### `"xPack_GCC_path"`

* Where to find the RISC-V GCC Toolchain.
* Default value: `"${config:MAXIM_PATH}/Tools/xPack/riscv-none-elf-gcc/${config:v_xPack_GCC}"`

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

### Project Creation

#### Option 1.  Copying a Pre-Made Project

Copying a pre-made example project is a great way to get rolling quickly, and is currently the recommended method for creating new projects.

The release package for this project (Located at `Tools/VSCode-Maxim` in the MSDK) contains a `New_Project` folder designed for such purposes. Additionally, any of the VS Code-enabled Example projects can be copied from the MSDK.

1. Copy the existing project folder to an accessible location.  This will be the location of your new project.

    ???+ warning "**⚠️ Warning**"
        The full path to the project must _not_ have any spaces in it.

2. (Optional) Rename the folder.  For example, I might rename the folder to `MyProject`.

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your [target](#target) microcontroller and [board](#board) correctly.

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. That's it!  The existing project is ready to build, debug, and modify.

#### Option 2 - Injecting

VSCode-Maxim releases provide the `Inject` folder for "injecting" into an existing folder.  If you want to start from scratch or use the project files with existing source code, take this option.

1. Create your project folder if necessary.  For example, I might create a new project in a workspace folder with the path: `C:/Users/Jake.Carter/workspace/MyNewProject`.

2. Copy the **contents** of the `Inject` folder into the project folder from step 1.  The contents to copy include a `.vscode` folder, a `Makefile`, and a `project.mk` file.  For this example, the contents of the 'MyProject' folder would be the following:

        :::shell
        C:/Users/Jake.Carter/workspace/MyNewProject
        |- .vscode
        |- Makefile
        |- project.mk

3. Open the project in VS Code (`File -> Open Folder...`)

4. Set your [target](#target) microcontroller and [board](#board) correctly.

5. `CTRL+SHIFT+P -> Reload Window` to re-parse the project settings.

6. Configure the [build system](#build-system) for use with any pre-existing source code.

7. That's it!  Your new project can now be opened with `File > Open Folder` from within VS Code.

## Eclipse

For setup/quick-start instructions, see ["Getting Started with Eclipse"](#getting-started-with-eclipse) first.  This section offers detailed usage info focusing on the typical development cycle.

### Running Eclipse

Eclipse _must_ be launched with the **Eclipse MaximSDK** shortcut. The shortcut points to the `Tools/Eclipse/cdt/eclipse(.bat/.sh)` file, which configures Eclipse's system environment for use with the MSDK toolchain.

![Figure 22](res/Fig22.jpg)

When Eclipse is launched, it will prompt for a **_workspace_** location. This is a local folder that Eclipse will copy its projects into.

![Figure 39](res/Fig39.jpg)

### Creating a New Project

1. [Launch](#running-eclipse) Eclipse.

2. Ensure that the Eclipse is set to the **C/C++ perspective** in the top right corner. Otherwise, the new project wizard will not show up.

3. Navigate to **File -> New -> Maxim Microcontrollers**.

    ![Figure 31](res/Fig31.jpg)

4. Enter the project name and hit **Next**.

    ![Figure 32](res/Fig32.jpg)

5. Follow the new project wizard.

    - Chip type selects the _Target Microcontroller_
    - Board type selects the [_Board Support Package (BSP)_](#board-support-packages)
    - Example type selects the example project to be copied as the template for the new project.
    - Adapter type selects the debug adapter to use.

    ![Figure 33](res/Fig33.jpg)

6. Select **Finish** to create the new project.

### Importing Examples

1. [Launch](#running-eclipse) Eclipse.

2. Use **File -> Import** to open the import wizard.

3. Select **General -> Existing Projects into Workspace** and hit **Next**.

    ![Figure 23](res/Fig23.jpg)

4. **Browse** to the [`Examples`](https://github.com/analogdevicesinc/msdk/tree/main/Examples) folder in the MSDK installation for your target microcontroller and select the example projects to import into the workspace.

    ![Figure 24](res/Fig24.jpg)

5. Ensure that **Copy projects into workspace** is selected. This will copy the projects out of the MSDK and leave the originals unmodified.

6. Select **Finish** to import the project(s).

7. The projects should now show up in the Project Explorer.

    ![Figure 25](res/Fig25.jpg)

### How to Set the BSP (Eclipse)

[Imported](#importing-examples) Eclipse projects files are configured for the **EVKIT**-type _BSP_ by default. To set the BSP:

1. Right click the project name and select _Properties_.  Navigate to **C/C++ Build -> Environment**.
2. Set the **`BOARD`** _[Build Configuration Variable](#build-tables)_ to match the target evaluation platform.

    See [Board Support Packages](#board-support-packages) for a table of possible values.

    ![Figure 26](res/Fig26.jpg)

3. **clean** and rebuild the project.

### Building a Project

1. Ensure that the Eclipse is set to the **C/C++ perspective** (top right).

2. Select the correct project in the **Launch Configuration** dropdown.

3. Use the **Build** hammer button (top left) to build the project.

    ![Figure 27](res/Fig27.jpg)

### Flashing and Debugging

1. Connect a debug adapter between the host PC and the evaluation platform. For more detailed instructions on this hardware setup, refer to the evaluation platform's Datasheet and Quick-Start Guide, which are available on its [analog.com](https://analog.com) product page.

2. Ensure the correct project in the **Launch Configuration** dropdown is selected in **Debug** mode.

3. Use the **Debug** button (top left) to flash the program binary and connect the debugger.

    ![Figure 28](res/Fig28.jpg)

4. The Eclipse view will switch to debug mode, and the debugger will break on entry into the main.

    ![Figure 29](res/Fig29.jpg)

5. **Resume** the program (**`F8`**) using the top control bar and exercise the debugger.

    ![Figure 30](res/Fig30.jpg)

6. **Terminate** the debugger (**`CTRL+F2`**) when finished.

#### Segger J-Link Setup Guide (Eclipse)

Eclipse offers built-in support for Segger J-Link debuggers.  J-Link debugging can be enabled following the steps below:

1. Download and install the latest Segger J-Link Software and Documentation from [**here**](https://www.segger.com/downloads/jlink/)

2. Follow the instructions from the Segger J-Link Eclipse plugin [**here**](https://eclipse-embed-cdt.github.io/debug/jlink/) with the following modifications specific to the MSDK.  Other options an be left at their defaults.

    1. Modify the Executable name under "GDB Client Setup" to `arm-none-eabi-gdb${cross_suffix}`

        ![Figure 44](res/Fig44.jpg)

    2. Modify the "Startup" options to issue a `monitor reset halt` under initialization commands and _uncheck_ `Pre-run/Restart reset`

        ![Figure 45](res/Fig45.jpg)

## Keil MDK

The [Keil MDK Microcontroller Development Kit](https://www2.keil.com/mdk5) is developed and maintained by Arm.  ADI maintains CMSIS Pack files supporting this environment.

Supporting documentation is maintained by Arm, and can be found on the [**MDK5 page**](https://www2.keil.com/mdk5).  The latest pack files can be found under the "Maxim" section of the [device list](https://www.keil.com/dd2/).

## IAR Embedded Workbench

IAR Embedded Workbench is a third-party IDE that requires a software license.  ADI maintains support files for this IDE in the form of CMSIS Pack files.

Supporting documentation is maintained by IAR, and can be found on the [**Embedded Workbench Product Page**](https://www.iar.com/products/architectures/arm/iar-embedded-workbench-for-arm/) under "User Guides and documentation".

## Command-Line Development

This section offers more detailed info on command-line development.

For setup/quick-start, see ["Getting Started with Command-Line Development"](#getting-started-with-command-line-development).

### How to Set the BSP (Command-Line)

- To _persistently_ the BSP, set the **`BOARD`** _[Build Configuration Variable](#build-configuration-variables)_ by editing the **project.mk** that can be found inside each project.

        :::makefile
        # This file can be used to set build configuration
        # variables. These variables are defined in a file called
        # "Makefile" that is located next to this one.

        # For instructions on how to use this system, see
        # https://analogdevicesinc.github.io/msdk/USERGUIDE/

        # **********************************************************

        # Add your config here!

        BOARD=FTHR_RevA # Set the BSP for the MAX78000FTHR

- Alternatively, set **`BOARD`** on the command line when building (i.e., `make -r -j BOARD=FTHR_RevA`) to set/override the BSP for a single build.

### Building on the Command-Line

1. `cd` into the project folder.

2. Run `make`

   - **Parallel Build** (fastest build, but console message formatting may be mangled):

        make -r -j

   - **Serial Build**

        make -r

3. Take note of the output filename and location, which by default is the lowercase name of the _Target microcontroller_ and created in the `build` folder.

### Cleaning on the Command-Line

1. `cd` into the project folder.
2. Run `make clean`
   - **Project clean**: `make clean` deletes the project `build` folder and all of its contents.
   - **Library clean**: `make distclean` can be used to clean out _all_ build products, including the project `build` folder and all [peripheral driver](#peripheral-driver-api) libraries.

### Flashing on the Command-Line

???+ note "ℹ️ **A Note on Flashing**"
    The commands below are not a comprehensive list of all the possible options for flashing.  They are the most common and useful ones.  For full documentation, see the "Flash Programming" section of the [**OpenOCD User Manual**](https://openocd.org/doc/pdf/openocd.pdf)

1. [Build](#building-on-the-command-line) the project.

2. Connect a debug adapter between the host PC and the evaluation platform. For more detailed instructions on this hardware setup, refer to the evaluation platform's Datasheet and Quick-Start Guide, which are available on its [analog.com](https://analog.com) product page.

3. Flash the program using `openocd`.  It's recommended to use the `make` command below for convenience.

    - ???+ note "ℹ️ **Flashing with Make**"

                :::shell
                make flash.openocd

            Expected output:

                :::bash
                Open On-Chip Debugger 0.11.0+dev-g4cdaa275b (2022-03-02-09:57)
                Licensed under GNU GPL v2
                For bug reports, read
                    http://openocd.org/doc/doxygen/bugs.html
                DEPRECATED! use 'adapter driver' not 'interface'
                Info : CMSIS-DAP: SWD supported
                Info : CMSIS-DAP: Atomic commands supported
                Info : CMSIS-DAP: Test domain timer supported
                Info : CMSIS-DAP: FW Version = 0256
                Info : CMSIS-DAP: Serial# = 044417016af50c6500000000000000000000000097969906
                Info : CMSIS-DAP: Interface Initialised (SWD)
                Info : SWCLK/TCK = 1 SWDIO/TMS = 1 TDI = 0 TDO = 0 nTRST = 0 nRESET = 1
                Info : CMSIS-DAP: Interface ready
                Info : clock speed 2000 kHz
                Info : SWD DPIDR 0x2ba01477
                Info : max32xxx.cpu: Cortex-M4 r0p1 processor detected
                Info : max32xxx.cpu: target has 6 breakpoints, 4 watchpoints
                Info : starting gdb server for max32xxx.cpu on 3333
                Info : Listening on port 3333 for gdb connections
                Info : SWD DPIDR 0x2ba01477
                target halted due to debug-request, current mode: Thread
                xPSR: 0x81000000 pc: 0x0000fff4 msp: 0x20003ff0
                ** Programming Started **
                ** Programming Finished **
                ** Verify Started **
                ** Verified OK **
                ** Resetting Target **
                Info : SWD DPIDR 0x2ba01477
                shutdown command invoked

            This command is a build target added to the MSDK as of the [June 2023 Release](https://github.com/analogdevicesinc/msdk/releases/tag/v2023_06) to make flashing over the command-line easier.  It will **flash** _and_ **run** the project with OpenOCD.  See the `Tools/Flash/flash.mk` file for implementation details.

    - ???+ note "ℹ️ **OpenOCD Flash & Hold**"
        The following command template can be used if you just want to flash the program with OpenOCD manually, and halt the target micro.  This is used when you want to start a command-line debugging session.

                :::shell
                openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/<target>.cfg -c "program build/<filename>.elf verify; init; reset halt"

            - ???+ note "**ℹ️ `-s $MAXIM_PATH/Tools/OpenOCD/scripts`**"
                This option tells OpenOCD to search the `Tools/OpenOCD/scripts` folder of the MSDK installation for files.
                ???+ warning "**⚠️ Warning: Windows**"
                    On Windows you should use `%MAXIM_PATH%` (Command Prompt) or `$env:MAXIM_PATH` (PowerShell) to dereference the `MAXIM_PATH` environment variable

            - ???+ note "**ℹ️ `-f target/<target>.cfg`**"
                This option loads an OpenOCD config file for the _target microcontroller_.  Supported options can be found in the `Tools/OpenOCD/scripts/target` folder.
                ???+ warning "⚠️**Change `<target>` to match the target micro**"

            - ???+ note "**ℹ️ `-f interface/cmsis-dap.cfg`**"
                This option loads an OpenOCD config file for the MAX32625PICO SWD debugger that is included with most EVKITs.  You may need to change this option for other debuggers. Supported options can be found in the `Tools/OpenOCD/scripts/interface` folder.

            - ???+ note "`-c "program build/<filename>.elf verify; init; reset halt"`"
                This command flashes the program binary (`program`), performs a flash verification (`verify`), initializes the connection to the target micro (`init`), and finally resets/halts the micro to prepare for debug (`reset halt`).

            Expected output:

                :::bash
                Open On-Chip Debugger 0.11.0+dev-g4cdaa275b (2022-03-02-09:57)
                Licensed under GNU GPL v2
                For bug reports, read
                    http://openocd.org/doc/doxygen/bugs.html
                DEPRECATED! use 'adapter driver' not 'interface'
                Info : CMSIS-DAP: SWD supported
                Info : CMSIS-DAP: Atomic commands supported
                Info : CMSIS-DAP: Test domain timer supported
                Info : CMSIS-DAP: FW Version = 0256
                Info : CMSIS-DAP: Serial# = 044417016af50c6500000000000000000000000097969906
                Info : CMSIS-DAP: Interface Initialised (SWD)
                Info : SWCLK/TCK = 1 SWDIO/TMS = 1 TDI = 0 TDO = 0 nTRST = 0 nRESET = 1
                Info : CMSIS-DAP: Interface ready
                Info : clock speed 2000 kHz
                Info : SWD DPIDR 0x2ba01477
                Info : max32xxx.cpu: Cortex-M4 r0p1 processor detected
                Info : max32xxx.cpu: target has 6 breakpoints, 4 watchpoints
                Info : starting gdb server for max32xxx.cpu on 3333
                Info : Listening on port 3333 for gdb connections
                Info : SWD DPIDR 0x2ba01477
                target halted due to debug-request, current mode: Thread
                xPSR: 0x81000000 pc: 0x0000fff4 msp: 0x20003ff0
                ** Programming Started **
                ** Programming Finished **
                ** Verify Started **
                ** Verified OK **
                Info : Listening on port 6666 for tcl connections
                Info : Listening on port 4444 for telnet connections
                # Note: OpenOCD is now waiting for a GDB client connection

### Debugging on the Command-Line

1. [Flash](#flashing-on-the-command-line) the program using the **Flash and Hold** command above.

2. Launch an **_new_ separate terminal**.

    ???+ warning "⚠️ On **Windows**, use the MinGW shortcut or `Tools/MSYS2/msys.bat` file to launch the MSYS2 terminal."

3. `cd` into the location of the copied example project.

4. Run the following command to launch a **GDB *client***.

        arm-none-eabi-gdb --se=build/<filename>.elf

    - ???+ note "**ℹ️ `--se=build/<filename>.elf`**"
        This sets the symbol and executable file to the compiled program file.
        ???+ warning "⚠️ **Change this to match the build output filename.**"

    Expected output:

        :::bash
        GNU gdb (GNU Arm Embedded Toolchain 10.3-2021.10) 10.2.90.20210621-git
        Copyright (C) 2021 Free Software Foundation, Inc.
        License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
        This is free software: you are free to change and redistribute it.
        There is NO WARRANTY, to the extent permitted by law.
        Type "show copying" and "show warranty" for details.
        This GDB was configured as "--host=i686-w64-mingw32 --target=arm-none-eabi".
        Type "show configuration" for configuration details.
        For bug reporting instructions, please see:
        <https://www.gnu.org/software/gdb/bugs/>.
        Find the GDB manual and other documentation resources online at:
            <http://www.gnu.org/software/gdb/documentation/>.

        For help, type "help".
        Type "apropos word" to search for commands related to "word"...
        Reading symbols from build/max78002.elf...
        (gdb)

    ???+ note "**ℹ️ Note**"
        The terminal is now in an interactive GDB client window.  It accepts GDB commands.  Run `help` at any time, or see [Common GDB Commands](#common-gdb-commands) in this document.

5. Connect the GDB Client to the OpenOCD server with the following command.

        target extended-remote localhost:3333

    Expected output:

        :::bash
        Remote debugging using localhost:3333
        0x0000fff4 in ?? () # Note: ?? may be present at this stage, which is OK.

6. Reset the target microcontroller.

        monitor reset halt

    Expected output:

        :::bash
        SWD DPIDR 0x2ba01477
        target halted due to debug-request, current mode: Thread
        xPSR: 0x81000000 pc: 0x0000fff4 msp: 0x20003ff0

7. Set a breakpoint on `main`.

        b main

    Expected output:

        :::bash
        Breakpoint 1 at 0x10000224: file main.c, line 62.
        Note: automatically using hardware breakpoints for read-only addresses.

8. Continue the debugger.

        continue

    Expected output (for the Hello World example):

        :::bash
        Continuing.

        Breakpoint 1, main () at main.c:62
        62     printf("Hello World!\n");

9. (Optional) Continue exercising the debugger.

10. Quit GDB.

        quit

    Expected output:

        :::bash
        A debugging session is active.

        Inferior 1 [Remote target] will be detached.

        Quit anyway? (y or n) [answered Y; input not from terminal]
        Detaching from program: C:\Users\User\codespace\Hello_World\build\max78002.elf, Remote target
        [Inferior 1 (Remote target) detached]

11. Quit OpenOCD. In the terminal window running the OpenOCD _server_, press `CTRL + C` to issue the shutdown command.

### Common GDB Commands

| **Command**                     | **Short Command** | **Description**                                              |
| ------------------------------- | ----------------- | ------------------------------------------------------------ |
| `monitor halt`                  |                   | Halt the microcontroller.                                    |
| `monitor reset halt`            |                   | Reset the microcontroller and immediately halt.              |
| `monitor max32xxx mass_erase 0` |                   | Mass erase flash bank 0.                                        |
| `file <filename>`               |                   | Set the program file to `<filename>`                         |
| `load`                          |                   | Flash the current program file                               |
| `continue`                      | `c`               | Continue execution.                                          |
| `break <arg>`                   | `b <arg>`         | Set a breakpoint. `<arg>` can be a function name, file:line\_number, or address. |
| `print <variable>`              | `p`               | Print the value of a variable. The variable must be in the current scope. |
| `backtrace`                     | `bt`              | Print contents of the stack frame.                           |
| `step`                          | `s`               | Execute the next instruction.                                |
| `next`                          | `n`               | Execute the next line of code.                               |
| `finish`                        | `f`               | Continue to the end of the current function.                 |
| `info reg`                      |                   | Print the values of the ARM registers.                       |
| `help`                          |                   | Print descriptions for available commands                    |
| `help <cmd>`                    |                   | Print description for given command.                         |
| `quit`                          | `q`               | Quit the GDB client                                          |

## Build System

### Build System Overview

The **Build System** manages the compilation of source code into program binaries and offers a **Command-Line Interface (CLI)** for setting **Build Configuration Variables**. All IDEs interface with this system.

The Build System is managed by two files found in a project's root directory, one called **Makefile** and one called **project.mk**. These files are used by the [GNU Make](https://www.gnu.org/software/make/) program (which is a part of the MSDK toolchain) to locate and build a project's source code.

* **Makefile** is the "core" file and should not be edited directly. Instead, it exposes the **CLI** that can be accessed in the _project.mk_ file, on the command line, in your system's environment, or through your IDE. It also comes with a default configuration that is suitable for most projects.
* **project.mk** offers a convenient and stable access point for advanced build configuration, and this is the file that should be edited if necessary.

When the command

    make

is run from inside of a project folder, the program `make` will resolve any project-specific settings and then build the project's source code.

### Default Build Behavior

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

### Build Configuration Variables

A **Build Configuration Variable** is a [Makefile variable](https://www.gnu.org/software/make/manual/make.html#Using-Variables) and therefore follows the same rules. However, they have been streamlined to be made much easier to use, so most of the [official GNU Make documentation](https://www.gnu.org/software/make/manual/make.html) is only needed for advanced use cases.

#### How to Set a Build Configuration Variable

To set a **standard** configuration variable, **use the `=` syntax**...

    VARIABLE=VALUE

The **`=`** operator is used for _most_ configuration variables with a few exceptions (documented in the [reference table](#build-tables)) when a variable should contain a **_list_ of values**. In such cases, **use `+=` the syntax** to _add_ values to the list.

    VARIABLE+=VALUE1
    VARIABLE+=VALUE2

#### Where to Set a Build Configuration Variable

For most variables, you should set them in the **project.mk** file (exceptions are documented in the [reference table](#build-tables) and IDE-specific sections).

For example, to enable hardware floating-point acceleration for a project, the **`MFLOAT_ABI`** configuration variable can be used with a value of **`hard`**. The contents of **project.mk** might then look as follows:

(_Inside project.mk_)

    :::Make
    # This file can be used to set build configuration
    # variables. These variables are defined in a file called
    # "Makefile" that is located next to this one.

    # For instructions on how to use this system, see
    # https://analogdevicesinc.github.io/msdk/USERGUIDE/

    # **********************************************************

    MFLOAT_ABI=hard # Enable hardware floating point acceleration

It should also be noted that configuration variables can be set on the **command line** as well. For example

    make MFLOAT_ABI=hard

will have the same effect.

Additionally, **environment variables** can be used. For example (on Linux)

    export MFLOAT_ABI=hard

will set the hardware floating point acceleration as the default for all projects with an environment variable.

However, there is a _precedence hierarchy_ that should be taken into consideration.

#### Precedence Hierarchy

The precedence hierarchy for the value of a configuration variable is:

- **IDE/command-line > project.mk > environment variable > default value**

If a value is set in an IDE _and_ project.mk, the IDE's value will take precedence. However, the ["override" directive](https://www.gnu.org/software/make/manual/make.html#Override-Directive) can be used in project.mk to give it max precedence.

### Build Tables

The following sections present the available [Build Configuration Variables](#build-configuration-variables).

#### Primary Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MAXIM_PATH`           | (Optional) Specify the location of the MSDK                | This optional variable can be used to change where the Makefile looks for the MSDK installation. By default, the build system will attempt to locate the MSDK with a relative path. If a project is moved _outside_ of the SDK, this variable must be set to the absolute path of the MSDK installation. |
| `TARGET`               | Set the _Target Microcontroller_                           | **If you are using an IDE, set this variable in the IDE's settings instead of project.mk** |
| `BOARD`                | Set the _Board Support Package (BSP)_                      | **If you are using an IDE, set this variable in the IDE's settings instead of project.mk.**  See [Board Support Packages](#board-support-packages) for more details.  When you change this option, it's usually a good idea to fully clean your project, then rebuild. |
| `BSP_SEARCH_DIR`       | Set the directory to search for the _Board Support Package (BSP)_                      | By default, the `Libraries/Boards` folder of the MSDK is searched for the `TARGET` microcontroller.  This setting is useful for loading custom BSPs from outside of the MSDK.  When `LIB_BOARD=1`, the build system looks for the file path at `$(BSP_SEARCH_DIR)/$(BOARD)/board.mk`.<br>See [BSP Search Directory](#bsp-search-directory) for more details. |

#### Project Build Variables

The following variables deal with fundamental project tasks such as adding source code, include paths, changing the output filename, etc.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `VPATH`                | Where to search for source (.c/.cpp) files                      | **Use the `+=` operator with this variable**.  This controls where the Makefile will look for **source code** files. If `AUTOSEARCH` is enabled (which it is by default), all source code files in the directories specified by this option will be automatically added to the build. If `AUTOSEARCH` is disabled, this tells the Makefile where to look for the files specified by `SRCS`. |
| `IPATH`                | Where to search for header (.h) files                      | **Use the `+=` operator with this variable**.  This controls where the Makefile will look for **header** files. _Unlike_ the `VPATH` option, this is not related to `AUTOSEARCH`. Individual header files are _not_ ever manually added to the build. Instead, you only need to specify the _location_ of your header files. |
| `SRCS`                 | List of source (.c/.cpp) files to add to the build              | **Use the `+=` operator with this variable**. All of the files in this list will be added to the build. If `AUTOSEARCH` is enabled, this is most useful for adding the full absolute path to a singular source file to selectively add to the build. If `AUTOSEARCH` is disabled, _all_ of the source files for the project must be added to `SRCS`, and they must also all be located on an entry in `VPATH`. Otherwise, a full path relative to the Makefile must be used. |
| `AUTOSEARCH`           | Automatically search for source (.c/.cpp) files                 | Enable or disable the automatic detection of .c files on `VPATH` (enabled by default). Set to `0` to disable or `1` to enable. If auto-search is disabled, source files must be manually added to `SRCS`. |
| `PROJECT`              | Set the output filename                                    | This controls the output filename of the build.  File extensions should _not_ be included in the filename.  **For VS Code, you should use the [project_name](#project_name) advanced config option instead of project.mk.** |
| `PROJ_LIBS`            | Add a static library file (.a) to the project              | **Use the `+=` operator with this variable**.  Additional static libraries to link against can be added with this option.<br>It should be noted that static library files are named with the `lib<libraryname>.a` convention.  Only add `<libraryname>` to this variable.<br>Ex: Give a file called `libEXAMPLE.a`, write `PROJ_LIBS += EXAMPLE`<br>Additionally, ensure that the location of the library is added to `PROJ_LDFLAGS`.<br>Ex: `PROJ_LDFLAGS += -Lsome/library/search/directory` |

#### Build Variables for the Compiler

The following variables can be used to interface with the compiler to perform common tasks such as changing the optimization level, adding compiler definitions to the build, and changing floating point acceleration.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MXC_OPTIMIZE_CFLAGS`  | Set the optimization level                                 | See [Optimize Options](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html) for more details.  Normal builds will default to `-Og`, which is good for debugging, while release builds will default to `-O2`. |
| `PROJ_CFLAGS`          | Add compiler flags to the build                            | **Use the `+=` operator with this variable**.  Compiler flags can be added with this option, including compiler definitions. For each value, the same syntax should be used as if the compiler flag was passed in over the command line. These can include standard [GCC options](https://gcc.gnu.org/onlinedocs/gcc-10.4.0/gcc/Option-Summary.html#Option-Summary) and/or [ARM-specific](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) options. |
| `PROJ_AFLAGS`          | Add assemblers flag to the build                           | **Use the `+=` operator with this variable**.  Assembler flags can be added with this option. |
| `PROJ_OBJS`            | Add object files to the build                              | **Use the `+=` operator with this variable**.  If needed, object files (.o) can be added to the build with this option. |
| `DEBUG`                | Toggle extra debug information  | Set this to `1` to enable extra debug information at compile time.  This generally improves the reliability of debugging at some increase in code size.  Set to `0` to disable.

#### Build Variables for the Linker

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `LINKERFILE`           | Set the linkerfile to use                                  | A linkerfile is responsible for specifying the available memory banks, their layout, and the organization of program binaries memory.  The file should exist in `Libraries/CMSIS/Device/Maxim/TARGET/Source/GCC` in the MSDK, or it should be placed inside the root directory of the project. |
| `PROJ_LDFLAGS`         | Add a linker flag to the build                             | **Use the `+=` operator with this variable**.  Flags can be passed to the linker with this option. See [GCC Options for Linking](https://gcc.gnu.org/onlinedocs/gcc/Link-Options.html#Link-Options) |

#### Build Variables for Arm Cores

The following build variables are used to control options specific to the Arm Cortex-M4 core available.  They are available on all microcontrollers, and for all projects unless that project is built for a RISC-V core.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MFLOAT_ABI`           | Set the floating point acceleration level                  | Sets the floating-point acceleration level.  Permitted values are `hard`, `soft`, and `softfp` (default). To enable full hardware acceleration instructions, use `hard`, but keep in mind that _all_ libraries your source code uses must also be compiled with `hard`. If there is any conflict, you'll get a linker error. For more details, see `-mfloat-abi` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html). |
| `DEFAULT_OPTIMIZE_FLAGS` | Override the default extra optimization flags | Extra compiler optimization flags are added to the build.  They are defined in `Libraries/CMSIS/Device/Maxim/GCC/gcc.mk`.  These can be disabled entirely by setting this variable to empty (`DEFAULT_OPTIMIZE_FLAGS=`). |
| `DEFAULT_WARNING_FLAGS` | Override the default warning flags | Default flags controlling warning output are added in `Libraries/CMSIS/Device/Maxim/GCC/gcc.mk`.  These can be disabled entirely by setting this variable to empty (`DEFAULT_OPTIMIZE_FLAGS=`). |
| `MCPU`           | Set the processor type                  | Set the target ARM processor.  Directly maps to `-mcpu` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html).  This flag is handled by the MSDK and not typically changed manually. |
| `MFPU`           | Set the FPU architecture                  | Set the floating point unit (FPU) architecture.  Directly maps to `-mfpu` under [ARM Options](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html).  This flag is handled by the MSDK and not typically changed manually. |

#### Build Variables for RISC-V Cores

The following build variables are used for RISC-V development.  They are only available on microcontrollers with RISC-V cores.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `RISCV_CORE`           | Build a project for the RISC-V core                        | Set to `1` to convert an entire project to use the RISC-V toolchain.  Only available on microcontrollers with a RISC-V core. |
| `RISCV_LOAD`           | Compile and load project for the RISC-V core               | **Only available on the MAX32655, MAX32680, and MAX32690**.  Set to `1` compile the project specified by `RISCV_APP` for the RISC-V core and link it into the same binary as the current project.  Useful for dual-core projects.
| `RISCV_APP`            | Project folder to compile for the `RISCV_LOAD` option      | **Only available on the MAX32655, MAX32680, and MAX32690**.  This option specifies the project to build for the RISC-V core when `RISCV_LOAD` is enabled.  Must be a path relative to the project that enables `RISCV_LOAD`, or an absolute path. |
| `RISCV_PREFIX`         | Change the toolchain prefix                                | This option can be used to override the GCC toolchain prefix if needed.  For example, to use the legacy RISC-V toolchain `RISCV_PREFIX = riscv-none-embed` will attempt to compile with `riscv-none-embed-gcc`. |

#### Build Variables for Toggling Libraries

The following variables can be used to enable the [available libraries](#libraries) in the MSDK.  Each library may also offer its own build configuration variables when enabled, which are documented in the [libraries](#libraries) section.

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
| `LIB_SDHC`             | Include the SDHC library                                   | This option toggles the Secure Digital High Capacity (SDHC) library, which can be used to interface with SD cards. Additionally, it enables the [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) library, which implements a generic FAT filesystem. |
| `LIB_CLI`             | Include the MSDK's built-in CLI library                     | This option toggles the MSDK's built-in CLI library, which can be used to process received commands over UART. |
| `LIB_USS`             | Include the USS Library                                     | This option toggles the Unified Security Software library.  It is only available via NDA. |

#### Build Variables for the PeriphDrivers Library

The following variables are specific to the PeriphDrivers library.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `MXC_SPI_VERSION`            | Set the SPI drivers to use (default is `v1`) | The PeriphDrivers offer two versions of the SPI API in order to maintain backwards compatibility.  Acceptable values are `v1` (legacy) or `v2`.  See [The SPI V2 Developer Note](#spi-v2-library) for more details. |

#### Build Variables for Secure Boot Tools (SBTs)

For microcontrollers with a secure bootloader, the following build configuration variables can be used to enable integration with the Secure Boot Tools.  These are a suite of applications designed for use with microcontrollers that have secure bootloaders.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `SBT`                  | Toggle SBT integration                                     | Toggles integration with the [Secure Boot Tools (SBTs)](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download.html?swpart=SFW0015360C). These are a suite of applications designed for use with microcontrollers that have secure bootloaders. When this is enabled, some additional rules become available such as `make sla` and `make scpa`. Set to `0` to disable or `1` to enable. |
| `MAXIM_SBT_DIR`        | Where to find the SBTs                                     | This option can be used to manually specify the location of the SBTs. Usually, this is not necessary. By default, the `Tools/SBT` directory of the MaximSDK will be searched. If the [SBT installer](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download.html?swpart=SFW0015360C) is used, it will set the `MAXIM_SBT_DIR` environment variable to point to itself automatically. |
| `TARGET_SEC`           | Secure part number to use                                  | Some secure microcontrollers have multiple secure variants, and this option can be used to specify the variant to use with the SBTs.  Defaults are intelligently selected and can be found in `$(MAXIM_SBT_DIR)/SBT-config.mk` |
| `SCP_PACKETS`          | Where to build the scp_packets folder                      | Defaults to `build/scp_packets`                              |
| `TEST_KEY`             | Which test key to sign applications with                   | Defaults to `$(MAXIM_SBT_DIR)/devices/$(TARGET_SEC)/keys/maximtestcrk.key`, which is the Maxim test key that can be used for development. |

#### Build Variables Controlling the Output

The following build variables can be used to control how to build output is formatted.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `VERBOSE`              | Toggle verbose builds             | Set to `1` to enable a verbose build that prints exactly what the compiler is doing for each step.  This is useful for troubleshooting.
| `FORCE_COLOR`          | Force colorized compiler output   | By default, GCC will attempt to autodetect whether colorized output is supported or not.  Set to `1` to force color (equivalent to `PROJ_CFLAGS += -fdiagnostics-color=always`).  This is useful for forcing color in CI systems. |

## Board Support Packages

The MSDK supports multiple parts and evaluation platforms (see [supported parts](#supported-parts)) through **"Board Support Packages" (BSPs)**. For microcontrollers with multiple evaluation platforms, multiple BSPs will be available.

The role of a _BSP_ is to provide a hardware abstraction layer for the initialization and management of board-level hardware such as serial interfaces, pushbuttons, LEDs, external peripheral devices, TFT displays, etc. which will vary between evaluation platforms. The BSP abstraction layer also improves code portability to custom devices.

???+ note "ℹ️ **Note**"
    The first task when opening or creating any project is to ensure the BSP is set correctly.

### How to Set the BSP

To set the BSP for a project:

- In **VS Code**:  [How to Set the BSP (VS Code)](#how-to-set-the-bsp-vs-code)
- In **Eclipse**:  [How to Set the BSP (Eclipse)](#how-to-set-the-bsp-eclipse)
- **Command-Line** Development:  [How to Set the BSP (Command-Line)](#how-to-set-the-bsp-command-line)

### BSP Table

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

### Custom BSPs

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

#### BSP Search Directory

By default, the MSDK searches for BSPs in the `Libraries/Boards` folder for each microcontroller.  This can be changed using the `BSP_SEARCH_DIR` [build configuration variable](#build-configuration-variables), which allows users to load a BSP from a directory outside of the MSDK.  The MSDK also uses the `BOARD` variable in its search path.

For example, the configuration...

    :::Makefile
    # project.mk

    BSP_SEARCH_DIR = /home/username/mybsps
    # ^ "root" of the BSP search path
    BOARD = CustomBSP
    # "stem" of the BSP search path

... will attempt to load the `/home/username/msbsps/CustomBSP/board.mk` file.

#### Custom BSP Template

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

### Disabling BSPs

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

## Libraries

The MSDK contains a large number of libraries, both third-party and in-house. The main library is the [Peripheral Driver API](#peripheral-driver-api), but the MSDK also contains drivers for various _external_ components such as TFT displays, cameras, accelerometers, audio codecs, and other devices. Additionally, dedicated libraries for more complex _internal_ hardware peripherals such as USB, the SDHC interface, and the Cordio BLE stack are also available.  These usually build on _top_ of the Peripheral Driver API.

???+ note "ℹ️ **Note: Enabling Libraries**"
    Libraries can be enabled for a project with a convenient *toggle switch* provided by the build system (See [Build Variables for Toggling Libraries](#build-variables-for-toggling-libraries))).

### Peripheral Driver API

A microcontroller is made up of a Central Processing Unit (CPU) that is surrounded by additional _peripheral_ hardware blocks such as timers, memory controllers, UART controllers, ADCs, RTCs, audio interfaces, and many more. The **Peripheral Driver API** is an important core library in the MSDK that allows the CPU to utilize the microcontroller's hardware blocks over a higher-level **_Application Programming Interface (API)_**.

![Figure 38](res/Fig38.jpg)

#### API Documentation (PeriphDrivers)

The links below will open detailed API references for each microcontroller. Offline copies of these API references can also be found in the `Documentation` folder of the MSDK installation.

- [MAX32520 API](Libraries/PeriphDrivers/Documentation/MAX32520/index.html)

- [MAX32650 API](Libraries/PeriphDrivers/Documentation/MAX32650/index.html)

- [MAX32655 API](Libraries/PeriphDrivers/Documentation/MAX32655/index.html)

- [MAX32660 API](Libraries/PeriphDrivers/Documentation/MAX32660/index.html)

- [MAX32665-MAX32666 API](Libraries/PeriphDrivers/Documentation/MAX32665/index.html)

- [MAX32670 API](Libraries/PeriphDrivers/Documentation/MAX32670/index.html)

- [MAX32672 API](Libraries/PeriphDrivers/Documentation/MAX32672/index.html)

- [MAX32675 API](Libraries/PeriphDrivers/Documentation/MAX32675/index.html)

- [MAX32680 API](Libraries/PeriphDrivers/Documentation/MAX32680/index.html)

- [MAX32690 API](Libraries/PeriphDrivers/Documentation/MAX32690/index.html)

- [MAX78000 API](Libraries/PeriphDrivers/Documentation/MAX78000/index.html)

- [MAX78002 API](Libraries/PeriphDrivers/Documentation/MAX78002/index.html)

#### PeriphDrivers Organization

The Peripheral Driver API's source code is organized as follows:

- **Header files _(.h)_** can be found in the `Libraries/PeriphDrivers/Include` folder.
    - These files contain function _declarations_ for the API, describing the function prototypes and their associated documentation.
- **Source files _(.c)_** can be found in the `Libraries/PeriphDrivers/Source` folder.
    - These files contain the function _definitions_ for the API - the _implementations_ of the functions declared by the header files.

The _**implementation**_ files are further organized based on _**die type**_ and **_hardware revision_**. This is worth noting when browsing or debugging through the drivers.

- The **_die type_** files follow the **`_ESXX`** , **`_MEXX`** , or **`_AIXX`** naming convention.
    - These files' responsibility is to manage microcontroller-specific implementation details that may interact with other peripheral APIs _before_ ultimately calling the revision-specific files.  See [Die Types to Part Numbers](#die-types-to-part-numbers)

- The **_hardware revision_** files follow the **`_revX`** naming convention.
    - These files contain the _pure_ driver implementation for a peripheral block and typically interact with the hardware almost entirely at the register level.

#### Die Types to Part Numbers

The following table matches external part numbers to internal die types.  This is useful for browsing through the PeriphDrivers source code, which uses the die types.

- ???+ note "ℹ️ **Note: Die Types Table**"
    | Part Number | Die Type
    | -------- | ----------- |
    | MAX32520 | ES17 |
    | MAX32570 | ME13 |
    | MAX32650 | ME10 |
    | MAX32655 | ME17 |
    | MAX32660 | ME11 |
    | MAX32662 | ME12 |
    | MAX32665 | ME14 |
    | MAX32670 | ME15 |
    | MAX32672 | ME21 |
    | MAX32675 | ME16 |
    | MAX32680 | ME20 |
    | MAX32690 | ME18 |
    | MAX78000 | AI85 |
    | MAX78002 | AI87 |

---

### CMSIS-DSP

The CMSIS-DSP library provides a suite of common **Digital Signal Processing _(DSP)_** functions that take advantage of hardware accelerated _Floating Point Unit (FPU)_ available on microcontrollers with Arm Cortex-M cores. This library is distributed in the MSDK as a pre-compiled static library file, and the MSDK maintains a port of the official code examples in the **ARM-DSP** [Examples](https://github.com/analogdevicesinc/msdk/tree/main/Examples) folder for each microcontroller.

Please refer to the [CMSIS-DSP official documentation](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html) for more detailed documentation on the library functions and usage.

#### CMSIS-DSP Supported Parts

- All microcontrollers with a Cortex M4 core are supported.

---

### Cordio Bluetooth Low Energy

The Cordio Bluetooth Low Energy (BLE) library provides a full BLE stack for microcontrollers with an integrated BLE controller.

The Cordio library warrants its own separate documentation. See the **[Cordio BLE User Guide](Libraries/Cordio/docs/CORDIO_USERGUIDE.md)**.

#### Cordio Supported Parts

- MAX32655
- MAX32665
- MAX32680
- MAX32690

---

### MAXUSB

The MAXUSB library provides a higher-level interface for utilizing the built-in USB controller hardware available on some microcontrollers. This allows the microcontroller to enumerate as a USB device without the need for an external USB controller IC.

#### MAXUSB Supported Parts

- MAX32570
- MAX32650
- MAX32655 and MAX32656
- MAX32665-MAX32666
- MAX32690
- MAX78002

---

### Miscellaneous Drivers

The `Libraries/MiscDrivers` folder of the MSDK contains drivers for miscellaneous external components such as TFT displays, cameras, audio codecs, PMICs, pushbuttons, etc. These resources are usually closely tied with the [Board Support Packages](#board-support-packages).

#### Miscellaneous Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
|                        |                                                            |                                                              |
| `CAMERA`               | (Optional) Set the Camera drivers to use                   | This option is only useful for the MAX78000 and MAX78002 and sets the camera drivers to use for the project. Permitted values are `HM01B0`, `HM0360_MONO`, `HM0360_COLOR`, `OV5642`, `OV7692` (default), or `PAG7920`. Camera drivers can be found in the [`Libraries/MiscDrivers/Camera`](Libraries/MiscDrivers/Camera) folder. Depending on the selected camera, a compiler definition may be added to the build. See the `board.mk` file for the active BSP for more details. |

---

### SDHC

The **Secure Digital High Capacity *(SDHC)*** library offers a higher-level interface built on top of the SDHC [Peripheral Driver API](#peripheral-driver-api) that includes a [FatFS File System](http://elm-chan.org/fsw/ff/00index_e.html) implementation for managing files on SD cards.

See [Build Variables for Toggling Libraries](#build-variables-for-toggling-libraries) for instructions on enabling the SDHC library.

#### SDHC Supported Parts

- MAX32650
- MAX32570
- MAX32665-MAX32666
- MAX78000
- MAX78002

#### SDHC Build Variables

Once enabled, the following [build configuration variables](#build-configuration-variables) become available.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `FATFS_VERSION`            | Specify the version of [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) to use | FatFS is a generic FAT/exFAT filesystem that comes as a sub-component of the SDHC library.  This variable can be used to change the [version](http://elm-chan.org/fsw/ff/updates.html) to use.  Acceptable values are `ff13` (R0.13), `ff14` (R0.14b), or `ff15` (R0.15) |
| `SDHC_CLK_FREQ`            | Sets the clock freq. for the SDHC library (Hz) | Sets the target clock frequency in units of Hz (Default is 30Mhz).  Reducing the SDHC clock frequency is a good troubleshooting step when debugging communication issues. |
| `FF_CONF_DIR`            | Sets the search directory for `ffconf.h` | (Available for `FATFS_VERSION = ff15` only) FatFS configuration is done via an `ffconf.h` file.  This option allows specifying the location of a custom `ffconf.h` file for a project. |

---

### FreeRTOS

[FreeRTOS](https://www.freertos.org/index.html) is a Real-Time Operating System (RTOS), which offers basic abstractions for multi-tasking and an OS layer specifically targeted at embedded systems with real-time requirements.  The MSDK maintains an official support layer for the FreeRTOS kernel.  Official documentation can be found on the [FreeRTOS website](https://www.freertos.org/index.html).

#### FreeRTOS Supported Parts

FreeRTOS is supported by all parts in the MSDK.  See the `FreeRTOSDemo` example application.

#### FreeRTOS Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `FREERTOS_HEAP_TYPE`            | Specify the method of heap allocation to use for the FreeRTOS API | FreeRTOS provides options for the heap management alogirthms to optimize for memory size, speed, and risk of heap fragmentation. For more details, visit the [FreeRTOS MemMang Docs](https://www.freertos.org/a00111.html).  Acceptable values are `1`, `2`, `3`, `4`, or `5`. The default value is `4` for heap_4. |

#### FreeRTOS-Plus

[FreeRTOS-Plus](https://www.freertos.org/FreeRTOS-Plus/index.html) is an additional library that implements addon functionality for the FreeRTOS kernel.  The MSDK maintains support for some, but not all, available addons.

- [FreeRTOS-Plus-CLI](https://www.freertos.org/FreeRTOS-Plus/index.html): **Supported**
- [FreeRTOS-Plus-TCP](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html): **Not supported** (Contributions welcome!)


### CLI

Developing a UART Command-Line Interface (CLI) is a common task while developing embedded firmware.  The MSDK contains a pre-made command processing library in the `Libraries/CLI` that can be used to simplify and speed up development.

See the [`Libraries/CLI/README.md`](Libraries/CLI/README.md) document for more details.

### CoreMark

[EEMBC’s CoreMark®](https://www.eembc.org/coremark/) is a benchmark that measures the performance of microcontrollers (MCUs) and central processing units (CPUs) used in embedded systems.  CoreMark is a simple, yet sophisticated benchmark that is designed specifically to test the functionality of a processor core. Running CoreMark produces a single-number score allowing users to make quick comparisons between processors.

#### CoreMark Supported Parts

All parts in the MSDK support the Coremark library via a `Coremark` example application.

???+ note "ℹ️ **Note**"
    The source code of the `Coremark` examples are somewhat unique.  They only contain a `core_portme.c`/`core_portme.h`.  These files are provided by CoreMark libraries to give the MSDK an implementation layer for a few hardware-dependent functions.  Otherwise, the remainder of the source code (located in `Libraries/Coremark`) must remain unmodified to comply with the CoreMark rules.

## Examples

The MSDK contains examples for each microcontroller that demonstrate the usage of its [Peripheral APIs](#peripheral-driver-api) and other supported libraries. They can be found in the `Examples` folder of an MSDK installation.

![Figure 40](res/Fig40.jpg)

???+ warning "**⚠️ Copying Examples**"
    It's strongly recommended to copy example projects to an _outside_ folder before modifying them.  This keeps the MSDK's "source" copy preserved for reference.  Project folders must be copied to a location _without_ any spaces in its filepath.

Each example contains a `README.md` file describing what it does. In general, there is at least one example per peripheral block, and the example's name will indicate what it matches (i.e., `DMA`, `ADC`, `SPI`).

The tables below are offered to facilitate easier browsing through the MSDK examples.  They are autogenerated from the available projects for each microcontroller.  A link to the example's source code on Github is available, as well as its location inside of the MSDK.

<!-- Note for maintainers: The autogeneration is handled in "Documentation/build.py" -->
##__EXAMPLES_LIST__##

## Debuggers

### Debug Limitations

- ???+ warning "**⚠️ Warning**"
    It’s important to note some fundamental limitations of debugging the MSDK's supported parts.  These limitations may make the device difficult (or impossible) for the debugger to connect in certain states. In such cases, the device can attempt to be recovered using a [MAX32625PICO](#max32625pico-pico).  See [How to Unlock a Microcontroller That Can No Longer Be Programmed](#how-to-unlock-a-microcontroller-that-can-no-longer-be-programmed)
    * A debugger can not be connected to a microcontroller _while_ the device is in reset.
    * A microcontroller can not be debugged while it's _Sleep_, _Low Power Mode_, _Micro Power Mode_, _Standby_, _Backup_, or _Shutdown_ mode.  These modes shut down the SWD clock.
        * For low-power development, it's recommended to place a 2-second blocking delay at the start of `main`.

### MAX32625PICO (PICO)

A MAX32625PICO (affectionately called the "PICO") debug adapter is provided with almost all the evaluation platforms supported by the MSDK.  Additionally, most small form-factor evaluation kits have a PICO _embedded_ into the PCB.

It's good practice to update the PICO's firmware to the latest version, which can be found on the [MAX32625PICO Firmware Images](https://github.com/MaximIntegrated/max32625pico-firmware-images) Github page.

#### Updating the MAX32625PICO (PICO) Debug Adapter Firmware

1. Download the correct image for your evaluation platform from the [MAX32625PICO Firmware Images](https://github.com/MaximIntegrated/max32625pico-firmware-images) Github page.

2. Connect the included micro-USB cable to the PICO _without_ connecting the other side of the cable to your host PC yet.

    ![Pico USB Connection](res/pico_partial_connected.jpg)

3. Press and hold the pushbutton on the top of the PICO.

    ![Pico Pushbutton](res/pico_pushbutton.jpg)

4. _While holding down the pushbutton on the PICO_ connect the other side of the micro-USB cable to your host PC.

    Keep the pushbutton held down until the LED on the PICO blinks and becomes solid.

    ![Pico Connected](res/pico_connected.jpg)

5. A `MAINTENANCE` drive should now appear on your file system.

    ![Maintenance Drive](res/MAINTENANCE.jpg)

6. Drag and drop the downloaded file from step 1 onto the `MAINTENANCE` drive. This will flash the PICO with the updated firmware.

    ![Maintenance Drive](res/MAINTENANCE.jpg)

    ![Drag and Drop](res/drag_and_drop.JPG)

    ![Flashing](res/pico_flashing.JPG)

7. Once the flashing is complete, the PICO will restart and present itself as a `DAPLINK` drive.

    ![DAPLINK Drive](res/DAPLINK.jpg)

8. Open the `DAPLINK` drive.

    ![Opened DAPLINK Drive](res/DAPLINK_opened.jpg)

9. Open the `DETAILS.TXT` file and verify the Git SHA matches the expected value for the updated file.

    ![DETAILS.TXT](res/DETAILS_Git_SHA.jpg)

10. Your PICO debugger is now ready to use with the latest firmware.

#### How to Unlock a Microcontroller That Can No Longer Be Programmed

The [debug limitations](#debug-limitations) of the MSDK's supported parts may make some devices difficult to connect to if bad firmware has been flashed.  In such cases, the device can attempt to be recovered from the “locked out” firmware by mass erasing the application code from the flash memory bank.  Note that this does not always work.  Success will depend on a small window being available for the debugger to connect immediately after reset.

Before following the procedure below, ensure that you have updated the PICO debugger firmware to the latest version. See [Updating the MAX32625PICO (PICO) Debug Adapter Firmware](#updating-the-max32625pico-pico-debug-adapter-firmware)

##### Unlock with VS Code

For VS Code users, the `"erase flash"` [build task](#build-tasks) can be used to attempt a mass erase.  If this task fails to recover the part, attempt the procedure below.

##### Unlock with `erase.act`

1. Connect the PICO debugger to the microcontroller to recover.

2. Connect the PICO debugger to the host PC with the included micro-USB cable.

3. Open the DAPLINK` drive on the host PC.

    ![DAPLINK Drive](res/DAPLINK.jpg)

4. Open the `DETAILS.TXT` inside of the `DAPLINK` drive in a text editor.

    ![DAPLINK Opened](res/DAPLINK_opened.jpg)

5. Verify that the “Automation allowed” field is set to 1.

    ![Automation Allowed](res/DETAILS_automation_allowed.jpg)

    ??? note "ℹ️ **Enabling Automation**"
        If this field is _not_ set to 1, follow the procedure below:

        1. Create a new _empty_ file, and save it as `auto_on.cfg`.

        2. Press and hold the pushbutton on top of the PICO.

            ![PICO Pushbutton](res/pico_pushbutton.jpg)

        3. _While holding the pushbutton_, drag and drop the `auto_on.cfg` file onto the `DAPLINK` drive.

            ![Drag and Drop File](res/auto_on.cfg.jpg)

        4. Continue holding the pushbutton until the file is finished transferring over, then release it.

        5. The PICO should power cycle, and the DAPLINK drive should re-appear with “Automation allowed” set to 1.

6. Power on the evaluation platform (if it isn’t already).

7. Create an empty file called `erase.act`.

8. Drag and drop the `erase.act` file onto the `DAPLINK` drive.

    ![Drag and Drop erase.act](res/erase.act.jpg)

9. The PICO debugger will attempt to mass erase the microcontroller's flash bank, which will completely wipe any application firmware that is programmed on the device.

    ???+ note "ℹ️ **Note**"
        If this process fails, a `FAIL.TXT` file will be present in the DAPLINK drive with an additional error message inside of it.  A failure is generally indicative of firmware that has completely shut down the debug port, or has entered low power loop immediately on power-up.  In these cases, it's not possible to recover the device.

10. Power cycle the microcontroller. It step 8 succeeded, it's blank and ready to re-program. The debugger should have no issues connecting to the device in this blank state.

## Developer Notes

### SPI v2 Library

The SPI v2 Library is the latest version of the MSDK SPI drivers which highlights:

- Target Select (TS) Control Scheme which provides users the option to drive their own TS pins.
- Optional`MXC_SPI_Config(...)` and `mxc_spi_cfg_t` struct to reduce the use of multiple functions to select proper SPI settings.
- Re-mapped the `MXC_SPI_Init(...)` function input parameters (same behavior as SPI v1).
- Allow for re-arming an SPI transaction within the callback function for chained SPI messages.
- Decrease in setup overhead in a Transaction function call. Less nested function calls within drivers.
- Improved SPI DMA support and DMA Channel IRQ vector flexibility by providing acquired DMA channel numbers before a SPI DMA transaction call.
- The use of Controller and Target terms instead of Master and Slave, respectively.
- Still supports SPI v1 function prototypes for backwards-compatibility.
- Bug fixes from the SPI v1 API.

#### SPI v2 Supported Parts

- MAX32572
- MAX32690
- MAX78002

#### Porting Projects to use SPI v2

The latest SPI examples in the MSDK defaults to build the SPI v1 libraries. Set the `MXC_SPI_VERSION` [build configuration variable](#build-configuration-variables) to `v2` (case sensitive) use the SPI v2 API.

This guide shows how to update an existing project that is using the SPI v1 API to SPI v2. The SPI v2 Library still supports the SPI v1 function prototypes for backwards-compatibility with the main difference being the SPI DMA interrupt handling (see [SPI DMA Interrupt Handling](#spi-dma-interrupt-handling) section below for more info).

Note: The SPI v2 API is only a drop in replacement to SPI v1 if SPI DMA is **not** used; should the user choose to continue building with the SPI v1 convention but with the underlying SPI v2 implementation. This porting guide demonstrates how to use the full extent of the SPI v2 features.

##### SPI Init Function

The input parameters for the `MXC_SPI_Init(...)` function were updated in SPI v2 to allow for more user-selectable options during initialization. This should not cause any errors or behavioral differences with the `MXC_SPI_Init(...)` function when switching between SPI v1 and SPI v2 builds as the input parameters were essentially re-mapped to more descriptive names.

SPI v1:

    :::C
    int MXC_SPI_Init(mxc_spi_regs_t *spi,
                    int masterMode,
                    int quadModeUsed,
                    int numSlaves,
                    unsigned ssPolarity,
                    unsigned int hz,
                    mxc_spi_pins_t pins)

SPI v2:

    :::C
    int MXC_SPI_Init(mxc_spi_regs_t *spi,
                    mxc_spi_type_t controller_target,
                    mxc_spi_interface_t if_mode,
                    int numTargets,
                    uint8_t ts_active_pol_mask,
                    uint32_t freq,
                    mxc_spi_pins_t pins)

Input Parameters:

- `mxc_spi_regs_t *spi` remains unchanged.
- `int masterMode` -> `mxc_spi_type_t controller_target`. The enum `mxc_spi_type_t` was added for increased code readability.
- `int quadModeUsed` -> `mxc_spi_interface_t if_mode`. Previously, the `MXC_SPI_Init(...)` function could only select between standard (4wire) and quad interface modes. With SPI v2, the user can select either standard (`MXC_SPI_INTERFACE_STANDARD`), quad (`MXC_SPI_INTERFACE_QUAD`), 3wire (`MXC_SPI_INTERFACE_3WIRE`), or dual (`MXC_SPI_INTERFACE_DUAL`) mode.
- `int numSlaves` -> `int numTargets`. SPI v2 does not use this parameter and was kept to continue supporting SPI v1.
- `unsigned ssPolarity` -> `uint8_t ts_active_pol_mask`. Updated to a more descriptive name.
- `unsigned int hz` -> `uint32_t freq`.
- `mxc_spi_pins_t pins` remains unchanged.

##### SPI Config Function

The `int MXC_SPI_Config(mxc_spi_cfg_t cfg)` function was added to reduce the number of helper function calls to set the appropriate settings that the `MXC_SPI_Init(...)` function did not set.

This function also sets up the DMA and acquires DMA TX/RX channels for SPI DMA transactions.

`mxc_spi_cfg_t` struct:

- `mxc_spi_regs_t *spi` - Select SPI Instance to configure.
- `mxc_spi_clkmode_t clk_mode` - Select clock mode.
- `uint8_t frame_size` - Select single frame size (2 - 16 bits).
- `bool use_dma_tx` - Enable SPI DMA TX (acquire and configure TX channel).
- `bool use_dma_rx` - Enable SPI DMA RX (acquire and configure RX channel).
- `mxc_dma_regs_t *dma` - Select DMA Instance to configure for SPI DMA (Valid only if `use_dma_tx` or `use_dma_rx` is set to true).

##### SPI Request Struct

`mxc_spi_req_t` struct:

- `mxc_spi_regs_t *spi` - Remains unchanged.
- `int ssIdx` - Remains unchanged.
- `int ssDeassert` - Remains unchanged.
- `uint8_t *txData` - Remains unchanged.
- `uint8_t *rxData` - Remains unchanged.
- `uint32_t txLen` - Remains unchanged.
- `uint32_t rxLen` - Remains unchanged.
- `uint32_t txCnt` - Not used in SPI v2.
- `uint32_t rxCnt` - Not used in SPI v2.
- `mxc_spi_callback_t completeCB` - Type was renamed, but funtionally, remains unchanged.

##### SPI Transaction Functions

The SPI v2 Libraries follows the terms used in the user guide: Controller and Target instead of Master and Slave, respectively.

- `MXC_SPI_MasterTransaction(...)`        -> `MXC_SPI_ControllerTransaction(...)`
- `MXC_SPI_MasterTransactionAsync(...)`   -> `MXC_SPI_ControllerTransactionAsync(...)`
- `MXC_SPI_MasterTransactionDMA(...)`     -> `MXC_SPI_ControllerTransactionDMA(...)`
- `MXC_SPI_SlaveTransaction(...)`         -> `MXC_SPI_TargetTransaction(...)`
- `MXC_SPI_SlaveTransactionAsync(...)`    -> `MXC_SPI_TargetTransactionAsync(...)`
- `MXC_SPI_SlaveTransactionDMA(...)`      -> `MXC_SPI_TargetTransactionDMA(...)`

##### SPI DMA Setup for `MXC_SPI_ControllerTransactionDMA(...)`

The SPI v2 library allows for more flexibility in setting generic DMA TX/RX channel vectors for SPI DMA transactions during run-time. Compared to SPI v1 where the user must know the acquired SPI DMA TX/RX channel numbers and define the appropriate DMA channel handlers before compile-time.

There are two ways to initialize and configure the DMA before starting a SPI DMA transaction.

Method 1: Call `int MXC_SPI_DMA_Init(mxc_spi_regs_t *spi, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx)`.

Method 2: Set up the DMA options in the `mxc_spi_cfg_t` struct and call `int MXC_SPI_Config(mxc_spi_cfg_t cfg)`.

    :::C
    ...
    // Initialize SPI DMA ahead of time.
    MXC_SPI_DMA_Init(SPI, DMA, true, true);

    int TX_DMA_CH = MXC_SPI_DMA_GetTXChannel(SPI);
    int RX_DMA_CH = MXC_SPI_DMA_GetRXChannel(SPI);

    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(TX_DMA_CH));
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(RX_DMA_CH));

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(TX_DMA_CH), DMA_TX_IRQHandler);
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(RX_DMA_CH), DMA_RX_IRQHandler);

    MXC_SPI_ControllerTransactionDMA(&req);
    ...

The DMA is initialized in `MXC_SPI_DMA_Init(...)` or `MXC_SPI_Config(...)`. This provides information on what DMA channels were acquired for a SPI instance's TX and RX DMA before calling the DMA transaction function. Following the example above, it is recommended to set up a generic-named DMA TX/RX vector because the SPI TX and RX DMA channels won't always acquire DMA_CH0 and DMA_CH1, respectively.

##### SPI DMA Interrupt Handling

    :::C
    void DMA_TX_IRQHandler(void)
    {
        MXC_SPI_DMA_TX_Handler(SPI);
    }

    void DMA_RX_IRQHandler(void)
    {
        MXC_SPI_DMA_RX_Handler(SPI);
    }

The SPI v1 API requires `MXC_DMA_Handler()` to be called in the TX and RX DMA Channel interrupt handlers. Following the generic vector names used in the previous section, the SPI v2 supplies its own TX/RX DMA Handler processing functions (`MXC_SPI_DMA_RX_Handler(...)` and `MXC_SPI_DMA_RX_Handler(...)`) that must be called within their appropriate DMA channel interrupt handlers.
