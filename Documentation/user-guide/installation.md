# Installation

## Prerequisites

- **Elevated/Administrator rights**

- ???+ warning "**⚠️ MacOS**"
    On MacOS, please also download and install [Homebrew](https://brew.sh/).  It will be used in [Completing the Installation on MacOS](#completing-the-installation-on-macos) later on.

- ???+ warning "**⚠️ Ubuntu**"
    Several GUI packages are required by the QT installer framework _even on headless systems_.  Run the following command _before_ running the installer to retrieve them.

            :::shell
            sudo apt update && sudo apt install libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-shm0 libxcb-util1 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-render0 libxcb-shape0 libxcb-sync1 libxcb-xfixes0 libxcb-xinerama0 libxcb-xkb1 libxcb1 libxkbcommon-x11-0 libxkbcommon0 libgl1 libusb-0.1-4 libhidapi-libusb0 libhidapi-hidraw0

## Download

The MSDK installer is available for supported Operating Systems from the download links below.

- [**Windows 10**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0010820B)

- [**Linux (Ubuntu)**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018720B)

    - ???+ note "ℹ️ **Note**"
        This file must be made executable before running (`chmod +x MaximMicrosSDK_linux.run`). Alternatively, set `Allow executing as program" in the Ubuntu GUI.
        ![Figure 1](res/Fig1.jpg)

- [**MacOS**](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018610B)

    - ???+ note "ℹ️ **Note**"
        On MacOS, the installer is distributed inside a .dmg disk image file. Double-click the downloaded file to mount it. Afterward, the installer executable will be made available inside the mounted drive.

## Setup

The MSDK installer can be run through a [**GUI Installation**](#gui-installation) or a [**Command-Line Installation**](#command-line-installation)

### GUI Installation

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


### Command-Line Installation

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

### Completing the Installation on MacOS

???+ warning "⚠️ **Warning**"
    On MacOS, some additional missing packages must be manually installed with [Homebrew](https://brew.sh/).  There are also some manual setup steps required to retrieve `make` version 4.  The instructions in this section are critical.

1. Install [Homebrew](https://brew.sh/).

2. Run the command below to install dependencies for OpenOCD.

        :::shell
        brew install libusb-compat libftdi hidapi libusb

## Maintenance

An MSDK installation contains a `MaintenanceTool` executable program in its root directory. Use the Maintenance Tool to retrieve updates, manage components, and uninstall the MSDK.

![Figure 11](res/Fig11.jpg)

### Updates

The MSDK releases updates quarterly, and the Maintenance Tool will retrieve the latest release when **Update components** is run.

### Older Versions and Offline Installer

Older versions of the MSDK are available as an **_offline installer_** for each release tag. They are available on the [Releases page](https://github.com/analogdevicesinc/msdk/releases) of the MSDK GitHub and can be used to roll back to a specific MSDK release.

### Development Resources

Users can obtain development copies of the MSDK resources from [Github](https://github.com/analogdevicesinc/msdk).  Setup instructions can be found in the repository's [README](https://github.com/analogdevicesinc/msdk/blob/main/README.md).
