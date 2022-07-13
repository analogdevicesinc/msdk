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

The Maxim Microcontrollers SDK (MaximSDK), now a part of Analog Devices, contains tools and resources to develop firmware for the MAX-series of microcontrollers.  This includes register files, peripheral drivers, system startup files, documentation, various utilities, third-party libraries, and a toolchain.

In general, the MaximSDK can be broken into two main pieces:  Code vs toolchain.  This repository contains the code, while the toolchain has all the programs you need to _build_ that code for the MAX-series microcontrollers.

## Installation

### Automatic Installer

The MaximSDK is available via an automatic installer for the platforms below.

* [Windows 10](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0010820A)

* [Ubuntu Linux](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018720A)

    * This file must be made executable before it can be run. Use the command `chmod +x MaximMicrosSDK_linux.run`. Alternatively, set “Allow executing as program” in the Ubuntu GUI.

* [MacOS](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0018610A)

    * For MacOS the installer is distributed inside of a .dmg disk image file. Double click the downloaded file to mount it. Afterwards, the installer executable will be made available inside the mounted drive.

### Completing the Installation on MacOS

On MacOS, some additional missing packages must be manually installed via [Homebrew](https://brew.sh/).

**For non-M1 platforms:**

1. Follow the instructions on the [Homebrew home page](https://brew.sh/) to install Homebrew on your system.

2. Then, open a terminal and run the command...

```shell
brew install libusb-compat libftdi hidapi libusb
```


**For M1 platforms**, you must use a Rosetta terminal to install Homebrew:

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