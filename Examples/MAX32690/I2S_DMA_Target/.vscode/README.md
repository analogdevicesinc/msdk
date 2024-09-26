# VSCode-Maxim

_(If you're viewing this document from within Visual Studio Code you can press `CTRL+SHIFT+V` to open a Markdown preview window.)_

## Quick Links

* [MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)
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
* [Analog Devices MSDK](https://analogdevicesinc.github.io/msdk/)

## Installation

Install the MSDK, then set `"MAXIM_PATH"` in your _user_ VS Code settings.

See [Getting Started with Visual Studio Code](https://analogdevicesinc.github.io/msdk/USERGUIDE/#getting-started-with-visual-studio-code) in the MSDK User Guide for detailed instructions.

## Usage

See the [MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/#visual-studio-code) for detailed usage info.

## Issue Tracker

Bug reports, feature requests, and contributions are welcome via the [issues](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/issues) tracker on Github.

New issues should contain _at minimum_ the following information:

* Visual Studio Code version #s (see `Help -> About`)
* C/C++ Extension version #
* Target microcontroller and evaluation platform
* The projects `.vscode` folder and `Makefile` (where applicable).  Standard compression formats such as `.zip`, `.rar`, `.tar.gz`, etc. are all acceptable.
