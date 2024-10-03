# Overview

The MAX Microcontrollers SDK (MSDK), now a part of [Analog Devices](https://www.analog.com/en/index.html), contains the necessary software and tools to develop firmware for the [MAX32xxx and MAX78xxx Microcontrollers](https://www.analog.com/en/parametricsearch/10984). That includes register and system startup files to enable low-level development for its [supported parts](#supported-parts). It also provides higher-level peripheral driver APIs (written in C) alongside various utilities, third-party libraries, Board Support Packages (BSPs), and a set of example programs for each microcontroller.

Additionally, the MSDK includes a GCC-based toolchain, and builds are managed by a system of Makefiles (See [GNU Make](https://www.gnu.org/software/make/manual/)). A [custom fork of OpenOCD](https://github.com/analogdevicesinc/openocd) enables flashing and debugging. The MSDK's toolchain and build system offers a Command Line Interface (CLI), and project files for [supported development environments](#supported-development-environments) are maintained that build on top of that CLI.

This document describes the MSDK's installation, setup, and usage.

## Supported Operating Systems

- Windows (Windows 10 only)

- Linux (Ubuntu only)

- MacOS

## Supported Parts

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

## Supported Development Environments

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

## Supported Languages

- C
- C++
- Assembly (Arm and/or RISC-V instruction set depending on the microcontroller)
