# Libraries

The MSDK contains a large number of libraries, both third-party and in-house. The main library is the [Peripheral Driver API](#peripheral-driver-api), but the MSDK also contains drivers for various _external_ components such as TFT displays, cameras, accelerometers, audio codecs, and other devices. Additionally, dedicated libraries for more complex _internal_ hardware peripherals such as USB, the SDHC interface, and the Cordio BLE stack are also available. These usually build on _top_ of the Peripheral Driver API.

???+ note "ℹ️ **Note: Enabling Libraries**"
    Libraries can be enabled for a project with a convenient *toggle switch* provided by the build system (See [Build Variables for Toggling Libraries](#build-variables-for-toggling-libraries)).

## Peripheral Driver API

A microcontroller is made up of a Central Processing Unit (CPU) that is surrounded by additional _peripheral_ hardware blocks such as timers, memory controllers, UART controllers, ADCs, RTCs, audio interfaces, and many more. The **Peripheral Driver API** is an important core library in the MSDK that allows the CPU to utilize the microcontroller's hardware blocks over a higher-level **_Application Programming Interface (API)_**.

![Figure 38](res/Fig38.jpg)

### API Documentation (PeriphDrivers)

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

### PeriphDrivers Organization

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

### Die Types to Part Numbers

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

## CMSIS-DSP

The CMSIS-DSP library provides a suite of common **Digital Signal Processing _(DSP)_** functions that take advantage of hardware accelerated _Floating Point Unit (FPU)_ available on microcontrollers with Arm Cortex-M cores. This library is distributed in the MSDK as a pre-compiled static library file, and the MSDK maintains a port of the official code examples in the **ARM-DSP** [Examples](https://github.com/analogdevicesinc/msdk/tree/main/Examples) folder for each microcontroller.

Please refer to the [CMSIS-DSP official documentation](https://arm-software.github.io/CMSIS-DSP/v1.16.2/index.html) for more detailed documentation on the library functions and usage.

### CMSIS-DSP Supported Parts

- All microcontrollers with a Cortex M4 core are supported.

### CMSIS-DSP Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
|                        |                                                            |                                                              |
| `CMSIS_DSP_VERSION`    | (Optional) Set the CMSIS-DSP version to use.               | Defaults to `1.16.2`, which is currently the only supported version. |

---

## Cordio Bluetooth Low Energy

The Cordio Bluetooth Low Energy (BLE) library provides a full BLE stack for microcontrollers with an integrated BLE controller.

The Cordio library warrants its own separate documentation. See the **[Cordio BLE User Guide](Libraries/Cordio/docs/CORDIO_USERGUIDE.md)**.

### Cordio Supported Parts

- MAX32655
- MAX32665
- MAX32680
- MAX32690

---

## MAXUSB

The MAXUSB library provides a higher-level interface for utilizing the built-in USB controller hardware available on some microcontrollers. This allows the microcontroller to enumerate as a USB device without the need for an external USB controller IC.  MAXUSB provides a finer level of control of USB events and classes than TinyUSB.

### MAXUSB Supported Parts

- MAX32570
- MAX32650
- MAX32655 and MAX32656
- MAX32665-MAX32666
- MAX32690
- MAX78002

---

## TinyUSB

The TinyUSB library provides a high-level interface for utilizing the built-in USB controller hardware available on some microcontrollers. This allows the microcontroller to enumerate as a USB device without the need for an external USB controller IC. **[TinyUSB](https://github.com/hathach/tinyusb) provides a cross-platform USB stack for embedded systems, with a higher level of abstraction than MAXUSB,
supporting most standard USB device classes.

### TinyUSB Supported Parts

- MAX32650
- MAX32665-MAX32666
- MAX32690
- MAX78002

---

## Miscellaneous Drivers

The `Libraries/MiscDrivers` folder of the MSDK contains drivers for miscellaneous external components such as TFT displays, cameras, audio codecs, PMICs, pushbuttons, etc. These resources are usually closely tied with the [Board Support Packages](#board-support-packages).

### Miscellaneous Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
|                        |                                                            |                                                              |
| `CAMERA`               | (Optional) Set the Camera drivers to use                   | This option is only useful for the MAX78000 and MAX78002 and sets the camera drivers to use for the project. Permitted values are `HM01B0`, `HM0360_MONO`, `HM0360_COLOR`, `OV5642`, `OV7692` (default), or `PAG7920`. Camera drivers can be found in the [`Libraries/MiscDrivers/Camera`](Libraries/MiscDrivers/Camera) folder. Depending on the selected camera, a compiler definition may be added to the build. See the `board.mk` file for the active BSP for more details. |

---

## SDHC

The **Secure Digital High Capacity *(SDHC)*** library offers a higher-level interface built on top of the SDHC [Peripheral Driver API](#peripheral-driver-api) that includes a [FatFS File System](http://elm-chan.org/fsw/ff/00index_e.html) implementation for managing files on SD cards.

See [Build Variables for Toggling Libraries](#build-variables-for-toggling-libraries) for instructions on enabling the SDHC library.

### SDHC Supported Parts

- MAX32650
- MAX32570
- MAX32665-MAX32666
- MAX78000
- MAX78002

### SDHC Build Variables

Once enabled, the following [build configuration variables](#build-configuration-variables) become available.

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `FATFS_VERSION`            | Specify the version of [FatFS](http://elm-chan.org/fsw/ff/00index_e.html) to use | FatFS is a generic FAT/exFAT filesystem that comes as a sub-component of the SDHC library.  This variable can be used to change the [version](http://elm-chan.org/fsw/ff/updates.html) to use.  Acceptable values are `ff13` (R0.13), `ff14` (R0.14b), or `ff15` (R0.15) |
| `SDHC_CLK_FREQ`            | Sets the clock freq. for the SDHC library (Hz) | Sets the target clock frequency in units of Hz (Default is 30Mhz).  Reducing the SDHC clock frequency is a good troubleshooting step when debugging communication issues. |
| `FF_CONF_DIR`            | Sets the search directory for `ffconf.h` | (Available for `FATFS_VERSION = ff15` only) FatFS configuration is done via an `ffconf.h` file.  This option allows specifying the location of a custom `ffconf.h` file for a project. |

---

## FreeRTOS

[FreeRTOS](https://www.freertos.org/index.html) is a Real-Time Operating System (RTOS), which offers basic abstractions for multi-tasking and an OS layer specifically targeted at embedded systems with real-time requirements.  The MSDK maintains an official support layer for the FreeRTOS kernel.  Official documentation can be found on the [FreeRTOS website](https://www.freertos.org/index.html).

### FreeRTOS Supported Parts

FreeRTOS is supported by all parts in the MSDK.  See the `FreeRTOSDemo` example application.

### FreeRTOS Build Variables

| Configuration Variable | Description                                                | Details                                                      |
| ---------------------- | ---------------------------------------------------------- | ------------------------------------------------------------ |
| `FREERTOS_HEAP_TYPE`            | Specify the method of heap allocation to use for the FreeRTOS API | FreeRTOS provides options for the heap management alogirthms to optimize for memory size, speed, and risk of heap fragmentation. For more details, visit the [FreeRTOS MemMang Docs](https://www.freertos.org/a00111.html).  Acceptable values are `1`, `2`, `3`, `4`, or `5`. The default value is `4` for heap_4. |

### FreeRTOS-Plus

[FreeRTOS-Plus](https://www.freertos.org/FreeRTOS-Plus/index.html) is an additional library that implements addon functionality for the FreeRTOS kernel.  The MSDK maintains support for some, but not all, available addons.

- [FreeRTOS-Plus-CLI](https://www.freertos.org/FreeRTOS-Plus/index.html): **Supported**
- [FreeRTOS-Plus-TCP](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html): **Not supported** (Contributions welcome!)

## CLI

Developing a UART Command-Line Interface (CLI) is a common task while developing embedded firmware.  The MSDK contains a pre-made command processing library in the `Libraries/CLI` that can be used to simplify and speed up development.

See the [`Libraries/CLI/README.md`](Libraries/CLI/README.md) document for more details.

## CoreMark

[EEMBC’s CoreMark®](https://www.eembc.org/coremark/) is a benchmark that measures the performance of microcontrollers (MCUs) and central processing units (CPUs) used in embedded systems.  CoreMark is a simple, yet sophisticated benchmark that is designed specifically to test the functionality of a processor core. Running CoreMark produces a single-number score allowing users to make quick comparisons between processors.

### CoreMark Supported Parts

All parts in the MSDK support the Coremark library via a `Coremark` example application.

???+ note "ℹ️ **Note**"
    The source code of the `Coremark` examples are somewhat unique.  They only contain a `core_portme.c`/`core_portme.h`.  These files are provided by CoreMark libraries to give the MSDK an implementation layer for a few hardware-dependent functions.  Otherwise, the remainder of the source code (located in `Libraries/Coremark`) must remain unmodified to comply with the CoreMark rules.
