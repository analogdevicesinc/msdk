# ARM-DSP

Porting ARM's DSP examples to Maxim's M4-series microcontrollers.

## Overview

The projects in this folder are ports of ARM's DSP example projects that can be found in the <a href="https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html" >CMSIS-DSP library</a>.  They have been made compatible with the build system in Analog Device's MSDK Toolchain.  Some slight modifications have been made to the source code to eliminate compiler errors/warnings, but the examples have been left mostly "as-is".  The major changes here have been made to the build system.

Debug launch configurations are included in each project, and should appear in the "Favorites" menu in Eclipse.

Development with Visual Studio Code is also supported.  See [VSCode-Maxim](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop)

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

The ARM-DSP projects build with hardware floating point acceleration enabled by default setting the `MFLOAT_ABI` build configuration variable in [project.mk](project.mk).  This is the highest performance option, and is suitable for most projects.  However, if an external library is added to the project that has been compiled using soft-float calling conventions, then `MFLOAT_ABI=softfp` or `MFLOAT_ABI=soft` must be used.  See the `-mfloat-abi` section of the [GCC ARM-Options documentation](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) for more details.

