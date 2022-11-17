# ARM-DSP
Porting ARM's DSP examples to Maxim's M4-series microcontrollers.

# Introduction

The projects in this folder are ports of ARM's DSP example projects that can be found in the <a href="https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html" >CMSIS-DSP library</a>.  They have been made compatible with the build system in Analog Device's MSDK Toolchain.  Some slight modifications have been made to the source code to eliminate compiler errors/warnings, but the examples have been left mostly "as-is".  The major changes here have been made to the build system.

Debug launch configurations are included in each project, and should appear in the "Favorites" menu in Eclipse.

Development with Visual Studio Code is also supported.  See [VSCode-Maxim](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop)

# Build Notes
For detailed documentation on the MSDK's build system, see [Build Configuration](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration).  The following notes contain relevant "quick-start" info specific to the ARM-DSP projects.

The ARM-DSP projects build with hardware floating point acceleration enabled by default using the `MFLOAT_ABI=hard` (see each project's project.mk file).  This is the highest performance option.  This option is suitable for most projects and completely compatible with the MSDK's libraries.  However, if an external library is added to the project that has been compiled using soft-float calling conventions, then `MFLOAT_ABI=softfp` or `MFLOAT_ABI=soft` must be used.  See the `-mfloat-abi` section of the [GCC ARM-Options documentation](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html) for more details.

The ARM-DSP projects also come pre-configured for the EVKIT for the target microcontroller.  If you are using a different board, the project's can be easily reconfigured using the `BOARD` [configuration variable](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#how-to-set-a-configuration-variable).  Depending on your development environment, change the `BOARD` value in the following place:

* Command-line: project.mk
* Visual Studio Code: [.vscode/settings.json](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#project-configuration)
* Eclipse: Project Properties -> C/C++ Build -> Environment

