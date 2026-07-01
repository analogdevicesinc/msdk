# Command-Line Development

This section offers more detailed info on command-line development.

For setup/quick-start, see ["Getting Started with Command-Line Development"](getting-started.md#getting-started-with-command-line-development).

## How to Set the BSP (Command-Line)

- To _persistently_ the BSP, set the **`BOARD`** _[Build Configuration Variable](build-system.md#build-configuration-variables)_ by editing the **project.mk** that can be found inside each project.

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

## Building on the Command-Line

1. `cd` into the project folder.

2. Run `make`

   - **Parallel Build** (fastest build, but console message formatting may be mangled):

        make -r -j

   - **Serial Build**

        make -r

3. Take note of the output filename and location, which by default is the lowercase name of the _Target microcontroller_ and created in the `build` folder.

## Cleaning on the Command-Line

1. `cd` into the project folder.
2. Run `make clean`
   - **Project clean**: `make clean` deletes the project `build` folder and all of its contents.
   - **Library clean**: `make distclean` can be used to clean out _all_ build products, including the project `build` folder and all [peripheral driver](libraries.md#peripheral-driver-api) libraries.

## Flashing on the Command-Line

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

## Debugging on the Command-Line

1. [Flash](#flashing-on-the-command-line) the program using the **Flash and Hold** command above.

2. Launch an **_new_ separate terminal**.

    ???+ warning "⚠️ On **Windows**, use the MinGW shortcut or `Tools/MSYS2/msys.bat` file to launch the MSYS2 terminal."

3. `cd` into the location of the copied example project.

4. Run the following command to launch a **GDB _client_**.

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

## Common GDB Commands

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
