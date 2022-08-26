## Description

This example showcases the main functions of the MAX78000's flash controller.

The MAX78000's entire flash, including all application code, is initially cleared by performing a mass erase.  As such, the entire example is set to execute out of SRAM by using the special "_ram" linkerfile (see the project.mk and code comments for more details).  Then, a test pattern is written into flash one 32-bit word at a time and verified.  By default the entire flash bank is excercised, but this can be controlled with the `NUM_TEST_PAGES` compiler definition in `main.c`

## Building Firmware

### Command-line

1. This example comes pre-configured for the MAX78000EVKIT by default.  Select the correct value for _BOARD_, either `EvKit_V1` (for the MAX78000EVKIT) or `FTHR_RevA` (for the MAX78000FTHR) by editing the project `project.mk`:

    ```Makefile
    # Specify the board used
    BOARD=EvKit_V1
    #BOARD=FTHR_RevA
    ```

2. Build the project with the command:

    ```shell
    make -r -j 8 all
    ```

### Visual Studio Code

Visual Studio Code support is included via the VSCode-Maxim project files.  See:  [Readme](./.vscode/readme.md)

1. Select the correct value for `"board"` in `.vscode/settings.json` - either `EvKit_V1` (for the MAX78000EVKIT) or `FTHR_RevA` (for the MAX78000FTHR)

2. Run the "build" task with `CTRL+SHIFT+B` -> `build`.

3. Run the "flash & run" build task (`CTRL+SHIFT+B` -> `flash & run`) to run the example.  Since the example executes entirely out of SRAM it must be launched this way.

### Eclipse

1. Import the project into your Eclipse workspace.

2. Open the project properties and navigate to `C/C++ Build` -> `Environment`.

3. Set the `BOARD` variable to `EvKit_V1` (for the MAX78000EVKIT) or FTHR_RevA (for the MAX78000FTHR).

4. Run "build project" (right click -> "Build Project" or Hammer button)

5. Debug the project to flash and run it.

##### Required Connections:
If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** Flash Control Example *****
This example executes entirely from RAM, and
 will mass erase the entire flash contents before
 writing and verifying a test pattern from
 ADDR: 0x10000000 to ADDR: 0x10020000
Push PB1 to begin
Wiping flash...
Flash has been successfully wiped.Flash mass erase is verified.
Word 1 written properly and has been verified.
Word 2 written properly and has been verified.
Word 3 written properly and has been verified.
Word 4 written properly and has been verified.
Word 5 written properly and has been verified.
Word 6 written properly and has been verified.
Word 7 written properly and has been verified.
Word 8 written properly and has been verified.
Word 9 written properly and has been verified.
Word 10 written properly and has been verified.
Word 11 written properly and has been verified.
Word 12 written properly and has been verified.
Word 13 written properly and has been verified.
Word 14 written properly and has been verified.
Word 15 written properly and has been verified.
Continuing for 131056 more words...
3.89%
7.78%
11.66%
15.55%
19.44%
23.33%
27.22%
31.10%
34.99%
38.88%
42.77%
46.66%
50.54%
54.43%
58.32%
62.21%
66.09%
69.98%
73.87%
77.76%
81.65%
85.53%
89.42%
93.31%
97.20%
100%
Done!
Successfully verified erasure of page 2 (ADDR: 0x10004000)
Testing partial erasure between pages 1 (ADDR: 0x10000080) and 2 (ADDR: 0x10003b00)...
Successfully verified partial erasure.
Example succeeded!

```
