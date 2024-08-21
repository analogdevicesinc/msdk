## Description

This example determines the Coremark rating for the MAX32xxx and MAX78xxx series microcontrollers.

The Coremark benchmark is an industry standard benchmark which gives an MCU core a rating based on how fast it can execute multiple iterations of the standard Coremark test (which includes executing list processing, matrix manipulation and state machine algorithms, and computing CRC values). The final score is the number of Coremark test iterations/sec. For example, in the sample print out below, the micro received a score of 251.685201 iterations/sec.

For more information, visit the [CoreMark webpage](https://www.eembc.org/coremark/) and look through the [README](../../../Libraries/Coremark/README.md).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

To comply with the CoreMark rules, the only source files which are included in this example directory are the core_portme.c/.h files, the rest (including main) are located in the [Coremark](../../../Libraries/Coremark/) library.

## Required Connections
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
SystemCoreClock 96000000
I-Cache enabled
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 11919652
Total time (secs): 11.919652
Iterations/Sec   : 251.685201
Iterations       : 3000
Compiler version : GCC10.3.1 20210824 (release)
Compiler flags   :
Memory location  : STATIC
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0xcc42
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 251.685201 / GCC10.3.1 20210824 (release)  / STATIC
```

