## Description

This example determines the Coremark rating for the MAX32xxx and MAX78xxx series microcontrollers.

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

