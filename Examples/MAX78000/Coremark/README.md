## Description

This example determines the Coremark rating for the MAX32xxx and MAX78xxx series microcontrollers.

## Required Connections

If using the Standard EvKit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-	Select "EvKit_V1" for _BOARD_ in "project.mk"

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Select "FTHR_RevA" for _BOARD_ in "project.mk"

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

