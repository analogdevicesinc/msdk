## Description

A basic getting started program for RISC-V MSR ROM functionality.

Example will enable MSR on EvKit.
Shows two options, (i) run RISCV MSR ROM code as is (default option),
and (ii) programming RISC-V to run from SRAM with modified MSR parameters.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB TO UART0/PWR) connector on EV KIT board. 
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

There might be delay for up to 18 sec between powering up and starting code execution,
due to secure boot verification process.
The Console UART of the device will output these messages:
```
ARM: *********** MSR Example ***********
ARM: Starting RISC-V at 20068000, Nov 16 2023
                                                                                                                                 
RISCV: pclk = 50000000 
RISCV: swipe timeout = 45 sec
                                                                                                                                 
ARM: RISC-V MSR ROM version 4.1 started and ready
ARM: Memory control at Fast Wait State
                                                                                                                                 
ARM: Waiting for swipe 1...
```
After swiping Magstripe Q-card through maghead holder,
Console UART output should look similar to the following messages:

```
ARM printing swipe 1 results:
=== Track 1 ===                
LRC check passed                 
Decoded 76 chars, Reverse, Rate 23.5 in/sec
0123456789012345678901234567890123456789012345678901234567890123456789012345
=== Track 2 ===                 
LRC check passed                 
Decoded 37 chars, Reverse, Rate 23.1 in/sec
0123456789012345678901234567890123456
=== Track 3 ===                 
LRC check passed                 
Decoded 104 chars, Reverse, Rate 23.5 in/sec
01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123
Total (1-1-1) successful tracks in 1 swipes
Swipe was initiated by SRAM, complete by IRQ_RISCV

Waiting for swipe 2...
```
