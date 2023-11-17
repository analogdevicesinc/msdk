## Description

A program for RISC-V to use MSR ROM for reading 2-track card

Example shows how to configure MSR algorithm parameters to avoid scanning track 3
if card is known to have only 2 tracks recorded, like typical credit card.


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
ARM: *********** MSR Example (2-track version) ***********
ARM: Starting RISC-V at 20068000, Nov 17 2023 15:25:03

RISCV: pclk2aclk_div = 15
RISCV: adc_frame_len = 8
RISCV: adc_frame_seq = 1 1 2 1 1 2 1 2
RISCV: adc cm trim : register = 0x40000820 pos = 0 len = 5 value = 0x14
                                                                                                                                 
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
 Decoded 76 chars, Forward, Rate 40.7 in/sec
 0123456789012345678901234567890123456789012345678901234567890123456789012345
 === Track 2 ===
 LRC check passed           
 Decoded 37 chars, Forward, Rate 40.3 in/sec
 0123456789012345678901234567890123456           
 Total (1-1-0) successful tracks in 1 swipes

ARM: Waiting for swipe 2...
```
