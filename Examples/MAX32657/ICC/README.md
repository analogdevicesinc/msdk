## Description

This example demonstrates the time differences when running code with the instruction cache on vs. off.  It runs four trials, reporting the time required for each.  The trials are:

1. Perform 2.5 million multiplies in a loop with the instruction cache on.  The variables used in the loop are all declared as 'volatile'.
2.  Perform 2.5 million multiplies in a loop with the instruction cache off.  The variables used in the loop are all declared as 'volatile'.
3. Perform 25 million multiplies in a loop with the instruction cache on.  The variables used in the loop are all local variables.
4.  Perform 25 million multiplies in a loop with the instruction cache off.    The variables used in the loop are all local variables.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

## Setup

### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
******** ICC Example ********

***** Volatile  Example *****

With instruction cache enabled:
0%,      k=0
10%,     k=500
20%,     k=1000
30%,     k=1500
40%,     k=2000
50%,     k=2500
60%,     k=3000
70%,     k=3500
80%,     k=4000
90%,     k=4500

Time Elapsed: 1.17713 Seconds


With instruction cache disabled:
0%,      k=0
10%,     k=500
20%,     k=1000
30%,     k=1500
40%,     k=2000
50%,     k=2500
60%,     k=3000
70%,     k=3500
80%,     k=4000
90%,     k=4500

Time Elapsed: 2.599812 Seconds


***** Non-volatile Example *****

With instruction cache enabled:
10%,     k=500
20%,     k=1000
30%,     k=1500
40%,     k=2000
50%,     k=2500
60%,     k=3000
70%,     k=3500
80%,     k=4000
90%,     k=4500
100%,    k=5000

Time Elapsed: 2.92432 Seconds


With instruction cache disabled:
10%,     k=500
20%,     k=1000
30%,     k=1500
40%,     k=2000
50%,     k=2500
60%,     k=3000
70%,     k=3500
80%,     k=4000
90%,     k=4500
100%,    k=5000

Time Elapsed: 15.845614 Seconds


Example Succeeded
```

