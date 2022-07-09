## Description
This Example shows how to wake up a device after it is asleep with a wake up timer.  After a defined number of seconds it will wake up after going to sleep.

-    PB1: P0.26/SW1

## Setup

##### Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP22(RX_SEL) and JP23(TX_SEL) to UART1 header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
/************** Wakeup timer example ********************/
This example is to show how the Wakeup timer is used and configured
Press PB1 to put the chip into sleep and then the wakeup timer will wake up in 5000 Miliseconds
Entering SLEEP mode.
Waking up from SLEEP mode.
```
