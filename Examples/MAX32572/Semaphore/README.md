## Description

The example demonstartes the use of Semaphore. 

A semaphore is shared between task A and task A and task B. Use PB1 to start task A by acquiring the semaphore. Task B cannot start unless A drops the semaphore. Use PB2 to start task B.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Semaphore Example *****
KEY_1 (P3.07)= A tries to write
KEY_2 (P3.06)= B tries to write

A acquired semaphore!
A started writing...
Shared Variable = 100

A stopped writing.
A dropped the semaphore...
```
