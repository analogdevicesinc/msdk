## Description

The example demonstartes the use of Semaphore. 

A semaphore is shared between task A and task A and task B. 
Use PB0 to start task A by acquiring the semaphore. 
Task B cannot start unless A drops the semaphore. 
Use PB1 to start task B.
Please note that: Depend on the number of button on your board the example output might be different.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Semaphore Example *****
Push Button 0  = A tries to write
Push Button 1  = B tries to write

Semaphore acquired.
Semaphore locked.
Semaphore free.

Example running.
A acquired semaphore!
A started writing...
var = 100

A stopped writing.
A dropped the semaphore...

```
