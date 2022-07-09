## Description

The example demonstartes the use of Semaphore. 

A semaphore is shared between task A and task B. The buttons are used to start and stop the tasks (SW2 = Task A, SW3 = Task B). The first time the button is pressed the corresponding task will start and acquire the semaphore if it is available. The second time the button is pressed the corresponding task will end and free the semaphore. If a task is initiated and the sempahore is unavailable then the task will fail to run.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Semaphore Example *****
SW1 = A tries to write
SW2 = B tries to write

Semaphore acquired.
Semaphore locked.
Semaphore free.

Example running.
A acquired semaphore!
A started writing...
var = 100

B can't write right now.

var = 100
A stopped writing.
A dropped the semaphore...

B acquired semaphore!
B started writing...
var = 200

A can't write right now.

var = 200
```
