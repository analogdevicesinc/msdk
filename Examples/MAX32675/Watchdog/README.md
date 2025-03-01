## Description

A demonstration of the windowed features of the watchdog timer.

When the application begins, it initializes and starts the watchdog timer.  
The application then begins to reset the watchdog within the allowed window.  
Use SW0 and SW1 buttons on the board to trigger WDT.

Please check the board.c file in ${MSDKPath}\Libraries\Boards\MAX32675\${BoardName}\Source path to learn push button and LED pins for specific board.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   if you have EvKIT close jumper JP1 and JP2 (LED0_EN, LED1_EN).

## Expected Output

The Console UART of the device will output these messages:

```

************** Watchdog Timer Demo ****************
Watchdog timer is configured in Windowed mode. You can
select between two tests: Timer Overflow and Underflow by pressing SW0 and SW1
Press buttons to create watchdog interrupt and reset:
Push button 0 = timeout and reset program
Push button 1 = reset program

Enabling Timeout Interrupt...

TIMEOUT!

Watchdog Reset occured too late (OVERFLOW)
```

