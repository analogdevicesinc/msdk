## Description

This application cycles through the various low power modes. Switching between the different modes can be triggered by either pressing a button (PB1/SW4) or automatically with an RTC alarm. This is configured with the define statements at the top of the application.

Following modes can be tested:

 *            Active mode power with all clocks
 *            Active mode power with peripheral clocks disabled (if USE_CONSOLE is 0)
 *            SLEEP mode
 *            LPM mode
 *            UPM mode
 *            STANDBY mode
 *            BACKUP mode
 *            Power Down mode

## Software

### WARNING: Low-Power Mode Flash / Debug Access

Microcontrollers usually shutdown SWD /JTAG debug interfaces in very low-power mode operation.

This can cause an MCU to permanently lose debug access because, upon reset, the device may enter LP modes so fast that the debug probe cannot attach to and reset the target core before the debug interface is shutdown. This is true regardless of the debug probe you choose to use.

To avoid this problem, please observe the 2-second delay at the beginning of the main() function. If you find that your MCU can no longer be programmed or debugged after flashing an LP example, you can attempt to recover it using the following procedure from the MSDK User Guide:
https://analogdevicesinc.github.io/msdk//USERGUIDE/#how-to-unlock-a-microcontroller-that-can-no-longer-be-programmed

Another option for production builds would be to implement Lockout Protection using a free GPIO attached to a switch. Below is some example code for how this may be implemented:

```C
#ifdef LOCKOUT_PROTECT
/**
 * If debugger lockout protection is enabled,
 * define a switch for enabling the protection.
 *
 * Note this is not a board-specific configuration;
 * Please adapt the GPIO pin(s) to your specific hardware.
 **/
static const mxc_gpio_cfg_t lockoutConfig = {
    .func = MXC_GPIO_FUNC_ALT1,
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_0,
    .pad = MXC_GPIO_PAD_PULL_UP
};
#endif

int main(void) {
#ifdef LOCKOUT_PROTECT
    MXC_GPIO_Config(&lockoutConfig);

    // If lockout switch is pulled low, prevent sleep or trigger a long delay
    if (MXC_GPIO_InGet(lockoutConfig.port, lockoutConfig.mask)) {
        MXC_Delay(MXC_DELAY_SEC(2));
        allowSleep = 0;
    }
#endif
```

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
****Low Power Mode Example****

This code cycles through the MAX78002 power modes, using the RTC alarm to exit from each mode.  The modes will change every 4 seconds.

Set the EvKit power monitor display to System Power Mode to measure the power in each mode.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering Low power mode.
Waking up from Low power mode.
Entering STANDBY mode.
Waking up from STANDBY mode.
Entering BACKUP mode.

```

