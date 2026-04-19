## Description

This example showcases the various settings which can be configured to put the device into lower power states. Namely, putting the device RAMs in light sleep and shutdown modes, and cycling through the device sleep modes.

The wakeup source can be configured to be either the RTC clock or the push button based on the selection of the "USE_BUTTON" and "USE_ALARM" macros. Additionally, sleep modes which are cycled through can be enabled and disabled with the "DO_SLEEP", "DO_DEEPSLEEP", "DO_BACKGROUND" and "DO_BACKUP" macros.


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
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32650 power modes, using a push button (SW2) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs placed in LIGHT SLEEP mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Entering DEEPSLEEP mode.
Entering SLEEP mode.
```
