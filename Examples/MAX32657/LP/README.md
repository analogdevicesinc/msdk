## Description

This example demonstrates the steps to enter and exit the various low-power modes supported by the MAX32655.

Users may decide which low power modes they wish to enter by setting or clearing the DO_SLEEP, DO_BACKUP, DO_STANDBY, and DO_POWERDOWN macros at the top of main.

Users may also select whether they want to use a GPIO or RTC wakeup source. To use a GPIO wakeup source set the USE_BUTTON macro to 1; this will configure the GPIO connected to PB0 to send a wakeup signal each time the button is pressed. To use the RTC wakeup source set the USE_ALARM macro to 1; this will configure an RTC alarm to wake the device every DELAY_IN_SEC seconds.

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

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****Low Power Mode Example****

This code cycles through the MAX32657 power modes, using a push button (PB0) to exit from each mode and enter the next.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.

```
