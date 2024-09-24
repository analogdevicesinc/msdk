## Description

A basic getting started program using the Cortex-M33 TrustZone feature.

This version of Hello_World prints an incrementing count to the console UART and toggles a LED0 every 500 ms from the Non-Secure world. Incrementing the count is done in the Secure world.

Under the **Hello_World_TZ** example, the **Secure** and **NonSecure** projects demonstrate building the secure and non-secure code into a single image. You do not build directly in the **NonSecure** project.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

The **Secure** project must have the **partition_max32657.h** file included for the SAU setup done in `SystemInit()` at startup.

The Secure Project must set these flags:
- `TRUSTZONE=1`
- `MSECURITY_MODE=SECURE`
- `NONSECURE_CODE_DIR=path/to/nonsecure/project`

## Required Connections

TODO

## Expected Output

TODO

```
TODO
```
