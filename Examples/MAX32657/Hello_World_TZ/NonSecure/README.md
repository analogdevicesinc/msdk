## Description

A basic getting started program using Cortex-M33 TrustZone feature.

This version of Hello_World prints an incrementing count to the console UART and toggles a LED0 every 500 ms from the non-secure world. Incrementing the count is done in the Secure world.

Under the **Hello_World_TZ** example, the **Secure** and **NonSecure** projects demonstrate building the secure and non-secure code into a single image. You do not build directly in the **NonSecure** project.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

The Non-Secure Project must set these flags:
- `TRUSTZONE=1`
- `MSECURITY_MODE=NONSECURE`

## Required Connections

TODO

## Expected Output

TODO

```
TODO
```
