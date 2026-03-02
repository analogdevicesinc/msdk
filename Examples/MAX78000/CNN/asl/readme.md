# Description

- **What:** A small CNN example for American Sign Language (ASL) classification targeting the MAX78000 MCU. It demonstrates how to build the firmware and perform on-device inference.

## Getting Started

- This is a small CNN example for ASL classification targeting the MAX78000 MCU. The ASL utilizes an ASL dataset which consists of 26 hand symbols, one for each letter in the alphabet, and a total of 81000 images.

- Compilation (use repository-specific build system; examples below are typical):

```bash
cd Examples/MAX78000/CNN/asl
make
```

### Build Configuration Guidelines

- Use the MSDK checkout and toolchain versions compatible with the example. Mismatched toolchains often cause build failures.
- If the example uses a provided `ai8xize` or other conversion helper, follow its documented flags for quantization and IO layout (NHWC vs NCHW).

### Required Connections

- **Power & Debug:** Power the MAX78000 EVKit the board manual via USB port and flash the binary used by your toolchain.
- **Serial I/O:** Connect the board’s UART-to-USB to your host PC for console output (common baud rate: 115200). Use a serial terminal (e.g., PuTTY, miniterm) to view logs and results.
- **Host:** USB connection for flashing and (optionally) to stream input images from the host to the board if the example supports host-side image injection.

Check out the [MAX78000 EvKit](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html) page for details.

### Expected Output

- **Runtime Behavior:** Once flashed, the firmware runs the a sample input (provided in sampledata.h) and prints key information to the serial console.
