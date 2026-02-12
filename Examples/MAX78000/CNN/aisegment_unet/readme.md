# MAX78000 UNet Semantic Segmentation Example

This page presents a UNet-based semantic segmentation example designed for the MAX78000 microcontroller. It demonstrates how to build the firmware and perform on-device inference, highlighting the capabilities of low-power embedded semantic segmentation.

## Getting Started

- Start by preparing a MAX78000-compatible model from a trained UNet. In this example, the model is trained using the [ai8x-training](https://github.com/analogdevicesinc/ai8x-training/blob/develop/scripts/train_aisegment_unet.sh) repository. To convert the PyTorch model for use with the MAX78000 MCUs, utilize the [ai8x-synthesis](https://github.com/analogdevicesinc/ai8x-synthesis/blob/develop/gen-demos-max78000.sh) tools.

- Compilation (use repository-specific build system; examples below are typical):

```bash
cd Examples/MAX78000/CNN/aisegment_unet
make
```

### Build Configuration Guidelines
- Use the MSDK checkout and toolchain versions compatible with the example. Mismatched toolchains often cause build failures.
- If the example uses a provided `ai8xize` or other conversion helper, follow its documented flags for quantization and IO layout (NHWC vs NCHW).

### Required Connections

- **Power & Debug:** Power the MAX78000 EVKit board manually via USB port and flash the binary used by your toolchain.
- **Serial I/O:** Connect the board’s UART-to-USB to your host PC for console output (common baud rate: 115200). Use a serial terminal (e.g., PuTTY, miniterm) to view logs and results.
- **Host:** USB connection for flashing and (optionally) to stream input images from the host to the board if the example supports host-side image injection.

Check out the [MAX78000 EvKit](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max78000evkit.html) page for details.

### Expected Output

- **Runtime Behavior:**
Once flashed, the firmware runs the segmentation network on a sample input (provided in sampledata.h), compares the result to the expected output (sampleoutput.h), and prints key information to the serial console. The output generally includes the following:
  - Whether the test passed or failed, based on whether the inference output matches the expected result.
  - Inference timing, such as latency or cycle count.
  - Basic statistics, including mean confidence and per-class scores.

- **Performance:**
The example will display or log inference time and energy consumption for the converted model. Use these metrics to verify that your application meets real-time requirements.
