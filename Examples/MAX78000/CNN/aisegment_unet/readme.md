## Description
This provides an UNet-style semantic segmentation example targeting the MAX78000 MCU. This demonstrates model conversion, quantization, firmware build, and on-device inference for low-power embedded semantic segmentation.

## Software

### Project Usage

- Prepare a MAX78000-compatible model (kmodel) from a trained UNet. Typical steps: train/export a model (e.g., Keras `.h5` or `.tflite`), run the example conversion/quantization script to produce the MAX78000 model, place the produced model in the example folder, then build and flash the firmware.

- Example quick commands:

```bash
python -m venv venv
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
pip install -r requirements.txt  # or: pip install numpy pillow pyserial
python convert.py --input path/to/unet.h5 --output path/to/unet.kmodel --input-shape 1,128,128,3
```

- Build/flash (use repository-specific build system; examples below are typical):

```bash
cd Examples/MAX78000/CNN/aisegment_unet
make
make flash
```

### Project-Specific Build Notes
- Use the MSDK checkout and toolchain versions compatible with the example. Mismatched toolchains often cause build failures.
- Verify the build system expects the kmodel filename/location used by the firmware (place the converted `.kmodel` in the example `model/` or project root if required).
- If the example uses a provided `ai8xize` or other conversion helper, follow its documented flags for quantization and IO layout (NHWC vs NCHW).
- Some TensorFlow layers/operators may be unsupported on MAX78000 — simplify or replace unsupported ops before conversion.

## Required Connections

- **Power & Debug:** Power the MAX78000 EV kit per the board manual and connect the debugger (SWD) or programmer used by your toolchain for flashing.
- **Serial I/O:** Connect the board’s UART-to-USB to your host PC for console output (common baud rate: 115200). Use a serial terminal (e.g., PuTTY, miniterm) to view logs and results.
- **Host:** USB connection for flashing and (optionally) to stream input images from the host to the board if the example supports host-side image injection.

## Expected Output

- **Runtime behavior:** After flashing, the firmware runs the segmentation network on sample inputs (either on-board samples or host-sent images) and prints results to the serial console. The output typically includes:
  - Inference timing (latency / cycles).
  - Basic statistics (mean confidence, per-class scores).
  - Segmentation map representation — either printed as ASCII, as flattened class indexes, or saved/displayed if an attached display or host viewer is used.

- **Performance:** Example prints or logs that show inference time and memory usage for the converted model; use these to validate real-time constraints.

- **Debugging tips:** If outputs are empty or all-zero, verify preprocessing (resize, channel order, normalization) and confirm the kmodel matches the input shape expected by the firmware.
