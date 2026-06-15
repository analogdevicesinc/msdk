## Description
UART with DMA TX test code on the MAX32657 EVK. This example use DMA with UART of MAX32657 EVK to send a long transfer (512 bytes) to test the DMA RX of MAX32657 (DUT). The baud-rate can be change depending on the UART_BAUD macro definition from (115200 to 921600).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect the GND of 2 MAX32657 EVKs.
-   Connect the UART TX of MAX32657 EVK (UART_DMA_TX_32657) to the UART RX pin of MAX32657 EVK (UART_DMA_32657).
-   Power on and Press the button RESET to send 512 bytes via DMA from UART_DMA_TX_32657 EVK to UART_DMA_32657 EVK.

## Expected Output

The waveform of 512 bytes which are output from UART TX of UART_DMA_TX_32657 EVK