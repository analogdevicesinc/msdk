## Description
UART for DMA TX and RX of MAX32657 EVK. The baud-rate can be change depending on the UART_BAUD macro definition from (115200 to 921600).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect the GND of MAX32670 EVK to the GND of MAX32657 EVK.
-   Connect the P0.15 (UART 2 TX) of MAX32670 EVK to the UART RX pin of MAX32657 EVK.
-   Connect the a logic analyzer channel to UART TX pin of MAX32657 EVK.

## Expected Output

-   Ensure MAX32670 test code and MAX32657 (this file) setup UART in same baud-rate.
-   Run the logic analyzer and reset the MAX32657 to check the UART DMA TX waveform.
-   After MAX32657 finish to send UART TX, press the button P0_21 on MAX32670 EVK
-   In debug mode of MAX32657 check test_result == 0 and rx_index == 512
-   The MAX32657 can success to receive with baud-rate from 115200 to 460800 (NG with 921600)