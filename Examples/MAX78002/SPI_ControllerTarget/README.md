## Description

This example demonstrates a SPI transaction between two distinct SPI peripherals on the MAX78002. 

SPI1 is setup as the controller (L. Master) in this example and is configured by default to send/receive 1024 8-bit words to and from the slave. Likewise, SPI0 is setup as the slave and is also expecting to both send and receive 1024 8-bit words to and from the master.

Once the controller ends the transaction, the data received by the controller (L. Master) and the target (L. Slave) is compared to the data sent by their counterpart to ensure all bytes were received properly.

This example also demonstrates the feature to use custom Target Selects (TS) for the Controller. The SPI v2 Driver will automatically assert/deassert the custom TS pin during transactions. Set the CUSTOM_TARGET macro to 1 to use the custom target (P0.9). To use the default TS pins, set the CUSTOM_TARGET macro to 0 instead. Note, the TS pin connections must be reconnected depending on the selected CUSTOM_TARGET option.

Target Select (CS) Pin Connections
- CUSTOM_TARGET (0): Connect (P0.4 to P0.20).
- CUSTOM_TARGET (1): Connect (P0.4 to P0.12).

## Software

This example uses the SPI v2 Library. To use the previous SPI library, set `MXC_SPI_BUILD_LEGACY=1` in the Project's project.mk file.

### Porting Guide

The SPI v2 Library still supports the previous SPI API - meaning the previously existing function prototypes have not changed. However, there are functional differences with the interrupt handling for both SPI DMA and blocking SPI transactions that must be updated when porting a project from using the previous SPI API to SPI v2.

#### SPI v2 API Differences

##### SPI Init Function

The `MXC_SPI_Init(...)` function is still supported with SPI v2, but there is some added overhead due to the limited settings that this function can set.

Use the `MXC_SPI_Init_v2(...)` function for 1) to decrease overhead of initialization and 2) to give the caller more control in the SPI setup.

**`mxc_spi_init_t init` Fields**
- `mxc_spi_regs_t *spi`              //<== SPI Instance
- `mxc_gpio_cfg_t *spi_pins`         //<== (Optional) Caller supplied SPI pins
- `mxc_spi_type_t type`              //<== Controller (L. Master) or Target (L. Slave) Modes
- `uint32_t freq`                    //<== SPI Frequency
- `mxc_spi_clkmode_t clk_mode`       //<== Clock Mode (CPOL:CPHA)
- `mxc_spi_interface_t mode`         //<== Select Interface (Standard 4-wire, 3-wire, dual, quad)
- `mxc_spi_tscontrol_t ts_control`   //<== HW Auto, SW Driver, or SW Application Target Control
- `mxc_spi_target_t target`          //<== Target settings (custom TS pins, init mask, active polarity) 
- `mxc_gpio_vssel_t vssel`           //<== Select Pin Voltage Level (VDDIO/VDDIOH)
- `bool use_dma`                     //<== TRUE/FALSE DMA setting
- `mxc_dma_regs_t *dma`              //<== DMA Instance
- `mxc_spi_callback_t callback`      //<== Set Callback function for end of transaction
- `void* callback_data`              //<== Data to pass through callback function

##### SPI DMA Interrupt Handling

```c
void DMA_TX_IRQHandler(void)
{
    MXC_SPI_DMA_TX_Handler(SPI);
}

void DMA_RX_IRQHandler(void)
{
    MXC_SPI_DMA_RX_Handler(SPI);
}

```
The previous SPI API uses the MXC_DMA_Handler, but SPI v2 supplies its own Handler processing functions to call for TX and RX DMA.

##### SPI DMA Setup
```c
    ...
    TX_DMA_CH = MXC_SPI_DMA_GetTXChannel(SPI);
    RX_DMA_CH = MXC_SPI_DMA_GetRXChannel(SPI);

    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(TX_DMA_CH));
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(RX_DMA_CH));

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(TX_DMA_CH), DMA_TX_IRQHandler);
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(RX_DMA_CH), DMA_RX_IRQHandler);

    MXC_SPI_MasterTransactionDMA(&req);
    ...
```
Following the DMA channel interrupt changes from the previous section, it is recommended to set up a generic name DMA TX/RX vector because the the TX and RX DMA channels won't always acquire DMA_CH0 and DMA_CH1, respectively.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the SPI pins on headers JH6 and JH9. (P0.4-->P0.20 (CS), P0.5-->P0.21 (MOSI), P0.6-->P0.22 (MISO), and P0.7-->P0.23 (SCK))
-   If custom target select pin was selected, re-connect the CS pins (P0.4-->P0.12).

## Expected Output

The Console UART of the device will output these messages:

```
************************ SPI Controller-Target Example ************************
This example sends data between two SPI peripherals in the MAX78002.
SPI1 is configured as the target (L. Slave) and SPI0 is configured
as the controller (L. Master). Each SPI peripheral sends 1024 bytes
on the SPI bus. If the data received by each SPI instance matches the
the data sent by the other instance, then the green LED will illuminate,
otherwise the red LED will illuminate.

Press PB1 to begin transaction.

Example Succeeded
```
