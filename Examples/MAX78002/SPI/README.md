## Description

This example configures the SPI to send data between the MISO (P0.22) and
MOSI (P0.21) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, reset the BLOCKING macro to 0 and set the NON_BLOCKING macro to 1.  To use DMA transactions, set the DMA macro to 1 instead.

This example also demonstrates the feature to use custom Target Selects that the SPI v2 Driver will automatically assert/deassert during transactions. Set the CUSTOM_TARGET macro to 1 to use the custom target. To use the default TS pins, set the CUSTOM_TARGET macro to 0 instead.

## Software

This example uses the SPI v2 Library. To use the previous SPI library, set `MXC_SPI_BUILD_LEGACY=1` in the Project's project.mk file.

### Porting Guide

The SPI v2 Library is backwards compatible with the previous SPI API - meaning the previously existing function prototypes have not changed. There are functional differences with SPI DMA interrupt handling that must be updated when porting a project from using the previous SPI API to SPI v2.

#### SPI v2 API Differences

##### SPI Init Function

The `MXC_SPI_Init(...)` function is still supported with SPI v2, but there is some added overhead due to the limited settings that this function can set.

Use the `MXC_SPI_Init_v2(...)` function for 1) to decrease overhead and 2) to give the caller more control in the SPI setup.

**`mxc_spi_init_t init` Fields**
- `mxc_spi_regs_t *spi`              //<== SPI Instance
- `mxc_gpio_cfg_t *spi_pins`         //<== (Optional) Caller supplied SPI pins.
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

##### Blocking SPI Transaction (MXC_SPI_MasterTransaction(...))
```c
void SPI_IRQHandler(void)
{
    MXC_SPI_Handler(SPI); // Or MXC_SPI_AsyncHandler(SPI); Same function, different names.
}   

    ...
    NVIC_EnableIRQ(SPI);
    MXC_SPI_MasterTransaction(&req);
    ...
```
The blocking SPI transaction function is now interrupt driven for SPI v2 - meaning the SPI instances' IRQ must be enabled and the MXC_SPI_Handler(...) or MXC_SPI_AsyncHandler(...) function must be called in the interrupt routine.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
**************************** SPI CONTROLLER TEST *************************
This example configures the SPI to send data between the MISO (P0.22) and
MOSI (P0.21) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

Performing blocking (synchronous) transactions...
--> 2 Bits Transaction Successful
--> 3 Bits Transaction Successful
--> 4 Bits Transaction Successful
--> 5 Bits Transaction Successful
--> 6 Bits Transaction Successful
--> 7 Bits Transaction Successful
--> 8 Bits Transaction Successful
--> 9 Bits Transaction Successful
-->10 Bits Transaction Successful
-->11 Bits Transaction Successful
-->12 Bits Transaction Successful
-->13 Bits Transaction Successful
-->14 Bits Transaction Successful
-->15 Bits Transaction Successful
-->16 Bits Transaction Successful

Example Complete.
```
