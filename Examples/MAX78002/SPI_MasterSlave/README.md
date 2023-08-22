## Description

This example demonstrates a SPI transaction between two distinct SPI peripherals on the MAX78002. 

SPI1 is setup as the master in this example and is configured by default to send/receive 1024 8-bit words to and from the slave. Likewise, SPI0 is setup as the slave and is also expecting to both send and receive 1024 8-bit words to and from the master.

Once the master ends the transaction, the data received by the master and the slave is compared to the data sent by their counterpart to ensure all bytes were received properly.

## Software

### Porting Projects to use SPI v2

The latest SPI examples in the MSDK defaults to build the SPI v1 libraries. Set `MXC_SPI_BUILD_V1=0` in the Project's project.mk file to use the SPI v2 API.

This guide shows how to update an existing project that is using the SPI v1 API to SPI v2. The SPI v2 Library still supports the SPI v1 function prototypes for backwards-compatibility with the main difference in the SPI DMA interrupt handling (see **SPI DMA Interrupt Handling** section below for more info).

Note: The SPI v2 API is only a drop in replacement to SPI v1 if SPI DMA is **not** used; should the user choose to continue building with the SPI v1 convention but with the underlying SPI v2 implementation. This porting guide demonstrates how to use the full extent of the SPI v2 features.

#### SPI Init Function

The input parameters for the `MXC_SPI_Init(...)` function were updated in SPI v2 to allow for more user-selectable options during initialization. This should not cause any errors or behavioral differences with the `MXC_SPI_Init(...)` function when switching between SPI v1 and SPI v2 builds as the input parameters were essentially re-mapped to more descriptive names.

Previously: `int MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves, unsigned ssPolarity, unsigned int hz, mxc_spi_pins_t pins)`

Updated: `int MXC_SPI_Init(mxc_spi_regs_t *spi, mxc_spi_type_t controller_target, mxc_spi_interface_t if_mode, int numTargets, uint8_t ts_active_pol_mask, uint32_t freq, mxc_spi_pins_t pins)`

Input Parameters:
- `mxc_spi_regs_t *spi` remains unchanged.
- `int masterMode` -> `mxc_spi_type_t controller_target`. The enum `mxc_spi_type_t` was added for increased code readability.
- `int quadModeUsed` -> `mxc_spi_interface_t if_mode`. Previously, the `MXC_SPI_Init(...)` function could only select between standard (4wire) and quad interface modes. With SPI v2, the user can select either standard (`MXC_SPI_INTERFACE_STANDARD`), quad (`MXC_SPI_INTERFACE_QUAD`), 3wire (`MXC_SPI_INTERFACE_3WIRE`), or dual (`MXC_SPI_INTERFACE_DUAL`) mode.
- `int numSlaves` -> `int numTargets`. SPI v2 does not use this parameter and was kept to continue supporting SPI v1.
- `unsigned ssPolarity` -> `uint8_t ts_active_pol_mask`. Updated to more descriptive name.
- `unsigned int hz` -> `uint32_t freq`.
- `mxc_spi_pins_t pins` remains unchanged.

#### SPI Config Function

The `int MXC_SPI_Config(mxc_spi_cfg_t cfg)` function was added to reduce the number of helper function calls to set the appropriate settings that the `MXC_SPI_Init(...)` function did not set.

This function also sets up the DMA and acquires DMA TX/RX channels for SPI DMA transactions.

`mxc_spi_cfg_t` struct:
- `mxc_spi_regs_t *spi` - Select SPI Instance to configure.
- `mxc_spi_clkmode_t clk_mode` - Select clock mode.
- `uint8_t frame_size` - Select single frame size (2 - 16 bits).
- `bool use_dma_tx` - Enable SPI DMA TX (acquire and configure TX channel).
- `bool use_dma_rx` - Enable SPI DMA RX (acquire and configure RX channel).
- `mxc_dma_regs_t *dma` - Select DMA Instance to configure for SPI DMA (Valid only if `use_dma_tx` or `use_dma_rx` is set to true).

#### SPI Transaction Functions

The SPI v2 Libraries follows the terms used in the user guide: Controller and Target instead of Master and Slave, respectively.

- `MXC_SPI_MasterTransaction(...)`        -> `MXC_SPI_ControllerTransaction(...)`
- `MXC_SPI_MasterTransactionAsync(...)`   -> `MXC_SPI_ControllerTransactionAsync(...)`
- `MXC_SPI_MasterTransactionDMA(...)`     -> `MXC_SPI_ControllerTransactionDMA(...)`
- `MXC_SPI_SlaveTransaction(...)`         -> `MXC_SPI_TargetTransaction(...)`
- `MXC_SPI_SlaveTransactionAsync(...)`    -> `MXC_SPI_TargetTransactionAsync(...)`
- `MXC_SPI_SlaveTransactionDMA(...)`      -> `MXC_SPI_TargetTransactionDMA(...)`

#### SPI DMA Setup for `MXC_SPI_ControllerTransactionDMA(...)`

The SPI v2 library allows for more flexibility in setting generic DMA TX/RX channel vectors for SPI DMA transactions during run-time. Compared to SPI v1 where the user must know the acquired SPI DMA TX/RX channel numbers and define the appropriate DMA channel handlers before compile-time.

There are two ways to initialize and configure the DMA before starting a SPI DMA transaction.

Method 1: Call `int MXC_SPI_DMA_Init(mxc_spi_regs_t *spi, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx)`.

Method 2: Set up the DMA options in the `mxc_spi_cfg_t` struct and call `int MXC_SPI_Config(mxc_spi_cfg_t cfg)`.

```c
    ...
    // Initialize SPI DMA ahead of time.
    MXC_SPI_DMA_Init(SPI, DMA, true, true);

    int TX_DMA_CH = MXC_SPI_DMA_GetTXChannel(SPI);
    int RX_DMA_CH = MXC_SPI_DMA_GetRXChannel(SPI);

    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(TX_DMA_CH));
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(RX_DMA_CH));

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(TX_DMA_CH), DMA_TX_IRQHandler);
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(RX_DMA_CH), DMA_RX_IRQHandler);

    MXC_SPI_ControllerTransactionDMA(&req);
    ...
```
The DMA is initialized in `MXC_SPI_DMA_Init(...)` or `MXC_SPI_Config(...)`. This provides information on what DMA channels were acquired for a SPI instance's TX and RX DMA before calling the DMA transaction function. Following the example above, it is recommended to set up a generic-named DMA TX/RX vector because the SPI TX and RX DMA channels won't always acquire DMA_CH0 and DMA_CH1, respectively. 

#### SPI DMA Interrupt Handling

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
The SPI v1 API requires `MXC_DMA_Handler()` to be called in the TX and RX DMA Channel interrupt handlers. Following the generic vector names used in the previous section, the SPI v2 supplies its own TX/RX DMA Handler processing functions (`MXC_SPI_DMA_RX_Handler(...)` and `MXC_SPI_DMA_RX_Handler(...)`) that must be called within their appropriate DMA channel interrupt handlers.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

Set `MXC_SPI_BUILD_V1=0` to build the SPI v2 libraries.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Connect the SPI pins on headers JH6 and JH9. (P0.4-->P0.20 (CS), P0.5-->P0.21 (MOSI), P0.6-->P0.22 (MISO), and P0.7-->P0.23 (SCK))

## Expected Output

The Console UART of the device will output these messages:

```
************************ SPI Master-Slave Example ************************
This example sends data between two SPI peripherals in the MAX78002.
SPI1 is configured as the slave and SPI0 is configured as the master.
Each SPI peripheral sends 1024 bytes on the SPI bus. If the data received
by each SPI instance matches the data sent by the other instance, the
green LED will illuminate, otherwise the red LED will illuminate.

Press PB1 to begin transaction.

Example Succeeded
```