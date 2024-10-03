# Developer Notes

## SPI v2 Library

The SPI v2 Library is the latest version of the MSDK SPI drivers which highlights:

- Target Select (TS) Control Scheme which provides users the option to drive their own TS pins.
- Optional`MXC_SPI_Config(...)` and `mxc_spi_cfg_t` struct to reduce the use of multiple functions to select proper SPI settings.
- Re-mapped the `MXC_SPI_Init(...)` function input parameters (same behavior as SPI v1).
- Allow for re-arming an SPI transaction within the callback function for chained SPI messages.
- Decrease in setup overhead in a Transaction function call. Less nested function calls within drivers.
- Improved SPI DMA support and DMA Channel IRQ vector flexibility by providing acquired DMA channel numbers before a SPI DMA transaction call.
- The use of Controller and Target terms instead of Master and Slave, respectively.
- Still supports SPI v1 function prototypes for backwards-compatibility.
- Bug fixes from the SPI v1 API.

### SPI v2 Supported Parts

- MAX32572
- MAX32690
- MAX78002

### Porting Projects to use SPI v2

The latest SPI examples in the MSDK defaults to build the SPI v1 libraries. Set the `MXC_SPI_VERSION` [build configuration variable](#build-configuration-variables) to `v2` (case sensitive) use the SPI v2 API.

This guide shows how to update an existing project that is using the SPI v1 API to SPI v2. The SPI v2 Library still supports the SPI v1 function prototypes for backwards-compatibility with the main difference being the SPI DMA interrupt handling (see [SPI DMA Interrupt Handling](#spi-dma-interrupt-handling) section below for more info).

Note: The SPI v2 API is only a drop in replacement to SPI v1 if SPI DMA is **not** used; should the user choose to continue building with the SPI v1 convention but with the underlying SPI v2 implementation. This porting guide demonstrates how to use the full extent of the SPI v2 features.

#### SPI Init Function

The input parameters for the `MXC_SPI_Init(...)` function were updated in SPI v2 to allow for more user-selectable options during initialization. This should not cause any errors or behavioral differences with the `MXC_SPI_Init(...)` function when switching between SPI v1 and SPI v2 builds as the input parameters were essentially re-mapped to more descriptive names.

SPI v1:

    :::C
    int MXC_SPI_Init(mxc_spi_regs_t *spi,
                    int masterMode,
                    int quadModeUsed,
                    int numSlaves,
                    unsigned ssPolarity,
                    unsigned int hz,
                    mxc_spi_pins_t pins)

SPI v2:

    :::C
    int MXC_SPI_Init(mxc_spi_regs_t *spi,
                    mxc_spi_type_t controller_target,
                    mxc_spi_interface_t if_mode,
                    int numTargets,
                    uint8_t ts_active_pol_mask,
                    uint32_t freq,
                    mxc_spi_pins_t pins)

Input Parameters:

- `mxc_spi_regs_t *spi` remains unchanged.
- `int masterMode` -> `mxc_spi_type_t controller_target`. The enum `mxc_spi_type_t` was added for increased code readability.
- `int quadModeUsed` -> `mxc_spi_interface_t if_mode`. Previously, the `MXC_SPI_Init(...)` function could only select between standard (4wire) and quad interface modes. With SPI v2, the user can select either standard (`MXC_SPI_INTERFACE_STANDARD`), quad (`MXC_SPI_INTERFACE_QUAD`), 3wire (`MXC_SPI_INTERFACE_3WIRE`), or dual (`MXC_SPI_INTERFACE_DUAL`) mode.
- `int numSlaves` -> `int numTargets`. SPI v2 does not use this parameter and was kept to continue supporting SPI v1.
- `unsigned ssPolarity` -> `uint8_t ts_active_pol_mask`. Updated to a more descriptive name.
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

#### SPI Request Struct

`mxc_spi_req_t` struct:

- `mxc_spi_regs_t *spi` - Remains unchanged.
- `int ssIdx` - Remains unchanged.
- `int ssDeassert` - Remains unchanged.
- `uint8_t *txData` - Remains unchanged.
- `uint8_t *rxData` - Remains unchanged.
- `uint32_t txLen` - Remains unchanged.
- `uint32_t rxLen` - Remains unchanged.
- `uint32_t txCnt` - Not used in SPI v2.
- `uint32_t rxCnt` - Not used in SPI v2.
- `mxc_spi_callback_t completeCB` - Type was renamed, but funtionally, remains unchanged.

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

    :::C
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

The DMA is initialized in `MXC_SPI_DMA_Init(...)` or `MXC_SPI_Config(...)`. This provides information on what DMA channels were acquired for a SPI instance's TX and RX DMA before calling the DMA transaction function. Following the example above, it is recommended to set up a generic-named DMA TX/RX vector because the SPI TX and RX DMA channels won't always acquire DMA_CH0 and DMA_CH1, respectively.

#### SPI DMA Interrupt Handling

    :::C
    void DMA_TX_IRQHandler(void)
    {
        MXC_SPI_DMA_TX_Handler(SPI);
    }

    void DMA_RX_IRQHandler(void)
    {
        MXC_SPI_DMA_RX_Handler(SPI);
    }

The SPI v1 API requires `MXC_DMA_Handler()` to be called in the TX and RX DMA Channel interrupt handlers. Following the generic vector names used in the previous section, the SPI v2 supplies its own TX/RX DMA Handler processing functions (`MXC_SPI_DMA_RX_Handler(...)` and `MXC_SPI_DMA_RX_Handler(...)`) that must be called within their appropriate DMA channel interrupt handlers.
