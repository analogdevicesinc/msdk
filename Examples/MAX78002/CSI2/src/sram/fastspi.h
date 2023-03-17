#include "fastspi_config.h"

static volatile bool g_tx_done = 0;
static volatile bool g_rx_done = 0;
static volatile bool g_master_done = 0;

static const mxc_gpio_cfg_t spi_ss_pin = {
    .port = SPI_SS_PORT,
    .mask = SPI_SS_PIN,
    .func = MXC_GPIO_FUNC_ALT1,
    .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
    .vssel = MXC_GPIO_VSSEL_VDDIOH
};

static const mxc_gpio_cfg_t spi_pins = {
    .port = SPI_PINS_PORT,
    .mask = SPI_PINS_MASK,
    .func = MXC_GPIO_FUNC_ALT1,
    .pad = MXC_GPIO_PAD_NONE,
    .vssel = MXC_GPIO_VSSEL_VDDIOH
};


// TODO:  Generalize to multiple SPI instances
int spi_init();
int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert, bool use_dma, bool block);
