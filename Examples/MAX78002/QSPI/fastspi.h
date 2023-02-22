#include "fastspi_config.h"

static volatile bool g_tx_done = 0;
static volatile bool g_rx_done = 0;

int spi_init();
int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert, bool use_dma, bool block);
