/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/*******   Includes      *******/
#include <string.h>
#include "emac.h"
#include "emac_reva.h"

/*******   Variables     *******/
mxc_emac_reva_device_t mxc_emac_context;
static mxc_emac_reva_device_t *emac = &mxc_emac_context;

/*******   Functions     *******/
/* ************************************************************************* */
/* Private Functions                                                         */
/* ************************************************************************* */
static void emac_mdio_write(unsigned char reg, uint16_t value)
{
    unsigned int netctl;
    unsigned int netstat;
    unsigned int frame;

    netctl = EMAC_READL(emac, cn);
    netctl |= EMAC_BIT(CN, MPEN);
    EMAC_WRITEL(emac, cn, netctl);

    frame = (EMAC_BF(PHY_MT, SOP, 1) | EMAC_BF(PHY_MT, OP, 1) |
             EMAC_BF(PHY_MT, PHYADDR, emac->phy_addr) | EMAC_BF(PHY_MT, REGADDR, reg) |
             EMAC_BF(PHY_MT, CODE, 2) | EMAC_BF(PHY_MT, DATA, value));
    EMAC_WRITEL(emac, phy_mt, frame);

    do {
        netstat = EMAC_READL(emac, status);
    } while (!(netstat & EMAC_BIT(STATUS, IDLE)));

    netctl = EMAC_READL(emac, cn);
    netctl &= ~EMAC_BIT(CN, MPEN);
    EMAC_WRITEL(emac, cn, netctl);
}

static uint16_t emac_mdio_read(unsigned char reg)
{
    unsigned int netctl;
    unsigned int netstat;
    unsigned int frame;

    netctl = EMAC_READL(emac, cn);
    netctl |= EMAC_BIT(CN, MPEN);
    EMAC_WRITEL(emac, cn, netctl);

    frame = (EMAC_BF(PHY_MT, SOP, 1) | EMAC_BF(PHY_MT, OP, 2) |
             EMAC_BF(PHY_MT, PHYADDR, emac->phy_addr) | EMAC_BF(PHY_MT, REGADDR, reg) |
             EMAC_BF(PHY_MT, CODE, 2));
    EMAC_WRITEL(emac, phy_mt, frame);

    do {
        netstat = EMAC_READL(emac, status);
    } while (!(netstat & EMAC_BIT(STATUS, IDLE)));

    frame = EMAC_READL(emac, phy_mt);

    netctl = EMAC_READL(emac, cn);
    netctl &= ~EMAC_BIT(CN, MPEN);
    EMAC_WRITEL(emac, cn, netctl);

    return EMAC_BFEXT(PHY_MT, DATA, frame);
}

static void emac_reclaim_rx_buffers(unsigned int new_tail)
{
    unsigned int i;

    i = emac->rx_tail;

    while (i > new_tail) {
        emac->rx_ring[i].addr &= ~RXADDR_USED;
        i++;

        if (emac->rx_ring_size < i) {
            i = 0;
        }
    }

    while (i < new_tail) {
        emac->rx_ring[i].addr &= ~RXADDR_USED;
        i++;
    }

    barrier();
    emac->rx_tail = new_tail;
}

static void emac_phy_reset(void)
{
    uint16_t status;
    int i;

    emac_mdio_write(MII_ADVERTISE, (ADVERTISE_CSMA | ADVERTISE_ALL));
    emac_mdio_write(MII_BMCR, (BMCR_ANENABLE | BMCR_ANRESTART));

    for (i = 0; i < (CONFIG_SYS_EMAC_AUTONEG_TIMEOUT / 100); i++) {
        status = emac_mdio_read(MII_BMSR);

        if (status & BMSR_ANEGCOMPLETE) {
            break;
        }

        emac->delay_us(100);
    }
}

#ifdef CONFIG_EMAC_SEARCH_PHY
static int emac_phy_find(void)
{
    int i;
    uint16_t phy_id;

    for (i = 0; i < 32; i++) {
        emac->phy_addr = i;

        phy_id = emac_mdio_read(MII_PHYSID1);

        if (0xffff != phy_id) {
            return E_NO_ERROR;
        }
    }

    return E_NO_DEVICE;
}
#endif //CONFIG_EMAC_SEARCH_PHY

static unsigned int emac_mii_nway_result(unsigned int negotiated)
{
    unsigned int ret;

    if (negotiated & LPA_100FULL) {
        ret = LPA_100FULL;
    } else if (negotiated & LPA_100BASE4) {
        ret = LPA_100BASE4;
    } else if (negotiated & LPA_100HALF) {
        ret = LPA_100HALF;
    } else if (negotiated & LPA_10FULL) {
        ret = LPA_10FULL;
    } else {
        ret = LPA_10HALF;
    }

    return ret;
}

static int emac_phy_init(void)
{
    int result = E_NO_ERROR;
    uint16_t phy_id;
    uint16_t status;
    uint16_t adv;
    uint16_t lpa;
    int media;
    int speed;
    int duplex;
    int i;
    unsigned int ncfgr;

#ifdef CONFIG_EMAC_SEARCH_PHY
    result = emac_phy_find();

    if (result) {
        return result;
    }

#endif //CONFIG_EMAC_SEARCH_PHY

    phy_id = emac_mdio_read(MII_PHYSID1);

    if (0xffff == phy_id) {
        return E_NO_DEVICE;
    }

    status = emac_mdio_read(MII_BMSR);

    if (!(status & BMSR_LSTATUS)) {
        emac_phy_reset();

        for (i = 0; i < (CONFIG_SYS_EMAC_AUTONEG_TIMEOUT / 100); i++) {
            status = emac_mdio_read(MII_BMSR);

            if (status & BMSR_LSTATUS) {
                break;
            }

            emac->delay_us(100);
        }
    }

    if (!(status & BMSR_LSTATUS)) {
        return E_NO_RESPONSE; //PHY Link Down
    } else {
        adv = emac_mdio_read(MII_ADVERTISE);
        lpa = emac_mdio_read(MII_LPA);

        media = emac_mii_nway_result(lpa & adv);
        speed = ((media & (ADVERTISE_100FULL | ADVERTISE_100HALF)) ? 1 : 0);
        duplex = (media & ADVERTISE_FULL) ? 1 : 0;

        ncfgr = EMAC_READL(emac, cfg);
        ncfgr &= ~(EMAC_BIT(CFG, SPD) | EMAC_BIT(CFG, FULLDPLX));

        if (speed) {
            ncfgr |= EMAC_BIT(CFG, SPD);
        }

        if (duplex) {
            ncfgr |= EMAC_BIT(CFG, FULLDPLX);
        }

        /* Discard FCS Field */
        ncfgr |= EMAC_BIT(CFG, DCRXFCS);

        EMAC_WRITEL(emac, cfg, ncfgr);
    }

    return result;
}

/* ************************************************************************* */
/* Control/Configuration Functions                                           */
/* ************************************************************************* */
int MXC_EMAC_RevA_Init(mxc_emac_reva_config_t *config)
{
    int result = E_UNKNOWN;
    unsigned int ncfgr, emac_pclk_rate, clk_div;

    if (!config) {
        return E_NULL_PTR;
    }

    if (!emac->first_init) {
        /* Assign interface base address */
        emac->regs = (mxc_emac_reva_regs_t *)MXC_EMAC;

        /* Clock configuration */
        emac_pclk_rate = PeripheralClock;

        if (emac_pclk_rate < 20000000) {
            clk_div = EMAC_CLK_DIV8;
        } else if (emac_pclk_rate < 40000000) {
            clk_div = EMAC_CLK_DIV16;
        } else if (emac_pclk_rate < 80000000) {
            clk_div = EMAC_CLK_DIV32;
        } else {
            clk_div = EMAC_CLK_DIV64;
        }

        ncfgr = EMAC_BF(CFG, MDCCLK, clk_div);
        EMAC_WRITEL(emac, cfg, ncfgr);

        /* Initialization to be finished */
        emac->first_init = 1;
    }

    /* Set configuration */
    result = MXC_EMAC_RevA_SetConfiguration(config);

    return result;
}

int MXC_EMAC_RevA_SetConfiguration(mxc_emac_reva_config_t *config)
{
    if (!emac->first_init) {
        return E_UNINITIALIZED;
    }

    if (!(config->rx_buff) || !(config->rx_ring_buff) || !(config->tx_ring_buff)) {
        return E_NULL_PTR;
    }

    if ((config->rx_ring_buff_size % sizeof(mxc_emac_dma_desc_t)) ||
        (config->tx_ring_buff_size % sizeof(mxc_emac_dma_desc_t)) ||
        (config->rx_buff_size % EMAC_RX_BUFFER_SIZE)) {
        return E_INVALID;
    }

    if (((config->rx_ring_buff_size / sizeof(mxc_emac_dma_desc_t)) > MAX_SYS_EMAC_RX_RING_SIZE) ||
        ((config->tx_ring_buff_size / sizeof(mxc_emac_dma_desc_t)) > MAX_SYS_EMAC_TX_RING_SIZE) ||
        (config->rx_buff_size > MAX_SYS_EMAC_RX_BUFFER_SIZE)) {
        return E_INVALID;
    }

    if (!config->delay_us) {
        return E_INVALID;
    }

    emac->rx_buffer = (void *)(config->rx_buff);
    emac->rx_buffer_dma = (unsigned int)(config->rx_buff);
    emac->rx_buffer_size = config->rx_buff_size;

    emac->rx_ring = (mxc_emac_reva_dma_desc_t *)(config->rx_ring_buff);
    emac->rx_ring_dma = (unsigned int)(config->rx_ring_buff);
    emac->rx_ring_size = (config->rx_ring_buff_size / sizeof(mxc_emac_dma_desc_t));

    emac->tx_ring = (mxc_emac_reva_dma_desc_t *)(config->tx_ring_buff);
    emac->tx_ring_dma = (unsigned int)(config->tx_ring_buff);
    emac->tx_ring_size = (config->tx_ring_buff_size / sizeof(mxc_emac_dma_desc_t));

    emac->phy_addr = config->phy_addr;
    emac->delay_us = config->delay_us;

    if (config->interrupt_mode) {
        memcpy((void *)&emac->cb_funcs, (const void *)&config->conf_cb_funcs,
               sizeof(mxc_emac_cb_funcs_tbl_t));
        MXC_EMAC_RevA_EnableInterruptEvents(config->interrupt_events);
    } else {
        memset((void *)&emac->cb_funcs, 0, sizeof(mxc_emac_cb_funcs_tbl_t));
        MXC_EMAC_RevA_DisableInterruptEvents(0xFFFFFFFF);
    }

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_SetHwAddr(unsigned char *enetaddr)
{
    uint16_t hwaddr_top;
    unsigned int hwaddr_bottom;

    if (!enetaddr) {
        return E_NULL_PTR;
    }

    /* Set Hardware Address */
    hwaddr_bottom =
        ((enetaddr[0]) | (enetaddr[1] << 8) | (enetaddr[2] << 16) | (enetaddr[3] << 24));
    EMAC_WRITEL(emac, sa1l, hwaddr_bottom);

    hwaddr_top = (enetaddr[4] | (enetaddr[5] << 8));
    EMAC_WRITEL(emac, sa1h, hwaddr_top);

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_EnableInterruptEvents(unsigned int events)
{
    unsigned int ier, imr;

    if (!emac->first_init) {
        return E_UNINITIALIZED;
    }

    /* First Read from Interrupt Mask Register */
    imr = EMAC_READL(emac, int_mask);

    /* IER is Write-Only */
    ier = ~imr | events;
    EMAC_WRITEL(emac, int_en, ier);

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_DisableInterruptEvents(unsigned int events)
{
    unsigned int idr, imr;

    if (!emac->first_init) {
        return E_UNINITIALIZED;
    }

    /* First Read from Interrupt Mask Register */
    imr = EMAC_READL(emac, int_mask);

    /* IDR is Write-Only */
    idr = imr | events;
    EMAC_WRITEL(emac, int_dis, idr);

    return E_NO_ERROR;
}

/* ************************************************************************* */
/* Low-Level Functions                                                       */
/* ************************************************************************* */
int MXC_EMAC_RevA_Start(void)
{
    int result = E_UNKNOWN;
    unsigned int i;
    unsigned int paddr;
    unsigned int ncr;

    if (!emac->first_init) {
        return E_UNINITIALIZED;
    }

    /* DMA Descriptors */
    paddr = emac->rx_buffer_dma;

    for (i = 0; i < emac->rx_ring_size; i++) {
        if ((emac->rx_ring_size - 1) == i) {
            paddr |= RXADDR_WRAP;
        }

        emac->rx_ring[i].addr = paddr;
        emac->rx_ring[i].ctrl = 0;

        paddr += EMAC_RX_BUFFER_SIZE;
    }

    for (i = 0; i < emac->tx_ring_size; i++) {
        emac->tx_ring[i].addr = 0;

        if ((emac->tx_ring_size - 1) == i) {
            emac->tx_ring[i].ctrl = TXBUF_USED | TXBUF_WRAP;
        } else {
            emac->tx_ring[i].ctrl = TXBUF_USED;
        }
    }

    emac->tx_tail = 0;
    emac->tx_head = 0;
    emac->rx_tail = 0;

    EMAC_WRITEL(emac, rxbuf_ptr, emac->rx_ring_dma);
    EMAC_WRITEL(emac, txbuf_ptr, emac->tx_ring_dma);

#ifdef CONFIG_EMAC_MII_MODE
    EMAC_WRITEL(emac, usrio, EMAC_BIT(USRIO, MII));
#endif

    result = emac_phy_init();

    if (E_NO_ERROR == result) {
        /* For Diagnostic */
#ifdef CONFIG_EMAC_LOCAL_LOOPBACK_MODE
        ncr = EMAC_READL(emac, cn);

        ncr &= ~EMAC_BIT(CN, LB);
        ncr |= EMAC_BIT(CN, LBL);

        EMAC_WRITEL(emac, cn, ncr);
#endif
        /* Enable TX and RX */
        ncr = EMAC_READL(emac, cn);
        ncr |= EMAC_BIT(CN, TXEN);
        ncr |= EMAC_BIT(CN, TXSTART);
        EMAC_WRITEL(emac, cn, ncr);

        ncr = EMAC_READL(emac, cn);
        ncr |= EMAC_BIT(CN, RXEN);
        EMAC_WRITEL(emac, cn, ncr);
    }

    return result;
}

int MXC_EMAC_RevA_Stop(void)
{
    unsigned int ncr;
    unsigned int tsr;

    /* Halt the Controller and Wait for Any Ongoing Transmission to End */
    ncr = EMAC_READL(emac, cn);
    ncr |= EMAC_BIT(CN, TXHALT);
    EMAC_WRITEL(emac, cn, ncr);

    do {
        tsr = EMAC_READL(emac, tx_st);
    } while (tsr & EMAC_BIT(TX_ST, TXGO));

    /* Clear Statistics */
    EMAC_WRITEL(emac, cn, EMAC_BIT(CN, CLST));

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_ReadLinkStatus(void)
{
    int result = E_UNKNOWN;
    uint16_t status;

    status = emac_mdio_read(MII_BMSR);

    if (status & BMSR_LSTATUS) {
        result = E_NO_ERROR;
    } else {
        result = E_NO_DEVICE; //PHY Link Down
    }

    return result;
}

/* ************************************************************************* */
/* Transaction-Level Functions                                               */
/* ************************************************************************* */
int MXC_EMAC_RevA_SendSync(const void *packet, unsigned int length)
{
    int i;
    unsigned int paddr;
    unsigned int ctrl;
    unsigned int tx_head;
    unsigned int ncr;

    if (!packet) {
        return E_NULL_PTR;
    }

    if (!emac->delay_us) {
        return E_UNINITIALIZED;
    }

    tx_head = emac->tx_head;
    paddr = (unsigned int)packet;

    ctrl = length & TXBUF_FRMLEN_MASK;
    ctrl |= TXBUF_FRAME_END;

    if (tx_head == (emac->tx_ring_size - 1)) {
        ctrl |= TXBUF_WRAP;
        emac->tx_head = 0;
    } else {
        emac->tx_head++;
    }

    emac->tx_ring[tx_head].ctrl = ctrl;
    emac->tx_ring[tx_head].addr = paddr;
    barrier();

    ncr = EMAC_READL(emac, cn);
    ncr |= EMAC_BIT(CN, TXEN);
    ncr |= EMAC_BIT(CN, TXSTART);
    ncr |= EMAC_BIT(CN, RXEN);
    EMAC_WRITEL(emac, cn, ncr);

    for (i = 0; i <= CONFIG_SYS_EMAC_TX_TIMEOUT; i++) {
        barrier();

        ctrl = emac->tx_ring[tx_head].ctrl;

        if (ctrl & TXBUF_USED) {
            break;
        }

        emac->delay_us(1);
    }

    if (i <= CONFIG_SYS_EMAC_TX_TIMEOUT) {
        if (ctrl & TXBUF_UNDERRUN) {
            return E_UNDERFLOW;
        }

        if (ctrl & TXBUF_EXHAUSTED) {
            return E_OVERFLOW;
        }
    } else {
        return E_TIME_OUT;
    }

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_SendAsync(const void *packet, unsigned int length)
{
    unsigned int paddr;
    unsigned int ctrl;
    unsigned int tx_head;
    unsigned int ncr;

    if (!packet) {
        return E_NULL_PTR;
    }

    tx_head = emac->tx_head;
    paddr = (unsigned int)packet;

    ctrl = length & TXBUF_FRMLEN_MASK;
    ctrl |= TXBUF_FRAME_END;

    if (tx_head == (emac->tx_ring_size - 1)) {
        ctrl |= TXBUF_WRAP;
        emac->tx_head = 0;
    } else {
        emac->tx_head++;
    }

    emac->tx_ring[tx_head].ctrl = ctrl;
    emac->tx_ring[tx_head].addr = paddr;
    barrier();

    ncr = EMAC_READL(emac, cn);
    ncr |= EMAC_BIT(CN, TXEN);
    ncr |= EMAC_BIT(CN, TXSTART);
    ncr |= EMAC_BIT(CN, RXEN);
    EMAC_WRITEL(emac, cn, ncr);

    barrier();

    return E_NO_ERROR;
}

int MXC_EMAC_RevA_Recv(void *rx_buff, unsigned int max_len)
{
    int result = E_UNKNOWN;
    int wrapped = 0;
    unsigned int length;
    unsigned int status;
    unsigned int rx_tail;
    unsigned int headlen;
    unsigned int taillen;
    unsigned char packet_received = 0;
    unsigned char *tail_buff_ptr;
    void *buffer;

    if (!emac->first_init) {
        return E_UNINITIALIZED;
    }

    if (!rx_buff) {
        return E_NULL_PTR;
    }

    rx_tail = emac->rx_tail;

    while (1) {
        if (!(emac->rx_ring[rx_tail].addr & RXADDR_USED)) {
            /* No RX Frame */
            return 0;
        }

        status = emac->rx_ring[rx_tail].ctrl;

        if (status & RXBUF_FRAME_START) {
            if (rx_tail != emac->rx_tail) {
                emac_reclaim_rx_buffers(rx_tail);
            }

            wrapped = 0;
        }

        if (status & RXBUF_FRAME_END) {
            packet_received = 1;

            buffer = emac->rx_buffer + (EMAC_RX_BUFFER_SIZE * emac->rx_tail);
            length = status & RXBUF_FRMLEN_MASK;

            if (wrapped) {
                headlen = EMAC_RX_BUFFER_SIZE * (emac->rx_ring_size - emac->rx_tail);
                taillen = length - headlen;
                tail_buff_ptr = (unsigned char *)rx_buff + headlen;

                if ((headlen + taillen) <= max_len) {
                    memcpy(rx_buff, (const void *)buffer, headlen);
                    memcpy((void *)tail_buff_ptr, (const void *)emac->rx_buffer, taillen);
                    result = headlen + taillen;
                } else {
                    result = E_NONE_AVAIL; //RX User Buffer Full
                }
            } else if (length <= max_len) {
                memcpy(rx_buff, (const void *)buffer, length);
                result = length;
            } else {
                result = E_NONE_AVAIL; //RX User Buffer Full
            }

            if (++rx_tail >= emac->rx_ring_size) {
                rx_tail = 0;
            }

            emac_reclaim_rx_buffers(rx_tail);
        } else {
            if (++rx_tail >= emac->rx_ring_size) {
                wrapped = 1;
                rx_tail = 0;
            }
        }

        barrier();

        if (packet_received) {
            return result;
        }
    }

    return result;
}

void MXC_EMAC_RevA_IrqHandler(void)
{
    unsigned int isr = 0;

    isr = EMAC_READL(emac, int_st);

    if ((isr & MXC_EMAC_REVA_EVENT_MPS) && emac->cb_funcs.mps_handler) {
        emac->cb_funcs.mps_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_RXCMPL) && emac->cb_funcs.rxcmpl_handler) {
        emac->cb_funcs.rxcmpl_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_RXUBR) && emac->cb_funcs.rxubr_handler) {
        emac->cb_funcs.rxubr_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_TXUBR) && emac->cb_funcs.txubr_handler) {
        emac->cb_funcs.txubr_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_TXUR) && emac->cb_funcs.txur_handler) {
        emac->cb_funcs.txur_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_RLE) && emac->cb_funcs.rle_handler) {
        emac->cb_funcs.rle_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_TXERR) && emac->cb_funcs.txerr_handler) {
        emac->cb_funcs.txerr_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_TXCMPL) && emac->cb_funcs.txcmpl_handler) {
        emac->cb_funcs.txcmpl_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_LC) && emac->cb_funcs.lc_handler) {
        emac->cb_funcs.lc_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_RXOR) && emac->cb_funcs.rxor_handler) {
        emac->cb_funcs.rxor_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_HRESPNO) && emac->cb_funcs.hrespno_handler) {
        emac->cb_funcs.hrespno_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_PPR) && emac->cb_funcs.ppr_handler) {
        emac->cb_funcs.ppr_handler();
    }

    if ((isr & MXC_EMAC_REVA_EVENT_PTZ) && emac->cb_funcs.ptz_handler) {
        emac->cb_funcs.ptz_handler();
    }
}
