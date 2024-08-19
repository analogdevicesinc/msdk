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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_EMAC_EMAC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_EMAC_EMAC_REVA_H_

/* **** Includes **** */
#include <stddef.h>
#include "emac.h"
#include "emac_reva_regs.h"

/* **** Definitions **** */
#define MAX_SYS_EMAC_RX_BUFFER_SIZE 16384
#define MAX_SYS_EMAC_RX_RING_SIZE 128
#define MAX_SYS_EMAC_TX_RING_SIZE 16
#define EMAC_RX_BUFFER_SIZE 128
#define CONFIG_SYS_EMAC_TX_TIMEOUT 1000 /**! Transmission Timeout in Microseconds     */
#define CONFIG_SYS_EMAC_AUTONEG_TIMEOUT 500000 /**! Auto Negotiation Timeout in Microseconds */

#define RXADDR_USED 0x00000001
#define RXADDR_WRAP 0x00000002
#define RXBUF_FRMLEN_MASK 0x00000fff
#define RXBUF_FRAME_START 0x00004000
#define RXBUF_FRAME_END 0x00008000
#define RXBUF_TYPEID_MATCH 0x00400000
#define RXBUF_ADDR4_MATCH 0x00800000
#define RXBUF_ADDR3_MATCH 0x01000000
#define RXBUF_ADDR2_MATCH 0x02000000
#define RXBUF_ADDR1_MATCH 0x04000000
#define RXBUF_BROADCAST 0x80000000
#define TXBUF_FRMLEN_MASK 0x000007ff
#define TXBUF_FRAME_END 0x00008000
#define TXBUF_NOCRC 0x00010000
#define TXBUF_EXHAUSTED 0x08000000
#define TXBUF_UNDERRUN 0x10000000
#define TXBUF_MAXRETRY 0x20000000
#define TXBUF_WRAP 0x40000000
#define TXBUF_USED 0x80000000

/* Definitions for MII-Compatible Transceivers */
#define MII_BMCR 0x00 /**! Basic Mode Control Register    */
#define MII_BMSR 0x01 /**! Basic Mode Status Register     */
#define MII_PHYSID1 0x02 /**! PHYS ID 1                      */
#define MII_PHYSID2 0x03 /**! PHYS ID 2                      */
#define MII_ADVERTISE 0x04 /**! Advertisement Control Register */
#define MII_LPA 0x05 /**! Link Partner Ability Register  */

/* Basic Mode Control Register */
#define BMCR_ANRESTART 0x0200 /**! Auto Negotiation Restart       */
#define BMCR_ANENABLE 0x1000 /**! Enable Auto Negotiation        */

/* Basic Mode Status Register */
#define BMSR_LSTATUS 0x0004 /**! Link Status                    */
#define BMSR_ANEGCOMPLETE 0x0020 /**! Auto Negotiation Complete      */

/* Advertisement Control Register */
#define ADVERTISE_CSMA 0x0001
#define ADVERTISE_10HALF 0x0020
#define ADVERTISE_10FULL 0x0040
#define ADVERTISE_100HALF 0x0080
#define ADVERTISE_100FULL 0x0100
#define ADVERTISE_FULL (ADVERTISE_100FULL | ADVERTISE_10FULL | ADVERTISE_CSMA)
#define ADVERTISE_ALL (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL)

/* Link Partner Ability Register */
#define LPA_10HALF 0x0020
#define LPA_10FULL 0x0040
#define LPA_100HALF 0x0080
#define LPA_100FULL 0x0100
#define LPA_100BASE4 0x0200

/* Constants for CLK */
#define EMAC_CLK_DIV8 0
#define EMAC_CLK_DIV16 1
#define EMAC_CLK_DIV32 2
#define EMAC_CLK_DIV64 3

/* **** Macros **** */
/* Bit Manipulation */
#define EMAC_BIT(reg, name) (1 << MXC_F_EMAC_REVA_##reg##_##name##_POS)
#define EMAC_BF(reg, name, value)                                                         \
    (((value) & (MXC_F_EMAC_REVA_##reg##_##name >> MXC_F_EMAC_REVA_##reg##_##name##_POS)) \
     << MXC_F_EMAC_REVA_##reg##_##name##_POS)
#define EMAC_BFEXT(reg, name, value)                     \
    (((value) >> MXC_F_EMAC_REVA_##reg##_##name##_POS) & \
     (MXC_F_EMAC_REVA_##reg##_##name >> MXC_F_EMAC_REVA_##reg##_##name##_POS))

/* Register Access */
#define REG_READL(a) (*(volatile uint32_t *)(a))
#define REG_WRITEL(v, a) (*(volatile uint32_t *)(a) = (v))
#define EMAC_READL(port, reg) REG_READL(&(port)->regs->reg)
#define EMAC_WRITEL(port, reg, value) REG_WRITEL((value), &(port)->regs->reg)

/* Misc */
#define barrier() asm volatile("" ::: "memory")

/** @brief   Enumeration for the EMAC interrupt events */
typedef enum {
    MXC_EMAC_REVA_EVENT_MPS =
        MXC_F_EMAC_REVA_INT_EN_MPS, /**! Management Packet Sent Interrupt                   */
    MXC_EMAC_REVA_EVENT_RXCMPL =
        MXC_F_EMAC_REVA_INT_EN_RXCMPL, /**! Receive Complete Interrupt                         */
    MXC_EMAC_REVA_EVENT_RXUBR =
        MXC_F_EMAC_REVA_INT_EN_RXUBR, /**! RX Used Bit Read Interrupt                         */
    MXC_EMAC_REVA_EVENT_TXUBR =
        MXC_F_EMAC_REVA_INT_EN_TXUBR, /**! TX Used Bit Read Interrupt                         */
    MXC_EMAC_REVA_EVENT_TXUR =
        MXC_F_EMAC_REVA_INT_EN_TXUR, /**! Ethernet Transmit Underrun Interrupt               */
    MXC_EMAC_REVA_EVENT_RLE =
        MXC_F_EMAC_REVA_INT_EN_RLE, /**! Retry Limit Exceeded Interrupt                     */
    MXC_EMAC_REVA_EVENT_TXERR =
        MXC_F_EMAC_REVA_INT_EN_TXERR, /**! Transmit Buffers Exhausted In Mid-Frame Interrupt  */
    MXC_EMAC_REVA_EVENT_TXCMPL =
        MXC_F_EMAC_REVA_INT_EN_TXCMPL, /**! Transmit Complete Interrupt                        */
    MXC_EMAC_REVA_EVENT_LC =
        MXC_F_EMAC_REVA_INT_EN_LC, /**! Link Change Interrupt                              */
    MXC_EMAC_REVA_EVENT_RXOR =
        MXC_F_EMAC_REVA_INT_EN_RXOR, /**! Receive Overrun Interrupt                          */
    MXC_EMAC_REVA_EVENT_HRESPNO =
        MXC_F_EMAC_REVA_INT_EN_HRESPNO, /**! HRESP Not OK Interrupt                             */
    MXC_EMAC_REVA_EVENT_PPR =
        MXC_F_EMAC_REVA_INT_EN_PPR, /**! Pause Packet Received Interrupt                    */
    MXC_EMAC_REVA_EVENT_PTZ =
        MXC_F_EMAC_REVA_INT_EN_PTZ /**! Pause Time Zero Interrupt                          */
} mxc_emac_reva_events_t;

/* **** Structures **** */

/**
 * @brief   The information needed for an EMAC buffer descriptor
 *
 */
typedef struct {
    unsigned int addr;
    unsigned int ctrl;
} mxc_emac_reva_dma_desc_t;

/**
 * @brief   The information needed by the EMAC driver to operate
 *
 */
typedef struct {
    mxc_emac_reva_regs_t *regs;
    unsigned int rx_tail;
    unsigned int tx_head;
    unsigned int tx_tail;
    void *rx_buffer;
    void *tx_buffer;
    mxc_emac_reva_dma_desc_t *rx_ring;
    mxc_emac_reva_dma_desc_t *tx_ring;
    unsigned int rx_buffer_dma;
    unsigned int rx_ring_dma;
    unsigned int tx_ring_dma;
    uint16_t phy_addr;

    unsigned int first_init;
    unsigned int rx_buffer_size;
    unsigned int rx_ring_size;
    unsigned int tx_ring_size;
    mxc_emac_delay_func_t delay_us;
    mxc_emac_cb_funcs_tbl_t cb_funcs;
} mxc_emac_reva_device_t;

/**
 * @brief   The basic configuration information to set up EMAC module
 *
 */
typedef struct {
    unsigned char *rx_buff;
    unsigned char *rx_ring_buff;
    unsigned char *tx_ring_buff;
    unsigned int rx_buff_size;
    unsigned int rx_ring_buff_size;
    unsigned int tx_ring_buff_size;
    uint16_t phy_addr;
    unsigned int interrupt_mode;
    unsigned int interrupt_events;
    mxc_emac_delay_func_t delay_us;
    mxc_emac_cb_funcs_tbl_t conf_cb_funcs;
} mxc_emac_reva_config_t;

/* **** Function Prototypes **** */
/* ************************************************************************* */
/* Control/Configuration Functions                                           */
/* ************************************************************************* */
int MXC_EMAC_RevA_Init(mxc_emac_reva_config_t *config);
int MXC_EMAC_RevA_SetConfiguration(mxc_emac_reva_config_t *config);
int MXC_EMAC_RevA_SetHwAddr(unsigned char *enetaddr);
int MXC_EMAC_RevA_EnableInterruptEvents(unsigned int events);
int MXC_EMAC_RevA_DisableInterruptEvents(unsigned int events);

/* ************************************************************************* */
/* Low-Level Functions                                                       */
/* ************************************************************************* */
int MXC_EMAC_RevA_Start(void);
int MXC_EMAC_RevA_Stop(void);
int MXC_EMAC_RevA_ReadLinkStatus(void);

/* ************************************************************************* */
/* Transaction-Level Functions                                               */
/* ************************************************************************* */
int MXC_EMAC_RevA_SendSync(const void *packet, unsigned int length);
int MXC_EMAC_RevA_SendAsync(const void *packet, unsigned int length);
int MXC_EMAC_RevA_Recv(void *rx_buff, unsigned int max_len);
void MXC_EMAC_RevA_IrqHandler(void);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_EMAC_EMAC_REVA_H_
