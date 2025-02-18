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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "usbhs_regs.h"
#include "usb.h"

#ifdef MAX32690
#include "fcr_regs.h"
static bool g_is_clock_locked = false;
#endif

#define USBHS_M31_CLOCK_RECOVERY

typedef enum {
    SETUP_IDLE,
    SETUP_NODATA,
    SETUP_DATA_OUT,
    SETUP_DATA_IN
} setup_phase_t;

/* storage for active endpoint data request objects */
static MXC_USB_Req_t *MXC_USB_Request[MXC_USBHS_NUM_EP];
/* endpoint sizes */
static unsigned int ep_size[MXC_USBHS_NUM_EP];
/* MUSBHSFC does not track SETUP in hardware, so instantiate state variable */
static setup_phase_t setup_phase = SETUP_IDLE;
/* Driver options passed in during MXC_USB_Init() */
static maxusb_cfg_options_t driver_opts;

static volatile uint8_t *get_fifo_ptr(unsigned int ep)
{
    volatile uint32_t *ptr;

    ptr = &MXC_USBHS->fifo0;
    ptr += ep; /* Pointer math: multiplies ep by sizeof(uint32_t) */

    return (volatile uint8_t *)ptr;
}

static void load_fifo(volatile uint8_t *fifoptr, uint8_t *dataptr, unsigned int len)
{
    volatile uint32_t *fifoptr_32 = (uint32_t *)fifoptr;
    uint32_t *dataptr_32 = (uint32_t *)dataptr;
    unsigned int len_32;

    /* Calculate sizes to efficiently move data */
    len_32 = len >> 2;
    len &= 0x3;

    /* Load word-sized chunks */
    while (len_32--) {
        *fifoptr_32 = *dataptr_32++;
    }
    dataptr = (uint8_t *)dataptr_32;
    /* Load remainder as bytes */
    while (len--) {
        *fifoptr = *dataptr++;
    }
}

static void unload_fifo(uint8_t *dataptr, volatile uint8_t *fifoptr, unsigned int len)
{
    volatile uint32_t *fifoptr_32 = (uint32_t *)fifoptr;
    uint32_t *dataptr_32 = (uint32_t *)dataptr;
    unsigned int len_32;

    /* Calculate sizes to efficiently move data */
    len_32 = len >> 2;
    len &= 0x3;

    while (len_32--) {
        *dataptr_32++ = *fifoptr_32;
    }
    dataptr = (uint8_t *)dataptr_32;
    while (len--) {
        *dataptr++ = *fifoptr;
    }
}

int MXC_USB_Init(maxusb_cfg_options_t *options)
{
    int i;

    /* Save the init options */
    if (options) {
        memcpy(&driver_opts, options, sizeof(maxusb_cfg_options_t));
    } else {
        driver_opts.enable_hs = 0;
    }

    /* Enable peripheral at the chip level */
    if (driver_opts.init_callback) {
        if (driver_opts.init_callback() != E_NO_ERROR) {
            return -1;
        }
    }

    /* Endpoint 0 is CONTROL, size fixed in hardware. Does not need configuration. */
    ep_size[0] = 64;

    /* Reset all other endpoints */
    for (i = 1; i < MXC_USBHS_NUM_EP; i++) {
        MXC_USB_ResetEp(i);
    }

    setup_phase = SETUP_IDLE;

    /* Start out disconnected */
    MXC_USBHS->power = 0;

    /* Disable all interrupts */
    MXC_USBHS->intrinen = 0;
    MXC_USBHS->introuten = 0;
    MXC_USBHS->intrusben = 0;

    /* Unsuspend the MAC */
    MXC_USBHS->mxm_suspend = 0;

    /* Configure PHY */
#if 0
    MXC_USBHS->xcfgi0 = (0x1 << 3) | (0x1 << 11);
    MXC_USBHS->xcfgi1 = 0;
    MXC_USBHS->xcfgi2 = 0x1 << (72-64);
    MXC_USBHS->xcfgi3 = 0;
#endif
    MXC_USBHS->m31_phy_xcfgi_31_0 = (0x1 << 3) | (0x1 << 11);
    MXC_USBHS->m31_phy_xcfgi_63_32 = 0;
    MXC_USBHS->m31_phy_xcfgi_95_64 = 0x1 << (72-64);
    MXC_USBHS->m31_phy_xcfgi_127_96 = 0;


#ifdef USBHS_M31_CLOCK_RECOVERY
    MXC_USBHS->m31_phy_noncry_rstb = 1;
    MXC_USBHS->m31_phy_noncry_en = 1;
    MXC_USBHS->m31_phy_outclksel = 0;
    MXC_USBHS->m31_phy_coreclkin = 0;
    MXC_USBHS->m31_phy_xtlsel = 2; /* Select 25 MHz clock */
#else
    /* Use this option to feed the PHY a 30 MHz clock, which is them used as a PLL reference */
    /* As it depends on the system core clock, this should probably be done at the SYS level */
    MXC_USBHS->m31_phy_noncry_rstb = 0;
    MXC_USBHS->m31_phy_noncry_en = 0;
    MXC_USBHS->m31_phy_outclksel = 1;
    MXC_USBHS->m31_phy_coreclkin = 1;
    MXC_USBHS->m31_phy_xtlsel = 3; /* Select 30 MHz clock */
#endif
    MXC_USBHS->m31_phy_pll_en = 1;
    MXC_USBHS->m31_phy_oscouten = 1;

    /* Reset PHY */
    MXC_USBHS->m31_phy_ponrst = 0;
    MXC_USBHS->m31_phy_ponrst = 1;

    return 0;
}

#ifdef MAX32690

int MXC_USB_LockClockSource(bool lock)
{
    g_is_clock_locked = lock;
    return E_NO_ERROR;
}

int MXC_USB_SetClockSource(mxc_usb_clock_t clock_source)
{
    if (g_is_clock_locked) {
        return E_BAD_STATE; // Clock source must be unlocked to set it.
    }

    // The USB peripheral's clock source is set in the FCR register bank.
    // The actual clock source selected by each field value may vary between
    // microcontrollers, so it is the responsibility of the implementer to define
    // mxc_usb_clock_t correctly in the top-level "max32xxx.h" file.  The enum values
    // should match the field values when type-casted to an unsigned int.
    if ((unsigned int)clock_source < 0 || (unsigned int)clock_source >= 3) {
        return E_BAD_PARAM;
    }

    mxc_sys_system_clock_t current_sys_clk = (mxc_sys_system_clock_t)(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL);
    if (current_sys_clk != MXC_SYS_CLOCK_IPO && clock_source == MXC_USB_CLOCK_SYS_DIV_10) {
        return E_BAD_STATE; // System clock must be set to the IPO for the USB PHY to use the internal clock
    }

    MXC_SETFIELD(MXC_FCR->fctrl0, MXC_F_FCR_FCTRL0_USBCLKSEL, ((unsigned int)clock_source) << MXC_F_FCR_FCTRL0_USBCLKSEL_POS);

    return E_NO_ERROR;
}

#endif

int MXC_USB_Shutdown(void)
{
    /* Disconnect and disable HS, too. */
    MXC_USBHS->power = 0;

    /* Disable all interrupts */
    MXC_USBHS->intrinen = 0;
    MXC_USBHS->introuten = 0;
    MXC_USBHS->intrusben = 0;

    /* Suspend the MAC */
    MXC_USBHS->mxm_suspend = 3;

    /* Shut down PHY */
    MXC_USBHS->m31_phy_ponrst = 0;

    /* Disable peripheral at chip level -- ignore failures */
    if (driver_opts.shutdown_callback) {
        driver_opts.shutdown_callback();
    }

    return 0;
}

int MXC_USB_Connect(void)
{
    /* Should high-speed negotiation be attempted? */
    if (driver_opts.enable_hs) {
        MXC_USBHS->power |= MXC_F_USBHS_POWER_HS_ENABLE;
    } else {
        MXC_USBHS->power &= ~MXC_F_USBHS_POWER_HS_ENABLE;
    }

    /* Connect to bus, if present */
    MXC_USBHS->power |= MXC_F_USBHS_POWER_SOFTCONN;

    setup_phase = SETUP_IDLE;

    return 0;
}

int MXC_USB_Disconnect(void)
{
    /* Disconnect from bus */
    MXC_USBHS->power &= ~MXC_F_USBHS_POWER_SOFTCONN;

    setup_phase = SETUP_IDLE;

    return 0;
}

unsigned int MXC_USB_GetStatus(void)
{
    int status = 0;

    /* VBUS */
    if (MXC_USBHS->mxm_reg_a4 & MXC_F_USBHS_MXM_REG_A4_VRST_VDDB_N_A) {
        status |= MAXUSB_STATUS_VBUS_ON;
    }

    /* High-speed state */
    if (MXC_USBHS->power & MXC_F_USBHS_POWER_HS_MODE) {
        status |= MAXUSB_STATUS_HIGH_SPEED;
    }

    return status;
}

int MXC_USB_ConfigEp(unsigned int ep, maxusb_ep_type_t type, unsigned int size)
{
    if (!ep || (ep >= MXC_USBHS_NUM_EP) || (size > MXC_USBHS_MAX_PACKET)) {
        /* Can't configure this endpoint, invalid endpoint, or size too big */
        return -1;
    }

    /* Default to disabled */
    MXC_USB_ResetEp(ep);

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    /* Select register index for endpoint */
    MXC_USBHS->index = ep;

    switch (type) {
        case MAXUSB_EP_TYPE_DISABLED:
            break;
        case MAXUSB_EP_TYPE_OUT:
            MXC_USBHS->outcsrl = MXC_F_USBHS_OUTCSRL_CLRDATATOG;
            MXC_USBHS->outcsru = MXC_F_USBHS_OUTCSRU_DPKTBUFDIS;
            MXC_USBHS->outmaxp = size;
            ep_size[ep] = size;
            MXC_USBHS->introuten &= ~(1 << ep);
            break;
        case MAXUSB_EP_TYPE_IN:
            MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_CLRDATATOG;
            MXC_USBHS->incsru = MXC_F_USBHS_INCSRU_DPKTBUFDIS | MXC_F_USBHS_INCSRU_MODE;
            MXC_USBHS->inmaxp = size;
            ep_size[ep] = size;
            MXC_USBHS->intrinen |= (1 << ep);
            break;
        default:
            MXC_SYS_Crit_Exit();
            return -1;
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

int MXC_USB_IsConfigured(unsigned int ep)
{
    return !!(ep_size[ep]);
}

int MXC_USB_Stall(unsigned int ep)
{
    MXC_USB_Req_t *req;

    if (!MXC_USB_IsConfigured(ep)) {
        /* Can't stall an unconfigured endpoint */
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    MXC_USBHS->index = ep;

    if (ep == 0) {
        MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_SERV_OUTPKTRDY | MXC_F_USBHS_CSR0_SEND_STALL;
        setup_phase = SETUP_IDLE;
    } else {
        if (MXC_USBHS->incsru & MXC_F_USBHS_INCSRU_MODE) {
            /* IN endpoint */
            MXC_USBHS->incsrl |= MXC_F_USBHS_INCSRL_SENDSTALL;
        } else {
            /* Otherwise, must be OUT endpoint */
            MXC_USBHS->outcsrl |= MXC_F_USBHS_OUTCSRL_SENDSTALL;
        }
    }

    /* clear pending requests */
    req = MXC_USB_Request[ep];
    MXC_USB_Request[ep] = NULL;

    if (req) {
        /* complete pending requests with error */
        req->error_code = -1;
        if (req->callback) {
            req->callback(req->cbdata);
        }
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

int MXC_USB_Unstall(unsigned int ep)
{
    if (!MXC_USB_IsConfigured(ep)) {
        /* Can't unstall an unconfigured endpoint */
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    MXC_USBHS->index = ep;

    if (ep != 0) {
        if (MXC_USBHS->incsru & MXC_F_USBHS_INCSRU_MODE) {
            /* IN endpoint */
            if (MXC_USBHS->incsrl & MXC_F_USBHS_INCSRL_INPKTRDY) {
                /* Per musbhsfc_pg, only flush FIFO if IN packet loaded */
                MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_CLRDATATOG | MXC_F_USBHS_INCSRL_FLUSHFIFO;
            } else {
                MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_CLRDATATOG;
            }
        } else {
            /* Otherwise, must be OUT endpoint */
            if (MXC_USBHS->outcsrl & MXC_F_USBHS_OUTCSRL_OUTPKTRDY) {
                /* Per musbhsfc_pg, only flush FIFO if OUT packet is ready */
                MXC_USBHS->outcsrl = MXC_F_USBHS_OUTCSRL_CLRDATATOG | MXC_F_USBHS_OUTCSRL_FLUSHFIFO;
            } else {
                MXC_USBHS->outcsrl = MXC_F_USBHS_OUTCSRL_CLRDATATOG;
            }
        }
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

int MXC_USB_IsStalled(unsigned int ep)
{
    unsigned int stalled;

    MXC_USBHS->index = ep;

    if (ep) {
        if (MXC_USBHS->incsru & MXC_F_USBHS_INCSRU_MODE) {
            /* IN endpoint */
            stalled = !!(MXC_USBHS->incsrl & MXC_F_USBHS_INCSRL_SENDSTALL);
        } else {
            /* Otherwise, must be OUT endpoint */
            stalled = !!(MXC_USBHS->outcsrl & MXC_F_USBHS_OUTCSRL_SENDSTALL);
        }
    } else {
        /* Control (EP 0) */
        stalled = !!(MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_SEND_STALL);
    }

    return stalled;
}

int MXC_USB_ResetEp(unsigned int ep)
{
    MXC_USB_Req_t *req;

    if (ep >= MXC_USBHS_NUM_EP) {
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    /* clear pending requests */
    req = MXC_USB_Request[ep];
    MXC_USB_Request[ep] = NULL;

    if (ep) {
        ep_size[ep] = 0;

        /* Select register index for endpoint */
        MXC_USBHS->index = ep;

        /* Default to disabled */
        MXC_USBHS->intrinen &= ~(1 << ep);
        MXC_USBHS->introuten &= ~(1 << ep);

        if (MXC_USBHS->incsrl & MXC_F_USBHS_INCSRL_INPKTRDY) {
            /* Per musbhsfc_pg, only flush FIFO if IN packet loaded */
            MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_FLUSHFIFO;
        }
        MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_SENDSTALL;

        MXC_USBHS->incsru = MXC_F_USBHS_INCSRU_DPKTBUFDIS;
        MXC_USBHS->inmaxp = 0;

        if (MXC_USBHS->outcsrl & MXC_F_USBHS_OUTCSRL_OUTPKTRDY) {
            /* Per musbhsfc_pg, only flush FIFO if OUT packet is ready */
            MXC_USBHS->outcsrl = MXC_F_USBHS_OUTCSRL_FLUSHFIFO;
        }
        MXC_USBHS->outcsrl = MXC_F_USBHS_OUTCSRL_SENDSTALL;

        MXC_USBHS->outcsru = MXC_F_USBHS_OUTCSRU_DPKTBUFDIS;
        MXC_USBHS->outmaxp = 0;

        MXC_SYS_Crit_Exit();

        /* We specifically do not complete SETUP callbacks, as this causes undesired SETUP status-stage STALLs */
        if (req) {
            /* complete pending requests with error */
            req->error_code = -1;
            if (req->callback) {
                req->callback(req->cbdata);
            }
        }
    } else {
        MXC_SYS_Crit_Exit();
    }

    return 0;
}

int MXC_USB_Ackstat(unsigned int ep)
{
    uint32_t saved_index;

    if (ep) {
        /* Only valid for endpoint 0 */
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    saved_index = MXC_USBHS->index;
    MXC_USBHS->index = 0;

    /* On this hardware, only setup transactions with no data stage need to be explicitly ACKed */
    if (setup_phase == SETUP_NODATA) {
        MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_SERV_OUTPKTRDY | MXC_F_USBHS_CSR0_DATA_END;
    }

    setup_phase = SETUP_IDLE;

    MXC_USBHS->index = saved_index;

    MXC_SYS_Crit_Exit();

    return 0;
}

void MXC_USB_DmaIsr(void)
{
  /* Not implemented */
}

/* sent packet done handler*/
static void event_in_data(uint32_t irqs)
{
    uint32_t ep, buffer_bit, data_left;
    MXC_USB_Req_t *req;
    unsigned int len;

    /* Loop for each data endpoint */
    for (ep = 0; ep < MXC_USBHS_NUM_EP; ep++) {
        buffer_bit = (1 << ep);
        if ((irqs & buffer_bit) == 0) { /* Not set, next Endpoint */
            continue;
        }

        /* If an interrupt was received, but no request on this EP, ignore. */
        if (!MXC_USB_Request[ep]) {
            continue;
        }

        /* This function is called within interrupt context, so no need for a critical section */

        MXC_USBHS->index = ep;

        req = MXC_USB_Request[ep];
        data_left = req->reqlen - req->actlen;

        /* Check for more data left to transmit */
        if (data_left) {
            if (data_left >= ep_size[ep]) {
                len = ep_size[ep];
            } else {
                len = data_left;
            }

            /* Fill FIFO with data */
            load_fifo(get_fifo_ptr(ep), (req->data + req->actlen), len);
            req->actlen += len;

            if (!ep) {
                if (MXC_USB_Request[ep]->actlen == MXC_USB_Request[ep]->reqlen) {
#ifdef USE_ZEPHYR_USB_STACK
                    /* Send ZLP */
                    if (MXC_USB_Request[ep]->has_zlp) {
                        MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY;
                        continue;
                    }
#endif
                    /* Implicit status-stage ACK, move state machine back to IDLE */
                    setup_phase = SETUP_IDLE;
                    MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY | MXC_F_USBHS_CSR0_DATA_END;

                    /* free request */
                    MXC_USB_Request[ep] = NULL;

                    /* set done return value */
                    if (req->callback) {
                        req->callback(req->cbdata);
                    }
                } else {
                    MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY;
                }
            } else {
                /* Arm for transmit to host */
                MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_INPKTRDY;
            }

        } else {
            /* all done sending data */
#ifdef USE_ZEPHYR_USB_STACK
            if (MXC_USB_Request[ep]->has_zlp) {
                MXC_USB_Request[ep]->has_zlp = false;

                if (!ep) { /* For EP0, ZLP is sent, complete transmission */
                    setup_phase = SETUP_IDLE;
                    MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY | MXC_F_USBHS_CSR0_DATA_END;
                } else { /* Send ZLP */
                    MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_INPKTRDY;
                    return;
                }
            }
#endif

            /* free request */
            MXC_USB_Request[ep] = NULL;

            /* set done return value */
            if (req->callback) {
                req->callback(req->cbdata);
            }
        }
    }
}

/* received packet */
static void event_out_data(uint32_t irqs)
{
    uint32_t ep, buffer_bit, reqsize;
    MXC_USB_Req_t *req;

    /* Loop for each data endpoint */
    for (ep = 0; ep < MXC_USBHS_NUM_EP; ep++) {
        buffer_bit = (1 << ep);
        if ((irqs & buffer_bit) == 0) {
            continue;
        }
        /* If an interrupt was received, but no request on this EP, ignore. */
        if (!MXC_USB_Request[ep]) {
            continue;
        }

        req = MXC_USB_Request[ep];

        /* This function is called within interrupt context, so no need for a critical section */

        /* Select this endpoint for banked registers */
        MXC_USBHS->index = ep;

        if (!ep) {
            if (!(MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_OUTPKTRDY)) {
                continue;
            }
            if (MXC_USBHS->count0 == 0) {
                /* ZLP */
                MXC_USB_Request[ep] = NULL;
                /* Let the callback do the status stage */
                /* call it done */
                if (req->callback) {
                    req->callback(req->cbdata);
                }
                continue;
            } else {
                /* Write as much as we can to the request buffer */
                reqsize = MXC_USBHS->count0;
                if (reqsize > (req->reqlen - req->actlen)) {
                    reqsize = (req->reqlen - req->actlen);
                }
            }
        } else {
            if (!(MXC_USBHS->outcsrl & MXC_F_USBHS_OUTCSRL_OUTPKTRDY)) {
                /* No packet on this endpoint? */
                continue;
            }
            if (MXC_USBHS->outcount == 0) {
                /* ZLP */
                /* Clear out request */
                MXC_USB_Request[ep] = NULL;

                /* Signal to H/W that FIFO has been read */
                MXC_USBHS->outcsrl &= ~MXC_F_USBHS_OUTCSRL_OUTPKTRDY;

                /* Disable interrupt for this endpoint */
                MXC_USBHS->introuten &= ~(1 << ep);

                /* Complete request */
                if (req->callback) {
                    req->callback(req->cbdata);
                }
                continue;

            } else {
                /* Write as much as we can to the request buffer */
                reqsize = MXC_USBHS->outcount;
                if (reqsize > (req->reqlen - req->actlen)) {
                    reqsize = (req->reqlen - req->actlen);
                }
            }
        }

        unload_fifo(&req->data[req->actlen], get_fifo_ptr(ep), reqsize);

        req->actlen += reqsize;

        if (!ep) {
            if (req->actlen == req->reqlen) {
                /* No more data */
                MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_SERV_OUTPKTRDY | MXC_F_USBHS_CSR0_DATA_END;
                /* Done */
                MXC_USB_Request[ep] = NULL;

                if (req->callback) {
                    req->callback(req->cbdata);
                }
            } else {
                /* More data */
                MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_SERV_OUTPKTRDY;
            }
        } else {
            /* Signal to H/W that FIFO has been read */
            MXC_USBHS->outcsrl &= ~MXC_F_USBHS_OUTCSRL_OUTPKTRDY;

            if ((req->type == MAXUSB_TYPE_PKT) || (req->actlen == req->reqlen)) {
                /* Done */
                MXC_USB_Request[ep] = NULL;

                /* Disable interrupt for this endpoint */
                MXC_USBHS->introuten &= ~(1 << ep);

                /* Complete request */
                if (req->callback) {
                    req->callback(req->cbdata);
                }
            }
        }
    }
}

void MXC_USB_IrqHandler(maxusb_usbio_events_t *evt)
{
    uint32_t saved_index;
    uint32_t in_flags, out_flags, MXC_USB_flags, MXC_USB_mxm_flags;
    int i, aborted = 0;
    uint32_t intrusb, intrusben, intrin, intrinen, introut, introuten, mxm_int, mxm_int_en;

    /* Save current index register */
    saved_index = MXC_USBHS->index;

    /* Note: Hardware clears these after read, so we must process them all or they are lost */
    /*  Order of volatile accesses must be separated for IAR */
    intrusb = MXC_USBHS->intrusb;
    intrusben = MXC_USBHS->intrusben;
    MXC_USB_flags = intrusb & intrusben;

    intrin = MXC_USBHS->intrin;
    intrinen = MXC_USBHS->intrinen;
    in_flags = intrin & intrinen;

    introut = MXC_USBHS->introut;
    introuten = MXC_USBHS->introuten;
    out_flags = introut & introuten;

    /* These USB interrupt flags are W1C. */
    /*  Order of volatile accesses must be separated for IAR */
    mxm_int = MXC_USBHS->mxm_int;
    mxm_int_en = MXC_USBHS->mxm_int_en;
    MXC_USB_mxm_flags = mxm_int & mxm_int_en;
    MXC_USBHS->mxm_int = MXC_USB_mxm_flags;

    /* Map hardware-specific signals to generic stack events */
    evt->dpact  = !!(MXC_USB_flags & MXC_F_USBHS_INTRUSB_SOF_INT);
    evt->rwudn  = 0;        /* Not supported by this hardware */
    evt->bact   = !!(MXC_USB_flags & MXC_F_USBHS_INTRUSB_SOF_INT);
    evt->brst   = !!(MXC_USB_flags & MXC_F_USBHS_INTRUSB_RESET_INT);
    evt->susp   = !!(MXC_USB_flags & MXC_F_USBHS_INTRUSB_SUSPEND_INT);
    evt->novbus = !!(MXC_USB_mxm_flags & MXC_F_USBHS_MXM_INT_NOVBUS);
    evt->vbus   = !!(MXC_USB_mxm_flags & MXC_F_USBHS_MXM_INT_VBUS);
    evt->brstdn = !!(MXC_USB_flags & MXC_F_USBHS_INTRUSB_RESET_INT); /* Hardware does not signal this, so simulate it */
    evt->sudav = 0; /* Overwritten, if necessary, below */

    /* Handle control state machine */
    if (in_flags & MXC_F_USBHS_INTRIN_EP0_IN_INT) {
        /* Select endpoint 0 */
        MXC_USBHS->index = 0;
        /* Process all error conditions */
        if (MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_SENT_STALL) {
            /* Clear stall indication, go back to IDLE */
            MXC_USBHS->csr0 &= ~(MXC_F_USBHS_CSR0_SENT_STALL);
            /* Remove this from the IN flags so that it is not erroneously processed as data */
            in_flags &= ~MXC_F_USBHS_INTRIN_EP0_IN_INT;
            setup_phase = SETUP_IDLE;
            aborted = 1;
        }
        if (MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_SETUP_END) {
            /* Abort pending requests, clear early end-of-control-transaction bit, go back to IDLE */
            MXC_USBHS->csr0 |= (MXC_F_USBHS_CSR0_SERV_SETUP_END);
            setup_phase = SETUP_IDLE;

            /* Remove this from the IN flags so that it is not erroneously processed as data */
            in_flags &= ~MXC_F_USBHS_INTRIN_EP0_IN_INT;
            MXC_USB_ResetEp(0);
            aborted = 1;
        }
        /* Now, check for a SETUP packet */
        if (!aborted) {
            if ((setup_phase == SETUP_IDLE) && (MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_OUTPKTRDY)) {
                /* Flag that we got a SETUP packet */
                evt->sudav = 1;
                /* Remove this from the IN flags so that it is not erroneously processed as data */
                in_flags &= ~MXC_F_USBHS_INTRIN_EP0_IN_INT;
            } else {
                /* Otherwise, we are in endpoint 0 data IN/OUT */
                /* Fix interrupt flags so that OUTs are processed properly */
                if (setup_phase == SETUP_DATA_OUT) {
                    in_flags &= ~MXC_F_USBHS_INTRIN_EP0_IN_INT;
                    out_flags |= MXC_F_USBHS_INTRIN_EP0_IN_INT;
                }
                /* SETUP_NODATA is silently ignored by event_in_data() right now.. could fix this later */
            }
        }
    }
    /* do cleanup in cases of bus reset */
    if (evt->brst) {
        setup_phase = SETUP_IDLE;
        /* kill any pending requests */
        for (i = 0; i < MXC_USBHS_NUM_EP; i++) {
                MXC_USB_ResetEp(i);
        }
        /* no need to process events after reset */
        return;
    }

    if (in_flags) {
        event_in_data(in_flags);
    }

    if (out_flags) {
        event_out_data(out_flags);
    }

    /* Restore register index before exiting ISR */
    MXC_USBHS->index = saved_index;
}

int MXC_USB_IrqEnable(maxusb_event_t event)
{
    if (event >= MAXUSB_NUM_EVENTS) {
        return -1;
    }

    switch (event) {
        case MAXUSB_EVENT_BACT:
            /* Bus Active */
            MXC_USBHS->intrusben |= MXC_F_USBHS_INTRUSBEN_SOF_INT_EN;
            break;

        case MAXUSB_EVENT_BRST:
            /* Bus Reset */
            MXC_USBHS->intrusben |= MXC_F_USBHS_INTRUSBEN_RESET_INT_EN;
            break;

        case MAXUSB_EVENT_SUSP:
            /* Suspend */
            MXC_USBHS->power |= MXC_F_USBHS_POWER_EN_SUSPENDM;
            MXC_USBHS->intrusben |= MXC_F_USBHS_INTRUSBEN_SUSPEND_INT_EN;
            break;

        case MAXUSB_EVENT_SUDAV:
            /* Setup Data Available */
            MXC_USBHS->intrinen |= MXC_F_USBHS_INTRINEN_EP0_INT_EN;
            break;

        case MAXUSB_EVENT_VBUS:
            /* VBUS Detect */
            MXC_USBHS->mxm_int_en |= MXC_F_USBHS_MXM_INT_EN_VBUS;
            break;

        case MAXUSB_EVENT_NOVBUS:
            /* NOVBUS Detect */
            MXC_USBHS->mxm_int_en |= MXC_F_USBHS_MXM_INT_EN_NOVBUS;
            break;

        default:
            /* Other events not supported by this hardware */
            break;
    }

    return 0;
}

int MXC_USB_IrqDisable(maxusb_event_t event)
{
    if (event >= MAXUSB_NUM_EVENTS) {
        return -1;
    }

    switch (event) {
        case MAXUSB_EVENT_BACT:
            /* Bus Active */
            MXC_USBHS->intrusben &= ~MXC_F_USBHS_INTRUSBEN_SOF_INT_EN;
            break;

        case MAXUSB_EVENT_BRST:
            /* Bus Reset */
            MXC_USBHS->intrusben &= ~MXC_F_USBHS_INTRUSBEN_RESET_INT_EN;
            break;

        case MAXUSB_EVENT_SUSP:
            /* Suspend */
            MXC_USBHS->intrusben &= ~MXC_F_USBHS_INTRUSBEN_SUSPEND_INT_EN;
            MXC_USBHS->power &= ~MXC_F_USBHS_POWER_EN_SUSPENDM;
            break;

        case MAXUSB_EVENT_SUDAV:
            /* Setup Data Available */
            MXC_USBHS->intrinen &= ~MXC_F_USBHS_INTRINEN_EP0_INT_EN;
            break;

        case MAXUSB_EVENT_VBUS:
            /* VBUS Detect */
            MXC_USBHS->mxm_int_en &= ~MXC_F_USBHS_MXM_INT_EN_VBUS;
            break;

        case MAXUSB_EVENT_NOVBUS:
            /* NOVBUS Detect */
            MXC_USBHS->mxm_int_en &= ~MXC_F_USBHS_MXM_INT_EN_NOVBUS;
            break;

        default:
            /* Other events not supported by this hardware */
            break;
    }

    return 0;
}

int MXC_USB_IrqClear(maxusb_event_t event)
{
    /* No-op on this hardware, as reading the interrupt flag register clears it */

    return 0;
}

int MXC_USB_GetSetup(MXC_USB_SetupPkt *sud)
{
    volatile uint8_t *fifoptr = (uint8_t *)&MXC_USBHS->fifo0;

    /* Interrupts must be disabled while banked registers are accessed */
     MXC_SYS_Crit_Enter();

    /* Select endpoint 0 */
    MXC_USBHS->index = 0;

    if ((sud == NULL) || !(MXC_USBHS->csr0 & MXC_F_USBHS_CSR0_OUTPKTRDY)) {
         MXC_SYS_Crit_Exit();
        return -1;
    }

    /* Pull SETUP packet out of FIFO */
    sud->bmRequestType = *fifoptr;
    sud->bRequest = *fifoptr;
    sud->wValue = *fifoptr;
    sud->wValue += (*fifoptr) << 8;
    sud->wIndex = *fifoptr;
    sud->wIndex += (*fifoptr) << 8;
    sud->wLength = *fifoptr;
    sud->wLength += (*fifoptr) << 8;

    /* Check for follow-on data and advance state machine */
    if (sud->wLength > 0) {
#ifndef USE_ZEPHYR_USB_STACK
        MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_SERV_OUTPKTRDY;
#endif
        /* Determine if IN or OUT data follows */
        if (sud->bmRequestType & RT_DEV_TO_HOST) {
            setup_phase = SETUP_DATA_IN;
        } else {
            setup_phase = SETUP_DATA_OUT;
        }
    } else {
        /* SERV_OUTPKTRDY is set using MXC_USB_Ackstat() */
        setup_phase = SETUP_NODATA;
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

int MXC_USB_SetFuncAddr(unsigned int addr)
{
    if (addr > 0x7f) {
        return -1;
    }

    MXC_USBHS->faddr = addr;

    return 0;
}

MXC_USB_Req_t *MXC_USB_GetRequest(unsigned int ep)
{
    return MXC_USB_Request[ep];
}

int MXC_USB_RemoveRequest(MXC_USB_Req_t *req)
{
    if (req->ep >= MXC_USBHS_NUM_EP) {
        return -1;
    }

    if (MXC_USB_Request[req->ep] != req) {
        return -1;
    }

    /* Delete request */
    MXC_USB_Request[req->ep] = NULL;

    /* complete pending request with error */
    req->error_code = -1;
    if (req->callback) {
        req->callback(req->cbdata);
    }

    return 0;
}

int MXC_USB_WriteEndpoint(MXC_USB_Req_t *req)
{
    unsigned int ep  = req->ep;
    unsigned int len = req->reqlen;
    unsigned int armed;

    if (ep >= MXC_USBHS_NUM_EP) {
        return -1;
    }

    /* EP must be enabled (configured) */
    if (!MXC_USB_IsConfigured(ep)) {
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    MXC_USBHS->index = ep;

    /* if pending request; error */
    if (MXC_USB_Request[ep] || (MXC_USBHS->incsrl & MXC_F_USBHS_INCSRL_INPKTRDY)) {
        MXC_SYS_Crit_Exit();
        return -1;
    }

    /* assign req object */
    MXC_USB_Request[ep] = req;

    /* clear errors */
    req->error_code = 0;

    /* Determine if DMA can be used for this transmit */
    armed = 0;

    if (!armed) {
        /* EP0 or no free DMA channel found, fall back to PIO */

        /* Determine how many bytes to be sent */
        if (len > ep_size[ep]) {
            len = ep_size[ep];
        }
        MXC_USB_Request[ep]->actlen = len;

        load_fifo(get_fifo_ptr(ep), req->data, len);

        if (!ep) {
            if (MXC_USB_Request[ep]->actlen == MXC_USB_Request[ep]->reqlen) {
#ifdef USE_ZEPHYR_USB_STACK
                /* Send ZLP */
                if (MXC_USB_Request[ep]->has_zlp) {
                    MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY;
                    MXC_SYS_Crit_Exit();
                    return 0;
                }
#endif

                /* Implicit status-stage ACK, move state machine back to IDLE */
                setup_phase = SETUP_IDLE;
                MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY | MXC_F_USBHS_CSR0_DATA_END;
            } else {
                MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY;
            }
        } else {
            /* Arm for transmit to host */
            MXC_USBHS->incsrl = MXC_F_USBHS_INCSRL_INPKTRDY;
        }
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

int MXC_USB_ReadEndpoint(MXC_USB_Req_t *req)
{
    unsigned int ep  = req->ep;
    uint32_t reqsize;
    unsigned int armed;

    if (ep >= MXC_USBHS_NUM_EP) {
        return -1;
    }

    /* Interrupts must be disabled while banked registers are accessed */
    MXC_SYS_Crit_Enter();

    /* EP must be enabled (configured) and not stalled */
    if (!MXC_USB_IsConfigured(ep)) {
        MXC_SYS_Crit_Exit();
        return -1;
    }

    if (MXC_USB_IsStalled(ep)) {
        MXC_SYS_Crit_Exit();
        return -1;
    }

    /* if pending request; error */
    if (MXC_USB_Request[ep]) {
        MXC_SYS_Crit_Exit();
        return -1;
    }

    /* clear errors */
    req->error_code = 0;

    /* reset length */
    req->actlen = 0;

    /* assign the req object */
    MXC_USB_Request[ep] = req;

    /* Select endpoint */
    MXC_USBHS->index = ep;

    /* Since the OUT interrupt for EP 0 doesn't really exist, only do this logic for other endpoints */
    if (ep) {
        armed = 0;

        if (!armed) {
            /* EP0 or no free DMA channel found, fall back to PIO */

            /* See if data already in FIFO for this EP */
            if (MXC_USBHS->outcsrl & MXC_F_USBHS_OUTCSRL_OUTPKTRDY) {
                reqsize = MXC_USBHS->outcount;
                if (reqsize > (req->reqlen - req->actlen)) {
                    reqsize = (req->reqlen - req->actlen);
                }

                unload_fifo(&req->data[req->actlen], get_fifo_ptr(ep), reqsize);

                req->actlen += reqsize;

                /* Signal to H/W that FIFO has been read */
                MXC_USBHS->outcsrl &= ~MXC_F_USBHS_OUTCSRL_OUTPKTRDY;
                if ((req->type == MAXUSB_TYPE_PKT) || (req->actlen == req->reqlen)) {
                 /* Done with request, callback fires if configured */
                    MXC_SYS_Crit_Exit();
                    MXC_USB_Request[ep] = NULL;

                    if (req->callback) {
                        req->callback(req->cbdata);
                    }
                    return 0;
                } else {
                    /* Not done, more data requested */
                    MXC_USBHS->introuten |= (1 << ep);
                }
            } else {
                /* No data, will need an interrupt to service later */
                MXC_USBHS->introuten |= (1 << ep);
            }
        }
    }

    MXC_SYS_Crit_Exit();
    return 0;
}

void MXC_USB_RemoteWakeup(void)
{
    if (driver_opts.delay_us) {
        MXC_USBHS->power |= MXC_F_USBHS_POWER_RESUME;
        driver_opts.delay_us(10000);
        MXC_USBHS->power &= ~MXC_F_USBHS_POWER_RESUME;
    }
}

int MXC_USB_TestMode(unsigned int value)
{
    const uint8_t test_packet[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
                   0xAA, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
                   0xEE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xBF, 0xDF,
                   0xEF, 0xF7, 0xFB, 0xFD, 0xFC, 0x7E, 0xBF, 0xDF,
                   0xEF, 0xF7, 0xFB, 0xFD, 0x7E};

    switch (value) {
    case 0x01:
        /* Test_J */
        MXC_USBHS->testmode = MXC_F_USBHS_TESTMODE_TEST_J;
        break;
    case 0x02:
        /* Test_K */
        MXC_USBHS->testmode = MXC_F_USBHS_TESTMODE_TEST_K;
        break;
    case 0x03:
        /* Test_SE0_NAK */
        MXC_USBHS->testmode = MXC_F_USBHS_TESTMODE_TEST_SE0_NAK;
        break;
    case 0x04:
        /* Test_Packet */
        /* Load EP 0 with data provided by section 11.4 of musbhsfc_pg.pdf */
        /* sizeof() considered safe, since we use uint8_t explicitly */
        load_fifo(get_fifo_ptr(0), (uint8_t*)test_packet, sizeof(test_packet));
        MXC_USBHS->csr0 |= MXC_F_USBHS_CSR0_INPKTRDY;
        MXC_USBHS->testmode = MXC_F_USBHS_TESTMODE_TEST_PKT;
        break;
    default:
        /* Unsupported */
        return -1;
    }

    return 0;
}
