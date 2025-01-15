/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

/**
 * @file       mxc_sys.c
 * @brief      System layer driver.
 * @details    This driver is used to control the system layer of the device.
 */

/* **** Includes **** */
#include <stddef.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"
#include "pwrseq_regs.h"
#include "aes.h"
#include "flc.h"

/**
 * @ingroup mxc_sys
 * @{
 */

/* **** Definitions **** */
#define MXC_SYS_CLOCK_TIMEOUT MSEC(1)

// DAP Lock macros
#define INFOBLOCK_DAP_LOCK_OFFSET 0x30
#define DAP_LOCK_SEQUENCE_01 0x5A5AA5A5
#define DAP_LOCK_SEQUENCE_23 0xFFFFFFFF

/* **** Globals **** */

/* Symbol defined when loading RISCV image */
extern uint32_t _binary_riscv_bin_start;

/* **** Functions **** */

/* ************************************************************************** */
#if CONFIG_TRUSTED_EXECUTION_SECURE
int MXC_SYS_GetUSN(uint8_t *usn, uint8_t *checksum)
{
    int err = E_NO_ERROR;
    uint32_t *infoblock = (uint32_t *)MXC_INFO_MEM_BASE;

    if (usn == NULL) {
        return E_NULL_PTR;
    }

    /* Read the USN from the info block */
    MXC_FLC_UnlockInfoBlock(MXC_INFO_MEM_BASE);

    memset(usn, 0, MXC_SYS_USN_CHECKSUM_LEN);

    usn[0] = (infoblock[0] & 0x007F8000) >> 15;
    usn[1] = (infoblock[0] & 0x7F800000) >> 23;
    usn[2] = (infoblock[1] & 0x0000007F) << 1;
    usn[2] |= (infoblock[0] & 0x80000000) >> 31;
    usn[3] = (infoblock[1] & 0x00007F80) >> 7;
    usn[4] = (infoblock[1] & 0x007F8000) >> 15;
    usn[5] = (infoblock[1] & 0x7F800000) >> 23;
    usn[6] = (infoblock[2] & 0x007F8000) >> 15;
    usn[7] = (infoblock[2] & 0x7F800000) >> 23;
    usn[8] = (infoblock[3] & 0x0000007F) << 1;
    usn[8] |= (infoblock[2] & 0x80000000) >> 31;
    usn[9] = (infoblock[3] & 0x00007F80) >> 7;
    usn[10] = (infoblock[3] & 0x007F8000) >> 15;

    /* If requested, verify and return the checksum */
    if (checksum != NULL) {
        uint8_t check_csum[MXC_SYS_USN_CHECKSUM_LEN];
        uint8_t aes_key[MXC_SYS_USN_CHECKSUM_LEN] = { 0 }; // NULL Key (per checksum spec)

        checksum[0] = ((infoblock[3] & 0x7F800000) >> 23);
        checksum[1] = ((infoblock[4] & 0x007F8000) >> 15);

        // Info block only accessible from secure code.
        //  Use Secure DMA1.
        err = MXC_AES_Init(MXC_DMA1);
        if (err) {
            MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);
            return err;
        }

        // Set NULL Key
        MXC_AES_SetExtKey((const void *)aes_key, MXC_AES_128BITS);

        // Compute Checksum
        mxc_aes_req_t aes_req;
        aes_req.length = MXC_SYS_USN_CHECKSUM_LEN / 4;
        aes_req.inputData = (uint32_t *)usn;
        aes_req.resultData = (uint32_t *)check_csum;
        aes_req.keySize = MXC_AES_128BITS;
        aes_req.encryption = MXC_AES_ENCRYPT_EXT_KEY;
        aes_req.callback = NULL;

        err = MXC_AES_Generic(&aes_req);
        if (err) {
            MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);
            return err;
        }

        MXC_AES_Shutdown();

        // Verify Checksum
        if (check_csum[0] != checksum[1] || check_csum[1] != checksum[0]) {
            MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);
            return E_INVALID;
        }
    }

    /* Add the info block checksum to the USN */
    usn[11] = ((infoblock[3] & 0x7F800000) >> 23);
    usn[12] = ((infoblock[4] & 0x007F8000) >> 15);

    MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);

    return err;
}
#endif

/* ************************************************************************** */
int MXC_SYS_GetRevision(void)
{
    return MXC_GCR->revision;
}

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 and 64 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        return !(MXC_GCR->pclkdis1 & (0x1 << clock));
    } else {
        return !(MXC_GCR->pclkdis0 & (0x1 << clock));
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 and 64 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclkdis1 |= (0x1 << clock);
    } else {
        MXC_GCR->pclkdis0 |= (0x1 << clock);
    }
}

/* ************************************************************************** */
void MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 and 64 for the perckcn1 register. */
    if (clock > 31) {
        clock -= 32;
        MXC_GCR->pclkdis1 &= ~(0x1 << clock);
    } else {
        MXC_GCR->pclkdis0 &= ~(0x1 << clock);
    }
}
/* ************************************************************************** */
void MXC_SYS_RTCClockEnable()
{
    MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;
}

/* ************************************************************************** */
int MXC_SYS_RTCClockDisable(void)
{
    /* Check that the RTC is not the system clock source */
    if ((MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) != MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERTCO) {
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ERTCO_EN;
        return E_NO_ERROR;
    } else {
        return E_BAD_STATE;
    }
}

int MXC_SYS_Select32KClockSource(mxc_sys_32k_clock_t clock)
{
    if (clock > MXC_SYS_32K_CLOCK_RTC_IN) {
        return E_BAD_PARAM;
    }

    MXC_MCR->ctrl &= ~MXC_F_MCR_CTRL_CLKSEL;
    MXC_MCR->ctrl |= (clock << MXC_F_MCR_CTRL_CLKSEL_POS);

    return E_NO_ERROR;
}

/******************************************************************************/
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock)
{
    switch (clock) {
    case MXC_SYS_CLOCK_IPO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IPO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IPO_RDY);
        break;

    case MXC_SYS_CLOCK_IBRO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IBRO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IBRO_RDY);
        break;

    case MXC_SYS_CLOCK_EXTCLK:
        // No "RDY" bit to monitor, so just configure the GPIO
        return MXC_GPIO_Config(&gpio_cfg_extclk);
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 131k clock is always enabled
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_INRO_RDY);
        break;

    case MXC_SYS_CLOCK_ERFO:
        MXC_GCR->btleldoctrl |= MXC_F_GCR_BTLELDOCTRL_RF_EN | MXC_F_GCR_BTLELDOCTRL_BB_EN;

        /* Initialize kickstart circuit
           Select Kick start circuit clock source- IPO/ISO 
        */
        // TODO(ME30): MXC_FCR missing ERFOKS definition
        // MXC_FCR->erfoks = ((MXC_S_FCR_ERFOKS_KSCLKSEL_ISO)
        //                    /* Set Drive strengh - 0x1,0x2,0x3 */
        //                    | ((0x1) << MXC_F_FCR_ERFOKS_KSERFODRIVER_POS)
        //                    /* Set kick count 1-127 */
        //                    | (0x8)
        //                    /* Set double pulse length  On/Off*/
        //                    | (0 & MXC_F_FCR_ERFOKS_KSERFO2X)
        //                    /* Enable On/Off */
        //                    | (MXC_F_FCR_ERFOKS_KSERFO_EN));

        /* Enable ERFO */
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERFO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ERFO_RDY);
        break;

    case MXC_SYS_CLOCK_ERTCO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ERTCO_RDY);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }
}

/******************************************************************************/
int MXC_SYS_ClockSourceDisable(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;

    // Don't turn off the clock we're running on
    if (clock == current_clock) {
        return E_BAD_PARAM;
    }

    switch (clock) {
    case MXC_SYS_CLOCK_IPO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_IPO_EN;
        break;

    case MXC_SYS_CLOCK_IBRO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_IBRO_EN;
        break;

    case MXC_SYS_CLOCK_EXTCLK:
        /*
        There's not a great way to disable the external clock.
        Deinitializing the GPIO here may have unintended consequences
        for application code.
        Selecting a different system clock source is sufficient
        to "disable" the EXT_CLK source.
        */
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 131k clock is always enabled
        break;

    case MXC_SYS_CLOCK_ERFO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ERFO_EN;
        break;

    case MXC_SYS_CLOCK_ERTCO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ERTCO_EN;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_Clock_Timeout(uint32_t ready)
{
#ifdef __riscv
    // The current RISC-V implementation is to block until the clock is ready.
    // We do not have access to a system tick in the RV core.
    while (!(MXC_GCR->clkctrl & ready)) {}
    return E_NO_ERROR;
#else
#ifndef BOARD_ME17_TESTER
    // Start timeout, wait for ready
    MXC_DelayAsync(MXC_SYS_CLOCK_TIMEOUT, NULL);

    do {
        if (MXC_GCR->clkctrl & ready) {
            MXC_DelayAbort();
            return E_NO_ERROR;
        }
    } while (MXC_DelayCheck() == E_BUSY);

    return E_TIME_OUT;
#else

    return E_NO_ERROR;
#endif

#endif // __riscv
}
/* ************************************************************************** */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;
    int err = E_NO_ERROR;

    // Save the current system clock
    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;

    switch (clock) {
    case MXC_SYS_CLOCK_IPO:

        // Enable IPO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IPO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IPO_EN;

            // Check if IPO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IPO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set IPO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO);

        break;

    case MXC_SYS_CLOCK_IBRO:

        // Enable IBRO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IBRO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_IBRO_EN;

            // Check if IBRO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_IBRO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set IBRO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO);

        break;

    case MXC_SYS_CLOCK_EXTCLK:
        /*
        There's not "EXT_CLK RDY" bit for the ME17, so we'll
        blindly enable (configure GPIO) the external clock every time.
        */
        err = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_EXTCLK);
        if (err)
            return err;

        // Set EXT clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK);

        break;

    case MXC_SYS_CLOCK_ERFO:

        // Enable ERFO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERFO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERFO_EN;

            // Check if ERFO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ERFO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set ERFO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO);

        break;

    case MXC_SYS_CLOCK_INRO:
        // Set INRO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_INRO);

        break;

    case MXC_SYS_CLOCK_ERTCO:

        // Enable ERTCO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERTCO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;

            // Check if ERTCO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ERTCO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set ERTCO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERTCO);

        break;

    default:
        return E_BAD_PARAM;
    }

    // Wait for system clock to be ready
    if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_SYSCLK_RDY) != E_NO_ERROR) {
        // Restore the old system clock if timeout
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL, current_clock);

        return E_TIME_OUT;
    }

    // Update the system core clock
    SystemCoreClockUpdate();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SYS_SetClockDiv(mxc_sys_system_clock_div_t div)
{
    /* Return if this setting is already current */
    if (div == MXC_SYS_GetClockDiv()) {
        return;
    }

    MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_DIV, div);

    SystemCoreClockUpdate();
}

/* ************************************************************************** */
mxc_sys_system_clock_div_t MXC_SYS_GetClockDiv(void)
{
    return (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV);
}

/* ************************************************************************** */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset)
{
    /* The mxc_sys_reset_t enum uses enum values that are the offset by 32 and 64 for the rst register. */
    if (reset > 31) {
        reset -= 32;
        MXC_GCR->rst1 = (0x1 << reset);
        while (MXC_GCR->rst1 & (0x1 << reset)) {}
    } else {
        MXC_GCR->rst0 = (0x1 << reset);
        while (MXC_GCR->rst0 & (0x1 << reset)) {}
    }
}

#if CONFIG_TRUSTED_EXECUTION_SECURE
/* ************************************************************************** */
int MXC_SYS_LockDAP_Permanent(void)
{
#ifdef DEBUG
    // Locking the DAP is not supported while in DEBUG.
    // To use this function, build for release ("make release")
    // or set DEBUG = 0
    // (see https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-tables)
    return E_NOT_SUPPORTED;
#else
    int err;
    uint32_t info_blk_addr;
    uint32_t lock_sequence[4];

    // Infoblock address to write lock sequence to
    info_blk_addr = MXC_INFO_MEM_BASE + INFOBLOCK_DAP_LOCK_OFFSET;

    // Set lock sequence
    lock_sequence[0] = DAP_LOCK_SEQUENCE_01;
    lock_sequence[1] = DAP_LOCK_SEQUENCE_01;
    lock_sequence[2] = DAP_LOCK_SEQUENCE_23;
    lock_sequence[3] = DAP_LOCK_SEQUENCE_23;

    // Initialize FLC
    MXC_FLC_Init();

    // Unlock infoblock
    MXC_FLC_UnlockInfoBlock(info_blk_addr);

    // Write DAP lock sequence to infoblock
    err = MXC_FLC_Write128(info_blk_addr, lock_sequence);

    // Re-lock infoblock
    MXC_FLC_LockInfoBlock(info_blk_addr);

    return err;
#endif
}
#endif

/* ************************************************************************** */
void MXC_SYS_StartClockMeasure(mxc_sys_compare_clock_t clock, uint32_t compareClockTicks)
{
    /* Assuming that both clocks are already enabled */

    /* Setup the comparison clock */
    MXC_FCR->frqcntctrl = (MXC_FCR->frqcntctrl & ~(MXC_F_FCR_FRQCNTCTRL_CMP_CLKSEL)) | clock;

    /* Set ticks of the comparison clock */
    MXC_FCR->frqcntcmp = compareClockTicks;

    /*
     * Enable interrupt, note that we don't see the flag if we leave
     * this disabled.
     */
    MXC_FCR->inten |= MXC_F_FCR_INTFL_FRQCNT;

    /* Clear the interrupt flag */
    MXC_FCR->intfl = MXC_F_FCR_INTFL_FRQCNT;

    /* Start the procedure */
    MXC_FCR->frqcntctrl |= MXC_F_FCR_FRQCNTCTRL_START;
}

/* ************************************************************************** */
uint32_t MXC_SYS_GetClockMeasure(void)
{
    /* Return 0 if the procedure is incomplete */
    if (!(MXC_FCR->intfl & MXC_F_FCR_INTFL_FRQCNT)) {
        return 0;
    }

    /* Calculate the frequency */
    uint64_t freq = (uint64_t)ERFO_FREQ * (uint64_t)MXC_FCR->cmpclk / (uint64_t)MXC_FCR->refclk;

    return (uint32_t)freq;
}

/* ************************************************************************** */
uint32_t MXC_SYS_ClockMeasure(mxc_sys_compare_clock_t clock, uint32_t compareClockTicks)
{
    /* Assuming that both clocks are already enabled */
    MXC_SYS_StartClockMeasure(clock, compareClockTicks);

    /* Wait for the procedure to finish */
    while (!(MXC_FCR->intfl & MXC_F_FCR_INTFL_FRQCNT)) {}

    return MXC_SYS_GetClockMeasure();
}

/**@} end of mxc_sys */
