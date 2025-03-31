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

/**
 * @file mxc_sys.c
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
#include "lpgcr_regs.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"
#include "pwrseq_regs.h"
#include "rtc_regs.h"
#include "flc.h"
#include "ctb.h"

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
int MXC_SYS_GetUSN(uint8_t *usn, uint8_t *checksum)
{
    if (usn == NULL) {
        return E_NULL_PTR;
    }

    uint32_t *infoblock = (uint32_t *)MXC_INFO0_MEM_BASE;

    /* Read the USN from the info block */
    MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE);

    memset(usn, 0, MXC_SYS_USN_LEN);

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

    // Compute the checksum
    if (checksum != NULL) {
        uint32_t key[4];
        uint32_t pt32[4];
        uint32_t check_csum32[4];
        uint8_t check_csum[MXC_SYS_USN_CHECKSUM_LEN];

        /* Initialize key and plaintext */
        memset(key, 0, MXC_SYS_USN_CHECKSUM_LEN);
        memset(pt32, 0, MXC_SYS_USN_CHECKSUM_LEN);
        memcpy(pt32, usn, MXC_SYS_USN_LEN);

        /* Read the checksum from the inspfo block */
        checksum[1] = ((infoblock[3] & 0x7F800000) >> 23);
        checksum[0] = ((infoblock[4] & 0x007F8000) >> 15);

        MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER);

        /* Reset the CTB */
        MXC_CTB->ctrl = MXC_F_CTB_CTRL_RST;

        /* Set the legacy bit */
        MXC_CTB->ctrl |= MXC_F_CTB_CTRL_FLAG_MODE;

        /* Clear interrupt flags */
        MXC_CTB->ctrl |= MXC_F_CTB_CTRL_CPH_DONE;

        /* Setup the key source */
        MXC_CTB->cipher_ctrl = MXC_S_CTB_CIPHER_CTRL_SRC_CIPHERKEY;

        /* Setup the CT calculation */
        MXC_CTB->cipher_ctrl |= MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128;

        /* Load the key */
        MXC_CTB->cipher_key[0] = key[0];
        MXC_CTB->cipher_key[1] = key[1];
        MXC_CTB->cipher_key[2] = key[2];
        MXC_CTB->cipher_key[3] = key[3];

        /* Wait for the ready flag */
        while (!(MXC_CTB->ctrl & MXC_F_CTB_CTRL_RDY)) {}

        /* Copy data to start the operation */
        MXC_CTB->din[0] = pt32[0];
        MXC_CTB->din[1] = pt32[1];
        MXC_CTB->din[2] = pt32[2];
        MXC_CTB->din[3] = pt32[3];

        /* Wait for and clear the done flag */
        while (!(MXC_CTB->ctrl & MXC_F_CTB_CTRL_CPH_DONE)) {}
        MXC_CTB->ctrl |= MXC_F_CTB_CTRL_CPH_DONE;

        /* Copy out the cipher text */
        check_csum32[0] = MXC_CTB->dout[0];
        check_csum32[1] = MXC_CTB->dout[1];
        check_csum32[2] = MXC_CTB->dout[2];
        check_csum32[3] = MXC_CTB->dout[3];

        memcpy(check_csum, check_csum32, MXC_SYS_USN_CHECKSUM_LEN);

        /* Verify the checksum */
        if ((checksum[0] != check_csum[0]) || (checksum[1] != check_csum[1])) {
            MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
            return E_UNKNOWN;
        }
    }

    /* Add the info block checksum to the USN */
    usn[11] = ((infoblock[3] & 0x7F800000) >> 23);
    usn[12] = ((infoblock[4] & 0x007F8000) >> 15);

    MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock)
{
    /* The mxc_sys_periph_clock_t enum uses enum values that are the offset by 32 and 64 for the perckcn1 register. */
    if (clock > 63) {
        clock -= 64;
        return !(MXC_LPGCR->pclkdis & (0x1 << clock));
    } else if (clock > 31) {
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
    if (clock > 63) {
        clock -= 64;
        MXC_LPGCR->pclkdis |= (0x1 << clock);
    } else if (clock > 31) {
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
    if (clock > 63) {
        clock -= 64;
        MXC_LPGCR->pclkdis &= ~(0x1 << clock);
    } else if (clock > 31) {
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

    case MXC_SYS_CLOCK_ISO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ISO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ISO_RDY);
        break;

    case MXC_SYS_CLOCK_EXTCLK:
        return MXC_GPIO_Config(&gpio_cfg_extclk);
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 80k clock is always enabled
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_INRO_RDY);
        break;

    case MXC_SYS_CLOCK_ERFO:
        MXC_GCR->btleldoctrl |= MXC_F_GCR_BTLELDOCTRL_LDORFEN | MXC_F_GCR_BTLELDOCTRL_LDOBBEN;

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

    case MXC_SYS_CLOCK_ISO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ISO_EN;
        break;

    case MXC_SYS_CLOCK_IBRO:
        if ((MXC_GCR->pm & MXC_F_GCR_PM_MODE) == MXC_S_GCR_PM_MODE_UPM) {
            MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_IBRO_EN;
        }
        break;

    case MXC_SYS_CLOCK_EXTCLK:
        // MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_EXTCLK_EN;
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 80k clock is always enabled
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
    // Start timeout, wait for ready
    MXC_DelayAsync(MXC_SYS_CLOCK_TIMEOUT, NULL);
#ifndef ME18_WLP_TEST
    /* TODO: Timeout on clock switch, use this for untrimmed parts. */
    while (!(MXC_GCR->clkctrl & ready)) {}
    return E_NO_ERROR;
#else
    return E_NO_ERROR;
#endif
}

/* ************************************************************************** */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock)
{
    uint32_t current_clock;

    // Save the current system clock
    current_clock = MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL;

    switch (clock) {
    case MXC_SYS_CLOCK_IPO:

        // Enable IPO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IPO_EN)) {
            MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
        }

        // Set IPO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO);

        break;

    case MXC_SYS_CLOCK_ISO:

        // Enable ISO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ISO_EN)) {
            MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);
        }

        // Set ISO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ISO);

        break;

    case MXC_SYS_CLOCK_IBRO:

        // Enable IBRO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_IBRO_EN)) {
            MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        }

        // Set IBRO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO);

        break;

    case MXC_SYS_CLOCK_EXTCLK:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_EXTCLK);

        // Set external clock clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK);

        break;

    case MXC_SYS_CLOCK_ERFO:

        // Enable ERFO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERFO_EN)) {
            MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERFO);
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
            MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
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

int MXC_SYS_ClockCalibrate(mxc_sys_system_clock_t clock)
{
    if (clock != MXC_SYS_CLOCK_IPO) {
        return E_BAD_PARAM; // Only the IPO supports calibration
    }

    int err = E_NO_ERROR;
    // The following section implements section 4.1.1.1 of the MAX32690 UG Rev 1
    if ((err = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO)))
        return err;
    MXC_SETFIELD(MXC_FCR->autocal2, MXC_F_FCR_AUTOCAL2_ACDIV, 3662 << MXC_F_FCR_AUTOCAL2_ACDIV_POS);
    MXC_SETFIELD(MXC_FCR->autocal1, MXC_F_FCR_AUTOCAL1_INITTRM,
                 0x40 << MXC_F_FCR_AUTOCAL1_INITTRM_POS);
    MXC_FCR->autocal0 |= 0x7;
    MXC_Delay(MXC_DELAY_MSEC(10)); // Wait 10ms for calibration to complete
    // Calculated trim is loaded to MXC_FCR->hirc96mactmrout and is used by the hardware as long as
    // MXC_FCR->autocal0.acen is set to 1
    MXC_FCR->autocal0 &= ~MXC_F_FCR_AUTOCAL0_ACRUN; // Stop the calibration
    MXC_FCR->autocal0 |= MXC_F_FCR_AUTOCAL0_ACEN; // Enable use of calibration value
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
    if (reset > 63) {
        reset -= 64;
        MXC_LPGCR->rst = (0x1 << reset);
        while (MXC_LPGCR->rst & (0x1 << reset)) {}
    } else if (reset > 31) {
        reset -= 32;
        MXC_GCR->rst1 = (0x1 << reset);
        while (MXC_GCR->rst1 & (0x1 << reset)) {}
    } else {
        MXC_GCR->rst0 = (0x1 << reset);
        while (MXC_GCR->rst0 & (0x1 << reset)) {}
    }
}

#if !defined(__CC_ARM) && !defined(__ICCARM__)
/* ************************************************************************** */
void MXC_SYS_RISCVRun(void)
{
    /* Disable the the RSCV */
    MXC_GCR->pclkdis1 |= MXC_F_GCR_PCLKDIS1_CPU1;

    /* Set the interrupt vector base address */
    MXC_FCR->urvbootaddr = (uint32_t)&_binary_riscv_bin_start;

    /* Power up the RSCV */
    MXC_GCR->pclkdis1 &= ~(MXC_F_GCR_PCLKDIS1_CPU1);

    /* CPU1 reset */
    MXC_GCR->rst1 |= MXC_F_GCR_RST1_CPU1;
}

/* ************************************************************************** */
void MXC_SYS_RISCVShutdown(void)
{
    /* Disable the the RSCV */
    MXC_GCR->pclkdis1 |= MXC_F_GCR_PCLKDIS1_CPU1;
}

/* ************************************************************************** */
uint32_t MXC_SYS_RiscVClockRate(void)
{
    // If in LPM mode and the PCLK is selected as the RV32 clock source,
    if (((MXC_GCR->pm & MXC_F_GCR_PM_MODE) == MXC_S_GCR_PM_MODE_LPM) &&
        (MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_ISOCLK_SELECT)) {
        return ISO_FREQ;
    } else {
        return PeripheralClock;
    }
}
#endif // !__CC_ARM and !__ICCARM__

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

int MXC_SYS_SetBypass(mxc_sys_system_clock_t clock, bool bypass)
{
    // Only ERFO and ERTCO support this option.
    if (clock == MXC_SYS_CLOCK_ERFO) {
        MXC_SETFIELD(MXC_GCR->pm, MXC_F_GCR_PM_ERFO_BP, bypass << MXC_F_GCR_PM_ERFO_BP_POS);
    } else if (clock == MXC_SYS_CLOCK_ERTCO) {
        MXC_SETFIELD(MXC_RTC->oscctrl, MXC_F_RTC_OSCCTRL_BYPASS,
                     bypass << MXC_F_RTC_OSCCTRL_BYPASS_POS);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/**@} end of mxc_sys */
