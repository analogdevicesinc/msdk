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
#include "aes.h"
#include "flc.h"
#include "lpgcr_regs.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"
#include "pwrseq_regs.h"

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

/* Symbol defined by the build system when loading RISCV image */
extern volatile void const *_riscv_boot; // Defined in linker file

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SYS_GetUSN(uint8_t *usn, uint8_t *checksum)
{
    int err = E_NO_ERROR;
    uint32_t *infoblock = (uint32_t *)MXC_INFO0_MEM_BASE;

    if (usn == NULL) {
        return E_NULL_PTR;
    }

    /* Read the USN from the info block */
    MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE);

    uint32_t _usn_32[MXC_SYS_USN_CHECKSUM_LEN / 4];
    // ^ Declare as uint32_t to preserve mem alignment
    uint8_t *_usn_8 = (uint8_t *)_usn_32;
    memset(_usn_8, 0, MXC_SYS_USN_CHECKSUM_LEN);

    _usn_8[0] = (infoblock[0] & 0x007F8000) >> 15;
    _usn_8[1] = (infoblock[0] & 0x7F800000) >> 23;
    _usn_8[2] = (infoblock[1] & 0x0000007F) << 1;
    _usn_8[2] |= (infoblock[0] & 0x80000000) >> 31;
    _usn_8[3] = (infoblock[1] & 0x00007F80) >> 7;
    _usn_8[4] = (infoblock[1] & 0x007F8000) >> 15;
    _usn_8[5] = (infoblock[1] & 0x7F800000) >> 23;
    _usn_8[6] = (infoblock[2] & 0x007F8000) >> 15;
    _usn_8[7] = (infoblock[2] & 0x7F800000) >> 23;
    _usn_8[8] = (infoblock[3] & 0x0000007F) << 1;
    _usn_8[8] |= (infoblock[2] & 0x80000000) >> 31;
    _usn_8[9] = (infoblock[3] & 0x00007F80) >> 7;
    _usn_8[10] = (infoblock[3] & 0x007F8000) >> 15;

    /* If requested, verify and return the checksum */
    if (checksum != NULL) {
        uint32_t _check_csum_32[MXC_SYS_USN_CHECKSUM_LEN / 4];
        // ^ Declare as uint32_t to preserve mem alignment
        memset(_check_csum_32, 0, (MXC_SYS_USN_CHECKSUM_LEN / 4) * sizeof(uint32_t));
        uint8_t *check_csum = (uint8_t *)_check_csum_32;
        uint8_t aes_key[MXC_SYS_USN_CHECKSUM_LEN] = { 0 }; // NULL Key (per checksum spec)

        // Read Checksum from the infoblock
        checksum[0] = ((infoblock[3] & 0x7F800000) >> 23);
        checksum[1] = ((infoblock[4] & 0x007F8000) >> 15);

        err = MXC_AES_Init();
        if (err) {
            MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
            return err;
        }

        // Set NULL Key
        MXC_AES_SetExtKey((const void *)aes_key, MXC_AES_128BITS);

        // Compute Checksum
        mxc_aes_req_t aes_req;
        aes_req.length = MXC_SYS_USN_CHECKSUM_LEN / 4;
        aes_req.inputData = _usn_32;
        aes_req.resultData = _check_csum_32;
        aes_req.keySize = MXC_AES_128BITS;
        aes_req.encryption = MXC_AES_ENCRYPT_EXT_KEY;
        aes_req.callback = NULL;

        err = MXC_AES_Generic(&aes_req);
        if (err) {
            MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
            return err;
        }

        MXC_AES_Shutdown();

        // Verify Checksum
        // The checksum results will be in the least significant bytes of the aes output.
        if (check_csum[0] != checksum[1] || check_csum[1] != checksum[0]) {
            MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
            return E_INVALID;
        }
    }

    /* Add the info block checksum to the USN */
    _usn_8[11] = ((infoblock[3] & 0x7F800000) >> 23);
    _usn_8[12] = ((infoblock[4] & 0x007F8000) >> 15);

    MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);

    memcpy(usn, _usn_8, MXC_SYS_USN_LEN);

    return err;
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
        // No EXT_CLK "RDY" bit for the AI85 so we return the GPIO config
        return MXC_GPIO_Config(&gpio_cfg_extclk);
        break;

    case MXC_SYS_CLOCK_INRO:
        // The 80k clock is always enabled
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_INRO_RDY);
        break;

#if TARGET_NUM == 32655

    case MXC_SYS_CLOCK_ERFO:
        MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERFO_EN;
        return MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ERFO_RDY);
        break;
#endif

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

#if TARGET_NUM == 78000 // ai85 only

    case MXC_SYS_CLOCK_ISO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ISO_EN;
        break;
#endif

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
        // The 80k clock is always enabled
        break;

#if TARGET_NUM == 32655 // ME17 only

    case MXC_SYS_CLOCK_ERFO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ERFO_EN;
        break;
#endif

    case MXC_SYS_CLOCK_ERTCO:
        MXC_GCR->clkctrl &= ~MXC_F_GCR_CLKCTRL_ERTCO_EN;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
mxc_sys_system_clock_div_t MXC_SYS_GetClockDiv(void)
{
    return (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV);
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
int MXC_SYS_Clock_Timeout(uint32_t ready)
{
#ifdef __riscv
    // The current RISC-V implementation is to block until the clock is ready.
    // We do not have access to a system tick in the RV core.
    while (!(MXC_GCR->clkctrl & ready)) {}
    return E_NO_ERROR;
#else
    // Start timeout, wait for ready
    MXC_DelayAsync(MXC_SYS_CLOCK_TIMEOUT, NULL);

    do {
        if (MXC_GCR->clkctrl & ready) {
            MXC_DelayAbort();
            return E_NO_ERROR;
        }
    } while (MXC_DelayCheck() == E_BUSY);

    return E_TIME_OUT;
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

#if TARGET_NUM == 78000 // AI85 only

    case MXC_SYS_CLOCK_ISO:

        // Enable ISO clock
        if (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ISO_EN)) {
            MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ISO_EN;

            // Check if ISO clock is ready
            if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ISO_RDY) != E_NO_ERROR) {
                return E_TIME_OUT;
            }
        }

        // Set ISO clock as System Clock
        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ISO);

        break;
#endif

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
        // No EXT_CLK "RDY" bit for AI85 so we enable every time
        err = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_EXTCLK);
        if (err) {
            return err;
        }

        MXC_SETFIELD(MXC_GCR->clkctrl, MXC_F_GCR_CLKCTRL_SYSCLK_SEL,
                     MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK);

        break;

#if TARGET_NUM == 32655 // ME17 only

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
#endif

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

/* ************************************************************************** */
void MXC_SYS_RISCVRun(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CPU1);

    /* Disable the the RSCV */
    MXC_GCR->pclkdis1 |= MXC_F_GCR_PCLKDIS1_CPU1;

    /* Set the interrupt vector base address */
    MXC_FCR->urvbootaddr = (uint32_t)&_riscv_boot;

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
        ((MXC_PWRSEQ->lpcn & MXC_F_PWRSEQ_LPCN_LPMCLKSEL) == 0)) {
        return SystemCoreClock / 2;
    } else {
        return ISO_FREQ;
    }
}

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

/**@} end of mxc_sys */
