/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 **************************************************************************** */

/**
 * @file    pal_bb_afe.c
 * @brief   Functions to use the analog front end.
 * @details
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include <string.h>
#include "mxc_device.h"
#include "gcr_regs.h"
#include "simo_regs.h"
#include "afe_regs.h"
#include "dbb_registers.h"
#include "pal_bb_afe.h"
#include "pal_bb_dbb.h"
#include "pal_bb_seq.h"
#include "pal_bb_cfg.h"

#include "flc.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Trim values in OTP are valid */
#ifndef OTP_VALID
#define OTP_VALID TRUE
#endif

/* Default TX power level, in units of dBm */
#ifndef PAL_BB_AFE_INIT_TX_POWER
#define PAL_BB_AFE_INIT_TX_POWER 0
#endif

/* Starting address for the OTP trim values */
#define OTP_OFFSET 0x10800400

/* Number of FM gain calibration iterations */
#define FM_GAIN_ITERS 20

/* Number of B_OSC steps to use in calibration */
#define B_OSC_STEPS 70

/* Maximum number of readings for timeout */
#define IQ_CAPTURE_TO_ITERS 1000

/* Starting channel for the calibration */
#define INITIAL_CAL_CH 0

/* Definitions for TX and RX direction */
#define DIR_RX 0
#define DIR_TX 1

/* Number of channels that we have */
#define NUM_CH 40

/* Assumes that DBB_CLK_RATE_HZ, XTAL input is 32000000 */
#if (DBB_CLK_RATE_HZ != 32000000)
#error "DBB_CLK_RATE_HZ must be 32000000"
#endif

/*
 *
 * Note that F_XTAL is defined as 2x the XTAL freq
 * Freq = (DIV_INT + 16) * 64 MHz
 * DIV_INT = 0 .. 31
 *
 * TX_freq = Freq
 *
 * RX_Freq = (4/5) * TX_freq
 *
 * Frac = DIV_FRAC * (F_XTAL) / 2^15
 *
 * DIV_FRAC = 15 bits
 *
 */
#define F_XTAL 64000000UL

/* due to rounding, the FRAC frequency is multiplied by this number to increase
 * resolution */
#define FRAC_FREQ_MULTIPLIER 16UL

#define FRAC_FREQ 31250UL /* = 16 * (F_XTAL / 2^15) */

/* Calculate the channel frequencies and the divisors */
/* where n = 0 .. 39 base = 2.402 GHz, channel is 2 MHz */
#define BTLE_CHANNEL_FREQ(n) (2402000000UL + (n)*2000000UL)

#define CALC_DIV_INT(freq) (((freq) / F_XTAL) - 16UL)

#define FREQ_DIFF(freq) ((freq) - ((CALC_DIV_INT(freq) + 16UL) * F_XTAL))

#define CALC_DIV_FRAC(freq) \
    ((FREQ_DIFF(freq) * FRAC_FREQ_MULTIPLIER + (FRAC_FREQ / 2UL)) / FRAC_FREQ)
#define TX_FREQ(f) (f)
#define RX_FREQ(f) (((f) / 5UL) * 4UL)

#define TX_FREQ_CHAN(n) TX_FREQ(BTLE_CHANNEL_FREQ(n))
#define RX_FREQ_CHAN(n) RX_FREQ(BTLE_CHANNEL_FREQ(n))

#define RX_CHANNEL_ENTRY(n) CALC_DIV_FRAC(RX_FREQ_CHAN(n)), CALC_DIV_INT(RX_FREQ_CHAN(n))
#define TX_CHANNEL_ENTRY(n) CALC_DIV_FRAC(TX_FREQ_CHAN(n)), CALC_DIV_INT(TX_FREQ_CHAN(n))

/* LDO Trim Registers and trim values */
#define MXC_R_TX_LDO_TRIM *((uint32_t *)(0x40000454)) // SHR21
#define MXC_R_TX_LDO_TRIM_OTP *((uint32_t *)(0x108001D4))
#define MXC_R_TX_LDO_TRIM_MASK 0xFF
#define MXC_R_RX_LDO_TRIM *((uint32_t *)(0x4000045C)) // SHR23
#define MXC_R_RX_LDO_TRIM_OTP *((uint32_t *)(0x108001DC))
#define MXC_R_RX_LDO_TRIM_MASK 0xFF

#define OFFSET_250K 0x100
#define OFFSET_N250K 0x300

#define PAL_BB_AFE_SIMO_1_1V 0x3C
#define PAL_BB_AFE_SIMO_1_2V 0x44
#define PAL_BB_AFE_SIMO_1_3V 0x50
#define PAL_BB_AFE_SIMO_1_4V 0x59

/*! \brief      Definitions for amplitude measurement function. */
typedef enum {
    MEAS_AMP_HIZ = MXC_V_AFE_REG13_EN_SWING_HIZ,
    MEAS_AMP_PLL_TX_VCO = MXC_V_AFE_REG13_EN_SWING_PLL_TX_VCO,
    MEAS_AMP_PLL_LOBUF = MXC_V_AFE_REG13_EN_SWING_PLL_LOBUF,
    MEAS_AMP_PLL_FM_DAC = MXC_V_AFE_REG13_EN_SWING_PLL_FM_DAC,
    MEAS_AMP_LPF_CM = MXC_V_AFE_REG13_EN_SWING_LPF_CM,
    MEAS_AMP_VTUNE = MXC_V_AFE_REG13_EN_SWING_VTUNE,
    MEAS_AMP_VSS = MXC_V_AFE_REG13_EN_SWING_VSS,
} palBbAfeMeasAmp_t;

/*! \brief      LDO Settings. */
typedef struct {
    int8_t power;
    uint8_t paCoef;
    uint8_t paLsb;
    uint32_t vregod;
    uint32_t ldocr;
    uint32_t txLdoTrim;
    uint32_t rxLdoTrim;
} palBbAfeSettigns_t;

/*! \brief      PalBbAfe control block. */
typedef struct {
    int8_t txPower;

    /* RX and TX channel settings */
    palBbAfeCh_t txCh[NUM_CH];
    palBbAfeCh_t rxCh[NUM_CH];

    /* Calibration channel */
    uint8_t calCh;

    /* LDO settings for RX */
    palBbAfeSettigns_t rxSettings;

    /* Calibrated AFE registers */
    mxc_afe_regs_t afeRegsCal;

} palBbAfeCb_t;

/* Settings for TX */
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
// clang-format off

// Keep settings sorted by power level 
palBbAfeSettigns_t txSettings[] = {
/*  power, paCoef, paLsb,                 vregod, ldocr, txLdoTrim, rxLdoTrim */
    { -15,      2,     3,   PAL_BB_AFE_SIMO_1_1V,  0x55,       0x0,      0x00 },
    { -10,      4,     1,   PAL_BB_AFE_SIMO_1_1V,  0x55,       0x0,      0x00 },
    {  -5,      5,     1,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 },
    {  -2,      8,     0,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 },
    {   0,      8,     3,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 },
    {   2,     10,     0,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 },
    {   4,     15,     3,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 }
    // {   7,     15,     3,   PAL_BB_AFE_SIMO_1_4V,  0xD9,       0x0,      0x16 }
    // { -50,      0,     0,   PAL_BB_AFE_SIMO_1_1V,  0x55,       0x0,      0x00 },
};
// clang-format on
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#define PAL_BB_AFE_TX_POWER_UNDEF (-120)
#define PAL_BB_AFE_TX_POWER_LEVELS (sizeof(txSettings) / sizeof(palBbAfeSettigns_t))

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbAfeCb_t palBbAfeCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
static inline void palBbAfeLockOTP(void)
{
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    /* Write a bogus value to the unlock register to lock the info block */
    MXC_FLC0->actnl = 0x1234;

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*************************************************************************************************/
static inline void palBbAfeUnlockOTP(void)
{
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    /* Write a bogus value to the unlock register to lock the info block */
    palBbAfeLockOTP();

    /* Write the unlock sequence */
    MXC_FLC0->actnl = 0x3a7f5ca3;
    MXC_FLC0->actnl = 0xa1e34f20;
    MXC_FLC0->actnl = 0x9608b2c1;
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

/*************************************************************************************************/
static uint8_t palBbAfeOTPValue(uint32_t addr, int start_bit, uint8_t mask)
{
    return ((*((volatile uint8_t *)addr) >> start_bit) & mask); // NOLINT
}

/*************************************************************************************************/
static uint32_t palBbAfeAbsErr(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

/*************************************************************************************************/
bool_t PalBbAfeSaveCal(void)
{
    int i;
    uint8_t *calRegsPtr = (uint8_t *)&(palBbAfeCb.afeRegsCal.rsv_0x0);

    /* Read the first set of registers, addr 0 is reserved */
    for (i = MXC_R_AFE_REG1; i <= MXC_R_AFE_REG15; i++) {
        if (!PalBbDbbSpiRead(i, (calRegsPtr + i))) {
            return FALSE;
        }
    }

    /* Read the second set of registers, addr 0x10 - 0x20 is reserved */
    for (i = MXC_R_AFE_REGA1; i <= MXC_R_AFE_REGB7; i++) {
        if (!PalBbDbbSpiRead(i, (calRegsPtr + i))) {
            return FALSE;
        }
    }

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeSetChannel(uint8_t channel)
{
    PalBbSeqSetChannel(&palBbAfeCb.rxCh[channel], &palBbAfeCb.txCh[channel]);

    return TRUE;
}

/*************************************************************************************************/
int8_t PalBbAfeGetIndexTxPower(unsigned index)
{
    if (index >= PAL_BB_AFE_TX_POWER_LEVELS) {
        return PAL_BB_AFE_TX_POWER_UNDEF;
    }

    return txSettings[index].power;
}

/*************************************************************************************************/
int8_t PalBbAfeGetTxPower(void)
{
    return palBbAfeCb.txPower;
}

/*************************************************************************************************/
unsigned PalBbAfeGetTxPowerLevels(void)
{
    return PAL_BB_AFE_TX_POWER_LEVELS;
}

/*************************************************************************************************/
static int PalBbAfeGetTxPowerIndex(int8_t txPower)
{
    int i;

    /* Find the matching TX power level index */
    for (i = 0; i < PAL_BB_AFE_TX_POWER_LEVELS; i++) {
        if (txPower == txSettings[i].power) {
            return i;
        }
    }

    return PAL_BB_AFE_TX_POWER_UNDEF;
}

/*************************************************************************************************/
uint8_t PalBbAfeGetTxPowerPa(int8_t txPower)
{
    int i;

    /* Find the matching TX power level index */
    for (i = 0; i < PAL_BB_AFE_TX_POWER_LEVELS; i++) {
        if (txPower == txSettings[i].power) {
            return txSettings[i].paCoef;
        }
    }

    return 0;
}

/*************************************************************************************************/
bool_t PalBbAfeSetTxPower(int8_t txPower)
{
    int index;
    uint8_t reg;

    if (palBbAfeCb.txPower == txPower) {
        return TRUE;
    }

    /* Test the txPower */
    index = PalBbAfeGetTxPowerIndex(txPower);
    if (index == PAL_BB_AFE_TX_POWER_UNDEF) {
        return FALSE;
    }

    /* Update register B4 */
    reg = (txSettings[index].paLsb << MXC_F_AFE_REGB4_PA_LSB_POS);

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    if (txPower < 0) {
        reg |= (0x04 << MXC_F_AFE_REGB4_PA_CON_DT_POS);
    } else if (txPower > 3) {
        reg |= (0x19 << MXC_F_AFE_REGB4_PA_CON_DT_POS);
    } else {
        reg |= (0x1C << MXC_F_AFE_REGB4_PA_CON_DT_POS);
    }
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    PalBbDbbSpiWrite(MXC_R_AFE_REGB4, reg);

    /* Update the PA coefficient */
    PalBbSeqSetPA(txSettings[index].paCoef);

    /* Save the current setting */
    palBbAfeCb.txPower = txPower;

    return TRUE;
}

bool_t PalBbAfeSetTxPowerIndex(unsigned index)
{
    // Check the validity of the index
    if (index >= PAL_BB_AFE_TX_POWER_LEVELS) {
        return FALSE;
    }

    // Set TX power based on the index
    return PalBbAfeSetTxPower(txSettings[index].power);
}

/*************************************************************************************************/
bool_t PalBbAfeTxSetup(void)
{
    /* Get the appropriate power level index */
    int i = PalBbAfeGetTxPowerIndex(palBbAfeCb.txPower);
    if (i == PAL_BB_AFE_TX_POWER_UNDEF) {
        return FALSE;
    }

    /* Setup LDOs and VREGOD for TX */
    MXC_GCR->btle_ldocr = txSettings[i].ldocr;
    MXC_SIMO->vrego_d = txSettings[i].vregod;

    /* Disable RX LDO discharge */
    MXC_GCR->btle_ldocr &= ~MXC_F_GCR_BTLE_LDOCR_LDORXDISCH;

    /* Apply the LDO trim values */
    MXC_R_TX_LDO_TRIM = txSettings[i].txLdoTrim;
    MXC_R_RX_LDO_TRIM = txSettings[i].rxLdoTrim;

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeRxSetup(void)
{
    /* Setup LDOs and VREGOD for RX */
    MXC_GCR->btle_ldocr = palBbAfeCb.rxSettings.ldocr;
    MXC_SIMO->vrego_d = palBbAfeCb.rxSettings.vregod;

    /* Discharge the RX LDO to switch the voltage faster */
    MXC_GCR->btle_ldocr |= MXC_F_GCR_BTLE_LDOCR_LDORXDISCH;

    /* Apply the RX LDO trim values */
    MXC_R_TX_LDO_TRIM = palBbAfeCb.rxSettings.txLdoTrim;
    MXC_R_RX_LDO_TRIM = palBbAfeCb.rxSettings.rxLdoTrim;

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeTxDone(void)
{
    PalBbAfeRxSetup();

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeRxDone(void)
{
    /* Disable RX LDO Discharge */
    /* MXC_GCR->btle_ldocr &= ~MXC_F_GCR_BTLE_LDOCR_LDORXDISCH; */

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeRegsInit(void)
{
    uint8_t reg;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    /* Make sure OTP is unlocked before reading trim values */
    palBbAfeUnlockOTP();

    /* Set appropriate AFE version */
    MXC_DBB_RFFE->rffe_ifc_rffe_version = MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G;
    MXC_DBB_RFFE->general_param = 0;

    /*** REG1 ****/
    reg = (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_CP_EN | MXC_F_AFE_REG1_VCO_EN |
           /* MXC_F_AFE_REG1_LO_BUF_EN | */
           MXC_F_AFE_REG1_DIV_EN |
           /* MXC_F_AFE_REG1_PA_EN | */
           /* MXC_F_AFE_REG1_TX_SW_EN | */
           MXC_F_AFE_REG1_PLL_BIAS_EN | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, reg);

    /*** REG2 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG2, 0);

    /*** REG3 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, 0);

    /*** REG4 ****/
    /* do not use OTP value */
    reg = ((0x1 << MXC_F_AFE_REG4_CP_OFS_POS) | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG4, reg);

    /*** REG5 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG5, 0);

    /*** REG6 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG6, 0);

    /*** REG7 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG7, 0);

    /*** REG8 ****/
    reg = (MXC_S_AFE_REG8_D2S_MUX_LPF |
           /* MXC_F_AFE_REG8_SWAP_IQ | */
           0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG8, reg);

    /*** REG9 ****/
    reg = (
        /* MXC_F_AFE_REG9_LNA_EN | */
        /* MXC_F_AFE_REG9_MX1_EN | */
        /* MXC_F_AFE_REG9_MX2_EN | */
        /* MXC_F_AFE_REG9_LPF_EN | */
        /* MXC_F_AFE_REG9_CLK_ADC_EN | */
        MXC_F_AFE_REG9_D2S_BUF_EN |
        /* MXC_F_AFE_REG9_CON_SW_RW | */
        MXC_F_AFE_REG9_BIAS_EN | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG9, reg);

    /*** REG10 ****/
    reg = (MXC_S_AFE_REG10_LNA_HG_24 | MXC_F_AFE_REG10_MX1_HG | MXC_F_AFE_REG10_MX2_HG |
           MXC_S_AFE_REG10_LPF_GAIN_24DB | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG10, reg);

    /*** REG11 ****/
    static const uint8_t LNA_BC = 0x3;
    static const uint8_t MX1_BW = 0x7;
    reg = ((LNA_BC << MXC_F_AFE_REG11_LNA_BC_POS) | (MX1_BW << MXC_F_AFE_REG11_MX1_BW_POS) |
           MXC_F_AFE_REG11_DC_DAC_EN | MXC_F_AFE_REG11_CLK_DC_DAC | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG11, reg);

    /*** REG12 ****/
    reg = (MXC_S_AFE_REG12_LPF_BW_1_66_BW | MXC_S_AFE_REG12_ADC_SF_8M | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REG12, reg);

    /*** REG13 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG13, 0);

    /*** REG14 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG14, 0);

    /*** REG15 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REG15, 0);

    /*** REGA1 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA1, 0);

    /*** REGA2 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA2, 0);

    /*** REGA3 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA3, 0);

    /*** REGA4 ****/
    reg = (MXC_S_AFE_REGA4_SEL_HFP_FM_EN |
           /* MXC_F_AFE_REGA4_F_DEV | */
           0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGA4, reg);

    /*** REGA5 ****/
    static const uint8_t REGA5 = 0x46;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, REGA5);

    /*** REGA6 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA6, 0);

    /*** REGA7 ****/
    reg = (
        /* (0 << MXC_F_AFE_REGA7_DL_AMP_POS) | */
        (0x4 << MXC_F_AFE_REGA7_AMP_COEF_POS) | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGA7, reg);

    /*** REGA8 ****/
    reg = (MXC_F_AFE_REGA8_DATA_CLK_EN |
           /* MXC_F_AFE_REGA8_SDMA_EN | */
           MXC_S_AFE_REGA8_SEL_AMP_INTERNAL | MXC_S_AFE_REGA8_AM_FIX_AMP_COEF |
           MXC_S_AFE_REGA8_FDEV_D2_2K |
           /* MXC_F_AFE_REGA8_DCOCBY_EN | */
           MXC_S_AFE_REGA8_BLE_24MHZ_32M | MXC_S_AFE_REGA8_INV_FM_POL_NON_INV | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGA8, reg);

    /*** REGA9 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA9, 0);

    /*** REGA10 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGA10, 0);

    /*** REGA11 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1A, 0, 0xFF) : 0xa6;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA11, reg);

    /*** REGA12 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1B, 0, 0xFF) : 0x3e;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA12, reg);

    /*** REGA13 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1C, 0, 0x1F) : 0x17;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA13, reg);

    /*** REGA14 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1D, 0, 0xFF) : 0x66;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA14, reg);

    /*** REGA15 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1E, 0, 0x7F) : 0x30;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA15, reg);

    /*** REGA16 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x1F, 0, 0xFF) : 0x70;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA16, reg);

    /*** REGB1 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x20, 0, 0xFF) : 0x24;
    PalBbDbbSpiWrite(MXC_R_AFE_REGB1, reg);

    /*** REGB2 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGB2, 0);

    /*** REGB3 ****/
    reg = ((0x1E << MXC_F_AFE_REGB3_B_OSC_POS) | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGB3, reg);

    /*** REGB4 ****/
    reg = ((0x1 << MXC_F_AFE_REGB4_PA_LSB_POS) | /* 7 for 0dBm, 1 for -10dBm */
           (0x1F << MXC_F_AFE_REGB4_PA_CON_DT_POS) | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGB4, reg);

    /*** REGB5 ****/
    PalBbDbbSpiWrite(MXC_R_AFE_REGB5, 0);

    /*** REGB6 ****/
    reg = ((0x4 << MXC_F_AFE_REGB6_PA_CP_POS) | 0);

    PalBbDbbSpiWrite(MXC_R_AFE_REGB6, reg);

    /*** REGB7 ****/
    reg = OTP_VALID ? palBbAfeOTPValue(OTP_OFFSET | 0x26, 0, 0xFF) : 0x33;
    PalBbDbbSpiWrite(MXC_R_AFE_REGB7, reg);

    palBbAfeLockOTP();

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeInit(void)
{
    unsigned i;

    memset(&palBbAfeCb, 0, sizeof(palBbAfeCb_t)); // NOLINT

    /* Setup the RX LDO Settings */
    palBbAfeCb.rxSettings.ldocr =
        (MXC_F_GCR_BTLE_LDOCR_LDOTXEN | MXC_S_GCR_BTLE_LDOCR_LDOTXVSEL_0_85 |
         MXC_F_GCR_BTLE_LDOCR_LDORXEN | MXC_S_GCR_BTLE_LDOCR_LDORXVSEL_0_85);

    palBbAfeCb.rxSettings.vregod = PAL_BB_AFE_SIMO_1_3V;

    /* Unlock the info block so we can read the OTP trim settings */
    palBbAfeUnlockOTP();
    palBbAfeCb.rxSettings.txLdoTrim = (MXC_R_TX_LDO_TRIM_OTP & MXC_R_TX_LDO_TRIM_MASK);
    palBbAfeCb.rxSettings.rxLdoTrim = (MXC_R_RX_LDO_TRIM_OTP & MXC_R_RX_LDO_TRIM_MASK);
    palBbAfeLockOTP();

    /* Setup VREGO_D and the LDOs for RX */
    PalBbAfeRxSetup();

    /* Wait for VERGO_D to be ready */
    while (!(MXC_SIMO->buck_out_ready & MXC_F_SIMO_BUCK_OUT_READY_BUCKOUTRDYD)) {}

    /* Initialize channel map */
    for (i = 0; i < NUM_CH; i++) {
        palBbAfeCb.rxCh[i].div_frac = CALC_DIV_FRAC(RX_FREQ_CHAN(i));
        palBbAfeCb.rxCh[i].div_int = CALC_DIV_INT(RX_FREQ_CHAN(i));
        palBbAfeCb.txCh[i].div_frac = CALC_DIV_FRAC(TX_FREQ_CHAN(i));
        palBbAfeCb.txCh[i].div_int = CALC_DIV_INT(TX_FREQ_CHAN(i));

        // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

        if (i < 6) {
            palBbAfeCb.rxCh[i].b_osc = 18;
        } else if (i < 20) {
            palBbAfeCb.rxCh[i].b_osc = 19;
        } else if (i < 34) {
            palBbAfeCb.rxCh[i].b_osc = 20;
        } else {
            palBbAfeCb.rxCh[i].b_osc = 21;
        }

        if (i < 2) {
            palBbAfeCb.txCh[i].b_osc = 51;
        } else if (i < 11) {
            palBbAfeCb.txCh[i].b_osc = 52;
        } else if (i < 21) {
            palBbAfeCb.txCh[i].b_osc = 53;
        } else if (i < 33) {
            palBbAfeCb.txCh[i].b_osc = 54;
        } else {
            palBbAfeCb.txCh[i].b_osc = 55;
        }

        // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }

    palBbAfeCb.calCh = INITIAL_CAL_CH;

    return TRUE;
}

/*************************************************************************************************/
static void palBbAfeCalSetCh(unsigned ch, uint8_t dir)
{
    uint8_t data, REGA1_orig, REGA4_orig;
    palBbAfeCh_t *chCfg;

    if (dir == DIR_TX) {
        chCfg = &palBbAfeCb.txCh[ch];
    } else {
        chCfg = &palBbAfeCb.rxCh[ch];
    }

    /* B_OSC */
    data = chCfg->b_osc;
    if (dir == DIR_TX) {
        data |= MXC_F_AFE_REGB3_TX_RXB;
    } else {
        data &= ~(MXC_F_AFE_REGB3_TX_RXB);
    }

    PalBbDbbSpiWrite(MXC_R_AFE_REGB3, data);

    /* DIV_FRAC */
    PalBbDbbSpiRead(MXC_R_AFE_REGA1, &REGA1_orig);
    data = (REGA1_orig & ~(MXC_F_AFE_REGA1_DIV_FRAC)) |
           (chCfg->div_frac << MXC_F_AFE_REGA1_DIV_FRAC_POS);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA1, data);

    data = chCfg->div_frac >> 4; // NOLINT
    PalBbDbbSpiWrite(MXC_R_AFE_REGA2, data);

    data = (chCfg->div_frac >> 12) | (chCfg->div_int << MXC_F_AFE_REGA3_DIV_INT_POS); // NOLINT
    PalBbDbbSpiWrite(MXC_R_AFE_REGA3, data);

    /* IB_LOBUF:IB_VCO */
    data = (chCfg->ib_lobuf_dac << 4) | chCfg->ib_vco_dac; // NOLINT
    PalBbDbbSpiWrite(MXC_R_AFE_REGA12, data);

    /* FDEV */
    PalBbDbbSpiRead(MXC_R_AFE_REGA4, &REGA4_orig);
    MXC_SETFIELD(REGA4_orig, MXC_F_AFE_REGA4_F_DEV,
                 ((chCfg->f_dev & 1) << MXC_F_AFE_REGA4_F_DEV_POS));
    PalBbDbbSpiWrite(MXC_R_AFE_REGA4, REGA4_orig);

    data = chCfg->f_dev >> 1;
    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, data);
}

/*************************************************************************************************/
static bool_t palBbAfeCalIqSample(uint16_t *sample)
{
    unsigned toIter;

    /* Set the timeout count */
    toIter = IQ_CAPTURE_TO_ITERS;

    /* Initiate the sample procedure */
    MXC_DBB_RFFE->iq_ctrl = MXC_F_DBB_RFFE_IQ_CTRL_ENABLE;

    while (toIter--) {
        if (MXC_DBB_RFFE->iq_ctrl & MXC_F_DBB_RFFE_IQ_CTRL_DONE) {
            break;
        }
    }

    /* Timeout waiting for IQ sample */
    if (!toIter) {
        *sample = 0;
        return FALSE;
    }

    MXC_DBB_RFFE->iq_ctrl = 0;

    /* Save the sample */
    *sample = MXC_DBB_RFFE->iq_data;

    return TRUE;
}

/*************************************************************************************************/
static uint16_t palBbAfeCalGetIq(uint16_t margin, uint16_t *sample)
{
    unsigned toIter;
    uint16_t iq0, iq1;

    toIter = IQ_CAPTURE_TO_ITERS;
    while (toIter--) {
        if (!palBbAfeCalIqSample(&iq0)) {
            *sample = 0;
            return FALSE;
        }
        if (!palBbAfeCalIqSample(&iq1)) {
            *sample = 0;
            return FALSE;
        }

        if (palBbAfeAbsErr(iq0, iq1) <= margin) {
            *sample = iq0;
            return TRUE;
        }
    }

    *sample = 0;
    return FALSE;
}

/*************************************************************************************************/
static uint16_t palBbAfeCalMeasAmp(palBbAfeMeasAmp_t sel)
{
    uint16_t sample;

    /* Write the swing selection to Reg13 */
    PalBbDbbSpiWrite(MXC_R_AFE_REG13,
                     (0x1 << MXC_F_AFE_REG13_IB_OP_POS) | (sel << MXC_F_AFE_REG13_EN_SWING_POS) |
                         MXC_S_AFE_REG13_MON_SW_SW_ADC | MXC_F_AFE_REG13_SW_ADC_EN);

    /* Enable the ADC clock disable D2S_BUF */
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, MXC_F_AFE_REG9_BIAS_EN | MXC_F_AFE_REG9_CLK_ADC_EN);

    /* TODO: Timeout error here */
    static const uint16_t margin = 8;
    palBbAfeCalGetIq(margin, &sample);

    return sample;
}

/*************************************************************************************************/
static uint32_t palBbAfeCalMeasFreq(uint8_t freqResolution)
{
    uint32_t fmeas, hf_count, cycles;
    uint16_t iqSample;
    uint8_t REG9_orig;
    uint8_t res_mask = (freqResolution << MXC_F_AFE_REG14_FM_RES_POS);
    static const uint32_t settlementDelay = 150;

    PalBbDbbSpiRead(MXC_R_AFE_REG9, &REG9_orig);

    PalBbDbbSpiWrite(MXC_R_AFE_REG9, (REG9_orig | MXC_F_AFE_REG9_CLK_ADC_EN));
    PalBbDbbSpiWrite(MXC_R_AFE_REG13, MXC_S_AFE_REG13_MON_SW_FM_CNT);
    PalBbDbbSpiWrite(MXC_R_AFE_REG14, 0x0 | res_mask);

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, MXC_F_AFE_REG14_FM_CAL_EN | res_mask);

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, MXC_F_AFE_REG14_FM_CAL_EN | res_mask |
                                          MXC_F_AFE_REG14_START_FM_CNT |
                                          (0x5 << MXC_F_AFE_REG14_FM_CNT_ADDR_POS)); // NOLINT
    /* Wait for settlement */
    PalBbDbbDelay(settlementDelay);

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, MXC_F_AFE_REG14_FM_CAL_EN | MXC_F_AFE_REG14_FM_RES |
                                          MXC_F_AFE_REG14_START_FM_CNT |
                                          (0x0 << MXC_F_AFE_REG14_FM_CNT_ADDR_POS));
    /* Wait for settlement */
    PalBbDbbDelay(settlementDelay);

    palBbAfeCalGetIq(0, &iqSample);
    hf_count = iqSample;

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, MXC_F_AFE_REG14_FM_CAL_EN | MXC_F_AFE_REG14_FM_RES |
                                          MXC_F_AFE_REG14_START_FM_CNT |
                                          (0x1 << MXC_F_AFE_REG14_FM_CNT_ADDR_POS));
    palBbAfeCalGetIq(0, &iqSample);
    hf_count += (iqSample << 9); // NOLINT

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, MXC_F_AFE_REG14_FM_CAL_EN | MXC_F_AFE_REG14_FM_RES |
                                          MXC_F_AFE_REG14_START_FM_CNT |
                                          (0x2 << MXC_F_AFE_REG14_FM_CNT_ADDR_POS));
    palBbAfeCalGetIq(0, &iqSample);
    hf_count += (iqSample << 18); // NOLINT

    PalBbDbbSpiWrite(MXC_R_AFE_REG14, 0x00);
    PalBbDbbSpiWrite(MXC_R_AFE_REG13, 0x00);

    cycles = (1024 << freqResolution) - 256; // NOLINT
    fmeas = ((uint64_t)F_XTAL * hf_count) / cycles;

    PalBbDbbSpiWrite(MXC_R_AFE_REG9, REG9_orig);

    return fmeas;
}

/*************************************************************************************************/
static void palBbAfeAmpSearch(uint8_t addr, palBbAfeMeasAmp_t amp_sel, unsigned target_amp)
{
    uint8_t data_masked, data;
    unsigned best_error, this_trim, best_trim, this_amp, this_error;
    unsigned i;
    static const unsigned nbits = 7;
    static const unsigned nshift = 0;

    PalBbDbbSpiRead(addr, &data);

    data_masked = data & 0x80; // NOLINT
    best_error = 500; // NOLINT large starting value for best_error
    best_trim = 0;
    this_trim = 1 << (nbits + nshift - 1);

    for (i = 0; i < nbits; i++) {
        data = data_masked + this_trim;
        PalBbDbbSpiWrite(addr, data);

        this_amp = palBbAfeCalMeasAmp(amp_sel);
        this_error = palBbAfeAbsErr(this_amp, target_amp);

        if (this_error < best_error) {
            best_error = this_error;
            best_trim = this_trim;
        }

        if (i < (nbits - 1)) {
            unsigned temp = 1 << (nbits + nshift - 2 - i);
            this_trim = (this_amp > target_amp) ? (this_trim - temp) : (this_trim + temp);
        }
    }

    data = data_masked + best_trim;
    PalBbDbbSpiWrite(addr, data);
}

/*************************************************************************************************/
static void palBbAfeCalAmp(uint8_t dir, unsigned vcoTarget, unsigned lobufTarget, uint8_t *vco,
                           uint8_t *lobuf)
{
    uint8_t data, REG1_orig, REG9_orig, REG13_orig;

    /* capture the original register settings */
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG9, &REG9_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG13, &REG13_orig);

    /* turn the PLL on */
    /* turning off the PA for TX and RX so the output isn't visible during
     * calibration */
    /* TODO: Remove the turning off of PA? */
    if (dir == DIR_TX) {
        PalBbDbbSpiWrite(MXC_R_AFE_REG1, (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_VCO_EN |
                                          MXC_F_AFE_REG1_DIV_EN | MXC_F_AFE_REG1_PLL_BIAS_EN));
    } else {
        PalBbDbbSpiWrite(MXC_R_AFE_REG1,
                         (MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_VCO_EN | MXC_F_AFE_REG1_DIV_EN |
                          MXC_F_AFE_REG1_PLL_BIAS_EN | MXC_F_AFE_REG1_LO_BUF_EN));
    }

    /* Turn off the Output Buffer in the signal path (D2S_BUF_EN REG9 bit 6) */
    /* Turn on CLK_ADC_EN */
    /* The buffer in the test path is turned on when sel~000 */
    data = (REG9_orig & ~(MXC_F_AFE_REG9_CON_SW_RW)) | MXC_F_AFE_REG9_CLK_ADC_EN;
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, data);

    if (dir == DIR_RX) {
        palBbAfeAmpSearch(MXC_R_AFE_REGA12, MEAS_AMP_PLL_TX_VCO, vcoTarget);
        palBbAfeAmpSearch(MXC_R_AFE_REGA15, MEAS_AMP_PLL_LOBUF, lobufTarget);
        palBbAfeAmpSearch(MXC_R_AFE_REGA12, MEAS_AMP_PLL_TX_VCO, vcoTarget);

        PalBbDbbSpiRead(MXC_R_AFE_REGA12, &data);
        *vco = (data & 0x7F); // NOLINT

        PalBbDbbSpiRead(MXC_R_AFE_REGA15, &data);
        *lobuf = (data & 0x7F); // NOLINT

    } else {
        palBbAfeAmpSearch(MXC_R_AFE_REGA12, MEAS_AMP_PLL_TX_VCO, vcoTarget);

        PalBbDbbSpiRead(MXC_R_AFE_REGA12, &data);
        *vco = (data & 0x7F); // NOLINT
    }

    /* Restore original settings */
    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, REG9_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG13, REG13_orig);
}

/*************************************************************************************************/
static void palBbAfeCalPaEnable(void)
{
    uint8_t REG1_orig, REG2_orig, REG9_orig, REGA7_orig;

    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG2, &REG2_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG9, &REG9_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REGA7, &REGA7_orig);

    REG1_orig |= (MXC_F_AFE_REG1_PA_EN | MXC_F_AFE_REG1_TX_SW_EN);
    REG2_orig |= MXC_F_AFE_REG2_CLK_PA_EN;
    REG9_orig &= ~(MXC_F_AFE_REG9_CON_SW_RW);
    REGA7_orig = ((REGA7_orig & ~(MXC_F_AFE_REGA7_AMP_COEF)) |
                  (PalBbAfeGetTxPowerPa(palBbAfeCb.txPower) << MXC_F_AFE_REGA7_AMP_COEF_POS));

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG2, REG2_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, REG9_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA7, REGA7_orig);
}

/*************************************************************************************************/
static void palBbAfeCalPaDisable(void)
{
    uint8_t REG1_orig, REG2_orig;

    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG2, &REG2_orig);

    REG1_orig &= ~(MXC_F_AFE_REG1_PA_EN | MXC_F_AFE_REG1_TX_SW_EN);
    REG2_orig &= ~(MXC_F_AFE_REG2_CLK_PA_EN);

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG2, REG2_orig);
}

/*************************************************************************************************/
static uint8_t palBbAfeCalSelectBosc(uint32_t freq, unsigned startBosc, uint32_t *fmeas)
{
    unsigned b = startBosc;
    uint32_t min_err = palBbAfeAbsErr(fmeas[b], freq);
    uint8_t b_osc = b;

    for (b = (startBosc + 1); b < B_OSC_STEPS; b++) {
        uint32_t err = palBbAfeAbsErr(fmeas[b], freq);
        if (err < min_err) {
            min_err = err;
            b_osc = b;
        }
    }

    return b_osc;
}

/*************************************************************************************************/
// NOLINTNEXTLINE
static void palBbAfeCalFillChMap(uint8_t dir, uint32_t *fmeas, uint8_t *vco, uint8_t *lobuf)
{
    int ch;
    int startb = 0;

    for (ch = 0; ch < NUM_CH; ch++) {
        uint32_t freq;
        if (dir == DIR_RX) {
            freq = RX_FREQ_CHAN(ch);
        } else {
            freq = TX_FREQ_CHAN(ch);
        }

        uint8_t b_osc = palBbAfeCalSelectBosc(freq, startb, fmeas);

        if (dir == DIR_RX) {
            palBbAfeCb.rxCh[ch].b_osc = b_osc;
        } else {
            palBbAfeCb.txCh[ch].b_osc = b_osc;
        }

        if ((vco != NULL) && (lobuf != NULL)) {
            if (dir == DIR_RX) {
                palBbAfeCb.rxCh[ch].ib_lobuf_dac = lobuf[b_osc];
                palBbAfeCb.rxCh[ch].ib_vco_dac = vco[b_osc];
            } else {
                /* LOBUF is not used for TX */
                palBbAfeCb.txCh[ch].ib_lobuf_dac = 0;
                palBbAfeCb.txCh[ch].ib_vco_dac = vco[b_osc];
            }
        }
    }
}

/*************************************************************************************************/
static void palBbAfeCalChMap(bool_t initialSweep, unsigned vcoTarget, unsigned lobufTarget)
{
    unsigned bosc, startBosc, endBosc;
    uint8_t data;
    uint8_t freqResolution;
    uint8_t dir = DIR_RX;
    uint8_t REG1_orig, REG3_orig, REG9_orig, REG_B3_orig;

    uint8_t vco[B_OSC_STEPS];
    uint8_t lobuf[B_OSC_STEPS];
    uint32_t fmeas[B_OSC_STEPS];

    /* capture the original register settings */
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG3, &REG3_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG9, &REG9_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REGB3, &REG_B3_orig);

    /* The initial sweep is a full sweep of all RX or TX B_OSC settings. */
    /* The final sweep only looks at the B_OSC settings required to cover the */
    /* channels +/-1 code. */
    if (initialSweep) {
        startBosc = 0;
        endBosc = B_OSC_STEPS - 1;
        freqResolution = 1;
    } else {
        startBosc = palBbAfeCb.rxCh[0].b_osc - 1;
        endBosc = palBbAfeCb.txCh[NUM_CH - 1].b_osc + 1;
        freqResolution = 2;
    }

    memset(fmeas, 0, sizeof(fmeas)); // NOLINT

    /* Open the loop and apply the UGB to vco tune */
    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig & ~(MXC_F_AFE_REG1_CP_EN));
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig | MXC_F_AFE_REG3_LF_UGB_EN); /* set LF_UGB_EN */

    for (bosc = startBosc; bosc <= endBosc; bosc++) {
        data = (REG_B3_orig & MXC_F_AFE_REGB3_TX_RXB) | bosc;

        if (dir == DIR_TX) {
            data |= MXC_F_AFE_REGB3_TX_RXB;
        } else {
            data &= ~(MXC_F_AFE_REGB3_TX_RXB);
        }

        PalBbDbbSpiWrite(MXC_R_AFE_REGB3, data);

        if (!initialSweep) {
            palBbAfeCalAmp(dir, vcoTarget, lobufTarget, &vco[bosc], &lobuf[bosc]);
        }

        palBbAfeCalMeasAmp((dir == DIR_TX) ? MEAS_AMP_PLL_TX_VCO : MEAS_AMP_PLL_LOBUF);

        fmeas[bosc] = palBbAfeCalMeasFreq(freqResolution);

        // TODO: Find a better way to make this conditional
        static const uint32_t RX_TX_SWITCH_FREQ = 2100000000;
        if (fmeas[bosc] > RX_TX_SWITCH_FREQ) {
            if (dir == DIR_RX && !initialSweep) {
                palBbAfeCalPaEnable();
            }
            dir = DIR_TX;
        }
    }

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, REG9_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REGB3, REG_B3_orig);

    /* Determine the Channel Map */
    if (initialSweep) {
        palBbAfeCalFillChMap(DIR_RX, fmeas, NULL, NULL);
        palBbAfeCalFillChMap(DIR_TX, fmeas, NULL, NULL);
    } else {
        palBbAfeCalFillChMap(DIR_RX, fmeas, vco, lobuf);
        palBbAfeCalFillChMap(DIR_TX, fmeas, vco, lobuf);
    }

    palBbAfeCalPaDisable();
}

/*************************************************************************************************/
static unsigned palBbAfeCalGetDF(uint16_t trim, uint8_t REGA4_orig)
{
    unsigned k;
    int32_t dF_sum = 0;
    static const int32_t dFHigh = 1200000;

    /* calculate the data and write REGA5 and REGA4 based on this trim */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, trim >> 1); /* write FDEV<8:1> */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA4,
                     (REGA4_orig & ~MXC_F_AFE_REGA4_F_DEV) | (trim << MXC_F_AFE_REGA4_F_DEV_POS));

    for (k = 0; k < FM_GAIN_ITERS; k++) {
        int32_t FmeasHigh, FmeasLow;
        int32_t dF;

        do {
            /* Set constant output for +250kHz offset */
            MXC_DBB_TX->tx_phy_const_output_freq_high = OFFSET_250K;
            FmeasHigh = (int32_t)palBbAfeCalMeasFreq(0x3);

            /* Set constant output for -250kHz offset */
            MXC_DBB_TX->tx_phy_const_output_freq_high = OFFSET_N250K;
            FmeasLow = (int32_t)palBbAfeCalMeasFreq(0x3);

            dF = FmeasHigh - FmeasLow;
        } while ((dF < 0) || (dF > dFHigh));

        dF_sum += dF;
    }

    return dF_sum / FM_GAIN_ITERS;
}

/*************************************************************************************************/
static uint16_t palBbAfeCalFmGainCh(uint32_t dfIdeal, unsigned ch)
{
    unsigned x1, x2, dF1, dF2, m, b, x3;
    uint8_t REGA4_orig;
    PalBbDbbSpiRead(MXC_R_AFE_REGA4, &REGA4_orig);

    palBbAfeCalSetCh(ch, DIR_TX);

    /* Delay 10 us for settlement */
    static const uint32_t settlementDelay = 10;
    PalBbDbbDelay(settlementDelay);

    palBbAfeCalPaEnable();

    x1 = (0x20 << 1); /* NOLINT Low search limit */
    /* TODO: Actual value for ME14 should be around 0x40 */
    x2 = (0x60 << 1); /* NOLINT High search limit */

    dF1 = palBbAfeCalGetDF(x1, REGA4_orig);

    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, 0); /* write FDEV<8:1> */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA4, REGA4_orig & ~(MXC_F_AFE_REGA4_F_DEV)); /* write FDEV<0> */

    dF2 = palBbAfeCalGetDF(x2, REGA4_orig);

    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, 0); /* write FDEV<8:1> */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA4, REGA4_orig & ~(MXC_F_AFE_REGA4_F_DEV)); /* write FDEV<0> */

    m = (dF2 - dF1) / (x2 - x1);
    b = dF1 - x1 * m;
    x3 = (dfIdeal - b) / m;

    palBbAfeCalPaDisable();

    return x3;
}

/*************************************************************************************************/
static void palBbAfeCalFmGain(uint32_t target, uint8_t phy)
{
    uint16_t trim0, trim19, trim39, pattern_gen;
    int ch;
    uint32_t diff0, diff1;
    uint8_t REG1_orig, REG2_orig, REG3_orig, REG9_orig, REGA4_orig, REGA8_orig;

    /* capture the original register settings */
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG2, &REG2_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG3, &REG3_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG9, &REG9_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REGA4, &REGA4_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REGA8, &REGA8_orig);

    /* turn on Bias DIV VCO FM_DAC */
    PalBbDbbSpiWrite(MXC_R_AFE_REG1, MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_VCO_EN |
                                         MXC_F_AFE_REG1_DIV_EN | MXC_F_AFE_REG1_PLL_BIAS_EN);

    /* turn on CLK_DAC_EN */
    PalBbDbbSpiWrite(MXC_R_AFE_REG2, REG2_orig | MXC_F_AFE_REG2_CLK_DAC_EN);

    /* turn on UGB */
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, (REG3_orig | MXC_F_AFE_REG3_LF_UGB_EN));

    /* turn on BIAS_EN CLK_ADC_EN */
    PalBbDbbSpiWrite(MXC_R_AFE_REG9,
                     REG9_orig | MXC_F_AFE_REG9_CLK_ADC_EN | MXC_F_AFE_REG9_BIAS_EN);

    /* clear FDEV_D2 */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA8, REGA8_orig & ~(MXC_F_AFE_REGA8_FDEV_D2));

    /* TODO: Ideally this would work with this bit set, but the measurement does
     * not finish */
    /* if(high_speed) { */
    /* Workaound by halving the target frequency */
    /* PalBbDbbSpiWrite(MXC_R_AFE_REGA8, REGA8_orig | MXC_F_AFE_REGA8_FDEV_D2); */
    /* target /= 2; */
    /* } */

    /* Turn off RX PMU, interferes with this routine */
    MXC_DBB_CTRL->rx_pmu_ctrl = MXC_F_DBB_CTRL_RX_PMU_CTRL_SLEEP;

    /* Set constant output for +250kHz offset */
    MXC_DBB_TX->tx_phy_const_output_freq_high = OFFSET_250K;
    MXC_DBB_TX->tx_phy_const_output_enable = 1;

    pattern_gen = MXC_DBB_TX->tx_phy_pattern_gen;
    MXC_DBB_TX->tx_phy_pattern_gen =
        (MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_EN | MXC_F_DBB_TX_TX_PHY_PATTERN_GEN_RST |
         MXC_S_DBB_TX_TX_PHY_PATTERN_GEN_MODE_USER);

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    trim0 = palBbAfeCalFmGainCh(target, 0) << 6;
    trim19 = palBbAfeCalFmGainCh(target, 19) << 6;
    trim39 = palBbAfeCalFmGainCh(target, 39) << 6;

    /* Calculate a linear fit through three averages */
    diff0 = (trim0 - trim19) / 19;
    diff1 = (trim19 - trim39) / 20;

    if (phy != BB_PHY_BLE_2M) {
        /* Apply the linear fit for the first 18 channels */
        for (ch = 0; ch < 19; ch++) { palBbAfeCb.txCh[ch].f_dev = (trim0 - ch * diff0) >> 6; }

        /* Apply the linear fit for the remaining channels */
        for (ch = 0; ch <= 20; ch++) {
            palBbAfeCb.txCh[ch + 19].f_dev = (trim19 - ch * diff1) >> 6;
        }
    } else {
        /* Apply the linear fit for the first 18 channels */
        for (ch = 0; ch < 19; ch++) { palBbAfeCb.txCh[ch].f_dev_2M = (trim0 - ch * diff0) >> 7; }

        /* Apply the linear fit for the remaining channels */
        for (ch = 0; ch <= 20; ch++) {
            palBbAfeCb.txCh[ch + 19].f_dev_2M = (trim19 - ch * diff1) >> 7;
        }
    }

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    MXC_DBB_TX->tx_phy_pattern_gen = pattern_gen;
    MXC_DBB_TX->tx_phy_const_output_enable = 0;

    /* put the registers back to the original settings along with best trim
     * setting */
    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG2, REG2_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG9, REG9_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA4, REGA4_orig & ~(MXC_F_AFE_REGA4_F_DEV));
    PalBbDbbSpiWrite(MXC_R_AFE_REGA5, 0);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA8, REGA8_orig);
}

/*************************************************************************************************/
static void palBbAfeCalLdo(void)
{
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    int i, min_df_index, loop_count, retry_high;
    uint32_t last_freq, freq, deltaf, best_deltaf;
    static const uint32_t freq_target = 60000; /* May have to increase/decrase */

    uint8_t REG1_orig, REG3_orig, REGA7_orig;

    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG3, &REG3_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REGA7, &REGA7_orig);

    /* set output power to highest level for this calibration */
    PalBbDbbSpiWrite(MXC_R_AFE_REGA7, MXC_F_AFE_REGA7_AMP_COEF);

    /* Open the loop and apply the UGB to vco tune */
    /* TX_SW_EN=1 PA_EN=1 LO_BUF_EN=0 CP_EN=0 */
    PalBbDbbSpiWrite(MXC_R_AFE_REG1, MXC_F_AFE_REG1_FM_DAC_EN | MXC_F_AFE_REG1_VCO_EN |
                                         MXC_F_AFE_REG1_DIV_EN | MXC_F_AFE_REG1_PA_EN |
                                         MXC_F_AFE_REG1_TX_SW_EN | MXC_F_AFE_REG1_PLL_BIAS_EN);

    /* set LF_UGB_EN */
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, (REG3_orig | MXC_F_AFE_REG3_LF_UGB_EN));

    /* Initialize LDOs */
    // TODO: Do these need to match the given power levels?
    /* -10 dBm */
    if (PalBbAfeGetTxPower() <= -10) {
        MXC_GCR->btle_ldocr = 0x55; /* TX 0.9 RX 0.9 */
        MXC_R_TX_LDO_TRIM = 0x0; /* TX 2'complement */
        MXC_R_RX_LDO_TRIM = 0x05; /* RX 2'complement */
        palBbAfeCb.calCh = 28;
    } else if (PalBbAfeGetTxPowerPa(palBbAfeCb.txPower) <= 0) {
        /* 0 dBm */
        MXC_GCR->btle_ldocr = 0x9D; /* TX 1.0 RX 1.1 */
        MXC_R_TX_LDO_TRIM = 0x0; /* TX 2'complement */
        MXC_R_RX_LDO_TRIM = 0x12; /* RX 2'complement */
        palBbAfeCb.calCh = 19;
    } else {
        /* 4 dBm */
        MXC_GCR->btle_ldocr = 0xD9; /* TX 1.1 RX 1.0 */
        MXC_R_TX_LDO_TRIM = 0; /* TX */
        MXC_R_RX_LDO_TRIM = 0x16; /* RX 12 bits with VSEL at 10b Will have to calculate based on
                                     btle_ldocr */
        palBbAfeCb.calCh = 3;
    }

    loop_count = 1;
    retry_high = 0;
    best_deltaf = 100000000;
    while ((best_deltaf > freq_target) && (retry_high < 4)) {
        /* Wait 100us for LDO settings to settle */
        PalBbDbbDelay(100);

        if (loop_count > 1) {
            /* if min_df_index > 10 and best_delta_f is still too high then
             * it ran out of range so increase RX LDO
             * it's possible this could bounce back and fourth so check elsewhere
             * for retry_high=3 and retry_low=3 and force a higher RX LDO voltage
             * If (min_df_index < 16) Then 'min_df_index is signed
             */

            retry_high++;
            if (retry_high == 1) {
                MXC_R_RX_LDO_TRIM = 0x19;
            } else if (retry_high == 2) {
                MXC_R_RX_LDO_TRIM = 0x1F;
            } else if (retry_high == 3) {
                MXC_R_RX_LDO_TRIM = 0x05;
            } else {
                MXC_R_RX_LDO_TRIM = 0x0B;
            }
        }

        best_deltaf = 100000000;
        min_df_index = 0;
        freq = 0;
        for (i = 0; i < 11; i++) {
            /* take care of signed trim and sweep from lowest LDO voltage (trim=16) */
            /* to highest LDO voltage (trim=15) */

            MXC_R_TX_LDO_TRIM = i;

            /* Wait 100us when chaing RX LDO settings to settle */
            PalBbDbbDelay(100);

            last_freq = freq;
            freq = palBbAfeCalMeasFreq(0x3);

            if (i > 0) {
                /* calculate delta_f and compare to the lowest delta_f */
                deltaf = palBbAfeAbsErr(freq, last_freq);

                if (deltaf < best_deltaf) {
                    best_deltaf = deltaf;
                    min_df_index = MXC_R_TX_LDO_TRIM;
                }
            }
        }

        /* set the final LDO value */
        MXC_R_TX_LDO_TRIM = min_df_index;
        loop_count++;
    }

    /* If we failed to converge, restore the original settings from OTP */
    if (best_deltaf > freq_target) {
        palBbAfeUnlockOTP();
        MXC_R_TX_LDO_TRIM = (MXC_R_TX_LDO_TRIM_OTP & 0xFF);
        MXC_R_RX_LDO_TRIM = (MXC_R_RX_LDO_TRIM_OTP & 0xFF);
        palBbAfeLockOTP();
    }

    /* TODO: Restore trim settings if we didn't find a match */

    /* if the PA trim stoped at max and RX LDO=0.9V it means there wasn't enough trim range
     * to find 0 delta-f/vdd slope so increase the RX LDO range
     */
    /* Increasing RX LDO range causing failure with receiver */

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA7, REGA7_orig);

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

//******************************************************************************
static uint32_t palBbAfeCalcDeltaF(unsigned ch, int8_t power)
{
    uint32_t paOnFreq, paOffFreq, deltaF;
    uint8_t REG1_orig, REG3_orig;
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig);
    PalBbDbbSpiRead(MXC_R_AFE_REG3, &REG3_orig);

    PalBbAfeSetTxPower(power);

    palBbAfeCalPaEnable();

    palBbAfeCalSetCh(ch, DIR_TX);

    // Open the loop and apply the UGB to vco tune
    uint8_t REG1_orig_PAON;
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig_PAON);

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig_PAON & ~(MXC_F_AFE_REG1_CP_EN)); // clear CP_EN
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig | MXC_F_AFE_REG3_LF_UGB_EN); // set LF_UGB_EN

    /* Delay 100 us for settlement */
    static const uint32_t settlementDelay = 100;
    PalBbDbbDelay(settlementDelay);

    paOnFreq = palBbAfeCalMeasFreq(0x3);

    // TURN OFF PA with PLL OPEN Loop
    uint8_t REG1_orig_PAOPEN;
    PalBbDbbSpiRead(MXC_R_AFE_REG1, &REG1_orig_PAOPEN);

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig_PAOPEN & ~(MXC_F_AFE_REG1_PA_EN));

    /* Delay 100 us for settlement */
    PalBbDbbDelay(settlementDelay);

    paOffFreq = palBbAfeCalMeasFreq(0x3);

    deltaF = paOnFreq - paOffFreq;
    deltaF = (deltaF * 7) / 10; // NOLINT Scale deltaF value

    PalBbDbbSpiWrite(MXC_R_AFE_REG1, REG1_orig); // clear CP_EN
    PalBbDbbSpiWrite(MXC_R_AFE_REG3, REG3_orig); // set LF_UGB_EN

    return deltaF;
}

//******************************************************************************
void palBbAfeCalcFullDeltaF(void)
{
    unsigned i;
    uint32_t patargetfreq, deltaF_full;
    static const unsigned chBreak = 12;
    static const unsigned calCh = 19;

    deltaF_full = palBbAfeCalcDeltaF(calCh, 4);
    for (i = 0; i < NUM_CH; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_4 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_4 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = palBbAfeCalcDeltaF(calCh, 2);
    for (i = chBreak; i < NUM_CH; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_2 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_2 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = (deltaF_full * 7) / 10; // NOLINT Scale previous value
    for (i = 0; i < chBreak; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_2 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_2 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = palBbAfeCalcDeltaF(calCh, 0);
    for (i = chBreak; i < NUM_CH; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_0 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_0 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = (deltaF_full * 7) / 10; // NOLINT Scale previous value
    for (i = 0; i < chBreak; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_0 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_0 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = palBbAfeCalcDeltaF(calCh, -2);
    for (i = chBreak; i < NUM_CH; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_n2 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_n2 = CALC_DIV_FRAC(patargetfreq);
    }

    deltaF_full = (deltaF_full * 7) / 10; // NOLINT Scale previous value
    for (i = 0; i < chBreak; i++) {
        patargetfreq = TX_FREQ_CHAN(i) - deltaF_full;
        palBbAfeCb.txCh[i].div_int_n2 = CALC_DIV_INT(patargetfreq);
        palBbAfeCb.txCh[i].div_frac_n2 = CALC_DIV_FRAC(patargetfreq);
    }
}

/*************************************************************************************************/
bool_t PalBbAfeCalibrate(void)
{
    unsigned i;
    uint8_t vco, lobuf;

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    MXC_DBB_CTRL->cmu_gating_on = 0;

    /* Initialize channel map with default values */
    palBbAfeCb.rxCh[palBbAfeCb.calCh].b_osc = 16;
    palBbAfeCb.rxCh[palBbAfeCb.calCh].ib_vco_dac = 8;
    palBbAfeCb.rxCh[palBbAfeCb.calCh].ib_lobuf_dac = 5;

    palBbAfeCb.txCh[palBbAfeCb.calCh].b_osc = 48;
    palBbAfeCb.txCh[palBbAfeCb.calCh].ib_vco_dac = 8;
    palBbAfeCb.txCh[palBbAfeCb.calCh].f_dev = 0xA0;

    /* Initialize AFE with starting points */
    palBbAfeCalSetCh(palBbAfeCb.calCh, DIR_RX);

    /* Set initial amplitude for B_OSC sweep. Cal amp at B_OSC=0 */
    PalBbDbbSpiWrite(MXC_R_AFE_REGB3, 0);

    palBbAfeCalAmp(DIR_RX, 310 /* VCO Target */, 300 /* LOBUF Target */, &vco, &lobuf);

    for (i = 0; i < NUM_CH; i++) {
        palBbAfeCb.rxCh[i].ib_lobuf_dac = lobuf;
        palBbAfeCb.rxCh[i].ib_vco_dac = vco;

        palBbAfeCb.txCh[i].ib_vco_dac = vco;
    }

    /* Generate the initial channel map */
    palBbAfeCalChMap(TRUE /*initialSweep*/, 310 /*vcoTarget*/, 300 /*lobufTarget */);

    palBbAfeCalSetCh(palBbAfeCb.calCh, DIR_TX);

    /* Calibrate amplitude */
    palBbAfeCalAmp(DIR_RX, 330 /* VCO Target */, 300 /* LOBUF Target */, &vco, &lobuf);

    palBbAfeCalLdo();

    /* Set final amplitude */
    palBbAfeCalChMap(FALSE /*initialSweep*/, 330 /*vcoTarget*/, 310 /*lobufTarget */);

    palBbAfeCalSetCh(palBbAfeCb.calCh, DIR_TX);

    /* Set FDEV */
    palBbAfeCalFmGain(520000, BB_PHY_BLE_1M);
    palBbAfeCalFmGain(1040000, BB_PHY_BLE_2M);

    palBbAfeCalSetCh(palBbAfeCb.calCh, DIR_TX);

    for (i = 0; i < PAL_BB_AFE_TX_POWER_LEVELS; i++) {
        /* Trim LDO settings across power */
        PalBbAfeSetTxPowerIndex(i);
        PalBbAfeTxSetup();

        palBbAfeCalLdo();

        /* Save the LDO settings for each TX power level */
        txSettings[i].ldocr = MXC_GCR->btle_ldocr;
        txSettings[i].txLdoTrim = MXC_R_TX_LDO_TRIM;
        txSettings[i].rxLdoTrim = MXC_R_RX_LDO_TRIM;
    }

    palBbAfeCalcFullDeltaF();

    // Restore the initial power setting
    PalBbAfeSetTxPower(PAL_BB_AFE_INIT_TX_POWER);

    /* initialize settings for RX */
    PalBbAfeRxSetup();

    MXC_DBB_CTRL->cmu_gating_on = 1;

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeSetup(void)
{
    PalBbAfeRegsInit();

    PalBbSeqInit();

    /* Set a bogus value to init the starting power level */
    static const int8_t bogusTxPower = -100;
    palBbAfeCb.txPower = bogusTxPower;
    PalBbAfeSetTxPower(PAL_BB_AFE_INIT_TX_POWER);

    PalBbAfeCalibrate();

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeEnable(void)
{
    /* Restore BTLELDOCN setting */
    MXC_GCR->btle_ldocr = palBbAfeCb.rxSettings.ldocr;

    /* Enable 32Mhz oscillator */
    MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_X32M_EN;

    /* TODO: Apply the LDO trim values for RX */

    return TRUE;
}

/*************************************************************************************************/
bool_t PalBbAfeDisable(void)
{
    /* Disable the 32MHz XO if not being used as the system clock */
    if ((MXC_GCR->clkcn & MXC_F_GCR_CLKCN_CLKSEL) != MXC_S_GCR_CLKCN_CLKSEL_XTAL32M) {
        MXC_GCR->clkcn &= ~(MXC_F_GCR_CLKCN_X32M_EN);

        /* Disable BTLE LDOs */
        MXC_GCR->btle_ldocr =
            (MXC_F_GCR_BTLE_LDOCR_LDOTXOPULLD | MXC_F_GCR_BTLE_LDOCR_LDORXPULLD |
             0x1 << MXC_F_GCR_BTLE_LDOCR_LDOTXVSEL_POS | 0x1 << MXC_F_GCR_BTLE_LDOCR_LDORXVSEL_POS);

        return TRUE;
    }

    return FALSE;
}

/*************************************************************************************************/
bool_t PalBbAfeRestore(void)
{
    /* Set appropriate AFE version */
    MXC_DBB_RFFE->rffe_ifc_rffe_version = MXC_S_DBB_RFFE_RFFE_IFC_RFFE_VERSION_VERSION_PAN_2G;
    MXC_DBB_RFFE->general_param = 0;

#if 0
    /* Restore Calibrated AFE register settings */
    PalBbDbbSpiWrite(MXC_R_AFE_REG4, palBbAfeCb.afeRegsCal.reg4);
    PalBbDbbSpiWrite(MXC_R_AFE_REG8, palBbAfeCb.afeRegsCal.reg8);
    PalBbDbbSpiWrite(MXC_R_AFE_REG10, palBbAfeCb.afeRegsCal.reg10);
    PalBbDbbSpiWrite(MXC_R_AFE_REG11, palBbAfeCb.afeRegsCal.reg11);
    PalBbDbbSpiWrite(MXC_R_AFE_REG12, palBbAfeCb.afeRegsCal.reg12);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA7, palBbAfeCb.afeRegsCal.rega7);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA8, palBbAfeCb.afeRegsCal.rega8);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA11, palBbAfeCb.afeRegsCal.rega11);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA14, palBbAfeCb.afeRegsCal.rega14);
    PalBbDbbSpiWrite(MXC_R_AFE_REGA15, palBbAfeCb.afeRegsCal.rega15);
    PalBbDbbSpiWrite(MXC_R_AFE_REGB4, palBbAfeCb.afeRegsCal.regb4);
#else
    int i;
    uint8_t *calRegsPtr = (uint8_t *)&(palBbAfeCb.afeRegsCal.rsv_0x0);

    // TODO: Only restore necessary registers

    /* Read the first set of registers, addr 0 is reserved */
    for (i = MXC_R_AFE_REG1; i <= MXC_R_AFE_REG15; i++) {
        if (!PalBbDbbSpiWrite(i, *(calRegsPtr + i))) {
            return FALSE;
        }
    }

    /* Read the second set of registers, addr 0x10 - 0x20 is reserved */
    for (i = MXC_R_AFE_REGA1; i <= MXC_R_AFE_REGB7; i++) {
        if (!PalBbDbbSpiWrite(i, *(calRegsPtr + i))) {
            return FALSE;
        }
    }
#endif

    PalBbSeqInit();

    /* Set the starting power level */
    PalBbAfeSetTxPower(palBbAfeCb.txPower);

    return TRUE;
}
