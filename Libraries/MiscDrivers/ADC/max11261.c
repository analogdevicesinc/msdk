/**
 * @file    max11261.c
 * @brief   MAX11261 driver source.
 * @details This is the driver source of MAX11261 Analog-to-Digital converter.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 ******************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "max11261.h"
#include "max11261_regs.h"

/* **** Definitions **** */
#ifndef DBG_LVL
#define DBG_LVL 0
#endif

#if DBG_LVL > 0
#define log_err(fmt, ...) do { \
        printf("[MAX11261] ERR: %s:"  fmt "\n", __func__, ##__VA_ARGS__); \
} while (0)
#else
#define log_err(fmt, ...)
#endif

#if DBG_LVL > 1
#define log_inf(fmt, ...) do { \
        printf("[MAX11261] INF: " fmt "\n", ##__VA_ARGS__); \
} while (0)
#else
#define log_inf(fmt, ...)
#endif

#if DBG_LVL > 2
#define log_dbg(fmt, ...) do { \
        printf("[MAX11261] DBG: %s: " fmt "\n", __func__, ##__VA_ARGS__); \
} while (0)
#else
#define log_dbg(fmt, ...)
#endif

#define MAX11261_V_AVDD_MIN     2700
#define MAX11261_V_AVDD_MAX     3600
#define MAX11261_V_REF_MIN      1500
#define MAX11261_MAX_SERIAL_FREQ    5000 // KHz
#define MAX11261_MAX_REG_SIZE   4U
#define MAX11261_CHANNEL_COUNT  6
#define MAX11261_ADC_RESOLUTION 24

#define MAX11261_MUX_DELAY_MAX  1020
#define MAX11261_MUX_DELAY_MIN  4
/* **** Variable Declaration **** */

/**
 * @brief MAX11261 ADC platform context.
 *
 * Structure to hold platform specific methods used by this driver.
 */
typedef struct {
    max11261_transfer_func_t transfer;  /**< MCU specific I2C transfer function */
    max11261_reset_ctrl_t reset;        /**< MCU specific hard reset function */
    max11261_delay_func_t delayUs;      /**< MCU specific delay function */
    max11261_ready_func_t ready;        /**< MCU specific ready function */
} max11261_adc_platform_ctx_t;

/**
 * @brief Structure to hold configuration parameters.
 */
typedef struct {
    uint16_t avdd;  /**< V_AVDD in millivolts */
    uint16_t vref;  /**< REFP - REFN in millivolts */
    uint8_t slave;  /**< I2C slave address */
    uint8_t freq;   /**< Interface speed */
    uint8_t res;    /**< ADC resolution */
} max11261_adc_config_t;

/**
 * @brief Sequencer parameters.
 */
typedef struct {
    max11261_single_rate_t srate; /**< Conversion rate in single cycle mode */
    max11261_cont_rate_t crate;   /**< Conversion rate in continuous mode */
    max11261_adc_channel_t chan;  /**< Conversion channel */
    max11261_conversion_mode_t convMode;    /**< Conversion mode */
    max11261_sequencer_mode_t seqMode;      /**< Sequencer mode */
    max11261_pol_t polarity;      /**< Input polarity */
    max11261_fmt_t format;        /**< Result format */
    uint16_t delay;  /**< Multiplexer delay, 0 - 1020us */
} max11261_adc_seq_t;

struct max11261_reg {
    const char *name;
    uint8_t size;
};

static const struct max11261_reg regs[] = {
        {"STAT", 3},
        {"CTRL1", 1},
        {"CTRL2", 1},
        {"CTRL3", 1},
        {"SEQ", 2},
        {"CHMAP1", 3},
        {"CHMAP0", 3},
        {"DELAY", 3},
        {"LIMIT_LOW0", 3},
        {"LIMIT_LOW1", 3},
        {"LIMIT_LOW2", 3},
        {"LIMIT_LOW3", 3},
        {"LIMIT_LOW4", 3},
        {"LIMIT_LOW5", 3},
        {"SOC", 3},
        {"SGC", 3},
        {"SCOC", 3},
        {"SCGC", 3},
        {"GPO_DIR", 1},
        {"FIFO", 4},
        {"FIFO_LEVEL", 1},
        {"FIFO_CTRL", 1},
        {"INPUT_INT_EN", 1},
        {"INT_STAT", 2},
        {"HPF", 1},
        {"LIMIT_HIGH0", 3},
        {"LIMIT_HIGH1", 3},
        {"LIMIT_HIGH2", 3},
        {"LIMIT_HIGH3", 3},
        {"LIMIT_HIGH4", 3},
        {"LIMIT_HIGH5", 3},
};

/**
 * 10x microseconds delay for each sample rate.
 */
static const uint32_t max11261_single_cycle_delay[] = {
        2000,  // MAX11261_SINGLE_RATE_50
        1600,  // MAX11261_SINGLE_RATE_62_5
        1000,  // MAX11261_SINGLE_RATE_100
        800,   // MAX11261_SINGLE_RATE_125
        500,   // MAX11261_SINGLE_RATE_200
        400,   // MAX11261_SINGLE_RATE_250
        250,   // MAX11261_SINGLE_RATE_400
        200,   // MAX11261_SINGLE_RATE_500
        125,   // MAX11261_SINGLE_RATE_800
        100,   // MAX11261_SINGLE_RATE_1000
        62,    // MAX11261_SINGLE_RATE_1600
        50,    // MAX11261_SINGLE_RATE_2000
        31,    // MAX11261_SINGLE_RATE_3200
        25,    // MAX11261_SINGLE_RATE_4000
        16,    // MAX11261_SINGLE_RATE_6400
        8      // MAX11261_SINGLE_RATE_12800
};

/* **** Global Variables **** */

/**
 * @brief Platform specific methods and variables.
 */
static max11261_adc_platform_ctx_t platCtx;

/**
 * @brief Device specific variables.
 */
static max11261_adc_config_t cfg;

/**
 * @brief Variables concerning ADC behaviour.
 */
static max11261_adc_seq_t seq = {
        .srate = MAX11261_SINGLE_RATE_50,
        .crate = MAX11261_CONT_RATE_1_9,
        .chan = MAX11261_ADC_CHANNEL_0,
        .convMode = MAX11261_SINGLE_CYCLE,
        .seqMode = MAX11261_SEQ_MODE_1,
        .polarity = MAX11261_POL_UNIPOLAR,
        .format = MAX11261_FMT_OFFSET_BINARY,
        .delay = 0,
};

/* **** Function Prototypes **** */


void max11261_adc_platform_init(max11261_transfer_func_t transferFunc,
        max11261_reset_ctrl_t resetCtrl,
        max11261_delay_func_t delayFunc)
{
    platCtx.transfer = transferFunc;
    platCtx.reset = resetCtrl;
    platCtx.delayUs = delayFunc;
    platCtx.ready = NULL;
}

int max11261_adc_config_init(uint16_t vavdd, uint16_t vref, uint16_t freq,
        uint8_t slaveAddr)
{
    if (vavdd < MAX11261_V_AVDD_MIN || vavdd > MAX11261_V_AVDD_MAX)
            return -EINVAL;

    if (vref < MAX11261_V_REF_MIN || vref > vavdd)
        return -EINVAL;

    if (freq < 100 || freq >= MAX11261_MAX_SERIAL_FREQ)
        return -EINVAL;

    cfg.avdd = vavdd;
    cfg.vref = vref;
    cfg.freq = freq;
    cfg.slave = slaveAddr;
    cfg.res = MAX11261_ADC_RESOLUTION;

    return 0;
}

void max11261_adc_set_ready_func(max11261_ready_func_t readyFunc)
{
    platCtx.ready = readyFunc;
}

int max11261_adc_reset(void)
{
    int error;
    platCtx.reset(1);
    platCtx.delayUs(100);
    platCtx.reset(0);
    /* From any state to STANDBY state: 10ms */
    platCtx.delayUs(10000);
    error = platCtx.transfer(NULL, 0, NULL, 0, cfg.slave);
    if (error < 0) {
        log_err("Probe failed unexpectedly: %d", error);
        return error;
    }

    return error;
}

static int max11261_read_reg(uint8_t addr, uint32_t *val)
{
    int error;
    uint8_t txbuf[1];
    uint8_t rxbuf[MAX11261_MAX_REG_SIZE];

    txbuf[0] = MAX11261_CMD_READ(addr);
    error = platCtx.transfer(txbuf, 1, &rxbuf[0], regs[addr].size, cfg.slave);
    if (error < 0) {
        log_err("Failed to read reg 0x%02X (%s)", addr, regs[addr].name);
    } else {
        switch (regs[addr].size) {
            case 1:
                *val = rxbuf[0];
                break;
            case 2:
                *val = rxbuf[1] | rxbuf[0] << 8;
                break;
            case 3:
                *val = rxbuf[2] | rxbuf[1] << 8 | rxbuf[0] << 16;
                break;
            case 4:
            default:
                *val = rxbuf[3] | rxbuf[2] << 8 | rxbuf[1] << 16
                | rxbuf[0] << 24;
                break;
        }
    }

    return error;
}

static inline int max11261_write_byte(uint8_t byte)
{
    return platCtx.transfer(&byte, 1, NULL, 0, cfg.slave);
}

static int max11261_write_reg(uint8_t addr, uint32_t val)
{
    int error;
    uint8_t txbuf[MAX11261_MAX_REG_SIZE + 1];

    txbuf[0] = MAX11261_CMD_WRITE(addr);
    switch (regs[addr].size) {
        case 1:
            txbuf[1] = val & 0xFF;
            break;
        case 2:
            txbuf[1] = (val >> 8) & 0xFF;
            txbuf[2] = val & 0xFF;
            break;
        case 3:
            txbuf[1] = (val >> 16) & 0xFF;
            txbuf[2] = (val >> 8) & 0xFF;
            txbuf[3] = val & 0xFF;
            break;
        case 4:
        default:
            txbuf[1] = (val >> 24) & 0xFF;
            txbuf[2] = (val >> 16) & 0xFF;
            txbuf[3] = (val >> 8) & 0xFF;
            txbuf[4] = val & 0xFF;
            break;
    }

    error = platCtx.transfer(txbuf, regs[addr].size + 1, NULL, 0, cfg.slave);
    if (error < 0)
        log_err("Failed to write reg 0x%02X", addr);

    return error;
}

static int max11261_update_reg(uint8_t addr, uint32_t mask, uint32_t val)
{
    int error;
    uint32_t tmp;

    error = max11261_read_reg(addr, &tmp);
    if (error < 0)
        return error;

    tmp &= ~mask;
    tmp |= val;

    return max11261_write_reg(addr, tmp);
}

static int max11261_set_powerdown_mode(uint8_t mode)
{
    int error;
    uint32_t val, timeout;
    uint8_t mask;

    error = max11261_update_reg(MAX11261_CTRL1, MAX11261_CTRL1_PD, mode);
    if (error < 0)
        return error;

    error = max11261_write_byte(MAX11261_CMD_POWERDOWN);
    if (error < 0)
        return error;

    mask = (mode == MAX11261_CTRL1_PD_SLEEP) ? MAX11261_STAT_PDSTAT_SLEEP
            : MAX11261_STAT_PDSTAT_STANDBY;
    timeout = 2000; /* 2000 x 10us */
    max11261_read_reg(MAX11261_STAT, &val);
    while ((val & MAX11261_STAT_PDSTAT) != mask && --timeout) {
        platCtx.delayUs(10);
        max11261_read_reg(MAX11261_STAT, &val);
    }

    if (timeout == 0)
        return -ETIMEDOUT;

    log_dbg("Time remaining %u us", timeout * 10);
    return error;
}

int max11261_adc_set_polarity(max11261_pol_t pol)
{
    if (pol == MAX11261_POL_UNIPOLAR && seq.seqMode == MAX11261_SEQ_MODE_4)
        return -EINVAL;

    seq.polarity = pol;

    return 0;
}

int max11261_adc_set_format(max11261_fmt_t fmt)
{
    if (seq.polarity == MAX11261_POL_UNIPOLAR) {
        if (fmt == MAX11261_FMT_TWOS_COMPLEMENT)
            return -EINVAL;
    }

    if (seq.polarity == MAX11261_POL_BIPOLAR) {
        if (fmt == MAX11261_FMT_OFFSET_BINARY) {
            if (seq.seqMode == MAX11261_SEQ_MODE_4)
                return -EINVAL;
        }
    }

    seq.format = fmt;

    return 0;
}

void max11261_adc_set_rate_single(max11261_single_rate_t rate)
{
    seq.srate = rate;
}

int max11261_adc_set_rate_continuous(max11261_cont_rate_t rate)
{
    if (rate == MAX11261_CONT_RATE_16000 && cfg.freq < 1000)
        return -EINVAL;

    seq.crate = rate;

    return 0;
}

int max11261_adc_set_channel(max11261_adc_channel_t chan)
{
    if (chan < MAX11261_ADC_CHANNEL_0 || chan >= MAX11261_ADC_CHANNEL_MAX)
        return -EINVAL;

    seq.chan = chan;

    return 0;
}

int max11261_adc_set_mode(max11261_conversion_mode_t convMode,
        max11261_sequencer_mode_t seqMode)
{
    int error;
    if (convMode < MAX11261_LATENT_CONTINUOUS
        || convMode > MAX11261_SINGLE_CYCLE_CONTINUOUS)
        return -EINVAL;

    if (seqMode < MAX11261_SEQ_MODE_1 || seqMode > MAX11261_SEQ_MODE_4)
        return -EINVAL;

    if (max11261_adc_pd_state() == MAX11261_PDSTATE_CONVERSION) {
        log_inf("Device is in conversion state, stopping");
        error = max11261_adc_standby();
        if (error < 0) {
            log_err("Failed to stop conversion");
            return error;
        }
    }

    /* Sequencer mode 2, 3, 4 can only run single cycle */
    if (seqMode != MAX11261_SEQ_MODE_1) {
        if (convMode != MAX11261_SINGLE_CYCLE)
            return -EINVAL;
    }

    seq.seqMode = seqMode;
    seq.convMode = convMode;

    return 0;
}

int max11261_adc_set_mux_delay(uint16_t delay)
{
    if (delay > MAX11261_MUX_DELAY_MAX)
        return -EINVAL;

    if (delay && delay < MAX11261_MUX_DELAY_MIN)
        delay = MAX11261_MUX_DELAY_MIN;

    seq.delay = delay;

    return 0;
}

max11261_pd_state_t max11261_adc_pd_state(void)
{
    int error;
    uint32_t val;

    error = max11261_read_reg(MAX11261_STAT, &val);

    if (error == 0)
        return (val & MAX11261_STAT_PDSTAT) >> MAX11261_STAT_PDSTAT_POS;

    return error;
}

int max11261_adc_dump_regs(void)
{
    int error;
    uint32_t val;

    for (uint8_t reg = MAX11261_STAT; reg <= MAX11261_LIMIT_HIGH5; reg++) {
        error = max11261_read_reg(reg, &val);
        if (error == 0)
            log_inf("%s@%02Xh: 0x%08X", regs[reg].name, reg, val);
        else
            break;
    }

    return error;
}

int max11261_adc_sleep(void)
{
    return max11261_set_powerdown_mode(MAX11261_CTRL1_PD_SLEEP);
}

int max11261_adc_standby(void)
{
    return max11261_set_powerdown_mode(MAX11261_CTRL1_PD_STANDBY);
}

int max11261_adc_calibrate_self(void)
{
    int error = max11261_update_reg(MAX11261_CTRL1, MAX11261_CTRL1_CAL,
            MAX11261_CTRL1_CAL_SELF);

    if (error < 0)
        return error;

    error = max11261_write_byte(MAX11261_CMD_CALIBRATE);
    if (error < 0)
        return error;

    platCtx.delayUs(100000);

    return 0;
}

static inline int sif_freq(uint16_t freq)
{
    if (freq <= 400)
        return MAX11261_SIF_FREQ_100_400;
    else if (freq <= 1000)
        return MAX11261_SIF_FREQ_401_1000;
    else if (freq <= 3000)
        return MAX11261_SIF_FREQ_1001_3000;
    else
        return MAX11261_SIF_FREQ_3001_5000;
}

int max11261_adc_convert_prepare(void)
{
    int error;

    /* Set sequencer register */
    error = max11261_update_reg(MAX11261_SEQ,
            MAX11261_SEQ_MUX | MAX11261_SEQ_MDREN | MAX11261_SEQ_SIF_FREQ,
            (seq.chan << MAX11261_SEQ_MUX_POS)
            | (seq.delay ? MAX11261_SEQ_MDREN : 0)
            | sif_freq(cfg.freq));
    if (error < 0)
        return error;

    if (seq.delay) {
        /* Delay resolution is 4us */
        error = max11261_update_reg(MAX11261_DELAY, MAX11261_DELAY_MUX,
                (seq.delay / 4) << MAX11261_DELAY_MUX_POS);
        if (error < 0)
            return error;
    }

    /* Set control register 1
     * PD       : Go to standby after conversion
     * U_B      : Unipolar input range
     * FORMAT   : Offset binary format
     * SCYCLE   : Single cycle conversion
     */
    error = max11261_write_reg(MAX11261_CTRL1,
              MAX11261_CTRL1_PD_STANDBY
            | ((seq.format == MAX11261_FMT_TWOS_COMPLEMENT) ?
                    MAX11261_CTRL1_FORMAT_TWOS_COMP :
                    MAX11261_CTRL1_FORMAT_OFFSET_BIN)
            | ((seq.polarity == MAX11261_POL_UNIPOLAR) ?
                    MAX11261_CTRL1_U_B_UNIPOLAR :
                    MAX11261_CTRL1_U_B_BIPOLAR)
            | ((seq.convMode != MAX11261_LATENT_CONTINUOUS) ?
                    MAX11261_CTRL1_SCYCLE_SINGLE :
                    MAX11261_CTRL1_SCYCLE_CONT));
    if (error < 0)
        return error;

    /* Enable input interrupts if ready function is set, disable otherwise */
    error = max11261_update_reg(MAX11261_INPUT_INT_EN,
            MAX11261_INPUT_INT_EN_RDYB,
            platCtx.ready ? MAX11261_INPUT_INT_EN_RDYB : 0);

    return error;
}

int max11261_adc_result(max11261_adc_result_t *res)
{
    int error;
    uint32_t cnt, timeout, reg;
    int32_t tmp;

    /* 10 times delay amounts to the data rates given in the datasheet. Adding
     * two extra delay cycles to make up for the communication delay. */
    timeout = 12 + seq.delay / max11261_single_cycle_delay[seq.srate];
    if (platCtx.ready) {
        while (!platCtx.ready() && --timeout) {
            platCtx.delayUs(max11261_single_cycle_delay[seq.srate]);
        }
    } else {
        max11261_read_reg(MAX11261_STAT, &reg);
        while ((reg & MAX11261_STAT_RDY) == 0 && --timeout) {
            platCtx.delayUs(max11261_single_cycle_delay[seq.srate]);
            max11261_read_reg(MAX11261_STAT, &reg);
        }
    }
    if (timeout == 0)
        return -ETIMEDOUT;

    error = max11261_read_reg(MAX11261_STAT, &reg);
    if (error < 0)
        return error;

    res->dor = !!(reg & MAX11261_STAT_DOR);
    res->aor = !!(reg & MAX11261_STAT_AOR);

    error = max11261_read_reg(MAX11261_FIFO_LEVEL, &cnt);
    if (error < 0)
        return error;

    if (cnt > 1)
        log_inf("More than 1 conversion result is available: %u", cnt);

    while (cnt--) {
        error = max11261_read_reg(MAX11261_FIFO, &reg);
        if (error < 0)
            return error;
    }

    res->chn = (reg & MAX11261_FIFO_CH) >> MAX11261_FIFO_CH_POS;

    reg &= ((1 << cfg.res) - 1);
    tmp = reg;
    /* Extend signed 24-bit integer to 32-bit if the value read from the FIFO
     * is negative. */
    if (seq.format == MAX11261_FMT_TWOS_COMPLEMENT && (tmp & 0x800000)) {
        tmp |= 0xFF000000;
    }

    tmp = ((int64_t) (tmp) * cfg.vref) >> cfg.res;
    if (seq.polarity == MAX11261_POL_BIPOLAR) {
        if (seq.format == MAX11261_FMT_OFFSET_BINARY)
            tmp = 2 * tmp - cfg.vref;
        else
            tmp = 2 * tmp;
    }
    res->val = tmp;

    return 0;
}

int max11261_adc_convert(void)
{
    int error;

    /* Start conversion */
    error = max11261_write_byte(MAX11261_CMD_SEQUENCER(seq.srate));
    if (error < 0)
        return error;

    return 0;
}
