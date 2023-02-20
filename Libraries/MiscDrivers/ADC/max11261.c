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

#define MAX11261_MUX_DELAY_MAX  1023
#define MAX11261_MUX_DELAY_MIN  4
#define MAX11261_GPO_DELAY_MAX  5100
#define MAX11261_GPO_DELAY_MIN  20
#define MAX11261_AUTOSCAN_DELAY_MAX  1023
#define MAX11261_AUTOSCAN_DELAY_MIN  4

#define MAX11261_HPF_CUTOFF_MAX 7

#define MAX11261_READ_REG(addr, val) \
    do { \
            error = max11261_read_reg(addr, val); \
            if (error < 0) \
                return error; \
    } while (0);

#define MAX11261_WRITE_REG(addr, val) \
    do { \
        error = max11261_write_reg(addr, val); \
        if (error < 0) \
            return error; \
    } while(0);

#define MAX11261_UPDATE_REG(addr, mask, val) \
    do { \
        error = max11261_update_reg(addr, mask, val); \
        if (error < 0) \
            return error; \
    } while(0);


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
    uint16_t muxDelay;      /**< Multiplexer delay, 0 - 1020ms */
    uint16_t gpoDelay;      /**< GPO delay, 20 - 5100us */
    uint16_t autoscanDelay; /**< Autoscan delay, 4 - 1020ms */
    /**< Scan order of channels in mode 2,3 or 4 */
    uint8_t order[MAX11261_ADC_CHANNEL_MAX];
    max11261_gpo_t gpoMap[MAX11261_ADC_CHANNEL_MAX]; /**< GPO map */
    uint8_t srdy;    /**< STAT:SRDY mask depending on enabled channels */
} max11261_adc_seq_t;

/**
 * @brief Miscellaneous control parameters.
 */
typedef struct {
    uint8_t pga    : 3; /**< PGA gain */
    uint8_t pga_en : 1; /**< PGA enable switch */
    uint8_t lp_mode: 1; /**< PGA low-power mode */
    uint8_t ldo_en : 1; /**< Integrated LDO enable */
    uint8_t css_en : 1; /**< Current source and sink on inputs */
    uint8_t rfu    : 1;
} max11261_ctrl_t;

/**
 * @brief Highpass filter parameters.
 */
typedef struct {
    int16_t limitMin[MAX11261_ADC_CHANNEL_MAX];
    int16_t limitMax[MAX11261_ADC_CHANNEL_MAX];
    uint8_t freq    : 3;    /**< HPF cutoff frequency */
    uint8_t cmp_mode: 2;    /**< Compare method of input conversion results */
    uint8_t rfu     : 3;
} max11261_hpf_t;

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
 * 1/10th of delay for each sample rate.
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

/**
 * 1/10th of delay for each sample rate.
 */
static const uint32_t max11261_cont_cycle_delay[] = {
        52600, // MAX11261_CONT_RATE_1_9
        25600, // MAX11261_CONT_RATE_3_9
        12800, // MAX11261_CONT_RATE_7_8
        6400,  // MAX11261_CONT_RATE_15_6
        3200,  // MAX11261_CONT_RATE_31_2
        1600,  // MAX11261_CONT_RATE_62_5
        800,   // MAX11261_CONT_RATE_125
        400,   // MAX11261_CONT_RATE_250
        200,   // MAX11261_CONT_RATE_500
        100,   // MAX11261_CONT_RATE_1000
        50,    // MAX11261_CONT_RATE_2000
        25,    // MAX11261_CONT_RATE_4000
        12,    // MAX11261_CONT_RATE_8000
        6      // MAX11261_CONT_RATE_16000
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
        .muxDelay = 0,
        .gpoDelay = 0,
        .order = {0, 0, 0, 0, 0, 0},
        .gpoMap = {
                MAX11261_GPO_INVALID, MAX11261_GPO_INVALID,
                MAX11261_GPO_INVALID, MAX11261_GPO_INVALID,
                MAX11261_GPO_INVALID, MAX11261_GPO_INVALID
        },
        .srdy = 0,
};

/**
 * @brief Other control parameters.
 */
static max11261_ctrl_t ctrl = {
        .pga = MAX11261_PGA_GAIN_1,
        .pga_en = 0,
        .lp_mode = 0,
        .ldo_en = 1,
        .css_en = 0,
};

/**
 * @brief Highpass filter parameters.
 */
static max11261_hpf_t hpf = {
        .limitMin = { 0, 0, 0, 0, 0, 0},
        .limitMax = { 0, 0, 0, 0, 0, 0},
        .freq = 0,
        .cmp_mode = MAX11261_COMP_MODE_CURR,
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

    MAX11261_READ_REG(addr, &tmp);

    tmp &= ~mask;
    tmp |= val;

    MAX11261_WRITE_REG(addr, tmp);

    return 0;
}

static int max11261_set_powerdown_mode(uint8_t mode)
{
    int error;
    uint32_t val, timeout;
    uint8_t mask;

    MAX11261_UPDATE_REG(MAX11261_CTRL1, MAX11261_CTRL1_PD, mode);

    error = max11261_write_byte(MAX11261_CMD_POWERDOWN);
    if (error < 0)
        return error;

    mask = (mode == MAX11261_CTRL1_PD_SLEEP) ? MAX11261_STAT_PDSTAT_SLEEP
            : MAX11261_STAT_PDSTAT_STANDBY;
    timeout = 2000; /* 2000 x 10us */
    MAX11261_READ_REG(MAX11261_STAT, &val);
    while ((val & MAX11261_STAT_PDSTAT) != mask && --timeout) {
        platCtx.delayUs(10);
        MAX11261_READ_REG(MAX11261_STAT, &val);
    }

    if (timeout == 0)
        return -ETIMEDOUT;

    log_dbg("Time remaining %u us", timeout * 10);
    return error;
}

max11261_pd_state_t max11261_adc_pd_state(void)
{
    int error;
    uint32_t val;

    MAX11261_READ_REG(MAX11261_STAT, &val);

    return (val & MAX11261_STAT_PDSTAT) >> MAX11261_STAT_PDSTAT_POS;
}

int max11261_adc_dump_regs(void)
{
    int error;
    uint32_t val;

    for (uint8_t reg = MAX11261_STAT; reg <= MAX11261_LIMIT_HIGH5; reg++) {
        MAX11261_READ_REG(reg, &val);
        log_inf("%s@%02Xh: 0x%08X", regs[reg].name, reg, val);
    }

    return 0;
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
    int error;

    MAX11261_UPDATE_REG(MAX11261_CTRL1, MAX11261_CTRL1_CAL,
            MAX11261_CTRL1_CAL_SELF);

    error = max11261_write_byte(MAX11261_CMD_CALIBRATE);
    if (error < 0)
        return error;

    platCtx.delayUs(100000);

    return 0;
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

int max11261_adc_enable_gpo(max11261_gpo_t gpo)
{
    if (gpo >= MAX11261_GPO_MAX)
        return -EINVAL;

    return max11261_update_reg(MAX11261_GPO_DIR,
            ((1 << gpo) << MAX11261_GPO_DIR_GPO_POS),
            ((1 << gpo) << MAX11261_GPO_DIR_GPO_POS));
}

int max11261_adc_disable_gpo(max11261_gpo_t gpo)
{
    if (gpo >= MAX11261_GPO_MAX)
        return -EINVAL;

    return max11261_update_reg(MAX11261_GPO_DIR,
            ((1 << gpo) << MAX11261_GPO_DIR_GPO_POS), 0);
}

int max11261_adc_set_channel_order(max11261_adc_channel_t chan, uint8_t order)
{
    if (chan < MAX11261_ADC_CHANNEL_0 || chan >= MAX11261_ADC_CHANNEL_MAX)
        return -EINVAL;

    if (order > MAX11261_ADC_CHANNEL_MAX)
        return -EINVAL;

    seq.order[chan] = order;
    seq.srdy &= ~(1 << chan);
    if (order > 0)
        seq.srdy |= (1 << chan);

    return 0;
}

int max11261_adc_set_channel_gpo(max11261_adc_channel_t chan,
        max11261_gpo_t gpo)
{
    uint8_t c;

    if (chan < MAX11261_ADC_CHANNEL_0 || chan >= MAX11261_ADC_CHANNEL_MAX)
            return -EINVAL;
    if (gpo < MAX11261_GPO_INVALID || gpo >= MAX11261_GPO_MAX)
        return -EINVAL;

    /* Check if GPO has already been mapped to another channel */
    if (gpo != MAX11261_GPO_INVALID) {
        for (c = MAX11261_ADC_CHANNEL_0; c < MAX11261_ADC_CHANNEL_MAX; c++) {
            if (seq.gpoMap[c] == gpo && c != chan)
                return -EACCES;
        }
    }
    seq.gpoMap[chan] = gpo;

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

int max11261_adc_set_gain(max11261_pga_gain_t gain)
{
    if (gain < MAX11261_PGA_GAIN_1 || gain >= MAX11261_PGA_GAIN_MAX)
        return -EINVAL;

    ctrl.pga = gain;
    ctrl.pga_en = 1;

    return 0;
}

void max11261_adc_disable_pga(void)
{
    ctrl.pga_en = 0;
}

int max11261_adc_set_mux_delay(uint16_t delay)
{
    if (delay > MAX11261_MUX_DELAY_MAX)
        return -EINVAL;

    if (delay && delay < MAX11261_MUX_DELAY_MIN)
        delay = MAX11261_MUX_DELAY_MIN;

    seq.muxDelay = delay;

    return 0;
}

int max11261_adc_set_gpo_delay(uint16_t delay)
{
    if (delay > MAX11261_GPO_DELAY_MAX)
        return -EINVAL;

    if (delay && delay < MAX11261_GPO_DELAY_MIN)
        delay = MAX11261_GPO_DELAY_MIN;

    seq.gpoDelay = delay;

    return 0;
}

int max11261_adc_set_autoscan_delay(uint16_t delay)
{
    if (delay > MAX11261_AUTOSCAN_DELAY_MAX)
        return -EINVAL;

    if (delay && delay < MAX11261_AUTOSCAN_DELAY_MIN)
        delay = MAX11261_AUTOSCAN_DELAY_MIN;

    seq.autoscanDelay = delay;

    return 0;
}

int max11261_adc_set_hpf_frequency(uint8_t freq)
{
    if (freq > MAX11261_HPF_CUTOFF_MAX)
        return -EINVAL;

    hpf.freq = freq;

    return 0;
}

int max11261_adc_set_compare_mode(max11261_comp_mode_t mode)
{
    if (mode > MAX11261_COMP_MODE_HPF)
        return -EINVAL;

    hpf.cmp_mode = mode;

    return 0;
}

static int limit_valid(max11261_adc_channel_t chan, int16_t limit)
{
    double min, max;

    if (chan < MAX11261_ADC_CHANNEL_0 || chan >= MAX11261_ADC_CHANNEL_MAX)
        return -EINVAL;

    if (ctrl.pga_en) {
        max = (double) cfg.vref / (1 << ctrl.pga);
        min = -1 * max;
    } else {
        max = cfg.vref;
        min = -1 * max;
    }

    if (limit < min || limit > max)
        return -EINVAL;

    return 0;
}

int max11261_adc_set_limit_low(max11261_adc_channel_t chan, int16_t limit)
{
    if (limit_valid(chan, limit))
        return -EINVAL;

    hpf.limitMin[chan] = limit;

    return 0;
}

int max11261_adc_set_limit_high(max11261_adc_channel_t chan, int16_t limit)
{
    if (limit_valid(chan, limit))
        return -EINVAL;

    hpf.limitMax[chan] = limit;

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

/**
 * Return the given millivolts as 24-bit ADC representation.
 */
static uint32_t mv_to_d(int16_t mv)
{
    int64_t tmp = 0;

    tmp = mv / 2;
    if (tmp < 0) tmp = -1 * tmp;
    tmp = tmp << cfg.res;
    tmp = tmp / cfg.vref;

    /* Add sign bit if \a mv is negative. */
    if (mv < 0) {
        tmp |= 0x800000;
    }

    return (uint32_t) (tmp & ((1 << cfg.res) - 1));
}

int max11261_adc_convert_prepare(void)
{
    int error, j;
    uint32_t chmap0, chmap1;

    /* Set sequencer register */
    MAX11261_UPDATE_REG(MAX11261_SEQ,
              MAX11261_SEQ_MUX | MAX11261_SEQ_MODE | MAX11261_SEQ_GPODREN
            | MAX11261_SEQ_MDREN
            | MAX11261_SEQ_RDYBEN
            | MAX11261_SEQ_SIF_FREQ,
              (seq.chan << MAX11261_SEQ_MUX_POS)
            | (seq.seqMode << MAX11261_SEQ_MODE_POS)
            | (seq.gpoDelay ? MAX11261_SEQ_GPODREN : 0)
            | (seq.muxDelay ? MAX11261_SEQ_MDREN : 0)
            | MAX11261_SEQ_RDYBEN
            | sif_freq(cfg.freq));

    if (seq.muxDelay) {
        /* Delay resolution is 4us */
        MAX11261_UPDATE_REG(MAX11261_DELAY, MAX11261_DELAY_MUX,
                (seq.muxDelay / 4) << MAX11261_DELAY_MUX_POS);
    }

    chmap0 = 0;
    chmap1 = 0;
    /* Operations specific to mode 3 or 4 */
    if (seq.seqMode == MAX11261_SEQ_MODE_3
        || seq.seqMode == MAX11261_SEQ_MODE_4)
    {
        /* Apply GPO delay */
        if (seq.gpoDelay) {
            /* Delay resolution is 20us */
            MAX11261_UPDATE_REG(MAX11261_DELAY, MAX11261_DELAY_GPO,
                    (seq.gpoDelay / 20) << MAX11261_DELAY_GPO_POS);
        }

        /* Map GPO channels */
        /* Channel 0, 1 and 2 -> CHMAP0 */
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_0] != MAX11261_GPO_INVALID) {
            chmap0 |=  MAX11261_CHMAP0_CH0_GPOEN;
            chmap0 |= seq.gpoMap[MAX11261_ADC_CHANNEL_0] <<
                    MAX11261_CHMAP0_CH0_GPO_POS;
        }
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_1] != MAX11261_GPO_INVALID) {
            chmap0 |=  MAX11261_CHMAP0_CH1_GPOEN;
            chmap0 |= seq.gpoMap[MAX11261_ADC_CHANNEL_1] <<
                    MAX11261_CHMAP0_CH1_GPO_POS;
        }
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_2] != MAX11261_GPO_INVALID) {
            chmap0 |=  MAX11261_CHMAP0_CH2_GPOEN;
            chmap0 |= seq.gpoMap[MAX11261_ADC_CHANNEL_2] <<
                    MAX11261_CHMAP0_CH2_GPO_POS;
        }

        /* Channel 3, 4 and 5 -> CHMAP1 */
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_3] != MAX11261_GPO_INVALID) {
            chmap1 |=  MAX11261_CHMAP1_CH3_GPOEN;
            chmap1 |= seq.gpoMap[MAX11261_ADC_CHANNEL_3] <<
                    MAX11261_CHMAP1_CH3_GPO_POS;
        }
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_4] != MAX11261_GPO_INVALID) {
            chmap1 |=  MAX11261_CHMAP1_CH4_GPOEN;
            chmap1 |= seq.gpoMap[MAX11261_ADC_CHANNEL_4] <<
                    MAX11261_CHMAP1_CH4_GPO_POS;
        }
        if (seq.gpoMap[MAX11261_ADC_CHANNEL_5] != MAX11261_GPO_INVALID) {
            chmap1 |=  MAX11261_CHMAP1_CH5_GPOEN;
            chmap1 |= seq.gpoMap[MAX11261_ADC_CHANNEL_5] <<
                    MAX11261_CHMAP1_CH5_GPO_POS;
        }
    }

    /* If one of the multichannel scan modes are selected, set scan orders */
    if (seq.seqMode != MAX11261_SEQ_MODE_1) {
        /* Channel 0, 1 and 2 -> CHMAP0 */
        if (seq.order[MAX11261_ADC_CHANNEL_0]) {
            chmap0 |=  MAX11261_CHMAP0_CH0_EN;
            chmap0 |= seq.order[MAX11261_ADC_CHANNEL_0] <<
                    MAX11261_CHMAP0_CH0_ORD_POS;
        }
        if (seq.order[MAX11261_ADC_CHANNEL_1]) {
            chmap0 |=  MAX11261_CHMAP0_CH1_EN;
            chmap0 |= seq.order[MAX11261_ADC_CHANNEL_1] <<
                    MAX11261_CHMAP0_CH1_ORD_POS;
        }
        if (seq.order[MAX11261_ADC_CHANNEL_2]) {
            chmap0 |=  MAX11261_CHMAP0_CH2_EN;
            chmap0 |= seq.order[MAX11261_ADC_CHANNEL_2] <<
                    MAX11261_CHMAP0_CH2_ORD_POS;
        }
        MAX11261_WRITE_REG(MAX11261_CHMAP0, chmap0);

        /* Channel 3, 4 and 5 -> CHMAP1 */
        if (seq.order[MAX11261_ADC_CHANNEL_3]) {
            chmap1 |=  MAX11261_CHMAP1_CH3_EN;
            chmap1 |= seq.order[MAX11261_ADC_CHANNEL_3] <<
                    MAX11261_CHMAP1_CH3_ORD_POS;
        }
        if (seq.order[MAX11261_ADC_CHANNEL_4]) {
            chmap1 |=  MAX11261_CHMAP1_CH4_EN;
            chmap1 |= seq.order[MAX11261_ADC_CHANNEL_4] <<
                    MAX11261_CHMAP1_CH4_ORD_POS;
        }
        if (seq.order[MAX11261_ADC_CHANNEL_5]) {
            chmap1 |=  MAX11261_CHMAP1_CH5_EN;
            chmap1 |= seq.order[MAX11261_ADC_CHANNEL_5] <<
                    MAX11261_CHMAP1_CH5_ORD_POS;
        }
        MAX11261_WRITE_REG(MAX11261_CHMAP1, chmap1);
    }

    /* Set limits if operating in sequencer mode 4 */
    if (seq.seqMode == MAX11261_SEQ_MODE_4) {
        uint8_t inten = 0;
        for (j = MAX11261_ADC_CHANNEL_0; j < MAX11261_ADC_CHANNEL_MAX; j++) {
            if (seq.order[j] != 0) {
                MAX11261_WRITE_REG(MAX11261_LIMIT_LOW0 + j,
                        mv_to_d(hpf.limitMin[j]));
                MAX11261_WRITE_REG(MAX11261_LIMIT_HIGH0 + j,
                        mv_to_d(hpf.limitMax[j]));
                inten |= (1 << j);
            }
        }
        MAX11261_UPDATE_REG(MAX11261_INPUT_INT_EN, MAX11261_INPUT_INT_EN_CH,
                inten << MAX11261_INPUT_INT_EN_CH_POS);
        /* Write HPF settings */
        MAX11261_WRITE_REG(MAX11261_HPF,
                (hpf.cmp_mode << MAX11261_HPF_CMP_MODE_POS)
              | (hpf.freq << MAX11261_HPF_FREQUENCY_POS));
    }

    /* Set control register 1
     * PD       : Go to standby after conversion
     * U_B      : Unipolar input range
     * FORMAT   : Offset binary format
     * SCYCLE   : Single cycle conversion
     * CONTSC   : Continuous single cycle
     */
    MAX11261_WRITE_REG(MAX11261_CTRL1,
              MAX11261_CTRL1_PD_STANDBY
            | ((seq.format == MAX11261_FMT_TWOS_COMPLEMENT) ?
                    MAX11261_CTRL1_FORMAT_TWOS_COMP :
                    MAX11261_CTRL1_FORMAT_OFFSET_BIN)
            | ((seq.polarity == MAX11261_POL_UNIPOLAR) ?
                    MAX11261_CTRL1_U_B_UNIPOLAR :
                    MAX11261_CTRL1_U_B_BIPOLAR)
            | ((seq.convMode != MAX11261_LATENT_CONTINUOUS) ?
                    MAX11261_CTRL1_SCYCLE_SINGLE :
                    MAX11261_CTRL1_SCYCLE_CONT)
            | ((seq.convMode == MAX11261_SINGLE_CYCLE_CONTINUOUS) ?
                    MAX11261_CTRL1_CONTSC : 0));

    /* Set control register 2
     * CSSEN    : Current source and sink
     * LDOEN    : LDO enable
     * LPMODE   : Low power PGA
     * PGAEN    : PGA enable
     * PGA      : PGA level
     */
    MAX11261_WRITE_REG(MAX11261_CTRL2,
             (ctrl.css_en << MAX11261_CTRL2_CSSEN_POS)
           | (ctrl.ldo_en << MAX11261_CTRL2_LDOEN_POS)
           | (ctrl.lp_mode << MAX11261_CTRL2_LPMODE_POS)
           | (ctrl.pga_en << MAX11261_CTRL2_PGAEN_POS)
           | (ctrl.pga << MAX11261_CTRL2_PGA_POS));

    /* Enable input interrupts if ready function is set, disable otherwise */
    MAX11261_UPDATE_REG(MAX11261_INPUT_INT_EN,
            MAX11261_INPUT_INT_EN_RDYB,
            platCtx.ready ? MAX11261_INPUT_INT_EN_RDYB : 0);

    return error;
}

/**
 * @brief Return the number of enabled channels by counting the bits in srdy
 * mask.
 * See https://graphics.stanford.edu/~seander/bithacks.html.
 */
static inline uint8_t channel_count(uint8_t srdy)
{
    /* Counting bits set, Brian Kernighan's way */
    uint8_t c; // c accumulates the total bits set in v
    for (c = 0; srdy; c++)
    {
        srdy &= srdy - 1; // clear the least significant bit set
    }
    return c;
}

/**
 * @brief Calculate conversion delay count for each mode.
 *
 * The minimum delay interval is (SampleRateDelay / 10)us. If sample rate is
 * 50sps for instance, SampleRateDelay becomes 1s / 50sps = 0.02s = 20000us.
 * The delay interval in this case is 2000us which is also hardcoded in
 * \ref max11261_single_cycle_delay. Hence the delay function has to be called
 * at least 10 times before deciding if a timeout has occurred. Two extra delay
 * cycles are added to the final delay count to prevent false negatives.
 *
 * Extra delay sources such as multiplexer and GPO delays are also considered
 * when calculating the delay count.
 *
 * Sequencer mode 1: Sum of sample rate and mux delay.
 * Sequencer mode 2: Sum of sample rate and mux delay for each enabled
 * channel.
 * Sequencer mode 3: Delay count in sequencer mode 2 plus one GPO delay. The
 * GPOs mapped to other channels are enabled during conversion of the previous
 * channel. Thus only one GPO delay needs to be considered.
 * Sequencer mode 4: Delay count in sequencer mode 3 plus autoscan delay. Math
 * operation delay should be negligible.
 */
static inline uint32_t delay_count(void)
{
    switch (seq.seqMode) {
    case MAX11261_SEQ_MODE_1:
        switch (seq.convMode) {
        case MAX11261_LATENT_CONTINUOUS:
            return 12
                + (seq.muxDelay / max11261_cont_cycle_delay[seq.crate]);
        default:
            return 12
                + (seq.muxDelay / max11261_single_cycle_delay[seq.srate]);
        }

    case MAX11261_SEQ_MODE_2:
        return 12 * channel_count(seq.srdy)
        + ((channel_count(seq.srdy) * seq.muxDelay)
                / max11261_single_cycle_delay[seq.srate]);
    case MAX11261_SEQ_MODE_3:
        return 12 * channel_count(seq.srdy)
                + (((channel_count(seq.srdy) * seq.muxDelay) + seq.gpoDelay)
                        / max11261_single_cycle_delay[seq.srate]);
    case MAX11261_SEQ_MODE_4:
        return 12 * channel_count(seq.srdy)
                + (((channel_count(seq.srdy)
                        * (seq.muxDelay + 1000 * seq.autoscanDelay))
                        + seq.gpoDelay)
                        / max11261_single_cycle_delay[seq.srate]);
    default:
        return 0;
    }
    return 0;
}

int max11261_adc_result(max11261_adc_result_t *res, int count)
{
    int error, rd = 0;
    uint32_t cnt, to, reg;
    int32_t tmp;

    to = delay_count();

    /* Use ready function to check if data is available, otherwise poll STAT
     * register's RDY or SRDY bits */
    if (platCtx.ready) {
        while (!platCtx.ready() && --to) {
            if (seq.convMode != MAX11261_LATENT_CONTINUOUS)
                platCtx.delayUs(max11261_single_cycle_delay[seq.srate]);
            else
                platCtx.delayUs(max11261_cont_cycle_delay[seq.crate]);
        }
    } else {
        MAX11261_READ_REG(MAX11261_STAT, &reg);
        while (--to) {
            if (seq.seqMode == MAX11261_SEQ_MODE_1) {
                if (reg & MAX11261_STAT_RDY)
                    break;
            } else {
                /* RDY bit is invalid in modes 2, 3 and 4 */
                if ((reg & (seq.srdy << MAX11261_STAT_SRDY_POS))
                    == (seq.srdy << MAX11261_STAT_SRDY_POS))
                    break;
            }
            if (seq.convMode != MAX11261_LATENT_CONTINUOUS)
                platCtx.delayUs(max11261_single_cycle_delay[seq.srate]);
            else
                platCtx.delayUs(max11261_cont_cycle_delay[seq.crate]);
            MAX11261_READ_REG(MAX11261_STAT, &reg);
        }
    }
    if (to == 0)
        return -ETIMEDOUT;

    MAX11261_READ_REG(MAX11261_FIFO_LEVEL, &cnt);
    if (seq.seqMode == MAX11261_SEQ_MODE_4)
        MAX11261_READ_REG(MAX11261_INT_STAT, &reg);

    while (cnt-- && count--) {
        MAX11261_READ_REG(MAX11261_STAT, &reg);
        if (reg & MAX11261_STAT_GPOERR)
            return -EPERM;

        res->dor = !!(reg & MAX11261_STAT_DOR);
        res->aor = !!(reg & MAX11261_STAT_AOR);

        MAX11261_READ_REG(MAX11261_FIFO, &reg);
        res->chn = (reg & MAX11261_FIFO_CH) >> MAX11261_FIFO_CH_POS;
        res->oor = (reg & MAX11261_FIFO_OOR) >> MAX11261_FIFO_OOR_POS;
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
        res++;
        rd++;
    }

    return rd;
}

int max11261_adc_convert(void)
{
    int error;

    /* Start conversion */
    if (seq.convMode != MAX11261_LATENT_CONTINUOUS)
        error = max11261_write_byte(MAX11261_CMD_SEQUENCER(seq.srate));
    else
        error = max11261_write_byte(MAX11261_CMD_SEQUENCER(seq.crate));

    if (error < 0)
        return error;

    return 0;
}
