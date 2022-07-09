/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
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
 *******************************************************************************
 */

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <i2c.h>
#include <mxc_device.h>
#include <stddef.h>
#include <stdint.h>


//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define I2C_FREQ 400000

#define MAX9867_ADDR 0x18

#define MAX9867_REVID 0x42

// Register addresses
#define MAX9867_00_STATUS            0x00
#define MAX9867_01_JACKSENSE         0x01
#define MAX9867_02_AUX_HIGH          0x02
#define MAX9867_03_AUX_LOW           0x03
#define MAX9867_04_INT_EN            0x04
#define MAX9867_05_SYS_CLK           0x05
#define MAX9867_06_CLK_HIGH          0x06
#define MAX9867_07_CLK_LOW           0x07
#define MAX9867_08_DAI_FORMAT        0x08
#define MAX9867_09_DAI_CLOCK         0x09
#define MAX9867_0A_DIG_FILTER        0x0A
#define MAX9867_0B_SIDETONE          0x0B
#define MAX9867_0C_LVL_DAC           0x0C
#define MAX9867_0D_LVL_ADC           0x0D
#define MAX9867_0E_LVL_LINE_IN_LEFT  0x0E
#define MAX9867_0F_LVL_LINE_IN_RIGHT 0x0F
#define MAX9867_10_VOL_LEFT          0x10
#define MAX9867_11_VOL_RIGHT         0x11
#define MAX9867_12_MIC_GAIN_LEFT     0x12
#define MAX9867_13_MIC_GAIN_RIGHT    0x13
#define MAX9867_14_ADC_INPUT         0x14
#define MAX9867_15_MIC               0x15
#define MAX9867_16_MODE              0x16
#define MAX9867_17_PWR_SYS           0x17
#define MAX9867_FF_REV_ID            0xFF

// MAX9867_04_INT_EN
#define MAX9867_ICLD   (1<<7)
#define MAX9867_SDODLY (1<<2)

// MAX9867_05_SYS_CLK
#define MAX9867_PSCLK_POS 4

// MAX9867_06_CLK_HIGH
#define MAX9867_PLL (1<<7)
#define MAX9867_NI_UPPER_48KHZ      0x60

// MAX9867_07_CLK_LOW
#define MAX9867_NI_LOWER_48KHZ      0x00

// MAX9867_08_DAI_FORMAT
#define MAX9867_MAS    (1<<7)
#define MAX9867_WCI    (1<<6)
#define MAX9867_BCI    (1<<5)
#define MAX9867_DLY    (1<<4)
#define MAX9867_HIZOFF (1<<3)
#define MAX9867_TDM    (1<<2)

// MAX9867_09_DAI_CLOCK
#define MAX9867_BSEL_PCLK_DIV8      0x06

// MAX9867_0D_LVL_ADC
#define MAX9867_AVL_POS 4
#define MAX9867_AVR_POS 0

// MAX9867_0E_LVL_LINE_IN_LEFT
// MAX9867_0F_LVL_LINE_IN_RIGHT
#define MAX9867_LI_MUTE (1<<6)
#define MAX9867_LI_GAIN_POS 0

// MAX9867_10_VOL_LEFT
// MAX9867_11_VOL_RIGHT
#define MAX9867_VOL_POS 0

// MAX9867_14_ADC_INPUT
#define MAX9867_MXINL_POS 6
#define MAX9867_MXINR_POS 4
#define MAX9867_MXIN_DIS 0
#define MAX9867_MXIN_ANALOG_MIC 1
#define MAX9867_MXIN_LINE 2

// MAX9867_15_MIC
#define MAX9867_MICCLK_POS 6
#define MAX9867_DIGMICL_POS 5
#define MAX9867_DIGMICR_POS 4

// MAX9867_16_MODE
#define MAX9867_HPMODE_POS 0
#define MAX9867_STEREO_SE_CLICKLESS 4
#define MAX9867_MONO_SE_CLICKLESS   5

// MAX9867_17_PWR_SYS
#define MAX9867_SHDN  (1<<7)
#define MAX9867_LNLEN (1<<6)
#define MAX9867_LNREN (1<<5)
#define MAX9867_DALEN (1<<3)
#define MAX9867_DAREN (1<<2)
#define MAX9867_ADLEN (1<<1)
#define MAX9867_ADREN (1<<0)

#define MHZ(N) (N*1000000)


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
static mxc_i2c_req_t i2c_req;


//-----------------------------------------------------------------------------
// Local function declarations
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Function definitions
//-----------------------------------------------------------------------------
static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_len = 0;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

static int reg_read(uint8_t reg, uint8_t *dest)
{
    uint8_t buf[1] = { reg };

    i2c_req.tx_buf = buf;
    i2c_req.tx_len = sizeof(buf);
    i2c_req.rx_buf = dest;
    i2c_req.rx_len = 1;

    return MXC_I2C_MasterTransaction(&i2c_req);
}

int max9867_status(void)
{
    int err;
    uint8_t status;

    if ((err = reg_read(MAX9867_00_STATUS, &status)) != E_NO_ERROR) return err;

    return status;
}

/* Configure I2C master and MAX9867 for I2S data interface */
int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int lrclk)
{
    int err;
    uint8_t rev_id;
    int psclk;

    // Initialize I2C peripheral
    if ((err = MXC_I2C_Init(i2c_inst, TRUE, 0)) != E_NO_ERROR) return err;
    MXC_I2C_SetFrequency(i2c_inst, I2C_FREQ);

    // Initialize static I2C request members
    i2c_req.i2c = i2c_inst;
    i2c_req.addr = MAX9867_ADDR;
    i2c_req.restart = 0;
    i2c_req.callback = NULL;

    // Probe for MAX9867
    if ((err = reg_read(MAX9867_FF_REV_ID, &rev_id)) != E_NO_ERROR) return err;
    if (rev_id != MAX9867_REVID) return E_NOT_SUPPORTED;

    /*
        Hardware configuration -- Digital Audio Interface
    */

    // Shutdown MAX9867 during configuration
    if ((err = reg_write(MAX9867_17_PWR_SYS, 0x00)) != E_NO_ERROR) return err;

    /*  Clear all regs to POR state.
        The MAX9867 does not not have an external reset signal. */
    for (int i = MAX9867_04_INT_EN; i < MAX9867_17_PWR_SYS; i++) {
        if ((err = reg_write(i, 0x00)) != E_NO_ERROR) return err;
    }

    // Select MCLK prescaler. PSCLK divides MCLK to generate a PCLK between 10MHz and 20MHz.
    if ((mclk < MHZ(10)) || (mclk > MHZ(60))) return E_INVALID;
    else if (mclk < MHZ(20)) psclk = 0x1;
    else if (mclk < MHZ(40)) psclk = 0x2;
    else psclk = 0x3;

    // Set prescaler, FREQ field is 0 for Normal or PLL mode
    if ((err = reg_write(MAX9867_05_SYS_CLK, psclk << MAX9867_PSCLK_POS)) != E_NO_ERROR) return err;

    // // Enable PLL, NI[14:8] is 0 for PLL mode
    // if ((err = reg_write(MAX9867_06_CLK_HIGH, MAX9867_PLL)) != E_NO_ERROR) return err;

    // Configure codec to generate 48kHz sampling frequency in master mode
    if ((err = reg_write(MAX9867_06_CLK_HIGH, MAX9867_NI_UPPER_48KHZ)) != E_NO_ERROR) return err;
    if ((err = reg_write(MAX9867_07_CLK_LOW, MAX9867_NI_LOWER_48KHZ)) != E_NO_ERROR) return err;
    if ((err = reg_write(MAX9867_09_DAI_CLOCK, MAX9867_BSEL_PCLK_DIV8)) != E_NO_ERROR) return err;


    // I2S format
    if ((err = reg_write(MAX9867_08_DAI_FORMAT, MAX9867_MAS | MAX9867_DLY | MAX9867_HIZOFF)) != E_NO_ERROR) return err;

    // Left Justified
    // if ((err = reg_write(MAX9867_08_DAI_FORMAT, MAX9867_HIZOFF)) != E_NO_ERROR) return err;
    // if ((err = reg_write(MAX9867_04_INT_EN, MAX9867_SDODLY)) != E_NO_ERROR) return err;

    /*
        Application configuration -- Stream Interface
    */

    if ((err = reg_write(MAX9867_0A_DIG_FILTER, 0xA2)) != E_NO_ERROR) return err;

    // Select Digital microphone input
    if ((err = reg_write(MAX9867_15_MIC, ((0x1 << MAX9867_DIGMICR_POS)))) != E_NO_ERROR) return err;

    // ADC level
    if ((err = reg_write(MAX9867_0D_LVL_ADC, (3 << MAX9867_AVL_POS) | (3 << MAX9867_AVR_POS))) != E_NO_ERROR) return err;

    // Set line-in level, disconnect line input from playback amplifiers
    if ((err = reg_write(MAX9867_0E_LVL_LINE_IN_LEFT, (0x0C << MAX9867_LI_GAIN_POS) | MAX9867_LI_MUTE)) != E_NO_ERROR) return err;
    if ((err = reg_write(MAX9867_0F_LVL_LINE_IN_RIGHT, (0x0C << MAX9867_LI_GAIN_POS) | MAX9867_LI_MUTE)) != E_NO_ERROR) return err;

    // Headphone mode
    if ((err = reg_write(MAX9867_16_MODE, MAX9867_STEREO_SE_CLICKLESS << MAX9867_HPMODE_POS)) != E_NO_ERROR) return err;

    // Set playback volume
    if ((err = reg_write(MAX9867_10_VOL_LEFT, 0x04 << MAX9867_VOL_POS)) != E_NO_ERROR) return err;
    if ((err = reg_write(MAX9867_11_VOL_RIGHT, 0x04 << MAX9867_VOL_POS)) != E_NO_ERROR) return err;

    // Enable device
    // return reg_write(MAX9867_17_PWR_SYS, MAX9867_SHDN | MAX9867_LNLEN | MAX9867_ADLEN | MAX9867_DALEN | MAX9867_LNREN | MAX9867_ADREN | MAX9867_DAREN);
    return reg_write(MAX9867_17_PWR_SYS, MAX9867_SHDN | MAX9867_DALEN | MAX9867_DAREN | MAX9867_ADLEN);
}

int max9867_shutdown(void)
{
    int err;

    // I2S format
    if ((err = reg_write(MAX9867_08_DAI_FORMAT, 0)) != E_NO_ERROR) return err;

    // Disable device
    return reg_write(MAX9867_17_PWR_SYS, 0);
}
