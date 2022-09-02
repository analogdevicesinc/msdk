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

#include <stdint.h>
#include <stdio.h>
#include "spi.h"
#include "afe.h"
#include "mxc_sys.h"

#include "afe_gpio.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"

// AFE SPI Port Configuration
#if (TARGET != MAX32675 || TARGET_NUM == 32675)
#define AFE_SPI_PORT MXC_SPI0
#elif (TARGET != MAX32680 || TARGET_NUM == 32680)
#define AFE_SPI_PORT MXC_SPI1
#endif

#define AFE_SPI_BAUD        100000 // Can only run up to PCLK speed
#define AFE_SPI_BIT_WIDTH   8
#define AFE_SPI_SSEL_PIN    1
#define AFE_SPI_ADDRESS_LEN 1

// AFE Trim Storage Defines
//#define DUMP_TRIM_DATA

#if (TARGET != MAX32675 || TARGET_NUM == 32675)
#define AFE_TRIM_OTP_OFFSET_LOW  0x280
#define AFE_TRIM_OTP_OFFSET_HIGH 0x288
#elif (TARGET != MAX32680 || TARGET_NUM == 32680)
#define AFE_TRIM_OTP_OFFSET_LOW  0x0E10
#define AFE_TRIM_OTP_OFFSET_HIGH 0x0E18
#endif

#define AFE_TRIM0_ADC0_MASK      0x7FFFF
#define AFE_TRIM0_ADC0_BIT_WIDTH 19

#define AFE_TRIM1_ADC0_MASK      0x3FFF
#define AFE_TRIM1_ADC0_BIT_WIDTH 14

#define AFE_TRIM_DAC_MASK      0x0FFFF
#define AFE_TRIM_DAC_BIT_WIDTH 16

#define AFE_TRIM_ANA_ADC0_MASK      0x7FFF
#define AFE_TRIM_ANA_ADC0_BIT_WIDTH 15

#define AFE_TRIM0_ADC1_MASK      0x7FFFF
#define AFE_TRIM0_ADC1_BIT_WIDTH 19

#define AFE_TRIM1_ADC1_MASK      0x3FFF
#define AFE_TRIM1_ADC1_BIT_WIDTH 14

#define AFE_TRIM_HART_MASK      0x0FFFFF
#define AFE_TRIM_HART_BIT_WIDTH 20

// NOTE: These two bits are embedded inside the HART trim
#define AFE_TRIM_ANA_ADC1_MASK      0x060
#define AFE_TRIM_ANA_ADC1_BIT_WIDTH 2
#define AFE_TRIM_ANA_ADC1_OFFSET_1  5  // bit position in HART trim
#define AFE_TRIM_ANA_ADC1_OFFSET_2  10 // bit position in ANA TRIM ADC1

#define AFE_TRIM_VREF_MASK      0x7FF
#define AFE_TRIM_VREF_BIT_WIDTH 11

// Largest Possible AFE SPI transaction (BYTES): Address 1, Max Data 4, CRC 2
#define AFE_SPI_MAX_DATA_LEN 7

/***** Globals *****/
uint8_t afe_data[AFE_SPI_MAX_DATA_LEN];
mxc_spi_regs_t* pSPIm = AFE_SPI_PORT;
int check_done        = 0;

typedef struct {
    uint32_t adc_trim0_adc0;
    uint32_t adc_trim0_adc1;
    uint32_t adc_trim1_adc0;
    uint32_t adc_trim1_adc1;
    uint32_t ana_trim_adc0;
    uint32_t vref_trim;
    uint32_t hart_trim;
    uint32_t ana_trim_adc1;
    uint32_t dac_trim;
} trim_data_t;

trim_data_t trim_data;

uint32_t current_register_bank  = 0;
uint32_t adc0_conversion_active = 0;
uint32_t adc1_conversion_active = 0;

#if (TARGET != MAX32675 || TARGET_NUM == 32675)
static mxc_gpio_cfg_t gpio_cfg_spi0 = {
    MXC_GPIO0, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE};
#elif (TARGET != MAX32680 || TARGET_NUM == 32680)
static mxc_gpio_cfg_t gpio_cfg_spi1 = {
    MXC_GPIO0, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE};
#endif

/***** Private Prototypes *****/
static int raw_afe_write_register(uint8_t reg_address, uint32_t value, uint8_t reg_length);
static int raw_afe_read_register(uint8_t reg_address, uint32_t* value, uint8_t reg_length);

/***** Functions *****/
static int afe_spi_setup(void)
{
    int retval = 0;

    // Enable SPI Periph clock, and reset it
#if (TARGET != MAX32675 || TARGET_NUM == 32675)
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI0);

    retval = MXC_AFE_GPIO_Config(&gpio_cfg_spi0);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#elif (TARGET != MAX32680 || TARGET_NUM == 32680)
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);

    retval = MXC_AFE_GPIO_Config(&gpio_cfg_spi1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
#endif

    // NOTE: AFE uses SPI Mode 0, which is reset default

    // Use SDK SPI driver to set the baud rate, as these depends what periph clock we are at
    retval = MXC_SPI_SetFrequency(AFE_SPI_PORT, AFE_SPI_BAUD);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    pSPIm->ctrl2 |= ((AFE_SPI_BIT_WIDTH << MXC_F_SPI_CTRL2_NUMBITS_POS) & MXC_F_SPI_CTRL2_NUMBITS);

    pSPIm->ctrl0 |= (AFE_SPI_SSEL_PIN << MXC_F_SPI_CTRL0_SS_ACTIVE_POS);

    pSPIm->ctrl2 |= MXC_S_SPI_CTRL2_DATA_WIDTH_MONO;

    pSPIm->sstime = ((1 << MXC_F_SPI_SSTIME_PRE_POS) | (1 << MXC_F_SPI_SSTIME_POST_POS) |
                     (1 << MXC_F_SPI_SSTIME_INACT_POS));

    pSPIm->ctrl0 |= (MXC_F_SPI_CTRL0_EN | MXC_F_SPI_CTRL0_MST_MODE);

    pSPIm->dma = (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    // FIFO Threshold reset defaults are fine
    pSPIm->dma |= MXC_F_SPI_DMA_TX_FIFO_EN;
    pSPIm->dma |= MXC_F_SPI_DMA_RX_FIFO_EN;

    // Clear any existing interrupt status
    pSPIm->intfl = pSPIm->intfl;

    return E_NO_ERROR;
}

// This function block until transceive is completed
// TODO: Consider checking for timeout
static int afe_spi_transceive(uint8_t* data, int byte_length)
{
    int i = 0;

    if (byte_length > AFE_SPI_MAX_DATA_LEN) {
        return E_OVERFLOW;
    }

    while (pSPIm->dma & MXC_F_SPI_DMA_TX_LVL) {
    }

    if (check_done) {
        while (!(pSPIm->intfl & MXC_F_SPI_INTFL_MST_DONE))
            ;
    }

    check_done = 1;

    pSPIm->intfl = pSPIm->intfl;
    pSPIm->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    while (pSPIm->dma & (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH))
        ;

    pSPIm->ctrl1 = ((((byte_length) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS)) |
                    (byte_length << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS));

    pSPIm->ctrl0 |= MXC_F_SPI_CTRL0_START;

    //
    // Transmit the data
    //
    for (i = 0; i < byte_length; i++) {
        pSPIm->fifo8[0] = data[i];
    }

    //
    // Receive the data
    //
    for (i = 0; i < byte_length; i++) {
        // Wait for data to be available
        while (!(pSPIm->dma & MXC_F_SPI_DMA_RX_LVL))
            ;

        data[i] = pSPIm->fifo8[0];
    }

    return E_NO_ERROR;
}

int afe_setup(void)
{
    int retval        = 0;
    uint32_t read_val = 0;

    retval = afe_spi_setup();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Disable CRC for all reads, and Ensure bank is ADC0
    // NOTE: CRC works, but takes extra time which is in short supply at 2Mhz.
    // TODO: Add optional support for CRC5 of register reads.
    read_val = MXC_S_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL_ADC0_BANK;

    retval = raw_afe_write_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = afe_read_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, &read_val);

    return retval;
}

static int raw_afe_write_register(uint8_t reg_address, uint32_t value, uint8_t reg_length)
{
    int i      = 0;
    int retval = 0;

    int txLen = AFE_SPI_ADDRESS_LEN + reg_length;

    // First comes address
    // AFE requires bit 7 of byte 0 CLEAR to indicate write command
    afe_data[0] = reg_address & ~AFE_REG_ADDR_READ_BIT;

    // Next data to write MSB first
    for (i = reg_length; i > 0; i--) {
        afe_data[i] = value & 0x0FF;
        value >>= 8;
    }

    // Blocking SPI Transceive
    retval = afe_spi_transceive(afe_data, txLen);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int raw_afe_read_register(uint8_t reg_address, uint32_t* value, uint8_t reg_length)
{
    int i      = 0;
    int retval = 0;
    int txLen  = AFE_SPI_ADDRESS_LEN + reg_length;

    // First comes address
    // AFE requires bit 7 of byte 0 SET to indicate read command
    afe_data[0] = reg_address | AFE_REG_ADDR_READ_BIT;

    // Blocking SPI Transceive
    retval = afe_spi_transceive(afe_data, txLen);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    *value = 0;

    // Extract value
    for (i = 0; i < reg_length; i++) {
        *value <<= 8;
        *value |= afe_data[i + AFE_SPI_ADDRESS_LEN] & 0x0FF;
    }

    // TODO: If optional CRC5 is added in the future, verify CRC here

    return retval;
}

static int afe_bank_select(uint8_t bank_num)
{
    uint32_t read_val = 0;
    int retval        = 0;

    // First, No need to check for current bank if we are already in it.
    if (current_register_bank == bank_num) {
        return E_NO_ERROR;
    }

    // Read current value, and or in our bits
    // NOTE: SYC_CTRL exists in all register banks
    //
    // BIG NOTE: READING of this register arms the SPI_ABORT_DIS feature
    //
    retval = raw_afe_read_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // BIG NOTE: writing a register on the AFE during a ADC conversion will ABORT it.
    // If we are already in the correct bank, then do NOT write the SYS_CTRL register
    //

    //
    // NOTE: As a workaround to allow for multiple ADC conversions, one in ADC0 and simultaneous one in ADC1
    //	The SPI_ABORT_DIS feature in the sys_ctrl register can help.  Note that there is only 1 sys_cntl register
    //	but it is available from every register bank, ADC0, ADC1, DAC, and HART.
    //
    // Reading the sys_ctrl register arms the HW to set this bit automatically, it doesn't need to be set by SW.
    //	WARNING: It blocks the abort signal. This means, if a conversion is active on the selected ADC, and
    //	after reading sys_ctrl, a write occurs to ADC bank with conversion in progress, the ADC data will
    //	be corrupted.
    //

    if ((read_val & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL) !=
        (bank_num & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL)) {
        // Need to change the bank
        read_val = (read_val & ~MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL) |
                   (bank_num & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL);

        retval = raw_afe_write_register(
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, read_val,
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

        if (retval != E_NO_ERROR) {
            return retval;
        }

        // Update bank tracker
        current_register_bank = bank_num;
    }
    // Else already in the correct bank

    return retval;
}

int afe_write_register(uint32_t target_reg, uint32_t value)
{
    uint8_t reg_bank    = 0;
    uint8_t reg_address = 0;
    uint8_t reg_length  = 0;
    int retval          = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into ana_src_sel[1:0] as bank select
    // Bottom byte of address encodes the register width in bytes 1 - 4 (8bits to 32bits)

    reg_length  = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_bank    = (target_reg & AFE_REG_ADDR_BANK) >> AFE_REG_ADDR_BANK_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = raw_afe_write_register(reg_address, value, reg_length);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

int afe_read_register(uint32_t target_reg, uint32_t* value)
{
    uint8_t reg_bank    = 0;
    uint8_t reg_address = 0;
    uint8_t reg_length  = 0;
    int retval          = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into ana_src_sel[1:0] as bank select
    // Bottom byte of address encodes the register width in bytes 1 - 4 (8bits to 32bits)

    reg_length  = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_bank    = (target_reg & AFE_REG_ADDR_BANK) >> AFE_REG_ADDR_BANK_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = raw_afe_read_register(reg_address, value, reg_length);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

int afe_load_trims(void)
{
    int retval = 0;
    uint8_t info_buf[INFOBLOCK_LINE_SIZE];
    uint64_t afe_trim_low  = 0;
    uint64_t afe_trim_high = 0;
#ifdef DUMP_TRIM_DATA
    uint32_t read_val = 0;
#endif

    // setup the interface before we begin
    afe_setup();

    //
    // Read in AFE Trims
    //
    retval = infoblock_read(AFE_TRIM_OTP_OFFSET_LOW, info_buf, INFOBLOCK_LINE_SIZE);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    for (int i = 7; i >= 0; i--) {
        afe_trim_low <<= 8;
        afe_trim_low |= info_buf[i];
    }

    retval = infoblock_read(AFE_TRIM_OTP_OFFSET_HIGH, info_buf, INFOBLOCK_LINE_SIZE);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    for (int i = 7; i >= 0; i--) {
        afe_trim_high <<= 8;
        afe_trim_high |= info_buf[i];
    }

    //
    // Parse out individual trims
    //
    // afe_trim_low
    trim_data.adc_trim0_adc0 = afe_trim_low & AFE_TRIM0_ADC0_MASK;
    afe_trim_low >>= AFE_TRIM0_ADC0_BIT_WIDTH;

    trim_data.adc_trim1_adc0 = afe_trim_low & AFE_TRIM1_ADC0_MASK;
    afe_trim_low >>= AFE_TRIM1_ADC0_BIT_WIDTH;

    trim_data.dac_trim = afe_trim_low & AFE_TRIM_DAC_MASK;
    afe_trim_low >>= AFE_TRIM_DAC_BIT_WIDTH;

    trim_data.ana_trim_adc0 = afe_trim_low & AFE_TRIM_ANA_ADC0_MASK;

    // afe_trim_high
    trim_data.adc_trim0_adc1 = afe_trim_high & AFE_TRIM0_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM0_ADC1_BIT_WIDTH;

    trim_data.adc_trim1_adc1 = afe_trim_high & AFE_TRIM1_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM1_ADC1_BIT_WIDTH;

    // Now got to take care of the ANA_TRIM_ADC1 which is embedded inside the HART trim
    trim_data.ana_trim_adc1 = afe_trim_high & AFE_TRIM_ANA_ADC1_MASK;
    trim_data.ana_trim_adc1 >>= AFE_TRIM_ANA_ADC1_OFFSET_1;
    trim_data.ana_trim_adc1 <<= AFE_TRIM_ANA_ADC1_OFFSET_2;

    trim_data.hart_trim = afe_trim_high & AFE_TRIM_HART_MASK;
    // Force the embedded ANA trim to zeros.
    trim_data.hart_trim &= ~AFE_TRIM_ANA_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM_HART_BIT_WIDTH;

    trim_data.vref_trim = afe_trim_high & AFE_TRIM_VREF_MASK;

    //
    // Dump Trims
    //
#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  Trim Values Read from info block\n");
    printf("\n****************************************************\n");

    printf("ADC0 adc trim 0: %08X\n", trim_data.adc_trim0_adc0);
    printf("ADC0 adc trim 1: %08X\n", trim_data.adc_trim1_adc0);
    printf("DAC trim: %08X\n", trim_data.dac_trim);
    printf("ANA ADC0 trim: %08X\n", trim_data.ana_trim_adc0);

    printf("ADC1 adc trim 0: %08X\n", trim_data.adc_trim0_adc1);
    printf("ADC1 adc trim 1: %08X\n", trim_data.adc_trim1_adc1);
    printf("HART trim: %08X\n", trim_data.hart_trim);
    printf("ANA ADC1 trim: %08X\n", trim_data.ana_trim_adc1);
    printf("VREF trim: %08X\n", trim_data.vref_trim);
#endif

    //
    // Before Trimming, reset AFE to known state
    //

    // NOTE: SYS_CTRL is a global register, avaliable from all banks
    // NOTE: POR Flag can only be written to 1, this indicates NO POR has occured since setting to 1
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Reset ADCs, as the MAX32675 Reset has no effect on them
    // After restoring POR defaults, ADCS enter Standby mode
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_PD, MXC_S_AFE_ADC_ZERO_PD_PD_RESET);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_PD, MXC_S_AFE_ADC_ONE_PD_PD_RESET);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // Write Trims
    //

    // Unlock trim for ADC0
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD,
                                MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD,
                                MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_2);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Unlock trim for ADC1
    retval =
        afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, MXC_V_AFE_ADC_ONE_FT_PWORD_FT_PWORD_PWORD_1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval =
        afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, MXC_V_AFE_ADC_ONE_FT_PWORD_FT_PWORD_PWORD_2);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  AFE Trim Register before Writing (FT UNLOCKED)\n");
    printf("\n****************************************************\n");

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, &read_val);
    printf("ADC0 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, &read_val);
    printf("ADC0 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_TRIM, &read_val);
    printf("DAC trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, &read_val);
    printf("ANA ADC0 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, &read_val);
    printf("ADC1 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, &read_val);
    printf("ADC1 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_HART_TRIM, &read_val);
    printf("HART trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, &read_val);
    printf("ANA ADC1 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_TRIM, &read_val);
    printf("VREF trim: %08X\n", read_val);

    printf("\n****************************************************\n");
    printf("  Writing Trim Values Into AFE\n");
    printf("\n****************************************************\n");
#endif

    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, trim_data.adc_trim0_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, trim_data.adc_trim1_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_DAC_TRIM, trim_data.dac_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, trim_data.ana_trim_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, trim_data.adc_trim0_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, trim_data.adc_trim1_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_HART_TRIM, trim_data.hart_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, trim_data.ana_trim_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_DAC_VREF_TRIM, trim_data.vref_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  AFE Trim Register after Writing (FT UNLOCKED)\n");
    printf("\n****************************************************\n");

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, &read_val);
    printf("ADC0 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, &read_val);
    printf("ADC0 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_TRIM, &read_val);
    printf("DAC trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, &read_val);
    printf("ANA ADC0 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, &read_val);
    printf("ADC1 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, &read_val);
    printf("ADC1 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_HART_TRIM, &read_val);
    printf("HART trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, &read_val);
    printf("ANA ADC1 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_TRIM, &read_val);
    printf("VREF trim: %08X\n", read_val);
#endif

    // Lock Trims
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, 0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD, 0);

    return retval;
}

void afe_dump_registers(uint32_t reg_bank)
{
    uint32_t reg_add  = 0;
    uint32_t reg_len  = 0;
    uint32_t read_val = 0;

    if (reg_bank == AFE_ADC0_BANK) {
        printf("Dumping registers of AFE ADC0 Bank\n");
    } else if (reg_bank == AFE_ADC1_BANK) {
        printf("Dumping registers of AFE ADC1 Bank\n");
    } else if (reg_bank == AFE_DAC_BANK) {
        printf("Dumping registers of AFE DAC Bank\n");
    } else if (reg_bank == AFE_HART_BANK) {
        printf("Dumping registers of AFE HART Bank\n");
    } else {
        printf("Unknown AFE bank number.\n");
        return;
    }

    if ((reg_bank == AFE_ADC0_BANK) || (reg_bank == AFE_ADC1_BANK)) {
        reg_len = 1;

        // 8 Bit Registers
        printf("\n** 8-bit Control Registers 0x00 - 0x10 **\n");
        for (reg_add = 0; reg_add < 0x11; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%02X\n", reg_add, read_val);
        }

        // 24 bit Registers
        printf("\n** 24-bit Control, Data, Status Registers 0x11 - 0x39 **\n");
        reg_len = 3;
        for (reg_add = 0x11; reg_add < 0x3A; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }

        // 16 bit Registers
        printf("\n** 16-bit Sequencer Registers 0x3A - 0x6E **\n");
        reg_len = 2;
        for (reg_add = 0x3A; reg_add < 0x6F; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%04X\n", reg_add, read_val);
        }

        // UCADDR is only 8 bits
        printf("\n** 8-bit UCADDR Register **\n");
        reg_len = 1;
        reg_add = 0x6F;
        afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                              (reg_len << AFE_REG_ADDR_LEN_POS),
                          &read_val);
        printf("Reg 0x%02X is 0x%02X\n", 0x6F, read_val);
    } // End of ADC Register Dump

    if (reg_bank == AFE_DAC_BANK) {
        reg_len = 4; // 32 Bit registers
        printf("\n** 32-bit DAC Registers 0x00 - 0x04 **\n");
        for (reg_add = 0x00; reg_add < 0x05; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%08X\n", reg_add, read_val);
        }

        reg_len = 3; // 24 Bit Registers
        printf("\n** 24-bit DAC Registers 0x05 - 0x07 **\n");
        for (reg_add = 0x05; reg_add < 0x08; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }
    } // End of ADC DAC Register Dump

    if (reg_bank == AFE_HART_BANK) {
        reg_len = 3; // 24 Bit registers
        printf("\n** 24-bit HART Registers 0x00 - 0x0B **\n");
        for (reg_add = 0x00; reg_add < 0x0C; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }
    } // End of ADC HART Register Dump

    printf("\n\n");
}
