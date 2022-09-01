/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mxc_delay.h"
#include "mxc_device.h"

#include "gpio.h"
#include "spi.h"
#include "tft.h"

/************************************ DEFINES ********************************/
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define TFT_SPI_FREQ 12000000 // Hz
#define TFT_SPI1_PINS MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_20
#define PALETTE_OFFSET(x)                                                                          \
    concat(images_start_addr + images_header.offset2info_palatte                                   \
            + 1 /* nb_palette */ + (x) * sizeof(unsigned int),                                     \
        4)
#define FONT_OFFSET(x)                                                                             \
    concat(images_start_addr + images_header.offset2info_font                                      \
            + 1 /* nb_font    */ + (x) * sizeof(unsigned int),                                     \
        4)
#define BITMAP_OFFSET(x)                                                                           \
    concat(images_start_addr + images_header.offset2info_bitmap                                    \
            + 1 /* nb_bitmap  */ + (x) * sizeof(unsigned int),                                     \
        4)

/********************************* TYPE DEFINES ******************************/
#pragma pack(1)

typedef struct {
    unsigned int w;
    unsigned int h;
    unsigned char id_palette;
    unsigned char rle;
    unsigned int data_size;
} bitmap_info_t;

typedef struct {
    unsigned short x;
    unsigned char w;
    unsigned char hex_code;
} font_char_t;

typedef struct {
    // unsigned char size;
    unsigned char nb_char;
    unsigned char bitmap_id;
    // font_char_t    *char_info; // X nb_char
} font_info_t;

typedef struct {
    // unsigned short    type;                        /* Magic identifier               */
    // unsigned short    reserved1, reserved2;
    // unsigned short    bits;                        /* Bits per pixel                 */
    //
    unsigned int offset2info_palatte; // Palettes start address
    unsigned int offset2info_font; // Fonts start address
    unsigned int offset2info_bitmap; // Bitmap start address
    //
    unsigned int nb_palette; // number of palette
    unsigned int nb_font; // number of fonts
    unsigned int nb_bitmap; // number of bitmap
} Header_images_t;

#pragma pack()

extern unsigned int _bin_start_; // binary start address, defined in linker file
static unsigned char* images_start_addr = NULL;
static Header_images_t images_header;

static unsigned int g_background_color;
static unsigned int g_palette_ram[256];
static unsigned int g_fifo[4];

static unsigned int cursor_x;
static unsigned int cursor_y;

static area_t pf_area;
static int g_font_id = 0;

static tft_rotation_t tft_rotation = SCREEN_NORMAL;

static mxc_spi_regs_t* spi;
static int ssel;
static mxc_gpio_cfg_t* reset_pin;
static mxc_gpio_cfg_t* blen_pin;

/********************************* Static Functions **************************/
static int concat(unsigned char* var, int size)
{
    int result = 0;

    for (int i = 1; i <= size; i++) {
        result |= var[size - i] << (8 * (size - i));
    }

    return result;
}

static void get_font_info(int font_id, font_info_t* font_info, font_char_t** chr_pos)
{
    unsigned int offset;

    offset = FONT_OFFSET(font_id);
    memcpy(font_info, (images_start_addr + offset), sizeof(font_info_t));
    *chr_pos = (font_char_t*)(images_start_addr + offset + sizeof(font_info_t));
}

static void get_bitmap_info(int bitmap_id, bitmap_info_t* bitmap_info, unsigned char** pixel)
{
    unsigned int offset;

    offset = BITMAP_OFFSET(bitmap_id);
    memcpy(bitmap_info, (images_start_addr + offset), sizeof(bitmap_info_t));
    *pixel = (unsigned char*)((images_start_addr + offset + sizeof(bitmap_info_t)));
}

#if 0
static void spi_transmit(void* datain, unsigned int count)
{
    mxc_spi_req_t request = {
        spi,    // spi
        ssel,       // ssIdx
        1,      // ssDeassert
        (uint8_t*) datain, // txData
        NULL,       // rxData
        count,      // txCnt
        0       // rxCnt
    };
    
    MXC_SPI_MasterTransaction(&request);
}
#else
static void spi_transmit(void* datain, unsigned int count)
{
    unsigned int offset;
    unsigned int fifo;
    volatile unsigned short* u16ptrin = (volatile unsigned short*)datain;
    unsigned int start = 0;

    MXC_SPI_SetFrequency(spi, TFT_SPI_FREQ);
    MXC_SPI_SetDataSize(spi, 9);

    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    spi->ctrl0 &= ~(MXC_F_SPI_CTRL0_EN);

    spi->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_ACTIVE;

    // Setup the slave select
    MXC_SETFIELD(
        spi->ctrl0, MXC_F_SPI_CTRL0_SS_ACTIVE, ((1 << ssel) << MXC_F_SPI_CTRL0_SS_ACTIVE_POS));

    // number of RX Char is 0xffff
    spi->ctrl1 &= ~(MXC_F_SPI_CTRL1_RX_NUM_CHAR);

    // DMA RX FIFO disabled
    spi->dma &= ~(MXC_F_SPI_DMA_RX_FIFO_EN);

    // set number of char to be transmit
    MXC_SETFIELD(spi->ctrl1, MXC_F_SPI_CTRL1_TX_NUM_CHAR, count << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    // DMA TX fifo enable
    spi->dma |= MXC_F_SPI_DMA_TX_FIFO_EN;

    /* Clear TX and RX FIFO in DMA
        TX: Set this bit to clear the TX FIFO and all TX FIFO flags in the QSPIn_INT_FL register.
            Note: The TX FIFO should be disabled (QSPIn_DMA.tx_fifo_en = 0) prior to setting this
       field. Note: Setting this field to 0 has no effect. RX: Clear the RX FIFO and any pending RX
       FIFO flags in QSPIn_INTFL. This should be done when the RX FIFO is inactive.
    */
    spi->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);
    // QSPIn port is enabled
    spi->ctrl0 |= (MXC_F_SPI_CTRL0_EN);

    // Clear master done flag
    spi->intfl = MXC_F_SPI_INTFL_MST_DONE;

    /* Loop until all data is transmitted */
    offset = 0;

    do {
        fifo = (count > 8) ? 8 : count;
        count -= fifo;

        while (fifo > 0) {
            /* Send data */
            spi->fifo16[0] = u16ptrin[offset];
            offset++;
            fifo--;
        }

        /*
            Master Start Data Transmission
                Set this field to 1 to start a SPI master mode transaction.
                0: No master mode transaction active.
                1: Master initiates a data transmission. Ensure that all pending transactions are
                complete before setting this field to 1.
                Note: This field is only used when the QSPIn is configured for Master Mode
                (QSPIn_CTRL0.master = 1).
        */
        if (start == 0) {
            spi->ctrl0 |= MXC_F_SPI_CTRL0_START;
            start = 1;
        }

        /* Wait for data transmitting complete and then Deasserts nSS I/O */
        // Deassert slave select at the end of the transaction
        spi->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    } while (count);

    while (!(spi->intfl & MXC_F_SPI_INTFL_MST_DONE)) {
        // wait until done
    }

    return;
}
#endif

static void write_command(unsigned short command)
{
    unsigned short val = command & 0xFF;
    spi_transmit(&val, 1);
}

static void write_data(unsigned short data)
{
    unsigned short val = ((data >> 8) & 0xFF) | 0x0100;

    spi_transmit(&val, 1);
    val = (data & 0xFF) | 0x0100;
    spi_transmit(&val, 1);
}

static void write_color(unsigned int color_palette)
{
    unsigned short val = color_palette & 0xFFFF;

    spi_transmit(&val, 1);
    val = (color_palette >> 16) & 0xFFFF;
    spi_transmit(&val, 1);
}

static void print_line(const unsigned char* line, int nb_of_pixel)
{
    int i;
    int x;
    int loop_counter = nb_of_pixel >> 2; // div 4

    for (x = 0; x < loop_counter; x++) {
        for (i = 0; i < 4; i++) {
            g_fifo[i] = *(g_palette_ram + line[(x << 2) + i]);
        }

        spi_transmit((unsigned short*)g_fifo, 8);
    }

    x <<= 2;

    for (; x < nb_of_pixel; x++) {
        write_color(*(g_palette_ram + line[x]));
    }
}

static void print_line_rgb565(const unsigned char* line, int nb_of_pixel)
{
    line += (nb_of_pixel * 2) - 2;

    for (int i = 0; i < (nb_of_pixel >> 2); i++) {
        g_fifo[0] = (0x01000100 | (line[1] << 16) | line[0]);
        line -= 2;
        g_fifo[1] = (0x01000100 | (line[1] << 16) | line[0]);
        line -= 2;
        g_fifo[2] = (0x01000100 | (line[1] << 16) | line[0]);
        line -= 2;
        g_fifo[3] = (0x01000100 | (line[1] << 16) | line[0]);
        line -= 2;
        spi_transmit((unsigned short*)g_fifo, 8);
    }

    for (int i = 0; i < (nb_of_pixel & 0x03); i++) {
        g_fifo[0] = (0x01000100 | (line[1] << 16) | line[0]);
        line -= 2;
        spi_transmit((unsigned short*)g_fifo, 2);
    }
}

static void RLE_decode(unsigned char const* in, unsigned int length, int img_h, int img_w)
{
    unsigned char cmd, data;
    unsigned int i, inpos;
    unsigned char line[320] = {
        0,
    };
    unsigned int line_start_pos[320] = {
        0,
    };
    int index = 0;
    int is_ended = 0;
    unsigned int nb_of_pixel = 0;

    if (length < 1) {
        return;
    }

    /*
     *  Find start position for each line
     */
    inpos = 0;
    line_start_pos[index++] = inpos;

    do {
        cmd = in[inpos++];
        data = in[inpos++];

        if (cmd == 0x00) {
            switch (data) {
            case 0:
                line_start_pos[index++] = inpos;
                break;

            case 1: // end of image
                is_ended = 1;
                break;

            case 2:
                inpos += 2; // pass x and y
                break;

            default:
                inpos += data;

                if (data % 2) {
                    inpos++;
                }

                break;
            }
        }

        if (is_ended == 1) {
            break;
        } else if (index >= img_h) {
            break;
        }
    } while (inpos < length);

    for (index = index - 1; index >= 0; index--) {
        inpos = line_start_pos[index];
        is_ended = 0;
        nb_of_pixel = 0;

        /* Main decompression loop */
        do {
            cmd = in[inpos++];
            data = in[inpos++];

            if (cmd == 0x00) {
                switch (data) {
                case 0:
                case 1:
                    while (nb_of_pixel < (unsigned int)img_w) {
                        line[nb_of_pixel++] = 0;
                    }

                    print_line(line, img_w);
                    is_ended = 1;
                    break;

                case 2: {
                    unsigned int x, y;
                    x = in[inpos++];
                    y = in[inpos++];

                    for (i = 0; i < x; i++) {
                        line[nb_of_pixel++] = 0;
                    }

                    for (i = 0; i < y; i++) {
                        print_line(line, img_w);
                    }
                } break;

                default:
                    for (i = 0; i < data; i++) {
                        line[nb_of_pixel++] = in[inpos++];
                    }

                    if (data % 2) {
                        inpos++;
                    }

                    break;
                }
            } else {
                for (i = 0; i < cmd; i++) {
                    line[nb_of_pixel++] = data;
                }
            }

            if (is_ended == 1) {
                break;
            }
        } while (inpos < length);
    }
}

static void tft_spi_init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 2;
    int ssPol = 0;
    unsigned int tft_hz = TFT_SPI_FREQ;

    mxc_spi_pins_t tft_pins;

    tft_pins.clock = true;
    tft_pins.ss0 = (ssel == 0); ///< Slave select pin 0
    tft_pins.ss1 = (ssel == 1); ///< Slave select pin 1
    tft_pins.ss2 = (ssel == 2); ///< Slave select pin 2
    tft_pins.miso = true; ///< miso pin
    tft_pins.mosi = true; ///< mosi pin
    tft_pins.sdio2 = false; ///< SDIO2 pin
    tft_pins.sdio3 = false; ///< SDIO3 pin
    tft_pins.vddioh = true; ///< VDDIOHn (3.3V) select for display

    MXC_SPI_Init(spi, master, quadMode, numSlaves, ssPol, tft_hz, tft_pins);

    MXC_SPI_SetDataSize(spi, 9);
    MXC_SPI_SetWidth(spi, SPI_WIDTH_STANDARD);
}

static void displayInit(void)
{
    if (reset_pin) {
        // CLR Reset pin;
        MXC_GPIO_OutClr(reset_pin->port, reset_pin->mask);

        // at least 15usec low
        MXC_Delay(20);

        // SET Reset pin;
        MXC_GPIO_OutSet(reset_pin->port, reset_pin->mask);

        // delay after reset
        MXC_Delay(1000);
    }

    write_command(0x0028); // VCOM OTP
    write_data(0x0006); // Page 55-56 of SSD2119 datasheet

    write_command(0x0000); // start Oscillator
    write_data(0x0001); // Page 36 of SSD2119 datasheet

    write_command(0x0010); // Sleep mode
    write_data(0x0000); // Page 49 of SSD2119 datasheet

    write_command(0x0001); // Driver Output Control
    write_data(0x72EF); // Page 36-39 of SSD2119 datasheet

    write_command(0x0002); // LCD Driving Waveform Control
    write_data(0x0600); // Page 40-42 of SSD2119 datasheet

    write_command(0x0003); // Power Control 1
    write_data(0x6A38); // Page 43-44 of SSD2119 datasheet

    write_command(0x0011); // Entry Mode
    write_data(0x6870); // Page 50-52 of SSD2119 datasheet

    write_command(0X000F); // Gate Scan Position
    write_data(0x0000); // Page 49 of SSD2119 datasheet

    write_command(0X000B); // Frame Cycle Control
    write_data(0x5308); // Page 45 of SSD2119 datasheet

    write_command(0x000C); // Power Control 2
    write_data(0x0003); // Page 47 of SSD2119 datasheet

    write_command(0x000D); // Power Control 3
    write_data(0x000A); // Page 48 of SSD2119 datasheet

    write_command(0x000E); // Power Control 4
    write_data(0x2E00); // Page 48 of SSD2119 datasheet

    write_command(0x001E); // Power Control 5
    write_data(0x00B7); // Page 55 of SSD2119 datasheet

    write_command(0x0025); // Frame Frequency Control
    write_data(0x8000); // Page 53 of SSD2119 datasheet

    write_command(0x0026); // Analog setting
    write_data(0x3800); // Page 54 of SSD2119 datasheet

    write_command(0x0027); // Critical setting to avoid pixel defect
    write_data(0x0078); // per solomon systech, apparently undocumented.

    write_command(0x0048); // Screen driving position - start line
    write_data(0x0000); // Page 59 of SSD2119 datasheet

    write_command(0x0049); // Screen driving position - end line
    write_data(0x00ef); // Page 59 of SSD2119 datasheet

    write_command(0x004E); // Ram Address Set
    write_data(0x0000); // Page 58 of SSD2119 datasheet

    write_command(0x004F); // Ram Address Set
    write_data(0x0000); // Page 58 of SSD2119 datasheet

    write_command(0x0012); // Sleep mode
    write_data(0x0D99); // Page 49 of SSD2119 datasheet

    // Gamma Control (R30h to R3Bh) -- Page 56 of SSD2119 datasheet
    write_command(0x0030);
    write_data(0x0000);

    write_command(0x0031);
    write_data(0x0104);

    write_command(0x0032);
    write_data(0x0100);

    write_command(0x0033);
    write_data(0x0305);

    write_command(0x0034);
    write_data(0x0505);

    write_command(0x0035);
    write_data(0x0305);

    write_command(0x0036);
    write_data(0x0707);

    write_command(0x0037);
    write_data(0x0300);

    write_command(0x003A);
    write_data(0x1200);

    write_command(0x003B);
    write_data(0x0800);

    write_command(0x0007); // Display Control
    write_data(0x0033); // Page 45 of SSD2119 datasheet

    MXC_Delay(150000);

    write_command(0x0022); // RAM data write/read
}

static void setPalette(unsigned char id)
{
    int i;
    unsigned char* palette;

    if (id >= images_header.nb_palette) {
        return;
    }

    palette = (unsigned char*)(images_start_addr + PALETTE_OFFSET(id));

    /* set palette only if it was changed */
    for (i = 0; i < 16; i++) { // only test the first 16
        if (g_palette_ram[i]
            != (0x01000100 | ((palette[0] & 0xF8) << 13) | ((palette[1] & 0x1C) << 19)
                | ((palette[1] & 0xE0) >> 5) | (palette[2] & 0xF8))) {
            goto setup_palette;
        }

        palette += 4;
    }

    return;

setup_palette:

    /** Setup Palette */
    for (; i < 256; i++) {
        g_palette_ram[i] = (0x01000100 | ((palette[0] & 0xF8) << 13) | ((palette[1] & 0x1C) << 19)
            | ((palette[1] & 0xE0) >> 5) | (palette[2] & 0xF8));
        palette += 4;
    }
}

static void displayAll(void)
{
    write_command(0x004E); // RAM address set
    write_data(0x0000); // Page 58 of SSD2119 datasheet
    write_command(0x004F); // RAM address set
    write_data(0x0000); // Page 58 of SSD2119 datasheet
    write_command(0x0044); // Vertical RAM address position
    write_data(0xEF00); // Page 57 of SSD2119 datasheet
    write_command(0x0045); // Horizontal RAM address position
    write_data(0x0000); // Page 57 of SSD2119 datasheet
    write_command(0x0046); // Horizontal RAM address position
    write_data(0x013F); // Page 57 of SSD2119 datasheet
    write_command(0x0022); // RAM data write/read
}

static void displaySub(int x0, int y0, int width, int height)
{
    write_command(0x004E); // RAM address set
    write_data(x0 & 0x1FF); // Page 58 of SSD2119 datasheet
    write_command(0x004F); // RAM address set
    write_data(y0 & 0xFF); // Page 58 of SSD2119 datasheet
    write_command(0x0044); // Vertical RAM address position
    write_data((((y0 + height - 1) & 0xFF) << 8) | (y0 & 0xFF)); // Page 57 of SSD2119 datasheet
    write_command(0x0045); // Horizontal RAM address position
    write_data(x0 & 0x1FF); // Page 57 of SSD2119 datasheet
    write_command(0x0046); // Horizontal RAM address position
    write_data((x0 + width - 1) & 0x1FF); // Page 57 of SSD2119 datasheet
    write_command(0x0022); // RAM data write/read
}

static void displaySub_Rotated(int x0, int y0, int width, int height)
{
    write_command(0x004E); // RAM address set
    write_data(y0 & 0x1FF); // Page 58 of SSD2119 datasheet //ram start horizontal position

    write_command(0x004F); // RAM address set
    write_data((DISPLAY_HEIGHT - x0 - 1)
        & 0xFF); // Page 58 of SSD2119 datasheet //ram start vertical position

    write_command(0x0044); // Vertical RAM address window
    write_data((((DISPLAY_HEIGHT - x0 - 1) & 0xFF) << 8)
        | ((DISPLAY_HEIGHT - x0 - width) & 0xFF)); // Page 57 of SSD2119 datasheet

    write_command(0x0045); // Horizontal RAM address window
    write_data(y0 & 0x1FF); // Page 57 of SSD2119 datasheet
    write_command(0x0046); // Horizontal RAM address position
    write_data((y0 + height - 1) & 0x1FF); // Page 57 of SSD2119 datasheet

    write_command(0x0022); // RAM data write/read
}

static void displaySub_Special(int x0, int y0, int width, int height)
{
    write_command(0x004E); // RAM address set
    write_data(
        (y0 + height - 1) & 0x1FF); // Page 58 of SSD2119 datasheet //ram start horizontal position

    write_command(0x004F); // RAM address set
    write_data((DISPLAY_HEIGHT - x0 - 1)
        & 0xFF); // Page 58 of SSD2119 datasheet //ram start vertical position

    write_command(0x0044); // Vertical RAM address window
    write_data((((DISPLAY_HEIGHT - x0 - 1) & 0xFF) << 8)
        | ((DISPLAY_HEIGHT - x0 - width) & 0xFF)); // Page 57 of SSD2119 datasheet

    write_command(0x0045); // Horizontal RAM address window
    write_data(y0 & 0x1FF); // Page 57 of SSD2119 datasheet
    write_command(0x0046); // Horizontal RAM address position
    write_data((y0 + height - 1) & 0x1FF); // Page 57 of SSD2119 datasheet

    write_command(0x0022); // RAM data write/read
}

static void writeSubBitmap(
    int x0, int y0, int img_w, int img_h, const unsigned char* img_data, int sub_x, int sub_w)
{
    __disable_irq();
    int y, x, i;
    int img_w_rounded = ((8 * img_w + 31) / 32) * 4;

    if ((x0 + sub_w) > DISPLAY_WIDTH) {
        sub_w = DISPLAY_WIDTH - x0;
    }

    if ((y0 + img_h) > DISPLAY_HEIGHT) {
        img_h = DISPLAY_HEIGHT - y0;
    }

    displaySub(x0, y0, sub_w, img_h);

    for (y = img_h - 1; y >= 0; y--) {
        for (x = 0; x < (sub_w >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = *(g_palette_ram + img_data[y * img_w_rounded + sub_x + (x << 2) + i]);
            }

            spi_transmit((unsigned short*)g_fifo, 8);
        }

        x <<= 2;

        for (; x < sub_w; x++) {
            write_color(*(g_palette_ram + img_data[y * img_w_rounded + sub_x + x]));
        }
    }

    __enable_irq();
}
#if 0
static void writeSubBitmap_Rotated(int x0, int y0, int img_w, int img_h, const unsigned char* img_data, int sub_x, int sub_w)
{
    __disable_irq();
    int y, x, i;
    int img_w_rounded = ((8 * img_w + 31) / 32) * 4;

    if ((x0 + sub_w) > DISPLAY_HEIGHT) {
    	sub_w = DISPLAY_HEIGHT  - x0;
    }

    if ((y0 + img_h) > DISPLAY_WIDTH) {
    	img_h = DISPLAY_WIDTH - y0;
    }

    displaySub_Rotated(x0, y0, sub_w, img_h);

    for (y = img_h - 1; y >= 0; y--) {
        for (x = 0; x < (sub_w >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = * (g_palette_ram + img_data[y * img_w_rounded + sub_x + (x << 2) + i]);
            }

            spi_transmit((unsigned short*) g_fifo, 8);
        }

        x <<= 2;

        for (; x < sub_w; x++) {
            write_color(* (g_palette_ram + img_data[y * img_w_rounded + sub_x + x]));
        }
    }

    __enable_irq();
}
#endif

static void printfCheckBounds(int next_width, int line_height)
{
    area_t clear;

    if ((cursor_x + next_width) > (pf_area.w + pf_area.x)) {
        cursor_x = pf_area.x;
        cursor_y += line_height;

        if (cursor_y > (pf_area.h + pf_area.y)) {
            cursor_y = pf_area.y;
        }

        clear.x = cursor_x;
        clear.y = cursor_y;
        clear.w = pf_area.w;
        clear.h = line_height;

        MXC_TFT_FillRect(&clear, g_background_color);
    }
}

static void printCursor(char* str)
{
    int i;
    bitmap_info_t bitmap_info;
    font_info_t font_info;
    font_char_t* chr_pos;
    unsigned char chId;
    unsigned char* pixel;
    int len;

    if ((unsigned int)g_font_id >= images_header.nb_font) {
        return;
    }

    get_font_info(g_font_id, &font_info, &chr_pos);
    get_bitmap_info(font_info.bitmap_id, &bitmap_info, &pixel);
    setPalette(bitmap_info.id_palette);

    len = strlen(str);

    for (i = 0; i < len; i++) {
        if (str[i] == '\n') {
            printfCheckBounds(DISPLAY_WIDTH,
                bitmap_info.h); // using display size will force cursor to next line
        } else if (str[i] == ' ') {
            printfCheckBounds(8, bitmap_info.h); // Check if space will need to wrap
            cursor_x += 8;
        } else {
            // find char
            for (chId = 0; chId < font_info.nb_char; chId++) {
                if (str[i] == chr_pos[chId].hex_code) {
                    printfCheckBounds(chr_pos[chId].w + 1, bitmap_info.h);
                    writeSubBitmap(cursor_x, cursor_y, bitmap_info.w, bitmap_info.h, pixel,
                        chr_pos[chId].x, chr_pos[chId].w);
                    cursor_x += chr_pos[chId].w + 1; // font.intr_chr;
                    break;
                }
            }
        }
    }
}

/******************************** Public Functions ***************************/
int MXC_TFT_Init(
    mxc_spi_regs_t* tft_spi, int ss_idx, mxc_gpio_cfg_t* reset_ctrl, mxc_gpio_cfg_t* bl_ctrl)
{
    int result = E_NO_ERROR;

    spi = tft_spi;
    ssel = ss_idx;
    reset_pin = reset_ctrl;
    blen_pin = bl_ctrl;

    // set images start addr
    if (images_start_addr == NULL) {
        images_start_addr = (unsigned char*)&_bin_start_;
    }

    // set header
    memset(&images_header, 0, sizeof(Header_images_t));
    memcpy(&images_header, images_start_addr, sizeof(Header_images_t));

    /*
     *      Configure GPIO Pins
     */
    if (reset_pin) {
        MXC_GPIO_Config(reset_pin);
    }

    if (blen_pin) {
        MXC_GPIO_Config(blen_pin);
    }

    // Configure SPI Pins
    tft_spi_init();

    // Turn off backlight
    MXC_TFT_Backlight(0);

    // Send commands to configure display
    displayInit();

    // Set default palette
    setPalette(0);

    // Set background color
    MXC_TFT_SetBackGroundColor(0);

    // Turn on backlight
    MXC_TFT_Backlight(1);

    return result;
}

void MXC_TFT_SetRotation(tft_rotation_t rotation)
{
    tft_rotation = rotation;
    if (tft_rotation == SCREEN_FLIP)
        MXC_TFT_WriteReg(0x0001, 0x30EF); // flip screen horizontally
}

void MXC_TFT_SetBackGroundColor(unsigned int color)
{
    __disable_irq();
    unsigned int x, y, i;

    displayAll();

    for (y = 0; y < DISPLAY_HEIGHT; y++) {
        for (x = 0; x < (unsigned int)(DISPLAY_WIDTH >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = g_palette_ram[color];
            }

            spi_transmit((unsigned short*)g_fifo, 8);
        }
    }

    __enable_irq();
    // keep color
    g_background_color = color;

    return;
}

int MXC_TFT_SetPalette(int img_id)
{
    bitmap_info_t bitmap_info;
    unsigned char* pixel;

    if ((unsigned int)img_id >= images_header.nb_bitmap) {
        return E_BAD_PARAM;
    }

    get_bitmap_info(img_id, &bitmap_info, &pixel);
    setPalette(bitmap_info.id_palette);

    return E_NO_ERROR;
}

void MXC_TFT_Backlight(int on)
{
    if (blen_pin) {
        if (on) {
            MXC_GPIO_OutSet(blen_pin->port, blen_pin->mask);
        } else {
            MXC_GPIO_OutClr(blen_pin->port, blen_pin->mask);
        }
    }
}

void MXC_TFT_ShowImage(int x0, int y0, int id)
{
    int y, width, height, img_w_rounded;
    bitmap_info_t bitmap_info;
    unsigned char* pixel;

    if ((unsigned int)id >= images_header.nb_bitmap) {
        return;
    }

    get_bitmap_info(id, &bitmap_info, &pixel);

    width = bitmap_info.w;
    height = bitmap_info.h;

    if (tft_rotation == SCREEN_NORMAL || tft_rotation == SCREEN_FLIP) {
        if ((x0 + width) > DISPLAY_WIDTH) {
            width = DISPLAY_WIDTH - x0;
        }

        if ((y0 + height) > DISPLAY_HEIGHT) {
            height = DISPLAY_HEIGHT - y0;
        }

        displaySub(x0, y0, width, height);

    } else if (tft_rotation == SCREEN_ROTATE) {
        if ((x0 + width) > DISPLAY_HEIGHT) {
            width = DISPLAY_HEIGHT - x0;
        }

        if ((y0 + height) > DISPLAY_WIDTH) {
            height = DISPLAY_WIDTH - y0;
        }

        displaySub_Rotated(x0, y0, width, height);
    }

    setPalette(bitmap_info.id_palette);

    if (bitmap_info.rle == 1) {
        RLE_decode(pixel, bitmap_info.data_size, height, width);
    } else {
        img_w_rounded = ((8 * bitmap_info.w + 31) / 32) * 4;

        for (y = height - 1; y >= 0; y--) {
            print_line(&pixel[y * img_w_rounded], width);
        }
    }
}

void MXC_TFT_ShowImageCameraRGB565(int x0, int y0, uint8_t* image, int width, int height)
{
    int x, y;

    if (tft_rotation == SCREEN_NORMAL || tft_rotation == SCREEN_FLIP) {
        if ((x0 + width) > DISPLAY_WIDTH) {
            width = DISPLAY_WIDTH - x0;
        }

        if ((y0 + height) > DISPLAY_HEIGHT) {
            height = DISPLAY_HEIGHT - y0;
        }

        displaySub(x0, y0, width, height);

        for (y = 0; y < height; y++) {
            print_line_rgb565(&image[y * width * 2], width);
        }
    } else if (tft_rotation == SCREEN_ROTATE) {
        if ((x0 + width) > DISPLAY_HEIGHT) {
            width = DISPLAY_HEIGHT - x0;
        }

        if ((y0 + height) > DISPLAY_WIDTH) {
            height = DISPLAY_WIDTH - y0;
        }

        write_command(0x0011); // Entry Mode
        write_data(0x6840);

        displaySub_Special(x0, y0, width, height);

        for (x = 0; x < width; x++) {
            print_line_rgb565(&image[x * height * 2], height);
        }

        write_command(0x0011); // Entry Mode
        write_data(0x6858);
    }
}

void MXC_TFT_ClearScreen(void)
{
    MXC_TFT_SetBackGroundColor(g_background_color);
}

void MXC_TFT_FillRect(area_t* area, int color)
{
    __disable_irq();
    int y, x, i, h, w;

    w = area->w;
    h = area->h;

    if ((area->x + w) > DISPLAY_WIDTH) {
        w = DISPLAY_WIDTH - area->x;
    }

    if ((area->y + h) > DISPLAY_HEIGHT) {
        h = DISPLAY_HEIGHT - area->y;
    }

    displaySub(area->x, area->y, w, h);

    for (y = 0; y < h; y++) {
        for (x = 0; x < (w >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = g_palette_ram[color];
            }

            spi_transmit((unsigned short*)g_fifo, 8);
        }

        x <<= 2;

        for (; x < w; x++) {
            write_color(g_palette_ram[color]);
        }
    }

    __enable_irq();
}

void MXC_TFT_WritePixel(int pixelX, int pixelY, int width, int height, uint32_t color)
{
    area_t _area;
    area_t* area;

    area = &_area;
    area->x = pixelX;
    area->y = pixelY;
    area->w = width;
    area->h = height;

    __disable_irq();
    int y, x, i, h, w;

    w = area->w;
    h = area->h;

    if ((area->x + w) > DISPLAY_WIDTH) {
        w = DISPLAY_WIDTH - area->x;
    }

    if ((area->y + h) > DISPLAY_HEIGHT) {
        h = DISPLAY_HEIGHT - area->y;
    }

    displaySub(area->x, area->y, w, h);

    for (y = 0; y < h; y++) {
        for (x = 0; x < (w >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = color;
            }

            spi_transmit((unsigned short*)g_fifo, 8);
        }

        x <<= 2;

        for (; x < w; x++) {
            write_color(color);
        }
    }

    __enable_irq();
}

void MXC_TFT_PrintPalette(void)
{
    int i;
    area_t area = { 10, 10, 2, 25 };

    for (i = 0; i < 256; i++) {
        area.x += 4;
        MXC_TFT_FillRect(&area, i);

        if ((i & 63) == 63) {
            area.y += 25;
            area.x = 10;
        }
    }
}

/***************************************************************
 *          Printf Functions
 ***************************************************************/
void MXC_TFT_ResetCursor(void)
{
    cursor_x = pf_area.x;
    cursor_y = pf_area.y;
}

void MXC_TFT_SetFont(int font_id)
{
    g_font_id = font_id;
}

void MXC_TFT_Printf(const char* format, ...)
{
    char str[100];

    sprintf(str, format, *((&format) + 1), *((&format) + 2), *((&format) + 3));

    printCursor(str); // printf_message
}

void MXC_TFT_ConfigPrintf(area_t* area)
{
    pf_area.x = area->x;
    pf_area.y = area->y;
    pf_area.w = area->w;
    pf_area.h = area->h;

    MXC_TFT_ResetCursor();
}

void MXC_TFT_PrintFont(int x0, int y0, int id, text_t* str, area_t* area)
{
    int i;
    int x;
    bitmap_info_t bitmap_info;
    font_info_t font_info;
    font_char_t* chr_pos;
    unsigned char chId;
    unsigned char* pixel;

    if ((unsigned int)id >= images_header.nb_font) {
        return;
    }

    get_font_info(id, &font_info, &chr_pos);
    get_bitmap_info(font_info.bitmap_id, &bitmap_info, &pixel);
    setPalette(bitmap_info.id_palette);

    x = x0;

    for (i = 0; i < str->len; i++) {
        if (str->data[i] == ' ') {
            x += 8; // font.space; // TODO add space in font bitmap file
        } else {
            // find char
            for (chId = 0; chId < font_info.nb_char; chId++) {
                if (str->data[i] == chr_pos[chId].hex_code) {
                    writeSubBitmap(x, y0, bitmap_info.w, bitmap_info.h, pixel, chr_pos[chId].x,
                        chr_pos[chId].w);
                    x += chr_pos[chId].w + 1; // font.intr_chr;
                    break;
                }
            }
        }
    }

    if (area) {
        area->x = x0;
        area->y = y0;
        area->h = bitmap_info.h;
        area->w = x - x0;
    }
}

void MXC_TFT_Print(int x0, int y0, text_t* str, area_t* area)
{
    MXC_TFT_PrintFont(x0, y0, g_font_id, str, area);
}

void MXC_TFT_ClearArea(area_t* area, int color)
{
    int y, x, i, h, w;

    w = area->w;
    h = area->h;

    if (tft_rotation == SCREEN_NORMAL || tft_rotation == SCREEN_FLIP) {
        if ((area->x + w) > DISPLAY_WIDTH) {
            w = DISPLAY_WIDTH - area->x;
        }

        if ((area->y + h) > DISPLAY_HEIGHT) {
            h = DISPLAY_HEIGHT - area->y;
        }

        displaySub(area->x, area->y, w, h);

    } else if (tft_rotation == SCREEN_ROTATE) {
        if ((area->x + w) > DISPLAY_HEIGHT) {
            w = DISPLAY_HEIGHT - area->x;
        }

        if ((area->y + h) > DISPLAY_WIDTH) {
            h = DISPLAY_WIDTH - area->y;
        }

        displaySub_Rotated(area->x, area->y, w, h);
    }

    for (y = 0; y < h; y++) {
        for (x = 0; x < (w >> 2); x++) {
            for (i = 0; i < 4; i++) {
                g_fifo[i] = *(g_palette_ram + color);
            }

            spi_transmit((unsigned short*)g_fifo, 8);
        }

        x <<= 2;

        for (; x < w; x++) {
            write_color(*(g_palette_ram + color));
        }
    }
}

void MXC_TFT_WriteReg(unsigned short command, unsigned short data)
{
    write_command(command);
    write_data(data);
}
