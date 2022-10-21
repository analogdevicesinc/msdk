/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "bbfc_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "dma.h"
#include "led.h"
#include "tmr.h"
#include "pb.h"
#include "cnn.h"
#include "weights.h"
#include "sampledata.h"
#include "mxc_delay.h"
#include "camera.h"
#include "spi.h"

#ifdef BOARD_EVKIT_V1
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

// SELECT THE FOLLOWING BUILD OPTIONS
// To use camera, comment out both
//#define USE_SAMPLEDATA        // shows the sample data
//#define USE_SPIDATA			// shows images received from serial SPI - ONLY EVKIT

#define IMAGE_SIZE_X (80)
#define IMAGE_SIZE_Y (80)
#define CAMERA_FREQ (10 * 1000 * 1000)

#define MIRROR 0
#define TFT_BUFF_SIZE 50 // TFT buffer size

// Select overlay method
//#define USE_ALPHA  // enable to use alpha, disable to use op

#define ALPHA 0.35 // alpha value for overlaid mask
#define OP 0.95 // opacity for overlaid mask (if alpha method is not used)

#ifdef BOARD_EVKIT_V1
int image_bitmap_1 = ADI_256_bmp;
int image_bitmap_2 = logo_white_bg_darkgrey_bmp;
int font_1 = urw_gothic_12_white_bg_grey;
int font_2 = urw_gothic_13_white_bg_grey;
#endif
#ifdef BOARD_FTHR_REVA
#undef USE_SPIDATA // only supported in evkit
int image_bitmap_1 = (int)&img_1_rgb565[0];
int image_bitmap_2 = (int)&logo_rgb565[0];
int font_1 = (int)&SansSerif16x16[0];
int font_2 = (int)&SansSerif16x16[0];
#endif

#define FT4222_CLK 60000000
#define DATA_LEN 4 * IMAGE_SIZE_X *IMAGE_SIZE_Y
#define SPI_REGS MXC_SPI1
#define SPI_IRQ SPI1_IRQn
#define SPI_SPEED FT4222_CLK / 8 // Must match the SPI clock in python script

static int32_t ml_data32[(CNN_NUM_OUTPUTS + 3) / 4];
volatile uint32_t cnn_time; // Stopwatch

// 3-channel 80x80 data input (19200 bytes total / 6400 bytes per channel):
// HWC 80x80, channels 0 to 2
#ifdef USE_SAMPLEDATA //Sample DATA
static const uint32_t input_buffer[] = SAMPLE_INPUT_0;
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
uint32_t *input = input_buffer;

#elif defined(USE_SPIDATA) // SPI data
static uint32_t input_buffer[IMAGE_SIZE_X * IMAGE_SIZE_Y];
uint32_t *input = input_buffer;
uint8_t *rx_data = (uint8_t *)input_buffer; //[IMAGE_SIZE_X*IMAGE_SIZE_Y*3];
uint8_t tx_data[1] = { 'X' };
volatile bool SPI_FLAG = false;

static void spi_enable_interrupts(uint32_t mask)
{
    MXC_SPI_EnableInt(SPI_REGS, mask);
    NVIC_EnableIRQ(SPI_IRQ);
}

static void spi_clear_interrupts(uint32_t mask)
{
    SPI_REGS->intfl |= mask;
}

static void spi_init(void)
{
    int retVal;
    mxc_spi_pins_t spi_pins;

    spi_pins.clock = TRUE;
    spi_pins.miso = TRUE;
    spi_pins.mosi = TRUE;
    spi_pins.ss0 = TRUE;
    spi_pins.ss1 = FALSE;
    spi_pins.ss2 = FALSE;
    spi_pins.sdio2 = FALSE;
    spi_pins.sdio3 = FALSE;

    retVal = MXC_SPI_Init(SPI_REGS, 0, 0, 0, 0, SPI_SPEED, spi_pins);
    if (retVal != E_NO_ERROR) {
        printf("SPI Slave Initialization Error\n");
        while (1) {}
    }
    retVal = MXC_SPI_SetDataSize(SPI_REGS, 8); // 8 bits per character

    if (retVal != E_NO_ERROR) {
        printf("SPI SET DATASIZE ERROR: %d\n", retVal);
        while (1) {}
    }

    retVal = MXC_SPI_SetWidth(SPI_REGS, SPI_WIDTH_STANDARD);

    if (retVal != E_NO_ERROR) {
        printf("SPI SET WIDTH ERROR: %d\n", retVal);
        while (1) {}
    }

    spi_enable_interrupts(MXC_F_SPI_INTEN_SSA);
}

void SPI1_IRQHandler(void)
{
    if (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_20) == 1) {
        return;
    }

    if (MXC_SPI_GetFlags(SPI_REGS) & MXC_F_SPI_INTFL_SSA) {
        spi_clear_interrupts(MXC_F_SPI_INTFL_SSA);
        SPI_FLAG = true;
    }
}

#else // Camera
static uint32_t input_buffer[IMAGE_SIZE_X * IMAGE_SIZE_Y];
uint32_t *input = input_buffer;
uint8_t *rx_data = (uint8_t *)input_buffer; //[IMAGE_SIZE_X*IMAGE_SIZE_Y*3];
#endif

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1) {}
}

int8_t unsigned_to_signed(uint8_t val)
{
    return val - 128;
}

void load_input(void)
{
#ifdef USE_SAMPLEDATA
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t *)0x50408000, input, 6400);

#elif defined(USE_SPIDATA)
    // This function loads the input data from SPI
    int retVal;
    int i;
    mxc_spi_req_t req;

    // SPI transaction request
    req.spi = SPI_REGS;
    req.txData = (uint8_t *)tx_data;
    req.rxData = (uint8_t *)rx_data;
    req.txLen = 1;
    req.rxLen = DATA_LEN;
    req.ssIdx = 0;

    // Blocking SPI transaction
    retVal = MXC_SPI_SlaveTransaction(&req);

    if (retVal != E_NO_ERROR) {
        printf("SPI Transaction failed!\n");
    } else {
        for (i = 0; i < DATA_LEN; i++) {
            rx_data[i] = unsigned_to_signed(rx_data[i]);
        }

        memcpy32((uint32_t *)0x50408000, input, 6400);
    }
#else // Camera

    uint8_t *frame_buffer;
    uint8_t *buffer;
    uint32_t imgLen;
    uint32_t w, h, x, y;
    uint8_t r, g, b;
    int i = 0;

    camera_start_capture_image();
    while (!camera_is_image_rcv()) {}

    camera_get_image(&frame_buffer, &imgLen, &w, &h);
    buffer = frame_buffer;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            r = unsigned_to_signed(*buffer);
            buffer++;
            g = unsigned_to_signed(*buffer);
            buffer++;
            b = unsigned_to_signed(*buffer);
            buffer++;
            buffer++; // skip msb=0x00

            input[i] = r | (g << 8) | (b << 16);

            i++;
        }
    }

    // printf("r:%d g:%d b:%d  input:%x\r\n",r,g,b, input[i-1]);

    memcpy32((uint32_t *)0x50408000, input, 6400);

#endif
}

/* **************************************************************************** */
void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;

    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

/* **************************************************************************** */
void cnn_wait(void)
{
    while ((*((volatile uint32_t *)0x50100000) & (1 << 12)) != 1 << 12) {}
    CNN_COMPLETE; // Signal that processing is complete
    cnn_time = MXC_TMR_SW_Stop(MXC_TMR0);
}

/* **************************************************************************** */

static uint8_t signed_to_unsigned(int8_t val)
{
    uint8_t value;
    if (val < 0) {
        value = ~val + 1;
        return (128 - value);
    }
    return val + 128;
}

void show_image(uint32_t *image, int32_t *ml, uint32_t xcord, uint32_t ycord, uint32_t scale,
                uint32_t mask, uint32_t mirror)
{
    uint32_t i;

    int r;
    int g;
    int b;

    int8_t rr, gg, bb, uu;
    uint32_t x, y;
    uint32_t color;
    int32_t max = 0;
    uint32_t max_index = 0;
    int8_t *ml_data8 = (int8_t *)ml;

    x = xcord;
    y = ycord;

    if (mirror)
        x = xcord + IMAGE_SIZE_X;

    for (i = 0; i < IMAGE_SIZE_X * IMAGE_SIZE_Y - 1; i++) {
        max = -256; // smaller than -128
        max_index = 0;

        b = signed_to_unsigned(((image[i] >> 16) & 0xFF));
        g = signed_to_unsigned(((image[i] >> 8) & 0xFF));
        r = signed_to_unsigned(((image[i] >> 0) & 0xFF));

        // if ml mask display, or mask overlay
        if (ml != NULL) {
            rr = ml_data8[i];
            bb = ml_data8[i + CNN_NUM_OUTPUTS / 4]; //swapped b&g
            gg = ml_data8[i + 2 * CNN_NUM_OUTPUTS / 4];
            uu = ml_data8[i + 3 * CNN_NUM_OUTPUTS / 4];

            //printf("[%d] %d  %d  %d  %d \n",i,rr, bb, gg, uu);

            if (rr > max) {
                max = rr;
                max_index = 0;
            }
            if (gg > max) {
                max = gg;
                max_index = 1;
            }
            if (bb > max) {
                max = bb;
                max_index = 2;
            }
            if (uu > max) {
                max = uu;
                max_index = 3;
            }

#define BLACK_COLOR 5
            // display only mask if image is null, otherwise overlay
            switch (max_index) {
#ifdef USE_ALPHA // overlay using alpha value
            case 0:
                r = mask + ALPHA * (r - mask) * (image != NULL);
                g = (ALPHA * g) * (image != NULL);
                b = (ALPHA * b) * (image != NULL);
                break;
            case 1:
                r = (ALPHA * r) * (image != NULL);
                g = mask + ALPHA * (g - mask) * (image != NULL);
                b = (ALPHA * b) * (image != NULL);
                break;
            case 2:
                r = (ALPHA * r) * (image != NULL);
                g = (ALPHA * g) * (image != NULL);
                b = mask + ALPHA * (b - mask) * (image != NULL);
                break;
            case 3:
                r = (ALPHA * r) * (image != NULL);
                g = (ALPHA * g) * (image != NULL);
                b = (ALPHA * b) * (image != NULL);
                break;

#else // overlay using OP
            case 0:
                r = mask;
                g = (g * OP) * (image != NULL);
                b = (b * OP) * (image != NULL);
                break;
            case 1:
                r = (r * OP) * (image != NULL);
                {
                }
                g = mask;
                b = (b * OP) * (image != NULL);
                {
                }
                break;
            case 2:
                r = (r * OP) * (image != NULL);
                {
                }
                g = (g * OP) * (image != NULL);
                {
                }
                b = mask;
                break;
            case 3:
                r = BLACK_COLOR;
                g = BLACK_COLOR;
                b = BLACK_COLOR;
                break;
#endif

            default:
                printf("ERROR");
                while (1) {}
            }
        }
#ifdef BOARD_EVKIT_V1
        color =
            (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
        color = RGB(r, g, b); // convert to RGB565
#endif
        MXC_TFT_WritePixel(x * scale, y * scale, scale, scale, color);

        if (mirror) {
            x -= 1;
            if (x <= (xcord)) {
                x = xcord + IMAGE_SIZE_X;
                y += 1;
            }
        } else {
            x += 1;
            if (x >= (IMAGE_SIZE_X + xcord)) {
                x = xcord;
                y += 1;
            }
        }
    }
    // printf("[%d] %d  %d  %d  %d \n",i,rr, bb, gg, uu);
}

/* **************************************************************************** */

int main(void)
{
    char buff[TFT_BUFF_SIZE];

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    /* Enable camera power */
    Camera_Power(POWER_ON);
    //MXC_Delay(300000);
    printf("\n\nUNet Feather Demo\n");
#else
    printf("\n\nUNet Evkit Demo\n");
#endif

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("Waiting...\n");

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Configure P2.5, turn on the CNN Boost
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = MXC_GPIO2;
    gpio_out.mask = MXC_GPIO_PIN_5;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);
    MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

    // Initialize TFT display.
    printf("Init LCD.\n");
#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
    MXC_TFT_ClearScreen();
    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
#endif
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_270);
    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif

    MXC_Delay(1000000);

#if !defined(USE_SAMPLEDATA) && !defined(USE_SPIDATA)
    int dma_channel;
    // Initialize camera.
    printf("Init Camera.\n");
    camera_init(CAMERA_FREQ);

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA,
                 dma_channel);
#endif

    MXC_TFT_SetPalette(image_bitmap_2);
    MXC_TFT_SetBackGroundColor(4);
    //MXC_TFT_ShowImage(1, 1, image_bitmap_2);
    memset(buff, 32, TFT_BUFF_SIZE);

    TFT_Print(buff, 55, 30, font_2, sprintf(buff, "ANALOG DEVICES             "));

    TFT_Print(buff, 15, 50, font_2, sprintf(buff, "U-Net Segmentation Demo      "));

    TFT_Print(buff, 120, 90, font_1, sprintf(buff, "Ver. 1.1.0                   "));

    TFT_Print(buff, 55, 130, font_1, sprintf(buff, "Building(red), Sky(blue)          "));

    TFT_Print(buff, 5, 170, font_1, sprintf(buff, "Foliage(green), Unknown(black)  "));

    TFT_Print(buff, 30, 210, font_2, sprintf(buff, "PRESS PB1(SW1) TO START    "));
    while (!PB_Get(0)) {}

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: 50 MHz div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels

#if defined(USE_SPIDATA)
    spi_init();

    MXC_TFT_ClearScreen();

    TFT_Print(buff, 55, 40, font_1, sprintf(buff, "Waiting for SPI data ..."));
#endif
    while (1) {
#if defined(USE_SPIDATA)
        // wait for SPI
        if (!SPI_FLAG)
            continue;
#endif

        MXC_TFT_ClearScreen();

        //TFT_Print(buff, 5, 45, font_1, sprintf(buff, " Image        Mask      Overlay"));

        // Reload bias after wakeup
        cnn_init(); // Bring state machine into consistent state
        cnn_load_bias();
        cnn_configure(); // Configure state machine

        load_input(); // Load data input

        // show original image
        show_image(input, NULL, 0, 80, 1, 0, MIRROR);

        LED_On(LED1);
        cnn_start(); // Start CNN processing
        while (cnn_time == 0) __WFI(); // Wait for CNN
        LED_Off(LED1);
        cnn_unload((uint32_t *)ml_data32);

        // show mask
        show_image(NULL, ml_data32, 120, 80, 1, 0xf0, MIRROR);

        // show overlay
        show_image(input, ml_data32, 240, 80, 1, 0xa0, MIRROR);

        printf("Time for CNN: %d us\n\n", cnn_time);

        TFT_Print(buff, 40, 180, font_1,
                  sprintf(buff, "Inference Time: %.3f ms     ", (float)cnn_time / 1000));

#if !defined(USE_SPIDATA)

        TFT_Print(buff, 20, 212, font_1, sprintf(buff, "PRESS PB1(SW1) TO CONTINUE "));
        while (!PB_Get(0)) {}
#else
        SPI_FLAG = false;
        spi_clear_interrupts(MXC_F_SPI_INTFL_SSA);
        spi_enable_interrupts(MXC_F_SPI_INTEN_SSA);

#endif
        if (PB_Get(1)) {
            LED_On(LED1);
            MXC_Delay(cnn_time);
            LED_Off(LED1);
        }
    }

    return 0;
}
