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

/**
 * @file    main.c
 * @brief   Main for Snake game
 * @details
 *
 *
 */

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2s_regs.h"
#include "board.h"
#include "mxc.h"
#include "i2s.h"
#include "tmr.h"
#include "dma.h"
#include "led.h"
#include "pb.h"
#include "cnn.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "bitmap.h"
#endif

/* **** Definitions **** */
#define COUNT_20ms 250000
#define COUNT_30ms 375000
#define COUNT_200us 2500
#define COUNT_100us 1250

/* Enable/Disable Features */
//#define ENABLE_CLASSIFICATION_DISPLAY  // enables printing classification result
#define ENABLE_SILENCE_DETECTION // Starts collecting only after avg > THRESHOLD_HIGH, otherwise starts from first sample
#undef EIGHT_BIT_SAMPLES // samples from Mic or Test vectors are eight bit, otherwise 16-bit

/*-----------------------------*/
/* keep following unchanged */
#define SAMPLE_SIZE 16384 // size of input vector for CNN, keep it multiple of 128
#define CHUNK \
    128 // number of data points to read at a time and average for threshold, keep multiple of 128
#define TRANSPOSE_WIDTH 128 // width of 2d data model to be used for transpose
#define NUM_OUTPUTS 21 // number of classes
#define I2S_RX_BUFFER_SIZE 64 // I2S buffer size
#define TFT_BUFF_SIZE 50 // TFT buffer size
/*-----------------------------*/

/* Adjustables */
#define SAMPLE_SCALE_FACTOR \
    4 // multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH 350 // voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW 100 // voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD \
    20 // [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE 30 * CHUNK // how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD 49 // min probability (0-100) to accept an inference

/* Snake global variables */
#define scale 5
#define snakeInitialLength 12
#define snakeMaxLength 300 // max length of the snake
#define screenWidth 320
#define screenHeight 240
#define false 0
#define true 1

/* **** Globals **** */
volatile uint32_t cnn_time; // Stopwatch

static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];
uint8_t pAI85Buffer[SAMPLE_SIZE];
uint8_t pPreambleCircBuffer[PREAMBLE_SIZE];
int16_t Max, Min;
uint16_t thresholdHigh = THRESHOLD_HIGH;
uint16_t thresholdLow = THRESHOLD_LOW;

volatile uint8_t i2s_flag = 0, tmr_flag = 0, tmr2_flag = 0;
int32_t i2s_rx_buffer[I2S_RX_BUFFER_SIZE];
int xScreen = (screenWidth / scale);
int yScreen = (screenHeight / scale);
int snakeLength = snakeInitialLength; // length of the snake
int positions[2][snakeMaxLength]; // positions of the snake pieces
int fruit[2]; // poition of the fruit
unsigned long score = 0; // game score
unsigned long lastScore = 0; // score at the previous time
char buff[TFT_BUFF_SIZE];
int gameOver = 0, gameStart = 0, chooseLevel = 0, tmr1_cmp = 0;

/* **** Constants **** */
typedef enum _mic_processing_state {
    STOP = 0, /* No processing  */
    SILENCE = 1, /* Threshold not detected yet  */
    KEYWORD = 2 /* Threshold has been detected, gathering keyword samples */
} mic_processing_state;

/* Set of detected words */
typedef enum keyword_sample {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    STOPGAME,
    GO,
    ONE,
    TWO,
    THREE,
    FOUR,
    UNKNOWN
} key_detected;

int prevSnakeDirection = UNKNOWN;
key_detected snakeDirection = UNKNOWN; // Direction where the snake will move

typedef enum game_level {
    ZERO,
    LEVEL1, // Slowest speed
    LEVEL2,
    LEVEL3,
    LEVEL4 // Fastest speed
} speed_level;
speed_level gameSpeed = ZERO;

typedef int boolean;

/* Set of detected words */
const char keywords[NUM_OUTPUTS][10] = { "UP",    "DOWN", "LEFT",   "RIGHT", "STOP",  "GO",
                                         "YES",   "NO",   "ON",     "OFF",   "ONE",   "TWO",
                                         "THREE", "FOUR", "FIVE",   "SIX",   "SEVEN", "EIGHT",
                                         "NINE",  "ZERO", "Unknown" };

#ifdef BOARD_EVKIT_V1
int image_bitmap_1 = ADI_256_bmp;
int image_bitmap_2 = logo_white_bg_darkgrey_bmp;
int font_1 = urw_gothic_12_white_bg_grey;
int font_2 = urw_gothic_13_white_bg_grey;
#endif
#ifdef BOARD_FTHR_REVA
int image_bitmap_1 = (int)&img_1_rgb565[0];
int image_bitmap_2 = (int)&logo_rgb565[0];
int font_1 = (int)&SansSerif16x16[0];
int font_2 = (int)&SansSerif16x16[0];
#endif

/* **** Functions Prototypes **** */
void fail(void);
uint8_t cnn_load_data(uint8_t *pIn);
uint8_t MicReadChunk(uint8_t *pBuff, uint16_t *avg);
uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize, uint16_t outSize,
                     uint16_t width);
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data, int16_t *out_class, double *out_prob);
void I2SInit();
void HPF_init(void);
int16_t HPF(int16_t input);

void snakeIntro();
void setup();
void placeFruit();
boolean fruitIsEaten();
void drawSnakeHead();
void moveTheSnake();
void setSnakeDirection(int top_index);
void checkSnakeCollision();

// ******************************************************************************
static uint32_t setColor(int r, int g, int b)
{
    uint32_t color;

#ifdef BOARD_EVKIT_V1
    color = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
#endif
#ifdef BOARD_FTHR_REVA
    color = RGB(r, g, b); // convert to RGB565
#endif

    return color;
}

// ******************************************************************************
static int genRandomNum(int lower, int upper)
{
    return ((rand() % (upper - lower + 1)) + lower);
}

// ******************************************************************************
static void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

// ******************************************************************************
void i2s_isr(void)
{
    i2s_flag = 1;
    // Clear I2S interrupt flag
    MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_THD_CH0);
}

// ******************************************************************************
void tmr_isr(void)
{
    tmr_flag = 1;
    // Clear TMR1 interrupt flag
    MXC_TMR_ClearFlags(MXC_TMR1);
}

// ******************************************************************************
void tmr2_isr(void)
{
    uint32_t color;

    if (tmr2_flag == 1) {
        color = setColor(0, 255, 0);
        tmr2_flag = 0;
    } else {
        color = setColor(0, 0, 0);
        tmr2_flag = 1;
    }

    MXC_TFT_WritePixel(scale * fruit[0], scale * fruit[1], scale, scale, color);

    // Clear TMR2 interrupt flag
    MXC_TMR_ClearFlags(MXC_TMR2);
}

// *****************************************************************************
int main(void)
{
    uint32_t sampleCounter = 0;
    mxc_tmr_unit_t units;

    uint8_t pChunkBuff[CHUNK];

    uint16_t avg = 0;
    uint16_t preambleCounter = 0;
    uint16_t ai85Counter = 0;
    uint16_t wordCounter = 0;

    uint16_t avgSilenceCounter = 0;

    mic_processing_state procState = STOP;

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    /* Enable microphone power on Feather board */
    Microphone_Power(POWER_ON);
#endif

    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Switch to 100 MHz clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
    /* CNN clock: 50 MHz div 1 */
    cnn_enable((0 << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS), MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    /* Configure P2.5, turn on the CNN Boost */
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);

    printf("**************** Snake Game ****************\r\n");
    // Initialize TFT display.
    printf("\n*** Init LCD ***\n");

#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
    MXC_TFT_ClearScreen();
    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
#endif
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_90);
    MXC_TFT_ShowImage(0, 0, image_bitmap_1);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif

    MXC_Delay(1000000);

    // Select the color of the background
    MXC_TFT_SetPalette(image_bitmap_2);
    MXC_TFT_SetBackGroundColor(4);
    //MXC_TFT_ShowImage(0, 0, image_bitmap_2);
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 60, 40, font_2, sprintf(buff, "ANALOG DEVICES"));
    TFT_Print(buff, 90, 70, font_2, sprintf(buff, "SNAKE GAME"));
    TFT_Print(buff, 5, 110, font_1, sprintf(buff, "This game uses real-time"));
    TFT_Print(buff, 5, 135, font_1, sprintf(buff, "speech recognition to move"));
    TFT_Print(buff, 5, 160, font_1, sprintf(buff, "snake using certain commands"));
    TFT_Print(buff, 1, 210, font_2, sprintf(buff, "PRESS PB1(SW1) TO CONTINUE"));

    while (!PB_Get(0)) {}

    snakeIntro();

    /* Bring CNN state machine into consistent state */
    cnn_init();
    /* Load CNN kernels */
    cnn_load_weights();
    /* Configure CNN state machine */
    cnn_configure();

    /* switch to silence state*/
    procState = SILENCE;

    mxc_tmr_cfg_t tmr2 = { TMR_PRES_4,      TMR_MODE_CONTINUOUS,
                           TMR_BIT_MODE_32, MXC_TMR_APB_CLK,
                           1250000,         0 };
    MXC_TMR_Init(MXC_TMR2, &tmr2, false);
    MXC_NVIC_SetVector(TMR2_IRQn, tmr2_isr);
    NVIC_EnableIRQ(TMR2_IRQn);
    MXC_TMR_EnableInt(MXC_TMR2);

    /* initialize I2S interface to Mic */
    I2SInit();

    printf("\n*** READY ***\n");

    /* Read samples */
    while (1) {
        if (tmr_flag) {
            moveTheSnake();
            checkSnakeCollision();
            tmr_flag = 0;
        }

        if (gameOver) {
            MXC_TMR_Stop(MXC_TMR2);
            MXC_TMR_Stop(MXC_TMR1);
            gameOver = 0;
            MXC_Delay(100);
            MXC_TFT_ClearScreen();
            MXC_TFT_SetPalette(image_bitmap_2);
            MXC_TFT_SetBackGroundColor(4);
            //MXC_TFT_ShowImage(0, 0, image_bitmap_2);
            memset(buff, 0, TFT_BUFF_SIZE);
            TFT_Print(buff, 100, 80, font_2, sprintf(buff, "GAME OVER"));
            TFT_Print(buff, 95, 110, font_2, sprintf(buff, "Your Score: %d  ", score));
            memset(buff, 32, TFT_BUFF_SIZE);
            TFT_Print(buff, 10, 210, font_1, sprintf(buff, "SAY 'GO' TO RESTART GAME"));
            MXC_Delay(100);
            MXC_I2S_Shutdown();
            LED_Off(LED2);
            snakeDirection = UNKNOWN;
            prevSnakeDirection = UNKNOWN;
            snakeLength = snakeInitialLength;
            lastScore = score;
            score = 0;
            gameStart = 0;
            I2SInit();
        }

        /* Read from Mic driver to get CHUNK worth of samples, otherwise next sample*/
        if (MicReadChunk(pChunkBuff, &avg) == 0) {
            LED_Off(LED2);
            continue;
        }

        sampleCounter += CHUNK;

#ifdef ENABLE_SILENCE_DETECTION // disable to start collecting data immediately.

        /* copy the preamble data*/
        /* add the new chunk to the end of circular buffer*/
        memcpy(&pPreambleCircBuffer[preambleCounter], pChunkBuff, sizeof(uint8_t) * CHUNK);

        /* increment circular buffer pointer*/
        preambleCounter = (preambleCounter + CHUNK) % (PREAMBLE_SIZE);

        /* if we have not detected voice, check the average*/
        if (procState == SILENCE) {
            /* compute average, proceed if greater than threshold */
            if (avg >= thresholdHigh) {
                /* switch to keyword data collection*/
                procState = KEYWORD;
                printf("%.6d Word starts from index: %d, avg:%d > %d \n", sampleCounter,
                       sampleCounter - PREAMBLE_SIZE - CHUNK, avg, thresholdHigh);

                /* reorder circular buffer according to time at the beginning of pAI85Buffer */
                if (preambleCounter == 0) {
                    /* copy latest samples afterwards */
                    if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer, PREAMBLE_SIZE,
                                     SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }
                } else {
                    /* copy oldest samples to the beginning*/
                    if (AddTranspose(&pPreambleCircBuffer[preambleCounter], pAI85Buffer,
                                     PREAMBLE_SIZE - preambleCounter, SAMPLE_SIZE,
                                     TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }

                    /* copy latest samples afterwards */
                    if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer, preambleCounter,
                                     SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }
                }

                /* preamble is copied and state is changed, start adding keyword samples next run */
                ai85Counter += PREAMBLE_SIZE;
                continue;
            }
        }
        /* if it is in data collection, add samples to buffer*/
        else if (procState == KEYWORD)
#endif //#ifdef ENABLE_SILENCE_DETECTION

        {
            uint8_t ret = 0;

            /* add sample, rearrange buffer */
            ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE, TRANSPOSE_WIDTH);

            /* increment number of stored samples */
            ai85Counter += CHUNK;

            /* if there is silence after at least 1/3 of samples passed, increment number of times back to back silence to find end of keyword */
            if ((avg < thresholdLow) && (ai85Counter >= SAMPLE_SIZE / 3)) {
                avgSilenceCounter++;
            } else {
                avgSilenceCounter = 0;
            }

            /* if this is the last sample and there are not enough samples to
             * feed to CNN, or if it is long silence after keyword,  append with zero (for reading file)
             */
            if (avgSilenceCounter > SILENCE_COUNTER_THRESHOLD) {
                memset(pChunkBuff, 0, CHUNK);
                printf("%.6d: Word ends, Appends %d zeros \n", sampleCounter,
                       SAMPLE_SIZE - ai85Counter);
                ret = 0;

                while (!ret) {
                    ret =
                        AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE, TRANSPOSE_WIDTH);
                    ai85Counter += CHUNK;
                }
            }

            /* if enough samples are collected, start CNN */
            if (ai85Counter >= SAMPLE_SIZE) {
                int16_t out_class = -1;
                double probability = 0;

                /* reset counters */
                ai85Counter = 0;
                avgSilenceCounter = 0;

                /* new word */
                wordCounter++;

                /* change state to silence */
                procState = SILENCE;

                /* sanity check, last transpose should have returned 1, as enough samples should have already been added */
                if (ret != 1) {
                    printf("ERROR: Transpose incomplete!\n");
                    fail();
                }

                //----------------------------------  : invoke AI85 CNN
                printf("%.6d: Starts CNN: %d\n", sampleCounter, wordCounter);

                /* load to CNN */
                if (!cnn_load_data(pAI85Buffer)) {
                    printf("ERROR: Loading data to CNN! \n");
                    fail();
                }

                /* Start CNN */
                if (!cnn_start()) {
                    printf("ERROR: Starting CNN! \n");
                    fail();
                }

                /* Wait for CNN  to complete */
                while (cnn_time == 0) {
                    __WFI();
                }

                /* read data */
                cnn_unload((uint32_t *)ml_data);

                /* Get time */
                MXC_TMR_GetTime(MXC_TMR0, cnn_time, (void *)&cnn_time, &units);
                printf("%.6d: Completes CNN: %d\n", sampleCounter, wordCounter);

                switch (units) {
                case TMR_UNIT_NANOSEC:
                    cnn_time /= 1000;
                    break;

                case TMR_UNIT_MILLISEC:
                    cnn_time *= 1000;
                    break;

                case TMR_UNIT_SEC:
                    cnn_time *= 1000000;
                    break;

                default:
                    break;
                }

                printf("CNN Time: %d us\n", cnn_time);

                /* run softmax */
                softmax_q17p14_q15((const q31_t *)ml_data, NUM_OUTPUTS, ml_softmax);

#ifdef ENABLE_CLASSIFICATION_DISPLAY
                printf("\nClassification results:\n");

                for (int i = 0; i < NUM_OUTPUTS; i++) {
                    int digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
                    int tens = digs % 10;
                    digs = digs / 10;

                    printf("[%+.7d] -> Class %.2d %8s: %d.%d%%\n", ml_data[i], i, keywords[i], digs,
                           tens);
                }

#endif
                /* find detected class with max probability */
                ret = check_inference(ml_softmax, ml_data, &out_class, &probability);

                printf("----------------------------------------- \n");

                if (!ret) {
                    printf("LOW CONFIDENCE!: ");
                }

                printf("Detected word: %s (%0.1f%%)", keywords[out_class], probability);

                printf("\n----------------------------------------- \n");

                Max = 0;
                Min = 0;
                //------------------------------------------------------------
            }
        }
    }

    printf("Total Samples:%d, Total Words: %d \n", sampleCounter, wordCounter);

    while (1) {}
}

/* **************************************************************************** */
void I2SInit()
{
    mxc_i2s_req_t req;
    int32_t err;

    printf("\n*** I2S & Mic Init ***\n");
    /* Initialize High Pass Filter */
    HPF_init();
    /* Initialize I2S RX buffer */
    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
    /* Configure I2S interface parameters */
    req.wordSize = MXC_I2S_DATASIZE_WORD;
    req.sampleSize = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.justify = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    /* Get only left channel data from on-board microphone. Right channel samples are zeros */
    req.stereoMode = MXC_I2S_MONO_LEFT_CH;
    req.bitOrder = MXC_I2S_MSB_FIRST;
    /* I2S clock = PT freq / (2*(req.clkdiv + 1)) */
    /* I2S sample rate = I2S clock/64 = 16kHz */
    req.clkdiv = 5;
    req.rawData = NULL;
    req.txData = NULL;
    req.rxData = i2s_rx_buffer;
    req.length = I2S_RX_BUFFER_SIZE;

    if ((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
        printf("\nError in I2S_Init: %d\n", err);

        while (1) {}
    }

    /* Set I2S RX FIFO threshold to generate interrupt */
    MXC_I2S->ctrl0ch0 &= ~(0xff << 24);
    MXC_I2S_SetRXThreshold(8);
    MXC_NVIC_SetVector(I2S_IRQn, i2s_isr);
    NVIC_EnableIRQ(I2S_IRQn);
    /* Enable RX FIFO Threshold Interrupt */
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
    MXC_I2S_RXEnable();
    __enable_irq();
}

/* **************************************************************************** */
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data, int16_t *out_class, double *out_prob)
{
    int32_t temp[NUM_OUTPUTS];
    q15_t max = 0; // soft_max output is 0->32767
    int32_t max_ml = 1 << 31; // ml before going to soft_max
    int16_t max_index = -1;

    memcpy(temp, ml_data, sizeof(int32_t) * NUM_OUTPUTS);

    /* find the top 5 classes */
    for (int top = 0; top < 5; top++) {
        /* find the class with highest */
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            if ((int32_t)temp[i] > max_ml) {
                max_ml = (int32_t)temp[i];
                max = ml_soft[i];
                max_index = i;
            }
        }

        /* print top 1 separately */
        if (top == 0) {
            *out_class = max_index;
            setSnakeDirection(max_index);

            *out_prob = 100.0 * max / 32768.0;
            break;
        }
    }

    printf("Min: %d,   Max: %d \n", Min, Max);

    /* check if probability is low */
    if (*out_prob > INFERENCE_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

/* **************************************************************************** */
void fail(void)
{
    printf("\n*** FAIL ***\n\n");

    while (1) {}
}

/* **************************************************************************** */
uint8_t cnn_load_data(uint8_t *pIn)
{
    uint32_t mem;
    uint16_t index = 0;

    /* data should already be formated correctly */
    /* pIn is 16KB, each 1KB belongs to a memory group */
    for (mem = 0x50400000; mem <= 0x50418000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        //printf("%.10X \n",(uint8_t *)mem);
        index += 1024;
    }

    for (mem = 0x50800000; mem <= 0x50818000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x50C00000; mem <= 0x50C18000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x51000000; mem <= 0x51018000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    return CNN_OK;
}

/* **************************************************************************** */
uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize, uint16_t outSize, uint16_t width)
{
    /* Data order in Ai85 memory (transpose is included):
    input(series of 8 bit samples): (0,0) ...  (0,127)  (1,0) ... (1,127) ...... (127,0)...(127,127)    16384 samples
    output (32bit word): 16K samples in a buffer. Later, each 1K goes to a seperate CNN memory group
    0x0000:
        (0,3)(0,2)(0,1)(0,0)
        (0,67)(0,66)(0,65)(0,64)
        (1,3)(1,2)(1,1)(1,0)
        (1,67)(1,66)(1,65)(1,64)
        ....
        (127,67)(127,66)(127,65)(127,64)
    0x0400:
        (0,7)(0,6)(0,5)(0,4)
        (0,71)(0,70)(0,69)(0,68)
        ....
        (127,71)(127,70)(127,69)(127,68)
    ...
    0x3C00:
        (0,63)(0,62)(0,61)(0,60)
        (0,127)(0,126)(0,125)(0,124)
        ....
        (127,127)(127,126)(127,125)(127,124)
    */

    static uint16_t row = 0, col = 0, total = 0;
    uint16_t secondHalf = 0, wordRow = 0, byteInWord = 0, group = 0, index = 0;

    for (int i = 0; i < inSize; i++) {
        /* is it above 63? */
        if (col >= (width >> 1)) {
            secondHalf = 1; // odd word rows
        } else {
            secondHalf = 0; // even word rows
        }

        /* which group (0-15) it should be */
        group = (col % (width >> 1)) / 4;

        /* which word row (0-255) within the group */
        wordRow = secondHalf + (row << 1);

        /* which byte (0-3) in the word */
        byteInWord = col % 4;

        /* find output index */
        index = 1024 * group + 4 * wordRow + byteInWord;

        /* place sample in correct output location */
        pOut[index] = pIn[i];

        total++;

        /* increment row and col index */
        col++;

        if (col >= width) {
            col = 0;
            row++;
        }
    }

    if (total >= outSize) {
        /* sanity check */
        if (row != width) {
            printf("ERROR: Rearranging!\n");
        }

        total = 0;
        row = 0;
        col = 0;
        return 1;
    } else {
        return 0;
    }
}

/* **************************************************************************** */
uint8_t MicReadChunk(uint8_t *pBuff, uint16_t *avg)
{
    static uint16_t chunkCount = 0;
    static uint16_t sum = 0;
    static uint32_t index = 0;

    int32_t sample = 0;
    int16_t temp = 0;
    uint32_t rx_size = 0;

    /* sample not ready */
    if (!i2s_flag) {
        *avg = 0;
        return 0;
    }

    /* Clear flag */
    i2s_flag = 0;
    /* Read number of samples in I2S RX FIFO */
    rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;

    /* read until fifo is empty or enough samples are collected */
    while ((rx_size--) && (chunkCount < CHUNK)) {
        /* Read microphone sample from I2S FIFO */
        sample = (int32_t)MXC_I2S->fifoch0;
        /* The actual value is 18 MSB of 32-bit word */
        temp = sample >> 14;

        /* Remove DC from microphone signal */
        sample = HPF((int16_t)temp); // filter needs about 1K sample to converge

        /* Discard first 10k samples due to microphone charging cap effect */
        if (index++ < 10000) {
            continue;
        }

        /* Turn on LED2 (Red) */
        LED_On(LED2);

        /* absolute for averaging */
        if (sample >= 0) {
            sum += sample;
        } else {
            sum -= sample;
        }

        /* Convert to 8 bit unsigned */
        pBuff[chunkCount] = (uint8_t)((sample)*SAMPLE_SCALE_FACTOR / 256);

        temp = (int8_t)pBuff[chunkCount];

        chunkCount++;

        /* record max and min */
        if (temp > Max) {
            Max = temp;
        }

        if (temp < Min) {
            Min = temp;
        }
    }

    /* if not enough samples, return 0 */
    if (chunkCount < CHUNK) {
        *avg = 0;
        return 0;
    }

    /* enough samples are collected, calculate average and return 1 */
    *avg = ((uint16_t)(sum / CHUNK));

    chunkCount = 0;
    sum = 0;
    return 1;
}

static int16_t x0, x1, Coeff;
static int32_t y0, y1;

/************************************************************************************/
void HPF_init(void)
{
    Coeff = 32604; //0.995
    x0 = 0;
    y0 = 0;
    y1 = y0;
    x1 = x0;
}

/************************************************************************************/
int16_t HPF(int16_t input)
{
    int16_t Acc, output;
    int32_t tmp;

    /* a 1st order IIR high pass filter (100 Hz cutoff frequency)  */
    /* y(n)=x(n)-x(n-1)+A*y(n-1) and A =.995*2^15 */

    x0 = input;

    tmp = (Coeff * y1);
    Acc = (int16_t)((tmp + (1 << 14)) >> 15);
    y0 = x0 - x1 + Acc;

    /* Clipping */
    if (y0 > 32767) {
        y0 = 32767;
    }

    if (y0 < -32768) {
        y0 = -32768;
    }

    /* Update filter state */
    y1 = y0;
    x1 = x0;

    output = (int16_t)y0;

    return (output);
}

/************************************************************************************/
void snakeIntro(void)
{
    MXC_TFT_ClearScreen();
    //MXC_TFT_ShowImage(0, 0, image_bitmap_2);
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 5, 40, font_1, sprintf(buff, "Following keywords are used in"));
    TFT_Print(buff, 5, 65, font_1, sprintf(buff, "this game:"));
    TFT_Print(buff, 10, 90, font_1, sprintf(buff, "1...4, go, up, down, left, right"));
    TFT_Print(buff, 10, 115, font_1, sprintf(buff, "STOP command exits game"));
    MXC_Delay(2000000);
    TFT_Print(buff, 5, 160, font_2, sprintf(buff, "Choose game level now...."));
    TFT_Print(buff, 5, 190, font_1, sprintf(buff, "Say 1/2/3 or 4 to choose level"));
}

// ******************************************************************************
void setup()
{
    uint32_t color;
    mxc_tmr_cfg_t tmr1 = { TMR_PRES_4,      TMR_MODE_CONTINUOUS,
                           TMR_BIT_MODE_32, MXC_TMR_APB_CLK,
                           tmr1_cmp,        0 };

    MXC_TFT_ClearScreen();
    color = setColor(0, 0, 0); // complete black
    MXC_TFT_WritePixel(0, 0, 320, 240, color);
    // Top grey
    color = setColor(48, 48, 48);
    MXC_TFT_WritePixel(0, 0, 320, 24, color);
    //MXC_TFT_ShowImage(0, 0, image_bitmap_2);
    MXC_Delay(100);

    memset(buff, 0, TFT_BUFF_SIZE);
    TFT_Print(buff, 200, 1, font_1, sprintf(buff, "Score: %d   ", score));
    // Draw white boundary line
    color = setColor(255, 255, 255);
    MXC_TFT_WritePixel(0, 25, 320, 2, color);

    // Initialize the position of the snake
    const int startX = 8;
    const int startY = 10;

    // Draw snake with these positions
    for (int i = 0; i < snakeLength; i++) {
        positions[0][i] = startX + i;
        positions[1][i] = startY;
        // Draw square boxes to determine where is the snake. The snake dimension are proportional to scale
        color = setColor(255, 255, 255); // Snake in white
        MXC_TFT_WritePixel(scale * positions[0][i], scale * positions[1][i], scale, scale, color);
    }

    MXC_Delay(500000);

    // place the fruit
    placeFruit();
    MXC_TMR_Start(MXC_TMR2);

    // Initialize Timer to move snake on timeout
    MXC_TMR_Init(MXC_TMR1, &tmr1, false);
    MXC_NVIC_SetVector(TMR1_IRQn, tmr_isr);
    NVIC_EnableIRQ(TMR1_IRQn);
    MXC_TMR_EnableInt(MXC_TMR1);
    NVIC_SetPriority(TMR1_IRQn, 7);
    NVIC_SetPriority(I2S_IRQn, 5);
    MXC_TMR_Start(MXC_TMR1);
}

// ******************************************************************************
void placeFruit()
{
    uint32_t color;

    // Ensure not to place fruit where there is a snake
    while (true) {
        int flag = 0;
        // Don't put the fruit outside the screen boundary
        const int tolerance = 3;
        // locate the fruit at random place
        fruit[0] = (int)genRandomNum(tolerance, xScreen - tolerance);
        fruit[1] = (int)genRandomNum(20, yScreen - tolerance);

        // Check if a part of the snake is on the fruit
        for (int j = 0; j < snakeLength; j++) {
            // Check if x co-ordinate is same
            if (fruit[0] == positions[0][j]) {
                // check if y co-ordinate is same
                if (fruit[1] == positions[1][j]) {
                    // increment the flag, fruit lies over the snake
                    flag++;
                }
            }
        }

        // if the flag is 0 we don't have the fruit over a part of the snake
        if (flag == 0) {
            break;
        }
    }

    // Draw the fruit
    color = setColor(0, 255, 0);
    MXC_TFT_WritePixel(scale * fruit[0], scale * fruit[1], scale, scale, color);
}

// ******************************************************************************
void setSnakeDirection(int top_index)
{
    prevSnakeDirection = snakeDirection;

    if (gameStart) {
        if (top_index == 0 && snakeDirection != DOWN) {
            snakeDirection = UP;

        } else if (top_index == 1 && snakeDirection != UP) {
            snakeDirection = DOWN;

        } else if (top_index == 2 && snakeDirection != RIGHT) {
            snakeDirection = LEFT;

        } else if (top_index == 3 && snakeDirection != LEFT) {
            snakeDirection = RIGHT;

        } else if (top_index == 4) {
            snakeDirection = STOPGAME;
            gameOver = 1;

        } else {
            snakeDirection = prevSnakeDirection;
        }
    } else {
        if (chooseLevel) {
            if (top_index == 5 && snakeDirection == UNKNOWN) {
                snakeDirection = GO;
                gameStart = 1;
                setup();
            }
        } else {
            if (top_index == 10) {
                gameSpeed = LEVEL1;
                tmr1_cmp = 5000000; // Count for 400ms

            } else if (top_index == 11) {
                gameSpeed = LEVEL2;
                tmr1_cmp = 3750000; // Count for 300ms

            } else if (top_index == 12) {
                gameSpeed = LEVEL3;
                tmr1_cmp = 2500000; // Count for 200ms

            } else {
                if (top_index == 13) {
                    gameSpeed = LEVEL4;
                    tmr1_cmp = 1875000; // Count for 150ms
                }
            }

            if (gameSpeed != ZERO) {
                chooseLevel = 1;
                area_t area = { 0, 160, 320, 80 };
                MXC_TFT_ClearArea(&area, 4);
                TFT_Print(buff, 5, 160, font_1, sprintf(buff, "Level Selected: %d ", gameSpeed));
                TFT_Print(buff, 5, 190, font_1, sprintf(buff, "Say 'GO' to start the game"));
            }
        }
    }
}

// ******************************************************************************
void checkSnakeCollision()
{
    for (int k = 0; k < snakeLength - 1; k++) {
        if (positions[0][k] == positions[0][snakeLength - 1] &&
            positions[1][k] == positions[1][snakeLength - 1]) {
            gameOver = 1;
        }
    }
}

// ******************************************************************************
boolean fruitIsEaten()
{
    uint32_t color;

    // Check if snake head overlaps on fruit
    if (positions[0][snakeLength - 1] == fruit[0] && positions[1][snakeLength - 1] == fruit[1]) {
        // Increment score
        MXC_TMR_Stop(MXC_TMR2);
        score++;
        color = setColor(48, 48, 48); // make color same as background
        MXC_TFT_WritePixel(260, 0, 320, 24, color);
        memset(buff, 0, TFT_BUFF_SIZE);
        TFT_Print(buff, 260, 1, font_1, sprintf(buff, " %d ", score));

        // Draw the top boundary line
        color = setColor(255, 255, 255);
        MXC_TFT_WritePixel(0, 25, 320, 2, color);
        color = setColor(255, 255, 255);
        MXC_TFT_WritePixel(scale * fruit[0], scale * fruit[1], scale, scale, color);
        // Put the fruit at different place
        placeFruit();
        MXC_TMR_Start(MXC_TMR2);

        // Control snake speed based on score
        if (gameSpeed == LEVEL1) {
            if ((score % 2) == 0) {
                MXC_TMR_Stop(MXC_TMR1);
                MXC_TMR1->cnt = 0;

                if (score < 32) {
                    MXC_TMR1->cmp = MXC_TMR1->cmp - COUNT_20ms;
                } else {
                    MXC_TMR1->cmp = MXC_TMR1->cmp - COUNT_200us;
                }

                MXC_TMR_Start(MXC_TMR1);
            }
        } else {
            if ((score % gameSpeed) == 0) {
                MXC_TMR_Stop(MXC_TMR1);
                MXC_TMR1->cnt = 0;

                if (score < 16) {
                    MXC_TMR1->cmp = MXC_TMR1->cmp - COUNT_30ms;
                } else {
                    MXC_TMR1->cmp = MXC_TMR1->cmp - COUNT_100us;
                }

                MXC_TMR_Start(MXC_TMR1);
            }
        }

        return true;
    }

    return false;
}

// ******************************************************************************
void updatePositions()
{
    uint32_t color;
    // Delete the tail of the snake
    color = setColor(0, 0, 0); // make it the same color of the background
    MXC_TFT_WritePixel(scale * positions[0][0], scale * positions[1][0], scale, scale, color);

    // next update positions
    for (int i = 0; i < snakeLength - 1; i++) {
        // update the positions
        positions[0][i] = positions[0][i + 1];
        positions[1][i] = positions[1][i + 1];
    }
}

// ******************************************************************************
void drawSnakeHead()
{
    uint32_t color;
    // this function draw the new head of the snake
    color = setColor(255, 255, 255);
    MXC_TFT_WritePixel(scale * positions[0][snakeLength - 1], scale * positions[1][snakeLength - 1],
                       scale, scale, color);
}

// ******************************************************************************
void moveTheSnake()
{
    // move the snake according to the direction
    switch (snakeDirection) {
    case UP: { // move up
        if (fruitIsEaten()) {
            snakeLength++; // increment snake length
            positions[1][snakeLength - 1] = (positions[1][snakeLength - 2] - 1);

            if (positions[1][snakeLength - 1] <= 5) {
                positions[1][snakeLength - 1] = yScreen - 1;
            }

            positions[0][snakeLength - 1] = positions[0][snakeLength - 2];
            // draw the new head of the snake
            drawSnakeHead();
        } else {
            // update position
            updatePositions();
            // move also the terminal of the snake
            positions[1][snakeLength - 1] = (positions[1][snakeLength - 1] - 1);

            if (positions[1][snakeLength - 1] <= 5) {
                positions[1][snakeLength - 1] = yScreen - 1;
            }

            // draw the new head
            drawSnakeHead();
        }

        break;
    }

    case DOWN: { // move down
        if (fruitIsEaten()) {
            snakeLength++; // increment snake length

            if (((positions[1][snakeLength - 2] + 1) % yScreen) == 0) {
                positions[1][snakeLength - 1] = (positions[1][snakeLength - 2] + 1) % yScreen + 6;
            } else {
                positions[1][snakeLength - 1] = (positions[1][snakeLength - 2] + 1) % yScreen;
            }

            positions[0][snakeLength - 1] = positions[0][snakeLength - 2];
            // draw the new head of the snake
            drawSnakeHead();
        } else {
            // update positions
            updatePositions();

            // move also the terminal of the snake
            if (((positions[1][snakeLength - 1] + 1) % yScreen) == 0) {
                positions[1][snakeLength - 1] = (positions[1][snakeLength - 1] + 1) % yScreen + 6;
            } else {
                positions[1][snakeLength - 1] = (positions[1][snakeLength - 1] + 1) % yScreen;
            }

            // draw the new head
            drawSnakeHead();
        }

        break;
    }

    case LEFT: { // move left
        if (fruitIsEaten()) {
            snakeLength++; // increment snake length
            positions[0][snakeLength - 1] = (positions[0][snakeLength - 2] - 1);

            // if it goes utside replace on the other part
            if (positions[0][snakeLength - 1] <= 0) {
                positions[0][snakeLength - 1] = xScreen - 1;
            }

            positions[1][snakeLength - 1] = positions[1][snakeLength - 2];
            // draw the new head of the snake
            drawSnakeHead();
        } else {
            // update positions
            updatePositions();
            // move also the terminal of the snake
            positions[0][snakeLength - 1] = (positions[0][snakeLength - 1] - 1);

            if (positions[0][snakeLength - 1] <= 0) {
                positions[0][snakeLength - 1] = xScreen - 1;
            }

            // draw the new head
            drawSnakeHead();
        }

        break;
    }

    case RIGHT: { // move right
        if (fruitIsEaten()) {
            snakeLength++; // increment snake length
            positions[0][snakeLength - 1] = (positions[0][snakeLength - 2] + 1) % xScreen;
            positions[1][snakeLength - 1] = positions[1][snakeLength - 2];
            // draw the new head of the snake
            drawSnakeHead();
        } else {
            // update positions
            updatePositions();
            // move also the terminal of the snake
            positions[0][snakeLength - 1] = (positions[0][snakeLength - 1] + 1) % xScreen;
            // draw the new head
            drawSnakeHead();
        }

        break;
    }

    default: {
        break;
    }
    }
}
