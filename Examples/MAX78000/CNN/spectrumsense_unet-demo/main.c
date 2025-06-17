/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "mxc_delay.h"
#include "led.h"
#include "rtc.h"
#ifdef BOARD_EVKIT_V1
#include "bitmap.h"
#include "tft_ssd2119.h"
#endif
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif

#ifdef BOARD_EVKIT_V1
int font = urw_gothic_12_grey_bg_white;
#endif
#ifdef BOARD_FTHR_REVA
int font = (int)&Liberation_Sans16x16[0];
#endif

volatile uint32_t cnn_time; // Stopwatch

#define CON_BAUD 115200
#define NUM_PIXELS 7744 // 88x88
#define NUM_IN_CHANNLES 48
#define NUM_OUT_CHANNLES 48
#define INFER_SIZE 23232 // size of inference 48x88x88/16
#define TFT_BUFF_SIZE 50 // TFT buffer size

void fail(void)
{
    printf("\n*** FAIL ***\n\n");

    while (1) {}
}

int console_UART_init(uint32_t baud)
{
    mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
    int err;
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_SetPriority(MXC_UART_GET_IRQ(CONSOLE_UART), 1);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    if ((err = MXC_UART_Init(ConsoleUart, baud, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return err;
    }

    return 0;
}

uint8_t gen_crc(const void *vptr, int len)
{
    const uint8_t *data = vptr;
    unsigned crc = 0;
    int i, j;

    for (j = len; j; j--, data++) {
        crc ^= (*data << 8);

        for (i = 8; i; i--) {
            if (crc & 0x8000) {
                crc ^= (0x1070 << 3);
            }

            crc <<= 1;
        }
    }

    return (uint8_t)(crc >> 8);
}

static void console_uart_send_byte(uint8_t value)
{
    while (MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), value) == E_OVERFLOW) {}
}

static void console_uart_send_bytes(uint8_t *ptr, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        console_uart_send_byte(ptr[i]);
        //printf("%d\n", ptr[i]);
    }
}

uint32_t utils_get_time_ms(void)
{
    uint32_t sec, ssec;
    double subsec;
    uint32_t ms;
    MXC_RTC_GetSubSeconds(&ssec);
    subsec = ssec / 4096.0;
    MXC_RTC_GetSeconds(&sec);

    ms = (sec * 1000) + (int)(subsec * 1000);

    return ms;
}

void load_input_serial(void)
{
    uint32_t in_data[NUM_PIXELS];
    uint8_t rxdata[4];
    uint32_t tmp;
    uint8_t crc, crc_result;
    uint32_t index = 0;
    LED_Off(LED2);

    printf("READY\n");

    uint32_t *data_addr = (uint32_t *)0x50400700;

    for (int ch = 0; ch < NUM_IN_CHANNLES; ch += 4) {
        LED_Toggle(LED1);

        for (int i = 0; i < NUM_PIXELS; i++) {
            index++;
            tmp = 0;

            for (int j = 0; j < 4; j++) {
                rxdata[j] = MXC_UART_ReadCharacter(MXC_UART_GET_UART(CONSOLE_UART));
                tmp = tmp | (rxdata[j] << 8 * (3 - j));
            }

            //read crc
            crc = MXC_UART_ReadCharacter(MXC_UART_GET_UART(CONSOLE_UART));
            crc_result = gen_crc(rxdata, 4);

            if (crc != crc_result) {
                printf("E %d", index);
                LED_On(LED2);

                while (1) {}
            }

            //fill input buffer
            in_data[i] = tmp;
        }

        // load data to cnn
        memcpy32(data_addr, in_data, NUM_PIXELS);
        // printf("%d- %08X \n",ch,data_addr);
        data_addr += 0x2000;

        if ((data_addr == (uint32_t *)0x50420700) || (data_addr == (uint32_t *)0x50820700) ||
            (data_addr == (uint32_t *)0x50c20700)) {
            data_addr += 0x000f8000;
        }
    }
}

// Expected output given the sample input (known-answer test)
// Delete this function for production code
//static const uint32_t sample_output[] = SAMPLE_OUTPUT;
int check_output(void)
{
    int i;
    uint32_t mask, len;
    volatile uint32_t *addr;
    const uint32_t *ptr = 0; //sample_output;

    while ((addr = (volatile uint32_t *)*ptr++) != 0) {
        mask = *ptr++;
        len = *ptr++;

        for (i = 0; i < len; i++)
            if ((*addr++ & mask) != *ptr++) {
                return CNN_FAIL;
            }
    }

    return CNN_OK;
}

void send_output(void)
{
    uint8_t *data_addr = (uint8_t *)0x50400000;

    printf("SENDING_OUTPUT\n");

    for (int ch = 0; ch < NUM_OUT_CHANNLES; ch += 4) {
        console_uart_send_bytes(data_addr, 4 * NUM_PIXELS);
        data_addr += 0x8000;

        if ((data_addr == (uint8_t *)0x50420000) || (data_addr == (uint8_t *)0x50820000) ||
            (data_addr == (uint8_t *)0x50c20000)) {
            data_addr += 0x003e0000;
        }
    }
}

void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;

    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

int main(void)
{
    char buff[TFT_BUFF_SIZE];

    /* Get current time */
    static uint32_t t1, t2, t3, t4, t5;

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
    printf("\n\nRF Spectrum Sensing Feather Demo\n");
#else
    printf("\n\nRF Spectrum Sensing Evkit Demo\n");
#endif
    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    // Initialize UART
    console_UART_init(CON_BAUD);

    // Initialize RTC
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    printf("Start SerialLoader.py script...\n");

    // Initialize TFT display.
    printf("Init TFT\n");
#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
#endif
#ifdef BOARD_FTHR_REVA
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_270);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif
    MXC_TFT_ClearScreen();
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 55, 30, font, snprintf(buff, sizeof(buff), "ANALOG DEVICES                "));
    TFT_Print(buff, 15, 50, font, snprintf(buff, sizeof(buff), "RF Spectrum Sensing Demo      "));
    TFT_Print(buff, 120, 90, font, snprintf(buff, sizeof(buff), "Ver. 1.0.0                    "));
    MXC_Delay(SEC(2));

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: 50 MHz div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5); // Turn on the boost circuit
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine

    while (1) {
        LED_Toggle(LED1);

        t1 = utils_get_time_ms();

        load_input_serial(); // Load data input from serial port

        t2 = utils_get_time_ms();

        cnn_start(); // Start CNN processing

        t3 = utils_get_time_ms();
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0

        while (cnn_time == 0) {
            __WFI(); // Wait for CNN
        }

        t4 = utils_get_time_ms();

        send_output(); // send CNN output to UART

        t5 = utils_get_time_ms();

        if (PB_Get(0)) {
#ifdef CNN_INFERENCE_TIMER
            printf("\n*** Approximate inference time: %u us ***\n\n", cnn_time);
#endif
        }

        // print timing data
        printf("load:%d cnn_start:%d cnn_wait:%d send:%d Total:%dms\n", t2 - t1, t3 - t2, t4 - t3,
               t5 - t4, t5 - t1);
    }
}
