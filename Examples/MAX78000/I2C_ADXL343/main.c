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

/*
  main.c

  MAX78000FTHR I2C example for ADXL343.

  This example continuously prints ADXL343 data.

  The ADXL343 INT2 output is connected GPIO6, P0_19_GPIO of the MAX78000FTHR.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mxc.h"
#include "adxl343.h"
#include "board.h"
#include "led.h"

#ifdef BOARD_EVKIT_V1
#warning "This example is not supported by the MAX78000EVKIT."
#endif

#define WAIT_FOR_KEYPRESS

// The I2C peripheral the ADXL343 is connected to and the bus speed.
#define I2C_INST MXC_I2C1
#define I2C_FREQ 400000

// The GPIO pin used for ADXL343 interrupt.
#define ADXL343_IRQ_PORT MXC_GPIO0
#define ADXL343_IRQ_PIN  MXC_GPIO_PIN_19

static mxc_gpio_cfg_t adxl343_irq_cfg = {
  .port = ADXL343_IRQ_PORT,
  .mask = ADXL343_IRQ_PIN,
  .pad = MXC_GPIO_PAD_NONE,
  .func = MXC_GPIO_FUNC_IN,
  .vssel = MXC_GPIO_VSSEL_VDDIOH
};

// Flag shared between interrupt handler and differed work service loop
static volatile bool axis_data_ready = false;

/*
  Print message and blink LEDs until reset.
*/
void blink_halt(char *msg)
{
  puts(msg);
  puts("Reset to restart application.");

  for (;;) {
    MXC_Delay(MXC_DELAY_MSEC(250));
    LED_Toggle(LED1);
  }
}

/*
  ADXL343 interrupt handler.

  Set global data ready flag.
*/
void adxl343_handler(void *cbdata)
{
  (void)cbdata;

  axis_data_ready = true;
}

/*
  ADXL343 configuration.

  Configures GPIO pin as external interrupt and ADXL343 for continuous operation.
*/
int adxl343_config(void)
{
  int result;
  int16_t tmp[3];
  int8_t axis_offsets[3] = { -2, -2, 7 };   // Device specific offset calibration values

  MXC_GPIO_Config(&adxl343_irq_cfg);
  MXC_GPIO_RegisterCallback(&adxl343_irq_cfg, adxl343_handler, NULL);
  MXC_GPIO_IntConfig(&adxl343_irq_cfg, MXC_GPIO_INT_RISING);
  MXC_GPIO_EnableInt(adxl343_irq_cfg.port, adxl343_irq_cfg.mask);
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(ADXL343_IRQ_PORT)));

  result = 0;
  result |= adxl343_set_power_control(ADXL343_PWRCTL_STANDBY);
  result |= adxl343_set_int_enable(0);
  result |= adxl343_get_axis_data(tmp);               // Unload any unread data
  result |= adxl343_set_data_rate(ADXL343_DR_25HZ);
  result |= adxl343_set_range(ADXL343_RANGE_2G);
  result |= adxl343_set_power_mode(ADXL343_PWRMOD_NORMAL);
  result |= adxl343_set_offsets(axis_offsets);
  result |= adxl343_set_fifo_mode(ADXL343_FIFO_BYPASS);
  result |= adxl343_set_int_map(ADXL343_INT_DATA_READY);
  result |= adxl343_set_int_enable(ADXL343_INT_DATA_READY);
  result |= adxl343_set_power_control(ADXL343_PWRCTL_MEASURE);

  return result;
}

/*
  Print message and wait for keypress
*/
void wait_for_keypress(void)
{
  const char msg[] = "Press Enter or Return to continue ...";

  printf("%s", msg);
  getchar();
  printf("%*s", sizeof(msg) - 1, "");
}

int main(void)
{
  int16_t axis_data[3];

  MXC_ICC_Enable(MXC_ICC0);
  MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
  SystemCoreClockUpdate();

   // Disable stdout buffering
  setbuf(stdout, NULL);
  printf("\nMAX78000FTHR I2C ADXL343 demo.\n");

  if (MXC_I2C_Init(I2C_INST, 1, 0) != E_NO_ERROR) {
    blink_halt("Trouble initializing I2C instance.");
  }

  MXC_I2C_SetFrequency(I2C_INST, I2C_FREQ);

  if (adxl343_init(I2C_INST) != E_NO_ERROR) {
    blink_halt("Trouble initializing ADXL343.");
  }

  if (adxl343_config() != E_NO_ERROR) {
    blink_halt("Trouble configuring ADXL343.");
  }

  // Use delay or wait for keypress to allow debugger to attach before entering low power mode
#if !defined(WAIT_FOR_KEYPRESS)
  MXC_Delay(MXC_DELAY_SEC(3));
#else
  wait_for_keypress();
#endif

  for (;;) {
    if (axis_data_ready) {
      axis_data_ready = false;

      if (adxl343_get_axis_data(axis_data) != E_NO_ERROR) {
        blink_halt("Trouble reading ADXL343.");
      }

      printf("\rx:% -2.2f  y:% -2.2f  z:% -2.2f",
             axis_data[0] * ADXL343_SF_2G,
             axis_data[1] * ADXL343_SF_2G,
             axis_data[2] * ADXL343_SF_2G);
    }

    MXC_LP_EnterSleepMode();
  }
}
