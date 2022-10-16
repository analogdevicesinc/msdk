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
 * @brief   A basic getting started program for the RISCV, run from the ARM
 * core.
 * @details Dual_core_sync_arm runs on the ARM core to load the RISCV code
 * space, setup the RISCV debugger pins, and start the RISCV core.
 */

/***** Includes *****/
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sema_reva.h"

/***** Definitions *****/
#define DUAL_CORE_SYNC (1)

#define NDX_ARM (0)
#define NDX_RISCV (1)

#define MAILBOX_OVERHEAD (2 * sizeof(uint16_t))
#define MAILBOX_PAYLOAD_LEN (MAILBOX_SIZE - MAILBOX_OVERHEAD)
typedef struct {
  uint16_t readLocation;
  uint16_t writeLocation;
#if (MAILBOX_SIZE == 0)
  uint8_t payload[1];
#else
  uint8_t payload[MAILBOX_PAYLOAD_LEN];
#endif

} mxcSemaBox_t;

/***** Globals *****/
extern mxcSemaBox_t *mxcSemaBox0; // ARM writes, RISCV reads,
extern mxcSemaBox_t *mxcSemaBox1; // ARM reads,  RISCV writes

/***** Functions *****/

// *****************************************************************************
int main(void) {
#if DUAL_CORE_SYNC
  printf("\n\n\n-----------------------------------\n");
  printf("ARM   : Start.\n");

  MXC_SEMA_Init();

  int ret = MXC_SEMA_CheckSema(NDX_ARM);
  printf("ARM   : After init, CheckSema(%d) returned %s.\n", NDX_ARM,
         ret == E_BUSY ? "BUSY" : "NOT BUSY");

  if ((MXC_SEMA_GetSema(NDX_ARM)) == E_NO_ERROR) {
    printf(
        "ARM   : GetSema returned NOT BUSY with previous semaphore value %d.\n",
        MXC_SEMA->semaphores[NDX_ARM]);
  } else {
    printf(
        "ARM   : GetSema returned - BUSY - with previous semaphore value %d.\n",
        MXC_SEMA->semaphores[NDX_ARM]);
  }

  printf("ARM   : Wait 2 secs then start the RISC-V core.\n");
#endif
  MXC_Delay(MXC_DELAY_SEC(2));

  /* Enable RISCV debugger GPIO */
  MXC_GPIO_Config(&gpio_cfg_rv_jtag);

  /* Start the RISCV core */
  MXC_SYS_RISCVRun();

#if DUAL_CORE_SYNC
  /* Wait the RISC-V core to start */
  ret = E_BUSY;
  while (E_BUSY == ret) {
    ret = MXC_SEMA_CheckSema(NDX_ARM);
  }
  MXC_SEMA_GetSema(NDX_ARM);

  /* Init code here. */
  printf("ARM   : Do initialization works here.\n");
  MXC_SEMA_InitBoxes();

  /* Signal RISC-V core to run */
  printf("ARM   : Signal RISC-V.\n");
  MXC_SEMA_FreeSema(NDX_RISCV);

  uint32_t cnt;
#endif
  /* Enter LPM */
  while (1) {
#if DUAL_CORE_SYNC
    /* Wait */
    ret = MXC_SEMA_CheckSema(NDX_ARM);
    if (E_BUSY != ret) {
      MXC_SEMA_GetSema(NDX_ARM);

      /* Do the job. */
      // Retrieve the data from the mailbox1
      cnt = mxcSemaBox1->payload[0] << (8 * 0);
      cnt += mxcSemaBox1->payload[1] << (8 * 1);
      cnt += mxcSemaBox1->payload[2] << (8 * 2);
      cnt += mxcSemaBox1->payload[3] << (8 * 3);

      printf("ARM   : cnt=%d\n", cnt++);

      mxcSemaBox0->payload[0] = (cnt >> 8 * 0) & 0xFF;
      mxcSemaBox0->payload[1] = (cnt >> 8 * 1) & 0xFF;
      mxcSemaBox0->payload[2] = (cnt >> 8 * 2) & 0xFF;
      mxcSemaBox0->payload[3] = (cnt >> 8 * 3) & 0xFF;

      /* Do other jobs here. */
      MXC_Delay(MXC_DELAY_SEC(1));

      /* Signal */
      MXC_SEMA_FreeSema(NDX_RISCV);
    }
#endif
  }
}
