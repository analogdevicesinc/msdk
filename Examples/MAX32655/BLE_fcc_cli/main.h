#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ll_init_api.h"
#include "chci_tr.h"
#include "lhci_api.h"
#include "hci_defs.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_timer.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "bb_ble_sniffer_api.h"
#include "pal_bb.h"
#include "pal_cfg.h"
#include "tmr.h"
#include "wut_regs.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "uart.h"
#include "mxc_delay.h"
typedef union {
    struct {
        uint8_t channel;
        uint16_t duration;
        uint8_t unused;
    };
    uint32_t allData;
} tx_task_command_t;

#endif