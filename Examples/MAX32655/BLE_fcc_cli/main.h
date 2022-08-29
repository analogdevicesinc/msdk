#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ll_init_api.h"
#include "ll_api.h"
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

typedef enum { RX_TEST, TX_TEST } test_t;
//used to post messages to a task via its notification parameter
typedef union {
    struct {
        uint16_t duration_ms;
        uint8_t channel;
        uint8_t testType;
    };
    uint32_t allData;
} tx_config_t;

typedef union {
    struct {
        uint8_t start_channel;
        uint8_t end_channel;
        uint8_t duration_per_ch_ms;
        uint8_t itteraton; /* how many times to sweep */
    };
    uint32_t allData;
} sweep_config_t;

void setPhy(uint8_t newPhy);
void startFreqHopping(void);

#endif