#ifndef CONFIG_H
#define CONFIG_H
#include "board.h"

#if defined(BOARD_EVKIT_V1)
#define ADC_CHANNEL     MXC_ADC_CH_0
#elif defined(BOARD_FTHR_REVA)
#define ADC_CHANNEL     MXC_ADC_CH_3
#endif

/* Change to #undef USE_INTERRUPTS for polling mode */
//#define USE_INTERRUPTS

#endif
