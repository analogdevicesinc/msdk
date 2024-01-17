#ifndef _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_OPTIONS_H
#define _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_OPTIONS_H

#include "mxc_microros_config.h"

/* UART instance to use for the micro-ROS serial transport. Ex: MXC_UART0 */
#ifndef configMXC_SERIAL_UART
#error mxc_microros_config.h error!  Missing config option: 'configMXC_SERIAL_UART'
#endif

/* BAUD rate to use for the micro-ROS serial transport.  Ex: 115200 */
#ifndef configMXC_SERIAL_BAUDRATE
#error mxc_microros_config.h error!  Missing config option: 'configMXC_SERIAL_BAUDRATE'
#endif

/* Depth of the stack given to the appMain task.  This is in units of "configSTACK_DEPTH_TYPE",
which is defined in FreeRTOS.h and optionally overridden in FreeRTOSConfig.h */
#ifndef configAPPMAIN_STACK_DEPTH
#define configAPPMAIN_STACK_DEPTH 2048
#endif

#endif // _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_OPTIONS_H