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

#endif // _LIBRARIES_MICROROS_SRC_PLATFORM_MXC_MICROROS_OPTIONS_H