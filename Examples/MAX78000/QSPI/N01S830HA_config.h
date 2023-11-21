#ifndef N01S830HA_CONFIG_H
#define N01S830HA_CONFIG_H

#include "N01S830HA.h"
#include "gpio.h"
#include "mxc_device.h"

#ifndef N01S830HA_HOLD_PIN_PORT
#define N01S830HA_HOLD_PIN_PORT MXC_GPIO0
#endif

#ifndef N01S830HA_HOLD_PIN_MASK
#define N01S830HA_HOLD_PIN_MASK MXC_GPIO_PIN_9
#endif

#ifndef N01S830HA_VSSEL
#define N01S830HA_VSSEL MXC_GPIO_VSSEL_VDDIOH
#endif

#ifndef N01S830HA_HOLD_PIN
#define N01S830HA_HOLD_PIN
// N01S830HA_HOLD_PIN can be defined by other files to completely re-define the
// hold pin struct if necessary.  Otherwise, the hold pin will be defined here using
// the port, pin, and vssel definitions above.

static const mxc_gpio_cfg_t N01S830HA_hold_pin = { .port = N01S830HA_HOLD_PIN_PORT,
                                           .mask = N01S830HA_HOLD_PIN_MASK,
                                           .func = MXC_GPIO_FUNC_OUT,
                                           .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
                                           .vssel = N01S830HA_VSSEL };
#endif

#endif // N01S830HA_CONFIG_H