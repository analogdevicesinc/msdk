
#include <stdint.h>
#include <stdbool.h>
#include "mxc_sys.h"

struct mxc_crit_state
{
    uint32_t primask;
    bool in_critical;
} _state = { 
    .primask = 0xFFFFFFFF, 
    .in_critical = false 
    };

void MXC_SYS_Crit_Enter(void) 
{
    _state.primask = __get_PRIMASK();
    __disable_irq();
    _state.in_critical = true;
}

void MXC_SYS_Crit_Exit(void)
{
    if (_state.primask == 0) {
        __enable_irq();
    }
    _state.in_critical = false;
}

int MXC_SYS_In_Crit_Section(void)
{
    return _state.in_critical;
}

