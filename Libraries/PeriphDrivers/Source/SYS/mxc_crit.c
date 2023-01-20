
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

void _get_state()
{
    /*
        The 0th bit of the Priority Mask register indicates
        whether interrupts are enabled or not.

        0 = enabled
        1 = disabled
    */
    _state.primask = __get_PRIMASK();
}

void MXC_SYS_Crit_Enter(void) 
{
    _get_state();
    if (_state.primask == 0) __disable_irq();
    _state.in_critical = true;
}

void MXC_SYS_Crit_Exit(void)
{
    if (_state.primask == 0) {
        __enable_irq();
    }
    _state.in_critical = false;
    _get_state(); 
    /*
        ^ Reset the state again to prevent edge case
        where interrupts get disabled, then Crit_Exit() gets
        called, which would inadvertently re-enable interrupts
        from old state.
    */
}

int MXC_SYS_In_Crit_Section(void)
{
    return _state.in_critical;
}

