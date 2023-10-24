
/**
 * @file    main.c
 * @brief   Hello World (C++ variant)
 * @details This example uses the UART to print to a terminal and flashes an LED.  It also demonstrates
 * basic class creation and instancing available in C++.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

class LED {
public:
    explicit LED(int index)
    {
        idx = index;
        blink_count = 0;
    }
    void on()
    {
        LED_On(idx);
        ++blink_count;
    }
    void off()
    {
        LED_Off(idx);
    }
    void print_blink_count()
    {
        printf("Number of blinks: %i\n", blink_count);
    }

private:
    int idx;
    int blink_count;
};

int main()
{
    LED led = LED(0);

    printf("C++ Hello World Example\n");
    /*
        Note: Use printf instead of std::cout.
        iostream consumes an extreme amount of code space.  Our printf
        function is better optimized for microcontrollers with limited flash
    */

    while (1) {
        led.on();
        MXC_Delay(500000);
        led.off();
        MXC_Delay(500000);
        led.print_blink_count();
    }

    return 0;
}
