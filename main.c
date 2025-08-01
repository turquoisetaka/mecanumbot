#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <math.h>

#include "tusb.h"
#include "xinput_host.h"


#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define DEADZONE 1500
#define AXIS_MAX 32768.0f
#define WHEEL_DIAMETER 0.086f
#define TRACK_RADIUS 0.085f
#define STEPS_PER_REV 200

//for delay 400 this is 3.377m/s
#define MAX_WHEEL_SPEED 3.14f*WHEEL_DIAMETER*(1000000.0f)/((float)MIN_STEP_DELAY*(float)STEPS_PER_REV)

//actual min is 300 but I want the min to be divisible by the control tick value
#define MIN_STEP_DELAY 400


// define control tick in microseconds and clkdiv as any int to calculate wrap val
//wrap value just needs to be below 65535
#define CONTROL_TICK_US 200
#define CONTROL_PWM_GPIO 15
#define CONTROL_PWM_SLICE pwm_gpio_to_slice_num(CONTROL_PWM_GPIO)
#define CONTROL_CLKDIV 5
#define SYSCLOCK_IN_MHZ 150
#define CONTROL_WRAP 1 + (150*CONTROL_TICK_US)/CONTROL_CLKDIV


int main() {
    
    stdio_init_all();
    
    uart_init(uart0, BAUD_RATE);

    board_init();
    
    tuh_init(BOARD_TUH_RHPORT);

    while(1) {
        tuh_task();
    }
    return 0;
}