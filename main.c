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
#define MIN_STEP_DELAY 300


// define control tick in microseconds and clkdiv as any int to calculate wrap val
//wrap value just needs to be below 65535
#define CONTROL_TICK_US 200
#define CONTROL_PWM_GPIO 15
#define CONTROL_PWM_SLICE pwm_gpio_to_slice_num(CONTROL_PWM_GPIO)
#define CONTROL_CLKDIV 5
#define SYSCLOCK_IN_MHZ 150
#define CONTROL_WRAP 1 + (150*CONTROL_TICK_US)/CONTROL_CLKDIV

typedef struct {
    volatile float x, y, w;
} MotionInput;

typedef struct {
    volatile float current_speed; 
    volatile int current_delay_us, accumulated_us, direction, step_pin_state, default_direction; 
    const char* name;
} WheelState; 


//declare functions


MotionInput currentInput;

WheelState wheelFL, wheelFR, wheelRL, wheelRR;

void init_wheelState() {
    wheelFL = (WheelState){.current_delay_us = 400, .default_direction = 0, .name="Front Left"};
    wheelRL = (WheelState){.current_delay_us = 400, .default_direction = 0, .name="Rear Left"};
    wheelFR = (WheelState){.current_delay_us = 400, .default_direction = 1, .name="Front Right"};
    wheelRR = (WheelState){.current_delay_us = 400, .default_direction = 1, .name="Rear Right"};
}

void printInputState() {
    TU_LOG1("x: %f, y: %f, z: %f\n", currentInput.x, currentInput.y, currentInput.w);
}

void printWheelState(WheelState* w) {
    TU_LOG1("Wheel:%s Speed: %f, Delay: %d, Direction:%d\n", w->name, w->current_speed, w->current_delay_us, w->direction);
}

void printBotState() {
    printWheelState(&wheelFL);
    printWheelState(&wheelRL);
    printWheelState(&wheelRR);
    printWheelState(&wheelFR);
}

//delay = pi*D / V * s where s is steps per rev(WheelState)
void update_wheel_delay(WheelState* w) {
    if(w->current_speed < 0.01) {
        w->current_delay_us = -1.0;
    } else {
        w->current_delay_us = 1e6*(M_PI*WHEEL_DIAMETER)/(w->current_speed * (float)STEPS_PER_REV);
    }
}

void update_all_wheel_delays() {
    update_wheel_delay(&wheelFL);
    update_wheel_delay(&wheelRL);
    update_wheel_delay(&wheelRR);
    update_wheel_delay(&wheelFR);
}

//check current speed, if value is negative then set direction to inverse of default direction
//DIR 0 = counterclockwise (need to verify)
//if absDir == default direction then direction should be 1, otherwise 0
void set_direction(WheelState* w) {
    int absDir = w->current_speed > 0;
    if(!absDir) {
        w->current_speed *= -1;
    }
    w->direction = (absDir == w->default_direction);
}

//calculate control signal strength and scale everything down if it's over the max speed
//also set wheel direction
void update_wheel_speeds() {

    wheelFL.current_speed = currentInput.y - currentInput.x - (WHEEL_DIAMETER*currentInput.w);
    wheelRL.current_speed = currentInput.y + currentInput.x - (WHEEL_DIAMETER*currentInput.w);
    wheelRR.current_speed = currentInput.y - currentInput.x + (WHEEL_DIAMETER*currentInput.w);
    wheelFR.current_speed = currentInput.y + currentInput.x + (WHEEL_DIAMETER*currentInput.w);

    

    // I want all wheel speeds to be positive so I'm going to set the direction here
    set_direction(&wheelFL);
    set_direction(&wheelRL);
    set_direction(&wheelRR);
    set_direction(&wheelFR);

    float wheelSpeeds[4] = {wheelFL.current_speed, wheelRL.current_speed, wheelRR.current_speed, wheelFR.current_speed};
    float scalar = 1.0f;
    float maxWheelSpeed = MAX_WHEEL_SPEED;
    for(int i = 0; i < 4; i++) {
        
        float tempScalar = wheelSpeeds[i] / maxWheelSpeed;
        //TU_LOG1("tempscalar: %f wheelSpeed: %f maxspeed: %f\n", tempScalar, wheelSpeeds[i], MAX_WHEEL_SPEED);
        if(tempScalar > 1.0f && tempScalar > scalar) {
            scalar = tempScalar;
        }
    }
    if(scalar > 1.0f) {
        wheelFL.current_speed /= scalar;
        wheelRL.current_speed /= scalar;
        wheelRR.current_speed /= scalar;
        wheelFR.current_speed /= scalar;
    }

}

void control_tick() {
    //perform motor logic and stepping here

    //update wheel speeds
    update_wheel_speeds();
    update_all_wheel_delays(); //update directions and delays

    // HANDLE ZERO WHEEL SPEEDS (delay is set to -1)

    //accumulate time   
    //check timers and toggle step output pins and set direction pins
    //reset timers if they have been triggered

}

//process pwm isr -> call control handler
void __isr pwm_wrap_isr(void) {
    pwm_clear_irq(CONTROL_PWM_SLICE);
    control_tick();
}

//initialize pwm loop to call the motor handler
bool init_pwm_isr_timer() {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CONTROL_CLKDIV);
    pwm_config_set_wrap(&config, CONTROL_WRAP);
    pwm_init(CONTROL_PWM_SLICE, &config, true);

    pwm_clear_irq(CONTROL_PWM_SLICE);
    pwm_set_irq_enabled(CONTROL_PWM_SLICE, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);
}

//scale the speed command to the maximum possible speed in m/s
float normalize_axis(int16_t value) {
    if ( (-1*DEADZONE < value) && (value < DEADZONE) ) { return 0.0f;}
    return MAX_WHEEL_SPEED * (float)value / AXIS_MAX;
}

void handle_gamepad_input(const xinput_gamepad_t* p) {
    currentInput.x = normalize_axis(p->sThumbLX);
    currentInput.y = normalize_axis(p->sThumbLY);
    currentInput.w = normalize_axis(p->sThumbRX);
    //TU_LOG1("LX: %d, LY: %d, RX: %d, RY: %d\n", p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);
    //TU_LOG1("x: %f, y: %f, z: %f\n", normalize_axis(p->sThumbLX), normalize_axis(p->sThumbLY), normalize_axis(p->sThumbRX));

    printBotState();
    printInputState();
}


usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count){
    *driver_count = 1;
    return &usbh_xinput_driver;
}

void tuh_xinput_report_received_cb(uint8_t dev_addr, uint8_t instance, xinputh_interface_t const* xid_itf, uint16_t len)
{
    const xinput_gamepad_t *p = &xid_itf->pad;
    const char* type_str;

    if (xid_itf->last_xfer_result == XFER_RESULT_SUCCESS)
    {
        switch (xid_itf->type)
        {
            case 1: type_str = "Xbox One";          break;
            case 2: type_str = "Xbox 360 Wireless"; break;
            case 3: type_str = "Xbox 360 Wired";    break;
            case 4: type_str = "Xbox OG";           break;
            default: type_str = "Unknown";
        }

        if (xid_itf->connected && xid_itf->new_pad_data)
        {
            //TU_LOG1("[%02x, %02x], Type: %s, Buttons %04x, LT: %02x RT: %02x, LX: %d, LY: %d, RX: %d, RY: %d\n",
            //    dev_addr, instance, type_str, p->wButtons, p->bLeftTrigger, p->bRightTrigger, p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);

            //How to check specific buttons
            //if (p->wButtons & XINPUT_GAMEPAD_A) TU_LOG1("You are pressing A\n");
            
            //TU_LOG1("LX: %d, LY: %d, RX: %d, RY: %d\n", p->sThumbLX, p->sThumbLY, p->sThumbRX, p->sThumbRY);
            
            
            //TU_LOG1("x=%f, y=%f, z=%f, FL=%f, FR=%f, RL=%f, RR=%f \n", currentInput.x, currentInput.y, currentInput.w, wheelFL.current_speed, wheelFR.current_speed, wheelRL.current_speed, wheelRR.current_speed);
            handle_gamepad_input(p);
        }
    }
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_mount_cb(uint8_t dev_addr, uint8_t instance, const xinputh_interface_t *xinput_itf)
{
    TU_LOG1("XINPUT MOUNTED %02x %d\n", dev_addr, instance);
    // If this is a Xbox 360 Wireless controller we need to wait for a connection packet
    // on the in pipe before setting LEDs etc. So just start getting data until a controller is connected.
    if (xinput_itf->type == XBOX360_WIRELESS && xinput_itf->connected == false)
    {
        tuh_xinput_receive_report(dev_addr, instance);
        return;
    }
    tuh_xinput_set_led(dev_addr, instance, 0, true);
    tuh_xinput_set_led(dev_addr, instance, 1, true);
    tuh_xinput_set_rumble(dev_addr, instance, 0, 0, true);
    tuh_xinput_receive_report(dev_addr, instance);
}

void tuh_xinput_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    TU_LOG1("XINPUT UNMOUNTED %02x %d\n", dev_addr, instance);
}

int main() {
    
    stdio_init_all();
    
    uart_init(uart0, BAUD_RATE);

    tuh_init(BOARD_TUH_RHPORT);

    board_init();
    init_wheelState();
    init_pwm_isr_timer();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(25, true);

    volatile void* dummy_tick_ref = (void*)control_tick;

    TU_LOG1("Max wheel speed %f:\n", MAX_WHEEL_SPEED);

    while(1) {
        tuh_task();
    }
    return 0;
}