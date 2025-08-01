#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "tusb.h"
#include "xinput_host.h"

int main() {
    
    stdio_init_all();
    while(1) {
        printf("Hello World!");
        sleep_ms(250);
    }
    return 0;
}