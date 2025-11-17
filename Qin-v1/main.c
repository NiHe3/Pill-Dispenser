#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main() {

    const uint led_pin = 22;
    uint count = 0;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();

    // Loop forever
    while (true) {

        // Blink LED
        printf("Blinking! %u\r\n", ++count);
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    }
}
