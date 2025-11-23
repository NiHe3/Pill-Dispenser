#include <stdio.h>
#include <sys/unistd.h>

#include "pico/stdlib.h"


const uint piezo_sensor_pin = 27;
const uint led_pin = 20;
// Initialize Pins
void init_pins() {
    //piezo sensor
    gpio_init(piezo_sensor_pin);
    gpio_set_dir(piezo_sensor_pin, GPIO_IN);
    gpio_pull_up(piezo_sensor_pin);

    //led for testing
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_pull_up(led_pin);
}

//Piezo
bool pill_dropped = false;

void piezo_interrupt(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        pill_dropped = true;
    }
}
bool detect_drop() {
    pill_dropped = false;
    sleep_ms(100);
    return pill_dropped;
}

//debug leds
void leds(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        gpio_put(led_pin, 1);
        sleep_ms(on_ms);
        gpio_put(led_pin, 0);
        sleep_ms(off_ms);
    }
}

int main() {
    //call fucntions here
    stdio_init_all();
    init_pins();

    //falling edge interrupt
    gpio_set_irq_enabled_with_callback(
        piezo_sensor_pin,
        GPIO_IRQ_EDGE_FALL,
        true,
        &piezo_interrupt);

    //debugging
    while (true) {
        // if detects drop in sensor will blink
    if (detect_drop()) {
        leds(3, 100, 100);
    }
        //pause
        sleep_ms(200);
    }
}