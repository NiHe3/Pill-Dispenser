#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "main.h"

int main() {
    const uint buttons[] = {SW_1, SW_0, SW_2};
    const uint leds[] = {LED_D1};

    // Initialize chosen serial port
    stdio_init_all();
    // Initialize buttons
    init_buttons(buttons);
    // Initialize LED pins
    leds_initialisation(leds);

    bool led_on = false;
    bool sw1_pre_state = true; // SW1 is pulled up, so "released" = true

    while (true) {
        if (gpio_get(SW_2) == 0) {
            blink_led(LED_D1, SW_2, LED_DELAY_MS);
        }
        sleep_ms(50); // debounce
    }
}

void init_buttons(const uint *buttons) {
    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_init(buttons[i]); // Enable the GPIO pin
        gpio_set_dir(buttons[i], GPIO_IN); // Set the pin as input
        gpio_pull_up(buttons[i]); // High ->1, released, LOW -> 0, pressed
    }
}

void leds_initialisation(const uint *leds) {
    // array to track if a slice has already been initialised
    bool slice_ini[8] = {false};

    // Get default PWM configuration
    pwm_config config = pwm_get_default_config();
    // Set clock divider (125 for 1MHz clock)
    pwm_config_set_clkdiv_int(&config, CLK_DIV);
    // Set wrap for 1KHz frequency
    pwm_config_set_wrap(&config, WRAP_VALUE);

    for (int i = 0; i < LED_SIZE; i++) {
        // Get slice and channel your GPIO pin
        const uint slice = pwm_gpio_to_slice_num(leds[i]);
        const uint chan = pwm_gpio_to_channel(leds[i]);

        // Stop PWM
        pwm_set_enabled(leds[i], false);

        // Initialize each slice once
        if (!slice_ini[slice]) {
            // Start set to true
            pwm_init(slice, &config, true);
            slice_ini[slice] = true;
        }

        // Set level to (CC) -> duty cycle
        pwm_set_chan_level(slice, chan, 0);
        // Select PWM model for your pin
        gpio_set_function(leds[i], GPIO_FUNC_PWM);
        // Start PWM
        pwm_set_enabled(slice, true);
    }
}

// Function to blink LED
void blink_led (uint led_pin, uint button_pin, int delay_ms) {
while (gpio_get(button_pin) == 0){ // sw_2 pressed

        gpio_put(led_pin, 1); // LED on
        sleep_ms(delay_ms);
        gpio_put(led_pin, 0); // LED off
        sleep_ms(delay_ms);
    }
}

// Ensures the brightness value stays within 0 - WRAP_VALUE
uint clamp_to_wrap(const int bright_value) {
    if (bright_value < 0) {
        return 0;
    }

    if (bright_value > WRAP_VALUE) {
        return WRAP_VALUE;
    }
    return bright_value;
}
