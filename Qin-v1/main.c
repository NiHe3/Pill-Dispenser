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
    const uint coil_pins[] = {IN1, IN2, IN3, IN4};
    int steps_per_rev = 4096; // Default steps per revolution before calibration
    int avg = 0;
    int revolution_steps[3] = {0, 0, 0}; // Array to store step counts between four consecutive edges

    // Initialise chosen serial port
    stdio_init_all();
    // Initialise buttons
    init_buttons(buttons);
    // Initialise LED pins
    leds_initialisation(leds);
    //Initialise stepper motor pins
    ini_coil_pins(coil_pins);
    //Initialise optical sensor input
    ini_sensor();

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

void ini_coil_pins(const uint *coil_pins) {
    // Initialize all coil pins as outputs and set them LOW at startup
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_init(coil_pins[i]);
        gpio_set_dir(coil_pins[i], GPIO_OUT);
        gpio_put(coil_pins[i], 0); // Ensure coils are off at startup
    }
}

void ini_sensor() {
    // Initialize the optical sensor input with internal pull-up resistor
    gpio_init(SENSOR_PIN);
    gpio_set_dir(SENSOR_PIN, GPIO_IN);
    // Internal pull-up: SENSOR reads HIGH (1) when not blocked, LOW (0) when blocked
    gpio_pull_up(SENSOR_PIN);
}
int do_calibration(const uint *coil_pins, const uint8_t sequence[8][4], const int max, int revolution_steps[3]) {
    int count = 0; // Number of falling edges detected
    int step = 0; // total half-steps taken
    int edge_step = 0; // Steps between consecutive edges (starts after first edge)
    bool first_edge_found = false;
    bool continue_loop = true;
    bool prev_state = gpio_get(SENSOR_PIN); // true = no obstacle, false = obstacle

    do {
        // Advance the motor by one half-step
        step_motor(coil_pins, 1, sequence);
        sleep_ms(STEP_DELAY_MS); // Time delay between steps (motor speed control)
        step++;

        // Start counting steps between edges after the first edge has been found
        if (first_edge_found)
            edge_step++;

        const bool sensor_state = gpio_get(SENSOR_PIN);

        // Detect falling edge: HIGH -> LOW transition (no obstacle -> obstacle)
        if (prev_state && !sensor_state) {
            if (!first_edge_found) {
                // First falling edge - start counting after this point
                printf("First low edge found. Starting revolution tracking...\r\n");
                first_edge_found = true;
            }
            else {
                // Store number of steps between consecutive edges
                revolution_steps[count-1] = edge_step;
                printf("Revolution %d steps: %d\r\n", count, edge_step);
                edge_step = 0; // Reset counter for the next interval
            }
            count++;
        }
        // Stop after 4 falling edges (3 intervals) or reaching safety limit
        if (count >= 4 || step > max)
            continue_loop = false;

        prev_state = sensor_state;

    } while (continue_loop);

    return 0; // Calibration failed
}

void step_motor(const uint *coil_pins, const int step, const uint8_t sequence[8][4]) {
    // Phase preserves the motor's current position (0-7) across function calls
    static int phase = 0;
    phase = (phase + step) & 7;

    // Set GPIO pins according to the new phase in the half-step sequence
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_put(coil_pins[i], sequence[phase][i]);
    }
}
