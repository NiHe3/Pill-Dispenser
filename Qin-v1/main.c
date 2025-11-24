#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "main.h"

#include "pico/util/queue.h"

// --- Global Variables ---
static queue_t events; // Global event queue
static st_dispenser current_state = STATE_WAITING;
static struct repeating_timer blink_timer;
static struct repeating_timer dispense_timer;

// Make coils, leds, and steps global so callbacks can access them
static const uint g_coil_pins[] = {IN1, IN2, IN3, IN4};
static const uint g_leds[] = {LED_D1};
static int g_steps_per_rev = 4096; // Default, will be updated by calibration
static int dispensed_count = 0;

// Blink timer guards
static bool waiting_blink_running = false;
static bool dispense_timer_active = false;
static void enqueue_event(event_type t);

// --- Main Application ---
int main() {
    const uint buttons[] = {SW_1, SW_0, SW_2};
    int revolution_steps[3] = {0, 0, 0}; // Array to store step counts

    // Initialise chosen serial port
    stdio_init_all();
    printf("Dispenser starting...\r\n");

    // Initialise event queue
    queue_init(&events, sizeof(event_t), 16);

    // Initialise hardware
    init_buttons(buttons);
    leds_initialisation(g_leds);
    init_coil_pins(g_coil_pins);
    init_opto();

    // Enable button interrupts (Callbacks will push to the queue)
    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_set_irq_enabled_with_callback(buttons[i], GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    }

    // Start in the waiting state
    start_blink(g_leds[0]);
    printf("State: WAITING. Press SW_1 to calibrate.\r\n");

    event_t event;

    while (true) {
        // Process all pending events from the queue
        while (queue_try_remove(&events, &event)) {

            // Only handle "press" events (data = 1)
            if (event.data != EVENT_DISPENSE_STEP && event.data != 1) continue;

            switch (current_state) {

                case STATE_WAITING:
                    if (event.type == EVENT_SW_1) {
                        printf("SW_1 pressed. Starting calibration...\r\n");
                        current_state = STATE_CALIBRATING; // Set transient state
                        stop_blink(g_leds[0]);

                        // Run calibration (this is a blocking function)
                        int avg_steps = do_calibration(g_coil_pins, sequence, SAFE_MAX_STEPS, revolution_steps);

                        if (avg_steps > 0) {
                            g_steps_per_rev = avg_steps;
                            printf("Calibration complete. Steps/rev: %d\r\n", g_steps_per_rev);
                            set_brightness(g_leds[0], WRAP_VALUE); // Solid LED on
                            current_state = STATE_READY;
                            printf("State: READY. Press SW_0 to start dispensing.\r\n");
                        } else {
                            printf("Calibration failed. Returning to wait state.\r\n");
                            start_blink(g_leds[0]); // Restart blink
                            current_state = STATE_WAITING;
                        }
                    }
                    break;

                case STATE_READY:
                    if (event.type == EVENT_SW_0) {
                        printf("SW_0 pressed. Starting dispense cycle.\r\n");
                        current_state = STATE_DISPENSING;

                        // 1. Dispense immediately on button press
                        dispensed_count = 0;
                        set_brightness(g_leds[0], 0); // LED off
                        printf("Dispensing (initial)...\r\n");
                        run_motor_and_check_pill(g_coil_pins, sequence, g_steps_per_rev / 7); // 7 compartments

                        // 2. Start 30-second timer for future dispenses
                        if (add_repeating_timer_ms(DISPENSE_INTERVAL_MS, dispense_timer_callback, NULL, &dispense_timer)) {
                            dispense_timer_active = true;
                        } else {
                            printf("ERROR: failed to start dispense timer.\r\n");
                            current_state =STATE_READY;
                        }
                        printf("State: DISPENSING. Press SW_1 to stop.\r\n");
                    }
                    break;

                case STATE_DISPENSING:
                    if (event.type == EVENT_SW_1) {
                        printf("SW_1 pressed. Stopping dispense cycle.\r\n");

                        // ensure no undefined behaviour if SW_1 is pressed before SW_0
                        if (dispense_timer_active) {
                            cancel_repeating_timer(&dispense_timer);
                            dispense_timer_active = false;
                            set_brightness(g_leds[0], 0); // LED off
                        }

                        current_state = STATE_WAITING;
                        start_blink(g_leds[0]); // Start waiting blink again
                        dispensed_count = 0;
                        printf("State: WAITING. Press SW_1 to calibrate.\r\n");
                    } else if (event.type == EVENT_DISPENSE_STEP) {
                        printf("Dispense timer event.\r\n");
                        run_motor_and_check_pill(g_coil_pins, sequence, g_steps_per_rev / 7);

                        if (dispensed_count >= 7) {
                            printf("All pills dispensed. Stopping cycle.\r\n");
                            if (dispense_timer_active) {
                                cancel_repeating_timer(&dispense_timer);
                                dispense_timer_active = false;
                            }
                            current_state = STATE_WAITING;
                            set_brightness(g_leds[0], 0);
                            start_blink(g_leds[0]);
                            dispensed_count = 0;
                        }
                    }
                    break;

                case STATE_CALIBRATING:
                    // Do nothing, wait for calibration to finish
                    break;
            }
        }
        // Allow the processor to sleep when no events are pending
        sleep_ms(10);
    }
}


// GPIO Interrupt Service Routine
void gpio_callback(uint gpio, uint32_t event_mask) {
    // Just create an event and push it to the queue.
    event_t ev;

    if (gpio == SW_0) {
        ev.type = EVENT_SW_0;
    }else if (gpio == SW_1) {
        ev.type = EVENT_SW_1;
    }else if (gpio == SW_2) {
        ev.type = EVENT_SW_2;
    }else {
        return;
    }
    ev.data = 1; // 1 = pressed (we only care about falling edge)

    // Try to add to queue. If it fails (full), we lose the event.
    queue_try_add(&events, &ev);
}

// Timer callback for the "waiting" blink
bool blink_timer_callback(struct repeating_timer *t) {
    static bool led_state = false;
    led_state = !led_state;
    set_brightness(g_leds[0], led_state ? WRAP_VALUE : 0);
    return true; // Keep repeating
}

// Timer callback for dispensing
bool dispense_timer_callback(struct repeating_timer *t) {
    event_t ev = { .type = EVENT_DISPENSE_STEP, .data = 1};
    queue_try_add(&events, &ev);
    return true; // Keep repeating
}


// --- Hardware Initialization ---

void init_buttons(const uint *buttons) {
    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_init(buttons[i]);
        gpio_set_dir(buttons[i], GPIO_IN);
        gpio_pull_up(buttons[i]);
    }
}

void leds_initialisation(const uint *leds) {
    bool slice_ini[8] = {false};
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, CLK_DIV);
    pwm_config_set_wrap(&config, WRAP_VALUE);

    for (int i = 0; i < LED_SIZE; i++) {
        const uint slice = pwm_gpio_to_slice_num(leds[i]);
        const uint chan = pwm_gpio_to_channel(leds[i]);

        pwm_set_enabled(slice, false); // Use slice, not pin

        if (!slice_ini[slice]) {
            pwm_init(slice, &config, true);
            slice_ini[slice] = true;
        }

        pwm_set_chan_level(slice, chan, 0);
        gpio_set_function(leds[i], GPIO_FUNC_PWM);
        pwm_set_enabled(slice, true);
    }
}

// --- LED Control Functions (PWM) ---

// Sets LED brightness using PWM
void set_brightness(const uint led_pin, uint brightness) {
    const uint slice = pwm_gpio_to_slice_num(led_pin);
    const uint chan = pwm_gpio_to_channel(led_pin);
    pwm_set_chan_level(slice, chan, clamp_to_wrap(brightness));
}

// Starts the non-blocking waiting blink
void start_blink(const uint led_pin) {
    // Ensure LED is off to start
    set_brightness(led_pin, 0);
    add_repeating_timer_ms(BLINK_INTERVAL_MS, blink_timer_callback, NULL, &blink_timer);
}

// Stops the non-blocking waiting blink
void stop_blink(const uint led_pin) {
    cancel_repeating_timer(&blink_timer);
    // Ensure LED is left in the OFF state
    set_brightness(led_pin, 0);
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

// --- Motor and Sensor Functions ---

void init_coil_pins(const uint *coil_pins) {
    for (int i = 0; i < INS_SIZE; i++) {
        gpio_init(coil_pins[i]);
        gpio_set_dir(coil_pins[i], GPIO_OUT);
        gpio_put(coil_pins[i], 0);
    }
}

void init_opto() {
    gpio_init(OPTO_PIN);
    gpio_set_dir(OPTO_PIN, GPIO_IN);
    gpio_pull_up(OPTO_PIN);
}

// Calibrates and returns average steps per revolution
int do_calibration(const uint *coil_pins, const uint8_t sequence[8][4], const int max, int revolution_steps[3]) {
    int count = 0;
    int step = 0;
    int edge_step = 0;
    bool first_edge_found = false;
    bool continue_loop = true;
    bool prev_state = gpio_get(OPTO_PIN);

    printf("Calibration running... turning motor to find 4 edges.\r\n");

    do {
        step_motor(coil_pins, 1, sequence); // Always step forward
        sleep_ms(STEP_DELAY_MS);
        step++;

        if (first_edge_found) {
            edge_step++;
        }

        const bool sensor_state = gpio_get(OPTO_PIN);

        // Detect falling edge: HIGH -> LOW transition
        if (prev_state && !sensor_state) {
            count++; // Increment edge count
            if (!first_edge_found) {
                printf("First edge found. Starting revolution tracking...\r\n");
                first_edge_found = true;
            } else {
                revolution_steps[count - 2] = edge_step; // Store steps for the interval
                printf("Edge %d found. Steps since last edge: %d\r\n", count, edge_step);
                edge_step = 0; // Reset counter for the next interval
            }
        }

        // Stop after 4 falling edges (which gives 3 intervals)
        if (count >= 4) {
            continue_loop = false;
            printf("Fourth edge found. Stopping motor aligned.\r\n");
        }

        // Safety break
        if (step > max) {
            printf("ERROR: Max steps reached during calibration.\r\n");
            continue_loop = false;
        }

        prev_state = sensor_state;

    } while (continue_loop);

    // Check if calibration was successful
    if (count >= 4) {
        // Calculate average of the 3 intervals
        int total = revolution_steps[0] + revolution_steps[1] + revolution_steps[2];
        return total / 3;
    }

    return 0; // Calibration failed
}

// Performs one half-step of the stepper motor
void step_motor(const uint *coil_pins, const int step, const uint8_t sequence[8][4]) {
    static int phase = 0;
    phase = (phase + step) & 7; // & 7 is a fast way to do (phase % 8)

    for (int i = 0; i < INS_SIZE; i++) {
        gpio_put(coil_pins[i], sequence[phase][i]);
    }
}

// Runs the motor for a specific number of steps
void run_motor_and_check_pill(const uint *coil_pins, const uint8_t sequence[8][4], int steps_to_move) {
    int direction = (steps_to_move > 0) ? 1 : -1;
    int steps_abs = (steps_to_move > 0) ? steps_to_move : -steps_to_move;

    printf("Moving motor %d steps...\r\n", steps_to_move);

    for (int i = 0; i < steps_abs; i++) {
        step_motor(coil_pins, direction, sequence);
        sleep_ms(STEP_DELAY_MS);
    }
    printf("Motor move complete.\r\n");
}

// Add piezo here
//     bool pill_deteced = read_piezo_detected(){
//     if (!pill_deteced) {
//         printf("No pill detected! Blinking LED 5 times.\r\n");
//         blink_n_times(g_leds[0], 5, 150); // 5 blinks
//     } else {
//         printf("Pill detected.\r\n")
//     }
//     dispensed_count++;
// }

// Blink n times
void blink_n_times(const uint led_pin, int n, int interval_ms) {
    for (int i = 0; i < n; i++) {
        set_brightness(led_pin, WRAP_VALUE);
        sleep_ms(interval_ms);
        set_brightness(led_pin, 0);
        sleep_ms(interval_ms);
    }
}