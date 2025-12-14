#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "hardware/watchdog.h"
#include "main.h"
#include "lora.h"
#include "eeprom.h"

// --- Global Variables ---
static queue_t events; // Global event queue
static st_dispenser current_state = STATE_WAITING;
static struct repeating_timer blink_timer;
static struct repeating_timer dispense_timer;

// Make coils, leds, and steps global so callbacks can access them
static const uint coil_pins[] = {IN1, IN2, IN3, IN4};
static const uint leds[] = {LED_D1};
static int steps_per_rev = 4096; // Default, will be updated by calibration
static int dispensed_count = 0;
static uint8_t log_index = 0; // read from eeprom

// Blink timer guards
static bool dispense_timer_active = false;
volatile bool pill_dropped = false;



// --- Main Application ---
int main() {
    const uint buttons[] = {SW_1, SW_0, SW_2};
    int revolution_steps[3] = {0, 0, 0}; // Array to store step counts

    // Initialise chosen serial port
    stdio_init_all();

    //init_lora();
    //lora_first_at();
    //bool connected = connect_network();
    printf("Dispenser starting...");

   // if (!connected) {
     //   fprintf(stderr, "FATAL ERROR: LoRaWAN connection failed. Continuing without remote reporting.\r\n");
    //}
    // Initialise I2C and read log index
    init_i2c();
    log_index = return_stored_value(LOG_INDEX_ADDR);
    printf("Loaded log index from EEPROM: %d\r\n", log_index);

    current_state = return_stored_value(DEVICE_STATE_ADDR); // read last state
    // steps_per_rev = return_stored_value(CALIBRATION_STEPS_ADDR); // read steps
    // dispensed_count = return_stored_value(DISPENSED_COUNT_ADDR); // read count

    // Read steps_per_rev (2 bytes)
    uint8_t steps_buf[2];
    read_byte(CALIBRATION_STEPS_ADDR, steps_buf, 2);
    steps_per_rev = (int)((steps_buf[0] << 8) | steps_buf[1]);

    // Read dispensed_count (2 bytes)
    uint8_t count_buf[2];
    read_byte(DISPENSED_COUNT_ADDR, count_buf, 2);
    dispensed_count = (count_buf[0] << 8) | count_buf[1];


    // If steps_per_rev is 0 or 255 (erased), use default
    if (steps_per_rev == 0 || steps_per_rev == 0xFF) {
        steps_per_rev = 4096;
        current_state = STATE_WAITING; // Force calibration
    }
    printf("Loaded state: %d, Steps/Rev: %d, Count: %d\r\n", current_state, steps_per_rev, dispensed_count);

    // Initialise event queue
    queue_init(&events, sizeof(event_t), 16);

    // Initialise hardware
    init_buttons(buttons);
    leds_initialisation(leds);
    init_coil_pins(coil_pins);
    init_piezo();
    init_opto();

    // log the power-up event
    write_log_entry("EVENT: Device Power-up/Reboot", &log_index);
    //report_status("Boot: Device Powered Up");

    // Enable button interrupts (Callbacks will push to the queue)
    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_set_irq_enabled_with_callback(buttons[i], GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    }

    // read motor active flag
    uint8_t motor_active_flag = return_stored_value(MOTOR_ACTIVE_ADDR);

    if (motor_active_flag == 1) {
        // flag = 1, log the failure
        write_log_entry("Alert: Power loss during motor movement", &log_index);
        //report_status("ALERT: Power Loss During Move");
        fprintf(stderr,"CRITICAL ERROR: Detected power loss during motor movement. Forcing WAITING state.\r\n");
        current_state = STATE_WAITING;

        // find the optical sensor edge to re-align the motor
        align_motor(coil_pins, sequence);
        // set the state to READY (steps are known)
        if (steps_per_rev > 0 && steps_per_rev != 4096) {
            current_state = STATE_READY;
            printf("Motor successfully realigned. System is READY.\r\n");
            //report_status("EVENT: Motor Aligned, READY");
        } else {
            // If steps_per_rev was invalid/default, we still need calibration
            current_state = STATE_WAITING;
            printf("Realignment complete, but calibration data is missing. Must recalibrate.\r\n");
        }

        // clear the flag
        uint8_t zero = 0;
        write_byte(MOTOR_ACTIVE_ADDR, &zero, 1);

        // Ensure the correct state is saved for the new recovery mode
        write_byte(DEVICE_STATE_ADDR, (uint8_t*)&current_state, 1);
    }

    // adjust state based on loaded state
    if (current_state == STATE_READY || current_state == STATE_DISPENSING) {
        set_brightness(leds[0], PWM_MAX);
        printf("State: READY (Restored). Press SW_0 to start dispensing.\\r\\n");
    } else {
        current_state = STATE_WAITING; // default to waiting
        // Start in the waiting state
        start_blink(leds[0]);
        printf("State: WAITING. Press SW_1 to calibrate.\r\n");
    }

    event_t event;

    while (true) {
        // Process all pending events from the queue
        while (queue_try_remove(&events, &event)) {

            // Only handle "press" events (data = 1)
            // if (event.data != EVENT_DISPENSE_STEP && event.data != 1) continue;

            // SW_2 event for logging, reading and erasing
            if (event.type == EVENT_SW_2 && event.data == 1) {
                // Check if SW_0 is also pressed ->erase logs
                if (gpio_get(SW_0) == false) {
                    printf("SW_2 pressed while SW_0 is held. Initiating manual log erase (Combination Press).\r\n");
                    erase_logs(&log_index);
                }
                // SW_2 pressed alone -> read logs
                else {
                    printf("SW_2 pressed alone. Reading log entries from EEPROM...\r\n");
                    read_log_entry(log_index);
                }
                continue;
            }

            switch (current_state) {

                case STATE_WAITING:
                    if (event.type == EVENT_SW_1) {
                        printf("SW_1 pressed. Starting calibration...\r\n");
                        current_state = STATE_CALIBRATING; // Set transient state
                        stop_blink(leds[0]);

                        // Run calibration (this is a blocking function)
                        int avg_steps = do_calibration(coil_pins, sequence, SAFE_MAX_STEPS, revolution_steps);

                        if (avg_steps > 0) {
                            steps_per_rev = avg_steps;
                            // save steps_per_rev
                            write_byte(CALIBRATION_STEPS_ADDR, (uint8_t*)&steps_per_rev, 1);
                            printf("Calibration complete. Steps/rev: %d\r\n", steps_per_rev);
                            align_motor(coil_pins, sequence);
                            set_brightness(leds[0], PWM_MAX); // Solid LED on

                            current_state = STATE_READY;
                            write_byte(DEVICE_STATE_ADDR, (uint8_t*)&current_state, 1);
                            // log success
                            write_log_entry("EVENT: Calibration SUCCESS", &log_index);
                            //report_status("EVENT: Calibration SUCCESS");
                            printf("State: READY. Press SW_0 to start dispensing.\r\n");
                        } else {
                            fprintf(stderr, "Calibration failed. Returning to wait state.\r\n");
                            // log failure
                            write_log_entry("ALERT: Calibration FAILED", &log_index);
                            //report_status("ALERT: Calibration FAILED");
                            start_blink(leds[0]); // Restart blink
                            current_state = STATE_WAITING;
                        }
                    }
                    break;

                case STATE_READY:
                    if (event.type == EVENT_SW_0) {
                        printf("SW_0 pressed. Starting dispense cycle.\r\n");
                        current_state = STATE_DISPENSING;
                        write_byte(DEVICE_STATE_ADDR, (uint8_t*)&current_state, 1);

                        // 1. Dispense immediately on button press
                        dispensed_count = 0;
                        write_byte(DISPENSED_COUNT_ADDR, (uint8_t*)&dispensed_count, 1);
                        set_brightness(leds[0], 0); // LED off
                        printf("Dispensing (initial)...\r\n");
                        run_motor_and_check_pill(coil_pins, sequence, steps_per_rev / 8); // 8 compartments

                        // 2. Start 30-second timer for future dispenses
                        if (add_repeating_timer_ms(DISPENSE_INTERVAL_MS, dispense_timer_callback, NULL, &dispense_timer)) {
                            dispense_timer_active = true;
                        } else {
                            fprintf(stderr, "ERROR: failed to start dispense timer.\r\n");
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
                            set_brightness(leds[0], 0); // LED off
                        }

                        current_state = STATE_WAITING;
                        // save current state
                        write_byte(DEVICE_STATE_ADDR, (uint8_t*)&current_state, 1);
                        start_blink(leds[0]); // Start waiting blink again
                        dispensed_count = 0;
                        // run_motor_and_check_pill(coil_pins, sequence, steps_per_rev / 8);
                        // save dispensed_count
                        write_byte(DISPENSED_COUNT_ADDR, (uint8_t*)&dispensed_count, 1);
                        printf("State: WAITING. Press SW_1 to calibrate.\r\n");
                    } else if (event.type == EVENT_DISPENSE_STEP) {
                        printf("Dispense timer event.\r\n");
                        run_motor_and_check_pill(coil_pins, sequence, steps_per_rev / 8);

                        if (dispensed_count >= 7) {
                            printf("All pills dispensed. Stopping cycle.\r\n");
                            // report_status("EVENT: Dispenser Empty");
                            if (dispense_timer_active) {
                                cancel_repeating_timer(&dispense_timer);
                                dispense_timer_active = false;
                            }
                            current_state = STATE_WAITING;
                            write_byte(DEVICE_STATE_ADDR, (uint8_t*)&current_state, 1);
                            set_brightness(leds[0], 0);
                            start_blink(leds[0]);
                            dispensed_count = 0;
                            write_byte(DISPENSED_COUNT_ADDR, (uint8_t*)&dispensed_count, 1);
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
// main ends here


// GPIO Interrupt Service Routine
void gpio_callback(uint gpio, uint32_t event_mask) {

    //piezo
    if (gpio == PIEZO_PIN) {
            pill_dropped = true;
        return;
    }
    if (gpio == SW_2) {
        if (gpio_get(SW_1) == false) {
            watchdog_reboot(0,0,100);
        }
    }
    // Just create an event and push it to the queue.
    // Buttons
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

    // Try to add to queue. If it fails (full), lose the event.
    queue_try_add_irq(&events, &ev);
}

// Timer callback for the "waiting" blink
bool blink_timer_callback(struct repeating_timer *t) {
    static bool led_state = false;
    led_state = !led_state;
    set_brightness(leds[0], led_state ? PWM_MAX : 0);
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

        pwm_set_enabled(slice, false);

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


// Ensures the brightness value stays within 0 - PWM_MAX
uint clamp_to_wrap(const int bright_value) {
    if (bright_value < 0) {
        return 0;
    }
    if (bright_value > PWM_MAX) {
        return PWM_MAX;
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

void init_piezo() {
    gpio_init(PIEZO_PIN);
    gpio_set_dir(PIEZO_PIN, GPIO_IN);
    gpio_pull_up(PIEZO_PIN);
    gpio_set_irq_enabled_with_callback(PIEZO_PIN, GPIO_IRQ_EDGE_FALL,true,&gpio_callback);
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

    uint8_t motor_active_flag = 1;
    write_byte(MOTOR_ACTIVE_ADDR, &motor_active_flag, 1);
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
            fprintf(stderr, "ERROR: Max steps reached during calibration.\r\n");
            continue_loop = false;
        }

        prev_state = sensor_state;

    } while (continue_loop);

    motor_active_flag = 0;
    write_byte(MOTOR_ACTIVE_ADDR, &motor_active_flag, 1);

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

void align_motor(const uint *coil_pins, const uint8_t sequence[8][4]) {
    bool prev_state = gpio_get(OPTO_PIN);
    int safe_counter = steps_per_rev + 50;

    while (safe_counter --> 0) {
        step_motor(coil_pins, 1, sequence); // step forward
        sleep_ms(STEP_DELAY_MS);
        const bool sensor_state = gpio_get(OPTO_PIN);

        if (prev_state && !sensor_state) {
            break; // stop motor when edge is found
        }
        prev_state = sensor_state;
    }
    if (safe_counter <= 0) {
        return;
    }
    int direction = (ALIGNMENT_STEPS >= 0) ? 1 : -1;
    int base_steps = (ALIGNMENT_STEPS >= 0) ? ALIGNMENT_STEPS : -ALIGNMENT_STEPS;
    for (int i = 0; i < base_steps; i++) {
        step_motor(coil_pins, direction, sequence);
        sleep_ms(STEP_DELAY_MS);
    }
}

// Runs the motor for a specific number of steps
void run_motor_and_check_pill(const uint *coil_pins, const uint8_t sequence[8][4], int steps_to_move) {
    int direction = (steps_to_move >= 0) ? 1 : -1;
    int base_steps = (steps_to_move >= 0) ? steps_to_move : -steps_to_move;

    // set flag: motor is starting
    uint8_t motor_active_flag = 1;
    write_byte(MOTOR_ACTIVE_ADDR, &motor_active_flag, 1);
    printf("Moving motor %d steps...\r\n", steps_to_move);

    for (int i = 0; i < base_steps; i++) {
        step_motor(coil_pins, direction, sequence);
        sleep_ms(STEP_DELAY_MS);
    }
    printf("Motor move complete.\r\n");

    // clear flag: motor has finished
    motor_active_flag = 0;
    write_byte(MOTOR_ACTIVE_ADDR, &motor_active_flag, 1);

    // Pill detection
    bool pill_detected = detect_drop();
    if(!pill_detected) {
        fprintf(stderr, "No pill detected! Blinking LED 5 times.\r\n");
        blink_n_times(leds[0], 5, 150); // 5 blinks
        // log the error
        write_log_entry("ALERT: Dispense FAILED (No Pill Detected)", &log_index);
        //report_status("ALERT: Dispense Failed");
    } else {
        printf("Pill detected.\r\n");
        // log success
        write_log_entry("EVENT: Pill Dispensed SUCCESSFULLY", &log_index);
        //report_status("EVENT: Pill Dispensed");
    }
    dispensed_count++;
    write_byte(DISPENSED_COUNT_ADDR, (uint8_t*)&dispensed_count, 1);
}

bool detect_drop() {
    // Pills take from global, and resets it after each usage case
    bool detected = pill_dropped;
    sleep_ms(20);
    pill_dropped = false;
    return detected;
}

// Blink n times
void blink_n_times(const uint led_pin, int n, int interval_ms) {
    for (int i = 0; i < n; i++) {
        set_brightness(led_pin, PWM_MAX);
        sleep_ms(interval_ms);
        set_brightness(led_pin, 0);
        sleep_ms(interval_ms);
    }
}

// Generate and write a status log entry
void log_current_status() {
    char status_str[STRLEN_EEPROM];
    const char *state_name;

    switch (current_state) {
        case STATE_WAITING: state_name = "WAITING"; break;
        case STATE_CALIBRATING: state_name = "CALIBRATING"; break;
        case STATE_READY: state_name = "READY"; break;
        case STATE_DISPENSING: state_name = "DISPENSING"; break;
            default: state_name = "UNKNOWN"; break;
    }

    // log message format: "STATUS: READY, D:0, P:F"
    // D: dispensed_count, P: pill_dropped status (T/F)
    snprintf(status_str, STRLEN_EEPROM, "STATUS: %s, D: %d, P:%c",
        state_name, dispensed_count, pill_dropped ? 'T' : 'F');
    printf("Saving log entry: %s\r\n", status_str);
    write_log_entry(status_str, &log_index);
}

void software_reboot() {
    printf("Initiating software reboot...\r\n");
    watchdog_reboot(0,0,100);
    while (true) {
        tight_loop_contents();
    }
}

void report_status(const char *event_message) {
    printf("LoRa report: %s\n", event_message);
    send_msg(event_message);
}


