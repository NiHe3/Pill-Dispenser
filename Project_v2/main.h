//
// Created by Quinn on 18.11.2025.
//

#ifndef QIN_V2_MAIN_H
#define QIN_V2_MAIN_H

#include "pico/util/queue.h"

#define CLK_DIV 125
#define WRAP_VALUE 999
#define PWM_MAX 1000

#define SW_1 8 // middle button - Power off/on the dispenser, starts the motor, calibration, LED
#define SW_0 9 // left button - dispense pills
#define SW_2 7 // right button - read/erase logs
#define BUTTONS_SIZE 3

#define LED_D1 22 // right LED
#define LED_SIZE 1

// Stepper motor control pins
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define INS_SIZE 4

#define OPTO_PIN 28 // Optical fork input with pull-up
#define PIEZO_PIN 27 // Piezo sensor input with pull-up
#define SAFE_MAX_STEPS 20480 // Safety limit: 5 * 4096 steps
#define STEP_DELAY_MS 2
#define ALIGNMENT_STEPS 100

#define DISPENSE_INTERVAL_MS 30000 // 30 seconds
#define BLINK_INTERVAL_MS 250      // Blink every 250ms
#define TOTAL_DISPENSES 7
#define PIEZO_ADC_THRESHOLD 2000 // adjust empirically (0-4095)
#define MOTOR_ACTIVE_ADDR 32761 // A transient flag (0=Inactive, 1=Active)

// --- Type Definitions (Enums and Structs) ---

// Type of event coming from the interrupt callback
typedef enum {
    EVENT_SW_0,
    EVENT_SW_1,
    EVENT_SW_2,
    EVENT_DISPENSE_STEP
} event_type;

// Generic event passed from ISR to main loop through a queue
typedef struct {
    event_type type; // EVENT_BUTTON 0, 1, or 2
    int32_t data; // 1 = press
} event_t;

// State machine for dispenser logic
typedef enum {
    STATE_WAITING,     // Blinking LED, waiting for calibration
    STATE_CALIBRATING, // (This is a transient state in the main loop)
    STATE_READY,       // Solid LED, waiting to start dispense
    STATE_DISPENSING   // Dispensing every 30 seconds
} st_dispenser;

// Half-step sequence: A, AB, B, BC, C, CD, D, DA
const uint8_t sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// --- Function Prototypes ---

// Hardware and LED Control
void init_buttons(const uint *buttons);
void leds_initialisation(const uint *leds);
void set_brightness(const uint led_pin, uint brightness);
void start_blink(const uint led_pin);
void stop_blink(const uint led_pin);
void blink_n_times(const uint led_pin, int n, int interval_ms);
uint clamp_to_wrap(int bright_value);

// Motor and Sensor Control
void init_coil_pins(const uint *coil_pins); // Initialize motor coil output pins
void init_opto(); // Initialize optical fork input
int do_calibration(const uint *coil_pins, const uint8_t sequence[8][4], int max, int revolution_steps[3]);
void step_motor(const uint *coil_pins, int step, const uint8_t sequence[8][4]);
void run_motor_and_check_pill(const uint *coil_pins, const uint8_t sequence[8][4], int steps_to_move);
void align_motor(const uint *coil_pins, const uint8_t sequence[8][4]);
void init_piezo();
bool detect_drop();

// --- ISR and Callback Prototypes ---
void gpio_callback(uint gpio, uint32_t event_mask);
bool blink_timer_callback(struct repeating_timer *t);
bool dispense_timer_callback(struct repeating_timer *t);

// EEPROM function prototype
void log_current_status();
// lora report
void report_status(const char *event_message);

// reboot software
void software_reboot();

#endif //QIN_V2_MAIN_H
