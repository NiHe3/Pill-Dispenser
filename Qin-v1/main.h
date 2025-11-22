//
// Created by Quinn on 18.11.2025.
//

#ifndef QIN_V1_MAIN_H
#define QIN_V1_MAIN_H

#define CLK_DIV 125
#define WRAP_VALUE 999
#define PWM_MAX 1000

#define SW_1 8 // middle button - Power off/on
#define SW_0 9 // left button - dispense pills
#define SW_2 7 // right button - starts motor, calib, LED
#define BUTTONS_SIZE 3

#define LED_D1 22 // right LED
#define LED_SIZE 1

// Stepper motor control pins
#define IN1 2
#define IN2 3
#define IN3 6
#define IN4 13
#define INS_SIZE 4

#define SENSOR_PIN 28 // Optical sensor input with pull-up
#define SAFE_MAX_STEPS 20480 // Safety limit: 5 * 4096 steps
#define STEP_DELAY_MS 2
#define LED_DELAY_MS 5 // no longer use, may delete later
#define DISPENSE_INTERVAL_MS 30000 // 30 seconds
#define BLINK_INTERVAL_MS 250 // Blink every 250ms

// Type of event coming from the interrupt callback
typedef enum {
    EVENT_SW_0,
    EVENT_SW_1,
    EVENT_SW_2
} event_type;

// Generic event passed from ISR to main loop through a queue
typedef struct {
    event_type type; // EVENT_BUTTON 0, 1, or 2
    int32_t data; // BUTTON: 1 = press, 0 = release;
} event_t;

// State machine for dispenser logic
typedef enum {
    STATE_WAITING,     // Blinking LED, waiting for calibration
    STATE_CALIBRATING, // Transient state in the main loop
    STATE_READY,       // Waiting to start dispense
    STATE_DISPENSING   // Dispensing every 30 seconds
} dispenser_state_t;

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


void init_buttons(const uint *buttons);
void leds_initialisation(const uint *leds);

void set_brightness(const uint *leds, uint brightness);
void start_blink_led (const uint led_pin);
void stop_blink_led (const uint led_pin);

uint clamp_to_wrap(int bright_value);

void init_coil_pins(const uint *coil_pins); // Initialize motor coil output pins as outputs
void init_sensor(); // Initialize optical sensor input with internal pull-up
int do_calibration(const uint *coil_pins, const uint8_t sequence[8][4], int max, int revolution_steps[3]); // Measure steps per revolution using the optical sensor
void step_motor(const uint *coil_pins, int step, const uint8_t sequence[8][4]); // Perform one half-step of the stepper motor
void run_motor(const uint *coil_pins, const uint8_t sequence[8][4], int count, int steps_to_move); // Run the motor for N * (1/8) revolutions using the calibrated steps per revolution

// --- ISR and Callback Prototypes ---
void gpio_callback(uint gpio, uint32_t events);
bool blink_timer_callback(struct repeating_timer *t);
bool dispense_timer_callback(struct repeating_timer *t);

#endif //QIN_V1_MAIN_H