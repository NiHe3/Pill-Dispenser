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
#define LED_DELAY_MS 5

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
void blink_led (uint led_pin, uint button_pin, int delay_ms);
void set_brightness(const uint *leds, uint brightness);
uint clamp_to_wrap(int bright_value);

void ini_coil_pins(const uint *coil_pins); // Initialize motor coil output pins as outputs
void ini_sensor(); // Initialize optical sensor input with internal pull-up
int do_calibration(const uint *coil_pins, const uint8_t sequence[8][4], int max, int revolution_steps[3]); // Measure steps per revolution using the optical sensor
void step_motor(const uint *coil_pins, int step, const uint8_t sequence[8][4]); // Perform one half-step of the stepper motor
int get_avg(const int revolution_steps[3]); // Calculate the average of three revolution step counts
void run_motor(const uint *coil_pins, const uint8_t sequence[8][4], int count, int steps_per_rev); // Run the motor for N * (1/8) revolutions using the calibrated steps per revolution


#endif //QIN_V1_MAIN_H