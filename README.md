# 💊 Pill Dispenser - Embedded Systems Programming Project

This project implements a state-machine-driven pill dispenser system using Raspberry RP2040. The system features motor control, optical/piezoelectric sensing for calibration and dispensing verification, non-volatile storage (EEPROM) for state recovery, and log management.

## 🌟 Features

* **State Machine Architecture:** Uses a robust state machine (`STATE_WAITING`, `STATE_READY`, `STATE_DISPENSING`, `STATE_CALIBRATING`) for clear operational flow.
* **Power-Loss Recovery:** Detects power loss during motor movement via a flag stored in EEPROM (`MOTOR_ACTIVE_ADDR`) and performs an `align_motor()` recovery sequence on reboot.
* **Calibration:** An automatic `do_calibration()` routine uses an optical sensor to measure the precise `steps_per_rev` for the stepper motor, ensuring accurate dispensing.
* **Dispense Cycle:** Dispenses pills automatically at a fixed interval (30 seconds, defined by `DISPENSE_INTERVAL_MS`).
* **Pill Detection:** Uses a Piezo sensor (`PIEZO_PIN`) to verify that a pill actually drops, logging a failure if no drop is detected.
* **Non-Volatile Logging:** Stores operational logs (events, alerts, status) in EEPROM, accessible via a button combination.
* **User Interface:** Simple interface using three buttons (SW_0, SW_1, SW_2) and an LED indicator (`LED_D1`).

## 🛠️ Hardware Requirements

The code relies on the following components and pin definitions:

| Component | Pin Type/Function | Role |
| :--- | :--- | :--- |
| **Microcontroller** | Raspberry RP2040 | Core processing unit. |
| **Stepper Motor** | `28BYJ-48`  Four Coil Pins (IN1-IN4) | Drives the carousel/dispensing mechanism. |
| **Motor Driver** | ULN2003 | Interface between the Microcontroller and the stepper motor. |
| **Optical Sensor** | `OPTO_PIN` | Used during calibration to detect a reference edge. |
| **Piezo Sensor** | `PIEZO_PIN` | Used during dispensing to detect the physical drop of a pill (ISR driven). |
| **LED** | `LED_D1` (PWM Capable) | Status indicator (Blinking: Waiting, Solid: Ready, Off: Dispensing/Busy). |
| **Buttons** | `SW_0`, `SW_1`, `SW_2` | User input for starting, stopping, and accessing logs. |
| **EEPROM** | `AT24C256` I2C Interface: SDA 16, SCL 17 | Non-volatile storage for state, calibration data, and logs. |

## 💻 Software Architecture

The application is built around an **Event-Driven State Machine** running in the `main` loop.

### 1. The Event Queue (`queue_t events`)

* All asynchronous inputs (button presses, timer expirations) push an `event_t` structure onto the global queue.
* The `while(true)` loop processes events from this queue, ensuring inputs are handled sequentially and safely.

### 2. State Definitions

| State | Purpose | LED Status | Trigger to Enter |
| :--- | :--- | :--- | :--- |
| **STATE\_WAITING** | Idle state. Awaiting calibration or user input. | Slow Blink | Power-up (invalid steps), Calibration Fail, Dispensing Stop/Empty. |
| **STATE\_READY** | Calibrated and ready to start dispensing. | Solid ON | Calibration Success, Power-up (valid steps, post-recovery). |
| **STATE\_CALIBRATING** | **Transient State.** Motor is moving and sensor readings are being taken (blocking call). | OFF | `EVENT_SW_1` from `STATE_WAITING`. |
| **STATE\_DISPENSING** | Automatic dispensing cycle is running (30s timer active). | OFF | `EVENT_SW_0` from `STATE_READY`. |

### 3. Key Button Functionality

| Button/Combination | Action | Valid States |
| :--- | :--- | :--- |
| **SW\_1** | **Start Calibration / Stop Dispense Cycle** | `WAITING` (Starts Cal), `DISPENSING` (Stops Cycle) |
| **SW\_0** | **Start Dispense Cycle** | `READY` |
| **SW\_2** | **Read Logs** | All |
| **SW\_2 + SW\_0** (Held) | **Erase Logs** | All |

## ⚙️ Core Function Descriptions

| Function | Role | Notes |
| :--- | :--- | :--- |
| `main()` | Handles initialization, power-loss recovery, and the main state machine loop. | Reads/writes multiple EEPROM addresses on startup/transitions. |
| `do_calibration()` | Spins the motor to find 4 edges via `OPTO_PIN`, calculating the average steps for a full revolution. | **Blocking function**. Returns steps or 0 (failure). |
| `run_motor_and_check_pill()`| Moves the motor by $1/8$th of a revolution (`steps_per_rev / 8`), then calls `detect_drop()`. | Writes `MOTOR_ACTIVE_FLAG` (1/0) to EEPROM for recovery. |
| `align_motor()` | Moves the motor until the `OPTO_PIN` detects a falling edge, aligning the carousel to a known starting position. | Used for power-loss recovery and post-calibration alignment. |
| `gpio_callback()` | The Interrupt Service Routine (ISR) for buttons and the Piezo sensor. | Pushes button events to the queue; sets `pill_dropped` flag for Piezo. |
| `dispense_timer_callback()`| Triggered every 30 seconds to push `EVENT_DISPENSE_STEP` to the queue. | Drives the recurring dispense cycle. |
