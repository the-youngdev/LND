# Simple PID Line Follower Robot

This is a streamlined, efficient PID Line Follower code designed for the Arduino Nano or Uno. It features pure autonomous tracking (no Bluetooth) and is highly configurable.

## ⚠️ Important: Read Before Building!

This code works for both Arduino Nano and Uno, but they require different wiring.

### 1. Arduino Uno Users (Wiring Change)
The Arduino Uno **does not have** pins A6 and A7. Because this code uses 8 sensors, you must move the first two sensors (S0 and S1) to digital pins.

* **Wiring:**
    * **Sensor S0 (Leftmost):** Connect to **Digital Pin 2** (instead of A7).
    * **Sensor S1:** Connect to **Digital Pin 10** (instead of A6).
    * **Sensors S2-S7:** Connect to **A5 - A0** (Same as Nano).
* **Code Update:**
    * Open `src/main.cpp`.
    * Find the **"SENSOR PIN CONFIGURATION"** section.
    * Comment out the Nano lines (`int S0 = A7...`).
    * Uncomment the Uno lines (`int S0 = 2...`).
    * Scroll down to `getSensorReadings()` and switch the read mode from `analogRead` to `digitalRead` as instructed in the comments.

### 2. Motor Driver Configuration
Different drivers work differently.
* **L298N:** Does **not** use a Standby pin.
    * Go to `src/Config.h` and set `#define USE_STANDBY_PIN 0`.
* **TB6612FNG:** **Does** use a Standby pin (Pin 6).
    * Go to `src/Config.h` and set `#define USE_STANDBY_PIN 1`.

## ⚙️ Hardware Setup

| Component | Pin Name | Arduino Nano Pin | **Arduino Uno Pin** |
| :--- | :--- | :--- | :--- |
| **Sensors** | S0 (Leftmost) | A7 (Analog) | **D2** (Digital) |
| | S1 | A6 (Analog) | **D10** (Digital) |
| | S2 - S7 | A5 - A0 | A5 - A0 |
| **Motors** | Left Inputs | 8, 7 | 8, 7 |
| | Left PWM | 9 | 9 |
| | Right Inputs | 4, 5 | 4, 5 |
| | Right PWM | 11 | 11 |
| **Driver** | Standby (STBY) | 6 | 6 (Only if using TB6612FNG) |
| **Indicator** | LED | 3 | 3 |

## 🔧 Tuning (src/Config.h)

You can adjust the robot's behavior in `src/Config.h`.
* `DEFAULT_KP`, `KI`, `KD`: PID constants.
* `DEFAULT_MOTOR_SPEED`: Base speed (0-255).
* `TURN_SPEED_REDUCTION_PERCENT`: Helps prevent flying off the track during sharp turns.

## 🚀 How to Run
1.  Open this folder in VS Code (PlatformIO) or Arduino IDE.
2.  Check `src/Config.h` for your motor driver.
3.  Check `src/main.cpp` for your board type (Nano vs Uno).
4.  Upload the code.
5.  **Calibrate:** Power on the robot and adjust the potentiometers on your sensor array so they correctly detect the line.