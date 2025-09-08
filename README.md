# Arduino PID Line Follower Robot

This project contains the complete source code for a high-performance, 8-sensor PID line follower robot based on the Arduino platform. It features automatic sensor calibration, real-time PID tuning via Bluetooth, and robust logic for handling intersections, sharp turns, and line gaps.


## 🤖 Features

* **PID Control**: Utilizes a Proportional-Integral-Derivative (PID) controller for smooth and accurate line tracking.
* **8-Sensor Array**: Uses an 8-channel analog IR sensor array for precise line position detection.
* **Automatic Calibration**: At startup, the robot automatically calibrates its sensors to the line and background, setting optimal thresholds.
* **Automatic Track Detection**: Automatically determines if the track is a black line on a white surface or a white line on a black surface.
* **Bluetooth PID Tuning**: Allows for real-time tuning of Kp, Ki, Kd constants, and base motor speed via a Bluetooth serial terminal, eliminating the need to re-upload code for tuning.
* **Special Case Handling**: Includes logic to manage intersections (all sensors detect line), acute angle turns, and temporary line gaps.
* **Modular Code**: Well-organized code separated into `main`, `motor_control`, and `config` files for easy reading and modification.
* **Configurable**: Key features like braking, gap handling, and debug prints can be easily toggled on or off in the `Config.h` file.

## ⚙️ Hardware Requirements

* **Microcontroller**: Arduino Uno, Nano, or any compatible board.
* **Sensor Array**: 8-Channel Analog IR Sensor Array (e.g., QTR-8A,RLS08).
* **Motor Driver**: Dual H-Bridge Motor Driver (e.g., L298N, TB6612FNG).
* **Motors**: 2 x DC Geared Motors.
* **Chassis**: A robot chassis to mount all components.
* **Bluetooth Module**: HC-05 or HC-06 for wireless tuning.
* **Power Source**: LiPo or Li-Ion batteries (e.g., 7.4V or 11.1V).

## 🔌 Hardware Connections

Connect the components to your Arduino according to the pins defined in the code:

| Component             | Pin Name              | Arduino Pin |
| --------------------- | --------------------- | ----------- |
| **Sensors** | S0 - S7               | A0 - A7     |
| **LED Indicator** | LED                   | 3           |
| **Motor Driver** | STBY (Standby)        | 6           |
|                       | LEFT\_MOTOR\_PIN\_1 (AIN1)  | 8           |
|                       | LEFT\_MOTOR\_PIN\_2 (AIN2)  | 7           |
|                       | LEFT\_MOTOR\_PWM\_PIN (PWMA) | 9 (PWM)     |
|                       | RIGHT\_MOTOR\_PIN\_1 (BIN1) | 4           |
|                       | RIGHT\_MOTOR\_PIN\_2 (BIN2) | 5           |
|                       | RIGHT\_MOTOR\_PWM\_PIN (PWMB) | 11 (PWM)    |
| **Bluetooth Module** | TX                    | RX (Pin 0)  |
|                       | RX                    | TX (Pin 1)  |

**Note**: The Bluetooth module uses the Arduino's hardware serial pins (0, 1). You must disconnect the module while uploading code via USB.

## 🚀 How to Use

### 1. Initial Setup
1.  Assemble the robot chassis and wire all the components as per the table above.
2.  Upload the code to your Arduino board using the Arduino IDE.
3.  Open the Serial Monitor at **115200 baud** to see debug messages and calibration prompts.

### 2. Calibration
The calibration routine runs automatically on every startup. Follow the prompts printed to the Serial Monitor:
1.  **Place on Line**: When prompted, place the robot so that all 8 sensors are centered over the line. The robot will sample the line's sensor readings for 3 seconds.
2.  **Place on Background**: Next, place the robot so that all 8 sensors are on the background surface (off the line). The robot will sample the background readings for 3 seconds.
3.  **Calibration Complete**: The robot will calculate the thresholds, determine the track type, and print the results. It is now ready to run.

### 3. Running the Robot
After calibration, the motor driver is enabled, and the robot will immediately start following the line based on the default PID and speed values in `Config.h`.

### 4. Real-Time Tuning via Bluetooth
You can fine-tune the robot's performance without re-uploading code.
1.  Connect to the robot's Bluetooth module from a serial terminal app on your phone or PC.
2.  Send the following commands followed by a newline character:
    * `p<value>`: Set the Kp (Proportional) value (e.g., `p15`).
    * `i<value>`: Set the Ki (Integral) value (e.g., `i0`).
    * `d<value>`: Set the Kd (Derivative) value (e.g., `d40`).
    * `m<value>`: Set the base motor speed (0-255) (e.g., `m90`).
    * `s`: **Standby**. Stops the robot and disables the motors.
    * `r`: **Run**. Resumes robot operation.

## 🔧 Configuration

All major parameters and features can be adjusted in the `Config.h` file before uploading.

* `DEFAULT_KP`, `DEFAULT_KI`, `DEFAULT_KD`: Initial PID values.
* `DEFAULT_MOTOR_SPEED`: The base speed of the robot.
* `BLUETOOTH_TUNING_ENABLED`: Set to `1` to enable, `0` to disable.
* `DEBUG_ENABLED`: Set to `1` to print error values and other debug info to the serial port.

## 📜 License

This project is open-source. You are free to use, modify, and distribute it. Consider giving credit to the original source if you use it in your own projects.
