# WRO 2025 Future Engineers: Self-Driving Car - Team void_error

![Team Photo](Pictures/team_photo.jpg) _(Link to team photo, add this image to your repo)_

**Team Members:**

-   Arman (Leader)
-   Sajid Hasan Rohit
-   Habibul Bashar Labib
-   Basim Al Zabir Shammo (Coach)

**Repository Last Updated:** 30 August, 2025

## 1. Project Overview

This repository contains the design, code, and documentation for Team void_error's autonomous vehicle developed for the WRO 2025 Future Engineers category. Our vehicle is engineered to navigate the Time Attack challenges (Open and Obstacle) using a combination of sensor data, control algorithms, and strategic decision-making, primarily executed on an Arduino Nano.

The core philosophy is to create a robust, efficient, and well-documented system that adheres to the WRO guiding principles, showcasing our engineering process and problem-solving skills.

## 2. Vehicle Description & Key Features

### 2.1. Mechanical Design

-   **Chassis:** Custom 3D printed frame designed for a 4WD vehicle with front-wheel steering. Models are located in the `RC Chassis/` directory.
-   **Drivetrain:** Rear-wheel drive using DC motors for propulsion.
-   **Steering:** Front-wheel steering actuated by a servo motor.
-   **Dimensions:** Complies with WRO regulations (max 300x200mm, 300mm height).
-   **Materials:** Primarily PLA for 3D printed parts, standard RC components.

### 2.2. Electronic Architecture

Our system employs a distributed control architecture for optimal performance and modularity.

-   **Primary Controller (Brain):** Arduino Nano (`nano_brain/nano_brain.ino`)
    -   **Role:** Central processing unit. Handles real-time sensor fusion (IMU, IR, Ultrasonics), core navigation logic (line following, turn detection, lap counting), actuator control (motors via L298N, servo), and serial communication.
    -   **Key Sensors:**
        -   MPU6050 IMU: Used for precise turn detection (gyro Z-axis integration) and lap counting.
        -   4x IR Sensors: Mounted for line edge detection (Front Left, Front Right, Right 45°, Left 45°).
        -   2x Ultrasonic Sensors: Front and rear distance measurement.
    -   **Actuators:**
        -   2x DC Motors (Rear Drive): Controlled via L298N H-Bridge driver.
        -   1x Servo Motor (Front Steering).
-   **Secondary Controller (Vision/Strategy):** Raspberry Pi Zero 2W (`coding_part/pi_main.py`)
    -   **Role:** Handles high-level processing tasks like camera vision (line following, traffic sign detection - if implemented), complex path planning, and strategy execution. Communicates with the Nano via high-speed serial.
    -   **Key Components:**
        -   Wide-angle USB Camera: Primary sensor for visual processing.
-   **Communication:** High-speed serial communication (500000 baud) between Pi Zero 2W and Arduino Nano for low-latency data exchange (sensor readings, commands).

_(Include a simplified block diagram here if possible, or describe connections based on `pin_diagram/` files)_

### 2.3. Power Management

-   **Main Battery:** [3.7V 18650 Rechargeable Battery Solderable(4pc), 7800 mAh].
-   **Distribution:**
    -   Direct 15V connection to L298N motor driver for high-current motors.
    -   Regulated 5V supply (e.g., via Buck Converter) for Nano, Pi, sensors, and servo logic.

## 3. Software Architecture & Modules

### 3.1. Arduino Nano (`nano_brain/nano_brain.ino`)

-   **Initialization (`setup`):**
    -   Configures pin modes for sensors, motors, servo.
    -   Initializes I2C bus and MPU6050 sensor.
    -   Attaches the steering servo.
    -   Initializes serial communication.
    -   Sends a ready signal ("NANO_READY_MPU_OK" or "\_FAIL").
-   **Main Loop (`loop`):**
    -   **Button Handling:** Checks for the start button press with debouncing.
    -   **Sensor Reading:** Periodically reads data from IR sensors, Ultrasonics, and MPU6050.
    -   **Data Packaging:** Packs sensor data into a structured binary format.
    -   **Serial Transmission:** Sends sensor data packet to the Pi at a fixed interval.
    -   **Serial Command Reception:** Listens for incoming commands from the Pi (motor speed, servo angle, start acknowledge, control mode switch).
    -   **Actuator Control:** Updates motor speeds (via L298N) and servo angle based on the latest commands received.
    -   **Autonomous Logic (Optional Backup):** If Pi control is disabled or fails, can execute basic line-following and turn-counting logic locally.

### 3.2. Raspberry Pi Zero 2W (`coding_part/pi_main.py`)

-   **Initialization:**
    -   Sets up serial communication with the Nano.
    -   Initializes the USB camera using OpenCV (`cv2.VideoCapture`).
    -   Loads configuration parameters (HSV ranges, control gains, etc.).
-   **Main Loop:**
    -   **Serial Listener Thread:** Runs concurrently to continuously read sensor data packets from the Nano and parse them into usable variables.
    -   **Camera Capture & Processing:**
        -   Grabs frames from the camera.
        -   Applies computer vision techniques (color filtering, edge detection, contour analysis) to:
            -   Determine lane position error (for line following).
            -   Detect traffic signs (red/green pillars).
            -   Locate the parking lot entrance.
    -   **Decision Making:**
        -   Integrates sensor data (especially IMU for turn/lap counting).
        -   Implements control algorithms (e.g., PD controller) based on vision data to calculate desired motor speeds and steering angles.
        -   Manages the state machine (Waiting for Start, Driving Laps, Obeying Signs, Searching for Parking, Parking Maneuver, Stopped).
        -   Sends calculated motor/servo commands to the Nano via serial.
    -   **(Optional) UI/Logging:** Displays debug information (camera feed, sensor data) or logs data to a file.

## 4. Building, Flashing, and Running

### 4.1. Prerequisites

-   **Hardware:** Assembled vehicle as per design in `RC Chassis/`. Arduino Nano, Raspberry Pi Zero 2W, sensors, motors, servo, L298N driver, camera, power supply.
-   **Software (Pi):**
    -   Python 3.x
    -   OpenCV (`cv2`)
    -   NumPy (`numpy`)
    -   PySerial (`serial`)
    -   RPi.GPIO (if needed)

### 4.2. Hardware Assembly

1.  Follow the 3D printing instructions for parts in `RC Chassis/`.
2.  Assemble the mechanical components according to the design intent.
3.  Connect electronic components based on the pin definitions in `nano_brain.ino` and schematics in `pin_diagram/`. Ensure correct power connections (battery to motors/L298N, regulated 5V to Nano/Pi/sensors).

### 4.3. Software Installation & Flashing

1.  **Clone this repository:**
    ```bash
    git clone https://github.com/armanislam527/WRO_2025.git
    cd WRO_2025
    ```
2.  **Flash Arduino Nano:**
    -   Open `nano_brain/nano_brain.ino` in the Arduino IDE.
    -   Select the correct Board (e.g., Arduino Nano(Old bootloader)) and Port (e.g., `/dev/ttyUSB0`).
    -   Upload the sketch to the Arduino Nano.
3.  **Setup Raspberry Pi Software:**
    -   Transfer the contents of the `coding_part/` directory to your Raspberry Pi Zero 2W.
    -   Install required Python libraries on the Pi (ensure internet connection or use offline installation packages):
        ```bash
        pip install opencv-python numpy pyserial
        ```
    -   Ensure the Pi's camera is enabled via `raspi-config`.

### 4.4. Running the Vehicle

1.  Ensure the vehicle is assembled and powered correctly.
2.  Connect the Pi Zero 2W to the Nano via the configured serial connection (pins and baud rate must match in both codes).
3.  Power on the vehicle.
4.  On the Pi, navigate to the directory containing the Pi code.
5.  Run the Pi script (ensure serial port is correct):
    ```bash
    python3 pi_main.py
    ```
6.  Press the designated start button on the Nano to initiate the run.

## 5. Repository Structure

-   `nano_brain/`: Contains the main Arduino Nano code (`nano_brain.ino`).
-   `coding_part/`: Contains the Raspberry Pi code (`rpi.py`, `pi_main.py`, etc.) and potentially older/test versions.
-   `RC Chassis/`: 3D model files (`.stl`, `.gcode`) for the custom chassis and components.
-   `pin_diagram/`: Visual and CSV representations of wiring connections.
-   `Pictures/`: Images of the team, vehicle (add team photo!), and during development/testing.
-   `rulebook/`: Official WRO 2025 rule documents.
-   `README.md`: This file.
-   `LICENSE`: Repository license information.

## 6. Development Process & Commits

This repository demonstrates our iterative development approach with regular commits showing progress, problem-solving, and refinements. Key milestones include:

-   Initial hardware setup and basic sensor integration.
-   Development and tuning of the IMU-based turn/lap counting algorithm.
-   Implementation of robust serial communication protocols.
-   Iterative improvements to the 3D printed chassis design.
-   Camera calibration and computer vision algorithm development.
-   Integration testing of Pi-Nano communication and control loops.

_(Note: Ensure your commit history on GitHub reflects this process as per WRO rules)_

## 7. Performance Videos

Links to demonstration videos will be added here. (YouTube links)

## 8. Known Issues & Future Work

-   [List any current limitations or bugs.]
-   [Outline planned improvements or features not yet implemented.]
