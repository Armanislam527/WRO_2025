# WRO 2025 Future Engineers: Self-Driving Car - Pi-Side Controller Software

This document provides a detailed description of the C++ software architecture and implementation for the Raspberry Pi Zero 2W component of our WRO 2025 Future Engineers Self-Driving Car entry, Team void_error. This software acts as the high-level intelligence unit, processing sensor data, making strategic decisions, and controlling the vehicle's actions.

## 1. Introduction & Philosophy

The Pi-side software is designed around a modular, event-driven architecture to handle the complex tasks of perception, navigation, and control required for the WRO 2025 challenge. It communicates with the Arduino Nano (handling real-time I/O and motor control) via a high-speed serial link. The code is structured to be maintainable, efficient, and to clearly separate concerns (e.g., vision, communication, navigation, control).

Our development process followed an iterative cycle: `Design -> Implement -> Test -> Analyze -> Refine`. This is reflected in the commit history and the evolution of the modules.

## 2. Core Components & Modules

The software is organized into several directories within the `src/` folder, each encapsulating a specific aspect of the vehicle's functionality.

### 2.1. Main Application (`src/main.cpp`)

This is the entry point of the application. It orchestrates the initialization of all subsystems, manages the main execution loop, and handles the overall mission state machine. It initializes and connects the `SerialHandler`, `CameraInterface`, `ImageProcessor`, `LapCounter`, `PathPlanner`, and `VehicleController`. The main loop continuously retrieves sensor and vision data, updates the `MissionState`, and dispatches control flow to specific handlers based on the current phase (e.g., `PRE_START`, `DRIVING_LAPS`, `STOPPING_AT_START`, `PARKING`).

### 2.2. Configuration (`src/config/`)

-   **`vehicle_config.h` / `vehicle_config.cpp`**: Centralizes configurable parameters such as serial port names, baud rates, camera settings, vehicle physical dimensions, and thresholds used across different modules. This makes tuning and adapting the software easier without modifying core logic.

### 2.3. Communication (`src/communication/`)

-   **`serial_handler.h` / `serial_handler.cpp`**: Manages the serial communication link with the Arduino Nano. It handles opening/closing the serial port, running a dedicated background thread for receiving data packets, parsing the binary protocol (start byte, command, length, data, checksum), and providing thread-safe access to the latest `SensorData`. It also includes methods to send commands to the Nano (motor speed, servo angle, heartbeat, GO signal, emergency stop).
-   **`serial_protocol.h` / `serial_protocol.cpp`**: Defines the constants for command IDs (e.g., `CMD_SET_MOTOR_SPEED`, `CMD_ALL_SENSOR_DATA_COMPACT`) and packet structure. It provides utility functions for calculating checksums (`calculateChecksum`) and creating packets (`createPacket`), ensuring consistent communication protocol implementation.

### 2.4. Sensors (`src/sensors/`)

-   **`sensor_data.h`**: Defines the `SensorData` struct, which mirrors the `CompactSensorData` struct on the Nano. This struct efficiently packs ultrasonic distance readings (front, right, back, left) and scaled IMU data (accelerometer X/Y/Z, gyroscope X/Y/Z) into a 20-byte binary format for fast transmission.
-   **`sensor_fusion.h` / `sensor_fusion.cpp`** _(Currently Placeholder)_: Intended for combining data from multiple sensors (IMU, ultrasonics) to produce a more robust estimate of the vehicle's state (position, orientation, velocity). This is crucial for accurate navigation and lap counting.

### 2.5. Vision (`src/vision/`)

This subsystem handles image capture and processing for detecting track features like traffic signs and edges.

-   **`frame_buffer.h` / `frame_buffer.cpp`**: A thread-safe buffer designed for efficient streaming of image frames from the capture thread to the processing thread. It holds only the _latest_ frame, minimizing latency and memory usage.
-   **`camera_interface.h` / `camera_interface.cpp`**: Manages the connection to the Pi Camera Module via V4L2 (using OpenCV). It initializes the camera, sets resolution and frame rate, and runs a background thread (`captureLoop`) to continuously capture frames and place them into the `FrameBuffer`.
-   **`image_processor.h` / `image_processor.cpp`**: The core of the vision system. It retrieves frames from the `FrameBuffer`, processes them (converts to HSV, applies ROI), and performs basic color blob detection for Red/Green signs and simplified edge detection (using thresholding and column sum analysis). It outputs a `VisionSnapshot` struct containing the results (signs detected, approximate edge positions).
-   **`color_detector.h` / `color_detector.cpp`**, **`edge_detector.h` / `edge_detector.cpp`**, **`roi_selector.h` / `roi_selector.h`**, **`line_follower.h` / `line_follower.cpp`** _(Currently Empty Placeholders)_: These files are intended to decompose the monolithic `image_processor.cpp` logic into more specialized modules for better modularity and clarity in future development.

### 2.6. Navigation (`src/navigation/`)

This layer is responsible for understanding the vehicle's position and progress within the challenge.

-   **`mission_state.h` / `mission_state.cpp`**: Encapsulates the overall state of the mission, including the challenge type (Open/Obstacle), the current phase (Pre-Start, Driving, Parking, Stopping), the start position (Regular/Parking Lot), and the current lap number. This state is used by other modules to make decisions.
-   **`positional_memory.h` / `positional_memory.cpp`**: Stores sensor and vision snapshots of key positions (initial start, pre-stop position, parking entrance). It provides methods to compare the current state with these stored states using simple similarity metrics (based on US distances and vision features). This is critical for tasks like stopping the vehicle in the exact starting position.
-   **`lap_counter.h` / `lap_counter.cpp`**: Tracks the number of laps completed. It uses sensor data (primarily IMU for rotation and US for distance estimation) and potentially vision data to detect transitions between track sections and determine when a full lap (8 sections) has been completed, ensuring the vehicle has left and returned to the start zone.
-   **`course_map.h` / `course_map.cpp`**, **`sign_detector.h` / `sign_detector.cpp`** _(Currently Empty Placeholders)_: `course_map` would build a representation of the known/observed track layout. `sign_detector` would be a dedicated module for interpreting vision data to robustly identify and classify traffic signs, separating this logic from `image_processor`.

### 2.7. Control (`src/control/`)

This layer translates navigation decisions into actionable commands.

-   **`path_planner.h` / `path_planner.cpp`**: The high-level decision-making module. Based on the `MissionState`, `LapCounter` data, and `VisionSnapshot`, it determines the vehicle's next action (e.g., `FOLLOW_LEFT_LANE`, `FOLLOW_RIGHT_LANE`, `GO_STRAIGHT`, `STOP_IN_START`, `EXECUTE_PARKING`). It implements basic obstacle avoidance logic.
-   **`vehicle_controller.h` / `vehicle_controller.cpp`**: The actuator interface. It receives `NavigationCommand`s from the `PathPlanner` and translates them into specific motor speed (int8_t) and servo angle (uint8_t) commands, sending them via the `SerialHandler`. It also manages complex maneuvers like the parallel parking sequence, taking over control until the maneuver is complete.
-   **`parking_assistant.h` / `parking_assistant.cpp`**, **`trajectory_generator.h` / `trajectory_generator.cpp`** _(Currently Empty Placeholders)_: `parking_assistant` would contain the detailed logic for the parking maneuver, potentially separating it from `vehicle_controller`. `trajectory_generator` would be an optional intermediate step to calculate smooth speed/steering profiles from path plans.

### 2.8. Actuators (`src/actuators/`)

-   **`vehicle_controller.h` / `vehicle_controller.cpp`** _(See Control section above)_: This is the primary actuator control module.

### 2.9. Utilities (`src/utils/`)

-   **`logger.h`**, **`timer.h` / `timer.cpp`**, **`math_utils.h`** _(Currently Empty Placeholders)_: Utility modules for logging debug information, measuring execution time, and providing common mathematical functions. Essential for development, debugging, and performance optimization.

## 3. Data Flow & Interaction

1.  **Initialization:** `main.cpp` initializes all modules and establishes dependencies (e.g., passes `SerialHandler` to `VehicleController`).
2.  **Background Tasks:**
    -   `CameraInterface` runs `captureLoop` to feed frames to `FrameBuffer`.
    -   `ImageProcessor` runs `processingLoop` to consume frames from `FrameBuffer` and produce `VisionSnapshot`.
    -   `SerialHandler` runs `receiveLoop` to consume bytes from the serial port, parse packets, and update its internal `SensorData` buffer.
3.  **Main Loop (`main.cpp`):**
    -   Retrieves latest `SensorData` from `SerialHandler`.
    -   Retrieves latest `VisionSnapshot` from `ImageProcessor`.
    -   Updates `LapCounter` with new data.
    -   Updates `MissionState` based on events (e.g., receiving `START_ACK` from Nano).
    -   Calls the appropriate phase handler (`handleDriving`, `handleStoppingAtStart`, etc.) based on `MissionState`.
    -   The phase handler (e.g., `handleDriving`) calls `PathPlanner::plan()` with current data and state.
    -   `PathPlanner` returns a `NavigationCommand`.
    -   The phase handler passes this command to `VehicleController::executeCommand()`.
    -   `VehicleController` sends the corresponding low-level commands to the Nano via `SerialHandler`.
    -   Sends periodic heartbeat to Nano.

## 4. Building the Software

This project uses CMake for building.

### Prerequisites

-   **Raspberry Pi OS (or compatible Linux)**
-   **CMake** (version 3.10 or higher): `sudo apt update && sudo apt install cmake`
-   **OpenCV Development Libraries**: `sudo apt install libopencv-dev`
-   **Git** (for version control, assumed): `sudo apt install git`

### Build Steps

1.  **Navigate to the Project Root:** Open a terminal and go to the directory containing this `README.md` file (e.g., `/home/user/WRO_2025/coding_part/wro2025_vehicle`).
2.  **Create a Build Directory:** It's good practice to build outside the source tree.
    ```bash
    mkdir build
    cd build
    ```
3.  **Configure the Build:** Run CMake from within the `build` directory, pointing it to the parent directory where `CMakeLists.txt` resides.
    ```bash
    cmake ..
    ```
4.  **Compile:** Use `make` to compile the project.
    ```bash
    make
    ```
5.  **Run:** If compilation is successful, the executable (named `WRO2025_Vehicle_Controller` based on the `CMakeLists.txt` provided earlier) will be created in the `build` directory. Run it from the terminal.
    ```bash
    ./WRO2025_Vehicle_Controller
    ```

## 5. Relationship to Electromechanical Components

-   **Raspberry Pi Zero 2W:** Runs this entire software stack. It receives processed sensor data and sends high-level commands.
-   **Pi Camera Module (CSI):** The source of image data for the `vision` modules.
-   **Arduino Nano:** Runs the real-time control firmware. Receives motor/servo commands via serial from this Pi software and sends back sensor data.
-   **MPU6050 IMU, Ultrasonic Sensors:** Their data is read by the Nano, packaged, and sent to the Pi via the serial link for processing by `sensors` and `navigation` modules.
-   **L298N Motor Driver, Servo Motor:** Controlled by the Nano based on commands received from this Pi software via the serial link.
