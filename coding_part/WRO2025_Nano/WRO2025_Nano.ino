// WRO2025_Nano.ino
// Main application for WRO 2025 Nano controller

// Include standard libraries
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>

// Include project configuration
#include "config.h"

// Include all module headers
#include "imu_handler.h"
#include "us_sensor_array.h"
#include "motor_controller.h"
#include "servo_controller.h"
#include "serial_protocol.h"
#include "serial_parser.h"
#include "watchdog.h"
#include "e_stop.h"

// --- Module instances ---
IMUHandler imuHandler;
USSensorArray usSensorArray;
MotorController motorController;
ServoController servoController;
SerialParser serialParser;
Watchdog watchdog;
EStopHandler eStopHandler;

// --- Global State variables ---
bool isStarted = false;         // Has the START button been pressed?
bool missionGoReceived = false; // Has the Pi sent the "GO" command?
unsigned long lastUSTime = 0;
unsigned long lastIMUTime = 0;
int motorSpeed = 0;
int servoAngle = SERVO_CENTER_ANGLE;
unsigned long lastHighSpeedSensorTime = 0;
const unsigned long HIGH_SPEED_SENSOR_INTERVAL = 50; // Send combined data every 50ms (20 Hz)

// Sensor data structures
IMUData currentIMUData;
USSensorData currentUSData;

// --- Function prototypes (good practice for Arduino IDE) ---
void processSerialCommands();

void setup()
{
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);

    // Initialize all modules
    imuHandler.initialize();
    usSensorArray.initialize(
        US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN,
        US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN,
        US_BACK_TRIG_PIN, US_BACK_ECHO_PIN,
        US_LEFT_TRIG_PIN, US_LEFT_ECHO_PIN);

    motorController.initialize(MOTOR_ENA_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
    servoController.initialize(SERVO_PIN);

    watchdog.initialize(WATCHDOG_TIMEOUT);
    eStopHandler.initialize();

    // Setup start button with internal pull-up
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    // Setup status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // Small delay to allow sensors to stabilize
    delay(100);

    // Calibrate IMU (this will take some time)
    // In a competition setting, you might want to do this during preparation time
    // rather than during the setup phase
    imuHandler.calibrate();

    // Indicate setup complete
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    // Reset timers to start fresh after setup indication
    lastUSTime = millis();
    lastIMUTime = millis();
    lastHighSpeedSensorTime = millis();
}

void loop()
{
    // --- PRE-START PHASE ---
    // Wait for the physical start button press
    if (!isStarted)
    {
        // Continuously process commands (e.g., heartbeat to keep watchdog happy)
        processSerialCommands();

        // Check for the physical start button press
        if (digitalRead(START_BUTTON_PIN) == LOW)
        {              // Assuming pull-up, LOW = pressed
            delay(20); // Debounce
            if (digitalRead(START_BUTTON_PIN) == LOW)
            { // Check again after debounce
                Serial.println("Nano: Physical START button pressed and debounced.");
                isStarted = true;
                digitalWrite(STATUS_LED_PIN, HIGH); // Visual confirmation

                // Send start acknowledgment to Pi
                serialParser.sendStartAck(true);

                // Reset timestamps for immediate sensor reads on start
                lastUSTime = millis();
                lastIMUTime = millis();
                lastHighSpeedSensorTime = millis();

                // Do not 'return' here, let the loop continue to the operational phase
                // The state change (isStarted = true) will take effect next iteration
            }
        }
        // If not started, ONLY processSerialCommands and check button
        return; // Exit loop early, wait for next iteration
    }

    // --- OPERATIONAL PHASE (isStarted == true) ---

    // 1. Process incoming serial commands (Priority 1: React to commands immediately)
    processSerialCommands();

    // 2. Wait for explicit GO command from Pi before doing main tasks
    if (!missionGoReceived)
    {
        // We are started (button pressed) but waiting for Pi's GO signal.
        // Only processSerialCommands and watchdog (for safety) are active.
        // Check watchdog even in this state to catch communication loss immediately
        if (watchdog.isExpired())
        {
            eStopHandler.activateEStop();
        }
        delay(10); // Small delay to prevent busy-waiting
        return;    // Do not proceed with sensors/actuators until GO received
    }

    // --- MAIN OPERATIONAL LOOP (runs only AFTER missionGoReceived == true) ---
    unsigned long currentTime = millis();

    // 3. Read sensors at defined intervals
    // Read ultrasonic sensors
    if (currentTime - lastUSTime > US_MEASURE_INTERVAL)
    {
        if (usSensorArray.readData(currentUSData))
        {
            // Optional: Send individual US updates if needed by Pi logic frequently
            // serialParser.sendSensorData(...);
        }
        lastUSTime = currentTime;
    }

    // Read IMU data (more frequently than US sensors if needed, but we send combined)
    if (currentTime - lastIMUTime > 50)
    {                                        // e.g., 20Hz
        imuHandler.readData(currentIMUData); // Result stored in currentIMUData
        lastIMUTime = currentTime;
    }

    // Send ALL sensor data together at high speed
    if (currentTime - lastHighSpeedSensorTime > HIGH_SPEED_SENSOR_INTERVAL)
    {
        // Prepare the compact data structure
        CompactSensorData compactData;

        // Fill US data (convert from unsigned int to uint16_t)
        compactData.frontDistance = static_cast<uint16_t>(currentUSData.frontDistance);
        compactData.rightDistance = static_cast<uint16_t>(currentUSData.rightDistance);
        compactData.backDistance = static_cast<uint16_t>(currentUSData.backDistance);
        compactData.leftDistance = static_cast<uint16_t>(currentUSData.leftDistance);

        // Fill IMU data (scale floats to integers for compact transmission)
        compactData.accelX = static_cast<int16_t>(currentIMUData.accelX * 1000.0f); // Convert g to milli-g
        compactData.accelY = static_cast<int16_t>(currentIMUData.accelY * 1000.0f);
        compactData.accelZ = static_cast<int16_t>(currentIMUData.accelZ * 1000.0f);
        compactData.gyroX = static_cast<int16_t>(currentIMUData.gyroX * 1000.0f); // Convert deg/s to milli-deg/s
        compactData.gyroY = static_cast<int16_t>(currentIMUData.gyroY * 1000.0f);
        compactData.gyroZ = static_cast<int16_t>(currentIMUData.gyroZ * 1000.0f);
        Serial.println("Nano: Sending CMD_ALL_SENSOR_DATA_COMPACT packet.");
        // --- END ADD ---
        // Send the compact data packet - This is the FASTEST way
        serialParser.sendCompactSensorData(compactData);

        lastHighSpeedSensorTime = currentTime;
    }

    // 4. Update actuators with current values (only if e-stop is not active)
    if (!eStopHandler.isEStopActive())
    {
        motorController.setSpeed(motorSpeed);
        servoController.setAngle(servoAngle);
    }

    // 5. Check watchdog (Priority: Safety)
    if (watchdog.isExpired())
    {
        eStopHandler.activateEStop();
    }

    // Small delay to prevent overwhelming the system
    delay(5);
}

// --- Function Definitions ---

// Function to process incoming serial commands
void processSerialCommands()
{
    Packet packet;

    // Try to read a packet
    if (serialParser.readPacket(packet))
    {
        // Process valid packet
        switch (packet.command)
        {
        case CMD_SET_MOTOR_SPEED:
            if (packet.length >= 1)
            {
                // Convert byte to signed int (-128 to 127 mapped to -255 to 255)
                int8_t speedByte = (int8_t)packet.data[0];
                motorSpeed = map(speedByte, -128, 127, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

                // Feed watchdog when receiving motor commands
                watchdog.feed();
            }
            break;

        case CMD_SET_SERVO_ANGLE:
            if (packet.length >= 1)
            {
                servoAngle = packet.data[0];
                if (servoAngle > SERVO_MAX_ANGLE)
                    servoAngle = SERVO_MAX_ANGLE;
                if (servoAngle < SERVO_MIN_ANGLE)
                    servoAngle = SERVO_MIN_ANGLE;

                // Feed watchdog when receiving servo commands
                watchdog.feed();
            }
            break;

        case CMD_EMERGENCY_STOP:
            eStopHandler.activateEStop();
            break;

        case CMD_HEARTBEAT:
            // Feed the watchdog on heartbeat
            watchdog.feed();
            break;

        case CMD_REQUEST_SENSOR_DATA:
            // Send current US sensor data (consider adding IMU if needed separately)
            serialParser.sendSensorData(
                currentUSData.frontDistance,
                currentUSData.rightDistance,
                currentUSData.backDistance,
                currentUSData.leftDistance);
            break;

        case CMD_PI_GO_SIGNAL:
            // Pi signals the start of the mission run
            if (isStarted)
            { // Only accept GO if button was pressed first (Rule 9.11)
                missionGoReceived = true;
                Serial.println("Nano: CMD_PI_GO_SIGNAL received, missionGoReceived = true.");
                // Optional: Send an ack back to Pi confirming GO received
                // serialParser.sendAck(CMD_PI_GO_SIGNAL); // You'd need to implement sendAck
                // Blink LED quickly to indicate GO
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
                delay(50);
                digitalWrite(STATUS_LED_PIN, HIGH); // Ensure LED stays on
            }
            else
            {
                // GO received before button press - protocol error
                serialParser.sendError(0x02); // Define error code, e.g., "GO before START"
            }
            break;

        default:
            // Unknown command, send error
            serialParser.sendError(0x01); // Unknown command error
            break;
        }
    }
}