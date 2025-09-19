// main.ino
// Main application for WRO 2025 Nano controller

#include "config.h"
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>

// Include all module headers
#include "sensors/imu_handler.h"
#include "sensors/us_sensor_array.h"
#include "actuators/motor_controller.h"
#include "actuators/servo_controller.h"
#include "communication/serial_protocol.h"
#include "communication/serial_parser.h"
#include "safety/watchdog.h"
#include "safety/e_stop.h"

// Module instances
IMUHandler imuHandler;
USSensorArray usSensorArray;
MotorController motorController;
ServoController servoController;
SerialParser serialParser;
Watchdog watchdog;
EStopHandler eStopHandler;

// State variables
bool isStarted = false;
unsigned long lastUSTime = 0;
unsigned long lastIMUTime = 0;
int motorSpeed = 0;
int servoAngle = SERVO_CENTER_ANGLE;
unsigned long lastHighSpeedSensorTime = 0;
const unsigned long HIGH_SPEED_SENSOR_INTERVAL = 50; // Send combined data every 50ms (20 Hz)
bool usDataUpdated = false;
bool imuDataUpdated = false;
// Sensor data structures
IMUData currentIMUData;
USSensorData currentUSData;

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
    // imuHandler.calibrate();

    // Indicate setup complete
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
}

void loop()
{
    // Check for start button press (only if not already started)
    if (!isStarted)
    {
        processSerialCommands();
        if (digitalRead(START_BUTTON_PIN) == LOW)
        {              // Assuming pull-up resistor
            delay(20); // Debounce
            if (digitalRead(START_BUTTON_PIN) == LOW)
            {
                isStarted = true;
                digitalWrite(STATUS_LED_PIN, HIGH);

                // Send start acknowledgment to Pi
                serialParser.sendStartAck(true);
                lastUSTime = millis();
                lastIMUTime = millis();
                lastHighSpeedSensorTime = millis();
                return; // Skip rest of loop this iteration
            }
        }

        return; // Don't do anything else until started
    }

    // Process incoming serial commands
    processSerialCommands();
    // --- NEW: Check for explicit GO command before doing main tasks ---
    if (!missionGoReceived)
    {
        // We are started (button pressed) but waiting for Pi's GO signal.
        // Only processSerialCommands and watchdog (for safety) are active.
        // Check watchdog even in this state to catch communication loss immediately after button press
        if (watchdog.isExpired())
        {
            eStopHandler.activateEStop();
        }
        return; // Do not proceed with sensors/actuators until GO received
    }
    // Read sensors at defined intervals
    unsigned long currentTime = millis();

    // Read ultrasonic sensors
    if (currentTime - lastUSTime > US_MEASURE_INTERVAL)
    {
        if (usSensorArray.readData(currentUSData))
        {
            // Send sensor data to Pi
            serialParser.sendSensorData(
                currentUSData.frontDistance,
                currentUSData.rightDistance,
                currentUSData.backDistance,
                currentUSData.leftDistance);
        }
        lastUSTime = currentTime;
    }

    // Read IMU data (more frequently than US sensors)
    if (currentTime - lastIMUTime > 50)
    { // 20Hz
        if (imuHandler.readData(currentIMUData))
        {
            // IMU data is sent with other sensor data or on request
        }
        lastIMUTime = currentTime;
    }

    // Update actuators with current values (only if e-stop is not active)
    if (!eStopHandler.isEStopActive())
    {
        motorController.setSpeed(motorSpeed);
        servoController.setAngle(servoAngle);
    }
    // Read ultrasonic sensors at defined interval
    if (currentTime - lastUSTime > US_MEASURE_INTERVAL)
    {
        if (usSensorArray.readData(currentUSData))
        {
            usDataUpdated = true;
            // Optional: Send individual US updates if needed by Pi logic
            // serialParser.sendSensorData(currentUSData.frontDistance, ...);
        }
        lastUSTime = currentTime;
    }

    // Read IMU data at a faster rate
    if (currentTime - lastIMUTime > 20)
    { // e.g., 50Hz
        if (imuHandler.readData(currentIMUData))
        {
            imuDataUpdated = true;
        }
        lastIMUTime = currentTime;
    }

    // Send ALL sensor data together at high speed (Priority 2: Regular sensor updates)
    // This combines US and IMU data into one efficient packet
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
        // Example scaling - adjust based on your sensor's typical range and required precision
        compactData.accelX = static_cast<int16_t>(currentIMUData.accelX * 1000.0f); // Convert g to milli-g
        compactData.accelY = static_cast<int16_t>(currentIMUData.accelY * 1000.0f);
        compactData.accelZ = static_cast<int16_t>(currentIMUData.accelZ * 1000.0f);
        compactData.gyroX = static_cast<int16_t>(currentIMUData.gyroX * 1000.0f); // Convert deg/s to milli-deg/s
        compactData.gyroY = static_cast<int16_t>(currentIMUData.gyroY * 1000.0f);
        compactData.gyroZ = static_cast<int16_t>(currentIMUData.gyroZ * 1000.0f);

        // Send the compact data packet - This is the FASTEST way
        serialParser.sendCompactSensorData(compactData);

        lastHighSpeedSensorTime = currentTime;
    }

    // Update actuators (Priority 3: Apply commands)
    if (!eStopHandler.isEStopActive())
    {
        motorController.setSpeed(motorSpeed);
        servoController.setAngle(servoAngle);
    }

    // Check watchdog (Priority 4: Safety)
    if (watchdog.isExpired())
    {
        eStopHandler.activateEStop();
    }

    // Minimal delay to prevent busy-waiting, but keep it short for responsiveness
    // Consider removing this if your sensor reading intervals are sufficient
    delay(5);
}
// Check watchdog
if (watchdog.isExpired())
{
    // Emergency stop if watchdog expires
    eStopHandler.activateEStop();
}

// Small delay to prevent overwhelming the system
delay(10);
}

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
            // Send all sensor data
            serialParser.sendSensorData(
                currentUSData.frontDistance,
                currentUSData.rightDistance,
                currentUSData.backDistance,
                currentUSData.leftDistance);
            // In a more complete implementation, we would also send IMU data
            break;
        case CMD_PI_GO_SIGNAL: // Define this in serial_protocol.h, e.g., #define CMD_PI_GO_SIGNAL 0x06
            if (isStarted)
            { // Only accept GO if button was pressed first
                missionGoReceived = true;
                // Optional: Send an ack back to Pi confirming GO received
                // serialParser.sendAck(CMD_PI_GO_SIGNAL); // You'd need to implement sendAck
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // Blink LED to indicate GO
            }
            else
            {
                // GO received before button press - ignore or log error
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