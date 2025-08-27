#include <Servo.h>
#include <Wire.h>

// --- Pin Definitions ---
// Start Button
const int START_BUTTON_PIN = A3; // D17

// IR Sensors
const int IR_FRONT_LEFT_PIN = 6;  // D6
const int IR_FRONT_RIGHT_PIN = 7; // D7
const int IR_RIGHT_45_PIN = 8;    // D8
const int IR_LEFT_45_PIN = 9;     // D9

// Ultrasonic Sensors
const int US_FRONT_TRIG_PIN = 2; // D2
const int US_FRONT_ECHO_PIN = 3; // D3
const int US_REAR_TRIG_PIN = 4;  // D4
const int US_REAR_ECHO_PIN = 5;  // D5

// Servo Motor (Front Steering)
const int SERVO_PIN = 10; // D10
Servo steeringServo;

// L298N Motor Driver (Single Rear Motor - using OUT1 & OUT2)
const int ENA_PIN = 11; // D11 - Enable pin for Motor A
const int IN1_PIN = 12; // D12 - Control pin 1 for Motor A
const int IN2_PIN = 13; // D13 - Control pin 2 for Motor A

// MPU6050 IMU - Custom implementation for clone (device ID 0x70)
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
bool imuReady = false;

// --- Variables ---
bool runStarted = false;
bool lastButtonState = HIGH;
unsigned long lastSensorSendTime = 0;
unsigned long lastButtonCheckTime = 0;
const long SENSOR_SEND_INTERVAL = 100; // Send sensor data every 100ms
const long BUTTON_DEBOUNCE_DELAY = 50;

// Sensor Variables
int irStates[4] = {0, 0, 0, 0};
long usDistances[2] = {0, 0};
int16_t imuAccelX, imuAccelY, imuAccelZ;
int16_t imuGyroX, imuGyroY, imuGyroZ;

// Command Variables
String receivedCommand = "";
int targetMotorSpeed = 0;  // Single motor speed (-255 to 255)
int targetServoAngle = 90; // Servo angle (0-180)

void setup()
{
    Serial.begin(115200);
    delay(2000); // Wait for serial to stabilize

    // Initialize Start Button Pin
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    // Initialize IR Sensor Pins
    pinMode(IR_FRONT_LEFT_PIN, INPUT);
    pinMode(IR_FRONT_RIGHT_PIN, INPUT);
    pinMode(IR_RIGHT_45_PIN, INPUT);
    pinMode(IR_LEFT_45_PIN, INPUT);

    // Initialize Ultrasonic Sensor Pins
    pinMode(US_FRONT_TRIG_PIN, OUTPUT);
    pinMode(US_FRONT_ECHO_PIN, INPUT);
    pinMode(US_REAR_TRIG_PIN, OUTPUT);
    pinMode(US_REAR_ECHO_PIN, INPUT);

    // Initialize L298N Pins (Single Motor)
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    // Initialize Servo
    if (steeringServo.attach(SERVO_PIN))
    {
        steeringServo.write(targetServoAngle);
        Serial.println("Servo: Initialized successfully");
    }
    else
    {
        Serial.println("Servo: Failed to attach");
    }

    // Initialize Motors to STOP
    stopMotor();

    // Initialize MPU6050 with custom implementation
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MPU6050
    writeMPU6050Register(PWR_MGMT_1, 0x00); // Wake up device
    delay(100);

    // Check if device responds
    byte whoami = readMPU6050Register(0x75); // WHO_AM_I register
    Serial.print("MPU6050 WHO_AM_I: 0x");
    Serial.println(whoami, HEX);

    if (whoami == 0x70 || whoami == 0x68)
    {
        imuReady = true;
        Serial.println("MPU6050: Initialized successfully");

        // Set ranges (±2g accelerometer, ±250°/s gyro)
        writeMPU6050Register(0x1B, 0x00); // Gyro config
        writeMPU6050Register(0x1C, 0x00); // Accel config

        // Perform quick calibration
        calibrateMPU6050();
    }
    else
    {
        imuReady = false;
        Serial.println("MPU6050: Connection failed");
    }

    Serial.println("NANO: Setup complete - Ready for communication");
}

void loop()
{
    // Check for start button press
    checkStartButton();

    // Read and send sensor data at regular intervals
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorSendTime >= SENSOR_SEND_INTERVAL)
    {
        readSensors();
        sendSensorData();
        lastSensorSendTime = currentMillis;
    }

    // Check for commands from Raspberry Pi
    if (Serial.available() > 0)
    {
        receivedCommand = Serial.readStringUntil('\n');
        parseAndExecuteCommand(receivedCommand);
    }

    // Update actuators if run has started
    if (runStarted)
    {
        updateMotor();
        updateServo();
    }

    delay(10); // Small delay to prevent overwhelming the loop
}

// --- MPU6050 Custom Functions ---
byte readMPU6050Register(byte reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (byte)1);
    return Wire.read();
}

void writeMPU6050Register(byte reg, byte data)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void readMPU6050Data()
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    imuAccelX = (Wire.read() << 8) | Wire.read();
    imuAccelY = (Wire.read() << 8) | Wire.read();
    imuAccelZ = (Wire.read() << 8) | Wire.read();
    // Skip temperature
    Wire.read();
    Wire.read();
    imuGyroX = (Wire.read() << 8) | Wire.read();
    imuGyroY = (Wire.read() << 8) | Wire.read();
    imuGyroZ = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU6050()
{
    Serial.println("Calibrating MPU6050...");

    long axOffset = 0, ayOffset = 0, azOffset = 0;
    long gxOffset = 0, gyOffset = 0, gzOffset = 0;
    int samples = 50;

    for (int i = 0; i < samples; i++)
    {
        readMPU6050Data();

        axOffset += imuAccelX;
        ayOffset += imuAccelY;
        azOffset += imuAccelZ - 16384; // Remove 1g gravity from Z

        gxOffset += imuGyroX;
        gyOffset += imuGyroY;
        gzOffset += imuGyroZ;

        delay(5);
    }

    // Apply offsets
    writeMPU6050Register(0x06, (axOffset / samples) >> 8);
    writeMPU6050Register(0x07, (axOffset / samples) & 0xFF);
    writeMPU6050Register(0x08, (ayOffset / samples) >> 8);
    writeMPU6050Register(0x09, (ayOffset / samples) & 0xFF);
    writeMPU6050Register(0x0A, (azOffset / samples) >> 8);
    writeMPU6050Register(0x0B, (azOffset / samples) & 0xFF);

    writeMPU6050Register(0x13, (gxOffset / samples) >> 8);
    writeMPU6050Register(0x14, (gxOffset / samples) & 0xFF);
    writeMPU6050Register(0x15, (gyOffset / samples) >> 8);
    writeMPU6050Register(0x16, (gyOffset / samples) & 0xFF);
    writeMPU6050Register(0x17, (gzOffset / samples) >> 8);
    writeMPU6050Register(0x18, (gzOffset / samples) & 0xFF);

    Serial.println("MPU6050 calibration complete");
}

// --- Button Handling ---
void checkStartButton()
{
    unsigned long currentMillis = millis();
    if (currentMillis - lastButtonCheckTime < BUTTON_DEBOUNCE_DELAY)
        return;
    lastButtonCheckTime = currentMillis;

    int buttonState = digitalRead(START_BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH)
    {
        Serial.println("START");
        Serial.flush(); // Ensure the message is sent immediately

        // Wait for button release with timeout
        unsigned long startWaitTime = millis();
        while (digitalRead(START_BUTTON_PIN) == LOW)
        {
            if (millis() - startWaitTime > 1000)
                break;
            delay(10);
        }
        delay(50); // Additional debounce
    }
    lastButtonState = buttonState;
}

// --- Sensor Reading ---
void readSensors()
{
    // Read IR Sensors
    irStates[0] = digitalRead(IR_FRONT_LEFT_PIN);
    irStates[1] = digitalRead(IR_FRONT_RIGHT_PIN);
    irStates[2] = digitalRead(IR_RIGHT_45_PIN);
    irStates[3] = digitalRead(IR_LEFT_45_PIN);

    // Read Ultrasonic Sensors
    usDistances[0] = readUltrasonic(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
    usDistances[1] = readUltrasonic(US_REAR_TRIG_PIN, US_REAR_ECHO_PIN);

    // Read MPU6050
    if (imuReady)
    {
        readMPU6050Data();
    }
    else
    {
        // Set default values if IMU not available
        imuAccelX = imuAccelY = imuAccelZ = 0;
        imuGyroX = imuGyroY = imuGyroZ = 0;
    }
}

long readUltrasonic(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    if (duration == 0)
        return -1; // Timeout or error

    // Convert to mm: (duration * speed_of_sound_mm_per_us) / 2
    // speed_of_sound = 343000 mm/s = 0.343 mm/μs
    return (duration * 0.343) / 2;
}

// --- Data Transmission ---
void sendSensorData()
{
    Serial.print("SENSORS,");
    // IR sensors
    Serial.print(irStates[0]);
    Serial.print(",");
    Serial.print(irStates[1]);
    Serial.print(",");
    Serial.print(irStates[2]);
    Serial.print(",");
    Serial.print(irStates[3]);
    Serial.print(",");
    // Ultrasonic sensors
    Serial.print(usDistances[0]);
    Serial.print(",");
    Serial.print(usDistances[1]);
    Serial.print(",");
    // IMU data
    Serial.print(imuAccelX);
    Serial.print(",");
    Serial.print(imuAccelY);
    Serial.print(",");
    Serial.print(imuAccelZ);
    Serial.print(",");
    Serial.print(imuGyroX);
    Serial.print(",");
    Serial.print(imuGyroY);
    Serial.print(",");
    Serial.println(imuGyroZ);
}

// --- Command Processing ---
void parseAndExecuteCommand(String command)
{
    command.trim();

    if (command.startsWith("MOTOR"))
    {
        // Format: "MOTOR,speed" (single motor)
        int commaIndex = command.indexOf(',');
        if (commaIndex != -1)
        {
            String speedStr = command.substring(commaIndex + 1);
            targetMotorSpeed = speedStr.toInt();
            targetMotorSpeed = constrain(targetMotorSpeed, -255, 255);
        }
    }
    else if (command.startsWith("SERVO"))
    {
        // Format: "SERVO,angle"
        int commaIndex = command.indexOf(',');
        if (commaIndex != -1)
        {
            String angleStr = command.substring(commaIndex + 1);
            targetServoAngle = angleStr.toInt();
            targetServoAngle = constrain(targetServoAngle, 0, 180);
        }
    }
    else if (command.startsWith("ACK_START"))
    {
        runStarted = true;
        Serial.println("NANO: Run started");
    }
    else if (command.length() > 0)
    {
        Serial.print("NANO: Unknown command: ");
        Serial.println(command);
    }
}

// --- Actuator Control ---
void updateMotor()
{
    // Control single motor using IN1 and IN2 for direction
    if (targetMotorSpeed >= 0)
    {
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
    }
    else
    {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
    }
    analogWrite(ENA_PIN, abs(targetMotorSpeed));
}

void updateServo()
{
    steeringServo.write(targetServoAngle);
}

void stopMotor()
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
    targetMotorSpeed = 0;
}