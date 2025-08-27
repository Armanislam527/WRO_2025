#include <Servo.h>
#include <Wire.h>

// --- Pin Definitions ---
const int START_BUTTON_PIN = A3;
const int IR_FRONT_LEFT_PIN = 6;
const int IR_FRONT_RIGHT_PIN = 7;
const int IR_RIGHT_45_PIN = 8;
const int IR_LEFT_45_PIN = 9;
const int US_FRONT_TRIG_PIN = 2;
const int US_FRONT_ECHO_PIN = 3;
const int US_REAR_TRIG_PIN = 4;
const int US_REAR_ECHO_PIN = 5;
const int SERVO_PIN = 10;
const int ENA_PIN = 11;
const int IN1_PIN = 12;
const int IN2_PIN = 13;

// --- MPU6050 Registers ---
const byte MPU6050_ADDR = 0x68;
const byte PWR_MGMT_1 = 0x6B;
const byte ACCEL_XOUT_H = 0x3B;
const byte GYRO_XOUT_H = 0x43;

// --- Constants ---
const uint16_t SENSOR_SEND_INTERVAL = 100;
const uint16_t BUTTON_DEBOUNCE_DELAY = 50;
const uint16_t ULTRASONIC_READ_INTERVAL = 50;
const uint16_t SERIAL_BAUD_RATE = 500000; // Increased baud rate

// --- Variables ---
volatile bool runStarted = false;
volatile bool lastButtonState = HIGH;
volatile uint32_t lastSensorSendTime = 0;
volatile uint32_t lastButtonCheckTime = 0;
volatile uint32_t lastUltrasonicReadTime = 0;
volatile bool ultrasonicToggle = false;

// Sensor data packed efficiently
struct SensorData
{
    uint8_t irStates; // Bit-packed: 0:FL, 1:FR, 2:R45, 3:L45
    int16_t usFront;
    int16_t usRear;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} sensorData;

// Command variables
volatile int8_t targetMotorSpeed = 0;
volatile uint8_t targetServoAngle = 90;

// Objects
Servo steeringServo;

// --- MPU6050 Functions ---
inline byte readMPU6050Register(byte reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1);
    return Wire.read();
}

inline void writeMPU6050Register(byte reg, byte data)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

inline void readMPU6050Data()
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    sensorData.accelX = (Wire.read() << 8) | Wire.read();
    sensorData.accelY = (Wire.read() << 8) | Wire.read();
    sensorData.accelZ = (Wire.read() << 8) | Wire.read();
    Wire.read();
    Wire.read(); // Skip temperature
    sensorData.gyroX = (Wire.read() << 8) | Wire.read();
    sensorData.gyroY = (Wire.read() << 8) | Wire.read();
    sensorData.gyroZ = (Wire.read() << 8) | Wire.read();
}

// --- Setup Function ---
void setup()
{
    // Initialize serial at high baud rate
    Serial.begin(SERIAL_BAUD_RATE);

    // Pin setup
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(IR_FRONT_LEFT_PIN, INPUT);
    pinMode(IR_FRONT_RIGHT_PIN, INPUT);
    pinMode(IR_RIGHT_45_PIN, INPUT);
    pinMode(IR_LEFT_45_PIN, INPUT);
    pinMode(US_FRONT_TRIG_PIN, OUTPUT);
    pinMode(US_FRONT_ECHO_PIN, INPUT);
    pinMode(US_REAR_TRIG_PIN, OUTPUT);
    pinMode(US_REAR_ECHO_PIN, INPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    // Initialize servo
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(targetServoAngle);

    // Initialize motor to stop
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);

    // Initialize MPU6050
    Wire.begin();
    Wire.setClock(400000);
    writeMPU6050Register(PWR_MGMT_1, 0x00);
    delay(100);

    // Set MPU6050 ranges
    writeMPU6050Register(0x1B, 0x00); // Gyro ±250°/s
    writeMPU6050Register(0x1C, 0x00); // Accel ±2g

    // Setup complete
    Serial.println("NANO:READY");
}

// --- Main Loop ---
void loop()
{
    uint32_t currentMillis = millis();

    // Check start button with debouncing
    if (currentMillis - lastButtonCheckTime >= BUTTON_DEBOUNCE_DELAY)
    {
        checkStartButton();
        lastButtonCheckTime = currentMillis;
    }

    // Read sensors
    readSensors();

    // Send sensor data at intervals
    if (currentMillis - lastSensorSendTime >= SENSOR_SEND_INTERVAL)
    {
        sendSensorData();
        lastSensorSendTime = currentMillis;
    }

    // Process serial commands
    processSerialCommands();

    // Update actuators if running
    if (runStarted)
    {
        updateMotor();
        updateServo();
    }
}

// --- Button Handling ---
void checkStartButton()
{
    bool buttonState = digitalRead(START_BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH)
    {
        Serial.println("START");
        // Simple debounce by waiting for button release
        while (digitalRead(START_BUTTON_PIN) == LOW)
        {
            delay(10);
        }
    }
    lastButtonState = buttonState;
}

// --- Sensor Reading ---
void readSensors()
{
    // Read IR sensors (bit-packed)
    sensorData.irStates = 0;
    if (digitalRead(IR_FRONT_LEFT_PIN))
        sensorData.irStates |= 0x01;
    if (digitalRead(IR_FRONT_RIGHT_PIN))
        sensorData.irStates |= 0x02;
    if (digitalRead(IR_RIGHT_45_PIN))
        sensorData.irStates |= 0x04;
    if (digitalRead(IR_LEFT_45_PIN))
        sensorData.irStates |= 0x08;

    // Read MPU6050
    readMPU6050Data();

    // Read ultrasonic sensors alternately
    uint32_t currentMillis = millis();
    if (currentMillis - lastUltrasonicReadTime >= ULTRASONIC_READ_INTERVAL)
    {
        if (ultrasonicToggle)
        {
            sensorData.usFront = readUltrasonic(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
        }
        else
        {
            sensorData.usRear = readUltrasonic(US_REAR_TRIG_PIN, US_REAR_ECHO_PIN);
        }
        ultrasonicToggle = !ultrasonicToggle;
        lastUltrasonicReadTime = currentMillis;
    }
}

// --- Ultrasonic Reading ---
int16_t readUltrasonic(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? -1 : (duration * 0.343) / 2;
}

// --- Data Transmission ---
void sendSensorData()
{
    // Binary data transmission for maximum efficiency
    Serial.write(0xAA); // Start byte
    Serial.write((uint8_t *)&sensorData, sizeof(sensorData));
    Serial.write(0x55); // End byte
}

// --- Command Processing ---
void processSerialCommands()
{
    while (Serial.available() > 0)
    {
        char command = Serial.read();

        switch (command)
        {
        case 'P': // PING
            if (Serial.read() == 'I' && Serial.read() == 'N' && Serial.read() == 'G')
            {
                Serial.println("PONG");
            }
            break;

        case 'M': // MOTOR
            if (Serial.read() == 'O' && Serial.read() == 'T' && Serial.read() == 'O' && Serial.read() == 'R')
            {
                targetMotorSpeed = Serial.parseInt();
            }
            break;

        case 'S': // SERVO or START acknowledgment
            if (Serial.read() == 'E' && Serial.read() == 'R' && Serial.read() == 'V' && Serial.read() == 'O')
            {
                targetServoAngle = Serial.parseInt();
            }
            else if (Serial.read() == 'T' && Serial.read() == 'A' && Serial.read() == 'R' && Serial.read() == 'T')
            {
                runStarted = true;
            }
            break;
        }

        // Clear any remaining data in the buffer
        while (Serial.available() > 0)
            Serial.read();
    }
}

// --- Actuator Control ---
void updateMotor()
{
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