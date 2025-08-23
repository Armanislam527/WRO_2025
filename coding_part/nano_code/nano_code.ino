#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// --- Pin Definitions ---
// Start Button
const int START_BUTTON_PIN = A3; // D17

// IR Sensors
const int IR_FRONT_LEFT_PIN = 6;   // D6
const int IR_FRONT_RIGHT_PIN = 7;  // D7
const int IR_RIGHT_45_PIN = 8;     // D8
const int IR_LEFT_45_PIN = 9;      // D9

// Ultrasonic Sensors
const int US_FRONT_TRIG_PIN = 2;   // D2
const int US_FRONT_ECHO_PIN = 3;   // D3
const int US_REAR_TRIG_PIN = 4;    // D4
const int US_REAR_ECHO_PIN = 5;    // D5

// Servo Motor
const int SERVO_PIN = 10;          // D10
Servo steeringServo;

// L298N Motor Driver Pins
// Motor A (Left Motor)
const int ENA_PIN = 11;            // D11
const int IN1_PIN = 12;            // D12
const int IN2_PIN = 13;            // D13
// Motor B (Right Motor)
const int ENB_PIN = A0;            // D14
const int IN3_PIN = A1;            // D15
const int IN4_PIN = A2;            // D16

// MPU6050 IMU
MPU6050 mpu;
bool imuReady = false;

// --- Variables ---
bool startButtonPressed = false;
bool runStarted = false;
unsigned long lastSensorSendTime = 0;
const long SENSOR_SEND_INTERVAL = 100; // Send sensor data every 100ms

// Sensor Variables
int irStates[4] = {0, 0, 0, 0}; // FL, FR, R45, L45
long usDistances[2] = {0, 0};   // Front, Rear
int16_t imuAccelX, imuAccelY, imuAccelZ;
int16_t imuGyroX, imuGyroY, imuGyroZ;

// Command Variables
String receivedCommand = "";
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int targetServoAngle = 90; // Center

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor (optional for debugging)

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

  // Initialize L298N Pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Initialize Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(targetServoAngle);

  // Initialize Motors to STOP
  stopMotors();

  // Initialize MPU6050
  Wire.begin();
  if (mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    imuReady = true;
    // Calibration might be needed here, but often not required for relative measurements
    // mpu.calibrateGyro();
    // mpu.setThreshold(3);
  } else {
    imuReady = false;
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // --- 1. Check Start Button ---
  if (digitalRead(START_BUTTON_PIN) == LOW && !startButtonPressed) {
    delay(50); // Simple debounce
    if (digitalRead(START_BUTTON_PIN) == LOW) {
      startButtonPressed = true;
      Serial.println("START"); // Send start signal to Pi
    }
  }

  // --- 2. Read Sensors Periodically ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
    readSensors();
    sendSensorData();
    lastSensorSendTime = currentMillis;
  }

  // --- 3. Check for Serial Commands from Pi ---
  if (Serial.available() > 0) {
    receivedCommand = Serial.readStringUntil('\n');
    parseAndExecuteCommand(receivedCommand);
  }

  // --- 4. Update Actuators if Run Started ---
  if (runStarted) {
    updateMotors();
    updateServo();
  }

  delay(10); // Main loop delay
}

// --- Function Definitions ---

void readSensors() {
  // Read IR Sensors
  irStates[0] = digitalRead(IR_FRONT_LEFT_PIN);
  irStates[1] = digitalRead(IR_FRONT_RIGHT_PIN);
  irStates[2] = digitalRead(IR_RIGHT_45_PIN);
  irStates[3] = digitalRead(IR_LEFT_45_PIN);

  // Read Ultrasonic Sensors
  usDistances[0] = readUltrasonic(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
  usDistances[1] = readUltrasonic(US_REAR_TRIG_PIN, US_REAR_ECHO_PIN);

  // Read MPU6050
  if (imuReady) {
    mpu.readRawAccel();
    mpu.readRawGyro();
    imuAccelX = mpu.getAccelX();
    imuAccelY = mpu.getAccelY();
    imuAccelZ = mpu.getAccelZ();
    imuGyroX = mpu.getGyroX();
    imuGyroY = mpu.getGyroY();
    imuGyroZ = mpu.getGyroZ();
  } else {
    imuAccelX = imuAccelY = imuAccelZ = 0;
    imuGyroX = imuGyroY = imuGyroZ = 0;
  }
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) {
    return -1; // Timeout or error
  }
  // Calculate distance in mm (speed of sound = 343 m/s)
  // duration is in microseconds, distance = (duration / 2) * speed_of_sound
  // speed_of_sound = 343000 mm/s, so distance (mm) = duration * 0.1715
  // Simplified: duration / 58.24 for cm, or duration / 582.4 for mm
  // Using integer math: (duration * 1000) / 5824
  return (duration * 1000) / 5824;
}

void sendSensorData() {
  Serial.print("SENSORS,");
  Serial.print(irStates[0]); Serial.print(",");
  Serial.print(irStates[1]); Serial.print(",");
  Serial.print(irStates[2]); Serial.print(",");
  Serial.print(irStates[3]); Serial.print(",");
  Serial.print(usDistances[0]); Serial.print(",");
  Serial.print(usDistances[1]); Serial.print(",");
  Serial.print(imuAccelX); Serial.print(",");
  Serial.print(imuAccelY); Serial.print(",");
  Serial.print(imuAccelZ); Serial.print(",");
  Serial.print(imuGyroX); Serial.print(",");
  Serial.print(imuGyroY); Serial.print(",");
  Serial.println(imuGyroZ);
}

void parseAndExecuteCommand(String command) {
  command.trim();
  if (command.startsWith("MOTOR")) {
    int commaIndex1 = command.indexOf(',');
    int commaIndex2 = command.indexOf(',', commaIndex1 + 1);
    if (commaIndex1 != -1 && commaIndex2 != -1) {
      String leftStr = command.substring(commaIndex1 + 1, commaIndex2);
      String rightStr = command.substring(commaIndex2 + 1);
      targetLeftSpeed = leftStr.toInt();
      targetRightSpeed = rightStr.toInt();
    }
  } else if (command.startsWith("SERVO")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      String angleStr = command.substring(commaIndex + 1);
      targetServoAngle = angleStr.toInt();
      targetServoAngle = constrain(targetServoAngle, 0, 180);
    }
  } else if (command.startsWith("ACK_START")) {
    runStarted = true;
    // Optionally acknowledge back or perform other actions
  }
  // Add other command handlers if needed
}

void updateMotors() {
  setMotor(ENA_PIN, IN1_PIN, IN2_PIN, targetLeftSpeed);  // Left Motor
  setMotor(ENB_PIN, IN3_PIN, IN4_PIN, targetRightSpeed); // Right Motor
}

void setMotor(int enPin, int in1Pin, int in2Pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    speed = -speed;
  }
  analogWrite(enPin, speed);
}

void updateServo() {
  steeringServo.write(targetServoAngle);
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
}
