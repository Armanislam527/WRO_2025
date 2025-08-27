#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// --- Pin Definitions ---
const int START_BUTTON_PIN = A3;  // D17
const int IR_FRONT_LEFT_PIN = 6;  // D6
const int IR_FRONT_RIGHT_PIN = 7; // D7
const int IR_RIGHT_45_PIN = 8;    // D8
const int IR_LEFT_45_PIN = 9;     // D9
const int US_FRONT_TRIG_PIN = 2;  // D2
const int US_FRONT_ECHO_PIN = 3;  // D3
const int US_REAR_TRIG_PIN = 4;   // D4
const int US_REAR_ECHO_PIN = 5;   // D5
const int SERVO_PIN = 10;         // D10
Servo steeringServo;
// L298N Motor Driver Pins
const int ENA_PIN = 11; // D11
const int IN1_PIN = 12; // D12
const int IN2_PIN = 13; // D13
const int ENB_PIN = A0; // D14
const int IN3_PIN = A1; // D15
const int IN4_PIN = A2; // D16

// MPU6050 IMU
MPU6050 mpu;
bool imuReady = false;

// --- Variables ---
bool startButtonPressed = false; // Tracks if button was physically pressed
bool runStarted = false;         // Flag to control main driving loop
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

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial Monitor (important for reliable startup)

  // Initialize Pins
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
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(targetServoAngle);
  stopMotors(); // Ensure motors are off at start

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection())
  {
    imuReady = true;
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    // Optional calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
  }
  else
  {
    imuReady = false;
    Serial.println("MPU6050 connection failed");
  }
  // Small delay after setup to stabilize
  delay(100);
  Serial.println("Nano Initialized and Ready");
}

void loop()
{
  // --- 1. Check Start Button (Robust Single Press Detection) ---
  // --- 1. Check Start Button (MODIFIED FOR MULTIPLE PRESSES) ---
// Check if the button is pressed (LOW due to INPUT_PULLUP)
if (digitalRead(START_BUTTON_PIN) == LOW) {
  // Simple debounce delay
  delay(50);
  // Re-check if the button is still pressed after debounce delay
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    // Button is confirmed pressed
    // Send the START command immediately
    Serial.println("START"); // Send start signal to Pi
    Serial.flush(); // Ensure the message is sent before potentially processing further

    // Wait for the button to be released to avoid multiple rapid triggers
    // This creates a "single press = single START" behavior per physical press-release cycle
    while(digitalRead(START_BUTTON_PIN) == LOW) {
      delay(10); // Small delay while waiting for release
    }
    // Optional: Add a small delay after release to prevent accidental re-triggering
    // if the button has mechanical bounce on release.
    delay(50); // Additional debounce after release
  }
}
// --- End Modified Start Button Check ---

  // --- 2. Read Sensors Periodically ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorSendTime >= SENSOR_SEND_INTERVAL)
  {
    readSensors();
    sendSensorData();
    lastSensorSendTime = currentMillis;
  }

  // --- 3. Check for Serial Commands from Pi ---
  if (Serial.available() > 0)
  {
    receivedCommand = Serial.readStringUntil('\n');
    receivedCommand.trim();
    if (receivedCommand == "ACK_START")
    {
      // Pi acknowledged the START. Enable the main driving loop.
      runStarted = true;
      Serial.println("ACK_START received, runStarted = true");
    }
    else
    {
      parseAndExecuteCommand(receivedCommand);
    }
  }

  // --- 4. Update Actuators if Run Started ---
  if (runStarted)
  {
    updateMotors();
    updateServo();
  }

  delay(10); // Main loop delay
}

// --- Function Definitions ---
void readSensors()
{
  irStates[0] = digitalRead(IR_FRONT_LEFT_PIN);
  irStates[1] = digitalRead(IR_FRONT_RIGHT_PIN);
  irStates[2] = digitalRead(IR_RIGHT_45_PIN);
  irStates[3] = digitalRead(IR_LEFT_45_PIN);
  usDistances[0] = readUltrasonic(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
  usDistances[1] = readUltrasonic(US_REAR_TRIG_PIN, US_REAR_ECHO_PIN);
  if (imuReady)
  {
    mpu.getMotion6(&imuAccelX, &imuAccelY, &imuAccelZ, &imuGyroX, &imuGyroY, &imuGyroZ);
  }
  else
  {
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
  {
    return -1; // Timeout or error
  }
  return (duration * 1000) / 5824; // Convert to mm
}

void sendSensorData()
{
  Serial.print("SENSORS,");
  Serial.print(irStates[0]);
  Serial.print(",");
  Serial.print(irStates[1]);
  Serial.print(",");
  Serial.print(irStates[2]);
  Serial.print(",");
  Serial.print(irStates[3]);
  Serial.print(",");
  Serial.print(usDistances[0]);
  Serial.print(",");
  Serial.print(usDistances[1]);
  Serial.print(",");
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

void parseAndExecuteCommand(String command)
{
  command.trim();
  if (command.startsWith("MOTOR"))
  {
    int commaIndex1 = command.indexOf(',');
    int commaIndex2 = command.indexOf(',', commaIndex1 + 1);
    if (commaIndex1 != -1 && commaIndex2 != -1)
    {
      String leftStr = command.substring(commaIndex1 + 1, commaIndex2);
      String rightStr = command.substring(commaIndex2 + 1);
      targetLeftSpeed = leftStr.toInt();
      targetRightSpeed = rightStr.toInt();
    }
  }
  else if (command.startsWith("SERVO"))
  {
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1)
    {
      String angleStr = command.substring(commaIndex + 1);
      targetServoAngle = angleStr.toInt();
      targetServoAngle = constrain(targetServoAngle, 0, 180);
    }
  }
  // Note: ACK_START is handled in the main loop now
}

void updateMotors()
{
  setMotor(ENA_PIN, IN1_PIN, IN2_PIN, targetLeftSpeed);
  setMotor(ENB_PIN, IN3_PIN, IN4_PIN, targetRightSpeed);
}

void setMotor(int enPin, int in1Pin, int in2Pin, int speed)
{
  speed = constrain(speed, -255, 255);
  if (speed >= 0)
  {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    speed = -speed;
  }
  analogWrite(enPin, speed);
}

void updateServo()
{
  steeringServo.write(targetServoAngle);
}

void stopMotors()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
}
