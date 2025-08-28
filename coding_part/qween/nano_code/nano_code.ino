#include <Servo.h>
#include <Wire.h>

// --- Hardware Configuration ---
#define MPU6050_ADDR         0x68
#define PWR_MGMT_1           0x6B
#define ACCEL_CONFIG         0x1C
#define GYRO_CONFIG          0x1B
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43
#define CONFIG               0x1A
#define SMPLRT_DIV           0x19

// Pin Definitions
const uint8_t START_BUTTON_PIN = A3; // D17
const uint8_t IR_PINS[] = {6, 7, 8, 9}; // FL, FR, R45, L45
const uint8_t US_FRONT_TRIG = 2;
const uint8_t US_FRONT_ECHO = 3;
const uint8_t US_REAR_TRIG = 4;
const uint8_t US_REAR_ECHO = 5;
const uint8_t SERVO_PIN = 10;
const uint8_t MOTOR_PWM = 11;  // ENA
const uint8_t MOTOR_IN1 = 12;
const uint8_t MOTOR_IN2 = 13;
const uint8_t MOTOR_IN3 = A0; // IN3 - Not used in current setup
const uint8_t MOTOR_IN4 = A1; // IN4 - Not used in current setup
const uint8_t MOTOR_ENB = A2; // ENB - Not used in current setup

// --- System Configuration ---
const uint32_t SERIAL_BAUD = 500000;
const uint16_t SENSOR_SEND_INTERVAL_MS = 50; // Send sensor data every 50ms
const uint16_t BUTTON_DEBOUNCE_MS = 50;
const uint16_t US_READ_INTERVAL_MS = 25; // Alternate reads every 25ms

// --- Data Structures ---
struct SensorData {
  uint8_t ir_states;       // Bit-packed IR states (4 bits used)
  int16_t us_front_mm;     // Front ultrasonic distance (mm)
  int16_t us_rear_mm;      // Rear ultrasonic distance (mm)
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

// --- Global State Variables ---
volatile bool run_started = false;
volatile bool last_button_state = HIGH;
volatile uint32_t last_sensor_send_time = 0;
volatile uint32_t last_button_check_time = 0;
volatile uint32_t last_us_read_time = 0;
volatile bool us_toggle = false; // Toggle between front/rear US reads

SensorData sensor_data = {0};
Servo steering_servo;

// Motor command buffer
volatile int16_t target_motor_speed = 0;
volatile uint8_t target_servo_angle = 90;

// --- MPU6050 Functions ---
bool initialize_mpu6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(WHO_AM_I); // 0x75
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1);
  uint8_t who_am_i = Wire.read();
  
  if (who_am_i != 0x68 && who_am_i != 0x70) { // 0x70 for some clones
    return false;
  }

  write_mpu6050_register(PWR_MGMT_1, 0x00); // Wake up
  delay(100);
  write_mpu6050_register(CONFIG, 0x00); // Disable FSYNC, set DLPF to 0
  write_mpu6050_register(SMPLRT_DIV, 0x07); // Sample rate = 1kHz / (7 + 1) = 125Hz
  write_mpu6050_register(GYRO_CONFIG, 0x00); // Gyro full scale: +/- 250 deg/s
  write_mpu6050_register(ACCEL_CONFIG, 0x00); // Accel full scale: +/- 2g
  delay(100);
  return true;
}

void write_mpu6050_register(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void read_mpu6050_data() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  sensor_data.accel_x = (Wire.read() << 8) | Wire.read();
  sensor_data.accel_y = (Wire.read() << 8) | Wire.read();
  sensor_data.accel_z = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  sensor_data.gyro_x = (Wire.read() << 8) | Wire.read();
  sensor_data.gyro_y = (Wire.read() << 8) | Wire.read();
  sensor_data.gyro_z = (Wire.read() << 8) | Wire.read();
}

// --- Sensor Reading Functions ---
void read_ir_sensors() {
  sensor_data.ir_states = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (digitalRead(IR_PINS[i])) {
      sensor_data.ir_states |= (1 << i);
    }
  }
}

int16_t read_ultrasonic(uint8_t trig_pin, uint8_t echo_pin) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  long duration = pulseIn(echo_pin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1;
  // Convert to mm: duration (microseconds) * speed of sound (343 m/s) / 2
  // Simplified: (duration * 343) / (2 * 1000) = duration * 0.1715
  // Using integer math for speed: (duration * 1715) / 10000
  return (duration * 1715L) / 10000L;
}

void read_sensors() {
  read_ir_sensors();
  read_mpu6050_data();
  
  uint32_t current_millis = millis();
  if (current_millis - last_us_read_time >= US_READ_INTERVAL_MS) {
    if (us_toggle) {
      sensor_data.us_front_mm = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO);
    } else {
      sensor_data.us_rear_mm = read_ultrasonic(US_REAR_TRIG, US_REAR_ECHO);
    }
    us_toggle = !us_toggle;
    last_us_read_time = current_millis;
  }
}

// --- Communication Functions ---
void send_sensor_data() {
  // Binary protocol for efficiency
  // Packet: 0xAA (Start) + 19 bytes data + 0x55 (End)
  Serial.write(0xAA);
  Serial.write((uint8_t*)&sensor_data, sizeof(sensor_data));
  Serial.write(0x55);
}

// --- Command Processing ---
void process_serial_command() {
  static uint8_t cmd_buffer[32];
  static uint8_t buffer_index = 0;
  static bool in_command = false;

  while (Serial.available()) {
    char c = Serial.read();
    
    if (!in_command) {
      if (c == 'C') { // Start of a command packet 'C'
        in_command = true;
        buffer_index = 0;
        cmd_buffer[buffer_index++] = c;
      } else if (c == 'P') { // PING command
        Serial.println("PONG");
      }
    } else {
      cmd_buffer[buffer_index++] = c;
      
      // Check for end of command '\n' or buffer full
      if (c == '\n' || buffer_index >= sizeof(cmd_buffer) - 1) {
        cmd_buffer[buffer_index] = '\0'; // Null-terminate
        in_command = false;
        
        // Process command
        if (cmd_buffer[1] == 'M') { // MOTOR,<speed>
          target_motor_speed = atoi((char*)cmd_buffer + 6); // Skip "CM: "
          target_motor_speed = constrain(target_motor_speed, -255, 255);
        } 
        else if (cmd_buffer[1] == 'S') { // SERVO,<angle>
          target_servo_angle = atoi((char*)cmd_buffer + 6); // Skip "CS: "
          target_servo_angle = constrain(target_servo_angle, 0, 180);
        }
        else if (cmd_buffer[1] == 'A') { // ACK_START
          run_started = true;
        }
        // Clear buffer
        buffer_index = 0;
      }
    }
  }
}

// --- Actuator Control ---
void update_motors() {
  if (target_motor_speed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    target_motor_speed = -target_motor_speed;
  }
  analogWrite(MOTOR_PWM, target_motor_speed);
  // If using L298N for 2 motors:
  // digitalWrite(MOTOR_IN3, !digitalRead(MOTOR_IN1));
  // digitalWrite(MOTOR_IN4, !digitalRead(MOTOR_IN2));
  // analogWrite(MOTOR_ENB, target_motor_speed);
}

void update_servo() {
  steering_servo.write(target_servo_angle);
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Pin setup
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_REAR_TRIG, OUTPUT);
  pinMode(US_REAR_ECHO, INPUT);
  
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  // pinMode(MOTOR_IN3, OUTPUT);
  // pinMode(MOTOR_IN4, OUTPUT);
  // pinMode(MOTOR_ENB, OUTPUT);
  
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  // digitalWrite(MOTOR_IN3, LOW);
  // digitalWrite(MOTOR_IN4, LOW);
  // analogWrite(MOTOR_ENB, 0);
  
  steering_servo.attach(SERVO_PIN);
  steering_servo.write(90);
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  bool mpu_ok = initialize_mpu6050();
  
  // Signal readiness
  if (mpu_ok) {
    Serial.println("NANO_READY_MPU_OK");
  } else {
    Serial.println("NANO_READY_MPU_FAIL");
  }
  
  last_sensor_send_time = millis();
  last_button_check_time = millis();
  last_us_read_time = millis();
}

void loop() {
  uint32_t current_millis = millis();

  // Check start button with debouncing
  if (current_millis - last_button_check_time >= BUTTON_DEBOUNCE_MS) {
    bool button_state = digitalRead(START_BUTTON_PIN);
    if (button_state == LOW && last_button_state == HIGH) {
      Serial.println("START");
      // Simple debounce by waiting for button release
      while (digitalRead(START_BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
    last_button_state = button_state;
    last_button_check_time = current_millis;
  }

  // Read sensors
  read_sensors();

  // Send sensor data at intervals
  if (current_millis - last_sensor_send_time >= SENSOR_SEND_INTERVAL_MS) {
    send_sensor_data();
    last_sensor_send_time = current_millis;
  }

  // Process serial commands
  process_serial_command();

  // Update actuators if run has started
  if (run_started) {
    update_motors();
    update_servo();
  }
  
  // Small delay to prevent overwhelming the loop
  delay(1);
}
