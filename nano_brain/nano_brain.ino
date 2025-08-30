// Include necessary libraries
#include <Servo.h>
#include <Wire.h>
// Include the Electronic Cats MPU6050 library
// Make sure you have installed the correct library:
// Arduino IDE -> Tools -> Manage Libraries... -> Search for "MPU6050 by Electronic Cats"
#include <MPU6050.h>

// --- Hardware Configuration ---
#define MPU6050_ADDR         0x68
// Register addresses (these are standard for MPU6050)
#define WHO_AM_I             0x75
#define PWR_MGMT_1           0x6B
#define SMPLRT_DIV           0x19
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43

// Pin Definitions
const uint8_t START_BUTTON_PIN = A3; // D17
const uint8_t IR_PINS[] = {6, 7, 8, 9}; // FL, FR, R45, L45
const uint8_t US_FRONT_TRIG = 2;
const uint8_t US_FRONT_ECHO = 3;
const uint8_t US_REAR_TRIG = 4;
const uint8_t US_REAR_ECHO = 5;
const uint8_t SERVO_PIN = 10;          // D10
const uint8_t MOTOR_PWM = 11;          // D11 (ENA)
const uint8_t MOTOR_IN1 = 12;          // D12
const uint8_t MOTOR_IN2 = 13;          // D13
// L298N IN3, IN4, ENB are not used for single motor drive axle in this basic setup

// --- System Configuration ---
const uint32_t SERIAL_BAUD = 115200; // Slower baud rate for easier PC monitoring
const uint16_t SENSOR_READ_INTERVAL_MS = 100; // Read/send sensor data every 100ms
const uint16_t BUTTON_DEBOUNCE_MS = 50;
const uint16_t CONTROL_LOOP_INTERVAL_MS = 50; // Control loop runs every 50ms
const uint16_t US_READ_INTERVAL_MS = 50; // Alternate ultrasonic reads

// --- WRO 2025 Open Challenge Specifics ---
const uint8_t REQUIRED_LAPS = 3;
const uint8_t REQUIRED_TURNS_PER_LAP = 4;
const int16_t TURN_MIN_ANGLE_DEGREES = 75;  // Degrees to count as a significant turn
const int16_t TURN_RESET_ANGLE_DEGREES = 20; // Degrees below which turn state resets
const uint8_t GYRO_INTEGRAL_WINDOW_SIZE = 5; // For smoothing gyro integration

// --- Control Parameters (Tune these!) ---
const int16_t BASE_SPEED = 70; // Base motor speed (0-255)
const int16_t MAX_SPEED = 90;
const int16_t MIN_SPEED = 30;
const float STEERING_P_GAIN = 0.8; // Proportional gain for steering based on line error
const float STEERING_D_GAIN = 0.2; // Derivative gain (simplified)

// --- Global State Variables ---
volatile bool run_started = false;
volatile bool last_button_state = HIGH;
volatile uint32_t last_sensor_read_time = 0;
volatile uint32_t last_button_check_time = 0;
volatile uint32_t last_control_loop_time = 0;
volatile uint32_t last_us_read_time = 0;
volatile bool us_toggle = false; // Toggle between front/rear US reads

// --- Sensor Data ---
struct SensorData {
  uint8_t ir_states;       // Bit-packed IR states (bit 0: FL, 1: FR, 2: R45, 3: L45)
  int16_t us_front_mm;     // Front ultrasonic distance (mm)
  int16_t us_rear_mm;      // Rear ultrasonic distance (mm)
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

SensorData sensor_data = {0};

// --- IMU Data for Lap Counting ---
MPU6050 mpu; // Electronic Cats MPU6050 object
bool imu_ready = false;
float gyro_z_history[GYRO_INTEGRAL_WINDOW_SIZE] = {0};
uint8_t gyro_history_index = 0;
bool gyro_history_full = false;
float last_turn_integral = 0.0;
uint8_t turn_count = 0;
uint8_t lap_count = 0;

// --- Actuator Control ---
Servo steering_servo;
int16_t target_motor_speed = 0;
int16_t target_servo_angle = 90; // Center

// --- MPU6050 Helper Functions ---
bool initialize_mpu6050_electroniccats() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1);
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    if (who_am_i != 0x68 && who_am_i != 0x70) { // 0x70 for some clones
      Serial.print("MPU6050 WHO_AM_I mismatch: 0x");
      Serial.println(who_am_i, HEX);
      return false;
    }
  } else {
    Serial.println("MPU6050 WHO_AM_I read failed");
    return false;
  }

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Reset sleep mode
  if (Wire.endTransmission(true) != 0) {
      Serial.println("MPU6050 wake-up failed");
      return false;
  }
  delay(100); // Let it wake up

  // Configure (optional, defaults might be okay)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x00); // Disable FSYNC, set DLPF to 0 (260Hz bandwidth)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(0x07); // Sample rate = 1kHz / (7 + 1) = 125Hz
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00); // Gyro full scale: +/- 250 deg/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00); // Accel full scale: +/- 2g
  Wire.endTransmission(true);

  delay(100); // Stabilize
  Serial.println("MPU6050 basic init sequence completed");
  return true;
}

// Function to read raw data from MPU6050 registers
void read_mpu6050_raw_data() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14); // Read 14 bytes (Accel XYZ, Temp, Gyro XYZ)

  if (Wire.available() == 14) {
    sensor_data.accel_x = (Wire.read() << 8) | Wire.read();
    sensor_data.accel_y = (Wire.read() << 8) | Wire.read();
    sensor_data.accel_z = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip Temperature
    sensor_data.gyro_x = (Wire.read() << 8) | Wire.read();
    sensor_data.gyro_y = (Wire.read() << 8) | Wire.read();
    sensor_data.gyro_z = (Wire.read() << 8) | Wire.read();
  } else {
    // Handle error or set data to zero
    sensor_data.accel_x = 0;
    sensor_data.accel_y = 0;
    sensor_data.accel_z = 0;
    sensor_data.gyro_x = 0;
    sensor_data.gyro_y = 0;
    sensor_data.gyro_z = 0;
    // Serial.println("MPU6050 data read incomplete");
  }
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

  long duration = pulseIn(echo_pin, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) {
    return -1; // Timeout or error
  }
  // Convert to mm: duration (microseconds) * speed of sound (343 m/s) / 2
  // Simplified: (duration * 343) / (2 * 1000) = duration * 0.1715
  // Using integer math: (duration * 1715) / 10000
  return (duration * 1715L) / 10000L;
}

void read_sensors() {
  read_ir_sensors();
  read_mpu6050_raw_data(); // Use our custom raw read function

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

// --- IMU-based Lap Counting ---
void update_lap_count_using_imu() {
  // Convert raw gyro Z rate to degrees/second
  // MPU6050 default sensitivity: 131 LSB/degree/second for +/- 250 deg/s range
  const float GYRO_SENSITIVITY_DPS_PER_LSB = 131.0;
  float gyro_z_dps = sensor_data.gyro_z / GYRO_SENSITIVITY_DPS_PER_LSB; // Use sensor_data.gyro_z

  // Add current rate to history for smoothing
  gyro_z_history[gyro_history_index] = gyro_z_dps;
  gyro_history_index = (gyro_history_index + 1) % GYRO_INTEGRAL_WINDOW_SIZE;
  if (gyro_history_index == 0) gyro_history_full = true;

  // Simple integral approximation (sum over time window)
  uint8_t num_samples = gyro_history_full ? GYRO_INTEGRAL_WINDOW_SIZE : gyro_history_index;
  float dt = (CONTROL_LOOP_INTERVAL_MS / 1000.0); // Convert ms to seconds
  float current_integral = 0;
  for (uint8_t i = 0; i < num_samples; i++) {
    current_integral += gyro_z_history[i] * dt;
  }
  if (num_samples > 0) {
      current_integral /= num_samples; // Average
  }

  float turn_angle = current_integral - last_turn_integral;
  last_turn_integral = current_integral;

  // Serial.print("Gyro Z DPS: "); Serial.print(gyro_z_dps);
  // Serial.print(", Integral: "); Serial.print(current_integral);
  // Serial.print(", Turn Angle: "); Serial.println(turn_angle);

  // Simple turn detection logic
  if (abs(turn_angle) > TURN_MIN_ANGLE_DEGREES && turn_count < REQUIRED_TURNS_PER_LAP) {
    turn_count++;
    Serial.print(">>> TURN DETECTED! Count: ");
    Serial.print(turn_count);
    Serial.print("/");
    Serial.println(REQUIRED_TURNS_PER_LAP);
  }

  // Reset turn detection if rotation slows down significantly
  if (abs(turn_angle) < TURN_RESET_ANGLE_DEGREES && turn_count > 0) {
     // State maintained or reset turn count if needed
  }

  // Check if a lap is completed
  if (turn_count >= REQUIRED_TURNS_PER_LAP) {
    lap_count++;
    turn_count = 0;
    last_turn_integral = 0.0;
    // Reset gyro history
    for (uint8_t i = 0; i < GYRO_INTEGRAL_WINDOW_SIZE; i++) {
        gyro_z_history[i] = 0;
    }
    gyro_history_index = 0;
    gyro_history_full = false;
    Serial.print(">>> LAP COMPLETED! Lap Count: ");
    Serial.print(lap_count);
    Serial.print("/");
    Serial.println(REQUIRED_LAPS);

    // Stop after 3 laps
    if (lap_count >= REQUIRED_LAPS) {
        target_motor_speed = 0;
        target_servo_angle = 90;
        Serial.println("*** THREE LAPS COMPLETED! STOPPING ***");
    }
  }
}

// --- Line Following Logic ---
int16_t calculate_line_following_error() {
  bool ir_fl = sensor_data.ir_states & (1 << 0);
  bool ir_fr = sensor_data.ir_states & (1 << 1);
  bool ir_r45 = sensor_data.ir_states & (1 << 2);
  bool ir_l45 = sensor_data.ir_states & (1 << 3);

  int16_t error = 0;
  // Basic line following using front IR sensors
  if (ir_fl && ir_fr) {
    error = 0; // On track center
  } else if (ir_fl) {
    error = -30; // Too far right, steer left
  } else if (ir_fr) {
    error = 30;  // Too far left, steer right
  } else if (ir_l45 && !ir_r45) {
     error = -15; // Slight right correction
  } else if (ir_r45 && !ir_l45) {
     error = 15;  // Slight left correction
  } else {
    error = 0; // Lost line? Maintain last known good direction or stop?
    // You might want a recovery strategy here
  }
  return error;
}

// --- Actuator Control Functions ---
void update_motors() {
  if (target_motor_speed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    target_motor_speed = -target_motor_speed;
  }
  analogWrite(MOTOR_PWM, constrain(target_motor_speed, 0, 255));
}

void update_servo() {
  steering_servo.write(constrain(target_servo_angle, 0, 180));
}

// --- Debugging Output ---
void print_sensor_data() {
  Serial.print("Sensors -> IR(FL,FR,R45,L45): ");
  Serial.print((sensor_data.ir_states >> 0) & 1); Serial.print(",");
  Serial.print((sensor_data.ir_states >> 1) & 1); Serial.print(",");
  Serial.print((sensor_data.ir_states >> 2) & 1); Serial.print(",");
  Serial.print((sensor_data.ir_states >> 3) & 1);
  Serial.print(" | US(F,R): ");
  Serial.print(sensor_data.us_front_mm); Serial.print("mm, ");
  Serial.print(sensor_data.us_rear_mm); Serial.print("mm");
  Serial.print(" | IMU(Accel X,Y,Z): ");
  Serial.print(sensor_data.accel_x); Serial.print(",");
  Serial.print(sensor_data.accel_y); Serial.print(",");
  Serial.print(sensor_data.accel_z);
  Serial.print(" | IMU(Gyro X,Y,Z): ");
  Serial.print(sensor_data.gyro_x); Serial.print(",");
  Serial.print(sensor_data.gyro_y); Serial.print(",");
  Serial.print(sensor_data.gyro_z);
  Serial.println();
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial); // Wait for Serial Monitor (useful for debugging)

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
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);

  steering_servo.attach(SERVO_PIN);
  steering_servo.write(target_servo_angle);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  // Initialize MPU6050 using our custom function
  if (initialize_mpu6050_electroniccats()) {
    imu_ready = true;
    Serial.println("MPU6050 initialized successfully (Electronic Cats lib style).");
  } else {
    imu_ready = false;
    Serial.println("ERROR: Failed to initialize MPU6050!");
  }

  Serial.println("Nano Brain Ready for Open Challenge. Press Start Button.");
  last_sensor_read_time = millis();
  last_button_check_time = millis();
  last_control_loop_time = millis();
  last_us_read_time = millis();
}

void loop() {
  uint32_t current_millis = millis();

  // --- 1. Check Start Button ---
  if (current_millis - last_button_check_time >= BUTTON_DEBOUNCE_MS) {
    bool button_state = digitalRead(START_BUTTON_PIN);
    if (button_state == LOW && last_button_state == HIGH) {
      Serial.println("START button pressed!");
      run_started = true;
      // Simple debounce by waiting for button release
      while (digitalRead(START_BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
    last_button_state = button_state;
    last_button_check_time = current_millis;
  }

  // --- 2. Read Sensors Periodically ---
  if (current_millis - last_sensor_read_time >= SENSOR_READ_INTERVAL_MS) {
    read_sensors();
    print_sensor_data(); // Print data for monitoring
    last_sensor_read_time = current_millis;
  }

  // --- 3. Main Control Logic (Runs when started) ---
  if (run_started && lap_count < REQUIRED_LAPS) { // Only run control if not finished
    if (current_millis - last_control_loop_time >= CONTROL_LOOP_INTERVAL_MS) {

      // --- a. Update Lap Count using IMU ---
      if (imu_ready) {
        update_lap_count_using_imu();
      }

      // --- b. Basic Line Following using IR Sensors ---
      int16_t line_error = calculate_line_following_error();
      // Simplified PD control for steering
      int16_t steering_adjustment = (int16_t)(STEERING_P_GAIN * line_error); // Simplified, no D-term yet
      target_servo_angle = 90 + steering_adjustment;
      target_servo_angle = constrain(target_servo_angle, 45, 135);

      // --- c. Adjust Speed based on Steering ---
      target_motor_speed = BASE_SPEED;
      int16_t steering_deviation = abs(target_servo_angle - 90);
      if (steering_deviation > 25) {
        target_motor_speed = max(MIN_SPEED, BASE_SPEED - steering_deviation * 0.5);
      }

      // --- d. Apply Commands to Actuators ---
      update_motors();
      update_servo();

      // --- e. Print Control Info ---
      Serial.print("Control -> Error: ");
      Serial.print(line_error);
      Serial.print(", Steer Adj: ");
      Serial.print(steering_adjustment);
      Serial.print(", Servo: ");
      Serial.print(target_servo_angle);
      Serial.print(", Speed: ");
      Serial.print(target_motor_speed);
      Serial.print(", Lap: ");
      Serial.print(lap_count);
      Serial.print("/");
      Serial.print(REQUIRED_LAPS);
      Serial.print(", Turn: ");
      Serial.println(turn_count);

      last_control_loop_time = current_millis;
    }
  } else if (run_started && lap_count >= REQUIRED_LAPS) {
    // Ensure vehicle stops after 3 laps
    target_motor_speed = 0;
    target_servo_angle = 90;
    update_motors();
    update_servo();
  } else {
    // Ensure motors are stopped if not started
    target_motor_speed = 0;
    update_motors();
  }

  delay(5); // Small delay to prevent overwhelming the loop
}
