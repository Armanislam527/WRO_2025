#include <Servo.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19

const uint8_t START_BUTTON_PIN = A3;
const uint8_t IR_PINS[] = {6, 7, 8, 9};
const uint8_t US_FRONT_TRIG = 2;
const uint8_t US_FRONT_ECHO = 3;
const uint8_t US_REAR_TRIG = 4;
const uint8_t US_REAR_ECHO = 5;
const uint8_t SERVO_PIN = 10;
const uint8_t MOTOR_PWM = 11;
const uint8_t MOTOR_IN1 = 12;
const uint8_t MOTOR_IN2 = 13;
const uint8_t MOTOR_IN3 = A0;
const uint8_t MOTOR_IN4 = A1;
const uint8_t MOTOR_ENB = A2;

const uint32_t SERIAL_BAUD = 500000;
const uint16_t SENSOR_SEND_INTERVAL_MS = 50;
const uint16_t BUTTON_DEBOUNCE_MS = 50;
const uint16_t US_READ_INTERVAL_MS = 25;

struct SensorData
{
  uint8_t ir_states;
  int16_t us_front_mm;
  int16_t us_rear_mm;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

volatile bool run_started = false;
volatile bool last_button_state = HIGH;
volatile uint32_t last_sensor_send_time = 0;
volatile uint32_t last_button_check_time = 0;
volatile uint32_t last_us_read_time = 0;
volatile bool us_toggle = false;
volatile bool start_button_flag = false;

SensorData sensor_data = {0};
Servo steering_servo;

volatile int16_t target_motor_speed = 0;
volatile uint8_t target_servo_angle = 90;

bool initialize_mpu6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1);
  uint8_t who_am_i = Wire.read();
  if (who_am_i != 0x68 && who_am_i != 0x70)
  {
    return false;
  }
  write_mpu6050_register(PWR_MGMT_1, 0x00);
  delay(100);
  write_mpu6050_register(CONFIG, 0x00);
  write_mpu6050_register(SMPLRT_DIV, 0x07);
  write_mpu6050_register(GYRO_CONFIG, 0x00);
  write_mpu6050_register(ACCEL_CONFIG, 0x00);
  delay(100);
  return true;
}

void write_mpu6050_register(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void read_mpu6050_data()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  sensor_data.accel_x = (Wire.read() << 8) | Wire.read();
  sensor_data.accel_y = (Wire.read() << 8) | Wire.read();
  sensor_data.accel_z = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  sensor_data.gyro_x = (Wire.read() << 8) | Wire.read();
  sensor_data.gyro_y = (Wire.read() << 8) | Wire.read();
  sensor_data.gyro_z = (Wire.read() << 8) | Wire.read();
}

void read_ir_sensors()
{
  sensor_data.ir_states = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    if (digitalRead(IR_PINS[i]))
    {
      sensor_data.ir_states |= (1 << i);
    }
  }
  if (start_button_flag)
  {
    sensor_data.ir_states |= (1 << 7);
    start_button_flag = false;
  }
}

int16_t read_ultrasonic(uint8_t trig_pin, uint8_t echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  long duration = pulseIn(echo_pin, HIGH, 30000);
  if (duration == 0)
    return -1;
  return (duration * 1715L) / 10000L;
}

void read_sensors()
{
  read_ir_sensors();
  read_mpu6050_data();
  uint32_t current_millis = millis();
  if (current_millis - last_us_read_time >= US_READ_INTERVAL_MS)
  {
    if (us_toggle)
    {
      sensor_data.us_front_mm = read_ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO);
    }
    else
    {
      sensor_data.us_rear_mm = read_ultrasonic(US_REAR_TRIG, US_REAR_ECHO);
    }
    us_toggle = !us_toggle;
    last_us_read_time = current_millis;
  }
}

void send_sensor_data()
{
  Serial.write(0xAA);
  Serial.write((uint8_t *)&sensor_data, sizeof(sensor_data));
  Serial.write(0x55);
}

void process_serial_command()
{
  static uint8_t cmd_buffer[32];
  static uint8_t buffer_index = 0;
  static bool in_command = false;

  while (Serial.available())
  {
    char c = Serial.read();
    if (!in_command)
    {
      if (c == 'C')
      {
        in_command = true;
        buffer_index = 0;
        cmd_buffer[buffer_index++] = c;
      }
      else if (c == 'P')
      {
        Serial.println("PONG");
      }
    }
    else
    {
      cmd_buffer[buffer_index++] = c;
      if (c == '\n' || buffer_index >= sizeof(cmd_buffer) - 1)
      {
        cmd_buffer[buffer_index] = '\0';
        in_command = false;
        if (cmd_buffer[1] == 'M')
        {
          target_motor_speed = atoi((char *)cmd_buffer + 6);
          target_motor_speed = constrain(target_motor_speed, -255, 255);
        }
        else if (cmd_buffer[1] == 'S')
        {
          target_servo_angle = atoi((char *)cmd_buffer + 6);
          target_servo_angle = constrain(target_servo_angle, 0, 180);
        }
        else if (cmd_buffer[1] == 'A')
        {
          run_started = true;
        }
        buffer_index = 0;
      }
    }
  }
}

void update_motors()
{
  if (target_motor_speed >= 0)
  {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    target_motor_speed = -target_motor_speed;
  }
  analogWrite(MOTOR_PWM, target_motor_speed);
}

void update_servo()
{
  steering_servo.write(target_servo_angle);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 4; i++)
  {
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
  steering_servo.write(90);
  Wire.begin();
  Wire.setClock(400000);
  bool mpu_ok = initialize_mpu6050();
  if (mpu_ok)
  {
    Serial.println("NANO_READY_MPU_OK");
  }
  else
  {
    Serial.println("NANO_READY_MPU_FAIL");
  }
  last_sensor_send_time = millis();
  last_button_check_time = millis();
  last_us_read_time = millis();
}

void loop()
{
  uint32_t current_millis = millis();
  if (current_millis - last_button_check_time >= BUTTON_DEBOUNCE_MS)
  {
    bool button_state = digitalRead(START_BUTTON_PIN);
    if (button_state == LOW && last_button_state == HIGH)
    {
      start_button_flag = true;
      while (digitalRead(START_BUTTON_PIN) == LOW)
      {
        delay(10);
      }
    }
    last_button_state = button_state;
    last_button_check_time = current_millis;
  }
  read_sensors();
  if (current_millis - last_sensor_send_time >= SENSOR_SEND_INTERVAL_MS)
  {
    send_sensor_data();
    last_sensor_send_time = current_millis;
  }
  process_serial_command();
  if (run_started)
  {
    update_motors();
    update_servo();
  }
  delay(1);
}