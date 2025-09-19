// config.h
// Pin definitions and vehicle configuration constants

#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------
// Pin Definitions
// -----------------------------

// Ultrasonic Sensors
#define US_FRONT_TRIG_PIN 3
#define US_FRONT_ECHO_PIN 4
#define US_RIGHT_TRIG_PIN 5
#define US_RIGHT_ECHO_PIN 6
#define US_BACK_TRIG_PIN 7
#define US_BACK_ECHO_PIN 8
#define US_LEFT_TRIG_PIN 9
#define US_LEFT_ECHO_PIN 10

// Motor Driver (L298N)
#define MOTOR_ENA_PIN 12
#define MOTOR_IN1_PIN 13
#define MOTOR_IN2_PIN A0

// Servo Motor
#define SERVO_PIN 11

// IMU (MPU6050) - Uses I2C
// SDA -> A4
// SCL -> A5

// Start Button
#define START_BUTTON_PIN 2

// Status LED (optional, for debugging)
#define STATUS_LED_PIN A1

// -----------------------------
// Vehicle Configuration
// -----------------------------

// Motor configuration
#define MOTOR_MIN_SPEED 0
#define MOTOR_MAX_SPEED 255
#define MOTOR_STOP 0

// Servo configuration
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER_ANGLE 90

// Ultrasonic sensor configuration
#define US_MAX_DISTANCE 200     // Maximum distance we want to measure (in cm)
#define US_MEASURE_INTERVAL 100 // Time between measurements (in milliseconds)

// Watchdog configuration
#define WATCHDOG_TIMEOUT 500 // Time in ms to wait for heartbeat before emergency stop

// Communication
#define SERIAL_BAUD_RATE 115200
#define COMMUNICATION_TIMEOUT 1000 // Time in ms to wait for valid packet

#endif // CONFIG_H