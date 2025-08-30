#include <Servo.h>
#include <Wire.h>
#include <NewPing.h> // For reliable ultrasonic sensor handling

// --- Hardware Configuration ---
#define MPU6050_ADDR 0x68
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Pin Definitions
const uint8_t START_BUTTON_PIN = A3;
const uint8_t IR_PINS[] = {6, 7, 8, 9}; // FL, FR, R45, L45
const uint8_t US_FRONT_TRIGGER = 2;
const uint8_t US_FRONT_ECHO = 3;
const uint8_t US_REAR_TRIGGER = 4;
const uint8_t US_REAR_ECHO = 5;
const uint8_t SERVO_PIN = 10;
const uint8_t MOTOR_PWM = 11;
const uint8_t MOTOR_IN1 = 12;
const uint8_t MOTOR_IN2 = 13;

// Ultrasonic Sensors (using NewPing for better reliability)
NewPing sonar_front(US_FRONT_TRIGGER, US_FRONT_ECHO, 200); // Max distance 200cm
NewPing sonar_rear(US_REAR_TRIGGER, US_REAR_ECHO, 200);

// --- System Configuration ---
const uint32_t SERIAL_BAUD = 500000;
const uint16_t SENSOR_SEND_INTERVAL_MS = 50;
const uint16_t BUTTON_DEBOUNCE_MS = 50;

// --- Data Structures ---
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

struct ControlCommand
{
    int16_t motor_speed;
    int16_t servo_angle;
};

// --- Global State Variables ---
volatile bool run_started = false;
volatile bool last_button_state = HIGH;
volatile uint32_t last_sensor_send_time = 0;
volatile uint32_t last_button_check_time = 0;
volatile bool start_button_flag = false;

SensorData sensor_data = {0};
Servo steering_servo;
volatile int16_t target_motor_speed = 0;
volatile uint8_t target_servo_angle = 90;

// --- WRO 2025 Free Round Specific ---
const uint8_t REQUIRED_LAPS = 3;
const uint8_t REQUIRED_TURNS = 4;
const int16_t TURN_THRESHOLD_DEGREES = 75;
const int16_t TURN_RESET_THRESHOLD_DEGREES = 20;
const uint8_t GYRO_INTEGRAL_WINDOW_SIZE = 5;
const uint16_t BASE_SPEED = 70;
const uint16_t MAX_SPEED = 100;
const uint16_t MIN_SPEED = 30;
const float STEERING_P_GAIN = 0.8;
const float STEERING_D_GAIN = 0.3;

volatile uint8_t lap_count = 0;
volatile uint8_t turn_count = 0;
volatile float last_turn_integral = 0.0;
volatile int16_t gyro_z_history[GYRO_INTEGRAL_WINDOW_SIZE] = {0};
volatile uint8_t gyro_history_index = 0;
volatile bool gyro_history_full = false;

volatile bool pi_control_override = false;
volatile bool initial_turn_direction_detected = false;
volatile int8_t learned_turn_direction = 0; // 1 for left, -1 for right

// --- MPU6050 Functions ---
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

    write_mpu6050_register(PWR_MGMT_1, 0x00); // Wake up
    delay(100);
    write_mpu6050_register(CONFIG, 0x03);       // DLPF: Gyro Bandwidth=44Hz
    write_mpu6050_register(SMPLRT_DIV, 0x00);   // Sample rate = 1kHz / (0 + 1) = 1000Hz
    write_mpu6050_register(GYRO_CONFIG, 0x00);  // Gyro full scale: +/- 250 deg/s
    write_mpu6050_register(ACCEL_CONFIG, 0x00); // Accel full scale: +/- 2g
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
    Wire.read(); // Skip temperature
    sensor_data.gyro_x = (Wire.read() << 8) | Wire.read();
    sensor_data.gyro_y = (Wire.read() << 8) | Wire.read();
    sensor_data.gyro_z = (Wire.read() << 8) | Wire.read();
}

// --- Sensor Reading Functions ---
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

void read_ultrasonic_sensors()
{
    // Use NewPing for more reliable readings
    sensor_data.us_front_mm = sonar_front.ping_cm() * 10; // Convert cm to mm
    if (sensor_data.us_front_mm == 0)
        sensor_data.us_front_mm = -1;                   // Timeout/error
    delay(29);                                          // Required delay between pings for NewPing to avoid interference
    sensor_data.us_rear_mm = sonar_rear.ping_cm() * 10; // Convert cm to mm
    if (sensor_data.us_rear_mm == 0)
        sensor_data.us_rear_mm = -1; // Timeout/error
    delay(29);
}

void read_sensors()
{
    read_ir_sensors();
    read_mpu6050_data();
    read_ultrasonic_sensors(); // Read both front and rear
}

// --- Communication Functions ---
void send_sensor_data()
{
    Serial.write(0xAA);
    Serial.write((uint8_t *)&sensor_data, sizeof(sensor_data));
    Serial.write(0x55);
}

void send_status_packet()
{
    struct StatusPacket
    {
        uint8_t header; // 0xBB
        uint8_t lap_count;
        uint8_t turn_count;
        int8_t learned_turn_dir;
        uint8_t control_mode; // 0 = Nano Auto, 1 = Pi Override
        uint8_t footer;       // 0xCC
    } status_packet;

    status_packet.header = 0xBB;
    status_packet.lap_count = lap_count;
    status_packet.turn_count = turn_count;
    status_packet.learned_turn_dir = learned_turn_direction;
    status_packet.control_mode = pi_control_override ? 1 : 0;
    status_packet.footer = 0xCC;

    Serial.write((uint8_t *)&status_packet, sizeof(status_packet));
}

// --- Lap and Turn Counting (IMU-based) ---
void update_lap_and_turn_count()
{
    const float GYRO_SENSITIVITY_DPS_PER_LSB = 131.0;
    float gyro_z_dps = (float)sensor_data.gyro_z / GYRO_SENSITIVITY_DPS_PER_LSB;

    gyro_z_history[gyro_history_index] = (int16_t)gyro_z_dps;
    gyro_history_index = (gyro_history_index + 1) % GYRO_INTEGRAL_WINDOW_SIZE;
    if (gyro_history_index == 0)
        gyro_history_full = true;

    uint8_t num_samples = gyro_history_full ? GYRO_INTEGRAL_WINDOW_SIZE : gyro_history_index;
    float integral_sum = 0.0;
    for (uint8_t i = 0; i < num_samples; i++)
    {
        integral_sum += (float)gyro_z_history[i];
    }
    float current_integral = integral_sum * (0.05 / (float)num_samples); // dt approx 50ms / 1000Hz rate
    float turn_angle = current_integral - last_turn_integral;
    last_turn_integral = current_integral;

    if (abs(turn_angle) > TURN_THRESHOLD_DEGREES && turn_count < REQUIRED_TURNS)
    {
        turn_count++;
        Serial.print("TURN_DETECTED:");
        Serial.println(turn_count);

        if (!initial_turn_direction_detected)
        {
            learned_turn_direction = turn_angle > 0 ? 1 : -1;
            initial_turn_direction_detected = true;
            Serial.print("LEARNED_TURN_DIR:");
            Serial.println(learned_turn_direction);
        }
    }

    if (abs(turn_angle) < TURN_RESET_THRESHOLD_DEGREES && turn_count > 0)
    {
        // State maintained
    }

    if (turn_count >= REQUIRED_TURNS)
    {
        lap_count++;
        turn_count = 0;
        last_turn_integral = 0.0;
        memset(gyro_z_history, 0, sizeof(gyro_z_history));
        gyro_history_index = 0;
        gyro_history_full = false;
        Serial.print("LAP_COMPLETED:");
        Serial.println(lap_count);

        if (lap_count >= REQUIRED_LAPS)
        {
            Serial.println("ALL_LAPS_COMPLETED");
        }
    }
}

// --- Control Logic (Simple Line Following) ---
ControlCommand calculate_control_nano_autonomous()
{
    ControlCommand cmd = {0, 90};
    int16_t error = 0;

    bool ir_fl = sensor_data.ir_states & (1 << 0);
    bool ir_fr = sensor_data.ir_states & (1 << 1);
    bool ir_r45 = sensor_data.ir_states & (1 << 2);
    bool ir_l45 = sensor_data.ir_states & (1 << 3);

    if (ir_fl && ir_fr)
    {
        error = 0;
    }
    else if (ir_fl)
    {
        error = -30;
    }
    else if (ir_fr)
    {
        error = 30;
    }
    else if (ir_l45 && !ir_r45)
    {
        error = -15;
    }
    else if (ir_r45 && !ir_l45)
    {
        error = 15;
    }
    else
    {
        error = 0;
    }

    int16_t steering_adjustment = (int16_t)(STEERING_P_GAIN * (float)error);
    cmd.servo_angle = 90 + steering_adjustment;
    cmd.servo_angle = constrain(cmd.servo_angle, 45, 135);

    cmd.motor_speed = BASE_SPEED;
    int16_t steering_deviation = abs(cmd.servo_angle - 90);
    if (steering_deviation > 25)
    {
        cmd.motor_speed = max(MIN_SPEED, BASE_SPEED - steering_deviation * 0.5);
    }

    if (lap_count >= REQUIRED_LAPS)
    {
        cmd.motor_speed = 0;
        cmd.servo_angle = 90;
    }

    return cmd;
}

void apply_control(ControlCommand cmd)
{
    target_motor_speed = cmd.motor_speed;
    target_servo_angle = cmd.servo_angle;
}

// --- Actuator Control ---
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
    analogWrite(MOTOR_PWM, constrain(target_motor_speed, 0, 255));
}

void update_servo()
{
    steering_servo.write(constrain(target_servo_angle, 0, 180));
}

// --- Serial Command Processing ---
void process_serial_command()
{
    static uint8_t buffer[32];
    static uint8_t buffer_index = 0;

    while (Serial.available())
    {
        char c = Serial.read();
        if (buffer_index < sizeof(buffer) - 1)
        {
            buffer[buffer_index++] = c;
            if (c == '\n')
            {
                buffer[buffer_index] = '\0';
                buffer_index = 0;

                if (strncmp((char *)buffer, "CMD:", 4) == 0)
                {
                    int16_t new_speed = atoi((char *)buffer + 4);
                    int16_t new_angle = atoi(strchr((char *)buffer, ',') + 1);
                    if (!pi_control_override)
                    {
                        Serial.println("ERR_NOT_IN_PI_MODE");
                    }
                    else
                    {
                        target_motor_speed = new_speed;
                        target_servo_angle = new_angle;
                        Serial.println("ACK_CMD");
                    }
                }
                else if (strcmp((char *)buffer, "REQ_STATUS\n") == 0)
                {
                    send_status_packet();
                }
                else if (strcmp((char *)buffer, "ENABLE_PI_CONTROL\n") == 0)
                {
                    pi_control_override = true;
                    target_motor_speed = 0;
                    target_servo_angle = 90;
                    Serial.println("ACK_PI_CONTROL_ENABLED");
                }
                else if (strcmp((char *)buffer, "DISABLE_PI_CONTROL\n") == 0)
                {
                    pi_control_override = false;
                    Serial.println("ACK_PI_CONTROL_DISABLED");
                }
                else if (strncmp((char *)buffer, "PING", 4) == 0)
                {
                    Serial.println("PONG");
                }
            }
        }
        else
        {
            buffer_index = 0;
        }
    }
}

// --- Setup & Loop ---
void setup()
{
    Serial.begin(SERIAL_BAUD);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(IR_PINS[i], INPUT);
    }
    pinMode(US_FRONT_TRIGGER, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_REAR_TRIGGER, OUTPUT);
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
                delay(10);
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
        update_lap_and_turn_count();

        if (pi_control_override)
        {
            // In this mode, Nano only executes commands received via serial (CMD:)
            // Sensor data is still sent for Pi's use.
        }
        else
        {
            // Autonomous mode: Nano makes all decisions
            ControlCommand cmd = calculate_control_nano_autonomous();
            apply_control(cmd);
        }

        update_motors();
        update_servo();
    }
    else
    {
        if (start_button_flag)
        {
            Serial.println("START");
            start_button_flag = false;
        }
    }

    delay(10);
}