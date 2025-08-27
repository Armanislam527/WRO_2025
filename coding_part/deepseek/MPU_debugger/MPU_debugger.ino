#include <Wire.h>

#define MPU6050_ADDR 0x68
#define WHO_AM_I 0x75

// Register addresses
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.println("=== MPU6050 Clone Debug ===");
  Serial.print("WHO_AM_I: 0x");
  Serial.println(readByte(MPU6050_ADDR, WHO_AM_I), HEX);
  
  // Initialize MPU6050
  writeByte(MPU6050_ADDR, PWR_MGMT_1, 0x00); // Wake up device
  delay(100);
  
  Serial.println("Device initialized successfully!");
  Serial.println("Accel(g)\tGyro(°/s)");
  Serial.println("X\tY\tZ\tX\tY\tZ");
}

void loop() {
  // Read and display sensor data
  int16_t ax, ay, az, gx, gy, gz;
  
  readAccelGyro(ax, ay, az, gx, gy, gz);
  
  // Convert to meaningful units
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
  
  // Print data
  Serial.print(accelX, 2); Serial.print("\t");
  Serial.print(accelY, 2); Serial.print("\t");
  Serial.print(accelZ, 2); Serial.print("\t");
  
  Serial.print(gyroX, 1); Serial.print("\t");
  Serial.print(gyroY, 1); Serial.print("\t");
  Serial.println(gyroZ, 1);
  
  delay(500);
}

void readAccelGyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  // Temperature (skip)
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

byte readByte(byte address, byte reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (byte)1);
  return Wire.read();
}

void writeByte(byte address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
