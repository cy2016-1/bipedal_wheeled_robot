#include <Arduino.h>
#include <Wire.h>
#include "SF_IMU.h"

SF_IMU mpu6050 = SF_IMU(Wire);
float roll,pitch,yaw;
float gyroX,gyroY,gyroZ;

void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  mpu6050.init();
}


void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  roll = mpu6050.angle[0];
  pitch = mpu6050.angle[1];
  yaw = mpu6050.angle[2];
  gyroX = mpu6050.gyro[0];
  gyroY = mpu6050.gyro[1];
  gyroZ =mpu6050.gyro[2];
  Serial.printf("%f,%f,%f,%f,%f,%f\n",roll,pitch,yaw,gyroX,gyroY,gyroZ);
  delay(100);
}