#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"

SF_Servo servos = SF_Servo(Wire);
SF_IMU mpu6050 = SF_IMU(Wire);
SF_BLDC motors = SF_BLDC(Serial2);

void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1);
  servos.setPWM(4, 0, val2);
  servos.setPWM(5, 0, val3);
  servos.setPWM(6, 0, val4);
}
float pitch;
int command;
float target,kp1,kp2,kp3;
int M0Dir,M1Dir;

uint16_t height0[4] = {275,168,244,178};
uint16_t height1[4] = {294,152,260,162};
uint16_t height2[4] = {320,170,265,110};

void read(){
  if (Serial.available() > 0) {
    command = Serial.parseInt();
  }
}

void setup() {
  Serial.begin(921600);
  Wire.begin(1, 2, 400000UL);
  servos.init();
  mpu6050.init();
  motors.init();
  motors.setModes(4,4);
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]);

  M0Dir = 1;
  M1Dir = -1;
  kp1 = 0.38;//该P值需要精调节
  kp2 = 0.4;
  kp3 = 0.45;
}

void loop() {

  mpu6050.update();
  pitch = mpu6050.angle[0];

  read();
  switch (command) {
    case 1:
      setRobotHeight(height0[0],height0[1],height0[2],height0[3]);
      target = kp1*(0-pitch);
      break;
      
    case 2:
      setRobotHeight(height1[0],height1[1],height1[2],height1[3]);
      target = kp3*(0-pitch);
      break;
      
    case 3:
      setRobotHeight(height2[0],height2[1],height2[2],height2[3]);
      target = kp3*(0-pitch);
      break;
      
    default:
      Serial.println("无效指令");
      break;
  }

  motors.setTargets(M0Dir*target, M1Dir*target);
}


