#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"



SF_Servo servos = SF_Servo(Wire);
SF_IMU mpu6050 = SF_IMU(Wire);
SF_BLDC motors = SF_BLDC(Serial2);

PIDController PID_VEL{0.2,0,0,1000,50};

void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1);
  servos.setPWM(4, 0, val2);
  servos.setPWM(5, 0, val3);
  servos.setPWM(6, 0, val4);
}

uint16_t height0[4] = {275,168,244,178};
uint16_t height1[4] = {294,152,260,162};
uint16_t height2[4] = {320,170,265,110};
float kp1 = 0.38;//该P值需要精调节
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;

float pitch;
SF_BLDC_DATA  BLDCData;
int M0Dir,M1Dir;
float targetSpeed;

void read(){
  static String receivedChars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    receivedChars += inChar;
    if (inChar == '\n')
    {
      command = receivedChars;
      const char *d = command.c_str();
      sscanf(d,"%f", &targetSpeed);
      receivedChars = "";
    }
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
}

uint8_t loopCnt;


void loop() {

  read();
  targetSpeed = 0;
  BLDCData = motors.getBLDCData();
  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2;
  mpu6050.update();
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]);
  pitch = mpu6050.angle[0];

  float targetAngle = PID_VEL(targetSpeed - speedAvg);
  float torque = kp1*(targetAngle - pitch);

  motors.setTargets(M0Dir*torque, M1Dir*torque);

  if(loopCnt>=100){
    Serial.printf("status: %.2f,%.2f,%.2f,%.2f\n",speedAvg,pitch,targetAngle,torque);
    loopCnt=0;
  }
  loopCnt++;
}
