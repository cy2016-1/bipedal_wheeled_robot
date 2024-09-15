#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

SF_Servo servos = SF_Servo(Wire);

void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1);
  servos.setPWM(4, 0, val2);
  servos.setPWM(5, 0, val3);
  servos.setPWM(6, 0, val4);
}

unsigned long currTime,prevTime;
void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  servos.init();
  prevTime = millis();
}

void loop() {
  setRobotHeight(275,168,244,178);
  delay(5000);
  setRobotHeight(294,154,260,162);
  delay(5000);
  setRobotHeight(313,139,277,146);
  delay(5000);
}