#include "SF_Servo.h"

SF_Servo::SF_Servo(TwoWire &i2c)
    :  _i2c(&i2c) {}

void SF_Servo::init(){
    _i2c->begin();
    reset();

    setPWMFreq(50);

    enable();
}

void SF_Servo::setPWMFreq(float freq){
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500;  // 限制为3052=50MHz/(4*4096)
    
    float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = readFromPCA(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // sleep
    writeToPCA(PCA9685_MODE1, newmode);                              // go to sleep
    writeToPCA(PCA9685_PRESCALE, prescale);                          // 设置预分频器
    writeToPCA(PCA9685_MODE1, oldmode);
    delay(5);
    // 将MODE1寄存器设置为自动递增
    writeToPCA(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void SF_Servo::enable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, HIGH);
}

void SF_Servo::disable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, LOW);
}

void SF_Servo::reset(){
    writeToPCA(PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

void SF_Servo::sleep(){
    uint8_t awake = readFromPCA(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;  // 睡眠位调高
    writeToPCA(PCA9685_MODE1, sleep);
    delay(5);
}

void SF_Servo::wakeup(){
  uint8_t sleep = readFromPCA(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP;  // 睡眠位调低
  writeToPCA(PCA9685_MODE1, wakeup);
}

void SF_Servo::setAngle(uint8_t num, uint16_t val){
    uint16_t angle = (int)map(val, _min, _max, cal_min / 20 * 4095, cal_max / 20 * 4095);  //比例放大范围在0~4096
    setPWM(num, 0, angle);
}

void SF_Servo::setAngleRange(uint8_t min, uint8_t max){
    _min = min;
    _max = max;

    cal_min = 0.5/45 * min +0.5;
    cal_max = 0.5/45 * max +0.5;
}

void SF_Servo::setPWM(uint8_t num, uint16_t on, uint16_t off){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(PCA9685_LED0_ON_L + 4 * num);
    _i2c->write(on);
    _i2c->write(on >> 8);
    _i2c->write(off);
    _i2c->write(off >> 8);
    _i2c->endTransmission();
}

void SF_Servo::setPin(uint8_t num, uint16_t val, bool invert){
  // 值在0~4095之间
    val = min(val, (uint16_t)4095);
    if (invert) {
        if (val == 0) {
        setPWM(num, 4096, 0);
        } 
        else if (val == 4095) {
        setPWM(num, 0, 4096);
        } else {
        setPWM(num, 0, 4095 - val);
        }
    } else {
        if (val == 4095) {

        setPWM(num, 4096, 0);
        } else if (val == 0) {

        setPWM(num, 0, 4096);
        } else {
        setPWM(num, 0, val);
        }
    }
}


void SF_Servo::writeToPCA(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->write(data);
    _i2c->endTransmission();
}

uint8_t SF_Servo::readFromPCA(uint8_t addr){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
    return _i2c->read();
}


