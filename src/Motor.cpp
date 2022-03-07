#include "Motor.h"
int Motor::getHeight()
{
    return encoder.getCount();
}
void Motor::setPwm(int pwm){
    Pwm = pwm;
    ledcWrite(nrpk, pwm);
}
void Motor::setDirection(bool direction){
    
    digitalWrite(GpioDir, direction);
}