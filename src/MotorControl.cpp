#include "MotorControl.h"
void MotorControl::addMotor(Motor motor)
{
    motors.push_back(motor);
    // set target height to current positions, to avoid moving on initialization
    this->setTargetHeight(motor.getHeight());
}
void MotorControl::setTargetHeight(int height)
{
    targetHeight = height;
}
void MotorControl::moveToHeight(int height)
{
//cycle , WHILE target reached!!
int h = 0;
int mostOff = 0;
int initialHeight = 0;
Serial.println("Move to height received:" );
// Get initial difference
for(auto & m : motors){
    Serial.println("bfmh");
    h = m.getHeight();
    initialHeight = h;
    Serial.print("motor height ");
    Serial.println(height);

    int absOff = height - h;
    Serial.print("absoff ");
    Serial.println(absOff);
    float off = abs(absOff);
    Serial.print("Off: ");
    Serial.print(off);
    Serial.println(";");
    if(off > mostOff){
        mostOff = off;
    }
}
Serial.print("Initial mostOff = ");
Serial.println(mostOff);
int softLength = PWM_MAX - PWM_MIN;
    while (mostOff > 5)
    {
        // @TODO softstart

        // reset mostoff
            mostOff = 0;
        for(auto & m : motors){
            h = m.getHeight();
            Serial.print("H1 ");
            Serial.println(h);
            bool up = h>height;
            int pwm = 100;
            uint8_t dir = LOW;
            if(up){
                dir = HIGH;
            }
            int off = abs(height - h);
            if(off > mostOff){
                mostOff = off;
            }
            

            // SOFT START
            if( abs(h-initialHeight)< softLength){
                pwm = max(PWM_MIN + abs(initialHeight-h), PWM_MIN);
            }
            // SOFT STOP
            if(softLength > off){
                pwm = min(PWM_MIN + off, PWM_MAX);
            }

            int pwmDuty = 1024/100*pwm;
            Serial.print("target: ");
            Serial.print(height);
            Serial.print(" current: ");
            Serial.print(h);
            Serial.print(" Mostoff: ");
            Serial.print(mostOff);
            Serial.print(" direction ");
            Serial.println(dir);
            Serial.print("pwm ");
            Serial.println(pwm);
            m.setDirection(dir);
            m.setPwm(pwmDuty);
        }
// @TODO REMOVE delay
        delay(100);
    }
    // STOP ALL MOTORS
    for(auto & m : motors){
        Serial.println("stop motor");
        m.setDirection(LOW);
        m.setPwm(0);
    }
}