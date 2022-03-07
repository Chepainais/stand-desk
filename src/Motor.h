#pragma once
#include <ESP32Encoder.h>
class Motor{
    ESP32Encoder encoder {};
    int GpioPWM {};
    int GpioDir {};
    int nrpk {};
    int PWMFreq = 5000;
    int PWMResolution = 10;// 1024
    public: 
        Motor(ESP32Encoder enc, int gpioPWM, int gpioDir, int nrpk) 
            : encoder {enc}, GpioPWM {gpioPWM}, GpioDir {gpioDir}, nrpk {nrpk}{
                ledcSetup(nrpk, PWMFreq, PWMResolution);
                ledcAttachPin(GpioPWM, nrpk);
            };
        int Pwm = 0;
        void setPwm(int pwm);
        void setDirection(bool direction);
        int getHeight();
};