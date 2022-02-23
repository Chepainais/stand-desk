#pragma once
#include <ESP32Encoder.h>
class Motor{
    ESP32Encoder encoder {};
    int GpioPWM {};
    int GpioDir {};
    public: Motor(ESP32Encoder enc, int gpioPWM, int gpioDir) 
        : encoder {enc}, GpioPWM {gpioPWM}, GpioDir {gpioDir}
    {    }
};