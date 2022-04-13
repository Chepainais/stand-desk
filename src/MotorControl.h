#pragma once
#include <vector>
#include "Motor.h"
class MotorControl {
    std::vector<Motor> motors = {};
    int targetHeight;
    public: 
        int PWM_MIN = 20;
        int PWM_MAX = 100;
        void addMotor(Motor motor);
        void setTargetHeight(int height);
        void moveToHeight(int height);
        int debugCounter = 0;
        int debugCounterMax = 10000;
};