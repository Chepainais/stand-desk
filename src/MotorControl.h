#pragma once
#include <vector>
#include "Motor.h"
class MotorControl {
    std::vector<Motor> motors = {};
    public: 
        void addMotor(Motor motor);
};