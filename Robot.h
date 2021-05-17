#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"

class Robot
{
private:
    Motor m_leftMotor;
    Motor m_rightMotor;
    int m_stdbyPin;


public:
    Robot(int pwmPinRight, int dirPinRight, int encPinRight,
          int pwmPinLeft, int dirPinLeft, int encPinLeft);
    int init(int stdbyPin);
    void move(double v, double w);
};
#endif