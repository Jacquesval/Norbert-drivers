#include "Robot.h"
#include <Arduino.h>

#define R 1
#define L 1

Robot::Robot(int pwmPinRight, int dirPinRight, int encPinRight,
          int pwmPinLeft, int dirPinLeft, int encPinLeft) : m_leftMotor(pwmPinLeft, dirPinLeft, encPinLeft), m_rightMotor(pwmPinRight, dirPinRight, encPinRight) {

          }

int Robot::init(int stdbyPin) {
    m_stdbyPin = stdbyPin;
    pinMode(m_stdbyPin, OUTPUT);
    digitalWrite(m_stdbyPin, HIGH);
    m_leftMotor.init();
    m_rightMotor.init();
    return 1;
}

void Robot::move(double v, double w){
    double vr = (2 * v + w * L) / (2 * R);
    double vl = (2 * v - w * L) / (2 * R);
    m_leftMotor.setSpeed(vl);
    m_rightMotor.setSpeed(vr);
}