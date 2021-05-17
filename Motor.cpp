#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int pwmPin, int dirPin, int encPin)
{
    m_pwmPin = pwmPin;
    m_dirPin = dirPin;
    m_encPin = encPin;

    m_pwm = 0.0;
    m_encCount = 0;
}

int Motor::init()
{
    pinMode(m_pwmPin, OUTPUT);
    pinMode(m_dirPin, OUTPUT);
}


void Motor::count()
{
    m_encCount ++;
}

int Motor::setSpeed(double speed)
{
    m_pwm = constrain(speed, -255, 255);
    if (m_pwm < 0)
    {
        digitalWrite(m_dirPin, 1);
        analogWrite(m_pwmPin, -m_pwm);
    }
    else
    {
        digitalWrite(m_dirPin, 0);
        analogWrite(m_pwmPin, m_pwm);
    }
    
    return 1;
    
}
