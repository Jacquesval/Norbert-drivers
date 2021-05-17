#ifndef MOTOR_H
#define MOTOR_H


class Motor
{
private:
    int m_pwmPin;
    int m_dirPin;
    int m_encPin;

    double m_pwm;
    int m_encCount;

public:
    Motor(int pwmPin, int dirPin, int encPin);
    int init();
    void count();
    int setEncCount(int count);
    int getEncCount();

    int setSpeed(double speed);
};


#endif