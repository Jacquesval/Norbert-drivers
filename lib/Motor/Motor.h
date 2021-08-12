#ifndef MOTOR_H
#define MOTOR_H


#define LEFT 0
#define RIGHT 1

class Motor
{
private:
    int m_pwmPin;
    int m_dirPin;
    int m_encPin;
    int m_id;


    double m_pwm;
    int m_encCount;
    static int counter;

public:
    Motor(int pwmPin, int dirPin, int encPin);
    ~Motor();
    bool init(int position);
    void count();
    int setEncCount(int count);
    int getEncCount();
    int getPwm();

    int setSpeed(double speed);
    int getId();
};


#endif