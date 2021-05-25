#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "PID.h"


#define LENGTH 0.17 // length between two wheels in meter
#define RADIUS 0.034 // radius of the wheels in meter 
#define N 780 // encoder resolution
#define PI 3.14

class Robot
{
private:
    Motor m_leftMotor;
    Motor m_rightMotor;
    int m_stdbyPin;
    double m_w;
    double m_v;
    double m_x, m_y, m_theta;
    double m_vRef, m_wRef;
    int m_controlPeriodInMs;
    double m_vControl, m_wControl;
    int m_EncL, m_EncR;
    int m_dEncL, m_dEncR;
    int m_oldEncL, m_oldEncR;
    PID m_pidR, m_pidL;



public:
    Robot(int pwmPinRight, int dirPinRight, int encPinRight,
          int pwmPinLeft, int dirPinLeft, int encPinLeft);
    ~Robot();
    int init(int controlPeriodInMs, int stdbyPin);
    int run(int speed = 0);
    int stop();
    void printPose();
    void move(double v, double w);
    void getPose(double &x, double &y);
    void control();
};
#endif