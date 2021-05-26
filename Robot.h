#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "PID.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

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
    PID m_pidV, m_pidW;

    ros::Subscriber<geometry_msgs::Twist, Robot> m_cmdVelSub;




public:
    Robot(int pwmPinRight, int dirPinRight, int encPinRight,
        int pwmPinLeft, int dirPinLeft, int encPinLeft);
    ~Robot();
    int init(int controlPeriodInMs, int stdbyPin, ros::NodeHandle& nh);
    int run(int speed = 0);
    int stop();
    void printPose();
    void cmdVelCallback(const geometry_msgs::Twist &cmdVel);
    void getPose(double &x, double &y);
    void control();
};
#endif