#ifndef ROBOT_H
#define ROBOT_H

#include "Arduino.h"
#include "HCSR04.h"
#include "Motor.h"
#include "PID.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>

#define LENGTH 0.17 // length between two wheels in meter
#define RADIUS 0.034 // radius of the wheels in meter 
#define N 780 // encoder resolution
#define LINEAR_SCALE 0.96 // sca
#define ANGULAR_SCALE 1.0

class Robot
{
private:
    Motor m_leftMotor;
    Motor m_rightMotor;
    int m_stdbyPin;

    HCSR04 m_usSensor;
    // sensor_msgs::Range m_usRange;
    
    float m_w;
    float m_v;
    float m_x, m_y, m_theta;
    float m_vRef, m_wRef;
    int m_controlPeriodInMs;
    int m_controlPeriodCount;
    float m_vControl, m_wControl;
    int m_EncL, m_EncR;
    PID m_pidV, m_pidW;

    geometry_msgs::Pose2D m_pose;
    std_msgs::Float32 m_usRange;
    int m_USPeriodInMs;

    ros::Subscriber<geometry_msgs::Twist, Robot> m_cmdVelSub;
    ros::Publisher m_posePub;
    ros::Publisher m_usPub;






public:
    Robot(int pwmPinRight, int dirPinRight, int encPinRight,
        int pwmPinLeft, int dirPinLeft, int encPinLeft,
        int usTrigPin, int usEchoPin);
    ~Robot();
    int init(int controlPeriodInMs, int stdbyPin, ros::NodeHandle& nh);
    int run(int speed = 0);
    int stop();
    void printPose();
    void cmdVelCallback(const geometry_msgs::Twist &cmdVel);
    void getPose(float &x, float &y);
    void control();
    void setSpeed(float speed);
    void setYaw(float yaw);
    void publishUsDistance();
};
#endif