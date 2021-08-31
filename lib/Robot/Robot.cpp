#include "Robot.h"
#include <Arduino.h>
#include <math.h>

// #include <TimerOne.h>
#include "MsTimer2.h"
#include <ros.h>


Robot *robotPointer = NULL;

static void controlWrapper(){
    robotPointer->control();
}


Robot::Robot(int pwmPinRight, int dirPinRight, int encPinRight,
             int pwmPinLeft, int dirPinLeft, int encPinLeft, int usTrigPin, int usEchoPin) :
          m_cmdVelSub("cmd_vel", &Robot::cmdVelCallback, this),
          m_posePub("norbert_state", &m_pose),
          m_leftMotor(pwmPinLeft, dirPinLeft, encPinLeft),
          m_rightMotor(pwmPinRight, dirPinRight, encPinRight),
          m_pidV(200, 50, 0),
          m_pidW(25, 5, 0),
          m_usSensor(usTrigPin, usEchoPin),
          m_usPub("usRange", &m_usRange){

    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    m_v = 0.0;
    m_w = 0.0;
    m_vRef = 0.0;
    m_wRef = 0.0;

    m_controlPeriodCount = 0;

    m_USPeriodInMs = 1000;

    robotPointer = this;
}

Robot::~Robot(){
    robotPointer = NULL;
}


int Robot::init(int controlPeriodInMs, int stdbyPin, ros::NodeHandle& nh) {
    m_stdbyPin = stdbyPin;
    pinMode(m_stdbyPin, OUTPUT);

    MsTimer2::set(controlPeriodInMs, controlWrapper);

    m_leftMotor.init(LEFT);
    m_rightMotor.init(RIGHT);
    m_controlPeriodInMs = controlPeriodInMs;

    nh.subscribe(m_cmdVelSub);
    nh.advertise(m_posePub);
    nh.advertise(m_usPub);

    return 0;
}

int Robot::run(int speed = 0){
    digitalWrite(m_stdbyPin, HIGH);
    m_rightMotor.setSpeed(speed);
    m_leftMotor.setSpeed(speed);
    MsTimer2::start();
    return 0;
}

int Robot::stop(){
    m_leftMotor.setSpeed(0);
    m_rightMotor.setSpeed(0);
    digitalWrite(m_stdbyPin, LOW);

    return 0;
}



void Robot::getPose(float &x, float &y){
    x = sqrt(m_x * m_x + m_y * m_y);
    y = m_theta;
}


void Robot::cmdVelCallback(const geometry_msgs::Twist &cmdVel){
    m_vRef = cmdVel.linear.x;
    m_wRef = cmdVel.angular.z;
}

void Robot::control(){
    m_controlPeriodCount++;
    m_EncL = m_leftMotor.getEncCount();
    m_EncR = m_rightMotor.getEncCount();
    m_leftMotor.setEncCount(0);
    m_rightMotor.setEncCount(0);
    
    float dx =LINEAR_SCALE * PI * RADIUS * (m_EncL + m_EncR) / N;
    float dTheta = ANGULAR_SCALE * 2 * PI * RADIUS * (m_EncR - m_EncL) / (N * LENGTH);

    m_v = dx / (m_controlPeriodInMs / 1000.0);
    m_w = dTheta / (m_controlPeriodInMs / 1000.0);

    m_x += dx * cos(m_theta);
    m_y += dx * sin(m_theta);
    m_theta += dTheta;

    m_pose.x = m_x;
    m_pose.y = m_y;
    m_pose.theta = m_theta;

    m_vControl = m_pidV.computeOutput(m_v, m_vRef);
    m_wControl = m_pidW.computeOutput(m_w, m_wRef);
    m_leftMotor.setSpeed(m_vControl - m_wControl);
    m_rightMotor.setSpeed(m_vControl + m_wControl);

    m_posePub.publish(&m_pose);

    publishUsDistance();

 
}

void Robot::setSpeed(float speed){
    m_vRef = speed;
}

void Robot::setYaw(float yaw){
    m_wRef = yaw;
}

void Robot::publishUsDistance(){
    int rangeInCm = m_usSensor.dist();
    rangeInCm = constrain(rangeInCm, 2, 400);
    if (m_usRange.data != rangeInCm){
        m_usRange.data = (float) rangeInCm;
        if (1){
            m_usPub.publish(&m_usRange);
        }
    }
}