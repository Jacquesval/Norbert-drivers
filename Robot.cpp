#include "Robot.h"
#include <Arduino.h>
#include <math.h>
#include "PID.h"

// #include <TimerOne.h>
#include "MsTimer2.h"



Robot *robotPointer = NULL;

static void controlWrapper(){
    robotPointer->control();
}



Robot::Robot(int pwmPinRight, int dirPinRight, int encPinRight,
          int pwmPinLeft, int dirPinLeft, int encPinLeft) :
          m_leftMotor(pwmPinLeft, dirPinLeft, encPinLeft),
          m_rightMotor(pwmPinRight, dirPinRight, encPinRight),
          m_pidL(10, 5, 0),
          m_pidR(10, 5, 0){

    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    m_v = 0.0;
    m_w = 0.0;
    m_vRef = 200;
    m_wRef = 0.0;


    m_dEncL = 0;
    m_dEncR = 0;

    robotPointer = this;
    m_oldEncL = 0;
    m_oldEncR = 0;


}

Robot::~Robot(){
    robotPointer = NULL;
}


int Robot::init(int controlPeriodInMs, int stdbyPin) {
    m_stdbyPin = stdbyPin;
    pinMode(m_stdbyPin, OUTPUT);

    MsTimer2::set(controlPeriodInMs, controlWrapper);

    m_leftMotor.init(LEFT);
    m_rightMotor.init(RIGHT);
    m_controlPeriodInMs = controlPeriodInMs;

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

void Robot::move(double v, double w){
    m_vRef = v;
    m_wRef = w;
}

void Robot::getPose(double &x, double &y){
    x = (double) m_EncL;
    y = (double) m_vControl;
}

void Robot::control(){
    m_EncL = m_leftMotor.getEncCount();
    m_EncR = m_rightMotor.getEncCount();
    m_leftMotor.setEncCount(0);
    m_rightMotor.setEncCount(0);
    m_vControl = m_pidL.computeOutput(m_EncL, m_vRef);
    m_wControl = m_pidR.computeOutput(m_EncL, m_vRef);
    m_leftMotor.setSpeed(m_vControl);
    m_rightMotor.setSpeed(m_wControl);
}