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
          m_pidV(200, 50, 0),
          m_pidW(25, 3, 0){

    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    m_v = 0.0;
    m_w = 0.0;
    m_vRef = 0.0;
    m_wRef = 0.0;



    robotPointer = this;


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
    x = (double) m_w;
    y = (double) m_wControl;
}

void Robot::control(){
    m_EncL = m_leftMotor.getEncCount();
    m_EncR = m_rightMotor.getEncCount();
    m_leftMotor.setEncCount(0);
    m_rightMotor.setEncCount(0);
    
    double dx = PI * RADIUS * (m_EncL + m_EncR) / N;
    double dTheta = 2 * PI * RADIUS * (m_EncR - m_EncL) / (N * LENGTH);

    m_v = dx / (m_controlPeriodInMs / 1000.0);
    m_w = dTheta / (m_controlPeriodInMs / 1000.0);

    m_x += dx * cos(m_theta);
    m_y += dx * sin(m_theta);
    m_theta += dTheta;

    m_vControl = m_pidV.computeOutput(m_v, m_vRef);
    m_wControl = m_pidW.computeOutput(m_w, m_wRef);
    m_leftMotor.setSpeed(m_vControl - m_wControl);
    m_rightMotor.setSpeed(m_vControl + m_wControl);

 
}