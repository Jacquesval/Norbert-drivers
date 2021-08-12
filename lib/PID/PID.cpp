#include "PID.h"
#include <Arduino.h>

PID::PID(double Kp, double Ki, double Kd){
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_lastupdateTime = 0;
    m_oldError = 0.0;
    m_sumError = 0.0;
    m_derivativeError = 0.0;

    firstCompute = true;
}

PID::PID() : PID(0,0,0){
}

double PID::computeOutput(double input, double setPoint){
    if (firstCompute){
        m_lastupdateTime = millis();
        firstCompute = false;
        return 0.0;
    }
    else{
        double error = setPoint - input;
        m_sumError += error;
        unsigned long updateTime = millis();
        m_derivativeError = (error - m_oldError) / (double) (updateTime -  m_lastupdateTime);
        m_oldError = error;
        m_lastupdateTime = updateTime;

        double output = m_Kp * error + m_Ki * m_sumError + m_Kd * m_derivativeError;

        return output;

    }
}

void PID::setGains(double Kp, double Ki, double Kd){
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

void PID::reset(){
    m_oldError = 0.0;
    m_sumError = 0.0;
    m_derivativeError = 0.0;
}