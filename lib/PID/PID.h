#ifndef PID_H
#define PID_H

class PID
{
private:
    double m_Kp, m_Ki, m_Kd;
    double m_oldError, m_sumError, m_derivativeError;
    unsigned long m_lastupdateTime;
    bool firstCompute;

public:
    PID(double Kp, double Ki, double Kd);
    PID();
    double computeOutput(double input, double setPoint);
    void setGains(double Kp, double Ki, double Kd);
    void reset();


};

#endif