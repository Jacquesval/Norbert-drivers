#include <Arduino.h>
#include "Motor.h"

#include "PinChangeInt.h"

// trick to have static callback -> can have 2 motors for the robot
Motor *instances[2] = {NULL, NULL};

static void countLeft(void){
    if (instances[LEFT] != NULL){
        instances[LEFT]->count();
    }
}

static void countRight(void){
    if (instances[RIGHT] != NULL){
        instances[RIGHT]->count();
    }
}



Motor::Motor(int pwmPin, int dirPin, int encPin)
{

    m_pwmPin = pwmPin;
    m_dirPin = dirPin;
    m_encPin = encPin;

    m_pwm = 0.0;
    m_encCount = 0;
    m_id = 0;
}

Motor::~Motor(){
    instances[m_id] = NULL;
}

bool Motor::init(int position = LEFT)
{
    if (position == LEFT || position == RIGHT){
        m_id = position;
        instances[position] = this;

        pinMode(m_pwmPin, OUTPUT);
        pinMode(m_dirPin, OUTPUT);
        pinMode(m_encPin, INPUT);

        // according to the position of the wheel we use different interrupts
        if (position == LEFT) {
            attachPinChangeInterrupt (m_encPin, countLeft, CHANGE);
        }
        else if (position == RIGHT){
            attachPinChangeInterrupt (m_encPin, countRight, CHANGE);
        }
        return true;
    }
    else {
        return false;
    }
}

void Motor::count()
{
    if (m_pwm > 0){
        m_encCount ++;
    }
    else {
        m_encCount --;
    }
}

int Motor::setSpeed(double speed)
{
    m_pwm = constrain(speed, -255, 255);
    if (m_pwm < 0)
    {
        digitalWrite(m_dirPin, 1);
        analogWrite(m_pwmPin, -m_pwm);
    }
    else
    {
        digitalWrite(m_dirPin, 0);
        analogWrite(m_pwmPin, m_pwm);
    }
    
    return 1;
    
}

int Motor::getId(){
    return m_id;
}

int Motor::getEncCount(){
    return m_encCount;
}

int Motor::setEncCount(int count = 0){
    m_encCount = count;
    return 0;
    }

int Motor::getPwm(){
    return m_pwm;
}