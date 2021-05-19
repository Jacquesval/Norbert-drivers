#include <Arduino.h>
#include "Motor.h"

#include "include/PinChangeInt.h"

Motor *instances[2] = {NULL, NULL};

static void countLeft(void){
    if (instances[LEFT] != NULL){
        // Serial.println("count left");
        instances[LEFT]->count();
    }
}

static void countRight(void){
    if (instances[RIGHT] != NULL){
        // Serial.println("count right");
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
}


bool Motor::init(int position = LEFT)
{
    if (position == LEFT || position == RIGHT){
        instances[position] = this;

        pinMode(m_pwmPin, OUTPUT);
        pinMode(m_dirPin, OUTPUT);
        pinMode(m_encPin, INPUT);
        if (position == LEFT) {
            Serial.print("attach left with"); Serial.println(m_encPin);
            attachInterrupt (digitalPinToInterrupt(m_encPin), countLeft, CHANGE);
        }
        else if (position == RIGHT){
            Serial.print("attach right with "); Serial.println(m_encPin);
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
    // Serial.print(id); Serial.println(" : count");
    if (m_pwm > 0){
        m_encCount ++;
    }
    else if (m_pwm < 0) {
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
    return id;
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