#include <Arduino.h>
#include "MsTimer2.h"
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "Robot.h"
#include "include/Pins.h"
#include "PID.h"



ros::NodeHandle nh;

Robot norbert(PWMB_RIGHT, BIN1, ENCODER_RIGHT_A_PIN,
            PWMA_LEFT, AIN1, ENCODER_LEFT_A_PIN);

void setup()
{ 
  nh.getHardware()->setBaud(115200);
  pinMode(STBY_PIN, OUTPUT);

  nh.initNode();
  delay(1000);

  norbert.init(100, STBY_PIN, nh);
  delay(500);
  norbert.run();

  
}

double x,y;
void loop(){
  delay(1);
  nh.spinOnce();  
}