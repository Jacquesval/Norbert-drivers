#include <Arduino.h>
#include "MsTimer2.h"
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "Robot.h"
#include "include/Pins.h"

ros::NodeHandle nh;


std_msgs::Float64MultiArray debugMsg;
ros::Publisher debug("debug", &debugMsg);

Robot norbert(PWMB_RIGHT, BIN1, ENCODER_RIGHT_A_PIN,
              PWMA_LEFT, AIN1, ENCODER_LEFT_A_PIN);


void setup()
{ 
  nh.getHardware()->setBaud(115200);
  // Serial.begin(115200);
  pinMode(STBY_PIN, OUTPUT);

  debugMsg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  debugMsg.layout.dim[0].label = "height";
  debugMsg.layout.dim[0].size = 2;
  debugMsg.layout.dim[0].stride = 1;
  debugMsg.layout.data_offset = 0;
  debugMsg.data = (float *)malloc(sizeof(float)*4);
  debugMsg.data_length =2;
  debugMsg.data[0] = 0;
  debugMsg.data[1] = 0;
  nh.advertise(debug);
  nh.initNode();
  delay(500);

  norbert.init(100, STBY_PIN);
  delay(500);
  norbert.run(0);
  // norbert.move(10, 0);

  
}

double x,y;
void loop(){
  delay(500);
  norbert.getPose(x, y) ;
  debugMsg.data[0] = x;
  debugMsg.data[1] = y;
  debug.publish(&debugMsg);
  nh.spinOnce();  
}