#include <ros.h>
#include <std_msgs/Empty.h>
#include "Robot.h"
#include "include/Pins.h"

// ros::NodeHandle  nh;


void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}
Robot norbert(PWMB_RIGHT, BIN1, ENCODER_RIGHT_A_PIN,
              PWMA_LEFT, AIN1, ENCODER_LEFT_A_PIN);
// Motor rightWheel(PWMB_RIGHT, BIN1, ENCODER_RIGHT_A_PIN);


// ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  
  pinMode(LED_BUILTIN, OUTPUT);
  norbert.init(STBY_PIN);
  
  // nh.initNode();
  // nh.subscribe(sub);
  // leftWheel.init();
  // rightWheel.init();
  // leftWheel.setSpeed(1000);
  // rightWheel.setSpeed(100);
  
  Serial.begin(9600);
}

void loop()
{ 
  for(int i =0; i < 10; i++) {
    norbert.move(i*10,0);
    delay(5000);
  }
  // nh.spinOnce();
  // delay(1);
}
