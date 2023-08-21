/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

void LEDcallback( const std_msgs::Bool &msg){
  if(!msg.data){
      digitalWrite(13,LOW);   // blink the led
  }
  else{
      digitalWrite(13,HIGH);   // blink the led
  }

}

ros::Subscriber<std_msgs::Bool> sub("LED_state", &LEDcallback);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}


