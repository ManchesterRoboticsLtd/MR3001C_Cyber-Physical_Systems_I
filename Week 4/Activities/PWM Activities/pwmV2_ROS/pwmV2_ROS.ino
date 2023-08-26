 /*
 * Copyright (c) 2019, Manchester Robotics Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Manchester Robotics Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file PWM_arduino.ino
 * \author Eduard Codres, Mario Martinez
 * \copyright Manchester Robotics Ltd.
 * \date March, 2020
 * \brief This programs remaps a float point value in the range [0,1] received from an external computer using ROS into a topic called /PWM to an integer analog PWM value in the range [0, 255] i.e., [0,1] -> [0,255]
 * \brief The program sends the PWM signal to the builtin LED of the Arduino Mega (For arduino Uno change ports). 
 */

 /*
 * INSTRUCTIONS
 * 1. Upload the code to the Arduino Mega 
 * 2. Run ROS and Rosserial in Ubuntu
 * 3. Publish a Float32 value in the range [0,1] to the /PWM topic
 * 4. Look at the Arduino Mega Builtin LED
 */


/*
  HOW IT WORKS?
  1. Subscribes to the /PWM topic.
  2. Remaps (trnaforms) the user value to a PWM value i.e., [0,1] -> [0,255]
  3. Displays the signals on the Arduino Builtin LED
*/

// Libraires used in this program.
#include <ros.h>
#include <std_msgs/Float32.h>

// Define the pwmPin ooutput pin.
#define pwmPin  13

// Create a ROS node Handler.
ros::NodeHandle  nh;

// Declare functions and variables to be used
float pwmVal = 0.0;
int pwmTransform(float pwmVal);                     // Declaration of the Transformation function
void PWMcallback( const std_msgs::Float32 &msg);    // Declaration of the callback function

// Define the subscribers and publishers
ros::Subscriber<std_msgs::Float32> sub("PWM", &PWMcallback);

// Subscriber Callback function, receive a Float32 Message
void PWMcallback( const std_msgs::Float32 &msg){
  pwmVal = msg.data;
}

//Setup
void setup() {
    pinMode(pwmPin, OUTPUT);          // Setup pins as outputs.
    nh.initNode();                    // Initialise ROS Node
    nh.subscribe(sub);                // Initialise subscriber
}

void loop() {
  analogWrite(pwmPin, pwmTransform(pwmVal));    // Write the PWM value to the pwmPin
  nh.spinOnce();                                // Spinonce ROS (verify the callbacks if applicable)
  delay(10);                                    // Delay
}

// Function to transform the pwmValue in the range [0, 1] received on the subscriber, to a PWM Value in the range [0, 255], returns an integer.
int pwmTransform(float pwmVal){
  if (pwmVal <= 0.0)
  {
    return 0;                           // If value received is negative, return 0 for the PWM.
  }
  else if (pwmVal >= 1.0)
  {
    return 255;                         // If value received is greater than 1, return 255 for the PWM.
  }
  else
  {
    return (int) (pwmVal* 255);         // Return the proportional value of the transformation [0,1] -> [0,255]
  }
}