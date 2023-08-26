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
 * \file button_arduino.ino
 * \author Eduard Codres, Mario Martinez
 * \copyright Manchester Robotics Ltd.
 * \date March, 2020
 * \brief This programs uses external interrupts to read the state of a normally open (NO) pushbutton using a debounce routine.
 * \brief The  
 * \brief The satate is then published using ROS, and visualised using the Arduino builtin LED (Pin 13).
 */

 /*
 * INSTRUCTIONS
 *  Connect a normally open (NO) pushbutton with in a pulldown resistor configuration to the ButtonPin.
    (https://www.electroduino.com/arduino-tutorial-8-arduino-digitalread-using-push-button/)
 *  ButtonPin must be selected as one with an external interrupt. Default for Arduino Mega/Uno = 2.
    (https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
 */


/*
  HOW IT WORKS?
  1. This sketch verifies if the pushbutton is PRESSED (Interrupt) AND THEN RELEASED.
  2. STARTS a debounce timer 
  3. AFTER the timer has elapsed, the change the status of the LED (toggle), and the ROS publisher. 
*/

// Libraires used in this program.
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// Define the LEDPin and ButtonPin.
#define LedPin  LED_BUILTIN       // LEDBuiltin (Pin13)
#define ButtonPin 2               // Connect Button to pin 2 (External Interrupt in Arduino Mega/Uno)

// Create a ROS node Handler.
ros::NodeHandle nh;

// Define the type of message and create a publisher
std_msgs::Bool pushed_msg;
std_msgs::Int32 counter_msg;
ros::Publisher pub_button("pushed", &pushed_msg);
ros::Publisher pub_counter("counter", &counter_msg);

// Declare functions and variables to be used
bool ledStatus = LOW;                      // Status of the LED
double debounceTime = 10;                  // Debounce time in mS
int32_t counter = 0;
volatile bool interruptStatus = false;     // Volatile global variable for the interrupt (Volatile variables are used due to the capacity of "Rapid changes")

// Interrupt Service Routine (ISR) for the button press.
void interruptHandler()
{
  if (!interruptStatus)                   //Check the status of the varible interruptStatus. If not activate (false), disregard the ISR (e.g., when bouncing) wait for the button to be released adn the timer has elapsed.
  {
    if (digitalRead(ButtonPin) == HIGH)   //If a new interrupt is perfomed, verify tge status of the Button and start the reading cycle (Read Button -> Verify release of button -> wait for timer -> change status of LED)
    {
      interruptStatus = true;             // Variable used to disregard the ISR (prevent multiple ISR to be fired when e.g, bouncing)
      counter++;                          // Update the counter
    }
  }
}

// Read the button routine adn verify timer routine
bool buttonRead(){
  // Initialise variables to be used in this function
  bool buttonStatus = false;            // Variable to verify if the switch is pressed or released.
  static bool buttonRelease = false;    // Variable to store the previous state of the button (static to keep the old values after initialising)
  static long int timer;                // Variable to store the elapsed time.

  if (interruptStatus)                  // Interrupt was triggered, start the reading cycle (Read Button -> Verify release of button -> wait for timer)
  {
    buttonStatus = digitalRead(ButtonPin);  // Verify it the button is still pressed
    if(buttonStatus)
    {
      buttonRelease = true;                 // Variable storing the status of the button
      timer = millis();                     // Start the timer
    } 
    // Verify if the button is released and the previous status of the button is pressed and the timer has finsihed. 
    if (buttonRelease && !buttonStatus)
    {
      if (millis() - timer >= debounceTime)
      {
        buttonStatus = false;               // Reset the button status variable
        interruptStatus = false;            // Reset the variable to "activate" the ISR again
        return true;                        // Return the status of the button as pressed
      }
    }
  }
  return false;                             // Return the status of the button as not pressed (hasn't gont through the reading cycle nor the ISR)
}

void setup() {
  nh.initNode();                          // Initialise ROS Node
  nh.advertise(pub_button);               // Advertise the Topics
  nh.advertise(pub_counter);               // Advertise the Topics
  pinMode(LedPin, OUTPUT);                // Initialise the Pins as Inputs/Outputs
  pinMode(ButtonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ButtonPin),interruptHandler,RISING);    //Attach an interrupt to the Button Pin (Arduino MEGA/UNO  interrupt 0 -> Pin 2), define the ISR and interrupt on rising edge.
}

void loop() {
    if (buttonRead() == true) {
      ledStatus = HIGH - ledStatus;           // Toggle state
      digitalWrite(LedPin, ledStatus);        // Turn the LED on/off
    }
    else {
      pushed_msg.data = ledStatus;            // Populate the message
      counter_msg.data = counter;             // Populate the message
      pub_button.publish(&pushed_msg);        // Publish the message
      pub_counter.publish(&counter_msg);      // Publish the message
    }
    nh.spinOnce();                            // Spin ROS (verify threads and callbacks if any)
    delay(10);
}