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
 * \brief The state of the switch is then visualised using the Arduino builtin LED (Pin 13).
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
  3. AFTER the timer has elapsed, the change the status of the LED (toggle). 
*/

// Libraires used in this program.
#include <ros.h>
#include <std_msgs/Bool.h>

// Define the LEDPin and ButtonPin.
#define LedPin  LED_BUILTIN       // LEDBuiltin (Pin13)
#define ButtonPin 2               // Connect Button to pin 2 (External Interrupt in Arduino Mega/Uno)


// Declare functions and variables to be used
bool ledStatus = LOW;                      // Status of the LED
bool buttonStatus = LOW;                   // Current Status of the button
bool PrevbuttonStatus = LOW;               // Previous Status of the button
double debounceTime = 10;                  // Debounce time in mS
int counter = 0;                           // Pulse counter
volatile bool interruptStatus = false;     // Volatile global variable for the interrupt (Volatile variables are used due to the capacity of "Rapid changes")
long int timer = 0.0;                      // Debounce timer

// Interrupt Service Routine (ISR) for the button press.
void interruptHandler()
{
  if (interruptStatus == false)            //Check the status of the varible interruptStatus. If not activate (false), disregard the ISR (e.g., when bouncing) wait for the button to be released adn the timer has elapsed.
  {
    if (digitalRead(ButtonPin) == HIGH)   //If a new interrupt is perfomed, verify tge status of the Button and start the reading cycle (Read Button -> Verify release of button -> wait for timer -> change status of LED)
    {
      interruptStatus = true;             // Variable used to disregard the ISR (prevent multiple ISR to be fired when e.g, bouncing)
      counter++;                          // Increase the counter
      timer = millis();                   // Initialise the timer
    }
  }
}

void setup() {
  Serial.begin(9600);                     // Begin Serial communication with Arduino IDE
  pinMode(LedPin, OUTPUT);                // Initialise the Pins as Inputs/Outputs
  pinMode(ButtonPin, INPUT);  
  attachInterrupt(digitalPinToInterrupt(ButtonPin),interruptHandler,RISING);    //Attach an interrupt to the Button Pin (Arduino MEGA/UNO  interrupt 0 -> Pin 2), define the ISR and interrupt on rising edge.
}

void loop() {
  if(interruptStatus == true)
  {
    buttonStatus = digitalRead(ButtonPin);      // Check the current status of the button
    if(PrevbuttonStatus != buttonStatus)
    {
      if (millis()-timer>debounceTime)          // Check if the debounce timer has finished
      {
        ledStatus = HIGH - ledStatus;           // Toggle state
        digitalWrite(LedPin, ledStatus);        // Turn the LED on/off
        interruptStatus = false;                //reset the variable
      }
    }
    PrevbuttonStatus = buttonStatus;            // Update the previous status
  }
  else{
    Serial.println(counter);                    // Print the value of the counter on the terminal
  }
}