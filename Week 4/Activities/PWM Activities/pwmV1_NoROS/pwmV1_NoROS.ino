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
 * \brief This programs sends a PWM signal to the builtin LED of the Arduino Mega (For arduino Uno change ports). 
 */

 /*
 * INSTRUCTIONS
 *  Upload the code to the Arduino Mega and look at the builtin LED.
 */


/*
  HOW IT WORKS?
  1. Increase the PWM value from 0 -> 255 and decreases the PWM Value 255->0 once it has reached the maximum value (255).
*/

// Define the PWM output Pin.
const int PWMPin = 13;

void setup() {
    pinMode(PWMPin, OUTPUT);          // Setup pins as outputs.
}

void loop() {
    // Increase power to LED from Off state to brightest
    for (int pwmVal = 0; pwmVal < 255; pwmVal++) {
      analogWrite(PWMPin, pwmVal);                  // Write the PWM value to the output
      delay(20);
    }
    // Decrease power to the LED from full power to off state
    for (int pwmVal = 255; pwmVal >= 0; pwmVal--) {
      analogWrite(PWMPin, pwmVal);                // Write the PWM value to the output
      delay(20);
    }
    delay(100);
}