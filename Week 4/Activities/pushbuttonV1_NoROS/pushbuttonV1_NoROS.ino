// Libraires used in this program.
#include <ros.h>
#include <std_msgs/Bool.h>

// Define the LEDPin and ButtonPin.
#define LedPin  LED_BUILTIN       // LEDBuiltin (Pin13)
#define ButtonPin 2               // Connect Button to pin 2 (External Interrupt in Arduino Mega/Uno)


// Declare functions and variables to be used
bool ledStatus = LOW;                      // Status of the LED
bool buttonStatus = LOW;
bool PrevbuttonStatus = LOW;
double debounceTime = 10;                  // Debounce time in mS
int counter = 0;
volatile bool interruptStatus = false;     // Volatile global variable for the interrupt (Volatile variables are used due to the capacity of "Rapid changes")
long int timer = 0.0;

// Interrupt Service Routine (ISR) for the button press.
void interruptHandler()
{
  if (interruptStatus == false)                   //Check the status of the varible interruptStatus. If not activate (false), disregard the ISR (e.g., when bouncing) wait for the button to be released adn the timer has elapsed.
  {
    if (digitalRead(ButtonPin) == HIGH)   //If a new interrupt is perfomed, verify tge status of the Button and start the reading cycle (Read Button -> Verify release of button -> wait for timer -> change status of LED)
    {
      interruptStatus = true;             // Variable used to disregard the ISR (prevent multiple ISR to be fired when e.g, bouncing)
      counter++;
      timer = millis();
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LedPin, OUTPUT);                // Initialise the Pins as Inputs/Outputs
  pinMode(ButtonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ButtonPin),interruptHandler,RISING);    //Attach an interrupt to the Button Pin (Arduino MEGA/UNO  interrupt 0 -> Pin 2), define the ISR and interrupt on rising edge.
}

void loop() {
  if(interruptStatus == true)
  {
    buttonStatus = digitalRead(ButtonPin);
    if(PrevbuttonStatus != buttonStatus)
    {
      if (millis()-timer>debounceTime)
      {
        ledStatus = HIGH - ledStatus;           // Toggle state
        digitalWrite(LedPin, ledStatus);        // Turn the LED on/off
        interruptStatus = false; 
      }
    }
    PrevbuttonStatus = buttonStatus;
  }
  else{
    Serial.println(counter);
  }
}