/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int knobPin = 46;
unsigned long knob6value = 0;
int motorspeed = 0;
/*unsigned long freq; */

void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(knobPin, INPUT);
}
 
void loop()
{
  knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
  motorspeed = knob6value*180.0/1024.0; //NOTE WHAT'S DONE HERE WHEN U WANNA HAVE A CONSTANT W DECIMAL VALUES
  RCServo0.write((int) motorspeed); //motorspeed=angle

  LCD.clear();  
  LCD.home();
  LCD.print("KnobValue: ");
  LCD.print(knob6value);
  LCD.setCursor(0,1);
  LCD.print("SerSpd:");
  LCD.print(motorspeed);
  delay(1000);
}
