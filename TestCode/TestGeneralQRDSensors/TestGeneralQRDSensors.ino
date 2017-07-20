/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int inPin = 0;

void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
    RCServo0.write(90);
}
 
void loop()
{

  LCD.clear();  LCD.home();
  LCD.print("AnalPin0:");
  LCD.print(analogRead(40));
  LCD.setCursor(0,1);
  LCD.print("DigiPin15:");
  LCD.print(digitalRead(15));
  
  delay(30);
}
