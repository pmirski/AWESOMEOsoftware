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
}
 
void loop()
{


  LCD.clear();  LCD.home();
  LCD.print("Sensor1 is: ");
  LCD.print(analogRead(40)); //40 = A0
  LCD.setCursor(0,1);
  LCD.print("Sensor2 is: ");
  LCD.print(digitalRead(15)); //pin 15 
 
  delay(100);
}
