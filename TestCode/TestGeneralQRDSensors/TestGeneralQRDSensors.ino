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

  LCD.clear();  
  LCD.home();
  LCD.print("IL");
  LCD.print(digitalRead(8));
  LCD.print(" IR");
  LCD.print(digitalRead(9));
  LCD.print(" OL");
  LCD.print(digitalRead(10));
  LCD.print(" OR");
  LCD.print(digitalRead(11));

  LCD.setCursor(0,1);
  LCD.print(" FL");
  LCD.print(digitalRead(41));
  LCD.print(" FR");
  LCD.print(digitalRead(42));
  LCD.print(" TR");
  LCD.print(digitalRead(0));
  LCD.print(" CB");
  LCD.print(digitalRead(1));
 

  


  
  delay(30);
}
