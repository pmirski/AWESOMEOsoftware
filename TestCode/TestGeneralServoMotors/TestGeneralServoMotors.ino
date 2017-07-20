#include <phys253.h>          
#include <LiquidCrystal.h>     


void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);
  RCServo0.write(90);
  RCServo1.write(40);
  
}
 
void loop()
{

  RCServo0.write( ( (double) (170.0 / 1023.0))*knob(6));
  RCServo1.write( ( (double) (170.0 / 1023.0))*knob(7));


  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("Srvo0:");
  LCD.print( ( (double) (170.0 / 1023.0))*knob(6));
  LCD.setCursor(0, 1);
  LCD.print("Srvo1:");
  LCD.print( ( (double) (170.0 / 1023.0))*knob(7));
  

  delay(30);

}
