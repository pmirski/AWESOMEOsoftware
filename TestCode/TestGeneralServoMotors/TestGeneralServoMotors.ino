#include <phys253.h>          
#include <LiquidCrystal.h>     


void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);
  RCServo0.write(90);   //crane
  RCServo1.write(40);   //agent grabber

    RCServo2.detach();
  pinMode(34, OUTPUT);       // Relay now works (needs to be output)
  digitalWrite(34, HIGH);    // Set up relay so that trolley is controlled, not platform
}
 
void loop()
{

  RCServo0.write( ( (double) (180.0 / 1023.0))*knob(6));
  RCServo1.write( ( (double) (180.0 / 1023.0))*knob(7));

  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("Crane:");
  LCD.print( ( (double) (180.0 / 1023.0))*knob(6));
  LCD.setCursor(0, 1);
  LCD.print("Claw:");
  LCD.print( ( (double) (180.0 / 1023.0))*knob(7));
  
  delay(30);

}
