#include <phys253.h>          
#include <LiquidCrystal.h>     

int Pin_SwitchMotTrol2Lift = 34;

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);  
  RCServo0.write(0);   //crane
  RCServo1.write(40);   //agent grabber
  RCServo2.detach();
  pinMode(Pin_SwitchMotTrol2Lift, OUTPUT);       // Relay now works (needs to be output)
  digitalWrite(Pin_SwitchMotTrol2Lift, LOW);    // Set up relay so that trolley is controlled, not platform
}
 
void loop()
{

  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("Pin34Val: ");
  LCD.print(digitalRead(Pin_SwitchMotTrol2Lift));
  motor.speed(2,40);
  
  delay(30);

}
