#include <phys253.h>          
#include <LiquidCrystal.h>     

int Pin_Mot_0 = 2;
double Speed_Mot_0;
int Pin_Mot_1 = 3;
double Speed_Mot_1;

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);  
  RCServo0.write(50);   //crane
  RCServo1.write(40);   //agent grabber

  RCServo2.detach();
  pinMode(34, OUTPUT);       // Relay now works (needs to be output)
  digitalWrite(34, HIGH);    // Set up relay so that trolley is controlled, not platform
}
 
void loop()
{

  Speed_Mot_1 = (double) ( (254*2 / 1023.0)*knob(7) - 254 );    //MAX SPEEDS ARE HALVED TO NOT DESTORY MOTORS
  Speed_Mot_0 = (double) ( (254*2 / 1023.0)*knob(6) - 254 );

  motor.speed(Pin_Mot_0, Speed_Mot_0);
  motor.speed(Pin_Mot_1, Speed_Mot_1);

  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("Trol:");
  LCD.print(Speed_Mot_0);
  LCD.setCursor(0, 1);
  LCD.print("CB:");
  LCD.print(Speed_Mot_1);
  
  delay(30);

}
