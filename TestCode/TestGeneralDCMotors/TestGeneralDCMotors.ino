#include <phys253.h>          
#include <LiquidCrystal.h>     

int Pin_Mot_0 = 0;
double Speed_Mot_0;
int Pin_Mot_1 = 1;
double Speed_Mot_1;

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);  
}
 
void loop()
{

  Speed_Mot_1 = (double) ( (254 / 1023.0)*knob(7) - 127 );
  Speed_Mot_0 = (double) ( (254 / 1023.0)*knob(6) - 127 );

  motor.speed(Pin_Mot_0, Speed_Mot_0);
  motor.speed(Pin_Mot_1, Speed_Mot_1);

  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("M0Spd:");
  LCD.print(Speed_Mot_0);
  LCD.setCursor(0, 1);
  LCD.print("M1Spd:");
  LCD.print(Speed_Mot_1);
  
  delay(30);

}
