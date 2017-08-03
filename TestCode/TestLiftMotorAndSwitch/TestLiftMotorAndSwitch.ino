#include <phys253.h>          
#include <LiquidCrystal.h>     

//both pins low when TINAH off (or when . Both high ( when TINAH on.
//Switch values with digitalWrite(pin, value)

int Pin_Mot_2 = 2;
double Speed_Mot_2;

int Pin_SwitchPltfrm = 7;
int SwitchValue;
int SwitchTriggered = false;
int MotorDirection = 1;

int Pin_SwitchMotTrol2Lift = 34;                  // Switches relay for motor. Pin on TINAH. As per TINAH log  

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600);  
  RCServo0.write(0);   //crane
  RCServo1.write(40);   //agent grabber
  digitalWrite(Pin_SwitchMotTrol2Lift, LOW);
}
 
void loop()
{

  Speed_Mot_2 = (double) ( (508 / 1023.0)*knob(7) - 254 );    //MAX SPEEDS ARE HALVED TO NOT DESTORY MOTORS

  if(SwitchTriggered){
    unsigned long startTime = millis();
    unsigned long testDuration = 100;
    int testSignal = 0;
    int count = 0;

    while(millis() - startTime < testDuration){
      count++;
      
    }
    delay(2500);
    MotorDirection = MotorDirection*(-1)
    
  }
  motor.speed(Pin_Mot_2, Speed_Mot_2*MotorDirection);

  if(!digitalRead(Pin_SwitchPltfrm)){
    
  }

  SwitchValue = digitalRead(Pin_SwitchPltfrm);

  LCD.clear();
  LCD.home();
  LCD.setCursor(0, 0);
  LCD.print("MotSpeed:");
  LCD.print(Speed_Mot_2);
  LCD.setCursor(0, 1);
  LCD.print("PltfrmSwtch: ");
  LCD.print(SwitchValue);
  
  delay(30);

}
