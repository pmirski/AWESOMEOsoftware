/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int inPin = 0;

int Black_Tape = 1; // Used to determine whether trolley and claw block sensors read white or black tape

//  VARIABLES ONLY FOR FUNCTION setTrolleyHorizontalPosition
// Important concerns: delay length, initial trolley position, trolley motor speed
int Previous_STP = 0;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU
int Sensor_Trolley_Position = 0; //Initialization value arbitrary
int Trolley_Direction_Backward = -1;
int Trolley_Direction_Forward = 1;
int Pin_Sensor_Trolley = 6; // Pin on TINAH. As per TINAH log
int Pin_Motor_Trolley = 2; // Pin on TINAH. As per TINAH log
int Motor_Speed_Trolley = 100; // Value is 1/2.55 of maximum, CAN CHANGE SPEED
int Motor_Trolley_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME
    // Values below are arbitrary – set in menu?
int Direction_Motor_Trolley = 1; 
int Destination_Trolley_Position = 3;
int Register_Trolley_Position = 2; //Values 0 to 3 correspond to trolley position: (AtBoom) = 0, (TubRimMax) = 1 , (OverBasket) = 2 , (OverDryAgent) = 3 , (MaxExtension) = 4 . First position: OverBasket


void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}
 
void loop()
{
//note. You want these 2 functions to be general purpose. Make it a method that is called in a loop elsewhere. After each time it’s called, 
//E.g.
//While(Register_Trolley_Position== !3)
//  Callmethod();
}


/* 
  Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
  Destination_ Trolley_Position compared with Register_Trolley_Position to see if higher or lower.
  Destination_ Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
  position and stops if same as destination.
*/
void setTrolleyHorizontalPosition(int Destination_Trolley_Position) {

//Set trolley motor direction according to destination. Return if already at destination 
if(Destination_Trolley_Position > Register_Trolley_Position)
  Direction_Motor_Trolley = Trolley_Direction_Forward;
else if(Destination_Trolley_Position < Register_Trolley_Position)
  Direction_Motor_Trolley = Trolley_Direction_Backward;
else if(Destination_Trolley_Position = Register_Trolley_Position)
  return;

//Drive motor until destination, checking if sense tape marker
while(Register_Trolley_Position  == !Destination_Trolley_Position) {
  motor.speed(Pin_Motor_Trolley, Motor_Speed_Trolley*Direction_Motor_Trolley);
  Previous_STP = Sensor_Trolley_Position;
  delay(Motor_Trolley_Delay_ms);
  Sensor_Trolley_Position = digitalRead(Pin_Sensor_Trolley);

  //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
  if(Previous_STP = !Sensor_Trolley_Position & Previous_STP == Black_Tape)
    if(Direction_Motor_Trolley == 1)
      Register_Trolley_Position++;
    else
      Register_Trolley_Position--;
  }
  
//return after while loop
return;
}

