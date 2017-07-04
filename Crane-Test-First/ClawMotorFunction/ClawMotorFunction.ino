/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int inPin = 0;

int Black_Tape = 1; // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0; // CONCERN: White tape = 0? test


//  VARIABLES ONLY FOR FUNCTION setClawBlockVerticalPosition
//Important concerns: delay length, initial claw block position, claw block motor speed,
int Previous_SCBP = White_Tape;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU
int Sensor_Claw_Block_Position = 0; //Initialization value arbitrary
int Claw_Block_Direction_Down = -1;
int Claw_Block_Direction_Up = 1;
int Pin_Sensor_Claw_Block = 7; // Pin on TINAH. As per TINAH log
int Pin_Motor_Claw_Block = 3; // Pin on TINAH. As per TINAH log
int Motor_Speed_Claw_Block = 100; // Value is 1/2.55 of maximum, CAN CHANGE SPEED
int Motor_Claw_Block_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME
    // Values below are arbitrary – set in menu?
int Direction_Motor_Claw_Block = 1; 
int Destination_Claw_Block_Position = 2;
int Register_Claw_Block_Position = 0; //Values 0 to 3 correspond to claw block position: (AtJib) = 0 , (AtDryAgent) = 1 , (AtWater) = 2 , (MaxExtension) = 3. First position: AtJib


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
//While(Register_Claw_Block_Position== !3)
//  Callmethod();
}


/* 
  Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
  Destination_ Claw_Block_Position compared with Register_Claw_Block_Position to see if higher or lower.
  Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers claw block 
  position and stops if same as destination.
*/
void setClawBlockVerticalPosition(int Destination_Claw_Block_Position) {

//Set claw block motor direction according to destination. Return if already at destination 
if(Destination_Claw_Block_Position > Register_Claw_Block_Position)
  Direction_Motor_Claw_Block = Claw_Block_Direction_Down;
else if(Destination_Claw_Block_Position < Register_Claw_Block_Position)
  Direction_Motor_Claw_Block = Claw_Block_Direction_Up;
else if(Destination_Claw_Block_Position = Register_Claw_Block_Position)
  return;

//Drive motor until destination, checking if sense tape marker
while(Register_Claw_Block_Position  == !Destination_Claw_Block_Position) {
  motor.speed(Pin_Motor_Claw_Block, Motor_Speed_Claw_Block*Direction_Motor_Claw_Block);
  Previous_SCBP = Sensor_Claw_Block_Position;
  delay(Motor_Claw_Block_Delay_ms);
  Sensor_Claw_Block_Position = digitalRead(Pin_Sensor_Claw_Block);

  //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
  if(Previous_SCBP = !Sensor_Claw_Block_Position & Previous_SCBP == Black_Tape)
    if(Direction_Motor_Claw_Block == 1)
      Register_Claw_Block_Position++;
    else
      Register_Claw_Block_Position--;
  }
 
//return after while loop
return;
}

