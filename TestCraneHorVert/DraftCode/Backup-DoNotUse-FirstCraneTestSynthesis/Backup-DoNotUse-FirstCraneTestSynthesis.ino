/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int inPin = 0;

//          VARIABLES USE IN MORE THAN ONE FUNCTION
int Black_Tape = 1; // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0; // CONCERN: White tape = 0? test
int Motor_Speed_Stop = 0; // Use this to stop motors

//          VARIABLES ONLY FOR FUNCTION setTrolleyHorizontalPosition
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
int Destination_Trolley_Position = 3; //ARBITRARY DYNAMIC INITIALIZATION VALUE
int Register_Trolley_Position = 0; //Values 0 to 3 correspond to trolley position: (AtBoom) = 0, (TubRimMax) = 1 , (OverBasket) = 2 , (OverDryAgent) = 3 , (MaxExtension) = 4 . First position: OverBasket


//          VARIABLES ONLY FOR FUNCTION setClawBlockVerticalPosition
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
int Destination_Claw_Block_Position = 2; //ARBITRARY DYNAMIC INITIALIZATION VALUE
int Register_Claw_Block_Position = 0; //Values 0 to 3 correspond to claw block position: (AtJib) = 0 , (AtDryAgent) = 1 , (AtWater) = 2 , (MaxExtension) = 3. First position: AtJib


//          VARIABLES ONLY FOR MENU (in main loop)
double Menu_Destination_Ratio = 3.0/ 1023.0;  //Ratio to convert knob(6) input in menu to get correct destination
double Menu_DCMotor_Ratio = 255.0 / 1023.0;   //Ratio to convert knob(6) input in menu to get correct DC motor speeds
int knob6value;                               //Used to store value from knob 6



//    THIS IS REQUIRED ARDUINO STUFF
void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}

 //   THIS IS YOUR MAIN LOOP
void loop()
{

  while (!stopbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStopToInputVals");
    delay(50);
  }

  if (stopbutton())
    {
        motor.stop_all();

        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterClawBlkDestination");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_Destination_Ratio*knob6value));
         delay(100);
        }
    
        Destination_Claw_Block_Position = round(Menu_Destination_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);
        
        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterClawBlkMotorSpeed");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_DCMotor_Ratio*knob6value));
        delay(100);
        }
    
        Motor_Speed_Claw_Block = round(Menu_DCMotor_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);
        
        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterTrolleyDestination");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_Destination_Ratio*knob6value));
        delay(100);
        }
    
        Destination_Trolley_Position = round(Menu_Destination_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);

        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterTrolleyMotorSpeed");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_DCMotor_Ratio*knob6value));
        delay(100);
        }
    
        Motor_Speed_Trolley = round(Menu_DCMotor_Ratio*knob6value); //prob hafta change this to int...YEA!
    
    }

                                                //REMOVE AFTER, THIS IS JUST FOR TESTING
                                                Register_Trolley_Position = 0;
                                                Register_Claw_Block_Position = 2;

    setTrolleyHorizontalPosition(Destination_Trolley_Position);
    setClawBlockVerticalPosition(Destination_Claw_Block_Position);


}

//      FUNCTIONS START HERE


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
while(Register_Trolley_Position != Destination_Trolley_Position) {
  motor.speed(Pin_Motor_Trolley, Motor_Speed_Trolley*Direction_Motor_Trolley);
  Previous_STP = Sensor_Trolley_Position;
  Sensor_Trolley_Position = digitalRead(Pin_Sensor_Trolley);

  //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
  if(Previous_STP != Sensor_Trolley_Position & Previous_STP == Black_Tape){
    if(Direction_Motor_Trolley == Trolley_Direction_Forward)
      Register_Trolley_Position++;
    else
      Register_Trolley_Position--;
  }
    
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Sensor_Trolley_Position);
    LCD.setCursor(0,1);
    LCD.print("Position: ");
    LCD.print(Register_Trolley_Position);
                                                                            delay(20);  //THIS DELAY LIMITS SPEED
  
}

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Register_Trolley_Position == Destination_Trolley_Position){
  motor.speed(Pin_Motor_Trolley, Motor_Speed_Stop);
  }

                                                                            delay(1000);  //FOR TESTING ONLY; REMOVE AFTER
  
//return after while loop
return;
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
  Direction_Motor_Claw_Block = Claw_Block_Direction_Up;
else if(Destination_Claw_Block_Position < Register_Claw_Block_Position)
  Direction_Motor_Claw_Block = Claw_Block_Direction_Down;
else if(Destination_Claw_Block_Position = Register_Claw_Block_Position)
  return;

//Drive motor until destination, checking if sense tape marker
while(Register_Claw_Block_Position != Destination_Claw_Block_Position) {
  motor.speed(Pin_Motor_Claw_Block, Motor_Speed_Claw_Block*Direction_Motor_Claw_Block);
  Previous_SCBP = Sensor_Claw_Block_Position;
  Sensor_Claw_Block_Position = digitalRead(Pin_Sensor_Claw_Block);
  
  //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
  if(Previous_SCBP != Sensor_Claw_Block_Position & Previous_SCBP == Black_Tape){
    if(Direction_Motor_Claw_Block == Claw_Block_Direction_Up)
      Register_Claw_Block_Position++;
    else
      Register_Claw_Block_Position--;
  }

  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Sensor_Claw_Block_Position);
    LCD.setCursor(0,1);
    LCD.print("Position: ");
    LCD.print(Register_Claw_Block_Position);
                                                                            delay(20);  //THIS DELAY LIMITS SPEED
  
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Register_Claw_Block_Position == Destination_Claw_Block_Position){
  motor.speed(Pin_Motor_Claw_Block, Motor_Speed_Stop);
  }

                                                                            delay(1000);  //FOR TESTING ONLY; REMOVE AFTER

//return after while loop
return;
}


