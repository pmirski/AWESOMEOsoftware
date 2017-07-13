#include <Servo.h>
#include <phys253.h>          
#include <LiquidCrystal.h>    

//README: 
// To do: ensure have correct function descriptions
// Not yet addded: 2nd search if unsuccessful first time; just single attempt right now
// Not yet addded: Wet retrievals (but setClawBlockVerticalPosition is ready for it)
// Just for testing: delays in functions called, and displays in those functions
// 1. Variable values do not reset during testing, unless you reset them.
// 2. Trolley (at tower), ClawBlock (at jib), and Claw (open) position values are initially 0.
// 3. Determine angle values for: Mot_Claw_Angle_Close, Mot_Claw_Angle_Close, Angle_Crane_Strips2to6, Angle_Crane_Strip1
// 4. Determine angle of having an agent re: Snsr_Claw_CnvrsnRatio, Snsr_Claw_CnvrsnRatio, all three open/haveagent/closed variables
// 5. Determine taking out delays from
// 6. Determine length of time to wait for servo to close
// 7. Note: you likely have redundancy with positioning signal


int inPin = 0;

//          VARIABLES USE IN MORE THAN ONE FUNCTION
int NumOfAgentsSaved = 0; // Number of agents in possession
int Black_Tape = 1; // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0; // CONCERN: White tape = 0? test
int Mot_Speed_Stop = 0; // Use this to stop motors //Mot_Speed_Stop


//          VARIABLES ONLY FOR FUNCTION setTrolleyHorizontalPosition
// Important concerns: delay length, initial trolley position, trolley motor speed
int Postn_Trol_PrvsSnsr = 0;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU 
int Snsr_Trol_Postn = 0; //Initialization value arbitrary 
int Mot_Trol_Dirctn_Bkwrd = -1;
int Mot_Trol_Dirctn_Fwrd = 1; 
int Pin_Snsr_Trol = 6; // Pin on TINAH. As per TINAH log
int Pin_Mot_Trol = 2; // Pin on TINAH. As per TINAH log
int Mot_Trol_Speed = 200; // Value is 2/2.55 of maximum, CAN CHANGE SPEED 
int Mot_Trol_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME 
    // Values below are arbitrary – set in menu?
int Mot_Trol_Dirctn = 1;  //ARBITRARY, Direction to go in
int Postn_Trol_Destntn = 3; //Values correspond to those of Postn_Trol_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Trol_Register = 0;  //Values 0 to 3 correspond to trolley position: (AtBoom) = 0, (TubRimMax) = 1 , (OverBasket) = 2 , (OverDryAgent) = 3 , (MaxExtnsn) = 4 . First position: OverBasket
int Postn_Trol_AtBoom = 0;
int Postn_Trol_TubRimMax = 1;
int Postn_Trol_OverBasket = 2;
int Postn_Trol_OverDryAgent = 3; 
int Postn_Trol_MaxExtnsn = 4;


//          VARIABLES ONLY FOR FUNCTION setClawBlockVerticalPosition
//Important concerns: delay length, initial claw block position, claw block motor speed,
int Postn_ClBlk_PrvsSnsr = White_Tape;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU //Postn_ClBlk_PrvsSns
int Snsr_ClBlk_Postn = 0; //Initialization value arbitrary 
int Mot_ClBlk_Dirctn_Down = -1;
int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 7; // Pin on TINAH. As per TINAH log 
int Pin_Mot_ClBlk = 3; // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 200; // Value is 2/2.55 of maximum, CAN CHANGE SPEED 
int Mot_ClBlk_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME
    // Values below are arbitrary – set in menu?
int Mot_ClBlk_Dirctn = 1;  //ARBITRARY, Direction to go in
int Postn_ClBlk_Destntn = 2; //Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_Register = 0;  //Values 0 to 3 correspond to claw block position: (AtJib) = 0 , (AtDryAgent) = 1 , (AtWater) = 2 , (MaxExtension) = 3. First position: AtJib
int Postn_ClBlk_AtJib = 0;
int Postn_ClBlk_AtDryAgent = 1;
int Postn_ClBlk_AtWater = 2;
int Postn_ClBlk_MaxExtension = 3;

//          VARIABLES ONLY FOR FUNCTION setClawPosition
//Important concerns: delay length, initial claw block position, claw block motor speed,
int Snsr_Claw_Postn = 0; //ANALOG pot, Initialization value arbitrary, Values 0 to 2 correspond to: (ClawOpen) = 0 , (ClawHasAgent) = 1 , (ClawFullyClosedMissedAgent) = 2 
                          ///Send signal, let it close, and THEN take reading!!!!
int Mot_Claw_Angle_Close = 0; //ASCERTAIN THESE VALUES ARE CORRECT
int Mot_Claw_Angle_Open = 90; //ASCERTAIN THESE VALUES ARE CORRECT
int Pin_Snsr_Claw = 43; // Pin on TINAH. As per TINAH log ANALOG
int Pin_Mot_Claw = 31; // Pin on TINAH. As per TINAH log
Servo Mot_Claw_Servo; //Pin is assigned in program
int Mot_Claw_Dirctn = 0;  //ARBITRARY, Direction to go in; 
int Postn_Claw_Destntn = 0; //Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Claw_Register = 0;  //YOU NEED TO LEARN THIS VALUE: Values 0 to 1 correspond to claw position:(ClawOpen) = 0, (Claw HasAgent) = 1, (ClawClosed) = 2,  Initialization value arbitrary  First position: ClawOpen
int Angle_Claw_Strips2to6 = 0; //YOU NEED TO LEARN THIS VALUE
double Snsr_Claw_CnvrsnRatio = (2.0/1023.0); //ASSUMPTION: have an agent means half open, This is the conversion used to get 0,1 or 2 values for Snsr_Claw_Postn that results in 
  //Values of next three variables are based on whether Snsr_Claw_Postn is when servo's open uses other ones too 
int Postn_Claw_Open = 0; 
int Postn_Claw_HaveAgent = 1;    
int Postn_Claw_Close = 2;

//          VARIABLES ONLY FOR FUNCTION setCranePosition
//Important concerns: delay length, initial crane position, claw block motor speed,
int Mot_Crane_Angle_OverBot = 0; //ASCERTAIN THESE VALUES ARE CORRECT
int Mot_Crane_Angle_OverTubLine2to6 = 90; //ASCERTAIN THESE VALUES ARE CORRECT
int Mot_Crane_Angle_OverTubLine1 = 70; //ASCERTAIN THESE VALUES ARE CORRECT
int Pin_Mot_Crane = 35; // Pin on TINAH. As per TINAH log
Servo Mot_Crane_Servo; //Pin is assigned in program
int Mot_Crane_Dirctn = 0;  //ARBITRARY, Direction to go in; 
int Postn_Crane_Destntn = 0; //Values correspond to those of Postn_Crane_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Crane_Register = 0;  //YOU NEED TO LEARN THIS VALUE: Values 0 to 2 correspond next 3 var values,  Initialization value arbitrary  First position: ClawOpen
int Postn_Crane_OverBot = 0; 
int Postn_Crane_OverTubLine2to6 = 1;    
int Postn_Crane_OverTubLine1 = 2;
int Angle_Crane_Strips2to6 = 90s; //YOU NEED TO LEARN THIS VALUE


/*
//          VARIABLES ONLY FOR MENU (in main loop)
double Menu_Destntn_Ratio = 3.0/ 1023.0;  //Ratio to convert knob(6) input in menu to get correct destination
double Menu_DCMotor_Ratio = 255.0 / 1023.0;   //Ratio to convert knob(6) input in menu to get correct DC motor speeds
int knob6value;                               //Used to store value from knob 6
*/


//    THIS IS REQUIRED ARDUINO STUFF
void setup()
{
  
   //Attaches TINAH pins to servos
    Mot_Claw_Servo.attach(Pin_Mot_Claw);
    Mot_Crane_Servo.attach(Pin_Mot_Crane);

  
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}

 //   THIS IS YOUR MAIN LOOP
void loop()
{

//Assumption: robot is on 2nd of 6 circle-lines
  while (!stopbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStartToGo!");
    delay(50);
  }

    //Drive

    //Function called to retrieve agent if at circle lines 2 to 6. 
    //Starts from turning crane to tub. Ends once claw is opened and drops agent
    doOneAgentRtrvlAtLines2to6();

}

//      FUNCTIONS START HERE
//This function is called when robot is at utilizes other functions 
  void doOneAgentRtrvlAtLines2to6() {

    Postn_Crane_Destntn = Postn_Crane_OverTubLine2to6;
    setCranePosition(Postn_Crane_Destntn);

    Postn_Trol_Destntn = Postn_Trol_OverDryAgent;
    setTrolleyHorizontalPosition(Postn_Trol_Destntn);

    Postn_ClBlk_Destntn = Postn_ClBlk_AtDryAgent;
    setClawBlockVerticalPosition(Postn_ClBlk_Destntn);

    Postn_Claw_Destntn = Postn_Claw_Close;
    setClawPosition(Postn_Claw_Destntn);

    Postn_ClBlk_Destntn = Postn_ClBlk_AtJib;
    setClawBlockVerticalPosition(Postn_ClBlk_Destntn);

    Postn_Trol_Destntn = Postn_Trol_OverBasket;
    setTrolleyHorizontalPosition(Postn_Trol_Destntn);

    Postn_Claw_Destntn = Postn_Claw_Open;
    setClawPosition(Postn_Claw_Destntn);

}

/* 
  Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
  Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
  If has agent, increments variable NumOfAgentsSaved.
*/
void setCranePosition(int Postn_Crane_Destntn) {

//Set crane servo direction according to destination. Return if already at destination 
if(Postn_Crane_Destntn > Postn_Crane_Register)
  Mot_Crane_Dirctn = Postn_Crane_OverTubLine2to6;
else if(Postn_Claw_Destntn < Postn_Crane_Register)
  Mot_Crane_Dirctn = Mot_Crane_Angle_OverBot;
else if(Postn_Crane_Destntn = Postn_Crane_Register)
  return;

//Drive motor until destination, checking if arrived
  Mot_Crane_Servo.write(Mot_Crane_Dirctn); //sends desired angle change to servo
  delay(1000); //Delay to wait to ensure had time to turn
  Postn_Crane_Register = Postn_Crane_Destntn; //No sensor; just assumes completed turn

return;
}

/* 
  Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
  Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
  If has agent, increments variable NumOfAgentsSaved.
*/
void setClawPosition(int Postn_Claw_Destntn) {

//Set claw servo direction according to destination. Return if already at destination 
if(Postn_Claw_Destntn > Postn_Claw_Register)
  Mot_Claw_Dirctn = Mot_Claw_Angle_Close;
else if(Postn_Claw_Destntn < Postn_Claw_Register)
  Mot_Claw_Dirctn = Mot_Claw_Angle_Open;
else if(Postn_Claw_Destntn = Postn_Claw_Register)
  return;

//Drive motor until destination, checking if arrived
  Mot_Claw_Servo.write(Mot_Claw_Dirctn); //sends desired angle change to servo
  delay(500); //Delay to wait to ensure had time to close/open
  Snsr_Claw_Postn = round( analogRead(Pin_Snsr_Claw) * Snsr_Claw_CnvrsnRatio ); //This rounding results in value of 0,1,2.
  Postn_Claw_Register = Snsr_Claw_Postn;

  //Register if agent detected in grip
  if(Postn_Claw_Register == Postn_Claw_HaveAgent){
    NumOfAgentsSaved++;
  }

  //DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
    LCD.clear();  
    LCD.home();
    LCD.print("NumAgntsSvd: ");
    LCD.print(NumOfAgentsSaved);
    LCD.setCursor(0,1);
    LCD.print("ClawPostn: ");
    LCD.print(Postn_Claw_Register);
    delay(1000);
  //DELETE ABOVE AFTER TESTING

  delay(1000); //Keep this delay in (via counter in switch case) to not take off before agent falls in thing

return;
}

/* 
  Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
  Destination_ Trolley_Position compared with Postn_Trol_Register to see if higher or lower.
  Destination_ Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
  position and stops if same as destination.
*/
void setTrolleyHorizontalPosition(int Postn_Trol_Destntn) {

//Set trolley motor direction according to destination. Return if already at destination 
if(Postn_Trol_Destntn > Postn_Trol_Register)
  Mot_Trol_Dirctn = Mot_Trol_Dirctn_Fwrd;
else if(Postn_Trol_Destntn < Postn_Trol_Register)
  Mot_Trol_Dirctn = Mot_Trol_Dirctn_Bkwrd;
else if(Postn_Trol_Destntn = Postn_Trol_Register)
  return;

//Drive motor until destination, checking if sense tape marker
while(Postn_Trol_Register != Postn_Trol_Destntn) {
  motor.speed(Pin_Mot_Trol, Mot_Trol_Speed*Mot_Trol_Dirctn);
  Postn_Trol_PrvsSnsr = Snsr_Trol_Postn;
  Snsr_Trol_Postn = digitalRead(Pin_Snsr_Trol);

  //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
  if(Postn_Trol_PrvsSnsr != Snsr_Trol_Postn & Postn_Trol_PrvsSnsr == Black_Tape){
    if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Fwrd)
      Postn_Trol_Register++;
    else
      Postn_Trol_Register--;
  }
    
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_Trol_Postn);
    LCD.setCursor(0,1);
    LCD.print("TrolPostn: ");
    LCD.print(Postn_Trol_Register);
    
delay(20);  //THIS DELAY LIMITS SPEED
  
}

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == Postn_Trol_Destntn){
  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  }

delay(1000);  //FOR TESTING ONLY; REMOVE AFTER

//return after while loop
return;
}



/* 
  Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
  Destination_ Claw_Block_Position compared with Postn_ClBlk_Register to see if higher or lower.
  Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers claw block 
  position and stops if same as destination.
*/
void setClawBlockVerticalPosition(int Postn_ClBlk_Destntn) {

//Set claw block motor direction according to destination. Return if already at destination 
if(Postn_ClBlk_Destntn > Postn_ClBlk_Register)
  Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Up;
else if(Postn_ClBlk_Destntn < Postn_ClBlk_Register)
  Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Down;
else if(Postn_ClBlk_Destntn = Postn_ClBlk_Register)
  return;

//Drive motor until destination, checking if sense tape marker
while(Postn_ClBlk_Register != Postn_ClBlk_Destntn) {
  motor.speed(Pin_Mot_ClBlk, Mot_ClBlk_Speed*Mot_ClBlk_Dirctn);
  Postn_ClBlk_PrvsSnsr = Snsr_ClBlk_Postn;
  Snsr_ClBlk_Postn = digitalRead(Pin_Snsr_ClBlk);
  
  //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
  if(Postn_ClBlk_PrvsSnsr != Snsr_ClBlk_Postn & Postn_ClBlk_PrvsSnsr == Black_Tape){
    if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Up)
      Postn_ClBlk_Register++;
    else
      Postn_ClBlk_Register--;
  }

  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_ClBlk_Postn);
    LCD.setCursor(0,1);
    LCD.print("ClBlkPostn: ");
    LCD.print(Postn_ClBlk_Register);

delay(20);  //THIS DELAY LIMITS SPEED
  
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == Postn_ClBlk_Destntn){
  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  }

delay(1000);  //FOR TESTING ONLY; REMOVE AFTER

//return after while loop
return;
}



