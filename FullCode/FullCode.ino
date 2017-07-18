//=======================================================================ARDUINO: INCLUDES=================================================================================    

#include <phys253.h>          
#include <LiquidCrystal.h>

//=======================================================================SETUP VARIABLES===================================================================================

int inPin = 0;

//=======================================================================VARIABLES: GENERAL================================================================================

int NumOfAgentsSaved = 0; // Number of agents in possession
int Black_Tape = 1;       // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;       // See Black_Tape comment
int Mot_Speed_Stop = 0;   // Use this to stop motors

//=======================================================================VARIABLES FOR ROBOT STATE=========================================================================

int State_Register = 0; 

int State_WaitAtStart = 0;                   
int State_Go2IRGate = 1;                  
int State_Wait4IRBeacon = 2;                
int State_ApprchRamp = 3;                   
int State_GoUpRamp = 4;                     
int State_AprchCircle = 5;              
int State_EntrgCircle = 6;
int State_AprchLine1 = 7;
int State_DryRetrvLine1 = 8;
int State_WetRetrvLine1 = 9;
int State_AprchNextLine = 10;
int State_DryRetrv = 11;
int State_WetRetrv = 12;
int State_Go2ZipLine = 13;

int RepeatState = 0;                        //Don't = 0, Do = 1


//=======================================================================VARIABLES ONLY FOR FUNCTION setTrolleyHorizontalPosition==========================================

int Postn_Trol_PrvsSnsr = 0;      // You must choose initialization value
int Snsr_Trol_Postn = 0;          // Initialization value completely arbitrary 
int Mot_Trol_Dirctn_Bkwrd = -1;
int Mot_Trol_Dirctn_Fwrd = 1; 
int Pin_Snsr_Trol = 6;            // Pin on TINAH. As per TINAH log
int Pin_Mot_Trol = 2;             // Pin on TINAH. As per TINAH log
int Mot_Trol_Speed = 200;         // Value is Mot_Trol_Speed/255 of maximum. User must choose initialization value.
int Mot_Trol_Dirctn = 1;          // Initialization value completely arbitrary 
int Postn_Trol_Destntn = 3;       // Values meanings correspond to those of Postn_Trol_Register. Initialization value completely arbitrary 
int Postn_Trol_Register = 0;      // Values 0 to 4 correspond to trolley position (see Postn_Trol variables). First position: Postn_Trol_AtBoom = 0
int Postn_Trol_AtBoom = 0;
int Postn_Trol_TubRimMax = 1;
int Postn_Trol_OverBasket = 2;
int Postn_Trol_OverDryAgent = 3; 
int Postn_Trol_MaxExtnsn = 4;


//=======================================================================VARIABLES ONLY FOR FUNCTION setClawBlockVerticalPosition==========================================

int Postn_ClBlk_PrvsSnsr = White_Tape;    // Initialization value important (re: whether Snsr_ClBlk_Postn initial reading results in transition assumption.) 
int Snsr_ClBlk_Postn = 0;                 // Initialization value completely arbitrary
int Mot_ClBlk_Dirctn_Down = -1;
int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 7;                   // Pin on TINAH. As per TINAH log 
int Pin_Mot_ClBlk = 3;                    // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 200;                // Value is Mot_ClBlk_Speed/255 of maximum.  User must choose initialization value.
int Mot_ClBlk_Dirctn = 1;                 // Initialization value completely arbitrary 
int Postn_ClBlk_Destntn = 2;              // Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_Register = 0;             // Values 0 to 4 correspond to claw block position (see Postn_ClBlk variables). First position: Postn_ClBlk_AtJib = 0
int Postn_ClBlk_AtJib = 0;
int Postn_ClBlk_AtDryAgentHigh = 1;
int Postn_ClBlk_AtDryAgentMedium = 2;
int Postn_ClBlk_AtDryAgentLow = 3;
int Postn_ClBlk_AtWater = 4;


//=======================================================================VARIABLES ONLY FOR FUNCTION setClawPosition. Using RCServo1.write(angle) as per TINAH log==========

//Important concerns: delay length, initial claw position
int Postn_Claw_Open = 0;
int Postn_Claw_HaveAgent = 1;
int Postn_Claw_Close = 2;
int Snsr_Claw_Postn = 0;                      // Analog pot. Initialization value completely arbitrary.  Values 0 to 2 correspond to the 3 position variables.
int Postn_Claw_Destntn = 0;                   // Values 0 to 2 correspond to the 3 position variables. Initialization value completely arbitrary 
int Postn_Claw_Register = 0;                  // User must choose initialization value. Values 0 to 2 correspond to the 3 position variables. 
double Snsr_Claw_CnvrsnRatio = (2.0/1023.0);  // ASSUMPTION: have an agent means half open, This is the conversion used to get 0,1 or 2 values for Snsr_Claw_Postn that results in 
int Mot_Claw_Angle_Close = 0;                 // ASCERTAIN THIS VALUE IS CORRECT
int Mot_Claw_Angle_Open = 90;                 // ASCERTAIN THIS VALUE IS CORRECT
int Pin_Snsr_Claw = 43;                       // Pin on TINAH. As per TINAH log ANALOG
int Mot_Claw_Dirctn = 0;                      // Initialization value completely arbitrary  


//=======================================================================VARIABLES ONLY FOR FUNCTION setCranePosition. Using RCServo1.write(angle) as per TINAH log/=========

//Important concerns: delay length, initial crane position, claw block motor speed,
int Mot_Crane_Dirctn = 0;                // ARBITRARY, Direction to go in; 
int Postn_Crane_Destntn = 0;             // Values correspond to those of Postn_Crane_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Crane_Register = 0;            // Values 0 to 2 correspond to the 3 position variables (0,70,90). User must choose initialization value. (First position: ClawOpen)
int Postn_Crane_AngleBot = 0;            // ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_AngleTubLine1 = 70;      // ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_AngleTubLine2to6 = 90;   // ASCERTAIN THIS VALUE IS CORRECT


//=======================================================================ARDUINO: SETUP===================================================================================

void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}


//=======================================================================ARDUINO: VOID LOOP===============================================================================

void loop()
{

//Assumption: robot is on 2nd of 6 circle-lines
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStartToGo!");
    delay(50);
  }

//Function called to retrieve agent if at circle lines 2 to 6. 
//Starts from turning crane to tub. Ends once claw is opened and drops agent
    doStdAgentRtrvl();

}


//=======================================================================FUNCTIONS START HERE============================================================================


//=======================================================================FUNCTION: doOneAgentRtrvlAtLines2to6() =========================================================

// Function called when robot is at one of lines 2to6. Function calls other functions 
  void doStdAgentRtrvl() {

    Postn_Crane_Destntn = Postn_Crane_AngleTubLine2to6;
    setCranePosition(Postn_Crane_Destntn);

    Postn_Trol_Destntn = Postn_Trol_OverDryAgent;
    setTrolleyHorizontalPosition(Postn_Trol_Destntn);

    Postn_ClBlk_Destntn = Postn_ClBlk_AtDryAgentHigh ;
    setClawBlockVerticalPosition(Postn_ClBlk_Destntn);

    Postn_Claw_Destntn = Postn_Claw_Close;
    setClawPosition(Postn_Claw_Destntn);

    Postn_ClBlk_Destntn = Postn_ClBlk_AtJib;
    setClawBlockVerticalPosition(Postn_ClBlk_Destntn);

    Postn_Trol_Destntn = Postn_Trol_OverBasket;
    setTrolleyHorizontalPosition(Postn_Trol_Destntn);

    Postn_Claw_Destntn = Postn_Claw_Open;
    setClawPosition(Postn_Claw_Destntn);

    return;
}


//=======================================================================FUNCTION: setCranePosition() =====================================================================

/* 
  Incoming argument: Postn_Crane_Destntn . Must have min value 0s and max value 180s.
  Sets position of crane.
*/
void setCranePosition(int Postn_Crane_Destntn) {

//Return if already at destination 
  if(Postn_Crane_Destntn = Postn_Crane_Register){
  return;
  }

//Set crane servo direction according to destination. 
  RCServo1.write(Postn_Crane_Destntn); //sends desired angle change to servo

//=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("CraneDestn: ");
    LCD.print(Postn_Crane_Destntn);
//=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER
  
  delay(1000); //Delay to wait to ensure had time to turn
  Postn_Crane_Register = Postn_Crane_Destntn; //No sensor; just assumes completed turn

return;
}



//=======================================================================FUNCTION: setClawPosition() =====================================================================

/* 
  Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
  Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
  If has agent, increments variable NumOfAgentsSaved.
*/
void setClawPosition(int Postn_Claw_Destntn) {

//Set claw servo direction according to destination. Return if already at destination 
  if(Postn_Claw_Destntn = Postn_Claw_Register){
  return;
  }

//Drive motor until destination, checking if arrived
  RCServo1.write(Postn_Claw_Destntn);//sends desired angle change to servo
  delay(500); //Delay to wait to ensure had time to close/open
  Snsr_Claw_Postn = round( analogRead(Pin_Snsr_Claw) * Snsr_Claw_CnvrsnRatio ); //This rounding results in value of 0,1,2, where Postn_Claw_Open = 0, Postn_Claw_HaveAgent = 1, Postn_Claw_Close = 2
  Postn_Claw_Register = Snsr_Claw_Postn;

//Register if agent detected in grip
  if(Postn_Claw_Register == Postn_Claw_HaveAgent){
    NumOfAgentsSaved++;
  }

//=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
    LCD.clear();  
    LCD.home();
    LCD.print("NumAgntsSvd: ");
    LCD.print(NumOfAgentsSaved);
    LCD.setCursor(0,1);
    LCD.print("ClawPostn: ");
    LCD.print(Postn_Claw_Register);
    delay(1000);
//=======================================================================DISPLAY; DELETE ABOVE AFTER TESTING

  delay(1000); //This delay lets you see menu after destination state reached AND,
               //Keep this delay in (via counter in switch case) to not take off before agent falls in thing

return;
}



//=======================================================================FUNCTION: setHorizontalTrolleyPosition() =====================================================================

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

//=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_Trol_Postn);
    LCD.setCursor(0,1);
    LCD.print("TrolPostn: ");
    LCD.print(Postn_Trol_Register);
//=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER

  }

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == Postn_Trol_Destntn){
  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  }

delay(1000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

//return after while loop
return;
}


//=======================================================================FUNCTION: setClawBlockVerticalPosition() =====================================================================

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

//=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_ClBlk_Postn);
    LCD.setCursor(0,1);
    LCD.print("ClBlkPostn: ");
    LCD.print(Postn_ClBlk_Register);
//=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER

delay(20);  //THIS DELAY LIMITS SPEED
  
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == Postn_ClBlk_Destntn){
  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  }

delay(1000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

//return after while loop
return;
}
