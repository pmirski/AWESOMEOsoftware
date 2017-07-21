



//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: INITIAL ARDUINO CODE*************************************************************************
//*****************************************************************************************************************************************************************************




//=======================================================================ARDUINO: INCLUDES=================================================================================    

#include <phys253.h>          
#include <LiquidCrystal.h>


//=======================================================================ARDUINO SETUP VARIABLES===================================================================================

int inPin = 0;




//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: GLOBAL VARIABLES*************************************************************************
//*****************************************************************************************************************************************************************************




//=======================================================================VARIABLES: GENERAL================================================================================

int Mot_Wheels_Speed = 125;                     // Wheels speed
int Mot_Wheels_Speed_Reset = Mot_Wheels_Speed;  // Used to reset Mot_Wheels_Speed when value changes                  
int Mot_Speed_Stop = 0;                         // Use this to stop motors

int NumOfAgentsSaved = 0;                       // Number of agents in possession
int Black_Tape = 1;                             // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;                             // See Black_Tape comment

int Time_BegngOfHeat;                           // Keeps track of time since beginning of heat
int Time_BeginWetRtrvls = 60000;                // After 1 minute, only wet retrievals are possible! (vs. dry retrievals)

int Snsr_Chassis_FrontCrnrR = 0;                // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrR = 42;
int ANALOGTHRESHOLD = 400;                      // Used to utilize side-of-chassis sensors as if digital input re: sensing tape


//=======================================================================VARIABLES: FOR ROBOT STATE & VARIABLES SPECIFIC TO A STATE (indented)===============================

int State_Register = 0; 

int State_WaitAtStart = 0;                   

int State_Go2IRGate = 1;     
    int Time_Go2IRGate = 2000;

int State_Wait4IRBeacon = 2;  
    int Snsr_IR; 
    int Pin_Snsr_IR;                   // TO DO:  Change when know if using. Random numbers right now! 
    int GoFrequency = 10000;
    int WaitFrequency = 1000;              

int State_ApprchRamp = 3;  
    int Time_WhenTimerWasLastRead = 0;
    int Time_ApprchRamp = 5000;             // TO DO: determine this!!! Right now: 5 seconds. In milliseconds.             

int State_GoUpRamp = 4;   
    int Time_GoUpRamp = 5000;               // TO DO: determine this!!! 
    int Mot_Wheels_Speed_Ramp = 150;

int State_AprchCircle = 5;
      int MotUpperPltfrmSlowDownFactor = 2.5;  //VERY IMPORTANT: THIS IS BY HOW MUCH MOT_DRIVE SPEED SLOWS AFTER RAMP, FOR REMAINDER OF HEAT        

int State_EntrgCircle = 6;

/*  Commented out because right now we're starting at Line 2 and continuing on around circle.
int State_AprchLine1 = 7;                   // TO DO: determine if can put just one sensor in, far enough back
    int Pin_Snsr_TowerBottom_Line1 = 99999; // TO DO:  Change when know if using. Random numbers right now!
    int Snsr_TowerBottom_Line1 = 99999;     // TO DO:  Change when know if using. Random numbers right now!     */

int State_AprchNextLine = 7;
    int Time_AfterReadLineToStop = 1.5;           // TO DO:  Change when know if using. COMPLETE GUESS RIGHT NOW!

int State_Retrvl = 8;
    int RetrievalType;
    int WetRetrieval = 1;
    int DryRetrieval = 0;   

int State_Go2ZipLine = 9;
    int Pin_Snsr_ZiplineArrival = 2;              // Pin on TINAH. As per TINAH log
    int Snsr_ZiplineArrival = 9999999;            // TO DO:  Change when know if using. Random numbers right now!
    int Postn_Robot_UnderZL = 1;                  // TO DO: Arbitrary until know what it needs to be re: sensor reading once zipline trips sensor

int State_RaisePltfrm = 10;
    int Pin_Mot_Pltfrm = 2;                       // Pin on TINAH. As per TINAH log (same as Mot_Trol)
    int Mot_Pltfrm_Speed_Up = 100;                // TO DO: Arbitrary until know what speed is good
    int Mot_Pltfrm_Speed_Down = 40;               // TO DO: Arbitrary until know what speed is good
    int Pin_Snsr_Pltfrm = 3;                      // Pin on TINAH. As per TINAH log
    int Postn_Pltfrm_Register;                    // TO DO: Values correspond to next two vars: Postn_Pltfrm_AtBtm = 0, Postn_Pltfrm_AtZL = 1  
    int Postn_Pltfrm_AtBtm = 0;                   // TO DO: Might switch re: sensor reading values
    int Postn_Pltfrm_AtZL = 1;                    // TO DO: Might switch re: sensor reading values

int State_Hook2ZL = 11;
  int Postn_Hook_NotAtZL = 0;                     // TO DO: Might switch re: sensor reading values       
  int Postn_Hook_AtZL = 1;                        // TO DO: Might switch re: sensor reading values
  int Postn_Hook_Register = Postn_Hook_NotAtZL;   // TO DO: Initial value
  int Pin_Mot_Hook = 9999999;                     // TO DO: Arbitrary until know what it needs to be
  int Mot_Hook_Speed = 999999999;                 // TO DO: Arbitrary until know what it needs to be
  int Pin_Snsr_Hook = 9999999;                    // TO DO: Arbitrary until know what it needs to be

int State_LowerPltfrm = 12;


//=======================================================================VARIABLES: FOR FUNCTION driveWheels()==========================================

int KD = 20;                            // Drive_Proporttional     16 or 20 was good value during testing
int KP = 16;                            // Drive_Derivative      16 or 20 was good value during testing

int Drive_Correction = 0;               // Initialization value, keep at 0. 
int Drive_Error = 0;                    // Initialization value, keep at 0.
int Drive_DerivError = 0;               // Initialization value, keep at 0.
int Drive_LastError = 0;                // Initialization value, keep at 0.

int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;

int Snsr_Drive_FrontInL = 0;            // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInL = 8;
boolean Postn_Drive_Snsr_leftLast;      // Used to record last sensor value.
int Snsr_Drive_FrontInR = 0;            // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInR = 9;
boolean Postn_Drive_Snsr_rightLast;     // Used to record last sensor value.
int Snsr_Drive_FrontOutL = 0;           // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontOutL = 10; 
int Snsr_Drive_FrontOutR = 0;           // Initialization value completely arbitrary
int Pin_Snsr_Drive_FrontOutR = 11;


// USED ONLY FOR TESTING driveWheels() to display menu
int count = 0;


//=======================================================================VARIABLES: FOR FUNCTION setTrolleyHorizontalPosition==========================================

int Postn_Trol_PrvsSnsr = 0;      // You must choose initialization value
int Snsr_Trol_Postn = 0;          // Initialization value completely arbitrary 
int Mot_Trol_Dirctn_Bkwrd = -1;
int Mot_Trol_Dirctn_Fwrd = 1; 
int Pin_Snsr_Trol = 0;            // Pin on TINAH. As per TINAH log
int Pin_Mot_Trol = 2;             // Pin on TINAH. As per TINAH log (same as Mot_Pltfrm)
int Mot_Trol_Speed = 65;          // Value is Mot_Trol_Speed/255 of maximum. User must choose initialization value.
int TrolBwdSpeedDivider = 1.3;    // Denominator of motor speed when Trol goes backward
int Mot_Trol_Dirctn = 1;          // Initialization value completely arbitrary 
int Postn_Trol_Destntn = 3;       // Values meanings correspond to those of Postn_Trol_Register. Initialization value completely arbitrary 
int Postn_Trol_AtBoom = 0;
int Postn_Trol_TubRimMax = 1;
int Postn_Trol_OverBasket = 2;
int Postn_Trol_OverDryAgent = 3;
int Postn_Trol_MaxExtnsn = 4;     // TO DO: You only have 4 white tape pieces on jib now.
int Postn_Trol_Register = Postn_Trol_AtBoom;  // Values 0 to 4 correspond to trolley position (see Postn_Trol variables). First position: Postn_Trol_AtBoom = 0


//=======================================================================VARIABLES: FOR FUNCTION setClawBlockVerticalPosition==========================================

int Postn_ClBlk_PrvsSnsr = White_Tape;    // Initialization value important (re: whether Snsr_ClBlk_Postn initial reading results in transition assumption.) 
int Snsr_ClBlk_Postn = 0;                 // Initialization value completely arbitrary
int Mot_ClBlk_Dirctn_Down = -1;
int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 1;                   // Pin on TINAH. As per TINAH log 
int Pin_Mot_ClBlk = 3;                    // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 90;                 // Value is Mot_ClBlk_Speed/255 of maximum.  User must choose initialization value.
int ClbBlkDwdSpeedDivider = 1.3;          // Denominator of motor speed when ClBlk goes downward
int Mot_ClBlk_Dirctn = 1;                 // Initialization value completely arbitrary 
int Postn_ClBlk_Destntn = 2;              // Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_AtJib = 0;
int Postn_ClBlk_AtDryAgentHigh = 1;
int Postn_ClBlk_AtDryAgentMedium = 2;
int Postn_ClBlk_AtDryAgentLow = 3;
int Postn_ClBlk_AtWater = 4;
int Postn_ClBlk_Register = Postn_ClBlk_AtJib;  // Values 0 to 4 correspond to claw block position (see Postn_ClBlk variables). First position: Postn_ClBlk_AtJib = 0


//=======================================================================VARIABLES: FOR FUNCTION setClawPosition. Using RCServo1.write(angle) as per TINAH log==========

/* 
 * Important concerns: delay length, initial claw position 
 */

int Postn_Claw_Open = 0;
//int Postn_Claw_HaveAgent = 1;     //Commented out because we're not using right now
int Postn_Claw_Close = 2;
int Snsr_Claw_Postn = 0;                      // Analog pot. Initialization value completely arbitrary.  Values 0 to 2 correspond to the 3 position variables.
int Postn_Claw_Destntn = 0;                   // Values 0 to 1 correspond to the 2 position variables. Initialization value completely arbitrary 
int Postn_Claw_Register = Postn_Claw_Open;                  // Values 0 to 1 correspond to the 2 position variables. First position: Postn_Claw_Open = 0
double Snsr_Claw_CnvrsnRatio = (2.0/1023.0);  // ASSUMPTION: have an agent means half open, This is the conversion used to get 0,1 or 2 values for Snsr_Claw_Postn that results in 
int Mot_Claw_Angle_Close = 0;                 // TO DO: ASCERTAIN THIS VALUE IS CORRECT
int Mot_Claw_Angle_Open = 90;                 // TO DO: ASCERTAIN THIS VALUE IS CORRECT
int Pin_Snsr_Claw = 43;                       // Pin on TINAH. As per TINAH log ANALOG
int Mot_Claw_Dirctn = 0;                      // Initialization value completely arbitrary  


//=======================================================================VARIABLES: FOR FUNCTION setCranePosition. Using RCServo0.write(angle) as per TINAH log/=========

/*
 * Important concerns: delay length, initial crane position, claw block motor speed,
 */
 
int Mot_Crane_Dirctn = 0;               // ARBITRARY, Direction to go in; 
int Postn_Crane_Destntn = 0;            // Values correspond to those of Postn_Crane_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Crane_AngleBot = 90;          // TO DO: ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_AngleTubLineStd = 70;   // TO DO: ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_Register = Postn_Crane_AngleBot;  // Values 0 to 2 correspond to the 3 position variables (0,70,90). User must choose initialization value. (First position: ClawOpen)



//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: ARDUINO SETUP*************************************************************************
//*****************************************************************************************************************************************************************************




void setup()
{
  #include <phys253setup.txt>
  Serial.begin(9600);  
  pinMode(inPin, INPUT);
  RCServo0.write(90);
}



//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: ARDUINO VOID LOOP*************************************************************************
//*****************************************************************************************************************************************************************************




void loop()
{
  
  // Push start to go!
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStartToGo!");
    delay(50);
  }

  Time_BegngOfHeat = millis();

  State_Register = State_Go2IRGate;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Go2IRGate");

  
  //=======================================================================State_Go2IRGate
  
  /*
   * Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
   */
   
  Time_WhenTimerWasLastRead = millis();  

  while(State_Register == State_Go2IRGate && !(millis() > (Time_WhenTimerWasLastRead + Time_Go2IRGate)) ){  
    driveWheels();
  }
  
  State_Register = State_Wait4IRBeacon;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Wait4IRBeacon");


  //=======================================================================State_Wait4IRBeacon 
        
  /*
   * Robot waits until 10000Hz is sensed. Delay and repeat of check to "double check" that GO reading was not due to error.
   */
   
/*                                                                     NO HARDWARE YET!!!

  while(Snsr_IR != GoFrequency){
    }
    
  delay(50);
  
  while(Snsr_IR != GoFrequency){
    }


*/

  State_Register = State_ApprchRamp;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_ApprchRamp");


  //=======================================================================State_ApprchRamp
    
  /*
   * Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
   */
   
  Time_WhenTimerWasLastRead = millis();

  while(State_Register == State_ApprchRamp && !(millis() > (Time_WhenTimerWasLastRead + Time_ApprchRamp)) ){  
    driveWheels();
  }

  State_Register == State_GoUpRamp;

  LCD.clear();
  LCD.home();   
  LCD.print("State_GoUpRamp");

  
  //=======================================================================State_GoUpRamp
    
  /*
   * Robot tape tracks for Timing_MilliscndsApprchRamp milliseconds until ostensibly at ramp
   */
   
  Time_WhenTimerWasLastRead = millis();

  if(State_Register == State_GoUpRamp){
    Mot_Wheels_Speed_Reset == Mot_Wheels_Speed;
    Mot_Wheels_Speed = Mot_Wheels_Speed_Ramp;
  }
    
  while( !( millis() > (Time_WhenTimerWasLastRead + Time_GoUpRamp) ) ){ 
    driveWheels();      
  }

  Mot_Wheels_Speed = Mot_Wheels_Speed_Reset;

  State_Register = State_AprchCircle;

  LCD.clear();
  LCD.home();   
  LCD.print("State_AprchCircle");


  //=======================================================================State_AprchCircle
    
  /*
   * Follow tape until Snsr_Drive_FrontOutR senses black tape. When does, pivot on right wheel (driving on left) until 
   * Snsr_Drive_FrontInL & Snsr_Drive_FrontInR see black again (i.e. are on the circle again). Then proceed to next state.
   * Pause is to ensure doesn't immediately scan for black tape, because will see immediately. 
   */

  Mot_Wheels_Speed = Mot_Wheels_Speed/MotUpperPltfrmSlowDownFactor;

  while(State_Register == State_AprchCircle){ 
    driveWheels();
    LCD.setCursor(0,1);
    LCD.print(Mot_Wheels_Speed);
    Snsr_Chassis_FrontCrnrR = analogRead(Pin_Snsr_Chassis_FrontCrnrR);

    if(Snsr_Chassis_FrontCrnrR > ANALOGTHRESHOLD){
      motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
      motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
      delay(1000);
      Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);
      
      LCD.clear();
      LCD.home();
      LCD.print("Found R-Corner");
                                                                                //IT'S NOT WORKING BELOW THIS LINE, BABY
      while(Snsr_Drive_FrontInR == White_Tape){
        LCD.setCursor(0,1);
        LCD.print("Look 4 Tape");
        
        Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);
        motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
        motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
      }
      LCD.clear();
      LCD.home();
      LCD.print("On line again");
      State_Register = State_AprchNextLine;   
                                                            
    }
      
  }

////CODE JUST FOR TESTING STARTS: Motor halt and time delay just for test
        motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
        motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  LCD.clear();
  LCD.home();
  LCD.setCursor(0,1);
  delay(5000);
  LCD.print("DONE.State_AprchNextLine");
  delay(10000);
////CODE JUST FOR TESTING ENDS: 
                                                                  //==========================================State_AprchLine1  <--------*********NOT USED
                                                            /*  
                                                             *If used, ensure "State_Register = State_AprchLine1" executed in previous code block. 
                                                             *Here, robot follows tape until SOME? sensor senses first line. 
                                                             */
                                                             
                                                            /*  while(State_Register == State_AprchLine1 && digitalRead(Pin_Snsr_TowerBottom_Line1) == White_Tape){
                                                                  driveWheels();
                                                                } */

  //=======================================================================State_AprchNextLine
    
  /*
   * Robot follows tape until sensors at tower base sense next line.
   */
   
  doAprchNextLine();
  
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");


  //=======================================================================State_Retrvl
    
  /*
   * Function called to retrieve agent at circle line, when perpendicular. Starts from turning crane to tub. 
   * Ends once claw is opened and drops agent. Second line (Line2) from entrance is first line
   */

  RetrievalType = informIfWetOrDryRtrvl();
  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentMedium);
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_AprchNextLine");


  //=======================================================================State_Retrvl, State_AprchNextLine, State_Retrvl, State_AprchNextLine, etc. ...

  /*
   * From here, we move to Line3 thru Line6, doing low(3), high(4), medium(5), low(6),  code skips over entrance, 
   * and do wet retrieval and high(1) at Line1.
   */

  doAprchNextLine();

  // At Line3
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentLow);
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_AprchNextLine");

  doAprchNextLine();

  // At Line4
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentHigh);
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_AprchNextLine");

  doAprchNextLine();

  // At Line5
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentMedium);
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_AprchNextLine");

  doAprchNextLine();

  // At Line6
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  // At Line6
  RetrievalType = informIfWetOrDryRtrvl();
  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentLow);
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_AprchNextLine");

  doAprchNextLine();


  State_Register = State_Go2ZipLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Go2ZipLine");

  //=======================================================================State_Go2ZipLine

  /*
   * Robot drives forward until zipline_arrival sensor is triggered
   */
   
  while(digitalRead(Pin_Snsr_ZiplineArrival) != Postn_Robot_UnderZL ){
    motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
    motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed);
  }

  State_Register = State_RaisePltfrm;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Go2ZipLine");

  //=======================================================================State_RaisePltfrm

  while(Postn_Pltfrm_Register != Postn_Pltfrm_AtZL) {
    motor.speed(Pin_Mot_Pltfrm, Mot_Pltfrm_Speed_Up);
    Postn_Pltfrm_Register = digitalRead(Pin_Snsr_Pltfrm);
  }

  motor.speed(Pin_Mot_Pltfrm, Mot_Speed_Stop);
   
  State_Register = State_Hook2ZL;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Hook2ZL");


  //=======================================================================State_Hook2ZL

  while(Postn_Hook_Register != Postn_Hook_AtZL) {
    motor.speed(Pin_Mot_Hook, Mot_Hook_Speed);
    Postn_Hook_Register = digitalRead(Pin_Snsr_Hook);
  }

  motor.speed(Pin_Mot_Hook, Mot_Speed_Stop);

  State_Register = State_LowerPltfrm;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_LowerPltfrm");


  //=======================================================================State_LowerPltfrm

  while(Postn_Pltfrm_Register != Postn_Pltfrm_AtBtm) {
    motor.speed(Pin_Mot_Pltfrm, Mot_Pltfrm_Speed_Down);
    Postn_Pltfrm_Register = digitalRead(Pin_Snsr_Pltfrm);
  }

  motor.speed(Pin_Mot_Pltfrm, Mot_Speed_Stop);


  //Delay just for test
  LCD.clear();
  LCD.home();
  LCD.setCursor(0,1);
  LCD.print("AtEndVoidLoop.1hrDly");
  delay(3600000);

                                      
}




//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: FUNCTONS, composite (call more than one other function) *************************************
//*****************************************************************************************************************************************************************************




//=======================================================================COMPOSITE FUNCTION: doAgentRtrvl() =========================================================

// Incoming argument: RetrievalType, AgentHeight. RetrievalType must be specified as either DryRetrieval or WetRetrieval.
// If DryRetrieval, AgentHeight must be specified. For WetRetrieval, AgentHeight does not need to be specified.
// Function meant to be called when tower-base aligned with  is at one of lines . Function calls other functions.
void doAgentRtrvl(int FXN_RetrievalType, int FXN_ClBlk_Destntn) {
  int FXN_Trol_Destntn;
  int FXN_Crane_Destntn;
  int FXN_Claw_Destntn;


  if(FXN_RetrievalType == DryRetrieval){
  FXN_Trol_Destntn = Postn_Trol_OverDryAgent;
  }
  else {
  FXN_Trol_Destntn = Postn_Trol_MaxExtnsn;
  }

  LCD.clear();
  LCD.home();
  LCD.print("TrolDest: ");
  LCD.print(FXN_Trol_Destntn); 
  LCD.setCursor(0, 1);
  LCD.print("TrolReg: ");
  LCD.print(Postn_Trol_Register);
  delay(2000);
  
  setTrolleyHorizontalPosition(FXN_Trol_Destntn);

  FXN_Crane_Destntn = Postn_Crane_AngleTubLineStd;

  LCD.clear();
  LCD.home();
  LCD.print("CrnDest: ");
  LCD.print(FXN_Crane_Destntn);
  delay(2000);
  
  setCranePosition(FXN_Crane_Destntn);


  if(FXN_RetrievalType == WetRetrieval){
    FXN_ClBlk_Destntn = Postn_ClBlk_AtWater;
  }

  LCD.clear();
  LCD.home();
  LCD.print("CBDest: ");
  LCD.print(FXN_ClBlk_Destntn);
  LCD.setCursor(0,1);
  LCD.print("CBReg: ");
  LCD.print(Postn_ClBlk_Register);
  delay(2000);
  
  setClawBlockVerticalPosition(FXN_ClBlk_Destntn);

  if(FXN_RetrievalType == WetRetrieval){
    FXN_Trol_Destntn = Postn_Trol_TubRimMax;
    setTrolleyHorizontalPosition(FXN_Trol_Destntn);      
  }


  FXN_Claw_Destntn = Postn_Claw_Close;
  
  LCD.clear();
  LCD.home();
  LCD.print("ClDest: ");
  LCD.print(FXN_Claw_Destntn);
  LCD.setCursor(0,1);
  LCD.print("ClReg: ");
  LCD.print(Postn_Claw_Register);
  delay(2000);
  
  setClawPosition(FXN_Claw_Destntn);


  FXN_ClBlk_Destntn = Postn_ClBlk_AtJib;

  LCD.clear();
  LCD.home();
  LCD.print("CBDest: ");
  LCD.print(FXN_ClBlk_Destntn);
  LCD.setCursor(0,1);
  LCD.print("CBReg: ");
  LCD.print(Postn_ClBlk_Register);
  delay(2000);
  
  setClawBlockVerticalPosition(FXN_ClBlk_Destntn);


  FXN_Crane_Destntn = Postn_Crane_AngleBot;

  LCD.clear();
  LCD.home();
  LCD.print("PreFxnCrnDstn: ");
  LCD.print(FXN_Crane_Destntn);
  delay(2000);
  
  setCranePosition(FXN_Crane_Destntn);


  FXN_Trol_Destntn = Postn_Trol_OverBasket;

  LCD.clear();
  LCD.home();
  LCD.print("TrolDest:");
  LCD.print(FXN_Trol_Destntn);  
  LCD.print("TrolReg: ");
  LCD.print(Postn_Trol_Register);
  delay(2000);
   
  setTrolleyHorizontalPosition(FXN_Trol_Destntn);

  FXN_Claw_Destntn = Postn_Claw_Open;
  
  LCD.clear();
  LCD.home();
  LCD.print("ClDest: ");
  LCD.print(FXN_Claw_Destntn);
  LCD.setCursor(0,1);
  LCD.print("ClReg: ");
  LCD.print(Postn_Claw_Register);
  delay(2000);
  
  setClawPosition(FXN_Claw_Destntn);

  return;
}




//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: FUNCTONS, non-composite (call one other function at most)************************************
//*****************************************************************************************************************************************************************************




//=======================================================================FUNCTION: driveWheels()=====================================================================
/* 
  Function drives wheels while tape-tracking.
*/
void driveWheels() {

    //=======================================================================P.I.D. CODE BLOCK
  //Continuous sensor readings
  Snsr_Drive_FrontInL = digitalRead(Pin_Snsr_Drive_FrontInL); 
  Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR); 
  
  // Calculate proportional error correction component
  if(Snsr_Drive_FrontInL == Black_Tape && Snsr_Drive_FrontInR == Black_Tape){
    Drive_Error = 0;
  }
  if(Snsr_Drive_FrontInL == White_Tape && Snsr_Drive_FrontInR == Black_Tape){
    Drive_Error = -1;    //turn right
    Postn_Drive_Snsr_leftLast = White_Tape;
    Postn_Drive_Snsr_rightLast = Black_Tape;
  }
  if(Snsr_Drive_FrontInL == Black_Tape && Snsr_Drive_FrontInR == White_Tape ){
    Drive_Error = 1;    //turn left
    Postn_Drive_Snsr_leftLast = Black_Tape;
    Postn_Drive_Snsr_rightLast = White_Tape;
  }
  if(Snsr_Drive_FrontInL == White_Tape && Snsr_Drive_FrontInR == White_Tape){
    if(Postn_Drive_Snsr_rightLast == Black_Tape)
      Drive_Error = -5;   //turn hard right
    if(Postn_Drive_Snsr_leftLast == Black_Tape)
      Drive_Error = 5;    //turn hard left
  }

  //Calculate derivative error correction component
  Drive_DerivError = Drive_Error - Drive_LastError;
  
  //Sum correction components into correction variable
  Drive_Correction = Drive_Error*KP + Drive_DerivError*KD; //Original "int Drive_Correction = error*kp + Drive_DerivError*kd" , but extracted Extracted "+ intError*ki" and earler declaration " intError += error;"

  //Add correction to wheel motor speeds
  motor.speed(Pin_Mot_Wheels_L, (Mot_Wheels_Speed - Drive_Correction)*-1);
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed + Drive_Correction);
  
  //To note last error to use in next iteration
  Drive_LastError = Drive_Error;


 /*   //=======================================================================START: (SCREEN + KD,KP,SPEED CONTROLS); CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK
  //Screen Display
  count += 1;
  if(count%300 == 0){
    LCD.clear();
    LCD.home();
    LCD.setCursor(0, 0);
    LCD.print("KD: ");
    LCD.print(KP);
    LCD.print("  KP: ");
    LCD.print(KD);
    LCD.setCursor(0, 1);
    LCD.print("Speed: ");
    LCD.print(Mot_Wheels_Speed);
  }
  
  //Control KD, KP, & Mot_Wheels_Speed COMMENTED OUT, UNLESS USER WANTS TO CONTROL
//   KD = knob(6);                             //Drive_Prprtnl       16 or 20 was good value
//   KP = knob(7);                             //Drive_Dervtv      16 or 20 was good value
  */
  while (startbutton()){
     Mot_Wheels_Speed = Mot_Wheels_Speed + 10;
     delay(100);
   }
  
  while (stopbutton()){
    Mot_Wheels_Speed = Mot_Wheels_Speed - 10;
    delay(100);
  }
    //=======================================================================END: CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK

}


//=======================================================================FUNCTION: doAprchNextLine() =====================================================================
/*
 *No arguments. Simply approaches next line and, upon sensing, continues for Time_AfterReadLineToStop before stopping robot.
*/
void doAprchNextLine() {

  while(digitalRead(Pin_Snsr_Drive_FrontOutL) == White_Tape && digitalRead(Pin_Snsr_Drive_FrontOutR) == White_Tape){
    driveWheels();
  } 

  delay(Time_AfterReadLineToStop);
  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);

}

//=======================================================================FUNCTION: informIfWetOrDryRtrvl() =====================================================================
/* 
 * Keeps track of time since beginning of heat. Returns whether or not it's time for wet retrievals.
 */

int informIfWetOrDryRtrvl() {

    if( (millis() - Time_BegngOfHeat) > Time_BeginWetRtrvls ) {
      return WetRetrieval;
    }
    else {
      return DryRetrieval;
    }

}


//=======================================================================FUNCTION: setCranePosition() =====================================================================
/* 
 * Incoming argument: Postn_Crane_Destntn . Must have min value 0s and max value 180s.
 * Sets position of crane.
*/

void setCranePosition(int FXN1_Crane_Destntn) {

  //Return if already at destination 
  if(FXN1_Crane_Destntn == Postn_Crane_Register){
    return;
  }

  //Set crane servo direction according to destination. 
  RCServo0.write(FXN1_Crane_Destntn); //sends desired angle change to servo

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN DELETE ALL
  //Displays sensor's voltage and position register
  LCD.clear();  
  LCD.home();
  LCD.print("InFxnCrnDstn: ");
  LCD.print(FXN1_Crane_Destntn);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN DELETE ALL
  
  delay(3000); //Delay to wait to ensure had time to turn
  Postn_Crane_Register = FXN1_Crane_Destntn; //No sensor; just assumes completed turn

  return;
}



//=======================================================================FUNCTION: setClawPosition() =====================================================================

/* 
 * Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
 * Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
 * If has agent, increments variable NumOfAgentsSaved.
*/
void setClawPosition(int FXN1_Claw_Destntn) {

  //Set claw servo direction according to destination. Return if already at destination
  if(FXN1_Claw_Destntn = Postn_Claw_Register){
    return;
  } 

  //Drive motor until destination, checking if arrived
  RCServo1.write(FXN1_Claw_Destntn);//sends desired angle change to servo
  delay(500); //Delay to wait to ensure had time to close/open                                              //Commented out because not using claw sensor right now
/*  Snsr_Claw_Postn = round( analogRead(Pin_Snsr_Claw) * Snsr_Claw_CnvrsnRatio ); //This rounding results in value of 0,1,2, where Postn_Claw_Open = 0, Postn_Claw_HaveAgent = 1, Postn_Claw_Close = 2
  Postn_Claw_Register = Snsr_Claw_Postn;
                                                                
  //Register if agent detected in grip
  if(Postn_Claw_Register == Postn_Claw_HaveAgent){                                          
    NumOfAgentsSaved++;
  } */

/*    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN DELETE BLOCK
    delay(1000); //This delay lets you see display command just before this funtion was called
  LCD.clear();  
  LCD.home();
  LCD.print("NumAgntsSvd: ");
  LCD.print(NumOfAgentsSaved);    
  LCD.setCursor(0,1);
  LCD.print("ClawPostn: ");
  LCD.print(Postn_Claw_Register);
  delay(1000);
*/    LCD.clear();  
  LCD.home();
  LCD.print("InFxnClawnDstn: ");
  LCD.print(FXN1_Claw_Destntn);   /*
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN DELETE BLOCK

  delay(1000); //This delay lets you see menu after destination state reached AND,
               //Keep this delay in (via counter in switch case) to not take off before agent falls in thing
*/

  delay(3000);
  
  Postn_Claw_Register = FXN1_Claw_Destntn;

  return;
}



//=======================================================================FUNCTION: setHorizontalTrolleyPosition() =====================================================================
/* 
 * Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
 * Destination_Trolley_Position compared with Postn_Trol_Register to see if higher or lower.
 * Destination_Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
 * Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
 * (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
 * position and stops if same as destination.
*/

void setTrolleyHorizontalPosition(int FXN_Trol_Destntn) {

  //Set trolley motor direction according to destination. Return if already at destination 
  if(FXN_Trol_Destntn > Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_Fwrd;
  else if(FXN_Trol_Destntn < Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_Bkwrd;
  else if(FXN_Trol_Destntn == Postn_Trol_Register)
    return;

  //Drive motor until destination, checking if sense tape marker
  while(Postn_Trol_Register != FXN_Trol_Destntn) {
      if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Bkwrd){
        motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn)/TrolBwdSpeedDivider);
      }
      else {
        motor.speed(Pin_Mot_Trol, Mot_Trol_Speed*Mot_Trol_Dirctn);
      }
    Postn_Trol_PrvsSnsr = Snsr_Trol_Postn;
    Snsr_Trol_Postn = digitalRead(Pin_Snsr_Trol);

    //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
    if(Postn_Trol_PrvsSnsr != Snsr_Trol_Postn & Postn_Trol_PrvsSnsr == Black_Tape){
      if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Fwrd)
        Postn_Trol_Register++;
      else
        Postn_Trol_Register--;
    }

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN DELETE BLOCK
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SnsrRdg: ");
    LCD.print(digitalRead(Pin_Snsr_Trol));
    LCD.setCursor(0,1);
    LCD.print("TrolPostn: ");
    LCD.print(Postn_Trol_Register);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN DELETE BLOCK

  }

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == FXN_Trol_Destntn){
  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  }

  delay(1000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

  //return after while loop
  return;
}


//=======================================================================FUNCTION: setClawBlockVerticalPosition() =====================================================================

/* 
 * Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
 * Destination_ Claw_Block_Position compared with Postn_ClBlk_Register to see if higher or lower.
 * Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
 * Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
 * (checks after every delay time, which is preset). If transition occurs, registers claw block 
 * position and stops if same as destination.
*/

void setClawBlockVerticalPosition(int FXN_ClBlk_Destntn) {

  //Set claw block motor direction according to destination. Return if already at destination 
  if(FXN_ClBlk_Destntn > Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Down;
  else if(FXN_ClBlk_Destntn < Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Up;
  else if(FXN_ClBlk_Destntn == Postn_ClBlk_Register)
    return;

  //Drive motor until destination, checking if sense tape marker
  while(Postn_ClBlk_Register != FXN_ClBlk_Destntn) {
    if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Down){
      motor.speed(Pin_Mot_ClBlk, (Mot_ClBlk_Speed*Mot_ClBlk_Dirctn)/ClbBlkDwdSpeedDivider);
    }
    else {
      motor.speed(Pin_Mot_ClBlk, Mot_ClBlk_Speed*Mot_ClBlk_Dirctn);
    }
    Postn_ClBlk_PrvsSnsr = Snsr_ClBlk_Postn;
    Snsr_ClBlk_Postn = digitalRead(Pin_Snsr_ClBlk);

    //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
    if(Postn_ClBlk_PrvsSnsr != Snsr_ClBlk_Postn & Postn_ClBlk_PrvsSnsr == Black_Tape){
      if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Up)
        Postn_ClBlk_Register--;
      else
        Postn_ClBlk_Register++;
    }

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN DELETE BLOCK
    //Displays sensor's voltage and position register
      LCD.clear();  
      LCD.home();
      LCD.print("SensorVoltage: ");
      LCD.print(Snsr_ClBlk_Postn);
      LCD.setCursor(0,1);
      LCD.print("ClBlkPostn: ");
      LCD.print(Postn_ClBlk_Register);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN DELETE BLOCK

    delay(20);  //THIS DELAY LIMITS SPEED
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == FXN_ClBlk_Destntn){
  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  }

  delay(3000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

  //return after while loop
  return;
}
